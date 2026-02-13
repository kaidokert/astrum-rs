//! Adversarial test: null pointer (0x0) and wrapping pointer (0xFFFF_FFF0 + 32)
//! validation. Both should return `SvcError::InvalidPointer` without faults.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_bad_ptrs

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    partition::{MpuRegion, PartitionConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

const NULL_PTR: u32 = 0x0;
const WRAP_PTR: u32 = 0xFFFF_FFF0;
const WRAP_LEN: u32 = 32;
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();
const TEST_NAME: &str = "syscall_bad_ptrs";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

/// Stack size in bytes (must be power of 2 for MPU).
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 1;
    const SCHED: usize = 4;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 4;
    const SM: usize = 4;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
}

// Use the unified harness macro: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS);

// ---------------------------------------------------------------------------
// Partition entry point
// ---------------------------------------------------------------------------

/// Partition 0 entry: test null pointer and wrapping pointer syscalls.
extern "C" fn test_partition_main() -> ! {
    let port_id = kernel::unpack_r0!();

    // Test 1: Null pointer
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, NULL_PTR);
    if result != EXPECTED_ERROR {
        hprintln!(
            "{}: FAIL - null ptr: expected {:#010x}, got {:#010x}",
            TEST_NAME,
            EXPECTED_ERROR,
            result
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!("{}: null pointer check passed", TEST_NAME);

    // Test 2: Wrapping pointer (0xFFFF_FFF0 + 32 overflows)
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, WRAP_LEN, WRAP_PTR);
    if result != EXPECTED_ERROR {
        hprintln!(
            "{}: FAIL - wrap ptr: expected {:#010x}, got {:#010x}",
            TEST_NAME,
            EXPECTED_ERROR,
            result
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!("{}: wrapping pointer check passed", TEST_NAME);

    hprintln!("{}: PASS", TEST_NAME);
    debug::exit(debug::EXIT_SUCCESS);

    loop {
        cortex_m::asm::wfi();
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    // Build partition config using the STACKS addresses.
    // SAFETY: single-core, interrupts disabled — exclusive access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        core::array::from_fn(|i| {
            let b = STACKS[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: STACK_SIZE,
                mpu_region: MpuRegion::new(b, STACK_SIZE, 0),
            }
        })
    };

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    let port_id = k
        .sampling
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(k);

    hprintln!(
        "  null={:#x} wrap={:#x}+{} expect={:#x}",
        NULL_PTR,
        WRAP_PTR,
        WRAP_LEN,
        EXPECTED_ERROR
    );

    // Build partition array for boot().
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        [(test_partition_main, port_id as u32)];

    boot(&parts, &mut p)
}
