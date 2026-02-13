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
    kernel::KernelState,
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

const MAX_SCHEDULE_ENTRIES: usize = 4;
const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

/// Stack size in bytes (must be power of 2 for MPU).
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

/// Base address of partition 0's data region.
const P0_BASE: u32 = 0x2000_0000;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 1;
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

// Use the standard harness for partition scheduling.
kernel::define_harness!(
    TestConfig,
    NUM_PARTITIONS,
    MAX_SCHEDULE_ENTRIES,
    STACK_WORDS
);

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

    // Create sampling port and kernel.
    #[cfg(feature = "dynamic-mpu")]
    let mut k = Kernel::<TestConfig>::new(kernel::virtual_device::DeviceRegistry::new());
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::new();

    let port_id = k
        .sampling
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(k);

    // Set up schedule table.
    let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");
    sched.start();

    // Build partition config.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = [PartitionConfig {
        id: 0,
        entry_point: 0,
        stack_base: P0_BASE,
        stack_size: STACK_SIZE,
        mpu_region: MpuRegion::new(P0_BASE, STACK_SIZE, 0),
    }];

    // SAFETY: single-core, interrupts not yet enabled — exclusive access to KS.
    // KernelState::new is safe but KS is a static mut.
    unsafe {
        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

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
