//! Adversarial test: partition calls SYS_SAMPLING_WRITE with kernel pointer.
//!
//! This test verifies that the kernel's `validate_user_ptr` check in svc.rs
//! correctly rejects syscall data pointers that point to kernel memory.
//! The partition invokes SYS_SAMPLING_WRITE with r3 set to 0x2000_F000 (kernel
//! RAM), which should be rejected with `SvcError::InvalidPointer` (0xFFFF_FFF9)
//! **without** dereferencing the pointer or causing a MemManage fault.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_kernel_ptr

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    kernel::KernelState,
    partition::PartitionConfig,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Kernel RAM address to pass as the data pointer.
/// This is outside the partition's MPU data region.
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Expected error code: SvcError::InvalidPointer.
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();

/// Test name for reporting.
const TEST_NAME: &str = "syscall_kernel_ptr";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const MAX_SCHEDULE_ENTRIES: usize = 4;
const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

struct TestConfig;
impl KernelConfig for TestConfig {
    // N must match NUM_PARTITIONS for KernelState consistency.
    const N: usize = 1;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 4; // Need at least one sampling port
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

/// Partition entry: invoke SYS_SAMPLING_WRITE with kernel address as data ptr.
extern "C" fn test_partition_main() -> ! {
    // Unpack the port ID passed via r0.
    let port_id = kernel::unpack_r0!() as u32;

    // Issue SYS_SAMPLING_WRITE with:
    //   r1 = port_id (valid sampling port)
    //   r2 = 4 (data length)
    //   r3 = KERNEL_ADDR (invalid: points to kernel memory)
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, KERNEL_ADDR);

    // The kernel should have rejected this with InvalidPointer.
    // If we reach here, no MemManage fault occurred (pointer was validated
    // before dereference).
    if result == EXPECTED_ERROR {
        hprintln!("{}: PASS (error code {:#010x})", TEST_NAME, result);
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!(
            "{}: FAIL - expected {:#010x}, got {:#010x}",
            TEST_NAME,
            EXPECTED_ERROR,
            result
        );
        debug::exit(debug::EXIT_FAILURE);
    }

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

    // Create a sampling port for the test partition.
    let port_id;
    // SAFETY: single-core, interrupts not yet enabled — exclusive access.
    unsafe {
        #[cfg(feature = "dynamic-mpu")]
        let mut k = Kernel::<TestConfig>::new(kernel::virtual_device::DeviceRegistry::new());
        #[cfg(not(feature = "dynamic-mpu"))]
        let mut k = Kernel::<TestConfig>::new();

        // Create a source port (partition will "write" to it).
        port_id = k
            .sampling
            .create_port(PortDirection::Source, 10)
            .expect("create port");

        store_kernel(k);

        // Set up schedule: single partition.
        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        sched
            .add(ScheduleEntry::new(0, 2))
            .expect("schedule entry must fit");
        sched.start();

        // Build partition config using proper struct fields.
        let cfgs: [PartitionConfig; NUM_PARTITIONS] = [{
            let b = 0x2000_0000u32;
            PartitionConfig {
                id: 0,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        }];

        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

    hprintln!("  port_id: {}", port_id);
    hprintln!("  kernel_addr (r3): {:#010x}", KERNEL_ADDR);
    hprintln!("  expected error: {:#010x}", EXPECTED_ERROR);

    // Build partition array for boot().
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        [(test_partition_main, port_id as u32)];

    boot(&parts, &mut p)
}
