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

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::{
    kpanic as _,
    partition::{entry_point_addr, ExternalPartitionMemory, MpuRegion},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
    AlignedStack1K, DebugEnabled, MsgMinimal, Partitions2, PortsSmall, StackStorage as _,
    SyncMinimal,
};

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

const NUM_PARTITIONS: usize = 1;

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsSmall,
    DebugEnabled > {
        buffer_pool_regions = 1;
        buffer_zone_size = 32;
        dynamic_regions = 4;
    }
);

// 0 = pending, result value when done (set SVC_DONE to 1)
static SVC_RESULT: AtomicU32 = AtomicU32::new(0);
static SVC_DONE: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if SVC_DONE.load(Ordering::Acquire) == 1 {
        let result = SVC_RESULT.load(Ordering::Acquire);
        if result == EXPECTED_ERROR {
            hprintln!("{}: PASS (error {:#010x})", TEST_NAME, result);
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
    }
    if tick > 100 {
        hprintln!("{}: FAIL - timeout", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition entry: invoke SYS_SAMPLING_WRITE with kernel address as data ptr.
extern "C" fn test_partition_main_body(r0: u32) -> ! {
    let port_id = r0;
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, KERNEL_ADDR);
    SVC_RESULT.store(result, Ordering::Release);
    SVC_DONE.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}
kernel::partition_trampoline!(test_partition_main => test_partition_main_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: single partition.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    let mut stack0 = AlignedStack1K::ZERO;
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mem0 = ExternalPartitionMemory::new(
        &mut stack0.0,
        entry_point_addr(test_partition_main),
        sentinel_mpu,
        0,
    )
    .expect("ext mem");
    let mems: [ExternalPartitionMemory; NUM_PARTITIONS] = [mem0];

    let mut k = Kernel::<TestConfig>::new(sched, &mems).expect("kernel creation");

    // Create a source port (partition will "write" to it).
    let port_id = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(k);

    hprintln!("  port_id: {}", port_id);
    hprintln!("  kernel_addr (r3): {:#010x}", KERNEL_ADDR);
    hprintln!("  expected error: {:#010x}", EXPECTED_ERROR);

    // Build partition array for boot().

    match boot(p).expect("syscall_kernel_ptr: boot failed") {}
}
