//! Adversarial test: syscalls with invalid resource IDs.
//!
//! Tests that the kernel correctly returns appropriate error codes when given:
//! 1. Invalid partition ID (out of range) for SYS_EVT_SET
//! 2. Invalid port ID (nonexistent) for SYS_QUEUING_SEND
//!
//! Verifies the kernel returns proper error codes without panicking or causing
//! array out-of-bounds access.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_bad_ids

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
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::{SYS_EVT_SET, SYS_QUEUING_SEND},
    AlignedStack1K, DebugEnabled, MsgMinimal, Partitions1, PortsMinimal, StackStorage as _,
    SyncMinimal,
};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Invalid partition ID: out of range.
const INVALID_PARTITION_ID: u32 = 99;

/// Invalid port ID: no ports are created, so any ID is invalid.
const INVALID_PORT_ID: u32 = 99;

/// Expected error for invalid partition ID.
const EXPECTED_INVALID_PARTITION: u32 = SvcError::InvalidPartition.to_u32();

/// Expected error for invalid port ID.
const EXPECTED_INVALID_RESOURCE: u32 = SvcError::InvalidResource.to_u32();

const TEST_NAME: &str = "syscall_bad_ids";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

kernel::compose_kernel_config!(TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsMinimal, DebugEnabled>);

const NUM_PARTITIONS: usize = TestConfig::N;

// 0 = pending, 1 = pass, 2 = fail (EVT_SET), 3 = fail (QUEUING_SEND)
static RESULT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == 1 {
        hprintln!("{}: PASS", TEST_NAME);
        debug::exit(debug::EXIT_SUCCESS);
    } else if r >= 2 {
        hprintln!("{}: FAIL (code {})", TEST_NAME, r);
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick > 100 {
        hprintln!("{}: FAIL - timeout", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition 0 entry: test invalid resource ID syscalls.
extern "C" fn test_partition_main() -> ! {
    let r1 = kernel::svc!(SYS_EVT_SET, INVALID_PARTITION_ID, 1u32, 0u32);
    if r1 != EXPECTED_INVALID_PARTITION {
        RESULT.store(2, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }
    let data: u32 = 0;
    let data_ptr = &data as *const u32 as u32;
    let r2 = kernel::svc!(SYS_QUEUING_SEND, INVALID_PORT_ID, 4u32, data_ptr);
    if r2 != EXPECTED_INVALID_RESOURCE {
        RESULT.store(3, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }
    RESULT.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
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

    // Create kernel (no ports needed - we're testing invalid IDs).
    let k = Kernel::<TestConfig>::new(sched, &mems).expect("kernel creation");

    store_kernel(k);

    hprintln!(
        "  invalid_partition_id={} expected={:#x}",
        INVALID_PARTITION_ID,
        EXPECTED_INVALID_PARTITION
    );
    hprintln!(
        "  invalid_port_id={} expected={:#x}",
        INVALID_PORT_ID,
        EXPECTED_INVALID_RESOURCE
    );

    // Build partition array for boot().

    match boot(p).expect("syscall_bad_ids: boot failed") {}
}
