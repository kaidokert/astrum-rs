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

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::PartitionCore,
    port_pools::PortPools,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    sync_pools::SyncPools,
    syscall::{SYS_EVT_SET, SYS_QUEUING_SEND},
};
use panic_semihosting as _;

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

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

/// Stack size in bytes (must be power of 2 for MPU).
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 2; // Need 2 partitions defined for EVT_SET test
    const SCHED: usize = 4;
    const STACK_WORDS: usize = 256;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 4;
    const QD: usize = 4;
    const QM: usize = 16;
    const QW: usize = 4;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

// Use the unified harness macro: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS);

// ---------------------------------------------------------------------------
// Partition entry point
// ---------------------------------------------------------------------------

/// Partition 0 entry: test invalid resource ID syscalls.
extern "C" fn test_partition_main() -> ! {
    // Test 1: SYS_EVT_SET with invalid partition ID
    // r0 = syscall ID, r1 = target partition ID (invalid), r2 = event mask, r3 = unused
    hprintln!(
        "  testing EVT_SET with partition_id={}",
        INVALID_PARTITION_ID
    );

    let result = kernel::svc!(SYS_EVT_SET, INVALID_PARTITION_ID, 1u32, 0u32);

    if result != EXPECTED_INVALID_PARTITION {
        hprintln!(
            "{}: FAIL - EVT_SET: expected {:#010x}, got {:#010x}",
            TEST_NAME,
            EXPECTED_INVALID_PARTITION,
            result
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!("  EVT_SET returned InvalidPartition: {:#010x}", result);

    // Test 2: SYS_QUEUING_SEND with invalid port ID
    // r0 = syscall ID, r1 = port ID (invalid), r2 = data length, r3 = data pointer
    // Use stack address as data pointer (within MPU region).
    hprintln!("  testing QUEUING_SEND with port_id={}", INVALID_PORT_ID);

    // Use a stack variable as the data pointer (valid within partition's MPU region)
    let data: u32 = 0;
    let data_ptr = &data as *const u32 as u32;

    let result2 = kernel::svc!(SYS_QUEUING_SEND, INVALID_PORT_ID, 4u32, data_ptr);

    if result2 != EXPECTED_INVALID_RESOURCE {
        hprintln!(
            "{}: FAIL - QUEUING_SEND: expected {:#010x}, got {:#010x}",
            TEST_NAME,
            EXPECTED_INVALID_RESOURCE,
            result2
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!("  QUEUING_SEND returned InvalidResource: {:#010x}", result2);

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

    // Create kernel (no ports needed - we're testing invalid IDs).
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

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
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(test_partition_main, 0)];

    match boot(&parts, &mut p).expect("syscall_bad_ids: boot failed") {}
}
