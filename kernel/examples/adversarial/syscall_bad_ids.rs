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
use kernel::{
    config::KernelConfig,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::{AlignedStack1K, PartitionCore},
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
    const N: usize = 1;
    const SCHED: usize = 4;
    const STACK_WORDS: usize = 256;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
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

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

// 0 = pending, 1 = pass, 2 = fail (EVT_SET), 3 = fail (QUEUING_SEND)
static RESULT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
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
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    // Build partition configs. Stack bases are derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| PartitionConfig {
        id: i as u8,
        entry_point: 0, // Not used by Kernel::new
        stack_base: 0,  // Ignored: internal stack used
        stack_size: STACK_SIZE,
        mpu_region: MpuRegion::new(0, 0, 0), // Base/size overridden by Kernel::new
        peripheral_regions: heapless::Vec::new(),
    });

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
