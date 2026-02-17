//! QEMU test: verify SYS_GET_TIME returns the actual tick count.
//!
//! Boots a single partition whose entry function calls `SYS_GET_TIME`
//! in a loop and stores each reading to an atomic. The SysTick hook
//! (running privileged) periodically checks the partition's readings:
//!
//! 1. After a few ticks the partition's reading must be non-zero,
//!    proving the tick counter is being incremented.
//! 2. A later reading must be >= the first (monotonicity).
//!
//! All semihosting output happens from handler mode (SysTick), since
//! partitions run unprivileged and BKPT traps fault on QEMU.
//!
//! This validates the end-to-end path: SysTick increments the kernel
//! tick via `advance_schedule_tick`, and `SYS_GET_TIME` returns the
//! correct value via the unified Kernel struct.

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
    partition_core::PartitionCore,
    port_pools::PortPools,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    sync_pools::SyncPools,
    syscall::SYS_GET_TIME,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

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

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

/// Latest SYS_GET_TIME reading from the partition (0 = not yet read).
static TIME_READING: AtomicU32 = AtomicU32::new(0);
/// First non-zero reading captured for monotonicity check.
static FIRST_NONZERO: AtomicU32 = AtomicU32::new(0);

// Use the unified harness macro with SysTick hook for verification.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
    match tick {
        // By tick 5, the partition should have read a non-zero time.
        5 => {
            let t = TIME_READING.load(Ordering::Acquire);
            if t == 0 {
                hprintln!("get_time_test: FAIL — tick still zero at tick 5");
                debug::exit(debug::EXIT_FAILURE);
            }
            hprintln!("get_time_test: first non-zero reading = {}", t);
            FIRST_NONZERO.store(t, Ordering::Release);
        }
        // By tick 10, a later reading must be >= the first (monotonicity).
        10 => {
            let t = TIME_READING.load(Ordering::Acquire);
            let first = FIRST_NONZERO.load(Ordering::Acquire);
            hprintln!("get_time_test: second reading = {}", t);
            if t >= first {
                hprintln!("get_time_test: PASS");
                debug::exit(debug::EXIT_SUCCESS);
            } else {
                hprintln!("get_time_test: FAIL — not monotonic: {} < {}", t, first);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
        _ => {}
    }
});

/// Partition entry: spin calling SYS_GET_TIME and publish each reading.
extern "C" fn partition_main() -> ! {
    loop {
        let t = kernel::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32);
        TIME_READING.store(t, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("get_time_test: start");

    // Build schedule: single partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched entry");

    // Build partition config. Stack base is derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = [PartitionConfig {
        id: 0,
        entry_point: 0,
        stack_base: 0,
        stack_size: (STACK_WORDS * 4) as u32,
        mpu_region: MpuRegion::new(0, 0, 0),
        peripheral_regions: heapless::Vec::new(),
    }];

    // Create the unified kernel with schedule and partition.
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");

    store_kernel(k);

    // Boot with partition entry and hint (0 for no packed data).
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    match boot(&parts, &mut p).expect("get_time_test: boot failed") {}
}
