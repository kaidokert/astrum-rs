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
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

/// Latest SYS_GET_TIME reading from the partition (0 = not yet read).
static TIME_READING: AtomicU32 = AtomicU32::new(0);
/// First non-zero reading captured for monotonicity check.
static FIRST_NONZERO: AtomicU32 = AtomicU32::new(0);

// Use the unified harness macro with SysTick hook for verification.
kernel::define_unified_harness!(TestConfig, |tick, _k| {
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
const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    loop {
        let t = plib::sys_get_time().unwrap_or(0);
        TIME_READING.store(t, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("get_time_test: start");

    // Build schedule: single partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched entry");

    let parts: [PartitionSpec; TestConfig::N] =
        [PartitionSpec::new(partition_main as PartitionEntry, 0)];
    init_kernel(sched, &parts).expect("kernel creation");

    // Boot with partition entry and hint (0 for no packed data).
    match boot(p).expect("get_time_test: boot failed") {}
}
