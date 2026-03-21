//! QEMU test: plib `sys_query_bottom_half` returns plausible tick counts
//! and the stale flag reflects kernel state.
//!
//! Schedule: P0 (3 ticks) + SystemWindow (1 tick) = 4-tick major frame.
//! Partition queries `ticks_since_bottom_half` (with stale flag) in a loop,
//! recording the max observed value and verifying monotonic-then-reset.
//!
//! SysTick hook verifies after 20 ticks (5 major frames):
//!   - Many successful calls completed (tight loop → ≥10)
//!   - Tick values bounded by schedule gap (never exceed major-frame len)
//!   - Resets observed match expected frame count (≥4)
//!   - Stale flag always 0 (schedule is healthy, gap=3 << threshold=100)
//!
//! Run:  cargo run --target thumbv7m-none-eabi \
//!         --features qemu,log-semihosting,dynamic-mpu \
//!         --example plib_query_bh_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::{EntryAddr, PartitionConfig};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 40;
/// Major frame = 3 (P0) + 1 (syswin) = 4 ticks; allow small margin.
const MAX_PLAUSIBLE: u32 = 5;

/// Number of successful `sys_query_bottom_half` calls.
static CALL_COUNT: AtomicU32 = AtomicU32::new(0);
/// Last returned `ticks_since_bottom_half`.
static LAST_TICKS: AtomicU32 = AtomicU32::new(0);
/// Maximum `ticks_since_bottom_half` observed.
static MAX_TICKS: AtomicU32 = AtomicU32::new(0);
/// Set to 1 if an implausible value or error is observed.
static BAD_VALUE: AtomicU32 = AtomicU32::new(0);
/// Number of times a reset (value decreased to 0) was observed.
static RESET_COUNT: AtomicU32 = AtomicU32::new(0);
/// Set to 1 once the stale flag (r1) has been observed as non-zero.
static STALE_SEEN: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if tick >= 20 {
        let calls = CALL_COUNT.load(Ordering::Acquire);
        let max = MAX_TICKS.load(Ordering::Acquire);
        let bad = BAD_VALUE.load(Ordering::Acquire);
        let resets = RESET_COUNT.load(Ordering::Acquire);
        let last = LAST_TICKS.load(Ordering::Acquire);
        let stale = STALE_SEEN.load(Ordering::Acquire);

        let pass = calls >= 10 && bad == 0 && max <= MAX_PLAUSIBLE && resets >= 4 && stale == 0;
        #[rustfmt::skip]
        hprintln!(
            "plib_query_bh_test: {} calls={} max={} bad={} resets={} last={} stale_seen={}",
            if pass { "PASS" } else { "FAIL" }, calls, max, bad, resets, last, stale
        );
        if pass {
            kernel::kexit!(success);
        }
    }
    if tick >= TIMEOUT_TICKS {
        hprintln!("plib_query_bh_test: FAIL timeout");
        kernel::kexit!(failure);
    }
});

extern "C" fn partition_main() -> ! {
    let dev = plib::DeviceId::new(0);
    // Start with u32::MAX so the first sample (expected 0) is detected as a
    // reset, avoiding the off-by-one where prev=0 misses a 0→0 transition.
    let mut prev: u32 = u32::MAX;
    let mut first = true;

    loop {
        match plib::sys_query_bottom_half_with_stale(dev) {
            Ok((ticks, stale)) => {
                if ticks > MAX_PLAUSIBLE {
                    BAD_VALUE.store(1, Ordering::Release);
                }
                // In a healthy 4-tick schedule (gap=3), stale should
                // never be set (threshold=100). Flag it as unexpected.
                if stale != 0 {
                    STALE_SEEN.store(1, Ordering::Release);
                    BAD_VALUE.store(1, Ordering::Release);
                }
                // Monotonicity check: if value decreased, it must have
                // reset to 0 (system window ran). Any other decrease is
                // an error.
                if !first && ticks < prev {
                    if ticks == 0 {
                        RESET_COUNT.fetch_add(1, Ordering::Release);
                    } else {
                        BAD_VALUE.store(1, Ordering::Release);
                    }
                }
                if first && ticks == 0 {
                    // Count the initial zero as a reset (partition started
                    // right after a system window).
                    RESET_COUNT.fetch_add(1, Ordering::Release);
                }
                first = false;
                MAX_TICKS.fetch_max(ticks, Ordering::Release);
                LAST_TICKS.store(ticks, Ordering::Release);
                prev = ticks;
                // Increment count AFTER all checks pass.
                CALL_COUNT.fetch_add(1, Ordering::Release);
            }
            Err(_) => {
                BAD_VALUE.store(1, Ordering::Release);
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("plib_query_bh_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("add P0");
    sched.add_system_window(1).expect("sys0");

    let mut cfgs = PartitionConfig::sentinel_array::<1>();
    cfgs[0].entry_point = EntryAddr::from_fn(partition_main).raw();
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("kernel");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("kernel");
    store_kernel(k);

    match boot(p).expect("boot") {}
}
