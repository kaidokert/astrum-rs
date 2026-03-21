//! QEMU tick-fairness test: Bug 16-pelican PENDSTCLR stale-tick prevention.
//!
//! Two equal-priority partitions increment counters in tight loops.  After 30+
//! ticks the SysTick hook asserts min(c0,c1) > 0.3 * max(c0,c1), proving
//! PENDSTCLR prevents systematic one-sided starvation.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::{EntryAddr, PartitionConfig};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

// Fast SysTick: 12 MHz * 83 µs / 1e6 ≈ 996 cycles per tick.
kernel::compose_kernel_config!(
    Config < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 83;
    }
);

const NUM_PARTITIONS: usize = 2;

/// Minimum tick count before checking fairness (≥ 30 ticks = 15 major frames).
const CHECK_TICK: u32 = 30;
/// Hard timeout — declare failure if not passed by this tick.
const TIMEOUT_TICK: u32 = 300;
/// Fairness threshold: min counter must exceed 30% of max counter.
const FAIRNESS_PCT: u32 = 30;

/// Per-partition loop iteration counters.
static COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Load both counters and return (c0, c1, min, max).
fn load_counts() -> (u32, u32, u32, u32) {
    let c0 = COUNTS[0].load(Ordering::Acquire);
    let c1 = COUNTS[1].load(Ordering::Acquire);
    let max = if c0 > c1 { c0 } else { c1 };
    let min = if c0 < c1 { c0 } else { c1 };
    (c0, c1, min, max)
}

kernel::define_unified_harness!(Config, |tick, _k| {
    // Pend PendSV on every tick to maximise preemption pressure.
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    if tick >= CHECK_TICK {
        let (c0, c1, min, max) = load_counts();

        // Both must have run, and fairness ratio must hold.
        // min > (FAIRNESS_PCT / 100) * max, rearranged to avoid floats:
        // min * 100 > FAIRNESS_PCT * max
        if min > 0 && min * 100 > FAIRNESS_PCT * max {
            hprintln!(
                "tick_fairness_test: PASS (p0={}, p1={}, ratio={}%)",
                c0,
                c1,
                min * 100 / max
            );
            // TODO: test verifies fairness outcome but cannot directly prove PENDSTCLR
            // is the cause; a negative test (disabling PENDSTCLR) would require kernel changes.
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= TIMEOUT_TICK {
        let (c0, c1, min, max) = load_counts();
        let ratio = (min * 100).checked_div(max).unwrap_or(0);
        hprintln!(
            "tick_fairness_test: FAIL (p0={}, p1={}, ratio={}%, need >{}%)",
            c0,
            c1,
            ratio,
            FAIRNESS_PCT
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        COUNTS[0].fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        COUNTS[1].fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("tick_fairness: Peripherals::take");
    hprintln!("tick_fairness_test: start");

    // 2 partitions, 1 tick per slot → major frame = 2 ticks.
    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1)
        .expect("tick_fairness: sched");

    let mut cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>();
    // TODO: PartitionConfig.entry_point is u32; should accept EntryAddr directly
    cfgs[0].entry_point = EntryAddr::from_fn(p0_main).raw();
    cfgs[1].entry_point = EntryAddr::from_fn(p1_main).raw();

    #[cfg(not(feature = "dynamic-mpu"))]
    let k =
        Kernel::<Config>::with_config(sched, &cfgs, &[]).expect("tick_fairness: Kernel::create");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<Config>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("tick_fairness: Kernel::create");

    store_kernel(k);

    match boot(p).expect("tick_fairness: boot") {}
}
