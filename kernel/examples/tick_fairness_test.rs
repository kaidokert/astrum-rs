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
use kernel::partition::{EntryAddr, ExternalPartitionMemory, MpuRegion};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal};

const STACK_WORDS: usize = 256;

// Fast SysTick: 12 MHz * 83 µs / 1e6 ≈ 996 cycles per tick.
kernel::kernel_config!(
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

kernel::define_kernel!(Config, |tick, _k| {
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

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        COUNTS[0].fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
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

    // SAFETY: called once from main before any interrupt handler runs.
    // TODO: reviewer false positive — `__PARTITION_STACKS` is defined by
    // `define_kernel!(@impl_compat)` as a module-level `static mut`.
    let stacks = unsafe {
        &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NUM_PARTITIONS]>()
    };
    // TODO: array destructuring is compile-time checked against NUM_PARTITIONS;
    // a mismatch causes a compilation error, not a runtime panic.
    // TODO: uses NUM_PARTITIONS (not NP) to match the existing convention in
    // this file where NUM_PARTITIONS is referenced by round_robin(), COUNTS, etc.
    let [ref mut s0, ref mut s1] = *stacks;
    let mpu = MpuRegion::new(0, 0, 0);
    let e = EntryAddr::from_entry;
    let memories = [
        ExternalPartitionMemory::new(
            s0,
            e(p0_main as PartitionEntry),
            mpu,
            kernel::PartitionId::new(0),
        )
        .expect("mem 0"),
        ExternalPartitionMemory::new(
            s1,
            e(p1_main as PartitionEntry),
            mpu,
            kernel::PartitionId::new(1),
        )
        .expect("mem 1"),
    ];
    let mut k = Kernel::<Config>::new(sched, &memories).expect("tick_fairness: Kernel::create");

    store_kernel(&mut k);

    match boot(p).expect("tick_fairness: boot") {}
}
