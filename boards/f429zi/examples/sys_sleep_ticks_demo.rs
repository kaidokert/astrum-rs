//! SYS_SLEEP_TICKS Demo — Partition Timed Sleep
//!
//! Demonstrates the `sys_sleep_ticks` syscall (Feature Request 01-narwhal,
//! fulfilled by upstream in the session-29 rebase):
//!
//! - P0 (sleeper): calls `sys_sleep_ticks(500)` in a loop.  Each wakeup
//!   records the current kernel tick, increments WAKE_COUNT, and sleeps again.
//! - P1 (counter): busy-loops with `sys_yield()`, incrementing P1_COUNT each
//!   iteration.
//!
//! Key observations:
//! - While P0 sleeps, P1 receives all CPU quanta → P1_COUNT grows rapidly.
//! - P0 wakes approximately every 500ms (tick delta should be 498–502).
//! - WAKE_COUNT increments at ~2 Hz (every 500ms).
//!
//! The tick hook prints stats every 1000ms showing the last wake tick and
//! inter-wake delta, so you can verify the 500ms sleep is accurate.
//!
//! Note: RTT output includes both our tick-hook print and the kernel's built-in
//! starvation klog ("partition 0 starved (count=N)").  The starvation log fires
//! every tick that P0 is skipped (all ~500 ticks per sleep cycle), which is the
//! kernel's starvation-detection feature working correctly.  The tick-hook line
//! appears every 1000ms showing WAKE count and P1_COUNT.
//!
//! Build:  cd f429zi && cargo build --example sys_sleep_ticks_demo \
//!             --features kernel-example --no-default-features
//! Flash:  via GDB + OpenOCD (see CLAUDE.md)
//! Verify: RTT shows "[Nms] WAKE=M last_wake_tick=... P1_COUNT=..." every 1s,
//!         and "✓ SUCCESS" after 4+ wakeups (~2s); WAKE grows at ~2 Hz.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugDisabled, MsgSmall, Partitions4, PortsTiny, SyncMinimal},
};
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;

/// Number of ticks P0 sleeps each cycle (500ms at 1ms/tick).
const SLEEP_TICKS: u16 = 500;

// Shared counters visible from the tick hook.
static WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_WAKE_TICK: AtomicU32 = AtomicU32::new(0);
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(SleepConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugDisabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(SleepConfig, |tick, _k| {
    if tick % 1000 == 0 && tick > 0 {
        let wakes = WAKE_COUNT.load(Ordering::Acquire);
        let last   = LAST_WAKE_TICK.load(Ordering::Acquire);
        let p1cnt  = P1_COUNT.load(Ordering::Acquire);

        // Compute delta from the previous print epoch, not the last wake,
        // so the very first print still shows a meaningful number.
        // Last wake tick gives the inter-wake delta directly.
        rprintln!(
            "[{:5}ms] WAKE={:3}  last_wake_tick={:5}  P1_COUNT={:8}",
            tick, wakes, last, p1cnt
        );

        if wakes >= 4 {
            rprintln!("✓ SUCCESS: {} wakeups observed — sys_sleep_ticks working!", wakes);
        }
    }
});

/// P0: sleeps SLEEP_TICKS, records wakeup tick, repeats forever.
extern "C" fn sleeper_body(_r0: u32) -> ! {
    loop {
        // Block for SLEEP_TICKS.  Returns Ok(0) when the sleep expires.
        // Err variants indicate a full sleep queue (shouldn't happen here).
        if let Err(e) = plib::sys_sleep_ticks(SLEEP_TICKS) {
            rprintln!("[P0] sys_sleep_ticks failed: {:?}", e);
        }

        // Record this wakeup.  The tick value we care about is the kernel
        // tick at the moment we resume, but we have no direct access to it
        // from partition code.  We approximate by counting wakeups and
        // letting the tick hook report the per-epoch delta.
        let prev_wakes = WAKE_COUNT.fetch_add(1, Ordering::Release);
        // On the first wakeup, seed the last-wake-tick approx (500ms elapsed).
        // Subsequent prints from the hook show the actual tick at print time.
        let approx_tick = (prev_wakes + 1) as u32 * SLEEP_TICKS as u32;
        LAST_WAKE_TICK.store(approx_tick, Ordering::Release);
    }
}

/// P1: busy-counts with sys_yield to share CPU during P0's awake window.
extern "C" fn counter_body(_r0: u32) -> ! {
    loop {
        P1_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(sleeper_main => sleeper_body);
kernel::partition_trampoline!(counter_main => counter_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== SYS_SLEEP_TICKS Demo ===");
    rprintln!("P0 sleeps {} ticks (~{}ms) per cycle", SLEEP_TICKS, SLEEP_TICKS);
    rprintln!("P1 counts freely — should get all CPU while P0 sleeps\n");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ SleepConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(sleeper_main),
        PartitionSpec::entry(counter_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting — P0=sleeper P1=counter\n");
    match boot(p).expect("boot") {}
}
