//! QEMU stress test: PENDSTCLR stale SysTick clearing (Bug 16-pelican).
//!
//! Validates that PendSV clears a stale pending SysTick (via PENDSTCLR) so the
//! incoming partition executes before the next tick fires:
//!
//! 1. Two partitions run tight loops, each writing its ID to a shared
//!    `WHO_RAN` atomic flag — proving execution at the partition entry point.
//! 2. SysTick hook pends PendSV on every tick and force-pends SysTick
//!    (PENDSTSET) on alternating ticks to stress the PENDSTCLR path.
//! 3. The harness detects context switches via `WHO_RAN` alternation:
//!    a different partition ID proves the incoming partition ran before
//!    the next tick of its slot (the flag was set by the partition, not
//!    inferred from aggregate counters).
//! 4. After detecting ≥ MIN_SWITCHES alternations with PENDSTSET stress,
//!    declares PASS.  Timeout without enough alternations → FAIL.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example pendstclr_systick_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
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
const TIMEOUT_TICK: u32 = 400;
/// Minimum number of verified context-switch alternations to pass.
const MIN_SWITCHES: u32 = 10;
const NO_PARTITION: u32 = u32::MAX;

/// Each partition writes its own ID here; the harness detects alternation
/// as proof that the incoming partition ran before the next tick.
static WHO_RAN: AtomicU32 = AtomicU32::new(NO_PARTITION);
static LAST_SEEN: AtomicU32 = AtomicU32::new(NO_PARTITION);
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);
/// Toggles PENDSTSET on alternating ticks to stress the PENDSTCLR path.
static REPEND_GUARD: AtomicBool = AtomicBool::new(false);

kernel::define_unified_harness!(Config, |_tick, _k| {
    // Pend PendSV on every tick to force context-switch attempts.
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    // Stress: force-pend SysTick on alternating ticks.  The extra SysTick
    // fires before PendSV (higher priority), so the harness sees the OLD
    // partition ID — no false alternation.  After PendSV switches context
    // and PENDSTCLR clears the stale pending, the new partition runs and
    // writes its ID to WHO_RAN.
    if !REPEND_GUARD.swap(true, Ordering::Relaxed) {
        // TODO: cortex_m crate does not expose a set_pendst() API;
        // use kernel ICSR constants for the PENDSTSET write.
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: Writing PENDSTSET (bit 26) to the ICSR register at
            // 0xE000_ED04 is a single volatile write to a fixed MMIO
            // address.  Other W1-to-clear bits in ICSR (PENDSVCLR,
            // PENDSTCLR) are zero in our value, so they are unaffected.
            // This pends SysTick to simulate the stale-tick race that
            // PENDSTCLR in PendSV must resolve.
            unsafe {
                core::ptr::write_volatile(
                    kernel::pendsv::ICSR_ADDR as *mut u32,
                    kernel::pendsv::PENDSTSET_BIT,
                );
            }
        }
    } else {
        REPEND_GUARD.store(false, Ordering::Relaxed);
    }

    // Detect context-switch alternation via the WHO_RAN flag.
    // Each partition writes its ID in a tight loop; a changed ID proves
    // the incoming partition executed at its entry point before this tick.
    let cur = WHO_RAN.load(Ordering::Acquire);
    if cur != NO_PARTITION {
        let prev = LAST_SEEN.swap(cur, Ordering::Relaxed);
        if prev != NO_PARTITION && prev != cur {
            let n = SWITCH_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            if n >= MIN_SWITCHES {
                hprintln!(
                    "pendstclr_systick_test: PASS ({} switches in {} ticks)",
                    n,
                    _tick
                );
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }

    if _tick >= TIMEOUT_TICK {
        hprintln!(
            "pendstclr_systick_test: FAIL ({} switches, timeout)",
            SWITCH_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        WHO_RAN.store(0, Ordering::Release);
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        WHO_RAN.store(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("pendstclr_systick: Peripherals::take");
    hprintln!("pendstclr_systick_test: start");

    // 2 ticks per slot absorbs the forced PENDSTSET re-entry tick.
    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 2)
        .expect("pendstclr_systick: sched");

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    init_kernel(sched, &parts).expect("pendstclr_systick: Kernel::create");
    match boot(p).expect("pendstclr_systick: boot") {}
}
