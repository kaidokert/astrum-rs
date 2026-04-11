//! SysTick Timer — PCA10100 (nRF52833-DK)
//!
//! Configures SysTick at 1 ms intervals (64 MHz core clock ÷ 64 000).
//! Counts ticks in an interrupt handler and reports via RTT every 100 ms.
//! Stops logging after 1000 ticks (1 second) then loops silently.
//!
//! Build:  cargo build --example systick
//! Flash:  cargo run --example systick

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use rtt_target::{rprintln, rtt_init_print};

// pulls in critical-section-single-core impl needed by rtt-target
use cortex_m as _;
use nrf52833_pac as _; // force PAC link so __INTERRUPTS from rt feature is included

static TICK: AtomicU32 = AtomicU32::new(0);

use nrf52::CORE_CLOCK_HZ;

const TICK_RATE_HZ: u32 = 1_000; // 1 ms
const RELOAD: u32 = CORE_CLOCK_HZ / TICK_RATE_HZ - 1;

const MAX_TICKS: u32 = 1_000;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== SysTick Demo — nRF52833 ===");
    rprintln!("Core clock: {} MHz, tick rate: {} Hz", CORE_CLOCK_HZ / 1_000_000, TICK_RATE_HZ);
    rprintln!("Reload: {}  ({}ms per tick)", RELOAD, 1000 / TICK_RATE_HZ);
    rprintln!("Logging every 100 ticks, stopping at {}", MAX_TICKS);

    let mut cp = cortex_m::Peripherals::take().unwrap();

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(RELOAD);
    cp.SYST.clear_current();
    cp.SYST.enable_interrupt();
    cp.SYST.enable_counter();

    rprintln!("SysTick started.");

    loop {
        let t = TICK.load(Ordering::Relaxed);
        if t > MAX_TICKS {
            // Done — spin silently (firmware still running, LEDs can be checked)
            cortex_m::asm::wfi();
        }
    }
}

#[exception]
fn SysTick() {
    let t = TICK.fetch_add(1, Ordering::Relaxed) + 1;
    if t % 100 == 0 {
        rprintln!("[{:5} ms] tick={}", t, t);
    }
    if t == MAX_TICKS {
        rprintln!("DONE — {} ticks counted.", MAX_TICKS);
    }
}
