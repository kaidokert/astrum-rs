#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};

static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

const MAX_TICKS: u32 = 5;

/// SysTick reload value for ~10 ms ticks at 12 MHz (QEMU lm3s6965evb).
const RELOAD: u32 = kernel::config::compute_systick_reload(12_000_000, 10_000);

#[exception]
fn SysTick() {
    TICK_COUNT.fetch_add(1, Ordering::Release);
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let mut syst = p.SYST;

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    hprintln!(
        "SysTick configured: reload={}, waiting for {} ticks",
        RELOAD,
        MAX_TICKS
    );

    let mut last_seen = 0u32;

    loop {
        let current = TICK_COUNT.load(Ordering::Acquire);
        if current != last_seen {
            hprintln!("tick {}", current);
            last_seen = current;
            if current >= MAX_TICKS {
                hprintln!("done");
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }
}
