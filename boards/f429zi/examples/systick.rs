#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use stm32f4xx_hal as _; // For device interrupt vectors

static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

const MAX_TICKS: u32 = 10;

/// SysTick reload value for ~10 ms ticks at 16 MHz (STM32F429 HSI default).
/// 16_000_000 * 0.01 = 160_000 cycles per tick; reload = cycles - 1.
const RELOAD: u32 = 160_000 - 1;

#[exception]
fn SysTick() {
    TICK_COUNT.fetch_add(1, Ordering::Release);
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let p = cortex_m::Peripherals::take().unwrap();
    let mut syst = p.SYST;

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    rprintln!("STM32F429ZI SysTick Test");
    rprintln!(
        "SysTick configured: reload={}, waiting for {} ticks (~100ms)",
        RELOAD,
        MAX_TICKS
    );

    let mut last_seen = 0u32;

    loop {
        let current = TICK_COUNT.load(Ordering::Acquire);
        if current != last_seen {
            rprintln!("tick {}", current);
            last_seen = current;
            if current >= MAX_TICKS {
                rprintln!("Done. SysTick working correctly.");
                break;
            }
        }
    }

    loop {
        cortex_m::asm::wfi();
    }
}
