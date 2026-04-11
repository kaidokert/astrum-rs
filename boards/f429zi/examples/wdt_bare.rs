//! Watchdog Timer — Bare-metal demo (STM32F429ZI)
//!
//! IWDG with ~2s timeout. Feeds in a loop, toggling LED (PB7).
//! Halt CPU with debugger → IWDG resets MCU after ~2s
//! (stop_on_debug disabled so WDT fires even when halted).
//!
//! Build: cd f429zi && cargo build --example wdt_bare --no-default-features
//! Flash: via GDB + OpenOCD

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use stm32f4xx_hal::{
    pac,
    prelude::*,
    watchdog::IndependentWatchdog,
};
use f429zi as _;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== WDT Bare-metal Demo — STM32F429ZI ===");

    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(stm32f4xx_hal::rcc::Config::hse(8.MHz()));

    // LED on PB7 (NUCLEO-144 user LED LD2).
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb7.into_push_pull_output();

    // IWDG: ~2 second timeout.
    let mut wdt = IndependentWatchdog::new(dp.IWDG);
    wdt.stop_on_debug(&dp.DBGMCU, false); // keep WDT running during debug halt
    wdt.start(2000u32.millis());
    rprintln!("[INIT] IWDG started: ~2s timeout, stop_on_debug=false");

    let mut count: u32 = 0;
    loop {
        wdt.feed();
        led.toggle();
        count += 1;
        if count % 100 == 0 {
            rprintln!("[WDT] feed count={}", count);
        }
        cortex_m::asm::delay(168_000); // ~1ms at 168 MHz
    }
}
