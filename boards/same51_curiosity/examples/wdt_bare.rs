//! Watchdog Timer — Bare-metal demo (SAME51 Curiosity Nano)
//!
//! Starts the WDT with a ~2s timeout (Cycles2K @ 1024 Hz). Feeds it in a
//! loop, toggling the LED (PA14) each second as a visual heartbeat.
//! Halt the CPU with a debugger → WDT resets the MCU after ~2 seconds
//! (LED stops toggling, then restarts from the blink pattern).
//!
//! Build: cd same51_curiosity && cargo build --example wdt_bare --release
//! Flash: probe-rs download --chip ATSAME51J20A --probe 03eb:2175 <elf>

#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use hal::clock::GenericClockController;
use hal::gpio::{Pin, PushPullOutput, PA14};
use hal::pac::Peripherals;
use hal::watchdog::{Watchdog, WatchdogTimeout};
use embedded_hal_02::watchdog::{Watchdog as _, WatchdogEnable as _};
use embedded_hal_02::digital::v2::ToggleableOutputPin;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    let _clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );

    // LED on PA14 (active low on Curiosity Nano).
    let pins = hal::gpio::Pins::new(peripherals.port);
    let mut led: Pin<PA14, PushPullOutput> = pins.pa14.into_push_pull_output();

    // WDT: ~2 second timeout (2048 cycles / 1024 Hz).
    let mut wdt = Watchdog::new(peripherals.wdt);
    wdt.start(WatchdogTimeout::Cycles2K as u8);

    loop {
        wdt.feed();
        led.toggle().ok();
        cortex_m::asm::delay(60_000_000); // ~500ms at 120 MHz
    }
}
