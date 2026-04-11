//! GPIO Blinky — PCA10100 (nRF52833-DK)
//!
//! Blinks all 4 onboard LEDs (active-low) using the nrf52833-pac register API.
//!
//! PCA10100 LED pinout (all active-low on P0):
//!   LED1 = P0.13   LED2 = P0.14   LED3 = P0.15   LED4 = P0.16
//!
//! Build:  cargo build --example blinky
//! Flash:  cargo run --example blinky

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52833_pac::Peripherals;
use panic_halt as _;

// LED bitmask: P0.13 | P0.14 | P0.15 | P0.16
const LEDS: u32 = (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16);

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    // Configure all 4 LED pins as outputs
    dp.P0.dirset.write(|w| unsafe { w.bits(LEDS) });
    // Start with all LEDs off (active-low → drive high)
    dp.P0.outset.write(|w| unsafe { w.bits(LEDS) });

    loop {
        // All on (active-low → clear = drive low)
        dp.P0.outclr.write(|w| unsafe { w.bits(LEDS) });
        spin_delay(1_000_000);

        // All off (active-low → set = drive high)
        dp.P0.outset.write(|w| unsafe { w.bits(LEDS) });
        spin_delay(1_000_000);
    }
}

#[inline(never)]
fn spin_delay(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}
