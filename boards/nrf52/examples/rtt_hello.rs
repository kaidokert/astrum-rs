//! RTT Hello World — PCA10100 (nRF52833-DK)
//!
//! Blinks the onboard LED (P0.13, active-low) and prints a counter via RTT.
//!
//! Build:  cd nrf52 && cargo build --example rtt_hello
//! Flash:  cargo run --example rtt_hello
//!         (probe-rs pinned in .cargo/config.toml to SN 000685893034)
//! RTT:    probe-rs pipes RTT to stdout automatically.
//!
//! Power cycle board if needed:
//!   uhubctl -l 1-2 -p 1 -a cycle

#![no_std]
#![no_main]

use cortex_m as _;  // pulls in critical-section-single-core impl
use cortex_m_rt::entry;
use nrf52833_pac::Peripherals;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

// PCA10100: LED1 = P0.13 (active-low)
const LED1: u32 = 1 << 13;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== RTT Hello from PCA10100 nRF52833 ===");

    let dp = Peripherals::take().unwrap();

    // Configure P0.13 as output, start with LED off (active-low → drive high)
    dp.P0.dirset.write(|w| unsafe { w.bits(LED1) });
    dp.P0.outset.write(|w| unsafe { w.bits(LED1) });

    let mut counter: u32 = 0;
    loop {
        counter = counter.wrapping_add(1);
        rprintln!("Hello from nRF52833! count={}", counter);

        // Toggle LED1: turn on (clear = low = active)
        dp.P0.outclr.write(|w| unsafe { w.bits(LED1) });
        spin_delay(400_000);

        // Turn off (set = high = inactive)
        dp.P0.outset.write(|w| unsafe { w.bits(LED1) });
        spin_delay(400_000);
    }
}

#[inline(never)]
fn spin_delay(cycles: u32) {
    for _ in 0..cycles {
        core::hint::spin_loop();
    }
}
