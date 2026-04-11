//! Minimal RTT hello — no defmt, no kernel, just rtt-target.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use atsamd_hal::pac as _; // link __INTERRUPTS

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== SAME51 rtt_hello ===");

    let mut count: u32 = 0;
    loop {
        rprintln!("[{}] alive", count);
        count += 1;
        cortex_m::asm::delay(4_000_000);
    }
}
