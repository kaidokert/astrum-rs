#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Hello from STM32F207ZG — NUCLEO-F207ZG (Cortex-M3, 128KB SRAM, MPU, no FPU)");

    let mut i: u32 = 0;
    loop {
        rprintln!("[{}] alive", i);
        cortex_m::asm::delay(16_000_000); // ~1s at 16 MHz HSI
        i += 1;
    }
}
