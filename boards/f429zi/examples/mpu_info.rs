#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use cortex_m_rt::entry;
use stm32f4xx_hal as _; // Include HAL for device interrupt vectors

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // Add delay for RTT to initialize
    for _ in 0..10000 {
        cortex_m::asm::nop();
    }

    rprintln!("STM32F429ZI MPU Info");

    let p = cortex_m::Peripherals::take().unwrap();
    let mpu_type = p.MPU._type.read();
    let dregion_count = (mpu_type >> 8) & 0xFF;

    rprintln!("MPU TYPE register: 0x{:08x}", mpu_type);
    rprintln!("MPU regions: {}", dregion_count);
    rprintln!("Done.");

    loop {
        cortex_m::asm::wfi();
    }
}
