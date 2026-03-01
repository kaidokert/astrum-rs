#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use kernel as _;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let mpu_type = p.MPU._type.read();
    let dregion_count = (mpu_type >> 8) & 0xFF;
    hprintln!("MPU regions: {}", dregion_count);
    debug::exit(debug::EXIT_SUCCESS);
    loop {
        cortex_m::asm::wfi();
    }
}
