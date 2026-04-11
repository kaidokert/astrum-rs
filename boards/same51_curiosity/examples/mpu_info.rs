#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use atsamd_hal::pac as _;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let p = cortex_m::Peripherals::take().unwrap();

    let mpu_type = p.MPU._type.read();
    let mpu_ctrl = p.MPU.ctrl.read();
    let dregion_count = (mpu_type >> 8) & 0xff;

    rprintln!("ATSAME51 MPU Info");
    rprintln!("MPU TYPE register: 0x{:08x}", mpu_type);
    rprintln!("MPU CTRL register: 0x{:08x}", mpu_ctrl);
    rprintln!("MPU regions: {}", dregion_count);
    rprintln!("done");

    loop {
        cortex_m::asm::wfi();
    }
}
