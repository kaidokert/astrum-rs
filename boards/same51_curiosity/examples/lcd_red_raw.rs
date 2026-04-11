#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity::lcd::Ili9488Lcd;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let pins = hal::gpio::Pins::new(peripherals.port);
    let mut lcd = Ili9488Lcd::from_board_pins(pins);

    lcd.init(&mut delay);
    lcd.fill_rgb565(0xF800);

    loop {
        cortex_m::asm::wfi();
    }
}
