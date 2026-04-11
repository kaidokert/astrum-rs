#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;

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
    // The Graphics & Touch Curiosity board user LED is on PB02 and is active-low.
    let mut led = pins.pb02.into_push_pull_output();
    let _ = led.set_high();

    loop {
        let _ = led.set_low();
        delay.delay_ms(200u16);
        let _ = led.set_high();
        delay.delay_ms(200u16);
    }
}
