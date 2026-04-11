#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::ehal::delay::DelayNs;
use hal::ehal_nb::serial::Write;
use hal::fugit::RateExtU32;
use hal::nb;
use hal::pac::{CorePeripherals, Peripherals};
use hal::sercom::{uart, Sercom2};

type VcomPads = uart::Pads<Sercom2, hal::typelevel::NoneT, hal::gpio::Pin<hal::gpio::PA12, hal::gpio::AlternateC>>;
type VcomUart = uart::Uart<uart::Config<VcomPads>, uart::Tx>;

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
    let gclk0 = clocks.gclk0();
    let pads = uart::Pads::<Sercom2>::default().tx(pins.pa12);
    let mut uart: VcomUart = uart::Config::new(
        &mut peripherals.mclk,
        peripherals.sercom2,
        pads,
        clocks.sercom2_core(&gclk0).unwrap().freq(),
    )
    .baud(
        115_200.Hz(),
        uart::BaudMode::Fractional(uart::Oversampling::Bits16),
    )
    .enable();

    loop {
        for &byte in b"same51 vcom uart alive\r\n" {
            nb::block!(uart.write(byte)).ok();
        }
        delay.delay_ms(1000);
    }
}
