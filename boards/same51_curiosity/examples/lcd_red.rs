#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use embedded_graphics_core::{
    draw_target::DrawTarget,
    pixelcolor::{Rgb565, RgbColor},
};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use mipidsi::{
    interface::{Generic8BitBus, ParallelInterface},
    models::ILI9488Rgb565,
    options::{ColorOrder, Orientation, Rotation},
    Builder,
};

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

    // EV14C17A LCD wiring:
    // PB04=CS, PB22=D/C, PB23=WR, PB16=RD, PB05=RST, PB17=TE, PA15=BL-
    // PA00..PA07 = D0..D7
    let mut cs = pins.pb04.into_push_pull_output();
    let dc = pins.pb22.into_push_pull_output();
    let wr = pins.pb23.into_push_pull_output();
    let mut rd = pins.pb16.into_push_pull_output();
    let rst = pins.pb05.into_push_pull_output();
    let mut backlight = pins.pa15.into_push_pull_output();

    let d0 = pins.pa00.into_push_pull_output();
    let d1 = pins.pa01.into_push_pull_output();
    let d2 = pins.pa02.into_push_pull_output();
    let d3 = pins.pa03.into_push_pull_output();
    let d4 = pins.pa04.into_push_pull_output();
    let d5 = pins.pa05.into_push_pull_output();
    let d6 = pins.pa06.into_push_pull_output();
    let d7 = pins.pa07.into_push_pull_output();

    // Ignore TE for now; read strobe is held inactive.
    let _te = pins.pb17;

    let _ = rd.set_high();
    let _ = cs.set_low();
    let _ = backlight.set_high();

    let bus = Generic8BitBus::new((d0, d1, d2, d3, d4, d5, d6, d7));
    let di = ParallelInterface::new(bus, dc, wr);

    let orientation = Orientation::default().rotate(Rotation::Deg90);

    let mut display = Builder::new(ILI9488Rgb565, di)
        .reset_pin(rst)
        .display_size(320, 480)
        .orientation(orientation)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    display.clear(Rgb565::RED).unwrap();

    // Backlight is active-low on this board.
    let _ = backlight.set_low();

    loop {
        cortex_m::asm::wfi();
    }
}
