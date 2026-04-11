#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity::lcd::{HEIGHT, Ili9488Lcd, WIDTH};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
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
    lcd.clear(Rgb565::BLACK).unwrap();

    let border = PrimitiveStyle::with_stroke(Rgb565::WHITE, 2);
    let accent = PrimitiveStyle::with_stroke(Rgb565::CYAN, 2);
    let filled = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::MAGENTA)
        .stroke_color(Rgb565::YELLOW)
        .stroke_width(3)
        .build();
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);

    Rectangle::new(Point::new(10, 10), Size::new(u32::from(WIDTH - 20), u32::from(HEIGHT - 20)))
        .into_styled(border)
        .draw(&mut lcd)
        .unwrap();

    Line::new(Point::new(20, 20), Point::new((WIDTH - 21) as i32, (HEIGHT - 21) as i32))
        .into_styled(accent)
        .draw(&mut lcd)
        .unwrap();

    Line::new(Point::new((WIDTH - 21) as i32, 20), Point::new(20, (HEIGHT - 21) as i32))
        .into_styled(accent)
        .draw(&mut lcd)
        .unwrap();

    Circle::with_center(
        Point::new((WIDTH / 2) as i32, (HEIGHT / 2) as i32),
        96,
    )
    .into_styled(filled)
    .draw(&mut lcd)
    .unwrap();

    Text::new("embedded-graphics", Point::new(70, 40), text_style)
        .draw(&mut lcd)
        .unwrap();

    Text::new("SAME51 + ILI9488", Point::new(105, HEIGHT as i32 - 36), text_style)
        .draw(&mut lcd)
        .unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}
