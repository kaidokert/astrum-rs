use atsamd_hal::{
    ehal::delay::DelayNs,
    gpio::{
        Pin, Pins, PushPullOutput, PA00, PA01, PA02, PA03, PA04, PA05, PA06, PA07, PA15, PB04,
        PB05, PB16, PB22, PB23,
    },
    prelude::*,
};
use core::convert::Infallible;
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::{IntoStorage, Rgb565},
    Pixel,
};

pub const WIDTH: u16 = 480;
pub const HEIGHT: u16 = 320;

pub struct Ili9488Lcd {
    d0: Pin<PA00, PushPullOutput>,
    d1: Pin<PA01, PushPullOutput>,
    d2: Pin<PA02, PushPullOutput>,
    d3: Pin<PA03, PushPullOutput>,
    d4: Pin<PA04, PushPullOutput>,
    d5: Pin<PA05, PushPullOutput>,
    d6: Pin<PA06, PushPullOutput>,
    d7: Pin<PA07, PushPullOutput>,
    dc: Pin<PB22, PushPullOutput>,
    wr: Pin<PB23, PushPullOutput>,
    cs: Pin<PB04, PushPullOutput>,
    rd: Pin<PB16, PushPullOutput>,
    rst: Pin<PB05, PushPullOutput>,
    bl: Pin<PA15, PushPullOutput>,
}

impl Ili9488Lcd {
    pub fn from_board_pins(pins: Pins) -> Self {
        let _te = pins.pb17;

        Self {
            d0: pins.pa00.into_push_pull_output(),
            d1: pins.pa01.into_push_pull_output(),
            d2: pins.pa02.into_push_pull_output(),
            d3: pins.pa03.into_push_pull_output(),
            d4: pins.pa04.into_push_pull_output(),
            d5: pins.pa05.into_push_pull_output(),
            d6: pins.pa06.into_push_pull_output(),
            d7: pins.pa07.into_push_pull_output(),
            dc: pins.pb22.into_push_pull_output(),
            wr: pins.pb23.into_push_pull_output(),
            cs: pins.pb04.into_push_pull_output(),
            rd: pins.pb16.into_push_pull_output(),
            rst: pins.pb05.into_push_pull_output(),
            bl: pins.pa15.into_push_pull_output(),
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayNs) {
        let _ = self.wr.set_high();
        let _ = self.rd.set_high();
        let _ = self.cs.set_low();
        let _ = self.dc.set_high();

        self.backlight_on();

        let _ = self.rst.set_low();
        delay.delay_ms(10);
        let _ = self.rst.set_high();
        delay.delay_ms(30);

        // Harmony-generated init sequence for EV14C17A / ILI9488.
        self.write_cmd_data(0x3A, &[0x05]); // RGB565
        self.write_cmd_data(0xE9, &[0x01]); // image function
        self.write_cmd_data(0xB0, &[0x00]); // interface mode control
        self.write_cmd_data(0x36, &[0x28]); // memory access control
        self.write_cmd(0x11); // sleep out
        delay.delay_ms(120);
        self.write_cmd(0x29); // display on
        delay.delay_ms(20);
    }

    pub fn backlight_on(&mut self) {
        let _ = self.bl.set_high();
    }

    pub fn backlight_off(&mut self) {
        let _ = self.bl.set_low();
    }

    pub fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        self.write_cmd_data(
            0x2A,
            &[
                (x0 >> 8) as u8,
                x0 as u8,
                (x1 >> 8) as u8,
                x1 as u8,
            ],
        );
        self.write_cmd_data(
            0x2B,
            &[
                (y0 >> 8) as u8,
                y0 as u8,
                (y1 >> 8) as u8,
                y1 as u8,
            ],
        );
        self.write_cmd(0x2C);
    }

    pub fn fill_rgb565(&mut self, color: u16) {
        self.fill_rect(0, 0, WIDTH, HEIGHT, color);
    }

    pub fn fill_rect(&mut self, x: u16, y: u16, width: u16, height: u16, color: u16) {
        if width == 0 || height == 0 || x >= WIDTH || y >= HEIGHT {
            return;
        }

        let x1 = x.saturating_add(width).min(WIDTH) - 1;
        let y1 = y.saturating_add(height).min(HEIGHT) - 1;
        self.set_window(x, y, x1, y1);
        self.write_repeated_color(color, u32::from(x1 - x + 1) * u32::from(y1 - y + 1));
    }

    pub fn hline(&mut self, x: u16, y: u16, length: u16, color: u16) {
        self.fill_rect(x, y, length, 1, color);
    }

    pub fn vline(&mut self, x: u16, y: u16, length: u16, color: u16) {
        self.fill_rect(x, y, 1, length, color);
    }

    pub fn write_cmd_data(&mut self, cmd: u8, data: &[u8]) {
        self.write_cmd(cmd);
        if !data.is_empty() {
            self.write_data(data);
        }
    }

    fn set_data(&mut self, value: u8) {
        if value & (1 << 0) != 0 {
            let _ = self.d0.set_high();
        } else {
            let _ = self.d0.set_low();
        }
        if value & (1 << 1) != 0 {
            let _ = self.d1.set_high();
        } else {
            let _ = self.d1.set_low();
        }
        if value & (1 << 2) != 0 {
            let _ = self.d2.set_high();
        } else {
            let _ = self.d2.set_low();
        }
        if value & (1 << 3) != 0 {
            let _ = self.d3.set_high();
        } else {
            let _ = self.d3.set_low();
        }
        if value & (1 << 4) != 0 {
            let _ = self.d4.set_high();
        } else {
            let _ = self.d4.set_low();
        }
        if value & (1 << 5) != 0 {
            let _ = self.d5.set_high();
        } else {
            let _ = self.d5.set_low();
        }
        if value & (1 << 6) != 0 {
            let _ = self.d6.set_high();
        } else {
            let _ = self.d6.set_low();
        }
        if value & (1 << 7) != 0 {
            let _ = self.d7.set_high();
        } else {
            let _ = self.d7.set_low();
        }
    }

    fn pulse_wr(&mut self) {
        let _ = self.wr.set_low();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        let _ = self.wr.set_high();
    }

    fn write8(&mut self, value: u8) {
        self.set_data(value);
        self.pulse_wr();
    }

    fn write_cmd(&mut self, cmd: u8) {
        let _ = self.dc.set_low();
        self.write8(cmd);
        let _ = self.dc.set_high();
    }

    fn write_data(&mut self, data: &[u8]) {
        let _ = self.dc.set_high();
        for &byte in data {
            self.write8(byte);
        }
    }

    fn write_repeated_color(&mut self, color: u16, count: u32) {
        let hi = (color >> 8) as u8;
        let lo = color as u8;
        let _ = self.dc.set_high();

        for _ in 0..count {
            self.write8(hi);
            self.write8(lo);
        }
    }
}

impl OriginDimensions for Ili9488Lcd {
    fn size(&self) -> Size {
        Size::new(u32::from(WIDTH), u32::from(HEIGHT))
    }
}

impl DrawTarget for Ili9488Lcd {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if point.x < 0 || point.y < 0 {
                continue;
            }

            let x = point.x as u16;
            let y = point.y as u16;
            if x >= WIDTH || y >= HEIGHT {
                continue;
            }

            self.fill_rect(x, y, 1, 1, color.into_storage());
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_rgb565(color.into_storage());
        Ok(())
    }
}
