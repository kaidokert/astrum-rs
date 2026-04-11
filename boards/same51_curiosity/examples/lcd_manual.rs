#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity::lcd::{HEIGHT, Ili9488Lcd, WIDTH};

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;
const RED: u16 = 0xF800;
const GREEN: u16 = 0x07E0;
const BLUE: u16 = 0x001F;
const YELLOW: u16 = 0xFFE0;
const CYAN: u16 = 0x07FF;
const MAGENTA: u16 = 0xF81F;

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
    lcd.fill_rgb565(BLACK);

    // Outer border.
    lcd.hline(0, 0, WIDTH, WHITE);
    lcd.hline(0, HEIGHT - 1, WIDTH, WHITE);
    lcd.vline(0, 0, HEIGHT, WHITE);
    lcd.vline(WIDTH - 1, 0, HEIGHT, WHITE);

    // Center crosshair.
    lcd.hline(0, HEIGHT / 2, WIDTH, CYAN);
    lcd.vline(WIDTH / 2, 0, HEIGHT, CYAN);

    // Corner blocks.
    lcd.fill_rect(16, 16, 48, 48, RED);
    lcd.fill_rect(WIDTH - 64, 16, 48, 48, GREEN);
    lcd.fill_rect(16, HEIGHT - 64, 48, 48, BLUE);
    lcd.fill_rect(WIDTH - 64, HEIGHT - 64, 48, 48, MAGENTA);

    // Three horizontal color bars in the middle.
    let bar_x = 80;
    let bar_w = WIDTH - 160;
    lcd.fill_rect(bar_x, 70, bar_w, 24, RED);
    lcd.fill_rect(bar_x, 102, bar_w, 24, GREEN);
    lcd.fill_rect(bar_x, 134, bar_w, 24, BLUE);

    // One vertical marker bar and a center box.
    lcd.fill_rect(WIDTH / 2 - 6, 40, 12, HEIGHT - 80, YELLOW);
    lcd.fill_rect(WIDTH / 2 - 40, HEIGHT / 2 - 30, 80, 60, MAGENTA);

    loop {
        cortex_m::asm::wfi();
    }
}
