#![no_main]
#![no_std]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::prelude::*;

#[entry]
fn main() -> ! {
    // Get peripherals
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    // Configure clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // Configure LED GPIOs on Port B
    let gpiob = dp.GPIOB.split();

    // NUCLEO-F429ZI LEDs:
    // LD1 (Green): PB0
    // LD2 (Blue):  PB7
    // LD3 (Red):   PB14
    let mut led1_green = gpiob.pb0.into_push_pull_output();
    let mut led2_blue = gpiob.pb7.into_push_pull_output();
    let mut led3_red = gpiob.pb14.into_push_pull_output();

    // Start with all LEDs off
    led1_green.set_low();
    led2_blue.set_low();
    led3_red.set_low();

    let delay_cycles = clocks.sysclk().raw() / 4; // ~250ms at default clock

    loop {
        // Pattern 1: Green only
        led1_green.set_high();
        led2_blue.set_low();
        led3_red.set_low();
        cortex_m::asm::delay(delay_cycles);

        // Pattern 2: Blue only
        led1_green.set_low();
        led2_blue.set_high();
        led3_red.set_low();
        cortex_m::asm::delay(delay_cycles);

        // Pattern 3: Red only
        led1_green.set_low();
        led2_blue.set_low();
        led3_red.set_high();
        cortex_m::asm::delay(delay_cycles);

        // Pattern 4: All on (White)
        led1_green.set_high();
        led2_blue.set_high();
        led3_red.set_high();
        cortex_m::asm::delay(delay_cycles);

        // Pattern 5: All off
        led1_green.set_low();
        led2_blue.set_low();
        led3_red.set_low();
        cortex_m::asm::delay(delay_cycles);
    }
}
