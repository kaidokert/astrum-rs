//! STM32F4 Discovery board — LED on PG13, SysTick delay.

use crate::Board;

use stm32f4xx_hal::{
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
    rcc::Config,
    timer::SysDelay,
};

/// STM32F4 Discovery board (STM32F407VG).
///
/// LED1 (green) is on PG13. Delay uses SysTick at 168 MHz.
pub struct Stm32f4Discovery;

impl Board for Stm32f4Discovery {
    type Led = Pin<'G', 13, Output<PushPull>>;
    type Delay = SysDelay;

    fn init() -> (Self::Led, Self::Delay) {
        let dp = pac::Peripherals::take().expect("pac::Peripherals::take() called more than once");
        let cp = cortex_m::Peripherals::take()
            .expect("cortex_m::Peripherals::take() called more than once");

        let mut rcc = dp.RCC.freeze(Config::hse(8.MHz()).sysclk(168.MHz()));

        let gpiog = dp.GPIOG.split(&mut rcc);
        let led = gpiog.pg13.into_push_pull_output();

        let delay = cp.SYST.delay(&rcc.clocks);

        (led, delay)
    }
}
