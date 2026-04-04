//! nRF52840 DK board — LED1 on P0.13, Timer0 delay.

use crate::Board;

use nrf52840_hal::{
    gpio::{p0, Level, Output, Pin, PushPull},
    pac, Timer,
};

/// nRF52840 Development Kit board.
///
/// LED1 is active-low on P0.13. Delay uses TIMER0 in one-shot mode.
pub struct Nrf52840Dk;

impl Board for Nrf52840Dk {
    type Led = Pin<Output<PushPull>>;
    type Delay = Timer<pac::TIMER0>;

    fn init() -> (Self::Led, Self::Delay) {
        let dp = pac::Peripherals::take().expect("pac::Peripherals::take() called more than once");

        let port0 = p0::Parts::new(dp.P0);
        // LED1 is active-low; start with LED off (pin high).
        let led = port0.p0_13.into_push_pull_output(Level::High).degrade();

        let delay = Timer::new(dp.TIMER0);

        (led, delay)
    }
}
