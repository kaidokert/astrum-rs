//! BME280 I2C sensor demo — STM32F429ZI NUCLEO-144
//!
//! Reads temperature, humidity, and pressure from a BME280 on I2C1
//! (PB8=SCL, PB9=SDA) and prints via RTT.
//!
//! Build: cd f429zi && cargo build --example i2c_scan --features hal,bme280

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    i2c::I2c,
    pac,
    prelude::*,
};
use bme280::i2c::BME280;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("\n=== BME280 I2C Sensor — PB8(SCL) PB9(SDA) ===\n");

    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(
        stm32f4xx_hal::rcc::Config::hse(8.MHz()).sysclk(168.MHz()),
    );

    let gpiob = dp.GPIOB.split(&mut rcc);
    let scl = gpiob.pb8.internal_pull_up(true);
    let sda = gpiob.pb9.internal_pull_up(true);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);

    let mut delay = cortex_m::Peripherals::take().unwrap().SYST.delay(&rcc.clocks);

    let mut bme = BME280::new_primary(i2c);
    rprintln!("Initializing BME280...");
    match bme.init(&mut delay) {
        Ok(()) => rprintln!("BME280 initialized."),
        Err(e) => {
            rprintln!("BME280 init error: {:?}", e);
            loop { cortex_m::asm::wfi(); }
        }
    }

    rprintln!("Reading sensor data...\n");
    loop {
        match bme.measure(&mut delay) {
            Ok(m) => {
                // Fixed-point display: multiply by 100, print integer with manual decimal.
                let temp_c = (m.temperature * 100.0) as i32;
                let hum_pct = (m.humidity * 100.0) as i32;
                let pres_hpa = (m.pressure / 100.0 * 100.0) as i32; // Pa → hPa × 100
                rprintln!(
                    "T={}.{:02}C  H={}.{:02}%  P={}.{:02}hPa",
                    temp_c / 100, (temp_c % 100).abs(),
                    hum_pct / 100, (hum_pct % 100).abs(),
                    pres_hpa / 100, (pres_hpa % 100).abs(),
                );
            }
            Err(e) => rprintln!("Measure error: {:?}", e),
        }
        cortex_m::asm::delay(168_000_000); // ~1s at 168MHz
    }
}
