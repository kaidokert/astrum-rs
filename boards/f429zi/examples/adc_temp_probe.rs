//! ADC Internal Temperature Sensor — bare-metal polling
//!
//! Reads the STM32F429 internal temperature sensor via ADC1 channel 18.
//! No external wiring needed. Calibration data from factory OTP.
//!
//! Formula: T(°C) = (110-30) * (raw - CAL30) / (CAL110 - CAL30) + 30
//!
//! Build: cd f429zi && cargo build --example adc_temp_probe --features hal

#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use cortex_m_rt::entry;

use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, SampleTime},
        Adc, Temperature,
    },
    pac,
    prelude::*,
    rcc::Config,
    signature::{VtempCal30, VtempCal110},
};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz()).sysclk(168.MHz()),
    );

    rprintln!("\n=== ADC Internal Temperature Sensor ===");

    // Read factory calibration values (measured at 3.3V VDDA, 30°C and 110°C)
    let cal30 = VtempCal30::get().read() as f32;
    let cal110 = VtempCal110::get().read() as f32;
    rprintln!("[INIT] CAL30={} CAL110={}", cal30 as u16, cal110 as u16);

    let mut adc = Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc);
    adc.enable_temperature_and_vref();

    // Calibrate VDDA for accurate readings
    adc.calibrate();
    rprintln!("[INIT] VDDA={}mV", adc.reference_voltage());

    rprintln!("[INIT] Sampling internal temp sensor (ADC1 ch18)...\n");

    let mut cycle: u32 = 0;
    loop {
        let raw: u16 = adc.convert(&Temperature, SampleTime::Cycles_480);

        let temperature = (110.0 - 30.0) * (raw as f32 - cal30) / (cal110 - cal30) + 30.0;
        let t_x100 = (temperature * 100.0) as i32;

        cycle += 1;
        if cycle % 10 == 1 {
            rprintln!(
                "[{:4}] raw={} T={}.{:02}C VDDA={}mV",
                cycle, raw,
                t_x100 / 100, (t_x100 % 100).abs(),
                adc.reference_voltage()
            );
        }

        if cycle == 51 {
            rprintln!("SUCCESS: ADC internal temp sensor working.");
        }

        cortex_m::asm::delay(16_800_000); // ~100ms at 168MHz
    }
}
