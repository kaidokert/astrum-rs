//! SPI FRAM Probe — MB85RS64V bare-metal ID read + write/verify
//!
//! Reads RDID (0x9F) from the MB85RS64V 8KB SPI FRAM.
//! Expected: manufacturer=0x04 (Fujitsu), density=0x03 (64Kbit).
//! Then does a write/read-back test at address 0x0000.
//!
//! Hardware: SPI1 on PA5(SCK)/PA6(MISO)/PA7(MOSI), CS on PD14 (Arduino D10).
//!
//! Build: cd f429zi && cargo build --example spi_fram_probe --features hal

#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use cortex_m_rt::entry;

use stm32f4xx_hal::{
    gpio::{Output, PushPull, Pin},
    pac,
    prelude::*,
    rcc::Config,
    spi::{Mode, Phase, Polarity, Spi},
};

// Use embedded-hal 1.0 SpiDevice/SpiBus — HAL's Spi implements transfer_in_place().

// MB85RS64V SPI opcodes
const WREN: u8 = 0x06;
const WRITE: u8 = 0x02;
const READ: u8 = 0x03;
const RDID: u8 = 0x9F;
const RDSR: u8 = 0x05;

type CsPin = Pin<'D', 14, Output<PushPull>>;

fn cs_low(cs: &mut CsPin) { cs.set_low(); }
fn cs_high(cs: &mut CsPin) { cs.set_high(); }

#[entry]
fn main() -> ! {
    rtt_init_print!();
    // Delay for probe-rs to detect RTT control block. At 16 MHz HSI (pre-PLL),
    // 32M cycles ≈ 2 seconds — enough for probe-rs RTT scan.
    cortex_m::asm::delay(32_000_000);
    rprintln!("\n=== SPI FRAM Probe — MB85RS64V ===");

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz()).sysclk(168.MHz()),
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    let mut cs = gpiod.pd14.into_push_pull_output();
    cs.set_high();

    // Pass raw pins — Spi::new() configures alternate function internally.
    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let mut spi = Spi::new(
        dp.SPI1,
        (Some(sck), Some(miso), Some(mosi)),
        mode,
        1.MHz(),
        &mut rcc,
    );

    rprintln!("[INIT] SPI1: PA5/PA6/PA7, 1 MHz, Mode 0, CS=PD14");

    // ── RDID ──
    rprintln!("\n--- RDID ---");
    let mut buf = [RDID, 0, 0, 0, 0];
    cs_low(&mut cs);
    spi.transfer_in_place(&mut buf).ok();
    cs_high(&mut cs);
    rprintln!("  raw: {:02x?}", &buf[1..]);
    rprintln!("  manufacturer: 0x{:02x} (expect 0x04=Fujitsu)", buf[1]);
    rprintln!("  continuation: 0x{:02x}", buf[2]);
    rprintln!("  density:      0x{:02x}{:02x}", buf[3], buf[4]);
    let fujitsu = buf[1] == 0x04;

    // ── Status Register ──
    rprintln!("\n--- Status Register ---");
    let mut sr = [RDSR, 0];
    cs_low(&mut cs);
    spi.transfer_in_place(&mut sr).ok();
    cs_high(&mut cs);
    rprintln!("  SR: 0x{:02x}", sr[1]);

    // ── Write/Read test ──
    rprintln!("\n--- Write/Read Test ---");
    let test_data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

    // WREN
    let mut wren = [WREN];
    cs_low(&mut cs);
    spi.transfer_in_place(&mut wren).ok();
    cs_high(&mut cs);

    // WRITE at 0x0000
    let mut wcmd = [WRITE, 0x00, 0x00, test_data[0], test_data[1], test_data[2], test_data[3]];
    cs_low(&mut cs);
    spi.transfer_in_place(&mut wcmd).ok();
    cs_high(&mut cs);
    rprintln!("  wrote {:02x?} at 0x0000", test_data);

    // READ at 0x0000
    let mut rcmd = [READ, 0x00, 0x00, 0, 0, 0, 0];
    cs_low(&mut cs);
    spi.transfer_in_place(&mut rcmd).ok();
    cs_high(&mut cs);
    let readback = [rcmd[3], rcmd[4], rcmd[5], rcmd[6]];
    rprintln!("  read  {:02x?} at 0x0000", readback);

    let match_ok = readback == test_data;
    rprintln!("  verify: {}", if match_ok { "MATCH" } else { "MISMATCH" });

    if fujitsu && match_ok {
        rprintln!("\nSUCCESS: MB85RS64V SPI FRAM detected and verified.");
    } else {
        rprintln!("\nFAIL: fujitsu={} match={}", fujitsu, match_ok);
    }

    // Spin (not wfi) so probe-rs can poll RTT while CPU is running.
    loop { cortex_m::asm::nop(); }
}
