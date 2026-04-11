//! Bidirectional USART6 echo test — wiring + adapter loopback diagnostics
//!
//! Sends incrementing bytes (0x01..=0xFE) to the host echo server and
//! expects each byte echoed back.  Also checks for a *second* echo within
//! 20 ms: if present, the adapter has hardware TX→RX loopback, which
//! explains the cascading MISMATCH pattern seen in uart_irq_partition_echo.
//!
//! Two scenarios it can distinguish:
//!   A) Single echo  — host round-trip only, no adapter loopback.
//!   B) Double echo  — fast adapter loopback first (~1 ms), then the
//!                     slower host echo (~5–30 ms later).
//!
//! Hardware: STM32F429ZI NUCLEO-144
//!   PG14 = USART6_TX (AF8) — CN10 D1 (TX0)  → adapter RX
//!   PG9  = USART6_RX (AF8) — CN10 D0 (RX0)  ← adapter TX
//!   GND                                       — adapter GND
//!
//! Host:  ./run_echo_server.sh --stop-at 9600   (start BEFORE MCU reset)
//! Build: cd f429zi && cargo build --example uart_echo --features hal
//! Flash: (via GDB + OpenOCD)

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    gpio::GpioExt,
    pac,
    pac::USART6,
    prelude::*,
    serial::{config::Config, Rx, Serial},
};

// HSI = 16 MHz (default after reset, no PLL).
const CPU_HZ: u32 = 16_000_000;
// Cycles per millisecond for asm::delay().
const CYC_1MS: u32 = CPU_HZ / 1_000; // 16_000

/// Non-blocking poll on `rx` for up to `timeout_ms` milliseconds.
/// Checks once per millisecond.  Returns `Some(byte)` or `None` on timeout.
fn rx_poll(rx: &mut Rx<USART6>, timeout_ms: u32) -> Option<u8> {
    for _ in 0..timeout_ms {
        match rx.read() {
            Ok(b) => return Some(b),
            Err(_) => {} // WouldBlock or framing/overrun — keep waiting
        }
        cortex_m::asm::delay(CYC_1MS);
    }
    None
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("uart_hello bidir starting — USART6 PG14(TX)/PG9(RX) 9600 baud");

    let mut dp = pac::Peripherals::take().unwrap();

    // GPIO setup (same as before — PAC RCC borrow for clock enable).
    let gpiog = dp.GPIOG.split(&mut dp.RCC);
    let pg14 = gpiog.pg14.into_alternate::<8>(); // USART6_TX, AF8
    let pg9 = gpiog.pg9.into_alternate::<8>(); // USART6_RX, AF8

    // Constrain RCC → HSI 16 MHz default.  9600 baud at pclk2=16 MHz:
    //   BRR = 16_000_000 / 9_600 = 1666.67 → mantissa=104, frac=11 → 9597 bps (0.03%)
    let mut rcc = dp.RCC.constrain();

    let serial = Serial::new(
        dp.USART6,
        (pg14, pg9),
        Config::default().baudrate(9_600.bps()),
        &mut rcc,
    )
    .unwrap();

    let (mut tx, mut rx) = serial.split();

    // ── Startup drain ────────────────────────────────────────────────────────
    // Poll RX for ~500 ms to discard any power-on glitches or stale bytes.
    rprintln!("draining RX ~500 ms...");
    for _ in 0..500_u32 {
        let _ = rx.read();
        cortex_m::asm::delay(CYC_1MS);
    }
    rprintln!("drain done — echo test starting");
    rprintln!("  start echo server: ./run_echo_server.sh --stop-at 9600");
    rprintln!("  Scenario A=single echo (OK), Scenario B=double echo (loopback)");

    let mut sent_total: u32 = 0;
    let mut ok: u32 = 0;
    let mut mismatch: u32 = 0;
    let mut timeout_n: u32 = 0;
    let mut double_echo: u32 = 0;

    loop {
        // Cycle 0x01..=0xFE  (skip 0x00 and 0xFF which are reserved signals)
        for val in 0x01_u8..=0xFE_u8 {
            // ── Transmit ─────────────────────────────────────────────────────
            // bwrite_all blocks until TXE; one byte at 9600 baud ≈ 1.04 ms.
            tx.bwrite_all(&[val]).ok();
            sent_total += 1;

            // ── Wait for first echo (30 ms covers USB CDC round-trip) ────────
            match rx_poll(&mut rx, 30) {
                None => {
                    timeout_n += 1;
                    if timeout_n <= 8 {
                        rprintln!(
                            "[TO] #{}: val=0x{:02x} ok={} mm={} to={} dbl={}",
                            timeout_n,
                            val,
                            ok,
                            mismatch,
                            timeout_n,
                            double_echo
                        );
                    }
                }

                Some(echo1) if echo1 == val => {
                    ok += 1;

                    // ── Check for second echo (20 ms) ─────────────────────
                    // Adapter hardware loopback arrives fast (~1 ms);
                    // the host's delayed echo follows ~5–30 ms later.
                    // If there's NO loopback, nothing arrives here.
                    match rx_poll(&mut rx, 20) {
                        Some(echo2) => {
                            double_echo += 1;
                            if double_echo <= 12 {
                                rprintln!(
                                    "[DBL] #{}: val=0x{:02x} e1=0x{:02x} e2=0x{:02x} \
                                     ok={} dbl={}",
                                    double_echo,
                                    val,
                                    echo1,
                                    echo2,
                                    ok,
                                    double_echo
                                );
                            }
                        }
                        None => {} // clean — exactly one echo, no loopback
                    }
                }

                Some(echo1) => {
                    mismatch += 1;
                    if mismatch <= 10 {
                        rprintln!(
                            "[MM] #{}: sent=0x{:02x} got=0x{:02x} ok={} mm={}",
                            mismatch,
                            val,
                            echo1,
                            ok,
                            mismatch
                        );
                    }
                }
            }

            // Progress every 32 bytes.
            if sent_total % 32 == 0 {
                rprintln!(
                    "[I] sent={} ok={} mm={} to={} dbl={}",
                    sent_total,
                    ok,
                    mismatch,
                    timeout_n,
                    double_echo
                );
            }
        }

        rprintln!(
            "[DONE] round: sent={} ok={} mm={} to={} dbl={}",
            sent_total,
            ok,
            mismatch,
            timeout_n,
            double_echo
        );

        // Brief pause between rounds so RTT can drain.
        cortex_m::asm::delay(CPU_HZ); // ~1 s
    }
}
