//! Bare-minimum USART3 wiring test via ST-LINK VCP — PAC only, no HAL, no DMA, no interrupts.
//!
//! TX: sends "HELLO\n" every ~1s on PD8 (USART3_TX, AF7) → ST-LINK VCP
//! RX: polls PD9 (USART3_RX, AF7) and prints any received bytes via RTT
//!
//! PD8/PD9 are internally wired to the ST-LINK VCP via SB5/SB6 (default closed).
//! No external adapter needed — data appears on the ST-LINK's /dev/ttyACMx.
//!
//! Build: cd f429zi && cargo build --example uart_bare_test --features hal

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::pac as _; // link __INTERRUPTS

const HSI_HZ: u32 = 16_000_000;
const BAUD: u32 = 115_200;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== uart_bare_test — USART3 PD8(TX)/PD9(RX) via ST-LINK VCP {} baud ===", BAUD);

    let rcc = 0x4002_3800 as *mut u32;
    unsafe {
        let ahb1enr = rcc.add(0x30 / 4);
        ahb1enr.write_volatile(ahb1enr.read_volatile() | (1 << 3)); // GPIODEN
        let apb1enr = rcc.add(0x40 / 4);
        apb1enr.write_volatile(apb1enr.read_volatile() | (1 << 18)); // USART3EN
    }

    let gpiod = 0x4002_0C00 as *mut u32;
    unsafe {
        let moder = gpiod.add(0x00 / 4);
        let mut v = moder.read_volatile();
        v &= !((3 << 16) | (3 << 18));
        v |= (2 << 16) | (2 << 18);
        moder.write_volatile(v);

        let afrh = gpiod.add(0x24 / 4);
        let mut v = afrh.read_volatile();
        v &= !((0xF << 0) | (0xF << 4));
        v |= (7 << 0) | (7 << 4);
        afrh.write_volatile(v);
    }

    let usart3 = 0x4000_4800 as *mut u32;
    let (sr, dr, brr, cr1) = unsafe {
        (usart3.add(0x00 / 4), usart3.add(0x04 / 4),
         usart3.add(0x08 / 4), usart3.add(0x0C / 4))
    };
    unsafe {
        brr.write_volatile(HSI_HZ / BAUD);
        cr1.write_volatile((1 << 13) | (1 << 3) | (1 << 2));
    }

    rprintln!("USART3 configured. Sending HELLO every ~1s, polling RX...");

    let msg = b"HELLO\n";
    let mut tx_count: u32 = 0;
    let mut rx_count: u32 = 0;
    let mut round: u32 = 0;

    loop {
        for &b in msg {
            unsafe {
                while sr.read_volatile() & (1 << 7) == 0 {}
                dr.write_volatile(b as u32);
            }
            tx_count += 1;
        }

        for _ in 0..1000_u32 {
            unsafe {
                let status = sr.read_volatile();
                if status & (1 << 5) != 0 {
                    let byte = dr.read_volatile() as u8;
                    rx_count += 1;
                    if rx_count <= 20 {
                        rprintln!("[RX] byte=0x{:02x} '{}' (rx_count={})", byte,
                            if byte >= 0x20 && byte < 0x7F { byte as char } else { '.' },
                            rx_count);
                    }
                }
                if status & (1 << 3) != 0 {
                    let _ = dr.read_volatile();
                }
            }
            cortex_m::asm::delay(16_000);
        }

        round += 1;
        rprintln!("[{}s] TX={} RX={}", round, tx_count, rx_count);
    }
}
