//! Bare UARTE0 TX test — no kernel, no MPU.
//! Sends "HELLO\n" every 500ms on P1.01 (PCA10100 J-Link VCOM).
//! If nothing appears on ttyACM0, the issue is hardware/pin config.
//!
//! Build: cd nrf52 && cargo build --example uarte_bare_tx --features bare

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52833_pac as pac;
use rtt_target::rprintln;

use panic_halt as _;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();

    // P0.06 = TX, P0.08 = RX (PCA10100 VCOM via SB40/SB41)
    dp.UARTE0.psel.txd.write(|w| unsafe {
        w.pin().bits(6).port().bit(false).connect().connected()
    });
    dp.UARTE0.psel.rxd.write(|w| unsafe {
        w.pin().bits(8).port().bit(false).connect().connected()
    });
    dp.UARTE0.psel.cts.write(|w| w.connect().disconnected());
    dp.UARTE0.psel.rts.write(|w| w.connect().disconnected());

    dp.UARTE0.baudrate.write(|w| w.baudrate().baud115200());
    dp.UARTE0.config.write(|w| w.parity().excluded().hwfc().disabled().stop().one());
    dp.UARTE0.enable.write(|w| w.enable().enabled());

    rprintln!("UARTE0 bare TX test — P1.01 @ 115200");

    let msg: [u8; 6] = *b"HELLO\n";
    let mut cycle: u32 = 0;

    loop {
        // Clear event
        dp.UARTE0.events_endtx.write(|w| unsafe { w.bits(0) });

        // Set up DMA
        dp.UARTE0.txd.ptr.write(|w| unsafe { w.ptr().bits(msg.as_ptr() as u32) });
        dp.UARTE0.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(msg.len() as u16) });

        // Start TX
        dp.UARTE0.tasks_starttx.write(|w| unsafe { w.bits(1) });

        // Poll for ENDTX
        while dp.UARTE0.events_endtx.read().bits() == 0 {}

        cycle += 1;
        if cycle % 10 == 0 {
            rprintln!("[bare] TX cycle={}", cycle);
        }

        // Delay ~500ms at 64MHz
        cortex_m::asm::delay(32_000_000);
    }
}
