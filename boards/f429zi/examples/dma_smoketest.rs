//! DMA2 Memory-to-Memory Smoketest — STM32F429ZI NUCLEO-144
//!
//! Verifies that DMA2 Stream0 can copy 32 bytes from a static source buffer
//! to a static destination buffer in memory-to-memory mode.
//!
//! DMA2 is the only DMA controller on STM32F4 that supports M2M transfers
//! (RM0090 §10.3.3: "Memory-to-memory mode ... only available for DMA2").
//! DMA1 is peripheral-only.
//!
//! Transfer configuration:
//!   Stream: DMA2 Stream0
//!   Direction: Memory-to-Memory (DIR=0b10)
//!   Data size: Byte (PSIZE=MSIZE=0b00)
//!   Both address increments enabled (PINC=MINC=1)
//!   FIFO mode enabled (DMDIS=1): required for M2M — direct mode not allowed
//!   Priority: Low
//!   Count: 32 bytes
//!
//! RTT success criterion: "DMA2 M2M PASS: all 32 bytes match" logged once.
//!
//! Build: cd f429zi && cargo build --example dma_smoketest --features hal

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::pac::Peripherals;

const LEN: usize = 32;

// Source: filled with 0x00..0x1F at init.
static mut SRC: [u8; LEN] = [0u8; LEN];
// Destination: zeroed; DMA fills it.
static mut DST: [u8; LEN] = [0u8; LEN];

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("\n=== DMA2 Memory-to-Memory Smoketest ===");

    let dp = Peripherals::take().unwrap();

    // ------------------------------------------------------------------
    // 1. Fill SRC with a known pattern and ensure DST is zeroed.
    // ------------------------------------------------------------------
    // Use addr_of_mut! + raw pointer writes to avoid Rust 2024 static_mut_refs
    // hard error (creating &mut from static mut is denied even inside unsafe).
    let src_addr = core::ptr::addr_of_mut!(SRC) as *mut u8;
    let dst_addr = core::ptr::addr_of_mut!(DST) as *mut u8;
    // SAFETY: single-threaded before DMA starts; no other references exist.
    unsafe {
        for i in 0..LEN {
            core::ptr::write_volatile(src_addr.add(i), i as u8);
            core::ptr::write_volatile(dst_addr.add(i), 0u8);
        }
    }
    rprintln!("[INIT] SRC @ 0x{:08X}  DST @ 0x{:08X}", src_addr as u32, dst_addr as u32);

    // ------------------------------------------------------------------
    // 2. Enable DMA2 AHB1 clock (bit 22 of RCC_AHB1ENR).
    // ------------------------------------------------------------------
    dp.RCC.ahb1enr().modify(|_, w| w.dma2en().set_bit());
    // Read-back fence: ensures the clock is enabled before configuring DMA.
    let _ = dp.RCC.ahb1enr().read().dma2en().bit_is_set();
    cortex_m::asm::dsb();

    // ------------------------------------------------------------------
    // 3. Ensure Stream0 is disabled before configuring.
    // ------------------------------------------------------------------
    dp.DMA2.st(0).cr().modify(|_, w| w.en().disabled());
    // Wait until EN reads back 0 (stream may still be finishing a previous xfer).
    while dp.DMA2.st(0).cr().read().en().is_enabled() {
        cortex_m::asm::nop();
    }

    // ------------------------------------------------------------------
    // 4. Clear all Stream0 interrupt flags in LIFCR before starting.
    // ------------------------------------------------------------------
    dp.DMA2.lifcr().write(|w| {
        w.ctcif0().set_bit()
         .chtif0().set_bit()
         .cteif0().set_bit()
         .cdmeif0().set_bit()
         .cfeif0().set_bit()
    });

    // ------------------------------------------------------------------
    // 5. Configure DMA2 Stream0 for M2M byte transfer.
    //
    //    PAR  = source address (the "peripheral" side in M2M)
    //    M0AR = destination address (the "memory" side)
    //    NDTR = transfer count
    //    CR   = dir, sizes, increments, priority
    //    FCR  = enable FIFO (required for M2M)
    // ------------------------------------------------------------------
    // SAFETY: writing u32 to a 32-bit MMIO register; addresses are valid
    // static buffers; DMA engine is disabled while we configure.
    unsafe {
        dp.DMA2.st(0).par().write(|w| w.pa().bits(src_addr as u32));
        dp.DMA2.st(0).m0ar().write(|w| w.m0a().bits(dst_addr as u32));
    }
    dp.DMA2.st(0).ndtr().write(|w| unsafe { w.ndt().bits(LEN as u16) });

    // FIFO mode: DMDIS=1 means "direct mode disabled" = FIFO enabled.
    // The FIFO is required for memory-to-memory transfers (RM0090 §10.3.11).
    dp.DMA2.st(0).fcr().write(|w| w.dmdis().disabled());

    // Stream CR: M2M direction, byte sizes, both addresses increment, low priority.
    dp.DMA2.st(0).cr().write(|w| {
        w.dir().memory_to_memory()
         .psize().bits8()
         .msize().bits8()
         .pinc().incremented()
         .minc().incremented()
         .pl().low()
         .circ().disabled()
         .tcie().disabled()
         .teie().disabled()
    });

    rprintln!("[DMA]  Stream0 configured: M2M {} bytes, FIFO enabled", LEN);
    rprintln!("[DMA]  CR=0x{:08X}  FCR=0x{:08X}  NDTR={}",
        dp.DMA2.st(0).cr().read().bits(),
        dp.DMA2.st(0).fcr().read().bits(),
        dp.DMA2.st(0).ndtr().read().bits());

    // ------------------------------------------------------------------
    // 6. Enable the stream — DMA starts immediately.
    // ------------------------------------------------------------------
    dp.DMA2.st(0).cr().modify(|_, w| w.en().enabled());
    rprintln!("[DMA]  Stream0 enabled — transfer running...");

    // ------------------------------------------------------------------
    // 7. Poll for transfer complete (TCIF0) or transfer error (TEIF0).
    // ------------------------------------------------------------------
    let mut completed = false;
    let mut errored   = false;
    for _ in 0..1_000_000u32 {
        let lisr = dp.DMA2.lisr().read();
        if lisr.teif0().bit_is_set() || lisr.dmeif0().bit_is_set() {
            errored = true;
            break;
        }
        if lisr.tcif0().is_complete() {
            completed = true;
            break;
        }
        cortex_m::asm::nop();
    }

    let lisr_final = dp.DMA2.lisr().read();
    rprintln!("[DMA]  Poll done: LISR=0x{:08X}  completed={} errored={}",
        lisr_final.bits(), completed, errored);

    // ------------------------------------------------------------------
    // 8. Verify destination matches source.
    // ------------------------------------------------------------------
    // SAFETY: DMA is done (completed flag), no further writes; raw pointer reads.
    let (ok, fail_idx, fail_got) = unsafe {
        let mut fail = None;
        for i in 0..LEN {
            let s = core::ptr::read_volatile(src_addr.add(i));
            let d = core::ptr::read_volatile(dst_addr.add(i));
            if s != d {
                fail = Some((i, d));
                break;
            }
        }
        match fail {
            None => (true, 0usize, 0u8),
            Some((i, v)) => (false, i, v),
        }
    };

    if completed && ok {
        rprintln!("✓ DMA2 M2M PASS: all {} bytes match (0x00..0x{:02X})", LEN, LEN - 1);
    } else if !completed {
        let reason = if errored { "DMA error (TEIF/DMEIF set)" } else { "timed out (TCIF never set)" };
        rprintln!("✗ DMA2 M2M FAIL: transfer did not complete — {}", reason);
    } else {
        rprintln!("✗ DMA2 M2M FAIL: byte[{}] expected 0x{:02X}, got 0x{:02X}",
            fail_idx, fail_idx as u8, fail_got);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
