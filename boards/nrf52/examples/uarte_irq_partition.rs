//! UARTE IRQ Partition Demo — nRF52833 (Approach D + Model B IRQ)
//!
//! Proves Approach D (partition owns peripheral under MPU) works on Nordic
//! silicon with EasyDMA.  Full-duplex TX+RX verified via J-Link VCOM echo.
//!
//! Architecture:
//! - UARTE0 configured for TX on P0.06 / RX on P0.08 at 115200 baud
//! - `bind_interrupts!(UarteCfg, 48, 2 => (0, UARTE_EVENT))` wires
//!   the UARTE0 IRQ (IRQ 2) through the kernel dispatch table.
//!   The patched `__irq_dispatch` MASKS IRQ 2 before signalling P0.
//! - P0 holds UARTE0 (0x4000_2000 / 4 KB) in peripheral_regions
//! - P0 loop:
//!     1. start RX DMA (4 bytes)
//!     2. start TX DMA (4 bytes with sequence number)
//!     3. SYS_EVT_WAIT → ENDTX fires → clear + ACK
//!     4. SYS_EVT_WAIT → ENDRX fires → clear + ACK
//!     5. verify rx_buf == tx_buf, count cycle
//!     6. stop RX, repeat
//!
//! Host protocol (hw-test runner):
//!   Host reads 4 bytes from J-Link VCOM, echoes them back.
//!   Firmware verifies the echo matches. 20+ verified cycles = SUCCESS.
//!
//! Hardware: nRF52833-DK (PCA10100).
//!   P0.06 (TX) and P0.08 (RX) are wired to J-Link VCOM on PCA10100.
//!   UARTE0 base: 0x4000_2000 (4 KB).  EasyDMA can only access RAM.
//!
//! Build: cd nrf52 && cargo build --example uarte_irq_partition --features kernel-irq

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use nrf52833_pac as pac;
use rtt_target::rprintln;

const NUM_PARTITIONS: usize = 1;
const BUF_LEN: usize = 4;

/// UARTE0 IRQ number on nRF52833 (UARTE0_UART0 = 2).
const UARTE0_IRQ: u8 = 2;

/// Single kernel event bit for all UARTE0 events (ENDTX + ENDRX share IRQ 2).
const UARTE_EVENT: u32 = 0x0000_0001;

/// UARTE0 base address and size for MPU peripheral grant.
const UARTE0_BASE: u32 = 0x4000_2000;
const UARTE0_SIZE: u32 = 4096;

/// nRF52833 has 48 peripheral IRQs.
const IRQ_COUNT: usize = 48;

static TX_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static VERIFY_OK: AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
// Diagnostics: last received data + rxd.amount
static DIAG_RX_AMOUNT: AtomicU32 = AtomicU32::new(0xFFFF);
static DIAG_RX_WORD: AtomicU32 = AtomicU32::new(0);
static DIAG_TX_WORD: AtomicU32 = AtomicU32::new(0);

kernel::bind_interrupts!(UarteCfg, IRQ_COUNT,
    UARTE0_IRQ => (0, UARTE_EVENT),
);

kernel::kernel_config!(UarteCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(UarteCfg, |tick, _k| {
    if tick % 500 == 0 {
        let tx = TX_COUNT.load(Ordering::Acquire);
        let rx = RX_COUNT.load(Ordering::Acquire);
        let ok = VERIFY_OK.load(Ordering::Acquire);
        let fail = VERIFY_FAIL.load(Ordering::Acquire);
        let amt = DIAG_RX_AMOUNT.load(Ordering::Acquire);
        let rxw = DIAG_RX_WORD.load(Ordering::Acquire);
        let txw = DIAG_TX_WORD.load(Ordering::Acquire);
        rprintln!("[{:5}ms] TX={} RX={} OK={} FAIL={} amt={} tx={:08x} rx={:08x}",
            tick, tx, rx, ok, fail, amt, txw, rxw);
        if ok > 20 && fail == 0 {
            rprintln!("SUCCESS: UARTE IRQ partition echo verified! TX={} RX={} OK={}", tx, rx, ok);
        }
    }
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU CTRL=0x{:x} ---", ctrl);
        for r in 0u32..8 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            if rasr & 1 != 0 {
                rprintln!("  R{}: RBAR=0x{:08x} RASR=0x{:08x}", r, rbar, rasr);
            }
        }
        rprintln!("---");
    }
});

// ---------------------------------------------------------------------------
// Partition P0: UARTE driver (Approach D + IRQ Model B).
//
// Runs under MPU enforcement — only UARTE0 (0x4000_2000/4KB) is in
// peripheral_regions.  NVIC is NOT grantable (PPB), so IRQ unmask
// goes through SYS_IRQ_ACK.
//
// Protocol: TX 4 bytes → host echoes → RX 4 bytes → verify match.
// Both ENDTX and ENDRX share IRQ 2; partition checks which event fired.
// ---------------------------------------------------------------------------
extern "C" fn uarte_driver_body(irq_num: u32) -> ! {
    let dp = unsafe { pac::Peripherals::steal() };

    // Only ENDRX interrupt — we don't need ENDTX because we wait for the
    // full round-trip (TX on wire → host echo → RX DMA complete).
    // Using ENDTX would fire when DMA reads the last byte from RAM, NOT
    // when the byte finishes shifting out of the UART — too early.
    dp.UARTE0.intenset.write(|w| w.endrx().set());

    let mut tx_buf: [u8; BUF_LEN] = [0xA0, 0xB1, 0xC2, 0xD3];
    let mut rx_buf: [u8; BUF_LEN] = [0; BUF_LEN];
    let mut seq: u8 = 0;

    loop {
        // Stamp sequence number.
        tx_buf[0] = seq;
        seq = seq.wrapping_add(1);

        // Clear stale events.
        dp.UARTE0.events_endrx.write(|w| unsafe { w.bits(0) });
        dp.UARTE0.events_endtx.write(|w| unsafe { w.bits(0) });

        // Start RX DMA first (ready to receive echo before TX bytes arrive).
        rx_buf = [0; BUF_LEN];
        dp.UARTE0.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx_buf.as_mut_ptr() as u32) });
        dp.UARTE0.rxd.maxcnt.write(|w| unsafe { w.maxcnt().bits(BUF_LEN as u16) });
        dp.UARTE0.tasks_startrx.write(|w| unsafe { w.bits(1) });

        // Start TX DMA — bytes go to UART shift register, then on the wire.
        dp.UARTE0.txd.ptr.write(|w| unsafe { w.ptr().bits(tx_buf.as_ptr() as u32) });
        dp.UARTE0.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(BUF_LEN as u16) });
        dp.UARTE0.tasks_starttx.write(|w| unsafe { w.bits(1) });
        TX_COUNT.fetch_add(1, Ordering::Release);

        // Wait for ENDRX — fires after host echoes 4 bytes and RX DMA
        // receives all of them (rxd.amount == rxd.maxcnt).
        if plib::sys_event_wait(UARTE_EVENT.into()).is_err() {
            plib::sys_yield().ok();
            continue;
        }
        dp.UARTE0.events_endrx.write(|w| unsafe { w.bits(0) });
        RX_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_irq_ack(irq_num as u8).ok();

        // Compiler fence: DMA wrote to rx_buf behind the compiler's back.
        compiler_fence(Ordering::Acquire);

        // Diagnostics: record rxd.amount and first 4 bytes of both buffers.
        let amount = dp.UARTE0.rxd.amount.read().amount().bits();
        DIAG_RX_AMOUNT.store(amount as u32, Ordering::Release);
        let received = unsafe { core::ptr::read_volatile(&rx_buf as *const [u8; BUF_LEN]) };
        DIAG_RX_WORD.store(u32::from_le_bytes(received), Ordering::Release);
        DIAG_TX_WORD.store(u32::from_le_bytes(tx_buf), Ordering::Release);

        // Verify echo matches.
        if received == tx_buf {
            VERIFY_OK.fetch_add(1, Ordering::Release);
        } else {
            VERIFY_FAIL.fetch_add(1, Ordering::Release);
        }
    }
}

kernel::partition_trampoline!(uarte_driver => uarte_driver_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== UARTE IRQ Partition Demo — nRF52833 (Approach D + Model B IRQ) ===");
    rprintln!("UARTE0 EasyDMA TX+RX on P0.06/P0.08, ENDTX+ENDRX IRQ, host echo verify");

    let dp = pac::Peripherals::take().unwrap();

    // Configure UARTE0 pins.
    // nRF52 pin mux is done via UARTE PSEL registers (no GPIO AF config needed).
    // PCA10100 (nRF52833-DK) routes VCOM through P0.06 (TX) and P0.08 (RX)
    // via solder bridges SB40/SB41 (closed by default).
    dp.UARTE0.psel.txd.write(|w| unsafe { w.pin().bits(6).port().bit(false).connect().connected() });
    dp.UARTE0.psel.rxd.write(|w| unsafe { w.pin().bits(8).port().bit(false).connect().connected() });
    // Disconnect hardware flow control.
    dp.UARTE0.psel.cts.write(|w| w.connect().disconnected());
    dp.UARTE0.psel.rts.write(|w| w.connect().disconnected());

    // Baudrate: 115200.
    dp.UARTE0.baudrate.write(|w| w.baudrate().baud115200());

    // Config: 8N1, no HW flow control.
    dp.UARTE0.config.write(|w| w.parity().excluded().hwfc().disabled().stop().one());

    // Enable UARTE0.
    dp.UARTE0.enable.write(|w| w.enable().enabled());

    rprintln!("[INIT] UARTE0: P0.06 TX / P0.08 RX, 115200/8N1, EasyDMA");

    // Kernel setup.
    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ UarteCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0] = *stacks;

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, uarte_driver as kernel::PartitionEntry,
            MpuRegion::new(nrf52::SRAM_BASE, nrf52::SRAM_SIZE, 0), kernel::PartitionId::new(0),
        )
        .expect("mem0")
        .with_code_mpu_region(MpuRegion::new(nrf52::FLASH_BASE, 512 * 1024, 0))
        .expect("code0")
        .with_peripheral_regions(&[MpuRegion::new(UARTE0_BASE, UARTE0_SIZE, 0)])
        .expect("periph0")
        .with_r0_hint(UARTE0_IRQ as u32);

    let mems = [mem0];
    let mut k = Kernel::<UarteCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    // Unmask IRQ 2 (UARTE0) AFTER store_kernel.
    enable_bound_irqs(&mut p.NVIC, UarteCfg::IRQ_DEFAULT_PRIORITY).unwrap();
    rprintln!("[INIT] IRQ 2 (UARTE0) enabled — kernel ready");

    rprintln!("[INIT] Booting with MPU enforcement (deny-all + UARTE0 grant)\n");
    match boot(p).expect("boot") {}
}
