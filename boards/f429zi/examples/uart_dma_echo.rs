//! UART Full-Duplex DMA Echo — STM32F429ZI NUCLEO-144
//!
//! True zero-copy bidirectional DMA: the same kernel buffer slot is the
//! DMA destination for the RX transfer AND the DMA source for the TX
//! transfer. The CPU never touches the payload bytes in either direction.
//!
//! Hardware path:
//!   RPi → /dev/ttyACM0 → ST-LINK VCP → PD9 → DMA1 Stream1 CH4 (P→M) → kernel buf
//!   kernel buf → DMA1 Stream3 CH4 (M→P) → PD8 → ST-LINK VCP → /dev/ttyACM0 → RPi
//!
//! Protocol (one cycle):
//!   1. MCU sends "READY\n" (CPU TX) — signals host to transmit.
//!   2. Host sends 32-byte pattern (0x00..0x1F).
//!   3. DMA1 Stream1 receives 32 bytes into buf_ptr (no CPU reads of USART3_DR).
//!   4. DMA1 Stream3 transmits 32 bytes from buf_ptr (no CPU writes to USART3_DR).
//!   5. Host reads back 32 bytes; verifies byte-for-byte match.
//!   6. MCU sends "OK\n" (CPU TX) — cycle complete.
//!
//! Key property: bytes 0x00..0x1F arrive from the network, land in kernel-managed
//! memory, and leave back to the network — without the CPU ever reading or writing
//! the payload. buf_ptr is the sole DMA memory address for both transfers.
//!
//! Build:  cd f429zi && cargo build --example uart_dma_echo \
//!             --features kernel-mpu-hal --no-default-features --release
//! Host:   python3 f429zi/uart_dma_echo_host.py --port /dev/ttyACM0 --cycles 200

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack4K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::PartitionId;
use rtt_target::rprintln;
use stm32f4xx_hal::{
    ClearFlags,
    dma::{
        traits::{Stream, StreamISR},
        DmaChannel, DmaDataSize, DmaDirection, Stream1, Stream3, StreamsTuple,
    },
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    serial::{
        config::DmaConfig as UartDmaConfig,
        Config as SerialConfig, Rx as SerialRx, Serial, Tx as SerialTx,
    },
};
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 1024;
const BUF_LEN: usize = 32;

// P1 exists solely as a valid target PID for sys_buf_lend.
const P1_PID: PartitionId = PartitionId::new(1);

const USART3_DR_ADDR: u32 = 0x4000_4804;
const USART3_BASE: u32    = 0x4000_4800;
const DMA1_BASE: u32      = 0x4002_6000;

static CYCLE:    AtomicU32 = AtomicU32::new(0);
static DMA_ERR:  AtomicU32 = AtomicU32::new(0);
static API_ERR:  AtomicU32 = AtomicU32::new(0);

// Written by main() before boot(); read only by P0.
static mut TX_HANDLE:     Option<SerialTx<pac::USART3>>  = None;
static mut RX_HANDLE:     Option<SerialRx<pac::USART3>>  = None;
static mut DMA_RX_STREAM: Option<Stream1<pac::DMA1>>     = None;
static mut DMA_TX_STREAM: Option<Stream3<pac::DMA1>>     = None;

kernel::kernel_config!(EchoCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
    buffer_zone_size = 32;
});

kernel::define_kernel!(EchoCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle   = CYCLE.load(Ordering::Acquire);
        let dma_err = DMA_ERR.load(Ordering::Acquire);
        let api_err = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} DMA_ERR={} API_ERR={}",
            tick, cycle, dma_err, api_err
        );
        // DMA_ERR <= 2 tolerated: startup transients before host connects
        // cause RX timeouts (same pattern as uart_usb_pipeline).
        if cycle >= 20 && dma_err <= 2 && api_err == 0 {
            rprintln!("SUCCESS: full-duplex UART DMA echo working! CYCLE={} DMA_ERR={}", cycle, dma_err);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — full-duplex DMA echo:
//   arm RX DMA → "READY\n" → host sends 32 bytes → RX DMA fills buf_ptr →
//   arm TX DMA from same buf_ptr → 32 bytes echo back to host → "OK\n"
// ---------------------------------------------------------------------------
extern "C" fn echo_body(_r0: u32) -> ! {
    let tx        = unsafe { (*addr_of_mut!(TX_HANDLE)).as_mut().unwrap() };
    let rx        = unsafe { (*addr_of_mut!(RX_HANDLE)).as_mut().unwrap() };
    let stream_rx = unsafe { (*addr_of_mut!(DMA_RX_STREAM)).as_mut().unwrap() };
    let stream_tx = unsafe { (*addr_of_mut!(DMA_TX_STREAM)).as_mut().unwrap() };

    // ------------------------------------------------------------------
    // One-time RX stream init: DMA1 Stream1 CH4 — USART3_RX → memory.
    // ------------------------------------------------------------------
    unsafe { stream_rx.disable() };
    while stream_rx.is_enabled() { cortex_m::asm::nop(); }
    stream_rx.clear_all_flags();
    stream_rx.set_channel(DmaChannel::Channel4);          // USART3_RX = CH4
    stream_rx.set_direction(DmaDirection::PeripheralToMemory);
    stream_rx.set_peripheral_address(USART3_DR_ADDR);
    unsafe {
        stream_rx.set_memory_size(DmaDataSize::Byte);
        stream_rx.set_peripheral_size(DmaDataSize::Byte);
    }
    stream_rx.set_memory_increment(true);
    stream_rx.set_peripheral_increment(false);
    stream_rx.set_circular_mode(false);
    stream_rx.set_fifo_enable(false);                     // direct mode: 1 RXNE = 1 byte

    // ------------------------------------------------------------------
    // One-time TX stream init: DMA1 Stream3 CH4 — memory → USART3_TX.
    // RM0090 Table 28: DMA1 Stream3 Channel4 = USART3_TX.
    // ------------------------------------------------------------------
    unsafe { stream_tx.disable() };
    while stream_tx.is_enabled() { cortex_m::asm::nop(); }
    stream_tx.clear_all_flags();
    stream_tx.set_channel(DmaChannel::Channel4);          // USART3_TX = CH4
    stream_tx.set_direction(DmaDirection::MemoryToPeripheral);
    stream_tx.set_peripheral_address(USART3_DR_ADDR);
    unsafe {
        stream_tx.set_memory_size(DmaDataSize::Byte);
        stream_tx.set_peripheral_size(DmaDataSize::Byte);
    }
    stream_tx.set_memory_increment(true);
    stream_tx.set_peripheral_increment(false);
    stream_tx.set_circular_mode(false);
    stream_tx.set_fifo_enable(false);                     // direct mode: TXE triggers transfer

    rprintln!("[P0] RX: Stream1 CH4 P→M. TX: Stream3 CH4 M→P. Both direct mode.");
    rprintln!("[P0] Waiting for host on /dev/ttyACM0 at 115200 bps...");

    loop {
        // 1. Allocate buffer slot.
        let slot = match plib::sys_buf_alloc(true, 0) {
            Ok(s) => s,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                plib::sys_yield().ok();
                continue;
            }
        };

        // 2. Lend to P1 — get physical address for both DMA streams.
        //    writable=true: DMA RX writes into the slot; DMA TX reads from it.
        let buf_ptr = match plib::sys_buf_lend(slot, P1_PID.as_raw() as u8, true) {
            Ok((ptr, _region_id)) => ptr,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                if let Err(e) = plib::sys_buf_release(slot) {
                    rprintln!("[P0] sys_buf_release failed: {:?}", e);
                }
                continue;
            }
        };

        // 3. Per-cycle RX setup: only address and count change.
        unsafe { stream_rx.disable() };
        while stream_rx.is_enabled() { cortex_m::asm::nop(); }
        stream_rx.clear_all_flags();
        stream_rx.set_memory_address(buf_ptr as u32);
        stream_rx.set_number_of_transfers(BUF_LEN as u16);

        // 4. Arm RX DMA. Drain stale RXNE first: prevents first-byte corruption.
        rx.read().ok();
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().set_bit());
        }
        unsafe { stream_rx.enable() };

        // 5. Signal host (CPU TX; independent of DMA RX path).
        tx.bwrite_all(b"READY\n").unwrap();

        // 6. Poll RX transfer complete (32 bytes @ 115200 ≈ 2.8 ms).
        let rx_ok = poll_tcif(stream_rx);

        // Disarm RX DMA.
        unsafe { stream_rx.disable() };
        stream_rx.clear_all_flags();
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().clear_bit());
        }

        if !rx_ok {
            DMA_ERR.fetch_add(1, Ordering::Release);
            tx.bwrite_all(b"TIMEOUT\n").unwrap();
            if let Err(e) = plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8) {
                rprintln!("[P0] sys_buf_revoke failed: {:?}", e);
            }
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 7. Per-cycle TX setup: same buf_ptr, same count.
        //    The kernel buffer now holds the received bytes; DMA TX reads from it.
        unsafe { stream_tx.disable() };
        while stream_tx.is_enabled() { cortex_m::asm::nop(); }
        stream_tx.clear_all_flags();
        stream_tx.set_memory_address(buf_ptr as u32);
        stream_tx.set_number_of_transfers(BUF_LEN as u16);

        // 8. Arm TX DMA. Wait for TXE before enabling DMAT to avoid a spurious
        //    initial DMA request while the shift register is still busy.
        unsafe {
            let usart = pac::Peripherals::steal().USART3;
            // Drain any residual TX state (TXE=1 means DR is empty and ready).
            while usart.sr().read().txe().bit_is_clear() { cortex_m::asm::nop(); }
            usart.cr3().modify(|_, w| w.dmat().set_bit());
        }
        unsafe { stream_tx.enable() };

        // 9. Poll TX transfer complete (32 bytes @ 115200 ≈ 2.8 ms).
        let tx_ok = poll_tcif(stream_tx);

        // Disarm TX DMA, wait for USART shift register to drain.
        unsafe { stream_tx.disable() };
        stream_tx.clear_all_flags();
        unsafe {
            let usart = pac::Peripherals::steal().USART3;
            // TC=1: last bit has been shifted out — safe to disable DMAT.
            while usart.sr().read().tc().bit_is_clear() { cortex_m::asm::nop(); }
            usart.cr3().modify(|_, w| w.dmat().clear_bit());
        }

        // Revoke P1's MPU grant.
        if plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        if !tx_ok {
            DMA_ERR.fetch_add(1, Ordering::Release);
            continue;
        }

        // 10. Notify host that the echo cycle is complete.
        tx.bwrite_all(b"OK\n").unwrap();
        CYCLE.fetch_add(1, Ordering::Release);
    }
}

/// Poll TCIF for up to 2M iterations; return true on transfer complete.
/// Treats DMA transfer error or direct-mode error as failure.
#[inline(never)]
fn poll_tcif<S: Stream + StreamISR>(s: &S) -> bool {
    for _ in 0..2_000_000u32 {
        if s.is_transfer_error() || s.is_direct_mode_error() {
            return false;
        }
        if s.is_transfer_complete() {
            return true;
        }
        cortex_m::asm::nop();
    }
    false
}

// P1: dummy lendee — kernel needs a valid target PID for sys_buf_lend.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(echo_main  => echo_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== UART Full-Duplex DMA Echo === STM32F429ZI ===");
    rprintln!("Path: RPi -> Stream1 CH4 (P2M) -> kernel buf -> Stream3 CH4 (M2P) -> RPi");
    rprintln!("Run:  python3 f429zi/uart_dma_echo_host.py --port /dev/ttyACM0 --cycles 200");

    let dp = pac::Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(RccConfig::hse(8.MHz()).sysclk(168.MHz()));

    let gpiod  = dp.GPIOD.split(&mut rcc);
    let pd8_tx = gpiod.pd8.into_alternate::<7>();
    let pd9_rx = gpiod.pd9.into_alternate::<7>();

    // Full-duplex USART3 at 115200 bps. TxRx mode enables both DMAR and DMAT
    // in CR3; we clear both below and toggle them per-operation.
    let serial = Serial::new(
        dp.USART3,
        (pd8_tx, pd9_rx),
        SerialConfig::default()
            .baudrate(115_200.bps())
            .dma(UartDmaConfig::TxRx),
        &mut rcc,
    ).expect("USART3 init");

    rprintln!("[INIT] USART3 115200 8N1 full-duplex PD8/PD9.");

    let dma1 = StreamsTuple::new(dp.DMA1, &mut rcc);
    let (tx, rx) = serial.split();

    unsafe {
        *addr_of_mut!(TX_HANDLE)     = Some(tx);
        *addr_of_mut!(RX_HANDLE)     = Some(rx);
        *addr_of_mut!(DMA_RX_STREAM) = Some(dma1.1);  // Stream1 → RX
        *addr_of_mut!(DMA_TX_STREAM) = Some(dma1.3);  // Stream3 → TX
        // Clear DMAR + DMAT set by Serial::new(TxRx) before partition takes over.
        pac::Peripherals::steal()
            .USART3
            .cr3()
            .modify(|_, w| w.dmar().clear_bit().dmat().clear_bit());
    }

    let mut sched = ScheduleTable::<{ EchoCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    // Allocate per-partition stacks externally for ExternalPartitionMemory.
    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let sentinel_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, echo_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
            .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
            .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(USART3_BASE, 1024, 0),
            MpuRegion::new(DMA1_BASE,   1024, 0),
        ])
        .expect("periph0");
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
            .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
            .expect("code1");
    let mems = [mem0, mem1];

    let mut k = Kernel::<EchoCfg>::new(sched, &mems).expect("kernel");
    rprintln!("[INIT] Kernel ready. P0=echo P1=dummy. Buffer pool: 1 slot × 32 bytes.");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled IRQ {}", irqn);
}
