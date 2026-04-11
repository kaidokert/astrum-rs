//! UART DMA Zero-Copy Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates true zero-copy DMA from an external host (Raspberry Pi) into a
//! kernel buffer pool slot — no CPU involvement in the receive data path.
//!
//! Hardware path:
//!   RPi `/dev/ttyACM0` → ST-LINK VCP bridge → NUCLEO PD9 (USART3_RX) →
//!   DMA1 Stream1 CH4 → kernel buffer slot (buf_ptr from sys_buf_lend).
//!   MCU replies PD8 (USART3_TX) → ST-LINK VCP bridge → RPi.
//!   Full-duplex. No HDSEL. No extra wires — ST-LINK VCP is already connected.
//!
//! Protocol (one cycle):
//!   1. MCU arms DMA (memory address = buf_ptr, count = 32).
//!   2. MCU sends "READY\n" via TX — signals host to transmit.
//!   3. Host writes 32-byte pattern (0x00..0x1F) to serial port.
//!   4. DMA receives all 32 bytes into buf_ptr (no CPU reads of USART3_DR).
//!   5. MCU polls TCIF1; on complete: verify via sys_buf_read.
//!   6. MCU replies "OK\n" or "ERR:idx exp got\n" via TX.
//!   7. Revoke + release slot; CYCLE++.
//!
//! Key property: bytes 0x00-0x1F arrive from the network and land in
//! kernel-managed memory without the CPU ever touching the payload.
//! sys_buf_read is used only for post-transfer verification, not the
//! receive path itself.
//!
//! Build:  cd f429zi && cargo build --example uart_dma_demo \
//!             --features kernel-mpu-hal --release
//! Host:   python3 f429zi/uart_dma_host.py --port /dev/ttyACM0 --cycles 200

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
    partition_core::{AlignedStack2K, AlignedStack4K},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{BufferSlotId, PartitionId};
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    ClearFlags,
    dma::{
        traits::{Stream, StreamISR},
        DmaChannel, DmaDataSize, DmaDirection, Stream1, StreamsTuple,
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

// Expected receive pattern: host sends bytes 0x00..0x1F.
const EXPECTED: [u8; BUF_LEN] = {
    let mut a = [0u8; BUF_LEN];
    let mut i = 0usize;
    while i < BUF_LEN { a[i] = i as u8; i += 1; }
    a
};

const P1_PID: PartitionId = PartitionId::new(1);

// USART3 DR address (RM0090: USART3 base=0x4000_4800, DR offset=0x04).
const USART3_DR_ADDR: u32 = 0x4000_4804;

const USART3_BASE: u32 = 0x4000_4800;
const DMA1_BASE: u32   = 0x4002_6000;

static CYCLE:       AtomicU32 = AtomicU32::new(0);
static VERIFY_OK:   AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
static DMA_ERR:     AtomicU32 = AtomicU32::new(0);
static API_ERR:     AtomicU32 = AtomicU32::new(0);

// HAL object storage: written by main() before boot(), read only by P0.
static mut TX_HANDLE:  Option<SerialTx<pac::USART3>> = None;
static mut RX_HANDLE:  Option<SerialRx<pac::USART3>> = None;
static mut DMA_STREAM: Option<Stream1<pac::DMA1>>    = None;

kernel::kernel_config!(UartDmaCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
    buffer_zone_size = 32;
});

kernel::define_kernel!(UartDmaCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle       = CYCLE.load(Ordering::Acquire);
        let verify_ok   = VERIFY_OK.load(Ordering::Acquire);
        let verify_fail = VERIFY_FAIL.load(Ordering::Acquire);
        let dma_err     = DMA_ERR.load(Ordering::Acquire);
        let api_err     = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} VERIFY_OK={} VERIFY_FAIL={} DMA_ERR={} API_ERR={}",
            tick, cycle, verify_ok, verify_fail, dma_err, api_err
        );
        // DMA_ERR <= 2 tolerated: startup transients before host connects
        // cause RX timeouts (same pattern as uart_usb_pipeline).
        if cycle >= 20 && verify_fail == 0 && dma_err <= 2 && api_err == 0 {
            rprintln!("SUCCESS: real-wires UART DMA working! CYCLE={} DMA_ERR={}", cycle, dma_err);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — DMA receiver: arm DMA -> send READY -> host sends -> DMA fills buf ->
//      verify -> ACK -> repeat.
// ---------------------------------------------------------------------------
extern "C" fn producer_body(_r0: u32) -> ! {
    let tx     = unsafe { (*addr_of_mut!(TX_HANDLE)).as_mut().unwrap() };
    let rx     = unsafe { (*addr_of_mut!(RX_HANDLE)).as_mut().unwrap() };
    let stream = unsafe { (*addr_of_mut!(DMA_STREAM)).as_mut().unwrap() };

    // ------------------------------------------------------------------
    // One-time stream init: static settings that never change per cycle.
    // ------------------------------------------------------------------
    unsafe { stream.disable() };
    while stream.is_enabled() { cortex_m::asm::nop(); }
    stream.clear_all_flags();
    stream.set_channel(DmaChannel::Channel4);          // USART3_RX = CH4 (RM0090 Table 28)
    stream.set_direction(DmaDirection::PeripheralToMemory);
    stream.set_peripheral_address(USART3_DR_ADDR);     // PAR = USART3_DR (fixed)
    // SAFETY: u8 matches USART3 DR data width and kernel buffer element type.
    unsafe {
        stream.set_memory_size(DmaDataSize::Byte);
        stream.set_peripheral_size(DmaDataSize::Byte);
    }
    stream.set_memory_increment(true);
    stream.set_peripheral_increment(false);
    stream.set_circular_mode(false);
    // Direct mode: each RXNE immediately transfers 1 byte (no FIFO buffering).
    stream.set_fifo_enable(false);

    rprintln!("[P0] Stream initialised (CH4, P2M, byte, direct). Starting cycles.");
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

        // 2. Lend to P1 — get physical buffer address for DMA M0AR.
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

        // 3. Per-cycle update: only address and count change.
        unsafe { stream.disable() };
        while stream.is_enabled() { cortex_m::asm::nop(); }
        stream.clear_all_flags();
        stream.set_memory_address(buf_ptr as u32);
        stream.set_number_of_transfers(BUF_LEN as u16);

        // 4. Arm DMA BEFORE signalling host.
        //    Drain stale RXNE first: RXNE=1 when DMAR goes high latches an
        //    immediate DMA request that fires on enable, corrupting buf[0].
        rx.read().ok();
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().set_bit());
        }
        unsafe { stream.enable() };

        // 5. Signal host. TX and DMA-RX are independent (full-duplex).
        tx.bwrite_all(b"READY\n").unwrap();

        // 6. Poll for transfer complete (32 bytes @ 115200 bps ~= 2.8 ms).
        let mut completed = false;
        let mut errored   = false;
        for _ in 0..2_000_000u32 {
            if stream.is_transfer_error() || stream.is_direct_mode_error() {
                errored = true;
                break;
            }
            if stream.is_transfer_complete() {
                completed = true;
                break;
            }
            cortex_m::asm::nop();
        }

        // Cleanup: disable stream, clear flags, disable DMAR.
        unsafe { stream.disable() };
        stream.clear_all_flags();
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().clear_bit());
        }

        if !completed || errored {
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

        // 7. Revoke P1's MPU grant.
        if plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        // 8. Verify received data.
        let mut got = [0u8; BUF_LEN];
        match plib::sys_buf_read(BufferSlotId::new(0), &mut got) {
            Ok(_) => {
                let first_bad = got.iter().zip(EXPECTED.iter()).enumerate()
                    .find(|(_, (g, e))| g != e);
                match first_bad {
                    None => {
                        VERIFY_OK.fetch_add(1, Ordering::Release);
                        tx.bwrite_all(b"OK\n").unwrap();
                    }
                    Some((idx, (&g, &e))) => {
                        VERIFY_FAIL.fetch_add(1, Ordering::Release);
                        let mut reply = [0u8; 24];
                        let n = fmt_err(&mut reply, idx as u8, e, g);
                        tx.bwrite_all(&reply[..n]).unwrap();
                    }
                }
            }
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                tx.bwrite_all(b"APIERR\n").unwrap();
            }
        }

        // 9. Return slot to pool.
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

/// Write "ERR:idx exp got\n" into buf using ASCII decimal. Returns byte count.
fn fmt_err(buf: &mut [u8; 24], idx: u8, exp: u8, got: u8) -> usize {
    buf[..4].copy_from_slice(b"ERR:");
    let mut p = 4;
    p += fmt_u8(&mut buf[p..], idx);
    buf[p] = b' '; p += 1;
    p += fmt_u8(&mut buf[p..], exp);
    buf[p] = b' '; p += 1;
    p += fmt_u8(&mut buf[p..], got);
    buf[p] = b'\n'; p += 1;
    p
}

fn fmt_u8(buf: &mut [u8], v: u8) -> usize {
    if v >= 100 {
        buf[0] = b'0' + v / 100;
        buf[1] = b'0' + (v / 10) % 10;
        buf[2] = b'0' + v % 10;
        3
    } else if v >= 10 {
        buf[0] = b'0' + v / 10;
        buf[1] = b'0' + v % 10;
        2
    } else {
        buf[0] = b'0' + v;
        1
    }
}

// P1: dummy lendee — kernel needs a valid target PID for sys_buf_lend.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(producer_main => producer_body);
kernel::partition_trampoline!(dummy_main    => dummy_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== UART DMA Zero-Copy Demo === STM32F429ZI ===");
    rprintln!("Path: RPi /dev/ttyACM0 -> ST-LINK VCP -> PD9 -> DMA1 Stream1 -> kernel buffer.");
    rprintln!("Run:  python3 f429zi/uart_dma_host.py --port /dev/ttyACM0 --cycles 200");

    let dp = pac::Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(RccConfig::hse(8.MHz()).sysclk(168.MHz()));

    let gpiod  = dp.GPIOD.split(&mut rcc);
    let pd8_tx = gpiod.pd8.into_alternate::<7>();
    let pd9_rx = gpiod.pd9.into_alternate::<7>();

    // Full-duplex USART3 at 115200 bps. No HDSEL — PD9 is the real RX line.
    // .dma(DmaConfig::Rx) sets DMAR=1; cleared below and toggled per cycle.
    let serial = Serial::new(
        dp.USART3,
        (pd8_tx, pd9_rx),
        SerialConfig::default()
            .baudrate(115_200.bps())
            .dma(UartDmaConfig::Rx),
        &mut rcc,
    ).expect("USART3 init");

    rprintln!("[INIT] USART3 115200 8N1 full-duplex PD8/PD9 (168 MHz, PCLK1=42 MHz).");

    let dma1 = StreamsTuple::new(dp.DMA1, &mut rcc);

    let (tx, rx) = serial.split();
    unsafe {
        *addr_of_mut!(TX_HANDLE)  = Some(tx);
        *addr_of_mut!(RX_HANDLE)  = Some(rx);
        *addr_of_mut!(DMA_STREAM) = Some(dma1.1);
        // Clear DMAR set by Serial::new() to prevent warmup bytes triggering DMA.
        pac::Peripherals::steal().USART3.cr3().modify(|_, w| w.dmar().clear_bit());
    }

    let mut sched = ScheduleTable::<{ UartDmaCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    // Allocate per-partition stacks externally for ExternalPartitionMemory.
    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let sentinel_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, producer_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(0)
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

    let mut k = Kernel::<UartDmaCfg>::new(sched, &mems).expect("kernel");
    rprintln!("[INIT] Kernel created. Buffer pool: 1 slot x 32 bytes. Booting...\n");
    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
