//! UART→USB CDC Pipeline — STM32F429ZI NUCLEO-144
//!
//! Two-partition DMA pipeline: external data enters via UART DMA and exits
//! via USB CDC, passing through the kernel buffer pool without CPU copying.
//!
//! Data path:
//!   Host ttyACM0 → UART/DMA1 → kernel buf slot → sys_buf_read → USB CDC → Host ttyACM1
//!
//! Architecture:
//!   P0 (UART DMA, pid=0):
//!     alloc → lend(P1) → arm DMA → send "READY\n" → poll TCIF →
//!     set BUFFER_READY → yield-loop until BUFFER_DONE → revoke → release → CYCLE++
//!
//!   P1 (USB CDC, pid=1):
//!     tight poll loop: usb_dev.poll() + if BUFFER_READY →
//!       sys_buf_read → serial.write → BUFFER_DONE=true
//!
//! Signalling: AtomicBool BUFFER_READY (P0→P1) and BUFFER_DONE (P1→P0).
//! Both partitions have full SRAM in their MPU region; the atomics live in
//! static storage — accessible to both without a syscall.
//!
//! P1 never blocks on sys_event_wait; it polls USB continuously in its slot.
//! This ensures USB enumeration always succeeds regardless of P0 state.
//!
//! Hardware:
//!   UART: PD8 (TX) / PD9 (RX) via ST-LINK VCP → /dev/ttyACM0 at 115200 bps
//!   USB:  PA11 (DM) / PA12 (DP) → OTG FS connector CN12 → /dev/ttyACM1
//!
//! Build:  cd f429zi && cargo build --example uart_usb_pipeline \
//!             --features kernel-usb-mpu --no-default-features
//! Host:   python3 f429zi/uart_usb_pipeline_host.py \
//!             --uart /dev/ttyACM0 --usb /dev/ttyACMN --cycles 50
//!         (N depends on how many ST-LINK VCPs are connected; check `ls /dev/ttyACM*`)

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack4K,
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
    otg_fs::{UsbBus, USB},
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    serial::{
        config::DmaConfig as UartDmaConfig,
        Config as SerialConfig, Rx as SerialRx, Serial, Tx as SerialTx,
    },
};
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, USB_SYSCLK_HZ};

// ── Constants ──────────────────────────────────────────────────────────────────

const NUM_PARTITIONS: usize = 2;
// USB poll() allocates ~2352 bytes in debug builds; with outer frames P1 needs >2KB.
// AlignedStack4K (4KB per partition) keeps total kernel state ~12KB (< 16KB limit).
const STACK_WORDS: usize = 1024;

const BUF_LEN: usize = 32;

// Hardcoded slot 0 — with buffer_pool_regions=1 sys_buf_alloc always returns slot 0.
const BUFFER_SLOT: BufferSlotId = BufferSlotId::new(0);
const P1_PID: PartitionId = PartitionId::new(1);

// USART3 DR address (RM0090: USART3 base=0x4000_4800, DR offset=0x04).
const USART3_DR_ADDR: u32 = 0x4000_4804;
const USART3_BASE: u32    = 0x4000_4800;
const DMA1_BASE: u32      = 0x4002_6000;

// The pipeline firmware (USB + UART DMA + kernel) exceeds 256 KB (text~273 KB).
// The kernel uses mpu_region.size() for BOTH the flash code region and the SRAM
// data region, so we must size up to 512 KB.  Both 0x08000000 and 0x20000000 are
// 512 KB-aligned so validate_mpu_region passes.
// The 512 KB SRAM window covers all 256 KB SRAM; the 512 KB code window covers
// the full firmware image.
const MPU_REGION_SIZE: u32 = 512 * 1024;

// ── Statistics ─────────────────────────────────────────────────────────────────

static CYCLE:    AtomicU32 = AtomicU32::new(0);
static USB_SENT: AtomicU32 = AtomicU32::new(0);
static DMA_ERR:     AtomicU32 = AtomicU32::new(0);
static API_ERR:     AtomicU32 = AtomicU32::new(0);

// ── Inter-partition signals ────────────────────────────────────────────────────
//
// Both partitions have full SRAM in their mpu_region, so statics in SRAM are
// accessible to both without a kernel syscall. AtomicBool gives sequentially
// consistent visibility across the single-core context switches.
//
// Protocol per cycle:
//   P0: BUFFER_READY=false, BUFFER_DONE=false (reset at cycle start)
//   P0: DMA completes → BUFFER_READY=true
//   P1: sees BUFFER_READY → sys_buf_read → serial.write → BUFFER_DONE=true
//   P0: sees BUFFER_DONE → revoke → release → CYCLE++

static BUFFER_READY: AtomicBool = AtomicBool::new(false);
static BUFFER_DONE:  AtomicBool = AtomicBool::new(false);

// ── HAL object storage ────────────────────────────────────────────────────────
//
// Written by main() before boot(); each partition takes its objects on first call.
// No Mutex: UART objects are P0-only; USB objects are P1-only. No ISR races.

static mut TX_HANDLE:    Option<SerialTx<pac::USART3>> = None;
static mut RX_HANDLE:    Option<SerialRx<pac::USART3>> = None;
static mut DMA_STREAM:   Option<Stream1<pac::DMA1>>    = None;

static mut G_USB_DEVICE: Option<UsbDevice<'static, UsbBus<USB>>> = None;
static mut G_USB_SERIAL: Option<SerialPort<'static, UsbBus<USB>>> = None;

// ── Kernel configuration ───────────────────────────────────────────────────────

kernel::kernel_config!(PipelineCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = USB_SYSCLK_HZ;   // 168 MHz; require_pll48clk() derives 48 MHz for OTG
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;          // 1 slot: P0 allocs, lends to P1
    buffer_zone_size = 32;            // BZ: 32 bytes
});

// ── SysTick hook ──────────────────────────────────────────────────────────────

kernel::define_kernel!(PipelineCfg, |tick, _k| {
    if tick % 2_000 == 0 && tick > 0 {
        let cycle    = CYCLE.load(Ordering::Acquire);
        let usb_sent = USB_SENT.load(Ordering::Acquire);
        let dma_err  = DMA_ERR.load(Ordering::Acquire);
        let api_err  = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} USB_SENT={} DMA_ERR={} API_ERR={}",
            tick, cycle, usb_sent, dma_err, api_err
        );
        if cycle >= 20 && api_err == 0 {
            rprintln!("SUCCESS: UART→USB pipeline working! CYCLE={} USB_SENT={}", cycle, usb_sent);
        }
    }
});

// ── P0: UART DMA receiver ─────────────────────────────────────────────────────
//
// Owns USART3 + DMA1. Each cycle:
//   1. Alloc buffer slot from kernel pool.
//   2. Lend to P1 — get physical address for DMA M0AR.
//   3. Arm DMA stream (per-cycle: address + count only; static settings in one-time init).
//   4. Send "READY\n" via UART TX → host sends 32-byte payload.
//   5. Poll TCIF; on complete: set BUFFER_READY for P1.
//   6. Yield until P1 signals BUFFER_DONE.
//   7. Revoke P1's MPU grant; release slot.

extern "C" fn uart_dma_body(_r0: u32) -> ! {
    let tx     = unsafe { (*addr_of_mut!(TX_HANDLE)).as_mut().unwrap() };
    let rx     = unsafe { (*addr_of_mut!(RX_HANDLE)).as_mut().unwrap() };
    let stream = unsafe { (*addr_of_mut!(DMA_STREAM)).as_mut().unwrap() };

    // ------------------------------------------------------------------
    // One-time stream init: static settings that never change per cycle.
    // Channel, direction, PAR, data sizes, increments, FIFO mode.
    // ------------------------------------------------------------------
    unsafe { stream.disable() };
    while stream.is_enabled() { cortex_m::asm::nop(); }
    stream.clear_all_flags();
    stream.set_channel(DmaChannel::Channel4);           // USART3_RX = CH4 (RM0090 Table 28)
    stream.set_direction(DmaDirection::PeripheralToMemory);
    stream.set_peripheral_address(USART3_DR_ADDR);      // PAR = USART3_DR (fixed)
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

    rprintln!("[P0] DMA1 Stream1 initialised (CH4, P→M, byte, direct).");
    rprintln!("[P0] Waiting for UART host on /dev/ttyACM0 at 115200 bps...");

    loop {
        // Reset signals at the start of every cycle.
        BUFFER_READY.store(false, Ordering::Release);
        BUFFER_DONE.store(false, Ordering::Release);

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
        //    writable=false: P1 only reads via sys_buf_read (read-only MPU grant).
        let buf_ptr = match plib::sys_buf_lend(slot, P1_PID.as_raw() as u8, false) {
            Ok((ptr, _region_id)) => ptr,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                if let Err(e) = plib::sys_buf_release(slot) {
                    rprintln!("[P0] sys_buf_release failed: {:?}", e);
                }
                continue;
            }
        };

        // 3. Per-cycle update: only M0AR address and NDTR count change.
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

        // 5. Signal host: ready for 32-byte payload. TX and DMA-RX are independent.
        tx.bwrite_all(b"READY\n").unwrap();

        // 6. Poll for transfer complete (32 bytes @ 115200 bps ≈ 2.8 ms).
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
            // Revoke and release without notifying P1.
            if let Err(e) = plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8) {
                rprintln!("[P0] sys_buf_revoke failed: {:?}", e);
            }
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 7. Signal P1: DMA complete, buffer ready to read.
        BUFFER_READY.store(true, Ordering::Release);

        // 8. Yield until P1 finishes reading and signals BUFFER_DONE.
        //    With 2 partitions, sys_yield() triggers a real PendSV context switch to P1.
        //    This allows P1 to run, read the buffer, write USB, and set BUFFER_DONE.
        while !BUFFER_DONE.load(Ordering::Acquire) {
            plib::sys_yield().ok();
        }

        // 9. Revoke P1's MPU grant and return slot to pool.
        if plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ── P1: USB CDC forwarder ──────────────────────────────────────────────────────
//
// Owns OTG_FS. Polls USB continuously in a tight loop (same as usb_driver.rs).
// When BUFFER_READY is set by P0, reads via sys_buf_read and forwards via USB CDC.
//
// P1 must not block on sys_event_wait — USB enumeration requires continuous polling.
// The scheduler will preempt P1 via PendSV when its slot expires.

extern "C" fn usb_cdc_body(_r0: u32) -> ! {
    static mut USB_DEV: Option<UsbDevice<'static, UsbBus<USB>>> = None;
    static mut USB_SER: Option<SerialPort<'static, UsbBus<USB>>> = None;

    // SAFETY: G_USB_DEVICE/G_USB_SERIAL are set by main() before boot().
    // No ISR races — OTG_FS ISR is intentionally not unmasked (polling design).
    let (usb_dev, serial) = unsafe {
        let dev_ptr = addr_of_mut!(USB_DEV);
        let ser_ptr = addr_of_mut!(USB_SER);
        if (*dev_ptr).is_none() {
            *dev_ptr = (*addr_of_mut!(G_USB_DEVICE)).take();
        }
        if (*ser_ptr).is_none() {
            *ser_ptr = (*addr_of_mut!(G_USB_SERIAL)).take();
        }
        ((*dev_ptr).as_mut().unwrap(), (*ser_ptr).as_mut().unwrap())
    };

    loop {
        // Always poll USB to keep enumeration and control transfers alive.
        let _active = usb_dev.poll(&mut [serial]);

        // Discard any incoming data from the host on the USB CDC port.
        // (The pipeline is UART→USB, not USB→anything.)
        {
            let mut discard = [0u8; 64];
            serial.read(&mut discard).ok();
        }

        // Check if P0 has deposited a buffer for us to forward.
        if BUFFER_READY.load(Ordering::Acquire) {
            // Read from the lent slot via kernel API.
            let mut buf = [0u8; BUF_LEN];
            match plib::sys_buf_read(BUFFER_SLOT, &mut buf) {
                Ok(_) => {
                    // Write buffer to USB CDC. Try up to 64 times; each attempt
                    // polls usb_dev to advance the USB state machine and free
                    // endpoint buffers. 32 bytes fits in one FS bulk packet (64 B max).
                    let mut sent = 0usize;
                    for _ in 0..64u32 {
                        usb_dev.poll(&mut [serial]);
                        match serial.write(&buf[sent..]) {
                            Ok(n) if n > 0 => {
                                sent += n;
                                if sent >= BUF_LEN { break; }
                            }
                            _ => {}
                        }
                    }
                    USB_SENT.fetch_add(sent as u32, Ordering::Release);
                }
                Err(_) => {
                    // Kernel rejected read — lend may have been revoked before we ran.
                    API_ERR.fetch_add(1, Ordering::Release);
                }
            }

            // Signal P0: we are done with the buffer (revoke + release are P0's job).
            BUFFER_DONE.store(true, Ordering::Release);
        }

        // No sys_yield — partition runs its full slot then PendSV preempts.
        // Tight poll loop gives maximum USB responsiveness within the scheduler budget.
    }
}

kernel::partition_trampoline!(uart_dma_main => uart_dma_body);
kernel::partition_trampoline!(usb_cdc_main  => usb_cdc_body);

// ── main ───────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rprintln!("\n=== UART→USB CDC Pipeline — STM32F429ZI NUCLEO-144 ===");
    rprintln!("Path: ttyACM0/UART → DMA1 → kernel buf → sys_buf_read → USB CDC → ttyACM1");
    rprintln!("P0: UART DMA receiver  P1: USB CDC forwarder  MPU_ENFORCE=true");
    rprintln!("Host: python3 f429zi/uart_usb_pipeline_host.py --uart /dev/ttyACM0 --usb /dev/ttyACM1");

    let dp = pac::Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    // 168 MHz system clock + 48 MHz PLL48CLK for OTG FS.
    let mut rcc = dp.RCC.freeze(
        RccConfig::hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk(),
    );

    // ── UART (USART3, PD8/PD9, 115200 bps) ──
    let gpiod  = dp.GPIOD.split(&mut rcc);
    let pd8_tx = gpiod.pd8.into_alternate::<7>();
    let pd9_rx = gpiod.pd9.into_alternate::<7>();

    // .dma(DmaConfig::Rx) sets CR3.DMAR=1; cleared below; toggled per cycle by P0.
    let serial_uart = Serial::new(
        dp.USART3,
        (pd8_tx, pd9_rx),
        SerialConfig::default().baudrate(115_200.bps()).dma(UartDmaConfig::Rx),
        &mut rcc,
    ).expect("USART3 init");
    rprintln!("[INIT] USART3 115200 8N1 full-duplex PD8/PD9 (PCLK1=42 MHz).");

    let dma1  = StreamsTuple::new(dp.DMA1, &mut rcc);
    let (tx, rx) = serial_uart.split();

    unsafe {
        *addr_of_mut!(TX_HANDLE)  = Some(tx);
        *addr_of_mut!(RX_HANDLE)  = Some(rx);
        *addr_of_mut!(DMA_STREAM) = Some(dma1.1);
        // Clear DMAR set by Serial::new() to prevent warmup bytes triggering DMA.
        pac::Peripherals::steal().USART3.cr3().modify(|_, w| w.dmar().clear_bit());
    }

    // ── USB (OTG FS, PA11/PA12) ──
    let gpioa = dp.GPIOA.split(&mut rcc);
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    // SAFETY: USB_BUS is 'static; written once here before boot() and only read after.
    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    // Store USB objects in static mut — no Mutex needed, no ISR will race with P1.
    // OTG_FS ISR is intentionally not unmasked (polling design, same as usb_driver.rs).
    unsafe {
        G_USB_SERIAL = Some(SerialPort::new(usb_bus));
        G_USB_DEVICE = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(usbd_serial::USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("STM32")
                    .product("F429 UART→USB Pipeline")
                    .serial_number("PIPE001")])
                .unwrap()
                .build(),
        );
    }
    rprintln!("[INIT] USB initialized — VID:PID=16c0:27dd, product='F429 UART→USB Pipeline'");
    rprintln!("[INIT] OTG_FS ISR NOT unmasked — P1 polls USB directly in its slot.");

    // ── Kernel ──
    let mut sched = ScheduleTable::<{ PipelineCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");  // 4 ms: DMA poll + yield-loop
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");  // 4 ms: USB poll tight loop
    sched.add_system_window(1).expect("sched SW");           // 1 ms: RTT + stats

    // Allocate per-partition stacks externally for ExternalPartitionMemory.
    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let sentinel_mpu = MpuRegion::new(SRAM_BASE, MPU_REGION_SIZE, 0);

    // P0: USART3 (1KB) + DMA1 (1KB) peripheral access.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, uart_dma_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(USART3_BASE, 1024, 0),
            MpuRegion::new(DMA1_BASE,   1024, 0),
        ])
        .expect("periph0");

    // P1: OTG_FS register block (256 KB, naturally aligned).
    let otg_fs_base = pac::OTG_FS_GLOBAL::PTR as usize as u32;
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, usb_cdc_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
        .expect("code1")
        .with_peripheral_regions(&[MpuRegion::new(otg_fs_base, 256 * 1024, 0)])
        .expect("periph1");
    let mems = [mem0, mem1];

    let mut k = Kernel::<PipelineCfg>::new(sched, &mems).expect("kernel");
    rprintln!("[INIT] Kernel created. 2 partitions, 1 buffer slot × 32 bytes.");
    rprintln!("[INIT] P0: UART DMA (USART3+DMA1)  P1: USB CDC (OTG_FS)");
    rprintln!("[INIT] Schedule: P0=4ms P1=4ms SW=1ms (9ms frame, ~111Hz)");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}
