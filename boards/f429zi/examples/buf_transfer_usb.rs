//! Buffer Transfer → USB CDC Pipeline — STM32F429ZI NUCLEO-144
//!
//! Demonstrates `sys_buf_transfer` (permanent ownership handoff) in a real
//! I/O pipeline: UART DMA data enters via P0 and exits via USB CDC on P1.
//!
//! Data path:
//!   Host ttyACM0 → USART3/DMA1 → kernel buf slot → transfer(P1) →
//!   sys_buf_read → USB CDC → Host ttyACMN
//!
//! Architecture:
//!   P0 (UART DMA receiver, pid=0):
//!     alloc → arm DMA into local buf → poll TCIF → sys_buf_write(slot, buf) →
//!     transfer(P1) → set BUFFER_READY → event_wait(DONE) → CYCLE++
//!
//!   P1 (USB CDC sender, pid=1):
//!     tight USB poll loop → if BUFFER_READY →
//!     sys_buf_read → serial.write → sys_buf_release (P1 owns!) →
//!     event_set(P0, DONE)
//!
//! Key difference from uart_usb_pipeline (lend/revoke):
//!   - P0 transfers ownership permanently — no revoke, no release by P0.
//!   - P1 (the new owner) calls sys_buf_release to return the slot to the pool.
//!   - After transfer, P0 cannot touch the slot — any access would fail.
//!   - DMA goes into a stack-local buffer; sys_buf_write copies 32 bytes into the
//!     kernel buffer slot before transfer. The copy is negligible (32B) and avoids
//!     the self-lend restriction (sys_buf_lend rejects owner == target).
//!
//! Hardware:
//!   UART: PD8 (TX) / PD9 (RX) via ST-LINK VCP → /dev/ttyACM0 at 115200 bps
//!   USB:  PA11 (DM) / PA12 (DP) → OTG FS connector CN12 → /dev/ttyACMN
//!
//! Build:  cd f429zi && cargo build --example buf_transfer_usb \
//!             --features kernel-usb-mpu --no-default-features
//! Host:   python3 f429zi/uart_usb_pipeline_host.py \
//!             --uart /dev/ttyACM0 --usb /dev/ttyACMN --cycles 50

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
use plib::{BufferSlotId, EventMask, PartitionId};
use rtt_target::rprintln;
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
const STACK_WORDS: usize = 1024;
const BUF_LEN: usize = 32;

const BUFFER_SLOT: BufferSlotId = BufferSlotId::new(0);
const P0_PID: PartitionId = PartitionId::new(0);
const P1_PID: PartitionId = PartitionId::new(1);

// P1→P0 signal via kernel event (P0 can block on sys_event_wait).
const BUF_DONE: EventMask = EventMask::new(0x2);

// P0→P1 signal via AtomicBool (P1 must not block — needs continuous USB polling).
static BUFFER_READY: AtomicBool = AtomicBool::new(false);

// USART3 addresses.
const USART3_DR_ADDR: u32 = 0x4000_4804;
const USART3_BASE: u32    = 0x4000_4800;
const DMA1_BASE: u32      = 0x4002_6000;

// MPU region covering flash + SRAM (pipeline firmware exceeds 256KB).
const MPU_REGION_SIZE: u32 = 512 * 1024;

// ── Statistics ─────────────────────────────────────────────────────────────────

static CYCLE:    AtomicU32 = AtomicU32::new(0);
static USB_SENT: AtomicU32 = AtomicU32::new(0);
static DMA_ERR:  AtomicU32 = AtomicU32::new(0);
static API_ERR:  AtomicU32 = AtomicU32::new(0);

// ── HAL object storage ─────────────────────────────────────────────────────────

static mut TX_HANDLE:    Option<SerialTx<pac::USART3>> = None;
static mut RX_HANDLE:    Option<SerialRx<pac::USART3>> = None;
static mut DMA_STREAM:   Option<Stream1<pac::DMA1>>    = None;

static mut G_USB_DEVICE: Option<UsbDevice<'static, UsbBus<USB>>> = None;
static mut G_USB_SERIAL: Option<SerialPort<'static, UsbBus<USB>>> = None;

// ── Kernel configuration ───────────────────────────────────────────────────────

kernel::kernel_config!(XferCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
    buffer_zone_size = 32;
});

// ── SysTick hook ───────────────────────────────────────────────────────────────

kernel::define_kernel!(XferCfg, |tick, _k| {
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
            rprintln!("SUCCESS: UART DMA→transfer→USB pipeline! CYCLE={} USB_SENT={}", cycle, usb_sent);
        }
    }
});

// ── P0: UART DMA receiver ──────────────────────────────────────────────────────
//
// Owns USART3 + DMA1. Each cycle:
//   1. Arm DMA into stack-local buffer; send "READY\n" → host sends 32-byte payload.
//   2. Poll TCIF; on complete: alloc kernel buffer slot.
//   3. sys_buf_write(slot, &local_buf) — copy 32 bytes into kernel buffer.
//   4. Transfer ownership to P1 — permanent handoff.
//   5. Signal P1 via AtomicBool; wait for DONE event.

extern "C" fn uart_dma_body(_r0: u32) -> ! {
    let tx     = unsafe { (*addr_of_mut!(TX_HANDLE)).as_mut().unwrap() };
    let rx     = unsafe { (*addr_of_mut!(RX_HANDLE)).as_mut().unwrap() };
    let stream = unsafe { (*addr_of_mut!(DMA_STREAM)).as_mut().unwrap() };

    // Stack-local DMA receive buffer (written by DMA hardware, not Rust).
    #[allow(unused_mut)]
    let mut dma_buf = [0u8; BUF_LEN];

    // One-time DMA stream init: static settings that never change per cycle.
    unsafe { stream.disable() };
    while stream.is_enabled() { cortex_m::asm::nop(); }
    stream.clear_all_flags();
    stream.set_channel(DmaChannel::Channel4);
    stream.set_direction(DmaDirection::PeripheralToMemory);
    stream.set_peripheral_address(USART3_DR_ADDR);
    unsafe {
        stream.set_memory_size(DmaDataSize::Byte);
        stream.set_peripheral_size(DmaDataSize::Byte);
    }
    stream.set_memory_increment(true);
    stream.set_peripheral_increment(false);
    stream.set_circular_mode(false);
    stream.set_fifo_enable(false);

    rprintln!("[P0] DMA1 Stream1 initialised (CH4, P→M, byte, direct).");
    rprintln!("[P0] Waiting for UART host on /dev/ttyACM0 at 115200 bps...");

    loop {
        BUFFER_READY.store(false, Ordering::Release);

        // 1. Per-cycle DMA setup: target is stack-local buffer.
        unsafe { stream.disable() };
        while stream.is_enabled() { cortex_m::asm::nop(); }
        stream.clear_all_flags();
        stream.set_memory_address(dma_buf.as_ptr() as u32);
        stream.set_number_of_transfers(BUF_LEN as u16);

        // Drain stale RXNE then arm DMA.
        rx.read().ok();
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().set_bit());
        }
        unsafe { stream.enable() };

        // 2. Signal host: ready for 32-byte payload.
        tx.bwrite_all(b"READY\n").unwrap();

        // 3. Poll for transfer complete (32 bytes @ 115200 bps ≈ 2.8 ms).
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
            continue;
        }

        // 4. Allocate kernel buffer slot.
        let slot = match plib::sys_buf_alloc(true, 0) {
            Ok(s) => s,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                plib::sys_yield().ok();
                continue;
            }
        };

        // 5. Copy DMA data into kernel buffer (32 bytes — negligible).
        if plib::sys_buf_write(slot, &dma_buf).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 6. Transfer ownership to P1 — permanent handoff.
        //    After this call, P0 can NOT read/write/revoke/release this slot.
        if plib::sys_buf_transfer(slot, P1_PID.as_raw() as u8).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 7. Signal P1: buffer transferred, ready to consume.
        BUFFER_READY.store(true, Ordering::Release);

        // 8. Wait for P1 to finish (read + USB send + release).
        loop {
            match plib::sys_event_wait(BUF_DONE) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ── P1: USB CDC forwarder ────────────────────────────────────────────────────
//
// Owns OTG_FS. Polls USB continuously. When BUFFER_READY is set by P0,
// reads via sys_buf_read (P1 is now owner) and forwards via USB CDC.
// P1 calls sys_buf_release — ownership transferred, P1 is responsible.

extern "C" fn usb_cdc_body(_r0: u32) -> ! {
    static mut USB_DEV: Option<UsbDevice<'static, UsbBus<USB>>> = None;
    static mut USB_SER: Option<SerialPort<'static, UsbBus<USB>>> = None;

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
        // Always poll USB to keep enumeration alive.
        let _active = usb_dev.poll(&mut [serial]);

        // Discard any host→device data on the USB CDC port.
        {
            let mut discard = [0u8; 64];
            serial.read(&mut discard).ok();
        }

        // Non-blocking check: P1 must keep polling USB, cannot block.
        if BUFFER_READY.swap(false, Ordering::Acquire) {
            // P1 now owns the slot (transferred by P0). Read the data.
            let mut buf = [0u8; BUF_LEN];
            match plib::sys_buf_read(BUFFER_SLOT, &mut buf) {
                Ok(_) => {
                    // Write raw DMA payload to USB CDC with retry.
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
                    API_ERR.fetch_add(1, Ordering::Release);
                }
            }

            // P1 is the owner — release the slot back to the pool.
            if plib::sys_buf_release(BUFFER_SLOT).is_err() {
                API_ERR.fetch_add(1, Ordering::Release);
            }

            // Signal P0 via kernel event: done, slot released, safe to alloc again.
            if let Err(e) = plib::sys_event_set(P0_PID, BUF_DONE) {
                rprintln!("[P1] sys_event_set(P0, DONE) failed: {:?}", e);
            }
        }
    }
}

kernel::partition_trampoline!(uart_dma_main => uart_dma_body);
kernel::partition_trampoline!(usb_cdc_main  => usb_cdc_body);

// ── main ───────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rprintln!("\n=== UART DMA → Buffer Transfer → USB CDC — STM32F429ZI ===");
    rprintln!("Path: ttyACM0/UART → DMA1 → kernel buf → transfer(P1) → USB CDC → ttyACMN");
    rprintln!("P0: UART DMA receiver  P1: USB CDC forwarder  MPU_ENFORCE=true");
    rprintln!("Key: P0 transfers ownership, P1 releases. No lend/revoke round-trip.");

    let dp = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();

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

    let serial_uart = Serial::new(
        dp.USART3,
        (pd8_tx, pd9_rx),
        SerialConfig::default().baudrate(115_200.bps()).dma(UartDmaConfig::Rx),
        &mut rcc,
    ).expect("USART3 init");
    rprintln!("[INIT] USART3 115200 8N1 full-duplex PD8/PD9.");

    let dma1  = StreamsTuple::new(dp.DMA1, &mut rcc);
    let (tx, rx) = serial_uart.split();

    unsafe {
        *addr_of_mut!(TX_HANDLE)  = Some(tx);
        *addr_of_mut!(RX_HANDLE)  = Some(rx);
        *addr_of_mut!(DMA_STREAM) = Some(dma1.1);
        pac::Peripherals::steal().USART3.cr3().modify(|_, w| w.dmar().clear_bit());
    }

    // ── USB (OTG FS, PA11/PA12) ──
    let gpioa = dp.GPIOA.split(&mut rcc);
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    unsafe {
        G_USB_SERIAL = Some(SerialPort::new(usb_bus));
        G_USB_DEVICE = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(usbd_serial::USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("STM32")
                    .product("F429 BufTransfer Pipeline")
                    .serial_number("XFER002")])
                .unwrap()
                .build(),
        );
    }
    rprintln!("[INIT] USB initialized — VID:PID=16c0:27dd, product='F429 BufTransfer Pipeline'");

    // ── Kernel ──
    let mut sched = ScheduleTable::<{ XferCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");  // 4 ms: DMA poll + transfer
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");  // 4 ms: USB poll + read + send
    sched.add_system_window(1).expect("sched SW");

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

    let mut k = Kernel::<XferCfg>::new(sched, &mems).expect("kernel");
    rprintln!("[INIT] Kernel created. 2 partitions, 1 buffer slot x 32 bytes.");
    rprintln!("[INIT] P0: UART DMA (USART3+DMA1)  P1: USB CDC (OTG_FS)");
    rprintln!("[INIT] Schedule: P0=4ms P1=4ms SW=1ms (9ms frame, ~111Hz)");
    rprintln!("[INIT] Key difference: P0 transfers ownership, P1 releases.");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}
