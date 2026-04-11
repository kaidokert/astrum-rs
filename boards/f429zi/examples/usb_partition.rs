//! USB CDC from RTOS Kernel — heapless::spsc variant
//!
//! Same architecture as usb_partition.rs but uses heapless::spsc::Queue instead of
//! manually managed raw arrays with AtomicU32 head/tail indices.
//!
//! heapless::spsc advantages over the raw-array approach:
//!   - No manual modular index arithmetic
//!   - Data buffer uses UnsafeCell internally — no raw static mut [u8; N] aliasing
//!   - Type-level SPSC enforcement: split() yields (Producer, Consumer); each is
//!     moved to exactly one owner, preventing accidental dual-producer or dual-consumer
//!   - enqueue/dequeue/peek API is more self-documenting than index math
//!
//! The Producer/Consumer halves are `!Sync` (by design — they rely on ownership, not
//! locking). Since ISR and partition body are both `fn` items (not closures), they
//! can't capture state; the halves must reach them via static storage. This example
//! uses the same lazy-init-from-global pattern already used for USB_DEVICE/USB_SERIAL:
//!   1. main() splits both queues and stores all four halves in static mut Option<...>
//!   2. ISR takes its two halves (RX Producer, TX Consumer) on first call
//!   3. Partition body takes its two halves (RX Consumer, TX Producer) on first call
//!
//! Hardware: STM32F429ZI NUCLEO-144
//!   PA11 = USB_DM, PA12 = USB_DP (USB OTG FS connector CN12)
//!
//! Build:  cd f429zi && cargo build --example usb_partition --features kernel-usb
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: /dev/ttyACM* shows "[RTOS] " echo prefix; RTT shows rx_total advancing.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use heapless::spsc::{Consumer, Producer, Queue};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::{rprintln, rtt_init_print};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE, USB_SYSCLK_HZ};
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac::{interrupt, Interrupt},
    prelude::*,
    rcc::Config,
};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// ── Kernel configuration ──────────────────────────────────────────────────────

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;
const USB_DATA_EVENT: u32 = 0x1;
const USB_PART: usize = 0;

// ── SPSC queues ───────────────────────────────────────────────────────────────
//
// Two 256-byte SPSC queues:
//   RX_Q: ISR enqueues bytes received from host; partition dequeues them.
//   TX_Q: partition enqueues bytes to send to host; ISR dequeues and writes to USB.
//
// Queue::new() is const — static initialisation is valid.
// heapless::spsc uses UnsafeCell internally so the data buffer is sound (no raw
// static mut [u8; N] aliasing). split() yields (Producer, Consumer) with 'static
// lifetime because the queues are 'static.
//
// SAFETY: each half is moved to exactly one owner (ISR or partition) and never
// accessed from more than one execution context simultaneously. The ISR is the sole
// Producer for RX_Q and sole Consumer for TX_Q; the partition is the sole Consumer
// for RX_Q and sole Producer for TX_Q. Single-core ARM: no concurrent access.

static mut RX_Q: Queue<u8, 256> = Queue::new();
static mut TX_Q: Queue<u8, 256> = Queue::new();

// Transfer cells: main() places each half here after splitting; the ISR and partition
// body each take their half on first call (same lazy-init pattern as USB_DEVICE).
// Using separate statics rather than a Mutex because ISR and partition access disjoint
// halves and single-core ARM provides sequential consistency for normal memory.
static mut ISR_RX_PROD: Option<Producer<'static, u8, 256>> = None;
static mut ISR_TX_CONS: Option<Consumer<'static, u8, 256>> = None;
static mut PART_RX_CONS: Option<Consumer<'static, u8, 256>> = None;
static mut PART_TX_PROD: Option<Producer<'static, u8, 256>> = None;

// ── Signal and statistics ─────────────────────────────────────────────────────

// Set by ISR when RX bytes arrive; atomically cleared by SysTick hook which then
// calls event_set() to wake the partition.
static RX_READY: AtomicBool = AtomicBool::new(false);
static RX_TOTAL: AtomicU32 = AtomicU32::new(0);
static TX_TOTAL: AtomicU32 = AtomicU32::new(0);

// ── USB globals ───────────────────────────────────────────────────────────────

static G_USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));
static G_USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

// ── Kernel config ─────────────────────────────────────────────────────────────

kernel::kernel_config!(UsbKernelConfig[AlignedStack2K]<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = false;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

// ── SysTick hook ─────────────────────────────────────────────────────────────

kernel::define_kernel!(UsbKernelConfig, |tick, k| {
    if RX_READY.swap(false, Ordering::Acquire) {
        kernel::events::event_set(k.partitions_mut(), USB_PART, USB_DATA_EVENT);
    }
    if tick % 5_000 == 0 && tick > 0 {
        let rx = RX_TOTAL.load(Ordering::Relaxed);
        let tx = TX_TOTAL.load(Ordering::Relaxed);
        rprintln!("[{:6}ms] USB rx_total={} tx_total={}", tick, rx, tx);
        if rx >= 10 {
            rprintln!("✓ SUCCESS: USB CDC SPSC from RTOS partition! rx={} tx={}", rx, tx);
        }
    }
});

// ── USB partition body ────────────────────────────────────────────────────────

extern "C" fn usb_main_body(_r0: u32) -> ! {
    // Take the partition's SPSC halves on first call.
    // SAFETY: PART_RX_CONS and PART_TX_PROD are set by main() before boot() and before
    // this partition runs; they are not accessed from the ISR (ISR has ISR_* halves).
    static mut RX_CONS: Option<Consumer<'static, u8, 256>> = None;
    static mut TX_PROD: Option<Producer<'static, u8, 256>> = None;

    // SAFETY: addr_of_mut! avoids creating &mut refs to static mut (Rust 2024 hard error).
    // RX_CONS/TX_PROD are function-local statics; PART_* are module-level statics.
    // Single-threaded ARM: no concurrent access between partition and ISR on these halves.
    let rx_cons = unsafe {
        let p = core::ptr::addr_of_mut!(RX_CONS);
        if (*p).is_none() { *p = Some((*core::ptr::addr_of_mut!(PART_RX_CONS)).take().unwrap()); }
        (*p).as_mut().unwrap()
    };
    let tx_prod = unsafe {
        let p = core::ptr::addr_of_mut!(TX_PROD);
        if (*p).is_none() { *p = Some((*core::ptr::addr_of_mut!(PART_TX_PROD)).take().unwrap()); }
        (*p).as_mut().unwrap()
    };

    loop {
        match plib::sys_event_wait(USB_DATA_EVENT) {
            Err(_) | Ok(0) => continue,
            Ok(_) => {}
        }

        // Drain all available RX bytes into a local buffer, then echo with prefix.
        // Collecting first lets us emit the "[RTOS] " prefix once per burst rather
        // than once per byte.
        let mut buf = [0u8; 64];
        let mut count = 0;
        while count < buf.len() {
            match rx_cons.dequeue() {
                Some(b) => { buf[count] = b; count += 1; }
                None => break,
            }
        }

        if count > 0 {
            RX_TOTAL.fetch_add(count as u32, Ordering::Relaxed);

            // Push prefix then data into TX queue; ISR drains it to USB on next interrupt.
            for &b in b"[RTOS] " {
                tx_prod.enqueue(b).ok(); // ok() drops overflow bytes silently
            }
            for &b in &buf[..count] {
                tx_prod.enqueue(b).ok();
            }
        }

        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(usb_main => usb_main_body);

// ── OTG_FS interrupt handler ──────────────────────────────────────────────────

#[interrupt]
fn OTG_FS() {
    // USB device objects — same lazy-init-from-global pattern.
    static mut USB_SERIAL: Option<SerialPort<UsbBus<USB>>> = None;
    static mut USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;

    let usb_dev = USB_DEVICE.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_USB_DEVICE.borrow(cs).replace(None).unwrap())
    });
    let serial = USB_SERIAL.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_USB_SERIAL.borrow(cs).replace(None).unwrap())
    });

    // SPSC halves for this ISR — taken once from the transfer statics set by main().
    // SAFETY: ISR_RX_PROD and ISR_TX_CONS are not accessed from the partition (it has
    // PART_* halves). take() makes them None so no double-take is possible.
    static mut RX_PROD: Option<Producer<'static, u8, 256>> = None;
    static mut TX_CONS: Option<Consumer<'static, u8, 256>> = None;

    let rx_prod = unsafe {
        let p = core::ptr::addr_of_mut!(RX_PROD);
        if (*p).is_none() { *p = Some((*core::ptr::addr_of_mut!(ISR_RX_PROD)).take().unwrap()); }
        (*p).as_mut().unwrap()
    };
    let tx_cons = unsafe {
        let p = core::ptr::addr_of_mut!(TX_CONS);
        if (*p).is_none() { *p = Some((*core::ptr::addr_of_mut!(ISR_TX_CONS)).take().unwrap()); }
        (*p).as_mut().unwrap()
    };

    // Flush TX queue to USB before polling.
    // peek().copied() borrows the byte value without holding a borrow on tx_cons,
    // so we can call tx_cons.dequeue() in the same iteration.
    while let Some(byte) = tx_cons.peek().copied() {
        match serial.write(&[byte]) {
            Ok(1) => {
                let _ = tx_cons.dequeue();
                TX_TOTAL.fetch_add(1, Ordering::Relaxed);
            }
            _ => break,
        }
    }

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        if let Ok(count) = serial.read(&mut buf) {
            if count > 0 {
                for &b in &buf[..count] {
                    rx_prod.enqueue(b).ok();
                }
                RX_READY.store(true, Ordering::Release);
            }
        }
        // Second flush — poll() may have freed endpoint buffers, allowing more TX.
        while let Some(byte) = tx_cons.peek().copied() {
            match serial.write(&[byte]) {
                Ok(1) => {
                    let _ = tx_cons.dequeue();
                    TX_TOTAL.fetch_add(1, Ordering::Relaxed);
                }
                _ => break,
            }
        }
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rprintln!("\n=== USB CDC RTOS Partition — heapless::spsc variant (STM32F429ZI) ===");
    rprintln!("heapless::spsc::Queue<u8,256> replaces raw static mut [u8;256] arrays");

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk()
    );

    let gpioa = dp.GPIOA.split(&mut rcc);

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    unsafe {
        *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    }
    let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

    cortex_m::interrupt::free(|cs| {
        *G_USB_SERIAL.borrow(cs).borrow_mut() = Some(SerialPort::new(usb_bus));
        *G_USB_DEVICE.borrow(cs).borrow_mut() = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(usbd_serial::USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("STM32")
                    .product("F429 RTOS USB SPSC")
                    .serial_number("SPSC001")])
                .unwrap()
                .build(),
        );
    });

    // Split both SPSC queues and distribute the halves to their owners.
    // This must happen before NVIC::unmask() so the ISR sees valid pointers on first call.
    // SAFETY: RX_Q and TX_Q are 'static and not yet split; single-threaded at this point.
    unsafe {
        let (rx_prod, rx_cons) = (*core::ptr::addr_of_mut!(RX_Q)).split();
        let (tx_prod, tx_cons) = (*core::ptr::addr_of_mut!(TX_Q)).split();

        ISR_RX_PROD = Some(rx_prod);   // ISR enqueues received bytes here
        ISR_TX_CONS = Some(tx_cons);   // ISR dequeues bytes to send here
        PART_RX_CONS = Some(rx_cons);  // partition dequeues received bytes here
        PART_TX_PROD = Some(tx_prod);  // partition enqueues bytes to send here
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
    }

    rprintln!("[INIT] USB initialized — VID:PID=16c0:27dd, product='F429 RTOS USB SPSC'");
    rprintln!("[INIT] SPSC queues split: ISR gets (RX_PROD, TX_CONS), partition gets (RX_CONS, TX_PROD)");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ UsbKernelConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(usb_main, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting kernel — SPSC echo partition ready\n");
    match boot(p).expect("boot") {}
}
