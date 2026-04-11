//! USB CDC Partition with PartitionAcks — SAME51 Curiosity Nano
//!
//! Clean ISR-driven USB: standard PartitionAcks flow, no NeverMask, no
//! custom handler. `__irq_dispatch` masks USB IRQs and signals the partition.
//! The partition owns the USB stack, polls in its own MPU context, acks to
//! unmask. Zero kernel changes required.
//!
//! Flow:
//!   1. USB IRQ fires → __irq_dispatch masks IRQ, signals partition event
//!   2. Partition wakes from sys_event_wait
//!   3. Partition calls usb_dev.poll() + reads/writes (own MPU context)
//!   4. Partition calls sys_irq_ack for each USB IRQ → unmask
//!   5. Repeat
//!
//! Build: cd same51_curiosity && cargo build --example usb_cdc_partition_ack \
//!            --features kernel-mpu --release
//! Verify: echo text to ttyACM for 16c0:27dd, see "[ACK] " prefix echoed back.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use atsamd_hal as hal;
use cortex_m_rt::{entry, exception};
use hal::clock::GenericClockController;
use hal::pac;
use hal::pac::gclk::{genctrl::Srcselect, pchctrl::Genselect};
use hal::usb::UsbBus;
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::EventMask;
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const NUM_PARTITIONS: usize = 2;
const IRQ_COUNT: usize = 136;

// USB peripheral MPU grant — partition accesses USB registers directly.
const USB_BASE: u32 = 0x4100_0000;
const USB_SIZE: u32 = 0x400;

// USB IRQ numbers (SAME51 datasheet).
const USB_OTHER_IRQ: u8 = 80;
const USB_TRCPT0_IRQ: u8 = 81;
const USB_TRCPT1_IRQ: u8 = 82;

// All three USB IRQs signal the same event bit.
const USB_EVENT: EventMask = EventMask::new(0x01);

static RX_TOTAL: AtomicU32 = AtomicU32::new(0);
static TX_TOTAL: AtomicU32 = AtomicU32::new(0);

// USB objects — owned exclusively by partition P0 after boot.
static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<'static, UsbBus>> = None;

// ---------------------------------------------------------------------------
// Kernel config
// ---------------------------------------------------------------------------
kernel::kernel_config!(UsbAckCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

// Standard PartitionAcks — no handler: form. __irq_dispatch masks + signals.
kernel::bind_interrupts!(UsbAckCfg, IRQ_COUNT,
    USB_OTHER_IRQ  => (0, 0x01),
    USB_TRCPT0_IRQ => (0, 0x01),
    USB_TRCPT1_IRQ => (0, 0x01),
);

kernel::define_kernel!(UsbAckCfg, |tick, _k| {
    if tick % 5_000 == 0 && tick > 0 {
        let rx = RX_TOTAL.load(Ordering::Relaxed);
        let tx = TX_TOTAL.load(Ordering::Relaxed);
        rprintln!("[{:6}ms] USB rx={} tx={}", tick, rx, tx);
        if rx >= 10 {
            rprintln!("SUCCESS: USB CDC PartitionAcks under MPU! rx={} tx={}", rx, tx);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — USB CDC echo with event-driven IRQ ack flow
// ---------------------------------------------------------------------------
extern "C" fn usb_body(_r0: u32) -> ! {
    // Take USB objects from statics (set by main before boot).
    static mut LOCAL_DEV: Option<UsbDevice<'static, UsbBus>> = None;
    static mut LOCAL_SER: Option<SerialPort<'static, UsbBus>> = None;

    let (usb_dev, serial) = unsafe {
        let dev_ptr = addr_of_mut!(LOCAL_DEV);
        let ser_ptr = addr_of_mut!(LOCAL_SER);
        if (*dev_ptr).is_none() {
            *dev_ptr = (*addr_of_mut!(USB_DEVICE)).take();
        }
        if (*ser_ptr).is_none() {
            *ser_ptr = (*addr_of_mut!(USB_SERIAL)).take();
        }
        ((*dev_ptr).as_mut().unwrap(), (*ser_ptr).as_mut().unwrap())
    };

    // Phase 1: poll aggressively until USB is configured.
    // Enumeration needs many rapid poll() round-trips; event-driven mode
    // is too slow at 1ms tick (each event cycle costs up to one major frame).
    loop {
        usb_dev.poll(&mut [serial]);
        if usb_dev.state() == UsbDeviceState::Configured {
            break;
        }
        // Ack any masked IRQs to keep them flowing during enumeration.
        plib::sys_irq_ack(USB_OTHER_IRQ).ok();
        plib::sys_irq_ack(USB_TRCPT0_IRQ).ok();
        plib::sys_irq_ack(USB_TRCPT1_IRQ).ok();
    }

    // Phase 2: event-driven — wait for IRQ, poll, echo, ack.
    loop {
        // Wait for USB event (any of the three IRQs).
        match plib::sys_event_wait(USB_EVENT) {
            Ok(bits) if bits.as_raw() != 0 => {}
            _ => continue,
        }

        // Poll USB and handle data — in partition's own MPU context.
        if usb_dev.poll(&mut [serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                if count > 0 {
                    RX_TOTAL.fetch_add(count as u32, Ordering::Relaxed);
                    let sent_prefix = serial.write(b"[ACK] ").unwrap_or(0);
                    let sent_data = serial.write(&buf[..count]).unwrap_or(0);
                    TX_TOTAL.fetch_add((sent_prefix + sent_data) as u32, Ordering::Relaxed);
                }
            }
        }

        // Clear event and ack all three USB IRQs (unmask).
        // Acking an already-unmasked IRQ is harmless (ISER write is idempotent).
        if let Err(e) = plib::sys_event_clear(USB_EVENT) {
            rprintln!("[P0] event_clear: {:?}", e);
        }
        plib::sys_irq_ack(USB_OTHER_IRQ).ok();
        plib::sys_irq_ack(USB_TRCPT0_IRQ).ok();
        plib::sys_irq_ack(USB_TRCPT1_IRQ).ok();
    }
}

// P1: dummy.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(usb_main => usb_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("=== USB CDC PartitionAcks — SAME51 Curiosity Nano ===");
    rprintln!("Standard flow: __irq_dispatch masks → partition polls → ack unmasks");

    let pac::Peripherals {
        gclk,
        mut mclk,
        mut osc32kctrl,
        mut oscctrl,
        mut nvmctrl,
        port,
        usb,
        ..
    } = pac::Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        gclk,
        &mut mclk,
        &mut osc32kctrl,
        &mut oscctrl,
        &mut nvmctrl,
    );

    let pins = hal::gpio::Pins::new(port);
    clocks.configure_gclk_divider_and_source(Genselect::Gclk2, 1, Srcselect::Dfll, false);
    let usb_gclk = clocks.get_gclk(Genselect::Gclk2).unwrap();
    let usb_clock = clocks.usb(&usb_gclk).unwrap();

    static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
    let alloc_ptr = addr_of_mut!(USB_ALLOCATOR);
    unsafe {
        *alloc_ptr = Some(UsbBusAllocator::new(UsbBus::new(
            &usb_clock,
            &mut mclk,
            pins.pa24,
            pins.pa25,
            usb,
        )));
    }
    let bus_allocator = unsafe { (*alloc_ptr).as_ref().unwrap() };

    let serial = SerialPort::new(bus_allocator);
    let device = UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("Kaido")
            .product("SAME51 ACK USB")
            .serial_number("same51-ack")])
        .expect("Failed to set strings")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    unsafe {
        *addr_of_mut!(USB_DEVICE) = Some(device);
        *addr_of_mut!(USB_SERIAL) = Some(serial);
    }

    rprintln!("[INIT] USB configured. VID:PID=16c0:27dd");

    // Kernel setup.
    let mut sched = ScheduleTable::<{ UsbAckCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // P0: USB peripheral grant — partition polls USB directly.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, usb_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(USB_BASE, USB_SIZE, 0),
        ])
        .expect("periph0");

    // P1: no peripheral access.
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<UsbAckCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    enable_bound_irqs(&mut p.NVIC, UsbAckCfg::IRQ_DEFAULT_PRIORITY).expect("irqs");
    rprintln!("[INIT] USB IRQs bound: PartitionAcks model. P0 owns USB.");
    rprintln!("[INIT] Booting with MPU enabled...\n");

    match boot(p).expect("boot") {}
}
