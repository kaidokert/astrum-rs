//! USB CDC Option D — Partition IS the USB driver (SAME51 Curiosity Nano)
//!
//! The RTOS partition IS the USB driver. No USB ISR.
//! The partition polls `usb_dev.poll()` directly in its scheduler slot
//! under real MPU enforcement.
//!
//! Architecture:
//!   main() [privileged, pre-MPU]:
//!     Clocks → USB pins (PA24/PA25) → UsbBus → UsbDevice + SerialPort
//!     Stored in static muts. USB ISRs intentionally NOT unmasked.
//!     boot() arms MPU and enters scheduler.
//!
//!   USB partition (thread mode, MPU-enforced):
//!     Takes USB device + serial from statics on first call.
//!     Tight poll loop: usb_dev.poll() → serial.read() → echo → serial.write()
//!     No SYS_YIELD — runs full slot for max USB responsiveness.
//!
//! Build: cd same51_curiosity && cargo build --example usb_cdc_partition_d \
//!            --features kernel-mpu --release
//! Flash: probe-rs download --chip ATSAME51J20A --probe 03eb:2175 <elf>
//! Verify: echo text to /dev/ttyACM3 (16c0:27dd), see "[RTOS-D] " prefix echoed back.
//!         RTT shows rx_total advancing, MPU CTRL=0x5, MemManage=0.

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
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

// USB peripheral at 0x4100_0000, 1 KB MPU region covers all device registers.
const USB_BASE: u32 = 0x4100_0000;
const USB_SIZE: u32 = 0x400;

static RX_TOTAL: AtomicU32 = AtomicU32::new(0);
static TX_TOTAL: AtomicU32 = AtomicU32::new(0);

// USB objects — filled by main() before boot(), taken by partition.
static mut G_USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;
static mut G_USB_SERIAL: Option<SerialPort<'static, UsbBus>> = None;

kernel::kernel_config!(UsbDCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}

kernel::bind_interrupts!(UsbDCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(UsbDCfg, |tick, _k| {
    if tick % 5_000 == 0 && tick > 0 {
        let rx = RX_TOTAL.load(Ordering::Relaxed);
        let tx = TX_TOTAL.load(Ordering::Relaxed);
        rprintln!("[{:6}ms] USB rx_total={} tx_total={}", tick, rx, tx);
        if rx >= 10 {
            rprintln!("SUCCESS: Option D USB CDC under MPU! rx={} tx={}", rx, tx);
        }
    }

    // Dump MPU regions once to confirm USB wiring.
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU regions (usb_cdc_partition_d) --- CTRL=0x{:x}", ctrl);
        for r in 0u32..8 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            if rasr & 1 != 0 {
                rprintln!(
                    "  R{}: RBAR=0x{:08x} RASR=0x{:08x}",
                    r, rbar, rasr
                );
            }
        }
        rprintln!("---");
    }
});

// ---------------------------------------------------------------------------
// P0 — USB CDC driver (Option D: partition polls USB directly)
// ---------------------------------------------------------------------------
extern "C" fn usb_body(_r0: u32) -> ! {
    // Take USB objects from global statics (set by main() before boot()).
    static mut USB_DEV: Option<UsbDevice<'static, UsbBus>> = None;
    static mut USB_SER: Option<SerialPort<'static, UsbBus>> = None;

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
        if usb_dev.poll(&mut [serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                if count > 0 {
                    RX_TOTAL.fetch_add(count as u32, Ordering::Relaxed);
                    let sent_prefix = serial.write(b"[RTOS-D] ").unwrap_or(0);
                    let sent_data = serial.write(&buf[..count]).unwrap_or(0);
                    TX_TOTAL.fetch_add((sent_prefix + sent_data) as u32, Ordering::Relaxed);
                }
            }
        }
        // No SYS_YIELD — tight poll loop, PendSV preempts when slot expires.
    }
}

// P1: dummy — needed for Partitions2 config.
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
    static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

    rprintln!("=== USB CDC Option D — SAME51 Curiosity Nano ===");
    rprintln!("Partition polls USB directly. No USB ISR. MPU enforced.");

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
    let p = cortex_m::Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        gclk,
        &mut mclk,
        &mut osc32kctrl,
        &mut oscctrl,
        &mut nvmctrl,
    );

    // USB clock: GCLK2 from DFLL (48 MHz) → USB peripheral.
    let pins = hal::gpio::Pins::new(port);
    clocks.configure_gclk_divider_and_source(Genselect::Gclk2, 1, Srcselect::Dfll, false);
    let usb_gclk = clocks.get_gclk(Genselect::Gclk2).unwrap();
    let usb_clock = clocks.usb(&usb_gclk).unwrap();

    // Build USB bus allocator — must live in static for 'static lifetime.
    {
        *USB_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
            &usb_clock,
            &mut mclk,
            pins.pa24,
            pins.pa25,
            usb,
        )));
    }
    let bus_allocator = USB_ALLOCATOR.as_ref().unwrap();

    // Build USB device + serial. No Mutex needed — no ISR races.
    unsafe {
        G_USB_SERIAL = Some(SerialPort::new(bus_allocator));
        G_USB_DEVICE = Some(
            UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .strings(&[StringDescriptors::new(LangID::EN)
                    .manufacturer("Kaido")
                    .product("SAME51 RTOS-D USB")
                    .serial_number("same51-rtos-d")])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    // USB ISRs intentionally NOT unmasked — polling-only design.
    rprintln!("[INIT] USB initialized — VID:PID=16c0:27dd, polling-only (no ISR)");

    // Kernel setup.
    let mut sched = ScheduleTable::<{ UsbDCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // P0: SRAM + flash + USB peripheral.
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

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<UsbDCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    rprintln!("[INIT] Kernel ready. P0=USB driver (Option D), P1=dummy.");
    rprintln!("[INIT] USB peripheral MPU: 0x{:08X}/{}B", USB_BASE, USB_SIZE);
    rprintln!("[INIT] Booting with MPU enabled...\n");

    match boot(p).expect("boot") {}
}
