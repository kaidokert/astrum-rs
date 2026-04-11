//! USB CDC Virtual Device — SAME51 Curiosity Nano
//!
//! Model C: partition uses SYS_DEV_WRITE / SYS_DEV_READ via kernel's
//! virtual device layer. Partition never touches USB registers.
//!
//! Architecture:
//!   USB ISRs (USB_OTHER/TRCPT0/TRCPT1):
//!     Poll usb_dev, push RX data into StaticIsrRing, signal partition.
//!   UsbCdcDevice (impl VirtualDevice):
//!     read() pops from ring. write() calls serial.write() (safe: SVCall
//!     context runs with PRIMASK=1, no ISR race).
//!   Partition:
//!     sys_dev_open → loop { sys_event_wait → sys_dev_read → sys_dev_write }
//!
//! Build: cd same51_curiosity && cargo build --example usb_cdc_vdev \
//!            --features kernel-mpu --release
//! Verify: echo text to ttyACM for 16c0:27dd, see "[VDEV] " prefix echoed back.

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
    PartitionId,
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    split_isr::StaticIsrRing,
    svc::Kernel,
    virtual_device::{DeviceError, VirtualDevice},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::DeviceId;
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

// USB peripheral grant needed even though partition doesn't touch USB directly:
// the USB ISR (usb_isr/poll_usb) accesses USB registers and runs in the
// current partition's MPU context. Without a grant, the deny-all background
// region (R0) causes DACCVIOL when the ISR fires.
const USB_BASE: u32 = 0x4100_0000;
const USB_SIZE: u32 = 0x400;

const DEV_USB: DeviceId = DeviceId::new(0);

static RX_TOTAL: AtomicU32 = AtomicU32::new(0);
static TX_TOTAL: AtomicU32 = AtomicU32::new(0);

// RX ring: ISR pushes, VirtualDevice::read pops. 32 slots x 64 bytes.
static RX_RING: StaticIsrRing<32, 64> = StaticIsrRing::new();

// USB objects — shared between ISR and VirtualDevice (mutual exclusion
// guaranteed: ISR runs when PRIMASK=0, VDevice methods called from SVCall
// with PRIMASK=1).
static mut USB_BUS: Option<UsbDevice<'static, UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<'static, UsbBus>> = None;

// ---------------------------------------------------------------------------
// USB ISR handlers — poll USB, push RX into ring
// ---------------------------------------------------------------------------
fn poll_usb() {
    unsafe {
        let usb_dev = match (*addr_of_mut!(USB_BUS)).as_mut() {
            Some(d) => d,
            None => return,
        };
        let serial = match (*addr_of_mut!(USB_SERIAL)).as_mut() {
            Some(s) => s,
            None => return,
        };

        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 64];
        if let Ok(count) = serial.read(&mut buf) {
            if count > 0 {
                let _ = RX_RING.push_from_isr(0, &buf[..count]);
            }
        }
    }
}

// USB ISR handlers registered via bind_interrupts! (usb_isr → poll_usb).

// ---------------------------------------------------------------------------
// VirtualDevice impl — wraps USB CDC behind kernel device API
// ---------------------------------------------------------------------------
struct UsbCdcDevice {
    opened: bool,
}

impl UsbCdcDevice {
    const fn new() -> Self { Self { opened: false } }
}

unsafe impl Send for UsbCdcDevice {}

impl VirtualDevice for UsbCdcDevice {
    fn device_id(&self) -> u8 { 0 }

    fn open(&mut self, _pid: PartitionId) -> Result<(), DeviceError> {
        self.opened = true;
        Ok(())
    }

    fn close(&mut self, _pid: PartitionId) -> Result<(), DeviceError> {
        self.opened = false;
        Ok(())
    }

    fn read(&mut self, _pid: PartitionId, buf: &mut [u8]) -> Result<usize, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        let mut count = 0usize;
        while count < buf.len() {
            let mut got = false;
            unsafe {
                RX_RING.pop_with(|_tag, data| {
                    let n = data.len().min(buf.len() - count);
                    buf[count..count + n].copy_from_slice(&data[..n]);
                    count += n;
                    got = true;
                });
            }
            if !got { break; }
        }
        if count == 0 { Err(DeviceError::BufferEmpty) } else {
            RX_TOTAL.fetch_add(count as u32, Ordering::Relaxed);
            Ok(count)
        }
    }

    fn write(&mut self, _pid: PartitionId, data: &[u8]) -> Result<usize, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        // Safe: called from SVCall context with PRIMASK=1, USB ISR cannot preempt.
        let serial = unsafe {
            match (*addr_of_mut!(USB_SERIAL)).as_mut() {
                Some(s) => s,
                None => return Err(DeviceError::NotOpen),
            }
        };
        let sent = serial.write(data).unwrap_or(0);
        TX_TOTAL.fetch_add(sent as u32, Ordering::Relaxed);
        Ok(sent)
    }

    fn ioctl(&mut self, _pid: PartitionId, _cmd: u32, _arg: u32) -> Result<u32, DeviceError> {
        Ok(0)
    }
}

static mut USB_CDC_DEV: UsbCdcDevice = UsbCdcDevice::new();

// ---------------------------------------------------------------------------
// Kernel config
// ---------------------------------------------------------------------------
kernel::kernel_config!(VdevCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
unsafe extern "C" fn usb_isr() {
    poll_usb();
    // Signal partition 0 that USB data may be available.
    kernel::irq_dispatch::signal_partition_from_isr::<VdevCfg>(PartitionId::new(0), 0x02);
}

const USB_OTHER_IRQ: u8 = 80;
const USB_TRCPT0_IRQ: u8 = 81;
const USB_TRCPT1_IRQ: u8 = 82;

kernel::bind_interrupts!(VdevCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
    USB_OTHER_IRQ => (0, 0x0000_0002, never_mask, handler: usb_isr),
    USB_TRCPT0_IRQ => (0, 0x0000_0002, never_mask, handler: usb_isr),
    USB_TRCPT1_IRQ => (0, 0x0000_0002, never_mask, handler: usb_isr),
);

kernel::define_kernel!(VdevCfg, |tick, _k| {
    if tick % 5_000 == 0 && tick > 0 {
        let rx = RX_TOTAL.load(Ordering::Relaxed);
        let tx = TX_TOTAL.load(Ordering::Relaxed);
        rprintln!("[{:6}ms] USB rx_total={} tx_total={}", tick, rx, tx);
        if rx >= 10 {
            rprintln!("SUCCESS: Virtual Device USB CDC under MPU! rx={} tx={}", rx, tx);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — echo via virtual device API (never touches USB registers)
// ---------------------------------------------------------------------------
extern "C" fn echo_body(_r0: u32) -> ! {
    plib::sys_dev_open(DEV_USB).expect("dev_open");

    loop {
        // Poll virtual device for data. Partition never touches USB hardware.
        let mut buf = [0u8; 64];
        match plib::sys_dev_read(DEV_USB, &mut buf) {
            Ok(n) if n > 0 => {
                // Echo with prefix. Stack buffer avoids flash pointer trap.
                let prefix = b"[VDEV] ";
                let mut out = [0u8; 73]; // 7 prefix + 64 data max
                let plen = prefix.len();
                out[..plen].copy_from_slice(prefix);
                let dlen = (n as usize).min(64);
                out[plen..plen + dlen].copy_from_slice(&buf[..dlen]);
                if let Err(e) = plib::sys_dev_write(DEV_USB, &out[..plen + dlen]) {
                    rprintln!("[P0] dev_write failed: {:?}", e);
                }
            }
            _ => {
                // No data — yield to avoid busy-spinning.
                plib::sys_yield().ok();
            }
        }
    }
}

// P1: dummy.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(echo_main => echo_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("=== USB CDC Virtual Device — SAME51 Curiosity Nano ===");
    rprintln!("Model C: partition uses SYS_DEV_READ/WRITE. USB ISR-owned.");

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

    // Build USB stack in main (privileged, pre-MPU).
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
            .product("SAME51 VDEV USB")
            .serial_number("same51-vdev")])
        .expect("Failed to set strings")
        .device_class(USB_CLASS_CDC)
        .build();

    // Transfer to shared statics for ISR + VDevice access.
    unsafe {
        *addr_of_mut!(USB_BUS) = Some(device);
        *addr_of_mut!(USB_SERIAL) = Some(serial);
    }

    rprintln!("[INIT] USB configured. VID:PID=16c0:27dd");

    // Kernel setup.
    let mut sched = ScheduleTable::<{ VdevCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // Both partitions need USB peripheral grant: the USB ISR accesses USB
    // registers and inherits the current partition's MPU context.
    let usb_periph = [MpuRegion::new(USB_BASE, USB_SIZE, 0)];

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, echo_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&usb_periph)
        .expect("periph0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1")
        .with_peripheral_regions(&usb_periph)
        .expect("periph1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<VdevCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    // Register USB virtual device in kernel registry.
    let _ = kernel::state::with_kernel_mut::<VdevCfg, _, _>(|k| {
        unsafe {
            let dev: &'static mut dyn VirtualDevice = &mut *addr_of_mut!(USB_CDC_DEV);
            k.registry.add(dev).expect("register USB CDC dev");
        }
        Ok::<(), ()>(())
    }).expect("registry");

    enable_bound_irqs(&mut p.NVIC, VdevCfg::IRQ_DEFAULT_PRIORITY).expect("irqs");

    // Bootstrap USB enumeration: poll in a loop before boot() so the host
    // completes SET_ADDRESS / GET_DESCRIPTOR / SET_CONFIGURATION while we're
    // still in privileged main() with no MPU deny-all. After boot(), the
    // NeverMask ISR takes over for ongoing data flow.
    rprintln!("[INIT] Bootstrapping USB enumeration (polling for ~2s)...");
    for _ in 0..200_000u32 {
        poll_usb();
        cortex_m::asm::delay(600); // ~5µs at 120 MHz → ~1s total
    }

    rprintln!("[INIT] USB CDC virtual device registered. USB ISRs enabled.");
    rprintln!("[INIT] P0 uses SYS_DEV_READ/WRITE only.");
    rprintln!("[INIT] Booting with MPU enabled...\n");

    match boot(p).expect("boot") {}
}

