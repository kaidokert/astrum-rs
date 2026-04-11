//! Virtual Device UART Demo — STM32F429ZI
//!
//! Model C: partition uses SYS_DEV_WRITE / SYS_DEV_READ via kernel's
//! virtual device layer. Partition never touches USART3 registers.
//!
//! Build:  cd f429zi && cargo build --example vdev_uart \
//!             --features kernel-irq-mpu --no-default-features --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionId, PartitionSpec,
    scheduler::{ScheduleEntry, ScheduleTable},
    split_isr::StaticIsrRing,
    virtual_device::{DeviceError, VirtualDevice},
    DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal,
};
use plib::{DeviceId, EventMask};
use rtt_target::rprintln;
use f429zi as _;

const USART3_BASE: u32 = 0x4000_4800;
const USART3_SR: *const u32 = USART3_BASE as *const u32;
const USART3_DR: *mut u32 = (USART3_BASE + 0x04) as *mut u32;
const USART3_BRR: *mut u32 = (USART3_BASE + 0x08) as *mut u32;
const USART3_CR1: *mut u32 = (USART3_BASE + 0x0C) as *mut u32;
const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
const RCC_APB1ENR: *mut u32 = 0x4002_3840 as *mut u32;
const GPIOD_MODER: *mut u32 = 0x4002_0C00 as *mut u32;
const GPIOD_AFRH: *mut u32 = 0x4002_0C24 as *mut u32;

const USART3_IRQ: u8 = 39;
const UART_EVENT: EventMask = EventMask::new(0x01);
const DEV_UART: DeviceId = DeviceId::new(0);
const HSI_HZ: u32 = 16_000_000;
const BAUD: u32 = 115_200;

static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static TX_COUNT: AtomicU32 = AtomicU32::new(0);
static ISR_COUNT: AtomicU32 = AtomicU32::new(0);
static PUSH_FAIL: AtomicU32 = AtomicU32::new(0);
static RING: StaticIsrRing<128, 1> = StaticIsrRing::new();

// ── STM32 USART3 VirtualDevice ──
struct Stm32Usart3Device { opened: bool }

impl Stm32Usart3Device {
    const fn new() -> Self { Self { opened: false } }
}

unsafe impl Send for Stm32Usart3Device {}

impl VirtualDevice for Stm32Usart3Device {
    fn device_id(&self) -> u8 { 0 }
    fn open(&mut self, _pid: PartitionId) -> Result<(), DeviceError> { self.opened = true; Ok(()) }
    fn close(&mut self, _pid: PartitionId) -> Result<(), DeviceError> { self.opened = false; Ok(()) }

    fn write(&mut self, _pid: PartitionId, data: &[u8]) -> Result<usize, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        for &byte in data {
            unsafe {
                while core::ptr::read_volatile(USART3_SR) & (1 << 7) == 0 {}
                core::ptr::write_volatile(USART3_DR, byte as u32);
            }
            TX_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        Ok(data.len())
    }

    fn read(&mut self, _pid: PartitionId, buf: &mut [u8]) -> Result<usize, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        let mut count = 0usize;
        while count < buf.len() {
            let mut got = false;
            unsafe {
                RING.pop_with(|_tag, data| {
                    if let Some(&byte) = data.first() {
                        buf[count] = byte;
                        got = true;
                    }
                });
            }
            if got { count += 1; RX_COUNT.fetch_add(1, Ordering::Relaxed); }
            else { break; }
        }
        if count == 0 { Err(DeviceError::BufferEmpty) } else { Ok(count) }
    }

    fn ioctl(&mut self, _pid: PartitionId, _cmd: u32, _arg: u32) -> Result<u32, DeviceError> { Ok(0) }
}

static mut UART_DEV: Stm32Usart3Device = Stm32Usart3Device::new();

// ── Kernel config (use init_kernel + boot harness path) ──
kernel::kernel_config!(VdevCfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    mpu_enforce = false;
});

kernel::bind_interrupts!(VdevCfg, 91,
    USART3_IRQ => (0, 0x01, handler: usart3_rx_isr),
);

kernel::define_kernel!(VdevCfg, |tick, _k| {
    if tick % 1000 == 0 {
        kernel::klog!("[{:5}ms] ISR={} PUSH_FAIL={} RX={} TX={}", tick,
            ISR_COUNT.load(Ordering::Acquire), PUSH_FAIL.load(Ordering::Acquire),
            RX_COUNT.load(Ordering::Acquire), TX_COUNT.load(Ordering::Acquire));
    }
});

unsafe extern "C" fn usart3_rx_isr() {
    let sr = unsafe { core::ptr::read_volatile(USART3_SR) };
    if sr & (1 << 5) != 0 {
        let byte = unsafe { core::ptr::read_volatile(USART3_DR as *const u32) } as u8;
        ISR_COUNT.fetch_add(1, Ordering::Relaxed);
        if unsafe { RING.push_from_isr(USART3_IRQ, &[byte]) }.is_err() {
            PUSH_FAIL.fetch_add(1, Ordering::Relaxed);
        }
    }
    if sr & (1 << 3) != 0 {
        let _ = unsafe { core::ptr::read_volatile(USART3_DR as *const u32) };
    }
    kernel::irq_dispatch::signal_partition_from_isr::<VdevCfg>(PartitionId::new(0), 0x01);
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(USART3_IRQ));
}

/// Drain ring → echo via SYS_DEV_WRITE. Returns true if any bytes were echoed.
fn drain_and_echo() -> bool {
    let mut buf = [0u8; 32];
    let mut any = false;
    loop {
        match plib::sys_dev_read(DEV_UART, &mut buf) {
            Ok(n) if n > 0 => {
                // buf is on stack (RAM) → passes kernel ptr check
                if let Err(e) = plib::sys_dev_write(DEV_UART, &buf[..n as usize]) {
                    rprintln!("[ECHO] dev_write failed: {:?}", e);
                }
                any = true;
            }
            _ => break,
        }
    }
    any
}

extern "C" fn echo_body(_r0: u32) -> ! {
    plib::sys_dev_open(DEV_UART).expect("dev_open");
    loop {
        if let Err(e) = plib::sys_event_wait(UART_EVENT) {
            rprintln!("[ECHO] event_wait: {:?}", e);
            if let Err(e2) = plib::sys_yield() {
                rprintln!("[ECHO] yield: {:?}", e2);
            }
            continue;
        }
        // Drain → clear → ack → drain again until truly empty.
        // Bytes arriving between clear and ack are caught by the second drain.
        loop {
            drain_and_echo();
            if let Err(e) = plib::sys_event_clear(UART_EVENT) {
                rprintln!("[ECHO] event_clear: {:?}", e);
            }
            if let Err(e) = plib::sys_irq_ack(USART3_IRQ) {
                rprintln!("[ECHO] irq_ack: {:?}", e);
            }
            if !drain_and_echo() { break; }
        }
    }
}

kernel::partition_trampoline!(echo_partition => echo_body);

fn init_usart3() {
    unsafe {
        let ahb1 = core::ptr::read_volatile(RCC_AHB1ENR);
        core::ptr::write_volatile(RCC_AHB1ENR, ahb1 | (1 << 3));
        let apb1 = core::ptr::read_volatile(RCC_APB1ENR);
        core::ptr::write_volatile(RCC_APB1ENR, apb1 | (1 << 18));
        let moder = core::ptr::read_volatile(GPIOD_MODER);
        core::ptr::write_volatile(GPIOD_MODER, (moder & !((3 << 16) | (3 << 18))) | (2 << 16) | (2 << 18));
        let afrh = core::ptr::read_volatile(GPIOD_AFRH);
        core::ptr::write_volatile(GPIOD_AFRH, (afrh & !((0xF << 0) | (0xF << 4))) | (7 << 0) | (7 << 4));
        core::ptr::write_volatile(USART3_BRR, HSI_HZ / BAUD);
        core::ptr::write_volatile(USART3_CR1, (1 << 13) | (1 << 3) | (1 << 2) | (1 << 5));
    }
}

#[entry]
fn main() -> ! {
    rprintln!("=== Virtual Device UART — Model C ===");
    let mut p = cortex_m::Peripherals::take().unwrap();

    init_usart3();

    let mut sched = ScheduleTable::<{ VdevCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; 1] = [PartitionSpec::new(echo_partition as kernel::PartitionEntry, 0)];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Register USART3 virtual device in kernel registry
    kernel::state::with_kernel_mut::<VdevCfg, _, _>(|k| {
        unsafe {
            let dev: &'static mut dyn VirtualDevice = &mut *core::ptr::addr_of_mut!(UART_DEV);
            k.registry.add(dev).expect("register UART dev");
        }
        Ok::<(), ()>(())
    }).expect("registry");

    enable_bound_irqs(&mut p.NVIC, VdevCfg::IRQ_DEFAULT_PRIORITY).expect("irqs");
    rprintln!("[INIT] Booting — echo via SYS_DEV_READ/WRITE");

    match boot(p).expect("boot") {}
}
