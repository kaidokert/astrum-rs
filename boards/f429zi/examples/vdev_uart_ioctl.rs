//! Virtual Device UART IOCTL + CLOSE Demo — runtime baud rate change and device lifecycle
//!
//! Demonstrates SYS_DEV_IOCTL and SYS_DEV_CLOSE by changing USART3 baud rate
//! at runtime and performing a close/reopen cycle. The partition uses the
//! kernel's virtual device layer exclusively — never touches USART3 registers.
//!
//! Protocol:
//!   Phase 1 (115200 bps): partition echoes bytes, host verifies
//!   Phase 2: host sends "BAUD:9600\n", partition calls SYS_DEV_IOCTL
//!   Phase 3 (9600 bps): host reconnects at 9600, partition echoes, host verifies
//!   Phase 4: host sends "BAUD:115200\n", partition restores original rate
//!   Phase 5: host sends "CLOSE\n", partition closes device, attempts write
//!            (expects NotOpen error), reopens, sends "REOPENED\n"
//!
//! Success: both baud rates echo correctly, close/reopen cycle completes.
//!
//! Build:  cd f429zi && cargo build --example vdev_uart_ioctl \
//!             --features kernel-irq-mpu-hal --no-default-features

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionId, PartitionSpec, PartitionEntry,
    scheduler::{ScheduleEntry, ScheduleTable},
    split_isr::StaticIsrRing,
    virtual_device::{DeviceError, VirtualDevice},
    DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal,
};
use plib::{DeviceId, EventMask};
use rtt_target::rprintln;
use f429zi as _;

// ── USART3 registers (ST-LINK VCP: PD8 TX, PD9 RX) ──
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
const HSI_HZ: u32 = 16_000_000; // APB1 clock = HSI (no PLL configured)

// IOCTL commands
const IOCTL_SET_BAUD: u32 = 1;
const IOCTL_GET_BAUD: u32 = 2;

static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static TX_COUNT: AtomicU32 = AtomicU32::new(0);
static ISR_COUNT: AtomicU32 = AtomicU32::new(0);
static IOCTL_COUNT: AtomicU32 = AtomicU32::new(0);
static CLOSE_COUNT: AtomicU32 = AtomicU32::new(0);
static REOPEN_COUNT: AtomicU32 = AtomicU32::new(0);
static CURRENT_BAUD: AtomicU32 = AtomicU32::new(115_200);
static RING: StaticIsrRing<128, 1> = StaticIsrRing::new();

// ── USART3 Virtual Device with IOCTL support ──
struct Stm32Usart3Device {
    opened: bool,
    baud: u32,
}

impl Stm32Usart3Device {
    const fn new() -> Self { Self { opened: false, baud: 115_200 } }
}

unsafe impl Send for Stm32Usart3Device {}

impl VirtualDevice for Stm32Usart3Device {
    fn device_id(&self) -> u8 { 0 }

    fn open(&mut self, _pid: PartitionId) -> Result<(), DeviceError> {
        self.opened = true;
        Ok(())
    }

    fn close(&mut self, _pid: PartitionId) -> Result<(), DeviceError> {
        self.opened = false;
        Ok(())
    }

    fn write(&mut self, _pid: PartitionId, data: &[u8]) -> Result<usize, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        for &byte in data {
            unsafe {
                while core::ptr::read_volatile(USART3_SR) & (1 << 7) == 0 {} // TXE
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

    fn ioctl(&mut self, _pid: PartitionId, cmd: u32, arg: u32) -> Result<u32, DeviceError> {
        if !self.opened { return Err(DeviceError::NotOpen); }
        match cmd {
            IOCTL_SET_BAUD => {
                let new_baud = arg;
                if new_baud == 0 || new_baud > 921_600 {
                    return Err(DeviceError::BufferFull); // reuse as "invalid arg"
                }
                // Disable USART, change BRR, re-enable
                unsafe {
                    let cr1 = core::ptr::read_volatile(USART3_CR1);
                    core::ptr::write_volatile(USART3_CR1, cr1 & !(1 << 13)); // UE=0
                    core::ptr::write_volatile(USART3_BRR, HSI_HZ / new_baud);
                    core::ptr::write_volatile(USART3_CR1, cr1); // restore (UE=1)
                }
                self.baud = new_baud;
                CURRENT_BAUD.store(new_baud, Ordering::Release);
                IOCTL_COUNT.fetch_add(1, Ordering::Relaxed);
                Ok(new_baud)
            }
            IOCTL_GET_BAUD => {
                Ok(self.baud)
            }
            _ => Err(DeviceError::NotOpen), // reuse as "unknown cmd"
        }
    }
}

static mut UART_DEV: Stm32Usart3Device = Stm32Usart3Device::new();

// ── Kernel config ──
kernel::kernel_config!(IoctlCfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    mpu_enforce = false;
});

kernel::bind_interrupts!(IoctlCfg, 91,
    USART3_IRQ => (0, 0x01, handler: usart3_rx_isr),
);

kernel::define_kernel!(IoctlCfg, |tick, _k| {
    if tick % 500 == 0 {
        let baud = CURRENT_BAUD.load(Ordering::Acquire);
        let ioctl = IOCTL_COUNT.load(Ordering::Acquire);
        let closes = CLOSE_COUNT.load(Ordering::Acquire);
        let reopens = REOPEN_COUNT.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] baud={} ISR={} RX={} TX={} IOCTL={} CLOSE={} REOPEN={}",
            tick, baud,
            ISR_COUNT.load(Ordering::Acquire),
            RX_COUNT.load(Ordering::Acquire),
            TX_COUNT.load(Ordering::Acquire),
            ioctl, closes, reopens,
        );
        if ioctl >= 2 && reopens >= 1 {
            rprintln!("SUCCESS: IOCTL + CLOSE/REOPEN lifecycle verified!");
        }
    }
});

// ── ISR: USART3 RX → ring buffer ──
// Drain ALL available bytes per invocation — at 115200 baud, bytes arrive
// every ~87µs but kernel context switch takes ~700µs in debug builds, so
// multiple bytes can accumulate between IRQ fire and partition ack.
unsafe extern "C" fn usart3_rx_isr() {
    loop {
        let sr = unsafe { core::ptr::read_volatile(USART3_SR) };
        if sr & (1 << 5) != 0 { // RXNE
            let byte = unsafe { core::ptr::read_volatile(USART3_DR as *const u32) } as u8;
            ISR_COUNT.fetch_add(1, Ordering::Relaxed);
            let _ = unsafe { RING.push_from_isr(USART3_IRQ, &[byte]) };
        } else if sr & (1 << 3) != 0 { // ORE without RXNE — clear by reading DR
            let _ = unsafe { core::ptr::read_volatile(USART3_DR as *const u32) };
        } else {
            break;
        }
    }
    // Signal partition but do NOT mask NVIC — at 115200 baud, bytes arrive
    // every ~87µs but debug-build context switch is ~700µs. Masking causes
    // overrun because RXNE goes unserviced while IRQ is masked. The ISR's
    // drain loop ensures RXNE is clear on return, preventing retriggering.
    kernel::irq_dispatch::signal_partition_from_isr::<IoctlCfg>(PartitionId::new(0), 0x01);
}

// ── Line parser state (shared across drain passes) ──
struct LineParser {
    cmd_buf: [u8; 32],
    cmd_len: usize,
}

impl LineParser {
    const fn new() -> Self { Self { cmd_buf: [0; 32], cmd_len: 0 } }

    /// Process received bytes: accumulate into line buffer, dispatch on '\n'.
    fn process(&mut self, data: &[u8]) {
        for &byte in data {
            if byte == b'\n' {
                self.dispatch_line();
                self.cmd_len = 0;
            } else if self.cmd_len < self.cmd_buf.len() {
                self.cmd_buf[self.cmd_len] = byte;
                self.cmd_len += 1;
            }
        }
    }

    fn dispatch_line(&self) {
        if self.cmd_len >= 5 && &self.cmd_buf[..5] == b"BAUD:" {
            let baud = parse_u32(&self.cmd_buf[5..self.cmd_len]);
            if baud > 0 {
                // IOCTL changes the baud rate in the kernel's device driver.
                // The "OK\n" response is sent at the NEW rate — host must
                // switch rates before reading it.
                match plib::sys_dev_ioctl(DEV_UART, IOCTL_SET_BAUD, baud) {
                    Ok(_) => dev_write(b"OK\n"),
                    Err(e) => {
                        rprintln!("[IOCTL] SET_BAUD {} failed: {:?}", baud, e);
                        dev_write(b"ERR\n");
                    }
                }
            } else {
                dev_write(b"ERR\n");
            }
        } else if self.cmd_len == 5 && &self.cmd_buf[..5] == b"CLOSE" {
            // Close/reopen lifecycle test:
            // 1. Close device — should succeed
            // 2. Attempt write on closed device — should fail (NotOpen)
            // 3. Reopen device — should succeed
            // 4. Write "REOPENED\n" — proves full lifecycle works

            match plib::sys_dev_close(DEV_UART) {
                Ok(_) => CLOSE_COUNT.fetch_add(1, Ordering::Relaxed),
                Err(e) => { rprintln!("[CLOSE] close failed: {:?}", e); 0 }
            };

            // Write while closed — must be rejected
            let ghost = *b"GHOST\n";
            let closed_write = plib::sys_dev_write(DEV_UART, &ghost);
            if closed_write.is_ok() {
                rprintln!("[CLOSE] BUG: write succeeded on closed device!");
            }

            match plib::sys_dev_open(DEV_UART) {
                Ok(_) => {
                    REOPEN_COUNT.fetch_add(1, Ordering::Relaxed);
                    if closed_write.is_err() {
                        dev_write(b"REOPENED\n");
                    } else {
                        dev_write(b"CLOSE_BUG\n");
                    }
                }
                Err(e) => rprintln!("[CLOSE] reopen failed: {:?}", e),
            }
        } else {
            // Echo the line back (cmd_buf is on stack → passes kernel ptr check)
            if let Err(e) = plib::sys_dev_write(DEV_UART, &self.cmd_buf[..self.cmd_len]) {
                rprintln!("[ECHO] write failed: {:?}", e);
            }
            let nl = [b'\n'];
            if let Err(e) = plib::sys_dev_write(DEV_UART, &nl) {
                rprintln!("[ECHO] newline failed: {:?}", e);
            }
        }
    }
}

/// Drain the ring buffer, processing all bytes through the line parser.
/// Returns true if any bytes were read.
fn drain_ring(parser: &mut LineParser) -> bool {
    let mut buf = [0u8; 32];
    let mut any = false;
    loop {
        match plib::sys_dev_read(DEV_UART, &mut buf) {
            Ok(n) if n > 0 => {
                parser.process(&buf[..n as usize]);
                any = true;
            }
            _ => break,
        }
    }
    any
}

// ── Partition: echo + IOCTL command parsing ──
/// Write a byte string to the UART device, bouncing through a stack buffer.
///
/// The kernel's `check_user_ptr_dynamic` validates that SYS_DEV_WRITE data
/// pointers lie within the calling partition's RAM region. Flash-resident
/// string literals (0x0800xxxx) are rejected with `InvalidPointer`. This
/// helper copies to the stack first so the pointer passes validation.
fn dev_write(msg: &[u8]) {
    let mut buf = [0u8; 32];
    let len = msg.len().min(buf.len());
    buf[..len].copy_from_slice(&msg[..len]);
    if let Err(e) = plib::sys_dev_write(DEV_UART, &buf[..len]) {
        rprintln!("[UART] dev_write failed: {:?}", e);
    }
}

extern "C" fn ioctl_echo_body(_r0: u32) -> ! {
    plib::sys_dev_open(DEV_UART).expect("dev_open");
    dev_write(b"READY\n");

    let mut parser = LineParser::new();

    loop {
        if let Err(e) = plib::sys_event_wait(UART_EVENT) {
            rprintln!("[PART] event_wait failed: {:?}", e);
            if let Err(e2) = plib::sys_yield() {
                rprintln!("[PART] yield failed: {:?}", e2);
            }
            continue;
        }
        // Drain → clear → drain again (no IRQ ack needed — ISR doesn't mask)
        loop {
            drain_ring(&mut parser);
            if let Err(e) = plib::sys_event_clear(UART_EVENT) {
                rprintln!("[PART] event_clear failed: {:?}", e);
            }
            if !drain_ring(&mut parser) { break; }
        }
    }
}

kernel::partition_trampoline!(ioctl_partition => ioctl_echo_body);

/// Parse ASCII decimal digits from a byte slice into u32.
fn parse_u32(s: &[u8]) -> u32 {
    let mut val: u32 = 0;
    for &b in s {
        if b >= b'0' && b <= b'9' {
            val = val.saturating_mul(10).saturating_add((b - b'0') as u32);
        } else {
            break;
        }
    }
    val
}

fn init_usart3(baud: u32) {
    unsafe {
        // Enable GPIOD + USART3 clocks
        let ahb1 = core::ptr::read_volatile(RCC_AHB1ENR);
        core::ptr::write_volatile(RCC_AHB1ENR, ahb1 | (1 << 3)); // GPIODEN
        let apb1 = core::ptr::read_volatile(RCC_APB1ENR);
        core::ptr::write_volatile(RCC_APB1ENR, apb1 | (1 << 18)); // USART3EN
        // PD8 = AF7 (TX), PD9 = AF7 (RX)
        let moder = core::ptr::read_volatile(GPIOD_MODER);
        core::ptr::write_volatile(GPIOD_MODER,
            (moder & !((3 << 16) | (3 << 18))) | (2 << 16) | (2 << 18));
        let afrh = core::ptr::read_volatile(GPIOD_AFRH);
        core::ptr::write_volatile(GPIOD_AFRH,
            (afrh & !((0xF << 0) | (0xF << 4))) | (7 << 0) | (7 << 4));
        // Configure USART3: BRR, then enable UE+TE+RE+RXNEIE
        core::ptr::write_volatile(USART3_BRR, HSI_HZ / baud);
        core::ptr::write_volatile(USART3_CR1,
            (1 << 13) | (1 << 3) | (1 << 2) | (1 << 5)); // UE|TE|RE|RXNEIE
    }
}

#[entry]
fn main() -> ! {
    rprintln!("\n=== Virtual Device UART IOCTL + CLOSE Demo ===");
    rprintln!("USART3 on ST-LINK VCP (PD8/PD9)");
    rprintln!("Send 'BAUD:9600\\n' for IOCTL, 'CLOSE\\n' for close/reopen cycle\n");

    let mut p = cortex_m::Peripherals::take().unwrap();
    init_usart3(115_200);

    let mut sched = ScheduleTable::<{ IoctlCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; 1] = [
        PartitionSpec::new(ioctl_partition as PartitionEntry, 0),
    ];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Register USART3 virtual device
    kernel::state::with_kernel_mut::<IoctlCfg, _, _>(|k| {
        unsafe {
            let dev: &'static mut dyn VirtualDevice =
                &mut *core::ptr::addr_of_mut!(UART_DEV);
            k.registry.add(dev).expect("register UART dev");
        }
        Ok::<(), ()>(())
    }).expect("registry");

    enable_bound_irqs(&mut p.NVIC, IoctlCfg::IRQ_DEFAULT_PRIORITY).expect("irqs");
    rprintln!("[INIT] USART3 at 115200, device registered. Booting...\n");
    match boot(p).expect("boot") {}
}
