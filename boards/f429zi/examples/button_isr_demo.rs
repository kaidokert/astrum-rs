//! Button ISR Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates the split-ISR (top-half / bottom-half) driver pattern using
//! the kernel's VirtualDevice + DeviceWaitQueue subsystem:
//!
//!   Top-half    (EXTI15_10 ISR):     clears EXTI13 pending bit, increments BUTTON_EVENTS
//!   Bottom-half (tick hook, 1ms):    detects BUTTON_EVENTS > 0, wakes blocked partition
//!   Partition P0 (SYS_DEV_READ_TIMED): blocks until woken, increments PRESS_COUNT
//!
//! Hardware (NUCLEO-F429ZI):
//!   User button: PC13 (active-low, on-board pull-up; press = falling edge)
//!   EXTI source: EXTI13 → EXTI15_10 IRQ (interrupt #40)
//!
//! Note on peripheral access under kernel-mpu:
//!   The 4 GB background deny-all MPU region (AP=0b000) blocks peripheral
//!   access from ALL privilege levels — even handler mode.  Only the SCS
//!   (0xE000_0000, PPB) is always accessible.  To access GPIO peripherals
//!   under kernel-mpu, a dynamic window must be added to the DynamicStrategy
//!   (R5-R7).  This example focuses on the split-ISR kernel path and uses
//!   RTT output as the observable success signal.
//!
//! Build: cd f429zi && cargo build --example button_isr_demo --features kernel-irq-mpu

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use stm32f4::stm32f429::Peripherals;
use plib::DeviceId;
use kernel::{PartitionId, PartitionSpec,
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    virtual_device::{DeviceError, DeviceRegistry, VirtualDevice},
    {Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::{rprintln, rtt_init_print};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;

// VirtualDevice ID for the button
const BUTTON_DEVICE_ID: DeviceId = DeviceId::new(0);

// ISR → kernel communication: ISR increments, VirtualButtonDevice.read() drains
static BUTTON_EVENTS: AtomicU32 = AtomicU32::new(0);

// Cumulative press count for RTT logging (incremented by partition)
static PRESS_COUNT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// VirtualButtonDevice: maps BUTTON_EVENTS atomic into the VirtualDevice API.
//
// read()  — atomically drains BUTTON_EVENTS; returns Ok(1) if any presses
//           pending (triggers immediate return from SYS_DEV_READ_TIMED),
//           Ok(0) if none (triggers blocking).
// ---------------------------------------------------------------------------
struct VirtualButtonDevice {
    id: u8,
    open: bool, // partition 0 open flag
}

impl VirtualButtonDevice {
    const fn new(id: u8) -> Self {
        Self { id, open: false }
    }
}

impl VirtualDevice for VirtualButtonDevice {
    fn device_id(&self) -> u8 {
        self.id
    }

    fn open(&mut self, pid: PartitionId) -> Result<(), DeviceError> {
        if pid.as_raw() != 0 {
            return Err(DeviceError::InvalidPartition);
        }
        if self.open {
            return Err(DeviceError::AlreadyOpen);
        }
        self.open = true;
        Ok(())
    }

    fn close(&mut self, pid: PartitionId) -> Result<(), DeviceError> {
        if pid.as_raw() != 0 || !self.open {
            return Err(DeviceError::NotOpen);
        }
        self.open = false;
        Ok(())
    }

    fn read(&mut self, pid: PartitionId, buf: &mut [u8]) -> Result<usize, DeviceError> {
        if pid.as_raw() != 0 || !self.open {
            return Err(DeviceError::NotOpen);
        }
        // Atomically drain the ISR-incremented counter.
        // Acquire ordering ensures we see all stores from the ISR.
        let count = BUTTON_EVENTS.swap(0, Ordering::Acquire);
        if count > 0 {
            buf[0] = count.min(255) as u8;
            Ok(1) // n>0 → SYS_DEV_READ_TIMED returns n immediately (no block)
        } else {
            Ok(0) // n==0 → SYS_DEV_READ_TIMED blocks the partition
        }
    }

    fn write(&mut self, _pid: PartitionId, _data: &[u8]) -> Result<usize, DeviceError> {
        Err(DeviceError::PermissionDenied)
    }

    fn ioctl(&mut self, _pid: PartitionId, _cmd: u32, _arg: u32) -> Result<u32, DeviceError> {
        Err(DeviceError::NotFound)
    }
}

// Static button device — must outlive the registry ('static).
// SAFETY: accessed exclusively through the DeviceRegistry (single-entry
// registration prevents aliasing), and only from SVC/tick handler contexts
// (serialized by the kernel's critical section).
static mut BUTTON_DEVICE: VirtualButtonDevice = VirtualButtonDevice::new(BUTTON_DEVICE_ID.as_raw());

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------
kernel::kernel_config!(ButtonConfig[AlignedStack2K]<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
});

// ---------------------------------------------------------------------------
// Harness: SysTick hook acts as the bottom-half.
//
// Every 1 ms tick: if BUTTON_EVENTS > 0, wake any partition blocked in
// SYS_DEV_READ_TIMED via dev_wait_queue.
// ---------------------------------------------------------------------------
kernel::define_kernel!(ButtonConfig, |tick, k| {
    // Bottom-half: check for pending button events, wake blocked partition.
    // BUTTON_EVENTS is set by the EXTI15_10 ISR top-half.
    let pending = BUTTON_EVENTS.load(Ordering::Acquire);
    if pending > 0 {
        // Wake the oldest partition blocked on SYS_DEV_READ_TIMED.
        // NLL: dev_wait_queue borrow released before partitions_mut() borrow.
        if let Some(woken) = k.dev_wait_queue.wake_one_reader() {
            kernel::svc::try_transition(
                k.partitions_mut(),
                woken,
                kernel::partition::PartitionState::Ready,
            );
        }
    }

    if tick % 1000 == 0 {
        let presses = PRESS_COUNT.load(Ordering::Acquire);
        rprintln!("[{:5}ms] button_presses={}", tick, presses);
        if presses > 0 {
            rprintln!("✓ SUCCESS: Button ISR demo working! presses={}", presses);
        }
    }
});

// ---------------------------------------------------------------------------
// Partition P0: button handler
//
// Opens the VirtualButtonDevice then loops on SYS_DEV_READ_TIMED (blocks
// until the tick hook's bottom-half wakes it).  Increments PRESS_COUNT.
// ---------------------------------------------------------------------------
extern "C" fn button_handler() -> ! {
    // Open the button VirtualDevice.
    if plib::sys_dev_open(BUTTON_DEVICE_ID).is_err() {
        // Open failed — yield-spin (shouldn't happen)
        loop {
            plib::sys_yield().ok();
        }
    }

    loop {
        let mut buf = [0u8; 1];

        // Block until a button press or timeout (10 000 ticks = 10 s).
        // SYS_DEV_READ_TIMED semantics:
        //   - dev.read() returns n>0  → return n immediately (no block)
        //   - dev.read() returns n==0 → block partition, return 0 on wake
        //   Partition retries until it gets n>0.
        loop {
            match plib::sys_dev_read_timed(BUTTON_DEVICE_ID, &mut buf, 10_000) {
                Err(_) => continue,      // error — retry
                Ok(n) if n > 0 => break, // data available
                Ok(_) => {}              // blocked-then-woken (or timed out), retry
            }
        }

        let count = buf[0] as u32;
        PRESS_COUNT.fetch_add(count, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// EXTI15_10 ISR — top-half
//
// Overrides the DefaultHandler from device.x via #[no_mangle].
// Minimal work: clear the EXTI13 pending bit, increment BUTTON_EVENTS.
// ---------------------------------------------------------------------------
#[unsafe(no_mangle)]
extern "C" fn EXTI15_10() {
    // SAFETY: Peripherals::steal() is the standard ISR pattern — the singleton
    // ownership model doesn't work across the ISR boundary.
    // EXTI_PR is write-1-to-clear: writing only bit 13 leaves other lines unchanged.
    let dp = unsafe { Peripherals::steal() };
    if dp.EXTI.pr.read().pr13().bit_is_set() {
        dp.EXTI.pr.write(|w| w.pr13().set_bit());
        // Signal button press.  Release ordering pairs with the Acquire load
        // in the tick hook bottom-half, ensuring the press is visible.
        BUTTON_EVENTS.fetch_add(1, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// Hardware initialisation (called from main before boot())
// ---------------------------------------------------------------------------

/// Configure PC13 button → EXTI13 falling-edge → NVIC IRQ 40.
fn button_exti_init(dp: &Peripherals) {
    // Enable GPIOC (AHB1ENR) and SYSCFG (APB2ENR) clocks.
    dp.RCC.ahb1enr.modify(|_, w| w.gpiocen().set_bit());
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
    for _ in 0..8 {
        core::hint::spin_loop();
    }

    // PC13 reset value is already input (MODER = 0); no GPIOC config needed.

    // SYSCFG_EXTICR4: route PC (0b0010) to EXTI13.
    // No named PC variant in PAC — use bits() directly.
    dp.SYSCFG.exticr4.modify(|_, w| unsafe { w.exti13().bits(0b0010) });

    // EXTI: unmask line 13 and select falling-edge trigger.
    dp.EXTI.imr.modify(|_, w| w.mr13().set_bit());
    dp.EXTI.ftsr.modify(|_, w| w.tr13().set_bit());

    // NVIC: set priority then enable EXTI15_10 (IRQ 40).
    // IPR[40]: upper byte of word at 0xE000_E428 (4 IRQs per word, byte 0 = IRQ40).
    // Priority 0x80 sits between SVCall (0x00, highest) and SysTick (0xFE, near-lowest).
    // SCS (PPB) is always accessible; these are Cortex-M core peripheral writes.
    unsafe {
        let ipr40 = (0xE000_E400u32 + 40) as *mut u8;
        core::ptr::write_volatile(ipr40, 0x80);
        // ISER1 (IRQs 32–63): bit (40-32)=8.
        let iser1 = 0xE000_E104u32 as *mut u32;
        core::ptr::write_volatile(iser1, 1 << 8);
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("\n=== Button ISR Demo — STM32F429ZI ===");
    rprintln!("Press User button (PC13) — each press increments PRESS_COUNT via split-ISR path");

    let dp = Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    button_exti_init(&dp);
    rprintln!("[INIT] EXTI13 configured (IRQ40 enabled, priority 0x80)");

    // Schedule: P0 (2 ticks) + SystemWindow (2 ticks)
    // SystemWindow satisfies the dynamic-mpu kernel constraint (runs UART bottom-half).
    let mut sched = ScheduleTable::<{ ButtonConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(2).expect("sched SW");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(button_handler as kernel::PartitionEntry, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("[INIT] Kernel created, button device id=0 registered");

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}
