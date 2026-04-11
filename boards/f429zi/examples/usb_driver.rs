//! USB CDC Option D — Partition IS the USB driver (STM32F429ZI)
//!
//! True Option D: the RTOS partition IS the USB driver. No OTG_FS ISR.
//! The partition runs in thread mode under real MPU enforcement and polls
//! `usb_dev.poll()` directly in its 2 ms scheduler slot.
//!
//! This is architecturally distinct from usb_partition.rs (where the USB HAL
//! lives in the OTG_FS ISR and the partition only handles ring-buffer echo
//! logic). Here the partition owns the full USB stack and does everything:
//! poll, read, echo-write — zero kernel events, zero ring buffers, zero ISR.
//!
//! Architecture:
//!   main() [privileged, pre-MPU]:
//!     PLL → GPIOA.split() → USB::new() → UsbBusAllocator → UsbDevice + SerialPort
//!     Stored in static mut G_USB_DEVICE / G_USB_SERIAL (no Mutex — no ISR race)
//!     NVIC::unmask(OTG_FS) intentionally NOT called — polling-only design
//!     boot() arms MPU and enters scheduler
//!
//!   USB partition (thread mode, MPU-enforced):
//!     Takes USB_DEVICE + USB_SERIAL from globals on first call (lazy-init)
//!     loop { usb_dev.poll(&mut [serial]) → serial.read() → echo → serial.write() }
//!     Runs for its full 2 ms slot; PendSV preempts when the slot expires
//!     No SYS_YIELD — tight poll loop maximises USB responsiveness in-slot
//!
//! MPU region layout (after __boot_mpu_init + wire_boot_peripherals):
//!   R0: deny-all (4 GiB, no-access, XN)      — background
//!   R1: flash code RX (0x0800_0000, 256 KB)   — instruction fetch
//!   R2: SRAM RW       (0x2000_0000, 256 KB)   — data + stack + EP_MEMORY
//!   R3: stack guard   (stack_base, 32 B)       — overflow sentinel
//!   R4: OTG_FS MMIO   (0x5000_0000, 256 KB)   — Device, AP=full, XN
//!   R5: partition RAM (0x2000_0000, 256 KB)   — dynamic slot (same as R2)
//!
//! USB enumeration with polling:
//!   Major frame = 3 ms (2 ms USB slot + 1 ms system window).
//!   USB polls at ~333 Hz — within spec tolerance for FS CDC control transfers
//!   (host retries for ≥50 ms; 3 ms worst-case latency is well inside that).
//!
//! Hardware: STM32F429ZI NUCLEO-144
//!   PA11 = USB_DM, PA12 = USB_DP (USB OTG FS connector CN12)
//!
//! Build:  cd f429zi && cargo build --example usb_driver --features kernel-usb-mpu --no-default-features
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: /dev/ttyACM* echoes "[RTOS-D] " prefix; RTT shows rx_total advancing,
//!         MPU CTRL=0x5, R4=OTG_FS at 0x50000000/256KB, MemManage=0.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]


use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::{rprintln, rtt_init_print};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE, USB_SYSCLK_HZ};
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac,
    prelude::*,
    rcc::Config,
};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// ── Constants ──────────────────────────────────────────────────────────────────

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;

// 168 MHz (HSE 8 MHz + PLL) — USB requires precise 48 MHz from PLL48CLK.


// Data window: full 256 KB SRAM — covers partition stack, EP_MEMORY, USB objects, RTT.

// ── Statistics ─────────────────────────────────────────────────────────────────

static RX_TOTAL: AtomicU32 = AtomicU32::new(0);
static TX_TOTAL: AtomicU32 = AtomicU32::new(0);

// ── USB globals ────────────────────────────────────────────────────────────────
//
// main() fills these before boot(); the USB partition takes them on first call.
// No Mutex<RefCell<>> needed — there is no OTG_FS ISR to race with the partition.
// SAFETY: the only two access points are main() (store, before boot()) and the
// partition body (take, after boot() — sequentially after main()). No aliasing.

static mut G_USB_DEVICE: Option<UsbDevice<'static, UsbBus<USB>>> = None;
static mut G_USB_SERIAL: Option<SerialPort<'static, UsbBus<USB>>> = None;

// ── Kernel configuration ────────────────────────────────────────────────────────

kernel::kernel_config!(UsbMpuConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

// ── SysTick hook ───────────────────────────────────────────────────────────────
//
// No USB event signalling needed — the partition polls USB directly each slot.
// Just log stats every 5 s and dump MPU regions once at startup.

kernel::define_kernel!(UsbMpuConfig, |tick, _k| {
    if tick % 5_000 == 0 && tick > 0 {
        let rx = RX_TOTAL.load(Ordering::Relaxed);
        let tx = TX_TOTAL.load(Ordering::Relaxed);
        rprintln!("[{:6}ms] USB rx_total={} tx_total={}", tick, rx, tx);
        if rx >= 10 {
            rprintln!("✓ SUCCESS: Option D USB CDC under MPU! rx={} tx={}", rx, tx);
        }
    }

    // Dump MPU regions once to confirm OTG_FS wiring.
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU regions (usb_partition_d) --- CTRL=0x{:x}", ctrl);
        for r in 0u32..7 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            rprintln!(
                "  R{}: RBAR=0x{:08x} RASR=0x{:08x} EN={}",
                r, rbar, rasr, rasr & 1
            );
        }
        rprintln!("---");
    }
});

// ── USB partition body ──────────────────────────────────────────────────────────
//
// This partition IS the USB driver (Option D). It takes the USB device and serial
// objects from global statics on first call, then polls in a tight loop for the
// duration of its 2 ms scheduler slot. PendSV preempts it when the slot expires.
//
// Under MPU enforcement:
//   Accessible: SRAM (0x2000_0000/256KB) for stack, statics, EP_MEMORY, RTT buffers
//               OTG_FS (0x5000_0000/256KB) for USB register access by usb_dev.poll()
//   Blocked:    Everything else (RCC, other peripherals, kernel data outside SRAM)

extern "C" fn usb_main_body(_r0: u32) -> ! {
    // Lazy-take USB objects from global statics (set by main() before boot()).
    // Use addr_of_mut! to access static muts through raw pointers — Rust 2024 forbids
    // creating &mut references to named static muts even inside unsafe blocks.
    // SAFETY: no ISR, single-core; these statics are written once (main) then read once
    // (here). get_or_insert equivalent runs at most once and is not re-entrant.
    static mut USB_DEV: Option<UsbDevice<'static, UsbBus<USB>>> = None;
    static mut USB_SER: Option<SerialPort<'static, UsbBus<USB>>> = None;

    let (usb_dev, serial) = unsafe {
        let dev_ptr = core::ptr::addr_of_mut!(USB_DEV);
        let ser_ptr = core::ptr::addr_of_mut!(USB_SER);
        if (*dev_ptr).is_none() {
            *dev_ptr = (*core::ptr::addr_of_mut!(G_USB_DEVICE)).take();
        }
        if (*ser_ptr).is_none() {
            *ser_ptr = (*core::ptr::addr_of_mut!(G_USB_SERIAL)).take();
        }
        ((*dev_ptr).as_mut().unwrap(), (*ser_ptr).as_mut().unwrap())
    };

    loop {
        // poll() drives USB state machine: handles enumeration, control transfers,
        // bulk IN/OUT. Returns true if any class traffic was processed.
        if usb_dev.poll(&mut [serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                if count > 0 {
                    RX_TOTAL.fetch_add(count as u32, Ordering::Relaxed);

                    // Echo with "[RTOS-D] " prefix to distinguish from usb_partition.
                    let sent_prefix = serial.write(b"[RTOS-D] ").unwrap_or(0);
                    let sent_data = serial.write(&buf[..count]).unwrap_or(0);
                    TX_TOTAL.fetch_add((sent_prefix + sent_data) as u32, Ordering::Relaxed);
                }
            }
        }
        // No SYS_YIELD — partition runs its full 2 ms slot then PendSV preempts.
        // Tight poll loop gives maximum USB responsiveness within the scheduler budget.
    }
}
kernel::partition_trampoline!(usb_main => usb_main_body);

// ── main ────────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rprintln!("\n=== USB CDC Option D — Partition IS the USB driver (STM32F429ZI) ===");
    rprintln!("True Option D: USB HAL runs in thread-mode partition under MPU enforcement");
    rprintln!("No OTG_FS ISR. Partition polls USB directly. MPU_ENFORCE=true.");
    // OTG_FS block: covers OTG_FS_GLOBAL/HOST/DEVICE/PWRCLK sub-blocks (256 KB, base-aligned).
    let otg_fs_base = pac::OTG_FS_GLOBAL::PTR as usize as u32;
    rprintln!("OTG_FS peripheral_regions: 0x{:08x}/256KB", otg_fs_base);

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    // Configure clocks: 168 MHz system clock + 48 MHz USB PLL.
    // require_pll48clk() ensures the 48 MHz clock for OTG_FS is available.
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())    // 8 MHz external crystal on NUCLEO-144
            .sysclk(168.MHz())  // 168 MHz max SYSCLK for STM32F429ZI
            .require_pll48clk() // 48 MHz for USB OTG FS (mandatory)
    );

    // Split GPIOA for USB pin configuration (PA11 = DM, PA12 = DP).
    // USB::new() converts pa11/pa12 → alt::Dm/Dp (AF10 push-pull) automatically.
    let gpioa = dp.GPIOA.split(&mut rcc);

    // Initialise USB peripheral. USB::new() enables the OTG_FS AHB2 clock via RCC.
    // Called here in main() before boot() arms the MPU — RCC is fully accessible.
    // The partition never needs to touch RCC (clock already enabled).
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    // Build USB device and serial port objects. Store directly in static mut Option<>
    // — no Mutex needed since no ISR will race with the partition for these objects.
    unsafe {
        G_USB_SERIAL = Some(SerialPort::new(usb_bus));

        G_USB_DEVICE = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(usbd_serial::USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("STM32")
                    .product("F429 RTOS-D USB")
                    .serial_number("RTOSD001")])
                .unwrap()
                .build(),
        );
    }

    // NVIC::unmask(OTG_FS) intentionally NOT called — this is a polling design.
    // The OTG_FS interrupt is left masked; the partition drives enumeration and
    // data transfer by polling usb_dev.poll() directly.
    rprintln!("[INIT] USB initialized — VID:PID=16c0:27dd, product='F429 RTOS-D USB'");
    rprintln!("[INIT] OTG_FS ISR NOT unmasked — partition polls USB directly");

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: USB partition (2 ticks = 2 ms) + system window (1 ms).
    // Major frame = 3 ms; USB polls at ~333 Hz (adequate for FS CDC).
    let mut sched = ScheduleTable::<{ UsbMpuConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sched SW");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(usb_main, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting kernel — USB partition drives OTG_FS directly under MPU\n");
    match boot(p).expect("boot") {}
}
