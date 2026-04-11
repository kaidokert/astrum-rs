//! Watchdog Timer — RTOS Partition demo (SAME51 Curiosity Nano)
//!
//! P0 owns the WDT and feeds it every loop iteration. RTT harness logs
//! feed count every second. If P0 faults or stalls, the WDT resets the MCU.
//!
//! Demonstrates that a kernel partition can safely own a watchdog under
//! MPU enforcement — WDT registers are in the partition's peripheral grant.
//!
//! Build: cd same51_curiosity && cargo build --example wdt_partition \
//!            --features kernel-mpu --release
//! Flash: probe-rs download --chip ATSAME51J20A --probe 03eb:2175 <elf>

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use atsamd_hal as hal;
use cortex_m_rt::{entry, exception};
use hal::clock::GenericClockController;
use embedded_hal_02::watchdog::{Watchdog as WdtTrait, WatchdogEnable as _};
use hal::pac;
use hal::watchdog::{Watchdog, WatchdogTimeout};
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

const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

// WDT peripheral: 0x40002000, 32 bytes is enough (registers end at ~0x0C).
const WDT_BASE: u32 = 0x4000_2000;
const WDT_SIZE: u32 = 0x20; // 32 bytes, power-of-2

static FEED_COUNT: AtomicU32 = AtomicU32::new(0);

static mut WDT_HANDLE: Option<Watchdog> = None;

kernel::kernel_config!(WdtCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}

kernel::bind_interrupts!(WdtCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(WdtCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let feeds = FEED_COUNT.load(Ordering::Relaxed);
        rprintln!("[{:5}ms] WDT feeds={}", tick, feeds);
        if tick >= 5000 && feeds > 0 {
            rprintln!("SUCCESS: WDT partition running under MPU! feeds={}", feeds);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — WDT owner: feed the watchdog every iteration
// ---------------------------------------------------------------------------
extern "C" fn wdt_body(_r0: u32) -> ! {
    let wdt = unsafe { (*addr_of_mut!(WDT_HANDLE)).as_mut().unwrap() };

    loop {
        wdt.feed();
        FEED_COUNT.fetch_add(1, Ordering::Relaxed);
        // Yield to let other partitions run; WDT timeout (~2s) is much
        // longer than the major frame, so yielding is safe.
        plib::sys_yield().ok();
    }
}

// P1: dummy.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(wdt_main => wdt_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

#[entry]
fn main() -> ! {
    rprintln!("=== WDT Partition Demo — SAME51 Curiosity Nano ===");

    let pac::Peripherals {
        gclk,
        mut mclk,
        mut osc32kctrl,
        mut oscctrl,
        mut nvmctrl,
        wdt,
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

    // Start WDT in main (privileged) with ~2s timeout.
    let mut wdt_dev = Watchdog::new(wdt);
    wdt_dev.start(WatchdogTimeout::Cycles2K as u8);
    rprintln!("[INIT] WDT started: ~2s timeout (Cycles2K)");

    unsafe { *addr_of_mut!(WDT_HANDLE) = Some(wdt_dev); }

    // Kernel setup.
    let mut sched = ScheduleTable::<{ WdtCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // P0: WDT peripheral grant.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, wdt_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(WDT_BASE, WDT_SIZE, 0),
        ])
        .expect("periph0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<WdtCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] P0 owns WDT (0x{:08X}/{}B). Booting with MPU...\n", WDT_BASE, WDT_SIZE);
    match boot(p).expect("boot") {}
}
