//! Watchdog Timer — RTOS Partition demo (STM32F429ZI)
//!
//! P0 owns the IWDG and feeds it every loop iteration. RTT harness logs
//! feed count every second. If P0 faults or stalls, the IWDG resets the MCU.
//!
//! Build: cd f429zi && cargo build --example wdt_partition \
//!            --features kernel-mpu --no-default-features --release
//! Flash: via GDB + OpenOCD

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    watchdog::IndependentWatchdog,
};
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

// IWDG is at 0x4000_3000, 1KB region.
const IWDG_BASE: u32 = 0x4000_3000;
const IWDG_SIZE: u32 = 0x400;

static FEED_COUNT: AtomicU32 = AtomicU32::new(0);
static mut WDT_HANDLE: Option<IndependentWatchdog> = None;

kernel::kernel_config!(WdtCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

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
// P0 — IWDG owner
// ---------------------------------------------------------------------------
extern "C" fn wdt_body(_r0: u32) -> ! {
    let wdt = unsafe { (*addr_of_mut!(WDT_HANDLE)).as_mut().unwrap() };
    loop {
        wdt.feed();
        FEED_COUNT.fetch_add(1, Ordering::Relaxed);
        plib::sys_yield().ok();
    }
}

extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(wdt_main => wdt_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

#[entry]
fn main() -> ! {
    rprintln!("=== WDT Partition Demo — STM32F429ZI ===");

    let dp = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(stm32f4xx_hal::rcc::Config::hse(8.MHz()).sysclk(168.MHz()));

    // Start IWDG in main (privileged) with ~2s timeout.
    let mut wdt = IndependentWatchdog::new(dp.IWDG);
    wdt.stop_on_debug(&dp.DBGMCU, true); // pause during debug
    wdt.start(2000u32.millis());
    rprintln!("[INIT] IWDG started: ~2s timeout");

    unsafe { *addr_of_mut!(WDT_HANDLE) = Some(wdt); }

    let mut sched = ScheduleTable::<{ WdtCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, wdt_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(IWDG_BASE, IWDG_SIZE, 0),
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

    rprintln!("[INIT] P0 owns IWDG (0x{:08X}/{}B). Booting with MPU...\n", IWDG_BASE, IWDG_SIZE);
    match boot(p).expect("boot") {}
}
