//! All-Faulted + Watchdog Recovery — System recovers from total partition failure
//!
//! P0: feeds IWDG every loop iteration.
//! P1: healthy partition, just yields.
//! After 2 seconds, both partitions are forced to fault (write to ungranted addr).
//! Kernel enters safe idle (bug 42-meerkat). IWDG stops being fed → MCU resets.
//! On reboot, detect watchdog reset via RCC_CSR.IWDGRSTF and report SUCCESS.
//!
//! Proves: partition fault → safe idle → watchdog reset → clean recovery.
//!
//! Build: cd f429zi && cargo build --example wdt_recovery \
//!            --features kernel-mpu --no-default-features --release

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
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

// IWDG registers (PAC-free, for MPU-granted partition access)
const IWDG_BASE: u32 = 0x4000_3000;
const IWDG_SIZE: u32 = 0x400;
const IWDG_KR: *mut u32 = IWDG_BASE as *mut u32;
const IWDG_PR: *mut u32 = (IWDG_BASE + 0x04) as *mut u32;
const IWDG_RLR: *mut u32 = (IWDG_BASE + 0x08) as *mut u32;
const IWDG_SR: *const u32 = (IWDG_BASE + 0x0C) as *const u32;

// RCC_CSR: reset cause flags
const RCC_CSR: *mut u32 = 0x4002_3874 as *mut u32;
const IWDGRSTF: u32 = 1 << 29; // Independent watchdog reset flag
const RMVF: u32 = 1 << 24;     // Remove reset flags

// Counters
static FEED_COUNT: AtomicU32 = AtomicU32::new(0);
static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(RecoveryCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(RecoveryCfg, |tick, _k| {
    TICK_COUNT.store(tick, Ordering::Relaxed);
    if tick % 1000 == 0 {
        let feeds = FEED_COUNT.load(Ordering::Relaxed);
        rprintln!("[{:5}ms] WDT feeds={}", tick, feeds);
    }
});

// ---------------------------------------------------------------------------
// P0 — feeds IWDG, then faults after tick reaches 2000ms
// ---------------------------------------------------------------------------
extern "C" fn feeder_body(_r0: u32) -> ! {
    loop {
        // Feed watchdog (write 0xAAAA to KR)
        unsafe { core::ptr::write_volatile(IWDG_KR, 0xAAAA); }
        FEED_COUNT.fetch_add(1, Ordering::Relaxed);

        // After 2 seconds, stop feeding and fault
        let tick = TICK_COUNT.load(Ordering::Relaxed);
        if tick >= 2000 {
            rprintln!("[P0] Faulting after {}ms — WDT will expire!", tick);
            // Write to ungranted address → MemManage fault
            unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD); }
        }

        plib::sys_yield().ok();
    }
}

// ---------------------------------------------------------------------------
// P1 — healthy partition, faults after tick reaches 2000ms
// ---------------------------------------------------------------------------
extern "C" fn healthy_body(_r0: u32) -> ! {
    loop {
        let tick = TICK_COUNT.load(Ordering::Relaxed);
        if tick >= 2000 {
            rprintln!("[P1] Faulting after {}ms", tick);
            unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD); }
        }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(feeder_main => feeder_body);
kernel::partition_trampoline!(healthy_main => healthy_body);

/// Start IWDG via direct register writes (~2s timeout).
/// LSI clock = ~32 kHz. Prescaler /64 → 500 Hz. Reload = 1000 → 2s.
unsafe fn start_iwdg() {
    // Pause IWDG during debug FIRST (DBGMCU_APB1_FZ bit 12)
    let dbg_apb1_fz = 0xE004_2008 as *mut u32;
    let val = core::ptr::read_volatile(dbg_apb1_fz);
    core::ptr::write_volatile(dbg_apb1_fz, val | (1 << 12));

    // Start IWDG (enables LSI clock), then configure prescaler/reload.
    core::ptr::write_volatile(IWDG_KR, 0xCCCC); // start IWDG + enable LSI
    core::ptr::write_volatile(IWDG_KR, 0x5555); // unlock PR/RLR
    core::ptr::write_volatile(IWDG_PR, 4);       // prescaler /64
    core::ptr::write_volatile(IWDG_RLR, 1000);   // reload → ~2s
    // Wait for LSI domain to latch PR/RLR (PVU + RVU clear)
    while core::ptr::read_volatile(IWDG_SR) & 0x3 != 0 {}
    core::ptr::write_volatile(IWDG_KR, 0xAAAA); // initial feed
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    // Check reset cause FIRST, before clearing flags.
    let csr = unsafe { core::ptr::read_volatile(RCC_CSR) };
    let iwdg_reset = csr & IWDGRSTF != 0;

    // Clear reset flags for next boot.
    unsafe { core::ptr::write_volatile(RCC_CSR, csr | RMVF); }

    rprintln!("\n=== All-Faulted + Watchdog Recovery ===");
    rprintln!("RCC_CSR = {:#010x}, IWDGRSTF = {}", csr, iwdg_reset);

    if iwdg_reset {
        // IWDG is still running from previous boot — feed it immediately to
        // stop the reset loop, then re-enable debug freeze.
        unsafe {
            core::ptr::write_volatile(IWDG_KR, 0xAAAA); // feed
            let dbg_apb1_fz = 0xE004_2008 as *mut u32;
            let val = core::ptr::read_volatile(dbg_apb1_fz);
            core::ptr::write_volatile(dbg_apb1_fz, val | (1 << 12));
        }
        rprintln!("SUCCESS: watchdog recovery! System rebooted after all partitions faulted.");
        rprintln!("The IWDG fired because no partition was feeding it after all-faulted idle.");
        // Keep feeding so we don't reset again while RTT drains.
        loop {
            unsafe { core::ptr::write_volatile(IWDG_KR, 0xAAAA); }
            cortex_m::asm::nop();
        }
    }

    rprintln!("First boot — starting IWDG + 2 partitions.");
    rprintln!("Both partitions will fault at t=2s. IWDG should reset MCU ~2s later.\n");

    unsafe { start_iwdg(); }
    rprintln!("[INIT] IWDG started: ~2s timeout, debug-freeze enabled");

    let mut sched = ScheduleTable::<{ RecoveryCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("P1");
    sched.add_system_window(1).expect("SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, feeder_main as kernel::PartitionEntry, data, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(IWDG_BASE, IWDG_SIZE, 0),
        ])
        .expect("periph0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, healthy_main as kernel::PartitionEntry, data, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<RecoveryCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
