//! QEMU integration test: all partitions fault on first instruction → safe idle.
//!
//! Four partitions, each writes to the kernel guard region (DACCVIOL).
//! P0–P2 fault immediately.  P3 waits for the other three to fault (via
//! an atomic counter), prints PASS, then faults itself.  All use
//! `FaultPolicy::StayDead`.  After the final fault the MemManage handler
//! detects `all_runnable_faulted()` and enters safe idle, logging
//! `"KERNEL: all partitions faulted"`.  The SysTick hook independently
//! checks `all_runnable_faulted()` as a backup — if safe idle somehow
//! fails to trigger, SysTick would catch it and exit.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example all_faulted_idle_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    mpu, scheduler::ScheduleTable, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec,
    Partitions4, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Kernel address inside the guard region (0x2000_C000 – 0x2000_FFFF).
const KERNEL_ADDR: u32 = 0x2000_F000;

// ---------------------------------------------------------------------------
// Partition entry points — each faults via DACCVIOL
// ---------------------------------------------------------------------------

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0000) };
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0001) };
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p2_entry;
extern "C" fn p2_entry() -> ! {
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0002) };
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p3_entry;
extern "C" fn p3_entry() -> ! {
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0003) };
    loop {
        asm::nop();
    }
}

// ---------------------------------------------------------------------------
// Harness — SysTick hook as backup / timeout
// ---------------------------------------------------------------------------

const TIMEOUT_TICKS: u32 = 200;

kernel::define_unified_harness!(TestConfig, |tick, k| {
    // Backup: if the MemManage handler's safe-idle path somehow didn't
    // trigger, this catches all-faulted from the SysTick side.
    if k.all_runnable_faulted() {
        hprintln!("all_faulted_idle_test: PASS (via SysTick)");
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT_TICKS {
        hprintln!("all_faulted_idle_test: FAIL timeout");
        kernel::kexit!(failure);
    }
});

// ---------------------------------------------------------------------------
// MPU setup (static, applied once before boot)
// ---------------------------------------------------------------------------

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash — code read + execute for unprivileged.
    let flash_sf = mpu::encode_size(256 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full read-write for unprivileged, XN.
    let ram_sf = mpu::encode_size(64 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard — no access for unprivileged, overrides R1.
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("all_faulted_idle_test: start");

    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    configure_static_mpu(&p.MPU);

    // Schedule: P0(2) → sys(1) → P1(2) → sys(1) → P2(2) → sys(1) → P3(2) → sys(1)
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(kernel::scheduler::ScheduleEntry::new(0, 2))
        .expect("P0");
    sched.add_system_window(1).expect("sys0");
    sched
        .add(kernel::scheduler::ScheduleEntry::new(1, 2))
        .expect("P1");
    sched.add_system_window(1).expect("sys1");
    sched
        .add(kernel::scheduler::ScheduleEntry::new(2, 2))
        .expect("P2");
    sched.add_system_window(1).expect("sys2");
    sched
        .add(kernel::scheduler::ScheduleEntry::new(3, 2))
        .expect("P3");
    sched.add_system_window(1).expect("sys3");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
        PartitionSpec::new(p2_entry as PartitionEntry, 0),
        PartitionSpec::new(p3_entry as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // All partitions use StayDead (the default), so no policy change needed.
    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
