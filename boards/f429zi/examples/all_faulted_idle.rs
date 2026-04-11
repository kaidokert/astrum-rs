//! All-Faulted Safe Idle — Kernel survives when every partition dies
//!
//! 4 partitions, all fault immediately by writing to the kernel guard region.
//! The kernel must survive: tick handler keeps running, no HardFault, no hang.
//! Proves the scheduler handles the case where no partition is runnable.
//!
//! Success: all 4 partitions Faulted, tick handler still alive after 3 seconds.
//!
//! Build: cd f429zi && cargo build --example all_faulted_idle --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use kernel::{
    mpu, partition::PartitionState,
    scheduler::{ScheduleEntry, ScheduleTable},
    PartitionEntry, PartitionSpec,
    {DebugEnabled, MsgMinimal, Partitions4, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

kernel::kernel_config!(
    IdleCfg<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

static FAULT_COUNT: AtomicU32 = AtomicU32::new(0);
static TICK_ALIVE: AtomicU32 = AtomicU32::new(0);

/// Guard region — all partitions write here to fault.
const GUARD_BASE: u32 = 0x2000_8000;
const GUARD_SIZE: u32 = 32 * 1024;

kernel::define_kernel!(IdleCfg, |tick, k| {
    TICK_ALIVE.store(tick, Ordering::Release);

    let mut faulted = 0u32;
    for pid in 0..4u32 {
        if k.pcb(pid as usize).map(|p| p.state()) == Some(PartitionState::Faulted) {
            faulted += 1;
        }
    }
    let prev = FAULT_COUNT.load(Ordering::Relaxed);
    if faulted > prev {
        FAULT_COUNT.store(faulted, Ordering::Relaxed);
        rprintln!("[{:5}ms] Faulted: {}/4", tick, faulted);
    }

    if tick % 1000 == 0 && tick > 0 {
        rprintln!("[{:5}ms] IDLE — all {} partitions faulted, kernel alive", tick, faulted);
    }

    if tick >= 3000 && faulted == 4 {
        rprintln!("SUCCESS: all-faulted safe idle — kernel survived with 0 runnable partitions");
    }
});

// All 4 partitions: fault immediately by writing to guard region.
const _: PartitionEntry = p0;
extern "C" fn p0() -> ! {
    unsafe { core::ptr::write_volatile((GUARD_BASE + 0x000) as *mut u32, 0xDEAD_0000); }
    loop { asm::wfi(); }
}
const _: PartitionEntry = p1;
extern "C" fn p1() -> ! {
    unsafe { core::ptr::write_volatile((GUARD_BASE + 0x100) as *mut u32, 0xDEAD_0001); }
    loop { asm::wfi(); }
}
const _: PartitionEntry = p2;
extern "C" fn p2() -> ! {
    unsafe { core::ptr::write_volatile((GUARD_BASE + 0x200) as *mut u32, 0xDEAD_0002); }
    loop { asm::wfi(); }
}
const _: PartitionEntry = p3;
extern "C" fn p3() -> ! {
    unsafe { core::ptr::write_volatile((GUARD_BASE + 0x300) as *mut u32, 0xDEAD_0003); }
    loop { asm::wfi(); }
}

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb(); asm::isb();

    // R0: Flash 2 MB — RO+X
    let sf = mpu::encode_size(2 * 1024 * 1024).expect("flash");
    let rbar = mpu::build_rbar(0x0800_0000, 0).expect("flash rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    // R1: RAM 256 KB — RW+XN
    let sf = mpu::encode_size(256 * 1024).expect("ram");
    let rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    // R2: Guard — priv-only (partitions can't access)
    let sf = mpu::encode_size(GUARD_SIZE).expect("guard");
    let rbar = mpu::build_rbar(GUARD_BASE, 2).expect("guard rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb(); asm::isb();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== All-Faulted Safe Idle Test ===");
    rprintln!("4 partitions fault immediately. Kernel must survive in idle.");

    p.SCB.enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    configure_static_mpu(&p.MPU);

    let mut sched = ScheduleTable::<{ IdleCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("P1");
    sched.add(ScheduleEntry::new(2, 2)).expect("P2");
    sched.add(ScheduleEntry::new(3, 2)).expect("P3");
    sched.add_system_window(1).expect("SW");

    let parts: [PartitionSpec; 4] = [
        PartitionSpec::new(p0 as PartitionEntry, 0),
        PartitionSpec::new(p1 as PartitionEntry, 0),
        PartitionSpec::new(p2 as PartitionEntry, 0),
        PartitionSpec::new(p3 as PartitionEntry, 0),
    ];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting — all partitions will fault immediately\n");
    match boot(p).expect("boot") {}
}
