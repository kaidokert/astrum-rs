//! Fault Survival Test — P0 survives after P1 triggers MemManage fault
//!
//! Two partitions under static MPU. P0 increments a counter each loop.
//! P1 writes to a kernel guard region after a brief delay, triggering
//! MemManage (DACCVIOL). The kernel marks P1 as Faulted, and P0 continues
//! running. Success: P0 count > 100 while P1 is Faulted.
//!
//! MPU layout (static, applied once before boot):
//!   R0: Flash (0x0800_0000, 2 MB) — RO+X for unprivileged code
//!   R1: RAM   (0x2000_0000, 256 KB) — RW+XN for unprivileged data
//!   R2: Guard (0x2003_0000, 64 KB) — no-access, overrides R1
//!
//! P1 writes to 0x2003_8000 (inside guard region) → DACCVIOL fault.
//!
//! Build: cd f429zi && cargo build --example fault_survive_test --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use kernel::{
    mpu, partition::PartitionState, scheduler::ScheduleTable,
    PartitionEntry, PartitionSpec,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

// Kernel configuration — MPU enforcement off (we configure MPU manually)
kernel::kernel_config!(
    FaultCfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Shared atomics
static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static FAULT_TICK: AtomicU32 = AtomicU32::new(0);

/// Address inside the guard region — writing here triggers DACCVIOL.
const GUARD_ADDR: u32 = 0x2003_8000;

/// Ticks P1 burns before the illegal write.
const P1_DELAY_ITERS: u32 = 50;

/// Ticks P0 must survive after P1 faults.
const SURVIVE_TICKS: u32 = 10;

/// Hard timeout.
const TIMEOUT_TICKS: u32 = 500;

kernel::define_kernel!(FaultCfg, |tick, k| {
    if tick % 100 == 0 {
        let p0_count = P0_COUNTER.load(Ordering::Acquire);
        let p0_state = k.pcb(0).map(|pcb| pcb.state());
        let p1_state = k.pcb(1).map(|pcb| pcb.state());
        rprintln!(
            "[{:5}ms] P0: count={} state={:?} | P1: state={:?}",
            tick, p0_count, p0_state, p1_state
        );
    }

    let p1_state = k.pcb(1).map(|pcb| pcb.state());
    if p1_state == Some(PartitionState::Faulted) {
        let ft = FAULT_TICK.load(Ordering::Relaxed);
        if ft == 0 {
            FAULT_TICK.store(tick, Ordering::Relaxed);
            rprintln!("[{:5}ms] P1 FAULTED — monitoring P0 survival...", tick);
        } else if tick >= ft + SURVIVE_TICKS {
            let p0_count = P0_COUNTER.load(Ordering::Acquire);
            let p0_state = k.pcb(0).map(|pcb| pcb.state());
            if p0_count > 100 && p0_state != Some(PartitionState::Faulted) {
                rprintln!(
                    "SUCCESS: fault survival working! P0 survived {} ticks, count={}, P1=Faulted",
                    tick - ft, p0_count
                );
            }
        }
    }

    if tick >= TIMEOUT_TICKS {
        rprintln!("FAIL: timeout at tick={}, P1 state={:?}", tick, p1_state);
    }
});

// Partition entry points
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    for _ in 0..P1_DELAY_ITERS {
        cortex_m::asm::nop();
    }
    // Write to kernel guard region → triggers MemManage (DACCVIOL).
    unsafe {
        core::ptr::write_volatile(GUARD_ADDR as *mut u32, 0xDEAD_BEEF);
    }
    // Should never reach here.
    loop { cortex_m::asm::wfi(); }
}

// Static MPU configuration
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash 2 MB (0x0800_0000) — read-only + execute for unprivileged
    let flash_sf = mpu::encode_size(2 * 1024 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0800_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM 256 KB (0x2000_0000) — full read-write for unprivileged, XN
    let ram_sf = mpu::encode_size(256 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard 64 KB (0x2003_0000) — no access for unprivileged, overrides R1
    let guard_sf = mpu::encode_size(64 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2003_0000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Fault Survival Test — STM32F429ZI ===");
    rprintln!("P0: counter loop | P1: writes to guard region after {} iters", P1_DELAY_ITERS);
    rprintln!("Guard region: 0x{:08X} (64 KB, no unprivileged access)\n", 0x2003_0000u32);

    // Enable MemManage exception before configuring MPU
    p.SCB.enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    configure_static_mpu(&p.MPU);

    let sched = ScheduleTable::<{ FaultCfg::SCHED }>::round_robin(2, 2).expect("round_robin");

    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting with MPU enforcement + MemManage handler...\n");
    match boot(p).expect("boot") {}
}
