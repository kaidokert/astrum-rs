//! QEMU integration test: fault-restart-exhaust cycle.
//!
//! Two partitions:
//!   P0 — faults deliberately (writes to MPU guard region), configured with
//!         WarmRestart { max: 2 }.  Uses sys_get_start_condition() to detect
//!         restart and stores the result in an atomic for the SysTick hook.
//!   P1 — healthy monitor, increments a counter each iteration.
//!
//! Expected sequence:
//!   boot → fault → warm restart 1 → fault → warm restart 2 → fault → stay dead
//!
//! Verified: restart count reaches max, partition permanently Faulted,
//! sys_get_start_condition returns WarmRestart after restart, monitor
//! partition runs uninterrupted.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example fault_restart_exhaust_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    mpu,
    partition::{FaultPolicy, MpuRegion, PartitionState},
    scheduler::ScheduleTable,
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// ---------------------------------------------------------------------------
// Shared atomics for partition-to-hook signalling
// ---------------------------------------------------------------------------

/// P0 increments at entry — counts how many times P0 has started.
static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(0);

/// P0 stores its start condition here: 0=NormalBoot, 1=WarmRestart, 0xFF=not-yet.
static P0_START_COND: AtomicU32 = AtomicU32::new(0xFF);

/// P1 increments each loop iteration — liveness counter.
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Kernel address inside the guard region (0x2000_C000 – 0x2000_FFFF).
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Iterations P0 burns before the illegal write (gives SVC time to complete).
const P0_DELAY_ITERS: u32 = 50;

// ---------------------------------------------------------------------------
// Partition entry points
// ---------------------------------------------------------------------------

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    let run = P0_RUN_COUNT.fetch_add(1, Ordering::Release) + 1;

    // Query start condition via SVC and store for the hook to inspect.
    let cond = plib::sys_get_start_condition();
    match cond {
        Ok(plib::StartCondition::NormalBoot) => P0_START_COND.store(0, Ordering::Release),
        Ok(plib::StartCondition::WarmRestart) => P0_START_COND.store(1, Ordering::Release),
        Ok(plib::StartCondition::ColdRestart) => P0_START_COND.store(2, Ordering::Release),
        Err(_) => P0_START_COND.store(0xEE, Ordering::Release),
    }

    // Brief delay then trigger MemManage fault.
    for _ in 0..P0_DELAY_ITERS {
        asm::nop();
    }

    // SAFETY: intentional invalid write to a kernel guard region address.
    // The MPU denies unprivileged access, triggering DACCVIOL.
    unsafe {
        core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0000 + run);
    }
    // Should never reach here.
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Last observed P0 run count — used to detect new restarts.
static LAST_SEEN_RUN: AtomicU32 = AtomicU32::new(0);

/// Tick at which P0 became permanently Faulted.
static DEAD_TICK: AtomicU32 = AtomicU32::new(0);

/// Number of ticks P0 must stay dead before we declare PASS.
const STABLE_DEAD_TICKS: u32 = 15;

/// Hard timeout.
const TIMEOUT_TICKS: u32 = 300;

kernel::define_kernel!(TestConfig, |tick, k| {
    let p0_state = k.pcb(0).map(|pcb| pcb.state());
    let run_count = P0_RUN_COUNT.load(Ordering::Acquire);
    let start_cond = P0_START_COND.load(Ordering::Acquire);
    let last_seen = LAST_SEEN_RUN.load(Ordering::Relaxed);

    // Log each new P0 run as we observe it.
    if run_count > last_seen && start_cond != 0xFF {
        LAST_SEEN_RUN.store(run_count, Ordering::Relaxed);
        let cond_str = match start_cond {
            0 => "NormalBoot",
            1 => "WarmRestart",
            _ => "unknown",
        };
        hprintln!("p0 run #{}: start_condition={}", run_count, cond_str);

        // Verify: first run should be NormalBoot, subsequent runs WarmRestart.
        if run_count == 1 && start_cond != 0 {
            hprintln!("fault_restart_exhaust_test: FAIL first run not NormalBoot");
            kernel::kexit!(failure);
        }
        if run_count > 1 && start_cond != 1 {
            hprintln!(
                "fault_restart_exhaust_test: FAIL run #{} not WarmRestart (got {})",
                run_count,
                start_cond
            );
            kernel::kexit!(failure);
        }
    }

    // Detect when P0 is permanently dead (Faulted after max restarts).
    if p0_state == Some(PartitionState::Faulted) && run_count >= 3 {
        let dt = DEAD_TICK.load(Ordering::Relaxed);
        if dt == 0 {
            DEAD_TICK.store(tick, Ordering::Relaxed);
            hprintln!("p0 permanently faulted at tick {}", tick);
        } else if tick >= dt + STABLE_DEAD_TICKS {
            // P0 has stayed dead long enough. Verify monitor partition.
            let p1_count = P1_COUNTER.load(Ordering::Relaxed);
            let p1_state = k.pcb(1).map(|pcb| pcb.state());
            if p1_count > 0 && p1_state != Some(PartitionState::Faulted) {
                hprintln!(
                    "fault_restart_exhaust_test: PASS (p0 ran {} times, p1_count={}, dead_tick={})",
                    run_count,
                    p1_count,
                    dt
                );
                kernel::kexit!(success);
            } else {
                hprintln!(
                    "fault_restart_exhaust_test: FAIL (p1_count={}, p1_state={:?})",
                    p1_count,
                    p1_state
                );
                kernel::kexit!(failure);
            }
        }
    }

    if tick >= TIMEOUT_TICKS {
        hprintln!(
            "fault_restart_exhaust_test: FAIL timeout (run_count={}, p0_state={:?})",
            run_count,
            p0_state
        );
        kernel::kexit!(failure);
    }
});

// ---------------------------------------------------------------------------
// MPU setup (static, applied once before boot)
// ---------------------------------------------------------------------------

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
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

    // SAFETY: enabling the MPU after region configuration is complete.
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
    hprintln!("fault_restart_exhaust_test: start");

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard no-access.
    configure_static_mpu(&p.MPU);

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 2).expect("round_robin");
    sched.add_system_window(1).expect("system window");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
        PartitionSpec::new(p1_entry as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
    ];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // Configure P0 with WarmRestart { max: 2 } fault policy.
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_fault_policy(FaultPolicy::WarmRestart { max: 2 });

    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
