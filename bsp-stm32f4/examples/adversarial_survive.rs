//! Hardware adversarial survival demo for STM32F429ZI (Nucleo-144).
//!
//! P0 increments a counter each tick. P1 writes to a kernel-guarded
//! MPU region after ~5 iterations, triggering MemManage. RTT output
//! shows P0 continuing after P1 is faulted.
//!
//! MPU: R0=Flash 2MB RO+X, R1=RAM 256KB RW+XN, R2=Guard 32KB priv-only.
//! P1 writes to 0x2003_F000 inside R2, triggering DACCVIOL.
//!
//! Build:  cargo build -p bsp-stm32f4 --target thumbv7em-none-eabihf --features log-rtt --example adversarial_survive
//! Flash:  probe-rs run --chip STM32F429ZITx target/thumbv7em-none-eabihf/debug/examples/adversarial_survive
//! RTT:    probe-rs attach --chip STM32F429ZITx --protocol swd

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::asm;
#[allow(unused_imports)]
use cortex_m_rt::exception; // needed by define_unified_harness! expansion
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    klog, mpu, partition::PartitionState, scheduler::ScheduleTable, DebugEnabled, MsgMinimal,
    PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        // IMPORTANT: Build with --release. Debug builds spin in PendSV.
        mpu_enforce = true;
    }
);

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
// TODO: 0x2003_F000 is in reserved space beyond 192KB SRAM defined in memory.x;
// MPU still traps the access but it is not strictly active kernel memory.
// A higher-quality approach would use an extern linker symbol (e.g. __kernel_state_start)
// to target an actual kernel data structure rather than a hardcoded address.
const KERNEL_ADDR: u32 = 0x2003_F000; // inside guard region
const P1_DELAY_ITERS: u32 = 5;

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
    // Write to kernel guard region — triggers MemManage (DACCVIOL).
    // SAFETY: intentional invalid write. The MPU denies unprivileged
    // access at KERNEL_ADDR, so this traps before any memory is modified.
    unsafe {
        core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_BEEF);
    }
    loop {
        cortex_m::asm::nop();
    }
}

static FAULT_TICK: AtomicU32 = AtomicU32::new(0);
const SURVIVE_TICKS: u32 = 10;
const TIMEOUT_TICKS: u32 = 200;
const REPORT_INTERVAL: u32 = 10;

kernel::define_unified_harness!(TestConfig, |tick, k| {
    let p1_state = k.pcb(1).map(|pcb| pcb.state());

    // Periodic RTT status line so the user can observe P0 surviving.
    if tick.is_multiple_of(REPORT_INTERVAL) {
        let p0_count = P0_COUNTER.load(Ordering::Relaxed);
        klog!(
            "[t{}] P0_RUNS={} P1={:?}",
            tick,
            p0_count,
            p1_state.unwrap_or(PartitionState::Ready)
        );
    }

    if p1_state == Some(PartitionState::Faulted) {
        let ft = FAULT_TICK.load(Ordering::Relaxed);
        if ft == 0 {
            FAULT_TICK.store(tick, Ordering::Relaxed);
        } else if tick >= ft + SURVIVE_TICKS {
            let p0_count = P0_COUNTER.load(Ordering::Relaxed);
            let p0_state = k.pcb(0).map(|pcb| pcb.state());
            if p0_count > 0 && p0_state != Some(PartitionState::Faulted) {
                klog!(
                    "adversarial_survive: PASS (p0_count={}, fault_tick={}, now={})",
                    p0_count,
                    ft,
                    tick
                );
                kernel::kexit!(success);
            } else {
                klog!(
                    "adversarial_survive: FAIL (p0_count={}, p0_state={:?})",
                    p0_count,
                    p0_state
                );
                kernel::kexit!(failure);
            }
        }
    }

    if tick >= TIMEOUT_TICKS {
        klog!(
            "adversarial_survive: FAIL (timeout tick={}, p1_state={:?})",
            tick,
            p1_state
        );
        kernel::kexit!(failure);
    }
});

/// R0=Flash 2MB RO+X, R1=RAM 256KB RW+XN, R2=Guard 32KB priv-only.
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    // TODO: reviewer false positive — SAFETY comment is present above.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash — code read + execute for unprivileged.
    // TODO(panic-free): these .expect() calls are compile-time-knowable invariants
    // (power-of-two sizes, aligned bases) but could use const-eval or graceful errors.
    let flash_sf = mpu::encode_size(2 * 1024 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0800_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full read-write for unprivileged, XN.
    // TODO(panic-free): same as above — invariant sizes/addresses.
    let ram_sf = mpu::encode_size(256 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard — priv-only, overrides R1.
    // TODO(panic-free): same as above — invariant sizes/addresses.
    let guard_sf = mpu::encode_size(32 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2003_8000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    // SAFETY: enabling the MPU after region configuration is complete;
    // single-core, no concurrent access to MPU registers.
    // TODO: reviewer false positive — SAFETY comment is present above.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // TODO(panic-free): Peripherals::take() returns None only if called twice;
    // single-call guarantee is structural but could use a graceful error path.
    let mut p = cortex_m::Peripherals::take().expect("peripherals");

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard no-access.
    configure_static_mpu(&p.MPU);

    // TODO(panic-free): round_robin() fails only on zero-count args; these
    // are compile-time-knowable invariants but could use const-eval.
    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 2).expect("round_robin");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    // TODO(panic-free): init_kernel() fails on mis-sized partition tables;
    // structural guarantee from TestConfig::N but could propagate error.
    let k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(k);

    // Log after init_kernel so RTT is initialized (init_rtt is called inside init_kernel).
    klog!("adversarial_survive: start");

    // TODO(panic-free): boot() returns Err only on BootError variants;
    // could propagate to a kernel panic handler instead of .expect().
    match boot(p).expect("boot") {}
}
