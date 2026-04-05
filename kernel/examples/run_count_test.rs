//! QEMU integration test: `sys_get_partition_run_count` returns > 0 after execution.
//!
//! Two worker partitions spin briefly, then call `plib::sys_get_partition_run_count`
//! on themselves and store the result in atomics. The SysTick harness polls the
//! atomics and asserts both run counts are greater than zero.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example run_count_test
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
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled >
);

/// Sentinel meaning "partition has not yet reported its run count".
const NOT_YET: u32 = 0xFFFF_FFFF;

/// Run counts reported by each partition via `sys_get_partition_run_count`.
static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(NOT_YET);
static P1_RUN_COUNT: AtomicU32 = AtomicU32::new(NOT_YET);

/// Hard timeout — declare failure if not passed by this tick.
const TIMEOUT_TICK: u32 = 200;

// --- Worker partition entry points ---

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // Spin briefly so the partition accumulates scheduler run_count.
    for _ in 0..100 {
        asm::nop();
    }
    // Exercise the SVC API to query our own run count.
    match plib::sys_get_partition_run_count(0) {
        Ok(rc) => P0_RUN_COUNT.store(rc, Ordering::Release),
        Err(_) => P0_RUN_COUNT.store(0, Ordering::Release),
    }
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    for _ in 0..100 {
        asm::nop();
    }
    match plib::sys_get_partition_run_count(1) {
        Ok(rc) => P1_RUN_COUNT.store(rc, Ordering::Release),
        Err(_) => P1_RUN_COUNT.store(0, Ordering::Release),
    }
    loop {
        asm::nop();
    }
}

// --- Harness (privileged SysTick context) ---

kernel::define_kernel!(TestConfig, |tick, _k| {
    let rc0 = P0_RUN_COUNT.load(Ordering::Acquire);
    let rc1 = P1_RUN_COUNT.load(Ordering::Acquire);

    // Wait until both partitions have reported.
    if rc0 == NOT_YET || rc1 == NOT_YET {
        if tick >= TIMEOUT_TICK {
            hprintln!(
                "run_count_test: FAIL timeout (rc0={:#x}, rc1={:#x})",
                rc0,
                rc1
            );
            kernel::kexit!(failure);
        }
        return;
    }

    if rc0 > 0 && rc1 > 0 {
        hprintln!("run_count_test: PASS (rc0={}, rc1={})", rc0, rc1);
        kernel::kexit!(success);
    }

    hprintln!("run_count_test: FAIL (rc0={}, rc1={})", rc0, rc1);
    kernel::kexit!(failure);
});

// --- Boot ---

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("run_count_test: start");

    // Schedule: P0(5), sys(1), P1(5), sys(1) — 12-tick major frame.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 5)).expect("add P1");
    sched.add_system_window(1).expect("sys1");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];

    let mut k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
