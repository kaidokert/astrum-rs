//! QEMU stress test: rapid PendSV context switches under fast SysTick
//! with dynamic-MPU mode enabled.
//!
//! Variant of `pendsv_primask_test` that exercises the dynamic-mode PendSV
//! handler, where `__pendsv_program_mpu` performs significantly more work
//! inside the PRIMASK critical section (MPU disable, write cached base
//! regions R0-R3, compute and write dynamic strategy regions R4-R7, MPU
//! enable).  This validates no TOCTOU race, no MPU faults, and no missed
//! context switches under the longer critical-section window.
//!
//! 1. Two partitions run tight loops, each incrementing an `AtomicU32`.
//! 2. SysTick fires every ~996 cycles (tick_period_us = 83 at 12 MHz).
//! 3. The SysTick hook explicitly pends PendSV on every tick, forcing
//!    context-switch attempts even when the scheduler wouldn't.
//! 4. After several major frames the hook checks that both counters
//!    advanced — proving no missed switches, no MPU faults, no hangs.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,dynamic-mpu --example pendsv_primask_dynamic_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::MpuRegion;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

// Fast SysTick: 12 MHz * 83 µs / 1e6 = 996 cycles per tick.
kernel::kernel_config!(
    Config < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 83;
    }
);

const NUM_PARTITIONS: usize = 2;

/// Minimum tick count before checking (≈5 major frames of 4 ticks each).
const CHECK_TICK: u32 = 20;
/// Hard timeout — if we reach this tick without passing, declare failure.
const TIMEOUT_TICK: u32 = 200;
/// Minimum counter value to accept — ensures multiple context switches.
const MIN_COUNT: u32 = 4;

/// Incremented by partition 0 on every loop iteration.
static P0_COUNT: AtomicU32 = AtomicU32::new(0);
/// Incremented by partition 1 on every loop iteration.
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(Config, |tick, _k| {
    // Pend PendSV on every tick to maximise preemption pressure.
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    if tick >= CHECK_TICK {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        if c0 >= MIN_COUNT && c1 >= MIN_COUNT {
            hprintln!("pendsv_primask_dynamic_test: PASS (p0={}, p1={})", c0, c1);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= TIMEOUT_TICK {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        hprintln!("pendsv_primask_dynamic_test: FAIL (p0={}, p1={})", c0, c1);
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        P0_COUNT.fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        P1_COUNT.fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("pendsv_primask_dynamic_test: Peripherals::take");
    hprintln!("pendsv_primask_dynamic_test: start");

    // Dynamic-mpu requires system windows in the schedule table.
    // 2 partitions × (1 tick + 1 syswin) → major frame = 4 ticks.
    let mut sched = ScheduleTable::<{ Config::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 1)).expect("sched p0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 1)).expect("sched p1");
    sched.add_system_window(1).expect("syswin 1");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
        PartitionSpec::new(p1_main as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
    ];
    let mut k = init_kernel(sched, &parts).expect("pendsv_primask_dynamic_test: Kernel::create");
    store_kernel(&mut k);
    match boot(p).expect("pendsv_primask_dynamic_test: boot") {}
}
