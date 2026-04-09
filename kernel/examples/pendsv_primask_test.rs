//! QEMU stress test: rapid PendSV context switches under fast SysTick.
//!
//! Validates the Bug 14-numbat TOCTOU fix (PRIMASK critical section in PendSV):
//!
//! 1. Two partitions run tight loops, each writing its ID to a shared atomic.
//! 2. SysTick fires every ~996 cycles (tick_period_us = 83 at 12 MHz).
//! 3. The SysTick hook explicitly pends PendSV on every tick, forcing
//!    context-switch attempts even when the scheduler wouldn't.
//! 4. On every tick, verifies stack sentinels intact and SP in bounds.
//!    After 10 partition alternations, declares PASS.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example pendsv_primask_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
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

/// PendSV writes this into saved-SP on stack overflow detection.
const STACK_SENTINEL: u32 = kernel::partition_core::SP_SENTINEL_FAULT;
/// Hard timeout — if we reach this tick without passing, declare failure.
const TIMEOUT_TICK: u32 = 200;
const MIN_SWITCHES: u32 = 10;
const NO_PARTITION: u32 = u32::MAX;

/// Each partition stores its own ID here; the hook detects alternation.
static WHO_RAN: AtomicU32 = AtomicU32::new(NO_PARTITION);
static LAST_SEEN: AtomicU32 = AtomicU32::new(NO_PARTITION);
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(Config, |tick, k| {
    // Pend PendSV on every tick to maximise preemption pressure.
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    // Check no partition triggered PendSV's stack-overflow sentinel.
    let sp = k.partition_sp();
    for (i, &v) in sp.iter().enumerate().take(NUM_PARTITIONS) {
        if v == STACK_SENTINEL {
            hprintln!("FAIL: partition {} stack overflow", i);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    // Verify each partition's saved SP is within its stack region.
    for (i, &v) in sp.iter().enumerate().take(NUM_PARTITIONS) {
        if let Some(pcb) = k.pcb(i) {
            if v == 0 {
                continue;
            }
            let base = pcb.stack_base();
            let top = base.wrapping_add(pcb.stack_size());
            if v < base || v > top {
                hprintln!("FAIL: p{} SP {:#010x} out of bounds", i, v);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
    // Count partition alternations.
    let cur = WHO_RAN.load(Ordering::Acquire);
    if cur != NO_PARTITION {
        let prev = LAST_SEEN.swap(cur, Ordering::Relaxed);
        if prev != NO_PARTITION && prev != cur {
            let n = SWITCH_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            if n >= MIN_SWITCHES {
                hprintln!("pendsv_primask_test: PASS ({} switches)", n);
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }
    if tick >= TIMEOUT_TICK {
        hprintln!(
            "FAIL: timeout ({} switches)",
            SWITCH_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        WHO_RAN.store(0, Ordering::Release);
        // TODO: reviewer suggested compiler_fence here; Release ordering on the atomic
        // already prevents reordering, and nop() prevents the loop from being elided.
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        WHO_RAN.store(1, Ordering::Release);
        // TODO: reviewer suggested compiler_fence here; Release ordering on the atomic
        // already prevents reordering, and nop() prevents the loop from being elided.
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals already taken");
    hprintln!("pendsv_primask_test: start");

    // 2 partitions, 1 tick per slot → major frame = 2 ticks (~1992 cycles).
    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1)
        .expect("round-robin schedule for 2 partitions must fit");
    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("kernel create with 2 partitions must succeed");

    match boot(p).expect("boot must succeed after kernel create") {}
}
