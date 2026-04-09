//! QEMU integration test: two-thread round-robin partition.
//!
//! One partition configured with `max_threads=2` and `RoundRobin` policy.
//! The main thread creates a second thread via `SYS_THREAD_CREATE`. Both
//! threads increment separate volatile counters. The SysTick hook checks
//! that both counters exceed a minimum threshold (sustained scheduling) and
//! that neither thread starves the other (fairness ratio max/min <= 3).
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example qemu_thread_rr
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
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};
use rtos_traits::thread::SchedulingPolicy;

kernel::kernel_config!(
    TestConfig < Partitions1,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled >
);

static T0_COUNTER: AtomicU32 = AtomicU32::new(0);
static T1_COUNTER: AtomicU32 = AtomicU32::new(0);
const TIMEOUT_TICK: u32 = 300;
const MIN_COUNT: u32 = 5;
const MAX_FAIRNESS_RATIO: u32 = 3;
// Snapshot counters: captured when both threads are first detected running.
// This eliminates head-start bias from thread 0 running before thread 1 is created.
static SNAP_T0: AtomicU32 = AtomicU32::new(0);
static SNAP_T1: AtomicU32 = AtomicU32::new(0);
static SNAP_TAKEN: AtomicU32 = AtomicU32::new(0);

extern "C" fn thread1_entry() -> ! {
    loop {
        T1_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
}

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // Create a second thread via SYS_THREAD_CREATE syscall.
    let ret = rtos_traits::svc!(
        rtos_traits::syscall::SYS_THREAD_CREATE,
        thread1_entry as *const () as u32,
        1u32,
        0u32
    );
    if ret >= 0x8000_0000 {
        hprintln!("qemu_thread_rr: FAIL SYS_THREAD_CREATE returned {:#x}", ret);
        kernel::kexit!(failure);
    }
    loop {
        T0_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
}

kernel::define_kernel!(TestConfig, |tick, _k| {
    let c0 = T0_COUNTER.load(Ordering::Relaxed);
    let c1 = T1_COUNTER.load(Ordering::Relaxed);

    if c0 > 0 && c1 > 0 && SNAP_TAKEN.load(Ordering::Relaxed) == 0 {
        // Both threads are running — capture baseline to eliminate head-start bias.
        SNAP_T0.store(c0, Ordering::Relaxed);
        SNAP_T1.store(c1, Ordering::Relaxed);
        SNAP_TAKEN.store(1, Ordering::Relaxed);
    }

    if SNAP_TAKEN.load(Ordering::Relaxed) != 0 {
        let d0 = c0 - SNAP_T0.load(Ordering::Relaxed);
        let d1 = c1 - SNAP_T1.load(Ordering::Relaxed);

        if d0 > MIN_COUNT && d1 > MIN_COUNT {
            let (max, min) = if d0 >= d1 { (d0, d1) } else { (d1, d0) };
            if max > min * MAX_FAIRNESS_RATIO {
                hprintln!(
                    "qemu_thread_rr: FAIL unfair (d0={}, d1={}, ratio={})",
                    d0,
                    d1,
                    max / min
                );
                kernel::kexit!(failure);
            }
            hprintln!(
                "qemu_thread_rr: PASS (t0={}, t1={}, d0={}, d1={})",
                c0,
                c1,
                d0,
                d1
            );
            kernel::kexit!(success);
        }
    }

    if tick >= TIMEOUT_TICK {
        hprintln!("qemu_thread_rr: FAIL timeout (t0={}, t1={})", c0, c1);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("qemu_thread_rr: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 10)).expect("add P0");
    sched.add_system_window(2).expect("sys");

    let parts: [PartitionSpec; TestConfig::N] = [PartitionSpec::new(p0_entry as PartitionEntry, 0)
        .with_scheduling_policy(SchedulingPolicy::RoundRobin)
        .with_max_threads(2)];

    let mut k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
