//! Starvation-counter regression test (e2d5fbf): yield past Waiting
//! increments starvation for Ready partitions.
//!
//! Uses 3 partitions with a round-robin(2,1) schedule covering P0 and P1.
//! P0 yields repeatedly; P1 blocks on a semaphore (enters Waiting).
//! P2 is never scheduled but remains Ready, so it accumulates starvation
//! via `increment_starvation_for_ready_partitions`. The test asserts that
//! P2's starvation count reaches `STARVATION_THRESHOLD`.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception}; // `exception` is used by `define_unified_harness!` macro expansion
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::STARVATION_THRESHOLD;
use kernel::scheduler::ScheduleTable;
use kernel::semaphore::Semaphore;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions3, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    Config<Partitions3, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = Config::N;
// P2 is unscheduled but stays Ready — it accumulates starvation when P0
// yields past Waiting P1 (increment_starvation_for_ready_partitions).
const OBSERVER_PID: usize = 2;
const CHECK_TICK: u32 = (STARVATION_THRESHOLD as u32 + 2) * 2 + 4; // 14
const TIMEOUT_TICK: u32 = 60;

static P0_YIELDS: AtomicU32 = AtomicU32::new(0);
static P1_SEM_ERR: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(Config, |tick, k| {
    if tick >= CHECK_TICK {
        let yields = P0_YIELDS.load(Ordering::Acquire);
        if yields == 0 {
            hprintln!("starvation_counter_test: FAIL p0 never yielded");
            kernel::kexit!(failure);
        }
        if let Some(pcb) = k.partitions().get(OBSERVER_PID) {
            let count = pcb.starvation_count();
            if count >= STARVATION_THRESHOLD {
                hprintln!(
                    "starvation_counter_test: PASS (count={}, threshold={}, yields={})",
                    count,
                    STARVATION_THRESHOLD,
                    yields
                );
                kernel::kexit!(success);
            }
        }
    }
    if P1_SEM_ERR.load(Ordering::Acquire) != 0 {
        hprintln!("starvation_counter_test: FAIL sem_wait error");
        kernel::kexit!(failure);
    }
    if tick >= TIMEOUT_TICK {
        let count = k
            .partitions()
            .get(OBSERVER_PID)
            .map(|p| p.starvation_count())
            .unwrap_or(0);
        hprintln!(
            "starvation_counter_test: FAIL timeout (count={}, need>={})",
            count,
            STARVATION_THRESHOLD
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        if plib::sys_yield().is_ok() {
            P0_YIELDS.fetch_add(1, Ordering::Release);
        }
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    if let Err(e) = plib::sys_sem_wait(plib::SemaphoreId::new(0)) {
        P1_SEM_ERR.store(e.to_u32(), Ordering::Release);
    }
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p2_main;
extern "C" fn p2_main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    hprintln!("starvation_counter_test: start");
    // round_robin(2,1) schedules P0 and P1 only. P2 stays Ready but never
    // runs, so increment_starvation_for_ready_partitions() (which iterates
    // ALL partitions regardless of schedule membership) will count P2.
    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(2, 1).expect("round_robin");
    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
        PartitionSpec::new(p2_main as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("Kernel::create");
    store_kernel(&mut k);
    with_kernel_mut(|k| {
        k.semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
    });
    match boot(p).expect("boot") {}
}
