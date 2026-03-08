//! Defense-in-depth: verifies SVCall entry clears PRIMASK and BASEPRI.
//!
//! TODO: verification is indirect — we check register state after SVC return in the partition,
//! not strictly on SVCall handler entry. Partition progress (COUNTS) implies PendSV is not
//! blocked, which validates the architectural goal.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    Config < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 83;
    }
);

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = Config::STACK_WORDS;
const CHECK_TICK: u32 = 20;
const TIMEOUT_TICK: u32 = 200;
const MIN_COUNT: u32 = 4;
static COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

kernel::define_unified_harness!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    let c: [u32; 2] =
        core::array::from_fn(|i| COUNTS.get(i).map_or(0, |c| c.load(Ordering::Acquire)));
    if tick >= CHECK_TICK && c.iter().all(|&v| v >= MIN_COUNT) {
        hprintln!("svcall_basepri_clear_test: PASS ({:?})", c);
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= TIMEOUT_TICK {
        hprintln!("svcall_basepri_clear_test: FAIL timeout ({:?})", c);
        debug::exit(debug::EXIT_FAILURE);
    }
});

#[inline(always)]
fn primask_is_active() -> bool {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::primask::read().is_active()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        false
    }
}

#[inline(always)]
fn basepri_value() -> u8 {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::basepri::read()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        0
    }
}

// TODO: unify error reporting — fail_halt vs .expect() in main; low priority for test code
fn fail_halt(msg: &str) -> ! {
    hprintln!("{}", msg);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        cortex_m::asm::nop();
    }
}

fn partition_task(id: usize) -> ! {
    let counter = match COUNTS.get(id) {
        Some(c) => c,
        None => fail_halt("partition id out of range"),
    };
    loop {
        if plib::sys_yield().is_err() {
            fail_halt("sys_yield failed");
        }
        if primask_is_active() {
            fail_halt("PRIMASK set after SVC return");
        }
        if basepri_value() != 0 {
            fail_halt("BASEPRI non-zero after SVC return");
        }
        counter.fetch_add(1, Ordering::Release);
    }
}

extern "C" fn p0_main() -> ! {
    partition_task(0)
}
extern "C" fn p1_main() -> ! {
    partition_task(1)
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("basepri: peripherals");
    hprintln!("svcall_basepri_clear_test: start");
    let sched =
        ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1).expect("basepri: sched");
    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);
    let k = Kernel::<Config>::create(sched, &cfgs).expect("basepri: kernel");
    store_kernel(k);
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, &mut p).expect("basepri: boot") {}
}
