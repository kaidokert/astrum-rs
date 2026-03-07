//! Defense-in-depth test: two partitions call sys_yield() in a loop, checking
//! PRIMASK==0 and BASEPRI==0 after each SVC under fast SysTick+PendSV stress.
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

static PARTITION_COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

kernel::define_unified_harness!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    if tick >= CHECK_TICK {
        let c: [u32; 2] = core::array::from_fn(|i| {
            PARTITION_COUNTS
                .get(i)
                .map_or(0, |c| c.load(Ordering::Acquire))
        });
        if c.iter().all(|&v| v >= MIN_COUNT) {
            hprintln!("svcall_primask_clear_test: PASS ({:?})", c);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= TIMEOUT_TICK {
        let c: [u32; 2] = core::array::from_fn(|i| {
            PARTITION_COUNTS
                .get(i)
                .map_or(0, |c| c.load(Ordering::Acquire))
        });
        hprintln!("svcall_primask_clear_test: FAIL timeout ({:?})", c);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Returns true if PRIMASK is set (interrupts disabled).
#[inline(always)]
fn primask_is_set() -> bool {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::primask::read().is_inactive()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        false // host-build stub for clippy / check
    }
}

/// Returns the current BASEPRI value (0 means no priority masking).
#[inline(always)]
fn basepri_value() -> u8 {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::basepri::read()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        0 // host-build stub for clippy / check
    }
}

fn fail_halt(msg: &str) -> ! {
    hprintln!("{}", msg);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        cortex_m::asm::nop();
    }
}

fn partition_task(id: usize) -> ! {
    let counter = match PARTITION_COUNTS.get(id) {
        Some(c) => c,
        None => fail_halt("partition id out of range"),
    };
    loop {
        if plib::sys_yield().is_err() {
            fail_halt("sys_yield failed");
        }
        if primask_is_set() {
            fail_halt("PRIMASK leak after sys_yield");
        }
        if basepri_value() != 0 {
            fail_halt("BASEPRI leak after sys_yield");
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
    let mut p = cortex_m::Peripherals::take().expect("svcall: peripherals");
    hprintln!("svcall_primask_clear_test: start");
    let sched =
        ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1).expect("svcall: sched");
    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);
    let k = Kernel::<Config>::create(sched, &cfgs).expect("svcall: kernel");
    // TODO: reviewer false positive — boot() and store_kernel() are generated
    // by the define_unified_harness!() macro, not imported.
    store_kernel(k);
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, &mut p).expect("svcall: boot") {}
}
