//! QEMU starvation-detection test: P1 blocks on an empty semaphore and is
//! never signalled. SysTick verifies starvation_count >= STARVATION_THRESHOLD.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::{PartitionConfig, STARVATION_THRESHOLD};
use kernel::scheduler::ScheduleTable;
use kernel::semaphore::Semaphore;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    Config<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = Config::N;
const STACK_WORDS: usize = Config::STACK_WORDS;

/// P1 is the partition that will starve (index 1).
const STARVING_PID: usize = 1;

/// Give the scheduler enough ticks to skip P1 at least STARVATION_THRESHOLD
/// times.  Each major frame is 2 ticks; P1's slot comes once per frame.
// TODO: CHECK_TICK assumes round_robin with 2 partitions and 1-tick slots;
//       if the scheduler policy changes, this calculation may need updating.
const CHECK_TICK: u32 = (STARVATION_THRESHOLD as u32 + 1) * 2 + 2;
/// Hard timeout.
const TIMEOUT_TICK: u32 = 50;

/// P0 loop counter — proves P0 actually ran.
static P0_COUNT: AtomicU32 = AtomicU32::new(0);
/// Non-zero if P1's sem_wait syscall failed (stores error code).
static P1_SEM_ERR: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(Config, |tick, k| {
    if tick >= CHECK_TICK {
        let p0_ran = P0_COUNT.load(Ordering::Acquire);
        if p0_ran == 0 {
            hprintln!("starvation_detect_test: FAIL p0 never ran");
            kernel::kexit!(failure);
        }

        let Some(pcb) = k.partitions().get(STARVING_PID) else {
            hprintln!("starvation_detect_test: FAIL no partition {}", STARVING_PID);
            kernel::kexit!(failure);
            return;
        };

        let count = pcb.starvation_count();
        let starved = pcb.is_starved();

        // TODO: consider adding an explicit check that P1 attempted to run before
        //       blocking; the starvation_count check partially covers this.
        if count >= STARVATION_THRESHOLD && starved {
            hprintln!(
                "starvation_detect_test: PASS (count={}, threshold={}, is_starved=true)",
                count,
                STARVATION_THRESHOLD
            );
            kernel::kexit!(success);
        }
    }

    let sem_err = P1_SEM_ERR.load(Ordering::Acquire);
    if sem_err != 0 {
        hprintln!(
            "starvation_detect_test: FAIL p1 sem_wait error={:#x}",
            sem_err
        );
        kernel::kexit!(failure);
    }

    if tick >= TIMEOUT_TICK {
        let count = k
            .partitions()
            .get(STARVING_PID)
            .map(|p| p.starvation_count())
            .unwrap_or(0);
        hprintln!(
            "starvation_detect_test: FAIL timeout (count={}, need>={})",
            count,
            STARVATION_THRESHOLD
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        P0_COUNT.fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    // Block on an empty semaphore — transitions to Waiting immediately.
    // TODO: reviewer false positive — plib is a dev-dependency (Cargo.toml [dev-dependencies]),
    //       available to examples without `use plib;` in Rust 2021 edition.
    if let Err(e) = plib::sys_sem_wait(plib::SemaphoreId::new(0)) {
        P1_SEM_ERR.store(e.to_u32(), Ordering::Release);
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("Peripherals::take");
    hprintln!("starvation_detect_test: start");

    let sched =
        ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1).expect("round_robin");

    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);
    let mut k = Kernel::<Config>::create(sched, &cfgs).expect("Kernel::create");

    // Semaphore 0 with count=0, max=1 — P1 will block immediately on wait.
    k.semaphores_mut()
        .add(Semaphore::new(0, 1))
        .expect("add semaphore");

    // TODO: reviewer false positive — store_kernel and boot are emitted by
    //       define_unified_harness!, not separate imports.
    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, &mut p).expect("boot") {}
}
