//! 3-partition shared-config demo: blackboard + semaphore + events.
//!
//! Demonstrates: blackboard display/read with wake-all semantics,
//! semaphore-guarded critical sections, and event-based completion
//! signalling between a config partition and two worker partitions.
//!
//! Partitions run unprivileged and cannot use semihosting directly.
//! Progress is tracked via atomics; the SysTick handler verifies state
//! and prints results from privileged handler mode.
//!
//! R0 packing scheme (passed to each partition at entry):
//!   bits [31:24] = partition ID (so workers can self-identify)
//!   bits [23:16] = semaphore ID
//!   bits [15:0]  = blackboard ID
#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    svc,
    svc::Kernel,
    syscall::{
        SYS_BB_DISPLAY, SYS_BB_READ, SYS_EVT_SET, SYS_EVT_WAIT, SYS_SEM_SIGNAL, SYS_SEM_WAIT,
        SYS_YIELD,
    },
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config (tuned to this example's resource needs)
// ---------------------------------------------------------------------------
const NUM_PARTITIONS: usize = 3;
const STACK_WORDS: usize = 256;

// TODO: rename sampling_max_msg → sampling_msg_size, blackboard_max_msg → blackboard_msg_size
// for clarity (_size suffix convention); requires macro alias update in config.rs.
kernel::kernel_config! {
    /// Kernel configuration for the blackboard demo.
    ///
    /// Sized for 3 partitions, 1 semaphore, 1 blackboard (4-byte messages,
    /// 3-deep wait queue), and minimal allocations for unused resource pools.
    DemoConfig {
        partitions = 3;
        schedule_capacity = 8;
        semaphore_waitq = 3;
        sampling_max_msg = 1;
        blackboard_max_msg = 4;
        blackboard_waitq = 3;
    }
}

// ---------------------------------------------------------------------------
// R0 packing helpers
// ---------------------------------------------------------------------------

/// Pack partition ID, semaphore ID, and blackboard ID into a single u32.
const fn pack_r0(partition_id: u32, sem: u32, bb: u32) -> u32 {
    (partition_id << 24) | (sem << 16) | bb
}

// ---------------------------------------------------------------------------
// Atomic state for partition progress tracking (handler mode reads these)
// ---------------------------------------------------------------------------

/// Config: current round number (0..2)
static CONFIG_ROUND: AtomicU32 = AtomicU32::new(0);
/// Config: events received mask (bits 0=workerA, 1=workerB)
static CONFIG_EVENTS: AtomicU32 = AtomicU32::new(0);
/// Config: number of rounds completed
static CONFIG_DONE: AtomicU32 = AtomicU32::new(0);

/// WorkerA: number of configs read
static WORKER_A_READS: AtomicU32 = AtomicU32::new(0);
/// WorkerA: number of sem acquire/release cycles
static WORKER_A_SEM: AtomicU32 = AtomicU32::new(0);

/// WorkerB: number of configs read
static WORKER_B_READS: AtomicU32 = AtomicU32::new(0);
/// WorkerB: number of sem acquire/release cycles
static WORKER_B_SEM: AtomicU32 = AtomicU32::new(0);

// Use the unified harness macro with SysTick hook for progress verification.
// The hook runs in privileged handler mode and can use semihosting.
kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
    // Check progress every 10 ticks
    if tick.is_multiple_of(10) {
        let round = CONFIG_ROUND.load(Ordering::Acquire);
        let events = CONFIG_EVENTS.load(Ordering::Acquire);
        let done = CONFIG_DONE.load(Ordering::Acquire);
        let wa_reads = WORKER_A_READS.load(Ordering::Acquire);
        let wa_sem = WORKER_A_SEM.load(Ordering::Acquire);
        let wb_reads = WORKER_B_READS.load(Ordering::Acquire);
        let wb_sem = WORKER_B_SEM.load(Ordering::Acquire);

        hprintln!(
            "[tick {}] round={} events={:#x} done={} wa(r={},s={}) wb(r={},s={})",
            tick,
            round,
            events,
            done,
            wa_reads,
            wa_sem,
            wb_reads,
            wb_sem
        );

        // Test passes when config has completed 2 rounds
        if done >= 2 {
            hprintln!("blackboard_demo: all checks passed");
            debug::exit(debug::EXIT_SUCCESS);
        }

        // Timeout after 200 ticks
        if tick > 200 {
            hprintln!("blackboard_demo: FAIL - timeout");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
});

// ---------------------------------------------------------------------------
// Config partition: displays config on blackboard, waits for worker acks
// ---------------------------------------------------------------------------
extern "C" fn config_main_body(r0: u32) -> ! {
    let packed = r0;
    let bb = packed & 0xFFFF;

    for round in 0..2u8 {
        CONFIG_ROUND.store(round as u32, Ordering::Release);
        let cfg = [round + 1, 10 + round];
        svc!(SYS_BB_DISPLAY, bb, 2u32, cfg.as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);

        let mask = 0x03u32;
        let mut got = svc!(SYS_EVT_WAIT, 0u32, mask, 0u32);
        while got & mask != mask {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            got = svc!(SYS_EVT_WAIT, 0u32, mask, 0u32);
        }
        CONFIG_EVENTS.store(got, Ordering::Release);
        CONFIG_DONE.fetch_add(1, Ordering::Release);
    }

    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
kernel::partition_trampoline!(config_main => config_main_body);

// ---------------------------------------------------------------------------
// Worker: reads config from blackboard, acquires semaphore, signals event
// ---------------------------------------------------------------------------
fn worker(
    read_counter: &AtomicU32,
    sem_counter: &AtomicU32,
    bb: u32,
    sem: u32,
    partition_id: u32,
    evt: u32,
) -> ! {
    loop {
        let mut buf = [0u8; 4];
        let sz = svc!(SYS_BB_READ, bb, 0u32, buf.as_mut_ptr() as u32);
        if sz > 0 && sz != u32::MAX {
            read_counter.fetch_add(1, Ordering::Release);
            svc!(SYS_SEM_WAIT, sem, partition_id, 0u32);
            sem_counter.fetch_add(1, Ordering::Release);
            svc!(SYS_SEM_SIGNAL, sem, 0u32, 0u32);
            svc!(SYS_EVT_SET, 0u32, evt, 0u32);
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

extern "C" fn worker_a_body(r0: u32) -> ! {
    let p = r0;
    worker(
        &WORKER_A_READS,
        &WORKER_A_SEM,
        p & 0xFFFF,
        (p >> 16) & 0xFF,
        p >> 24,
        0x01,
    )
}
kernel::partition_trampoline!(worker_a => worker_a_body);

extern "C" fn worker_b_body(r0: u32) -> ! {
    let p = r0;
    worker(
        &WORKER_B_READS,
        &WORKER_B_SEM,
        p & 0xFFFF,
        (p >> 16) & 0xFF,
        p >> 24,
        0x02,
    )
}
kernel::partition_trampoline!(worker_b => worker_b_body);

// ---------------------------------------------------------------------------
// Entry point: create resources, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("blackboard_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..NUM_PARTITIONS as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    // Build partition configs. Stack bases are derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| PartitionConfig {
        id: i as u8,
        entry_point: 0, // Not used by Kernel::new
        stack_base: 0,  // Ignored: internal stack used
        stack_size: (STACK_WORDS * 4) as u32,
        mpu_region: MpuRegion::new(0, 0, 0), // Base/size overridden by Kernel::new
        peripheral_regions: heapless::Vec::new(),
    });

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<DemoConfig>::new(sched, &cfgs).expect("kernel creation");

    // Create blackboard and semaphore resources.
    let bb = k.blackboards_mut().create().unwrap() as u32;
    k.semaphores_mut().add(Semaphore::new(1, 1)).unwrap();
    let sem = 0u32; // first (and only) semaphore in the pool

    store_kernel(k);

    // Pack per-partition R0 values:
    //   config_main (partition 0): only needs blackboard ID
    //   worker_a    (partition 1): needs partition_id=1, sem, bb
    //   worker_b    (partition 2): needs partition_id=2, sem, bb
    let hints: [u32; NUM_PARTITIONS] = [
        pack_r0(0, sem, bb),
        pack_r0(1, sem, bb),
        pack_r0(2, sem, bb),
    ];
    let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [config_main, worker_a, worker_b];
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        core::array::from_fn(|i| (eps[i], hints[i]));

    match boot(&parts, &mut p).expect("blackboard_demo: boot failed") {}
}
