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
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    svc::Kernel,
    PartitionBody, PartitionEntry,
};

// ---------------------------------------------------------------------------
// Kernel configuration (tuned to this example's resource needs)
// ---------------------------------------------------------------------------

// Kernel configuration for the blackboard demo.
//
// Sized for 3 partitions, 1 semaphore, 1 blackboard (4-byte messages,
// 3-deep wait queue), and minimal allocations for unused resource pools.
kernel::kernel_config!(DemoConfig<kernel::Partitions3, kernel::SyncStandard, kernel::MsgStandard, kernel::PortsSmall, kernel::DebugEnabled>);

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
/// Data integrity errors (workers verify blackboard content)
static DATA_ERRORS: AtomicU32 = AtomicU32::new(0);

// Use the unified harness macro with SysTick hook for progress verification.
// The hook runs in privileged handler mode and can use semihosting.
kernel::define_kernel!(DemoConfig, |tick, _k| {
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

        // Data integrity: fail if any worker read corrupted blackboard data
        let data_err = DATA_ERRORS.load(Ordering::Acquire);
        if data_err > 0 {
            hprintln!("blackboard_demo: FAIL - {} data integrity errors", data_err);
            debug::exit(debug::EXIT_FAILURE);
        }

        // Test passes when config has completed 2 rounds and workers performed IPC
        if done >= 2 {
            if wa_reads == 0 || wb_reads == 0 {
                hprintln!("blackboard_demo: FAIL - no worker reads");
                debug::exit(debug::EXIT_FAILURE);
            }
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
const _: PartitionBody = config_main_body;
extern "C" fn config_main_body(r0: u32) -> ! {
    let packed = r0;
    let bb = plib::BlackboardId::new(packed & 0xFFFF);

    for round in 0..2u8 {
        let cfg = [round + 1, 10 + round];
        if plib::sys_bb_display(bb, &cfg).is_ok() {
            CONFIG_ROUND.store(round as u32, Ordering::Release);
        }
        let _ = plib::sys_yield();

        let mask = plib::EventMask::new(0x03);
        let mut got = plib::sys_event_wait(mask);
        while matches!(got, Ok(v) if v.as_raw() & mask.as_raw() != mask.as_raw()) {
            let _ = plib::sys_yield();
            got = plib::sys_event_wait(mask);
        }
        if let Ok(v) = got {
            CONFIG_EVENTS.store(v.as_raw(), Ordering::Release);
            CONFIG_DONE.fetch_add(1, Ordering::Release);
        }
    }

    loop {
        let _ = plib::sys_yield();
    }
}
kernel::partition_trampoline!(config_main => config_main_body);

// ---------------------------------------------------------------------------
// Worker: reads config from blackboard, acquires semaphore, signals event
// ---------------------------------------------------------------------------
fn worker(
    read_counter: &AtomicU32,
    sem_counter: &AtomicU32,
    bb: plib::BlackboardId,
    sem: u32,
    evt: u32,
) -> ! {
    loop {
        let mut buf = [0u8; 4];
        if let Ok(sz) = plib::sys_bb_read(bb, &mut buf) {
            if sz > 0 {
                // Verify blackboard data: config writes [round+1, 10+round],
                // so buf[1] must equal buf[0]+9 when both bytes are present.
                if sz >= 2 && buf[1] != buf[0].wrapping_add(9) {
                    DATA_ERRORS.fetch_add(1, Ordering::Release);
                }
                read_counter.fetch_add(1, Ordering::Release);
                // TODO: SemaphoreId call-site fix from ce7dd01; included here for build correctness
                if plib::sys_sem_wait(plib::SemaphoreId::new(sem)).is_ok() {
                    sem_counter.fetch_add(1, Ordering::Release);
                    if plib::sys_sem_signal(plib::SemaphoreId::new(sem)).is_err() {
                        DATA_ERRORS.fetch_add(1, Ordering::Release);
                    }
                    if plib::sys_event_set(plib::PartitionId::new(0), plib::EventMask::new(evt))
                        .is_err()
                    {
                        DATA_ERRORS.fetch_add(1, Ordering::Release);
                    }
                }
            }
        }
        let _ = plib::sys_yield();
    }
}

const _: PartitionBody = worker_a_body;
extern "C" fn worker_a_body(r0: u32) -> ! {
    let p = r0;
    worker(
        &WORKER_A_READS,
        &WORKER_A_SEM,
        plib::BlackboardId::new(p & 0xFFFF),
        (p >> 16) & 0xFF,
        0x01,
    )
}
kernel::partition_trampoline!(worker_a => worker_a_body);

const _: PartitionBody = worker_b_body;
extern "C" fn worker_b_body(r0: u32) -> ! {
    let p = r0;
    worker(
        &WORKER_B_READS,
        &WORKER_B_SEM,
        plib::BlackboardId::new(p & 0xFFFF),
        (p >> 16) & 0xFF,
        0x02,
    )
}
kernel::partition_trampoline!(worker_b => worker_b_body);

// ---------------------------------------------------------------------------
// Entry point: create resources, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("blackboard_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..DemoConfig::N as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    let eps: [PartitionEntry; DemoConfig::N] = [config_main, worker_a, worker_b];
    // TODO: DRY — this ExternalPartitionMemory init block is duplicated across examples.
    // Consider a safe Kernel API or helper that accepts entry points directly, eliminating
    // the manual pointer arithmetic on partition_stacks! output.
    let mut k = {
        use kernel::partition::{ExternalPartitionMemory, MpuRegion};
        let stacks = kernel::partition_stacks!(DemoConfig, DemoConfig::N);
        let sp = stacks.as_mut_ptr();
        let mems: [_; DemoConfig::N] = core::array::from_fn(|i| {
            // SAFETY: `sp` points to the first of `DemoConfig::N` contiguous
            // `StackStorage` elements returned by `partition_stacks!`, and `i`
            // is in 0..N, so `sp.add(i)` is in-bounds. Each index is visited
            // exactly once by `from_fn`, so no aliasing occurs.
            let s = unsafe { &mut *sp.add(i) };
            ExternalPartitionMemory::from_aligned_stack(
                s,
                eps[i],
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(i as u32),
            )
            .expect("mem")
        });
        sched.add_system_window(1).expect("sys window");
        Kernel::<DemoConfig>::new(sched, &mems).expect("kernel creation")
    };

    // Create blackboard and semaphore resources.
    let bb = k.blackboards_mut().create().unwrap() as u32;
    k.semaphores_mut().add(Semaphore::new(1, 1)).unwrap();
    let sem = 0u32; // first (and only) semaphore in the pool

    // Set r0 hints: partition_id | sem | bb packed per partition.
    let hints = [
        pack_r0(0, sem, bb),
        pack_r0(1, sem, bb),
        pack_r0(2, sem, bb),
    ];
    for (i, &h) in hints.iter().enumerate() {
        k.partitions_mut().get_mut(i).expect("pcb").set_r0_hint(h);
    }
    store_kernel(&mut k);
    // SAFETY: PCBs populated by Kernel::new() with valid stacks.
    match unsafe { kernel::boot::boot_preconfigured::<DemoConfig>(p) }
        .expect("blackboard_demo: boot") {}
}
