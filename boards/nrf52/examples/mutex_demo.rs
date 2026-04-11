//! Mutex Demo for nRF52833 (PCA10100)
//!
//! Demonstrates mutual exclusion with mutexes:
//! - Two worker partitions compete for a shared resource
//! - Mutex protects critical section (shared counter)
//! - Demonstrates proper lock/unlock patterns
//!
//! Two partitions:
//! - P0 (Worker A): Locks mutex, increments counter, unlocks
//! - P1 (Worker B): Locks mutex, increments counter, unlocks
//!
//! Without mutex protection, counter updates would be lost due to race conditions.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use plib;
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here.
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;

// Shared counter (protected by mutex)
static SHARED_COUNTER: AtomicU32 = AtomicU32::new(0);

// Progress tracking per partition
static WORKER_A_LOCKS: AtomicU32 = AtomicU32::new(0);
static WORKER_A_INCREMENTS: AtomicU32 = AtomicU32::new(0);
static WORKER_B_LOCKS: AtomicU32 = AtomicU32::new(0);
static WORKER_B_INCREMENTS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(MutexConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 4;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(MutexConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let shared = SHARED_COUNTER.load(Ordering::Acquire);
        let a_locks = WORKER_A_LOCKS.load(Ordering::Acquire);
        let a_inc = WORKER_A_INCREMENTS.load(Ordering::Acquire);
        let b_locks = WORKER_B_LOCKS.load(Ordering::Acquire);
        let b_inc = WORKER_B_INCREMENTS.load(Ordering::Acquire);

        rprintln!(
            "[{:5}ms] SHARED={:4} | A: locks={:3} inc={:3} | B: locks={:3} inc={:3}",
            tick, shared, a_locks, a_inc, b_locks, b_inc
        );

        if shared > 10 && a_inc > 5 && b_inc > 5 {
            rprintln!("✓ SUCCESS: Mutex working! Both workers incrementing shared counter.");
        }
    }
});

/// Worker A partition - competes for mutex to access shared counter
extern "C" fn worker_a_main_body(r0: u32) -> ! {
    let mtx_id = r0.into();

    loop {
        // Try to lock mutex
        if plib::sys_mtx_lock(mtx_id).is_ok() {
            // Got mutex - enter critical section
            WORKER_A_LOCKS.fetch_add(1, Ordering::Release);

            // Read current value
            let old_val = SHARED_COUNTER.load(Ordering::Acquire);

            // Simulate work (this delay makes race conditions more likely without mutex)
            for _ in 0..5000 {
                core::hint::spin_loop();
            }

            // Increment and write back
            let new_val = old_val + 1;
            SHARED_COUNTER.store(new_val, Ordering::Release);
            WORKER_A_INCREMENTS.fetch_add(1, Ordering::Release);

            // Release mutex
            plib::sys_mtx_unlock(mtx_id).ok();
        }

        // Delay between attempts
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

/// Worker B partition - competes for mutex to access shared counter
extern "C" fn worker_b_main_body(r0: u32) -> ! {
    let mtx_id = r0.into();

    loop {
        // Try to lock mutex
        if plib::sys_mtx_lock(mtx_id).is_ok() {
            // Got mutex - enter critical section
            WORKER_B_LOCKS.fetch_add(1, Ordering::Release);

            // Read current value
            let old_val = SHARED_COUNTER.load(Ordering::Acquire);

            // Simulate work
            for _ in 0..5000 {
                core::hint::spin_loop();
            }

            // Increment and write back
            let new_val = old_val + 1;
            SHARED_COUNTER.store(new_val, Ordering::Release);
            WORKER_B_INCREMENTS.fetch_add(1, Ordering::Release);

            // Release mutex
            plib::sys_mtx_unlock(mtx_id).ok();
        }

        // Delay between attempts
        for _ in 0..8000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(worker_a_main => worker_a_main_body);
kernel::partition_trampoline!(worker_b_main => worker_b_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Mutex Demo ===");
    rprintln!("nRF52833 - Mutual Exclusion and Critical Sections");

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: Equal time slices
    let mut sched = ScheduleTable::<{ MutexConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");
    rprintln!("Schedule: P0(WorkerA) P1(WorkerB)");

    let mtx = 0u32;

    let h: [u32; NUM_PARTITIONS] = [mtx, mtx];

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(worker_a_main as kernel::PartitionEntry, h[0]),
        PartitionSpec::new(worker_b_main as kernel::PartitionEntry, h[1]),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("Kernel created");
    rprintln!("Using mutex: {} (from {} available)", mtx, MutexConfig::MS);

    rprintln!("Booting...\n");

    match boot(p).expect("boot") {}
}
