//! QEMU test: P0 & P1 exercise plib mutex lock/unlock with actual contention.
//!
//! P0 acquires the mutex first (immediate), then yields. P1 attempts to lock
//! while P0 holds it, causing P1 to block. P0 then unlocks, waking P1. This
//! verifies the kernel's mutex blocking and wake-up logic.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;

const NOT_YET: u32 = 0xDEAD_C0DE;

static P0_LOCK_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static P0_UNLOCK_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static P1_LOCK_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static P1_UNLOCK_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let p0_lock = P0_LOCK_RC.load(Ordering::Acquire);
    let p0_unlock = P0_UNLOCK_RC.load(Ordering::Acquire);
    let p1_lock = P1_LOCK_RC.load(Ordering::Acquire);
    let p1_unlock = P1_UNLOCK_RC.load(Ordering::Acquire);

    if p0_lock == NOT_YET || p0_unlock == NOT_YET || p1_lock == NOT_YET || p1_unlock == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_mutex_test: FAIL timeout (p0_lock={:#x}, p0_unlock={:#x}, \
                 p1_lock={:#x}, p1_unlock={:#x})",
                p0_lock,
                p0_unlock,
                p1_lock,
                p1_unlock
            );
            kernel::kexit!(failure);
        }
        return;
    }

    // P0 acquired immediately (lock→1), P1 blocked then acquired (lock→0),
    // both unlocks succeed (→0).
    if p0_lock == 1 && p0_unlock == 0 && p1_lock == 0 && p1_unlock == 0 {
        hprintln!(
            "plib_mutex_test: PASS (p0_lock={}, p0_unlock={}, p1_lock={}, p1_unlock={})",
            p0_lock,
            p0_unlock,
            p1_lock,
            p1_unlock
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_mutex_test: FAIL (p0_lock={:#x}, p0_unlock={:#x}, \
             p1_lock={:#x}, p1_unlock={:#x})",
            p0_lock,
            p0_unlock,
            p1_lock,
            p1_unlock
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Lock the mutex — P0 runs first so this should be immediate (Ok(1)).
    match plib::sys_mtx_lock(plib::MutexId::new(0)) {
        Ok(rc) => P0_LOCK_RC.store(rc, Ordering::Release),
        Err(_) => P0_LOCK_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    // Yield to let P1 run and attempt to lock (P1 will block).
    let _ = plib::sys_yield();
    // Unlock — wakes P1 which was blocked on the mutex.
    match plib::sys_mtx_unlock(plib::MutexId::new(0)) {
        Ok(rc) => P0_UNLOCK_RC.store(rc, Ordering::Release),
        Err(_) => P0_UNLOCK_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    // Attempt to lock the mutex while P0 holds it — P1 blocks until P0 unlocks.
    // Expected: Ok(0) indicating blocked-then-acquired.
    match plib::sys_mtx_lock(plib::MutexId::new(0)) {
        Ok(rc) => P1_LOCK_RC.store(rc, Ordering::Release),
        Err(_) => P1_LOCK_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    match plib::sys_mtx_unlock(plib::MutexId::new(0)) {
        Ok(rc) => P1_UNLOCK_RC.store(rc, Ordering::Release),
        Err(_) => P1_UNLOCK_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_mutex_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_mutex_test: round_robin");

    init_kernel(
        sched,
        &[
            PartitionSpec::new(p0_main, 0),
            PartitionSpec::new(p1_main, 0),
        ],
    )
    .expect("plib_mutex_test: kernel");

    match boot(p).expect("plib_mutex_test: boot") {}
}
