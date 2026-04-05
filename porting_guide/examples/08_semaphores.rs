//! Two-partition semaphore synchronization. P0 signals, P1 waits.
#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::semaphore::Semaphore;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

kernel::kernel_config!(
    Cfg < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

const GOAL: u32 = 10;
const TIMEOUT: u32 = 500;
static SIGNALS: AtomicU32 = AtomicU32::new(0);
static WAITS: AtomicU32 = AtomicU32::new(0);
static ERROR: AtomicU32 = AtomicU32::new(0);
fn halt_with(code: u32) -> ! {
    ERROR.store(code, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

kernel::define_kernel!(Cfg, |tick, _k| {
    let err = ERROR.load(Ordering::Acquire);
    if err != 0 {
        klog!("08_semaphores: FAIL error ({:#x})", err);
        kernel::kexit!(failure);
    }
    let w = WAITS.load(Ordering::Acquire);
    let s = SIGNALS.load(Ordering::Acquire);
    if w >= GOAL {
        klog!("08_semaphores: PASS (s={}, w={})", s, w);
        kernel::kexit!(success);
    }
    if tick >= TIMEOUT {
        klog!("08_semaphores: FAIL timeout (s={}, w={})", s, w);
        kernel::kexit!(failure);
    }
    let _ = (s, w);
});

const _: PartitionEntry = signaller;
const _: PartitionEntry = waiter;

extern "C" fn signaller() -> ! {
    let sem = plib::SemaphoreId::new(0);
    let mut n: u32 = 0;
    loop {
        n += 1;
        SIGNALS.store(n, Ordering::Release);
        if plib::sys_sem_signal(sem).is_err() {
            halt_with(0xBAD0_0001);
        }
        if plib::sys_yield().is_err() {
            halt_with(0xBAD0_00FF);
        }
    }
}

extern "C" fn waiter() -> ! {
    let sem = plib::SemaphoreId::new(0);
    let mut n: u32 = 0;
    loop {
        if plib::sys_sem_wait(sem).is_err() {
            halt_with(0xBAD1_0001);
        }
        n += 1;
        WAITS.store(n, Ordering::Release);
        if plib::sys_yield().is_err() {
            halt_with(0xBAD1_00FF);
        }
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("08_semaphores: semaphore sync between two partitions");
    let mut sched: ScheduleTable<{ Cfg::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");
    sched.add_system_window(1).expect("syswin 1");
    let entries: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(signaller as PartitionEntry, 0),
        PartitionSpec::new(waiter as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &entries).expect("init_kernel");
    k.semaphores_mut().add(Semaphore::new(0, 1)).expect("sem");
    store_kernel(&mut k);
    match boot(p).expect("08_semaphores: boot") {}
}
