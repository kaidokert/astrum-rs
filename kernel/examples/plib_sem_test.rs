//! QEMU test: P0 signals semaphore 0, P1 waits on semaphore 0.
//!
//! Semaphore starts with count 0. P0 yields so P1 runs first and blocks on
//! `sys_sem_wait(0)`. P0 then signals via `sys_sem_signal(0)`, waking P1.
//! Both syscalls must return `Ok(0)`.

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
use kernel::semaphore::Semaphore;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;

const NOT_YET: u32 = 0xDEAD_C0DE;

static P0_SIGNAL_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static P1_WAIT_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let p0_signal = P0_SIGNAL_RC.load(Ordering::Acquire);
    let p1_wait = P1_WAIT_RC.load(Ordering::Acquire);

    if p0_signal == NOT_YET || p1_wait == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_sem_test: FAIL timeout (p0_signal={:#x}, p1_wait={:#x})",
                p0_signal,
                p1_wait
            );
            kernel::kexit!(failure);
        }
        return;
    }

    if p0_signal == 0 && p1_wait == 0 {
        hprintln!(
            "plib_sem_test: PASS (p0_signal={}, p1_wait={})",
            p0_signal,
            p1_wait
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_sem_test: FAIL (p0_signal={:#x}, p1_wait={:#x})",
            p0_signal,
            p1_wait
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn p0_main() -> ! {
    // Yield so P1 runs first and blocks on sem_wait(0).
    let _ = plib::sys_yield();
    // Signal the semaphore — wakes P1 which is blocked on wait.
    match plib::sys_sem_signal(plib::SemaphoreId::new(0)) {
        Ok(rc) => P0_SIGNAL_RC.store(rc, Ordering::Release),
        Err(e) => P0_SIGNAL_RC.store(e.to_u32(), Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    // Wait on the semaphore — blocks until P0 signals.
    match plib::sys_sem_wait(plib::SemaphoreId::new(0)) {
        Ok(rc) => P1_WAIT_RC.store(rc, Ordering::Release),
        Err(e) => P1_WAIT_RC.store(e.to_u32(), Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

// TODO: reviewer false positive – store_kernel and boot are provided by compose_kernel_config! macro
// TODO: .expect() calls below follow the established test/example pattern; convert to
//       panic-free error propagation if the project enforces the policy in examples.
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_sem_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_sem_test: round_robin");

    let mut k = Kernel::<TestConfig>::create_sentinels(sched).expect("plib_sem_test: kernel");
    k.semaphores_mut()
        .add(Semaphore::new(0, 1))
        .expect("plib_sem_test: add semaphore");
    store_kernel(k);

    match boot(&[(p0_main, 0), (p1_main, 0)], &mut p).expect("plib_sem_test: boot") {}
}
