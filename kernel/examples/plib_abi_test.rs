//! QEMU test: exercise plib syscall wrappers from unprivileged partition code.
//!
//! Boots a single partition that calls `plib::sys_yield()`,
//! `plib::sys_get_partition_id()`, and `plib::sys_get_time()`, storing
//! results to atomics.  The SysTick hook (privileged) verifies:
//!
//! 1. `sys_yield` returned `Ok(0)`
//! 2. `sys_get_partition_id` returned `Ok(0)`
//! 3. `sys_get_time` returned a non-zero tick

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
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Sentinel: syscall not yet attempted.
const NOT_YET: u32 = 0xDEAD_C0DE;
const TIMEOUT: u32 = 20;

/// sys_yield result: Ok(0)→0, Err→0xFFFF_FFFF.
static YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// sys_get_partition_id result: Ok(id)→id, Err→0xFFFF_FFFF.
static PID_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Latest sys_get_time reading (0 until partition reads a non-zero tick).
static TIME_VAL: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if tick < 5 {
        return;
    }
    let yield_rc = YIELD_RC.load(Ordering::Acquire);
    let pid = PID_RC.load(Ordering::Acquire);
    let time = TIME_VAL.load(Ordering::Acquire);

    if yield_rc == NOT_YET || pid == NOT_YET {
        if tick >= TIMEOUT {
            hprintln!("plib_abi_test: FAIL timeout");
            kernel::kexit!(failure);
        }
        return;
    }

    if yield_rc != 0 {
        hprintln!("plib_abi_test: FAIL yield_rc={:#x}", yield_rc);
        kernel::kexit!(failure);
    }
    if pid != 0 {
        hprintln!("plib_abi_test: FAIL pid={:#x}", pid);
        kernel::kexit!(failure);
    }
    if time == 0 {
        if tick >= TIMEOUT {
            hprintln!("plib_abi_test: FAIL time still zero");
            kernel::kexit!(failure);
        }
        return;
    }

    hprintln!("plib_abi_test: PASS (yield=0, pid=0, time={})", time);
    kernel::kexit!(success);
});

extern "C" fn partition_main() -> ! {
    // 1. sys_yield: expect Ok(0)
    match plib::sys_yield() {
        Ok(v) => YIELD_RC.store(v, Ordering::Release),
        Err(_) => YIELD_RC.store(0xFFFF_FFFF, Ordering::Release),
    }

    // 2. sys_get_partition_id: expect Ok(PartitionId(0))
    match plib::sys_get_partition_id() {
        Ok(id) => PID_RC.store(id.as_raw(), Ordering::Release),
        Err(_) => PID_RC.store(0xFFFF_FFFF, Ordering::Release),
    }

    // 3. sys_get_time: loop reading tick count
    loop {
        if let Ok(t) = plib::sys_get_time() {
            TIME_VAL.store(t, Ordering::Release);
        }
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_abi_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(1, 3).expect("round_robin");
    let k = Kernel::<TestConfig>::create_sentinels(sched).expect("kernel");
    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); TestConfig::N] = [(partition_main, 0)];
    match boot(&parts, &mut p).expect("boot") {}
}
