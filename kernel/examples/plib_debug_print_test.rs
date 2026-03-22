//! QEMU test: exercise `plib::sys_debug_print` from unprivileged partition code.
//!
//! Boots a single partition that calls `plib::sys_debug_print` with a short
//! message, storing the return code to an atomic.  The SysTick hook verifies
//! `sys_debug_print` returned `Ok(0)`.
//!
//! This validates that the plib wrapper passes ptr/len in the correct register
//! order (r1=ptr, r2=len), which differs from the resource-id pattern used by
//! most other syscalls.

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
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Sentinel: syscall not yet attempted.
const NOT_YET: u32 = 0xDEAD_C0DE;
const TIMEOUT: u32 = 20;

/// sys_debug_print result: Ok(0)→0, Err→e.to_u32().
static PRINT_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if tick < 5 {
        return;
    }
    let rc = PRINT_RC.load(Ordering::Acquire);

    if rc == NOT_YET {
        if tick >= TIMEOUT {
            hprintln!("plib_debug_print_test: FAIL timeout");
            kernel::kexit!(failure);
        }
        return;
    }

    if rc != 0 {
        hprintln!("plib_debug_print_test: FAIL print_rc={:#x}", rc);
        kernel::kexit!(failure);
    }

    hprintln!("plib_debug_print_test: PASS (print_rc=0)");
    kernel::kexit!(success);
});

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    match plib::sys_debug_print(b"hello from partition") {
        Ok(v) => PRINT_RC.store(v, Ordering::Release),
        Err(e) => PRINT_RC.store(e.to_u32(), Ordering::Release),
    }

    loop {
        let _ = plib::sys_yield();
    }
}

#[entry]
fn main() -> ! {
    // TODO: #[entry] requires `-> !` so main cannot return Result; .expect()
    // matches established test patterns until a panic-free boot helper exists.
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_debug_print_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(1, 3).expect("round_robin");
    let parts: [PartitionSpec; TestConfig::N] =
        [PartitionSpec::new(partition_main as PartitionEntry, 0)];
    init_kernel(sched, &parts).expect("kernel");

    match boot(p).expect("boot") {}
}
