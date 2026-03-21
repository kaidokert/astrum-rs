//! Minimal QEMU smoke test: boot → SYS_YIELD → exit.
//!
//! Single-partition baseline that validates the shortest kernel lifecycle:
//!
//! 1. Kernel boots with one partition via `define_unified_harness!`.
//! 2. Partition calls SYS_YIELD, stores the return code (0 = success).
//! 3. SysTick callback checks the return code and exits via `kexit!`.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example smoke_test
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

kernel::compose_kernel_config!(SmokeConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

const TIMEOUT_TICKS: u32 = 20;

/// SYS_YIELD success return code.
const YIELD_OK: u32 = 0;

/// Sentinel: partition has not yet executed its syscall.
const NOT_YET: u32 = 0xDEAD_C0DE;

/// Partition stores SYS_YIELD return code here.
static YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(SmokeConfig, |tick, _k| {
    let rc = YIELD_RC.load(Ordering::Acquire);

    if rc == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!("smoke_test: FAIL timeout (yield={:#x})", rc);
            kernel::kexit!(failure);
        }
        return;
    }

    if rc == YIELD_OK {
        hprintln!("smoke_test: PASS (yield={})", rc);
        kernel::kexit!(success);
    } else {
        hprintln!(
            "smoke_test: FAIL (yield={:#x} expected {:#x})",
            rc,
            YIELD_OK
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    match plib::sys_yield() {
        Ok(rc) => YIELD_RC.store(rc, Ordering::Release),
        Err(_) => YIELD_RC.store(u32::MAX, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("smoke_test: Peripherals::take");
    hprintln!("smoke_test: start");

    let sched = ScheduleTable::<{ SmokeConfig::SCHED }>::round_robin(1, 3)
        .expect("smoke_test: round_robin");

    let parts: [PartitionSpec; SmokeConfig::N] = [PartitionSpec::new(partition_main, 0)];
    init_kernel(sched, &parts).expect("smoke_test: init_kernel");

    match boot(p).expect("smoke_test: boot") {}
}
