//! QEMU smoke test: AlignedStack4K stacks with Partitions2.
//!
//! Validates that the fixed-offset assertions (LITERAL_POOL_OFFSET_LIMIT = 65536)
//! and MAX_KERNEL_SIZE (32 KB) work correctly with the larger 4 KiB stack alignment
//! that previously triggered false assertion failures.
//!
//! 1. Kernel boots with two partitions (AlignedStack4K stacks).
//! 2. Round-robin scheduler gives each partition 3 ticks.
//! 3. Partition 0 calls SYS_YIELD — exercises PendSV context switch with 4K offsets.
//! 4. Partition 1 calls SYS_YIELD — confirms both partitions schedule correctly.
//! 5. SysTick callback verifies both return codes, exits via `kexit!`.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example qemu_smoke_4k
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

kernel::kernel_config!(
    Smoke4KConfig[kernel::partition_core::AlignedStack4K] < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled >
);

const TIMEOUT_TICKS: u32 = 50;

/// SYS_YIELD success return code.
const YIELD_OK: u32 = 0;
/// Sentinel: partition has not yet executed its syscall.
const NOT_YET: u32 = 0xDEAD_C0DE;

/// Partition 0 stores SYS_YIELD return code here.
static P0_YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Partition 1 stores SYS_YIELD return code here.
static P1_YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_harness!(Smoke4KConfig, |tick, _k| {
    let p0 = P0_YIELD_RC.load(Ordering::Acquire);
    let p1 = P1_YIELD_RC.load(Ordering::Acquire);

    // Guard: wait until both partitions have executed their syscalls.
    if p0 == NOT_YET || p1 == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!("qemu_smoke_4k: FAIL timeout (p0={:#x}, p1={:#x})", p0, p1);
            kernel::kexit!(failure);
        }
        return;
    }

    if p0 == YIELD_OK && p1 == YIELD_OK {
        hprintln!("qemu_smoke_4k: PASS (p0={}, p1={})", p0, p1);
        kernel::kexit!(success);
    } else {
        hprintln!(
            "qemu_smoke_4k: FAIL (p0={:#x} expected {:#x}, p1={:#x} expected {:#x})",
            p0,
            YIELD_OK,
            p1,
            YIELD_OK
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let rc = plib::sys_yield().unwrap_or(u32::MAX);
    P0_YIELD_RC.store(rc, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    let rc = plib::sys_yield().unwrap_or(u32::MAX);
    P1_YIELD_RC.store(rc, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("qemu_smoke_4k: Peripherals::take");
    hprintln!("qemu_smoke_4k: start");

    let sched = ScheduleTable::<{ Smoke4KConfig::SCHED }>::round_robin(2, 3)
        .expect("qemu_smoke_4k: round_robin");

    let parts: [PartitionSpec; Smoke4KConfig::N] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("qemu_smoke_4k: init_kernel");
    store_kernel(&mut k);

    match boot(p).expect("qemu_smoke_4k: boot") {}
}
