//! First example using the full kernel API: `kernel_config!`, `define_kernel!`,
//! and `Kernel::new`.
//!
//! This is the critical transition from bare-metal (examples 00–05) to
//! kernel-managed execution.  A single partition boots via the kernel,
//! calls `SYS_YIELD`, and stores the return code in an atomic.  The
//! SysTick hook checks the return code and exits with PASS/FAIL.
//!
//! # Known Pitfall — accessor function type mismatch
//!
//! `define_kernel!` generates exception handlers (`SysTick`, `PendSV`,
//! `MemoryManagement`) that access the kernel via
//! `state::with_kernel_mut::<Config, _, _>`.  The `Config` type parameter
//! **must** match the concrete config used by `kernel_config!`.  If you
//! define two config types (e.g. one for tests, one for production) and
//! accidentally pass the wrong one, the accessor functions will operate on
//! an uninitialised kernel pointer and the system will fault silently.
//! Always use a single config type per binary.
//!
//! # Run
//!
//! ```text
//! cargo run --target thumbv7m-none-eabi \
//!     --features board-qemu,log-semihosting \
//!     --example 06_kernel_minimal
//! ```

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

// ── Configuration ──────────────────────────────────────────────────

kernel::kernel_config!(
    Cfg < Partitions1,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

// ── Shared state ───────────────────────────────────────────────────

const TIMEOUT_TICKS: u32 = 50;

/// SYS_YIELD expected success code.
const YIELD_OK: u32 = 0;

/// Sentinel: partition has not yet executed its syscall.
const NOT_YET: u32 = 0xDEAD_C0DE;

/// Partition stores the SYS_YIELD return code here.
static YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);

// ── SysTick hook (runs every tick inside the kernel critical section) ──

kernel::define_kernel!(Cfg, |tick, _k| {
    let rc = YIELD_RC.load(Ordering::Acquire);

    if rc == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            klog!("06_kernel_minimal: FAIL timeout (yield={:#x})", rc);
            kernel::kexit!(failure);
        }
        return;
    }

    if rc == YIELD_OK {
        klog!("06_kernel_minimal: PASS (yield={})", rc);
        kernel::kexit!(success);
    } else {
        klog!(
            "06_kernel_minimal: FAIL (yield={:#x} expected {:#x})",
            rc,
            YIELD_OK
        );
        kernel::kexit!(failure);
    }
});

// ── Partition entry point ──────────────────────────────────────────

/// Type-check: ensure `partition_main` matches the expected signature.
const _: PartitionEntry = partition_main;

extern "C" fn partition_main() -> ! {
    // Issue SYS_YIELD and record the result.
    match plib::sys_yield() {
        Ok(rc) => YIELD_RC.store(rc, Ordering::Release),
        Err(_) => YIELD_RC.store(u32::MAX, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

// ── main: build kernel, boot ───────────────────────────────────────

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("06_kernel_minimal: kernel API smoke test");

    // Schedule: P0 runs for 3 ticks, then a 1-tick system window.
    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("syswin");

    let parts: [PartitionSpec; Cfg::N] = [PartitionSpec::new(partition_main as PartitionEntry, 0)];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(&mut k);

    match boot(p).expect("06_kernel_minimal: boot") {}
}
