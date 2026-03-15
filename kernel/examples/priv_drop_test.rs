//! QEMU test: verify partitions run unprivileged via CONTROL register.
//!
//! Boots a single partition whose entry function reads the CONTROL
//! register and checks:
//!
//! - **Bit 0 (nPRIV) = 1** — partition executes in unprivileged mode.
//! - **Bit 1 (SPSEL) = 1** — partition uses the Process Stack Pointer.
//!
//! The partition stores its CONTROL reading in an atomic; the SysTick
//! hook (privileged Handler mode) reads it and reports via semihosting.
//!
//! This validates the end-to-end privilege drop from the PendSV handler
//! through partition execution.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

/// Partition stores CONTROL reading here; 0 = not yet read.
static CONTROL_VAL: AtomicU32 = AtomicU32::new(0);
/// Set to 1 once the partition has stored its reading.
static DONE: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if DONE.load(Ordering::Acquire) == 1 {
        let control = CONTROL_VAL.load(Ordering::Acquire);
        let npriv = control & 1;
        let spsel = (control >> 1) & 1;
        hprintln!("priv_drop_test: CONTROL = {:#06x}", control);
        if npriv == 1 && spsel == 1 {
            hprintln!("priv_drop_test: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("priv_drop_test: FAIL");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick > 100 {
        hprintln!("priv_drop_test: FAIL - timeout");
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition entry: reads CONTROL and stores it in an atomic.
extern "C" fn partition_main() -> ! {
    let control: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control = 0;
    }
    CONTROL_VAL.store(control, Ordering::Release);
    DONE.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals already taken");
    hprintln!("priv_drop_test: start");

    // Build schedule: single partition runs for 2 ticks.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("static schedule entry must fit");

    let k = Kernel::<TestConfig>::create_sentinels(sched).expect("kernel creation");

    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); TestConfig::N] = [(partition_main, 0)];
    match boot(&parts, p).expect("priv_drop_test: boot failed") {}
}
