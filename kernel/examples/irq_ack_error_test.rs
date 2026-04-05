//! QEMU integration test: SYS_IRQ_ACK error-path validation.
//!
//! Binds IRQ 0 → partition 0 and IRQ 1 → partition 1 (both PartitionAcks).
//! Only partition 0 runs test logic; it exercises two error paths:
//!   1. Ack unbound IRQ 99 → InvalidResource
//!   2. Ack IRQ 1 owned by partition 1 → PermissionDenied
//!
//! After verifying both error codes the partition enters its event_wait loop.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt --example irq_ack_error_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use plib::SvcError;

kernel::kernel_config!(ErrTestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// IRQ 0 → P0, IRQ 1 → P1 (both PartitionAcks).
kernel::bind_interrupts!(ErrTestConfig, 70,
    0 => (0, 0x01),
    1 => (1, 0x02),
);

/// Incremented after each error-path check passes.
static CHECKS_PASSED: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(ErrTestConfig, |tick, _k| {
    if tick >= 4 {
        let n = CHECKS_PASSED.load(Ordering::Acquire);
        if n >= 2 {
            hprintln!("irq_ack_error_test: PASS (checks={})", n);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let n = CHECKS_PASSED.load(Ordering::Acquire);
        hprintln!("irq_ack_error_test: FAIL (checks={}, expected 2)", n);
        debug::exit(debug::EXIT_FAILURE);
    }
});

fn unwrap_or_fail<T>(r: Result<T, plib::SvcError>, ctx: &str) {
    if let Err(e) = r {
        hprintln!("{}: FAIL ({:?})", ctx, e);
        debug::exit(debug::EXIT_FAILURE);
    }
}

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Error-path tests: run once before entering normal event loop.
    //
    // Test 1: ack unbound IRQ 99 → InvalidResource.
    match plib::sys_irq_ack(99) {
        Err(SvcError::InvalidResource) => {
            CHECKS_PASSED.fetch_add(1, Ordering::Release);
        }
        other => {
            hprintln!("FAIL test1 ({:?})", other);
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    // Test 2: ack IRQ 1 owned by partition 1 → PermissionDenied.
    match plib::sys_irq_ack(1) {
        Err(SvcError::PermissionDenied) => {
            CHECKS_PASSED.fetch_add(1, Ordering::Release);
        }
        other => {
            hprintln!("FAIL test2 ({:?})", other);
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    loop {
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(0x01)),
            "p0 evt_wait",
        );
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(0x02)),
            "irq_ack_error_test: p1 evt_wait",
        );
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_ack_error_test: Peripherals::take");
    hprintln!("irq_ack_error_test: start");

    let sched = ScheduleTable::<{ ErrTestConfig::SCHED }>::round_robin(2, 3)
        .expect("irq_ack_error_test: round_robin");
    let parts: [PartitionSpec; ErrTestConfig::N] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("irq_ack_error_test: init_kernel");
    enable_bound_irqs(&mut p.NVIC, ErrTestConfig::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("irq_ack_error_test: boot") {}
}
