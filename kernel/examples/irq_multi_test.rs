//! QEMU integration test: multi-IRQ routing to independent partitions.
//!
//! Validates that IRQ dispatch delivers events to the correct partition:
//!
//! 1. `bind_interrupts!` maps IRQ 0 → partition 0 (event 0x01) and
//!    IRQ 1 → partition 1 (event 0x02).
//! 2. Each partition loops on `event_wait` for its respective event bit
//!    and increments a separate AtomicU32 counter on wake.
//! 3. SysTick software-triggers IRQ 0 at tick 2 and IRQ 1 at tick 3,
//!    then checks both counters are nonzero by tick 8.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example irq_multi_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::svc::SvcError;
use kernel::syscall::SYS_EVT_WAIT;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(MultiIrqConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind IRQ 0 → partition 0, event 0x01; IRQ 1 → partition 1, event 0x02.
kernel::bind_interrupts!(MultiIrqConfig, 70,
    0 => (0, 0x01),
    1 => (1, 0x02),
);

const NUM_PARTITIONS: usize = 2;
/// Incremented by partition 0 after each successful `event_wait` return.
static P0_COUNT: AtomicU32 = AtomicU32::new(0);
/// Incremented by partition 1 after each successful `event_wait` return.
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(MultiIrqConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("irq_multi_test: pended IRQ 0 at tick {}", tick);
    }
    if tick == 3 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(1));
        hprintln!("irq_multi_test: pended IRQ 1 at tick {}", tick);
    }
    if tick >= 8 {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        if c0 > 0 && c1 > 0 {
            hprintln!("irq_multi_test: PASS (p0={}, p1={})", c0, c1);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        hprintln!("irq_multi_test: FAIL (p0={}, p1={})", c0, c1);
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x01u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("irq_multi_test: p0 FAIL (event_wait rc=0x{:08X})", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        P0_COUNT.fetch_add(1, Ordering::Release);
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x02u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("irq_multi_test: p1 FAIL (event_wait rc=0x{:08X})", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        P1_COUNT.fetch_add(1, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_multi_test: Peripherals::take");
    hprintln!("irq_multi_test: start");

    let sched = ScheduleTable::<{ MultiIrqConfig::SCHED }>::round_robin(2, 3)
        .expect("irq_multi_test: round_robin");

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    init_kernel(sched, &parts).expect("irq_multi_test: Kernel::create");

    // Unmask bound IRQs so software-triggered pends fire.
    enable_bound_irqs(&mut p.NVIC, MultiIrqConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("irq_multi_test: boot") {}
}
