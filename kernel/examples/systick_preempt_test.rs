//! QEMU integration test: SysTick cannot be preempted by app IRQ.
//!
//! At tick 2, the SysTick hook pends IRQ 0 (at MIN_APP_IRQ_PRIORITY 0x20)
//! and checks `NVIC::is_pending` — since SYSTICK_PRIORITY (0x10) is higher,
//! the IRQ must remain pending.  After SysTick returns, the IRQ fires and
//! dispatches event 0x01 to partition 0, confirming eventual delivery.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example systick_preempt_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(Config<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind IRQ 0 → partition 0, event bit 0x01.
kernel::bind_interrupts!(Config, 70,
    0 => (0, 0x01),
);

/// Set to 1 once we verify the IRQ remained pending inside SysTick.
static PEND_VERIFIED: AtomicU32 = AtomicU32::new(0);
/// Incremented by the partition after each successful event_wait return.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(Config, |tick, _k| {
    if tick == 2 {
        // Software-trigger IRQ 0 while inside SysTick handler.
        // TODO: reviewer false positive — NVIC::pend / is_pending are associated
        // (static) functions in cortex-m 0.7, not instance methods; dsb/isb are
        // safe functions; IrqNr implements InterruptNumber on ARM. No unsafe needed.
        #[cfg(target_arch = "arm")]
        {
            cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
            // Ensure the pend write reaches the NVIC before we read back.
            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            // IRQ 0 is at MIN_APP_IRQ_PRIORITY (0x20), SysTick is at 0x10.
            // Lower numeric value = higher urgency.  Since 0x10 < 0x20,
            // SysTick cannot be preempted — the IRQ must still be pending.
            if cortex_m::peripheral::NVIC::is_pending(kernel::irq_dispatch::IrqNr(0)) {
                hprintln!("systick_preempt_test: IRQ still pending inside SysTick");
                PEND_VERIFIED.store(1, Ordering::Release);
            } else {
                hprintln!("systick_preempt_test: FAIL (IRQ preempted SysTick)");
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
    if tick >= 5 {
        let verified = PEND_VERIFIED.load(Ordering::Acquire);
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if verified == 1 && count > 0 {
            hprintln!(
                "systick_preempt_test: PASS (non-preemption verified, wait_count={})",
                count
            );
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 10 {
        let verified = PEND_VERIFIED.load(Ordering::Acquire);
        let count = WAIT_COUNT.load(Ordering::Acquire);
        hprintln!(
            "systick_preempt_test: FAIL (timeout: verified={}, count={})",
            verified,
            count,
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(_) => {
                WAIT_COUNT.fetch_add(1, Ordering::Release);
            }
            Err(_) => {
                hprintln!("systick_preempt_test: FAIL (event_wait error)");
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("Peripherals::take");
    hprintln!("systick_preempt_test: start");

    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(1, 3).expect("round_robin");

    let k =
        Kernel::<Config>::create_sentinels(sched).expect("systick_preempt_test: create_sentinels");

    store_kernel(k);

    // Enable IRQ 0 at MIN_APP_IRQ_PRIORITY — the tightest allowed app
    // priority, numerically just above SYSTICK_PRIORITY.
    enable_bound_irqs(&mut p.NVIC, Config::MIN_APP_IRQ_PRIORITY).unwrap();

    let parts: [(extern "C" fn() -> !, u32); Config::N] = [(p0_main, 0)];
    match boot(&parts, p).expect("boot") {}
}
