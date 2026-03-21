//! QEMU test: `disable_bound_irqs` masks bound IRQs in the NVIC.
//!
//! Pends IRQ 0 → verifies dispatch → disables → re-pends → verifies
//! the ISR does NOT fire again (dispatch count unchanged).
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example irq_disable_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions1, PortsTiny, SyncMinimal};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::compose_kernel_config!(
    Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// SAFETY: vector-table entry; runs in ISR context.
unsafe extern "C" fn irq0_handler() {
    DISPATCH_COUNT.fetch_add(1, Ordering::Release);
    // `handler:` form bypasses __irq_dispatch, so manual signal + mask
    // are required here (see irq_ring_buffer_test.rs for the same pattern).
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<Cfg>(0, 0x01);
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(0));
}

kernel::bind_interrupts!(Cfg, 70, 0 => (0, 0x01, handler: irq0_handler));

/// ISR invocation count — only the custom handler writes this.
static DISPATCH_COUNT: AtomicU32 = AtomicU32::new(0);
/// 0 = not acked, 1 = partition acked, 2 = disable verified immediate.
static PHASE: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(Cfg, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("pended IRQ 0 at tick {}", tick);
    }
    // Wait for the full dispatch→ACK cycle, then disable and re-pend.
    if tick >= 5 && PHASE.load(Ordering::Acquire) == 1 {
        let before = DISPATCH_COUNT.load(Ordering::Acquire);
        if before >= 1 {
            disable_bound_irqs();
            #[cfg(target_arch = "arm")]
            cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
            hprintln!("disabled + re-pended at tick {} (count={})", tick, before);
            let after = DISPATCH_COUNT.load(Ordering::Acquire);
            if after != before {
                hprintln!("FAIL immediate ({} vs {})", before, after);
                debug::exit(debug::EXIT_FAILURE);
            }
            PHASE.store(2, Ordering::Release);
        }
    }
    // Deferred check: dispatch count still unchanged after several ticks.
    if tick >= 12 && PHASE.load(Ordering::Acquire) == 2 {
        let c = DISPATCH_COUNT.load(Ordering::Acquire);
        if c == 1 {
            hprintln!("PASS (dispatch_count=1 unchanged after disable)");
            debug::exit(debug::EXIT_SUCCESS);
        }
        hprintln!("FAIL deferred (dispatch_count={}, expected 1)", c);
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick >= 20 {
        hprintln!(
            "FAIL timeout (d={}, p={})",
            DISPATCH_COUNT.load(Ordering::Acquire),
            PHASE.load(Ordering::Acquire)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        // TODO: uses `if let Err` (discards Ok value) unlike match-based examples
        // that inspect the returned bits; both patterns are intentional.
        if let Err(e) = plib::sys_event_wait(plib::EventMask::new(0x01)) {
            hprintln!("FAIL (event_wait {:?})", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        // ACK once only — prevents re-unmasking after disable_bound_irqs.
        if PHASE.load(Ordering::Acquire) == 0 {
            if let Err(e) = plib::sys_irq_ack(0) {
                hprintln!("FAIL (irq_ack {:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
            PHASE.store(1, Ordering::Release);
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("take");
    hprintln!("irq_disable_test: start");
    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(1, 3).expect("sched");
    let parts: [PartitionSpec; 1] = [PartitionSpec::new(p0_main, 0)];
    init_kernel(sched, &parts).expect("irq_disable_test: init_kernel");
    enable_bound_irqs(&mut p.NVIC, Cfg::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("boot") {}
}
