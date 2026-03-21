//! QEMU test: unbound-IRQ dispatch resilience.
//! Pends unbound IRQ 10 (tick 2) then bound IRQ 5 (tick 3).
//! Asserts exactly 1 event delivery at tick 8.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    UnboundConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// IRQ 5 → partition 0, event 0x01.  IRQ 10 intentionally unbound.
kernel::bind_interrupts!(UnboundConfig, 70,
    5 => (0, 0x01),
);

const NUM_PARTITIONS: usize = 1;

static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(UnboundConfig, |tick, _k| {
    if tick == 2 {
        // Software-pend unbound IRQ 10 — must be a safe no-op.
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(10));
        hprintln!("irq_unbound_test: pended unbound IRQ 10 at tick {}", tick);
    }
    if tick == 3 {
        // Software-pend bound IRQ 5 — delivers event 0x01 to partition 0.
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(5));
        hprintln!("irq_unbound_test: pended bound IRQ 5 at tick {}", tick);
    }
    if tick == 8 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count == 1 {
            hprintln!("irq_unbound_test: PASS (wait_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("irq_unbound_test: FAIL (wait_count={}, expected 1)", count);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick >= 12 {
        hprintln!("irq_unbound_test: FAIL (timeout)");
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(bits) if bits == plib::EventMask::new(0) => {} // Entered waiting state, no event yet
            Ok(bits) => {
                if bits != plib::EventMask::new(0x01) {
                    hprintln!(
                        "irq_unbound_test: FAIL (event_wait bits=0x{:08X}, expected 0x01)",
                        bits.as_raw()
                    );
                    debug::exit(debug::EXIT_FAILURE);
                }
                WAIT_COUNT.fetch_add(1, Ordering::Release);
            }
            Err(e) => {
                hprintln!("irq_unbound_test: FAIL (event_wait {:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_unbound_test: Peripherals::take");
    hprintln!("irq_unbound_test: start");

    let sched = ScheduleTable::<{ UnboundConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_unbound_test: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(p0_main, 0)];
    init_kernel(sched, &parts).expect("irq_unbound_test: init_kernel");

    // Unmask bound IRQs (IRQ 5).
    enable_bound_irqs(&mut p.NVIC, UnboundConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    // Unmask unbound IRQ 10 so its IVT handler fires (lookup_binding → None).
    #[cfg(target_arch = "arm")]
    // SAFETY: IRQ 10 is a valid external interrupt on LM3S6965 (the QEMU
    // target has 70 external IRQs, 0..69).  Priority is set at boot time
    // before interrupts are unmasked.  Unmasking an unbound IRQ is
    // intentional: the kernel's IVT handler must tolerate an empty
    // binding-table slot and treat it as a no-op.
    unsafe {
        p.NVIC.set_priority(kernel::irq_dispatch::IrqNr(10), 0xC0);
        cortex_m::peripheral::NVIC::unmask(kernel::irq_dispatch::IrqNr(10));
    }

    match boot(p).expect("irq_unbound_test: boot") {}
}
