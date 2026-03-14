//! QEMU integration test: IRQ ack full-cycle (PartitionAcks model).
//!
//! Validates the complete PartitionAcks cycle end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 0 → partition 0 with event bit 0x01
//!    using the default 2-tuple form (`PartitionAcks` clear model).
//! 2. Partition 0 loops: `event_wait(0x01)` → `SYS_IRQ_ACK` → increment
//!    counter.  The ack syscall unmasks the IRQ so it can fire again.
//! 3. The SysTick harness pends IRQ 0 at tick 2 and again at tick 6
//!    (after the first ack re-enables it).
//! 4. By tick 12 the counter must be >= 2, proving the IRQ was dispatched,
//!    masked, acknowledged (unmasked), and dispatched again.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt --example irq_ack_test
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

kernel::compose_kernel_config!(AckTestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind IRQ 0 → partition 0, event bit 0x01 (PartitionAcks default).
kernel::bind_interrupts!(AckTestConfig, 70,
    0 => (0, 0x01),
);

const NUM_PARTITIONS: usize = 1;

/// Incremented by the partition after each successful wait+ack cycle.
static ACK_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(AckTestConfig, |tick, _k| {
    if tick == 2 || tick == 6 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("irq_ack_test: pended IRQ 0 at tick {}", tick);
    }
    // TODO: verify IRQ delivery timing, not just activity counter, to confirm
    // the IRQ actually fired at the expected ticks.
    if tick >= 8 {
        let count = ACK_COUNT.load(Ordering::Acquire);
        if count >= 2 {
            hprintln!("irq_ack_test: PASS (ack_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let count = ACK_COUNT.load(Ordering::Acquire);
        hprintln!("irq_ack_test: FAIL (ack_count={}, expected >=2)", count);
        debug::exit(debug::EXIT_FAILURE);
    }
});

fn unwrap_or_fail<T>(r: Result<T, plib::SvcError>, ctx: &str) {
    if let Err(e) = r {
        hprintln!("{}: FAIL ({:?})", ctx, e);
        debug::exit(debug::EXIT_FAILURE);
    }
}

extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(0x01)),
            "irq_ack_test: event_wait",
        );

        // Acknowledge (unmask) IRQ 0 so it can fire again.
        unwrap_or_fail(plib::sys_irq_ack(0), "irq_ack_test: irq_ack");

        // Only count after both syscalls succeeded.
        ACK_COUNT.fetch_add(1, Ordering::Release);
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_ack_test: Peripherals::take");
    hprintln!("irq_ack_test: start");

    let sched = ScheduleTable::<{ AckTestConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_ack_test: round_robin");

    let k = Kernel::<AckTestConfig>::create_sentinels(sched).expect("irq_ack_test: Kernel::create");

    store_kernel(k);

    // Unmask IRQ 0 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, AckTestConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0)];
    match boot(&parts, p).expect("irq_ack_test: boot") {}
}
