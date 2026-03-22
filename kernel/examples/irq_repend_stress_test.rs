//! QEMU stress test: rapid IRQ re-pend under sustained load.
//!
//! Validates the PartitionAcks lifecycle under continuous interrupt pressure:
//!
//! 1. `bind_interrupts!` maps IRQ 0 → partition 0, event bit 0x01.
//! 2. SysTick hook pends IRQ 0 on **every** tick >= 2 (sustained pressure).
//! 3. Partition loops: `event_wait(0x01)` → `SYS_IRQ_ACK` → increment counter.
//! 4. Test passes when counter >= 5 (proving 5 consecutive mask/ack/unmask/re-pend
//!    cycles), or fails at tick 30 if the counter hasn't reached the threshold.
//!
//! This catches edge cases in event-bit accumulation, NVIC mask/unmask sequencing,
//! and timing interactions between SysTick ticks and IRQ delivery.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example irq_repend_stress_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionBody, PartitionEntry, PartitionSpec, Partitions1, PortsTiny,
    SyncMinimal,
};

kernel::compose_kernel_config!(StressConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind IRQ 0 → partition 0, event bit 0x01 (PartitionAcks default).
kernel::bind_interrupts!(StressConfig, 70,
    0 => (0, 0x01),
);

const NUM_PARTITIONS: usize = 1;
const PASS_THRESHOLD: u32 = 5;
const TIMEOUT_TICK: u32 = 30;

/// Incremented by the partition after each successful wait+ack cycle.
static ACK_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(StressConfig, |tick, _k| {
    // Pend IRQ 0 on every tick from tick 2 onward (continuous pressure).
    if tick >= 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
    }
    // Check for pass: counter reached threshold.
    // Pends start at tick 2; 5 ack cycles need at least 5 ticks → earliest at tick 7.
    if tick >= 7 {
        let count = ACK_COUNT.load(Ordering::Acquire);
        if count >= PASS_THRESHOLD {
            hprintln!("irq_repend_stress: PASS (ack_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    // Timeout: fail if threshold not reached.
    if tick >= TIMEOUT_TICK {
        let count = ACK_COUNT.load(Ordering::Acquire);
        hprintln!(
            "irq_repend_stress: FAIL (ack_count={}, expected >={})",
            count,
            PASS_THRESHOLD
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

fn unwrap_or_fail<T>(r: Result<T, plib::SvcError>, ctx: &str) {
    if let Err(e) = r {
        hprintln!("{}: FAIL ({:?})", ctx, e);
        debug::exit(debug::EXIT_FAILURE);
    }
}

// TODO: consider moving error reporting to the harness to reduce partition noise.
const _: PartitionBody = p0_main_body;
extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(0x01)),
            "irq_repend_stress: event_wait",
        );

        // Acknowledge (unmask) IRQ 0 so the next pend fires.
        unwrap_or_fail(plib::sys_irq_ack(0), "irq_repend_stress: irq_ack");

        // Only count after both syscalls succeeded.
        ACK_COUNT.fetch_add(1, Ordering::Release);
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_repend_stress: Peripherals::take");
    hprintln!("irq_repend_stress: start");

    let sched = ScheduleTable::<{ StressConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_repend_stress: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    init_kernel(sched, &parts).expect("irq_repend_stress_test: init_kernel");

    // Unmask IRQ 0 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, StressConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("irq_repend_stress: boot") {}
}
