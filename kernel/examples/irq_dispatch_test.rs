//! QEMU integration test: software-triggered IRQ dispatch.
//!
//! Validates the full ISR dispatch chain end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 0 → partition 0 with event bit 0x01.
//! 2. Partition 0 calls `event_wait(0x01)` via SVC in a loop.
//! 3. At tick 2, SysTick software-triggers IRQ 0 via `NVIC::pend`.
//! 4. The dispatch handler calls `signal_partition_from_isr`, setting
//!    event 0x01 on partition 0 and pending PendSV.
//! 5. The partition wakes, increments an atomic counter, and the
//!    SysTick harness observes the counter to confirm delivery.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example irq_dispatch_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::compose_kernel_config!(IrqTestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind IRQ 0 → partition 0, event bit 0x01.
// NOTE: the second argument (70) is the IRQ *count* (vector table size for
// the LM3S6965), NOT a priority.  The NVIC priority is set separately via
// `enable_bound_irqs`.
kernel::bind_interrupts!(IrqTestConfig, 70,
    0 => (0, 0x01),
);

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = IrqTestConfig::STACK_WORDS;

/// Incremented by the partition after each successful `SYS_EVT_WAIT` return.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(IrqTestConfig, |tick, _k| {
    if tick == 2 {
        // Software-trigger IRQ 0.  The dispatch handler will call
        // signal_partition_from_isr which sets event 0x01 on partition 0.
        //
        // IrqNr only implements InterruptNumber on ARM, so gate the
        // NVIC::pend call to keep `cargo check` happy on the host.
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("irq_dispatch_test: pended IRQ 0 at tick {}", tick);
    }
    if tick >= 4 {
        // Check the side-effect: the partition increments WAIT_COUNT after
        // each successful event_wait return.  This avoids racing with the
        // scheduler which may have already consumed the transient event flags.
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count > 0 {
            hprintln!("irq_dispatch_test: PASS (wait_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 10 {
        hprintln!("irq_dispatch_test: FAIL (event not delivered by tick 10)");
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        match plib::sys_event_wait(0x01) {
            Ok(_) => {
                WAIT_COUNT.fetch_add(1, Ordering::Release);
            }
            Err(e) => {
                hprintln!("irq_dispatch_test: FAIL (event_wait err={:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_dispatch_test: Peripherals::take");
    hprintln!("irq_dispatch_test: start");

    let sched = ScheduleTable::<{ IrqTestConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_dispatch_test: round_robin");

    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);

    let k =
        Kernel::<IrqTestConfig>::create(sched, &cfgs).expect("irq_dispatch_test: Kernel::create");

    store_kernel(k);

    // Unmask IRQ 0 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, IrqTestConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0)];
    match boot(&parts, &mut p).expect("irq_dispatch_test: boot") {}
}
