//! QEMU test: `handler:` binding variant.
//! IRQ 5 → P0 custom handler (0x01), IRQ 6 → P1 standard dispatch (0x02).
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::svc::{Kernel, SvcError};
use kernel::syscall::{SYS_EVT_WAIT, SYS_IRQ_ACK};
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    CustomHandlerConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// SAFETY: This function is placed in the Cortex-M vector table by
// bind_interrupts! and is only called by hardware as an exception entry point.
// It runs in ISR context with interrupts of equal/lower priority masked.
unsafe extern "C" fn custom_irq5_handler() {
    ISR_COUNTER.fetch_add(1, Ordering::Relaxed);
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<CustomHandlerConfig>(0, 0x01);
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(5));
}

kernel::bind_interrupts!(CustomHandlerConfig, 70,
    5 => (0, 0x01, handler: custom_irq5_handler),
    6 => (1, 0x02),
);

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = CustomHandlerConfig::STACK_WORDS;

static ISR_COUNTER: AtomicU32 = AtomicU32::new(0);
static P0_COUNT: AtomicU32 = AtomicU32::new(0);
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(CustomHandlerConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(5));
        hprintln!("custom_handler: pended IRQ 5 at tick {}", tick);
    }
    if tick == 3 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(6));
        hprintln!("custom_handler: pended IRQ 6 at tick {}", tick);
    }
    if tick == 8 {
        let isr = ISR_COUNTER.load(Ordering::Relaxed);
        let c0 = P0_COUNT.load(Ordering::Relaxed);
        let c1 = P1_COUNT.load(Ordering::Relaxed);
        if isr == 1 && c0 == 1 && c1 == 1 {
            hprintln!("custom_handler: PASS (isr={}, p0={}, p1={})", isr, c0, c1);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let isr = ISR_COUNTER.load(Ordering::Relaxed);
        let c0 = P0_COUNT.load(Ordering::Relaxed);
        let c1 = P1_COUNT.load(Ordering::Relaxed);
        hprintln!("custom_handler: FAIL (isr={}, p0={}, p1={})", isr, c0, c1);
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x01u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("custom_handler: p0 evt_wait FAIL 0x{:08X}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        let rc = kernel::svc!(SYS_IRQ_ACK, 5u32, 0u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("custom_handler: p0 irq_ack FAIL 0x{:08X}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        P0_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x02u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("custom_handler: p1 evt_wait FAIL 0x{:08X}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        let rc = kernel::svc!(SYS_IRQ_ACK, 6u32, 0u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("custom_handler: p1 irq_ack FAIL 0x{:08X}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        P1_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("custom_handler: take");
    hprintln!("custom_handler: start");
    let sched = ScheduleTable::<{ CustomHandlerConfig::SCHED }>::round_robin(2, 3)
        .expect("custom_handler: round_robin");
    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);
    let k = Kernel::<CustomHandlerConfig>::create(sched, &cfgs).expect("custom_handler: create");
    store_kernel(k);
    enable_bound_irqs(&mut p.NVIC, CustomHandlerConfig::IRQ_DEFAULT_PRIORITY);
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, &mut p).expect("custom_handler: boot") {}
}
