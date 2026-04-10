//! QEMU test: `never_mask, handler:` combined binding variant.
//!
//! Binds IRQ 60 with `never_mask` + custom handler. The custom ISR increments
//! an atomic counter and signals the partition. Because NeverMask never disables
//! the IRQ line, the second pend fires without any ack call in between.
//! The partition verifies the counter increments on each IRQ pend.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt \
//!           --example irq_never_mask_handler_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, IsrHandler, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny,
    SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::kernel_config!(
    NeverMaskHandlerConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Incremented by the custom ISR each time it fires.
static ISR_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Incremented by the partition after each successful event_wait.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

/// # Safety
///
/// Placed in the vector table by `bind_interrupts!`. Runs in ISR context.
/// NeverMask means the kernel does NOT mask this IRQ — the handler must be
/// safe to re-enter or must complete before the next pend.
const _: IsrHandler = custom_irq60_handler;
unsafe extern "C" fn custom_irq60_handler() {
    ISR_COUNTER.fetch_add(1, Ordering::Release);
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<NeverMaskHandlerConfig>(0.into(), 0x01);
}

// Bind IRQ 60 -> partition 0, event bit 0x01, never_mask + custom handler.
kernel::bind_interrupts!(NeverMaskHandlerConfig, 70,
    60 => (0, 0x01, never_mask, handler: custom_irq60_handler),
);

const NUM_PARTITIONS: usize = 1;

kernel::define_kernel!(NeverMaskHandlerConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("never_mask_handler: pended IRQ 60 at tick {}", tick);
    }
    if tick == 4 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("never_mask_handler: pended IRQ 60 at tick {}", tick);
    }
    // After first pend: ISR counter == 1, partition has consumed event.
    if tick == 3 {
        let isr = ISR_COUNTER.load(Ordering::Acquire);
        if isr != 1 {
            hprintln!(
                "never_mask_handler: FAIL (expected isr==1 at tick 3, got {})",
                isr
            );
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    // After second pend: ISR counter == 2, no ack needed.
    if tick == 5 {
        let isr = ISR_COUNTER.load(Ordering::Acquire);
        if isr != 2 {
            hprintln!(
                "never_mask_handler: FAIL (expected isr==2 at tick 5, got {})",
                isr
            );
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick == 8 {
        let isr = ISR_COUNTER.load(Ordering::Acquire);
        let wait = WAIT_COUNT.load(Ordering::Acquire);
        if isr == 2 && wait == 2 {
            hprintln!("never_mask_handler: PASS (isr={}, wait={})", isr, wait);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let isr = ISR_COUNTER.load(Ordering::Acquire);
        let wait = WAIT_COUNT.load(Ordering::Acquire);
        hprintln!(
            "never_mask_handler: FAIL (isr={}, wait={}, expected 2 and 2)",
            isr,
            wait
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        let bits = match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(v) => v,
            Err(e) => {
                hprintln!("never_mask_handler: p0 evt_wait FAIL {:?}", e);
                debug::exit(debug::EXIT_FAILURE);
                continue;
            }
        };
        // event_wait returns 0 when entering Waiting state; skip that.
        if bits == plib::EventMask::new(0) {
            continue;
        }
        WAIT_COUNT.fetch_add(1, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("never_mask_handler: take");
    hprintln!("never_mask_handler: start");

    let mut sched = ScheduleTable::<{ NeverMaskHandlerConfig::SCHED }>::round_robin(1, 3)
        .expect("never_mask_handler: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    sched.add_system_window(1).expect("sys window");
    let mut k = init_kernel(sched, &parts).expect("never_mask_handler: init_kernel");
    store_kernel(&mut k);

    enable_bound_irqs(&mut p.NVIC, NeverMaskHandlerConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("never_mask_handler: boot") {}
}
