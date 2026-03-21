//! QEMU test: mixed PartitionAcks + KernelClears in one binding table.
//! IRQ 5 → P0 (PartitionAcks, 0x01), IRQ 6 → P1 (KernelClears(WriteRegister), 0x02).
//! P0 loops event_wait + SYS_IRQ_ACK. P1 loops event_wait only.
//! Pends at ticks 2-3; by tick 8 both counters must be nonzero.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::irq_dispatch::{ClearStrategy, IrqClearModel};
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions2, PortsTiny, SyncMinimal};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::compose_kernel_config!(
    MixedConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// IRQ 5 → P0, PartitionAcks (2-tuple default); IRQ 6 → P1, KernelClears(WriteRegister).
// UART0 ICR at 0x4000_C044 on lm3s6965evb — safe MMIO target for the write.
kernel::bind_interrupts!(MixedConfig, 70,
    5 => (0, 0x01),
    6 => (1, 0x02, IrqClearModel::KernelClears(
        ClearStrategy::WriteRegister { addr: 0x4000_C044, value: 0x7F0 },
    )),
);

const NUM_PARTITIONS: usize = 2;
static P0_COUNT: AtomicU32 = AtomicU32::new(0);
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(MixedConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(5));
        hprintln!("irq_mixed_model_test: pended IRQ 5 at tick {}", tick);
    }
    if tick == 3 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(6));
        hprintln!("irq_mixed_model_test: pended IRQ 6 at tick {}", tick);
    }
    if tick == 8 {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        if c0 >= 1 && c1 >= 1 {
            hprintln!("irq_mixed_model_test: PASS (p0={}, p1={})", c0, c1);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let c0 = P0_COUNT.load(Ordering::Acquire);
        let c1 = P1_COUNT.load(Ordering::Acquire);
        hprintln!(
            "irq_mixed_model_test: FAIL (p0={}, p1={}, expected both >=1)",
            c0,
            c1
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition 0: PartitionAcks — event_wait + SYS_IRQ_ACK loop.
extern "C" fn p0_main() -> ! {
    loop {
        if let Err(e) = plib::sys_event_wait(plib::EventMask::new(0x01)) {
            hprintln!("irq_mixed_model_test: p0 FAIL (evt_wait {:?})", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        if let Err(e) = plib::sys_irq_ack(5) {
            hprintln!("irq_mixed_model_test: p0 FAIL (irq_ack {:?})", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        P0_COUNT.fetch_add(1, Ordering::Release);
    }
}

/// Partition 1: KernelClears — event_wait only, no ack needed.
extern "C" fn p1_main() -> ! {
    loop {
        if let Err(e) = plib::sys_event_wait(plib::EventMask::new(0x02)) {
            hprintln!("irq_mixed_model_test: p1 FAIL (evt_wait {:?})", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        P1_COUNT.fetch_add(1, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_mixed_model_test: Peripherals::take");
    hprintln!("irq_mixed_model_test: start");

    let sched = ScheduleTable::<{ MixedConfig::SCHED }>::round_robin(2, 3)
        .expect("irq_mixed_model_test: round_robin");
    let parts: [PartitionSpec; NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    init_kernel(sched, &parts).expect("irq_mixed_model_test: init_kernel");
    enable_bound_irqs(&mut p.NVIC, MixedConfig::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("irq_mixed_model_test: boot") {}
}
