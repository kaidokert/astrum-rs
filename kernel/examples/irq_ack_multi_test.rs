//! QEMU test: two-partition independent IRQ ack cycles (PartitionAcks).
//! IRQ 0 → P0, IRQ 1 → P1. Each loops: event_wait → SYS_IRQ_ACK → count.
//! Pends at ticks 2, 3, 6; by tick 12 both counters must be ≥ 2.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(AckMultiConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

kernel::bind_interrupts!(AckMultiConfig, 70,
    0 => (0, 0x01),
    1 => (1, 0x02),
);

const NUM_PARTITIONS: usize = 2;
static P0_ACK: AtomicU32 = AtomicU32::new(0);
static P1_ACK: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(AckMultiConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("irq_ack_multi_test: pended IRQ 0 at tick {}", tick);
    }
    if tick == 3 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(1));
        hprintln!("irq_ack_multi_test: pended IRQ 1 at tick {}", tick);
    }
    if tick == 6 {
        #[cfg(target_arch = "arm")]
        {
            cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
            cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(1));
        }
        hprintln!("irq_ack_multi_test: re-pended IRQ 0+1 at tick {}", tick);
    }
    if tick >= 8 {
        let c0 = P0_ACK.load(Ordering::Acquire);
        let c1 = P1_ACK.load(Ordering::Acquire);
        if c0 >= 2 && c1 >= 2 {
            hprintln!("irq_ack_multi_test: PASS (p0={}, p1={})", c0, c1);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let c0 = P0_ACK.load(Ordering::Acquire);
        let c1 = P1_ACK.load(Ordering::Acquire);
        hprintln!(
            "irq_ack_multi_test: FAIL (p0={}, p1={}, expected >=2)",
            c0,
            c1
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

fn unwrap_or_fail<T>(r: Result<T, plib::SvcError>, tag: &str, op: &str) {
    if let Err(e) = r {
        hprintln!("irq_ack_multi_test: {} FAIL ({} {:?})", tag, op, e);
        debug::exit(debug::EXIT_FAILURE);
    }
}

fn ack_loop(evt: u32, irq: u8, ctr: &AtomicU32, tag: &str) -> ! {
    loop {
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(evt)),
            tag,
            "evt_wait",
        );
        unwrap_or_fail(plib::sys_irq_ack(irq), tag, "irq_ack");
        ctr.fetch_add(1, Ordering::Release);
    }
}

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    ack_loop(0x01, 0, &P0_ACK, "p0")
}
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    ack_loop(0x02, 1, &P1_ACK, "p1")
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_ack_multi_test: Peripherals::take");
    hprintln!("irq_ack_multi_test: start");

    let sched = ScheduleTable::<{ AckMultiConfig::SCHED }>::round_robin(2, 3)
        .expect("irq_ack_multi_test: round_robin");
    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main, 0),
        PartitionSpec::new(p1_main, 0),
    ];
    init_kernel(sched, &parts).expect("irq_ack_multi_test: init_kernel");
    enable_bound_irqs(&mut p.NVIC, AckMultiConfig::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("irq_ack_multi_test: boot") {}
}
