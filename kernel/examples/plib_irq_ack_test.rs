//! QEMU integration test: plib `sys_irq_ack` wrapper full-cycle.
//!
//! Validates the plib IRQ-acknowledge path end-to-end using IRQ 3:
//!
//! 1. `bind_interrupts!` maps IRQ 3 → partition 0 with event bit 0x01
//!    (default `PartitionAcks` clear model).
//! 2. Partition loops: `sys_event_wait(0x01)` → check Ok →
//!    `sys_irq_ack(3)` → check Ok(0) → increment counter.
//! 3. SysTick hook pends IRQ 3 at ticks 2, 6, and 10 (three cycles).
//! 4. By tick 14 the counter must be == 3, proving all three IRQs were
//!    dispatched, masked, acknowledged (unmasked via plib wrapper), and
//!    re-dispatched without loss or double-counting.
//!
//! Run:  cargo run --target thumbv7m-none-eabi \
//!         --features qemu,log-semihosting,custom-ivt \
//!         --example plib_irq_ack_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Bind IRQ 3 → partition 0, event bit 0x01 (PartitionAcks default).
kernel::bind_interrupts!(TestConfig, 70,
    3 => (0, 0x01),
);

const IRQ_NUM: u8 = 3;
const EVENT_BIT: u32 = 0x01;

/// Incremented by the partition after each successful wait+ack cycle.
static ACK_COUNT: AtomicU32 = AtomicU32::new(0);
/// Last return code from sys_irq_ack (should be 0 on success).
static LAST_ACK_RC: AtomicU32 = AtomicU32::new(0xFFFF_FFFF);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    // Pend IRQ 3 at ticks 2, 6, and 10 to drive three ack cycles.
    if tick == 2 || tick == 6 || tick == 10 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(IRQ_NUM));
        hprintln!("plib_irq_ack_test: pended IRQ {} at tick {}", IRQ_NUM, tick);
    }
    if tick >= 14 {
        let count = ACK_COUNT.load(Ordering::Acquire);
        let ack_rc = LAST_ACK_RC.load(Ordering::Acquire);

        if count == 3 && ack_rc == 0 {
            hprintln!(
                "plib_irq_ack_test: PASS (ack_count={}, ack_rc={})",
                count,
                ack_rc
            );
            kernel::kexit!(success);
        }
    }
    if tick >= 20 {
        let count = ACK_COUNT.load(Ordering::Acquire);
        let ack_rc = LAST_ACK_RC.load(Ordering::Acquire);
        hprintln!(
            "plib_irq_ack_test: FAIL (count={}, ack_rc={:#x})",
            count,
            ack_rc
        );
        kernel::kexit!(failure);
    }
});

fn unwrap_or_fail<T>(r: Result<T, plib::SvcError>, ctx: &str) -> T {
    match r {
        Ok(v) => v,
        Err(e) => {
            hprintln!("{}: FAIL ({:?})", ctx, e);
            kernel::kexit!(failure);
            #[allow(clippy::empty_loop)]
            loop {} // unreachable — kexit terminates QEMU
        }
    }
}

extern "C" fn partition_main() -> ! {
    loop {
        // Wait for event bit signalled by the IRQ dispatch handler.
        unwrap_or_fail(
            plib::sys_event_wait(plib::EventMask::new(EVENT_BIT)),
            "plib_irq_ack_test: event_wait",
        );

        // Acknowledge (unmask) IRQ so it can fire again.
        let rc = unwrap_or_fail(plib::sys_irq_ack(IRQ_NUM), "plib_irq_ack_test: irq_ack");
        LAST_ACK_RC.store(rc, Ordering::Release);

        // Count AFTER both syscalls verified OK.
        ACK_COUNT.fetch_add(1, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_irq_ack_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(1, 3).expect("round_robin");
    let parts: [PartitionSpec; TestConfig::N] = [(partition_main, 0)];
    init_kernel(sched, &parts).expect("kernel");

    enable_bound_irqs(&mut p.NVIC, TestConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("boot") {}
}
