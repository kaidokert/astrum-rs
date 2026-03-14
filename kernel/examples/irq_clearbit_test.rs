//! QEMU integration test: KernelClears(ClearBit) dispatch path.
//!
//! Validates the KernelClears(ClearBit { addr, bit }) dispatch arm end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 60 → partition 0 with event bit 0x01
//!    using the 3-tuple form with `KernelClears(ClearBit)`.
//!    The target register is UART0 ICR (0x4000_C044) on lm3s6965evb,
//!    bit 4 — the kernel will write (1 << 4) to clear the source.
//!    We use IRQ 60 (unconnected in QEMU) to ensure software-pend fires
//!    exactly once with no level-sensitive re-assertion from hardware.
//! 2. Partition 0 loops on `event_wait(0x01)` — no SYS_IRQ_ACK needed
//!    because the kernel clears the interrupt source directly.
//! 3. The SysTick harness software-pends IRQ 60 at tick 2.
//! 4. At tick 6 the counter must be exactly 1, proving the kernel computed
//!    1<<bit, wrote it via write_volatile, signalled the partition, and
//!    cleared the IRQ (so it did not re-fire).
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt \
//!           --example irq_clearbit_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::irq_dispatch::{ClearStrategy, IrqClearModel};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::compose_kernel_config!(
    ClearBitConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Bind IRQ 60 → partition 0, event bit 0x01, KernelClears(ClearBit).
// UART0 ICR at 0x4000_C044, bit 4 — safe MMIO target for the write.
// Uses IRQ 60 (unconnected in QEMU) to ensure software-pend fires
// exactly once with no level-sensitive re-assertion from hardware.
kernel::bind_interrupts!(ClearBitConfig, 70,
    60 => (0, 0x01, IrqClearModel::KernelClears(
        ClearStrategy::ClearBit { addr: 0x4000_C044, bit: 4 },
    )),
);

const NUM_PARTITIONS: usize = 1;

/// Incremented by the partition after each successful event_wait.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(ClearBitConfig, |tick, _k| {
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("irq_clearbit_test: pended IRQ 60 at tick {}", tick);
    }
    if tick == 6 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count == 1 {
            hprintln!("irq_clearbit_test: PASS (wait_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 10 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        hprintln!(
            "irq_clearbit_test: FAIL (wait_count={}, expected ==1)",
            count
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        // No SYS_IRQ_ACK needed — kernel already cleared the source.
        match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(bits) if bits != plib::EventMask::new(0) => {
                WAIT_COUNT.fetch_add(1, Ordering::Release);
            }
            Ok(_) => {}
            Err(e) => {
                hprintln!("irq_clearbit_test: FAIL (event_wait err={:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_clearbit_test: Peripherals::take");
    hprintln!("irq_clearbit_test: start");

    let sched = ScheduleTable::<{ ClearBitConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_clearbit_test: round_robin");

    let k = Kernel::<ClearBitConfig>::create_sentinels(sched)
        .expect("irq_clearbit_test: Kernel::create");

    store_kernel(k);

    // Unmask IRQ 60 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, ClearBitConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0)];
    match boot(&parts, p).expect("irq_clearbit_test: boot") {}
}
