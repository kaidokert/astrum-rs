//! QEMU integration test: KernelClears dispatch path.
//!
//! Validates the KernelClears(WriteRegister) dispatch arm end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 60 → partition 0 with event bit 0x01
//!    using the 3-tuple form with `KernelClears(WriteRegister)`.
//!    IRQ 60 is unconnected in QEMU — we software-pend it so it fires
//!    exactly once with no level-sensitive re-assertion from hardware.
//!    The `WriteRegister` target is UART0 ICR (0x4000_C044), used only as
//!    a safe MMIO side-effect destination (it has no relationship to
//!    IRQ 60; we just need a harmless register the kernel can write).
//! 2. Partition 0 loops on `event_wait(0x01)` — no SYS_IRQ_ACK needed
//!    because the kernel clears the interrupt source directly.
//! 3. The SysTick harness software-pends IRQ 60 at tick 2.
//! 4. At tick 6 the counter must be exactly 1, proving the kernel executed
//!    the write_volatile, signalled the partition, and cleared the IRQ
//!    (so it did not re-fire).
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt \
//!           --example irq_kernel_clears_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::irq_dispatch::{ClearStrategy, IrqClearModel};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionBody, PartitionEntry, PartitionSpec, Partitions1, PortsTiny,
    SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::kernel_config!(
    KClearsConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Bind IRQ 60 (unconnected in QEMU, software-pend only) → partition 0,
// event bit 0x01, KernelClears(WriteRegister).  UART0 ICR (0x4000_C044)
// is used as a harmless MMIO side-effect target — not related to IRQ 60.
kernel::bind_interrupts!(KClearsConfig, 70,
    60 => (0, 0x01, IrqClearModel::KernelClears(
        ClearStrategy::WriteRegister { addr: 0x4000_C044, value: 0x7F0 },
    )),
);

const NUM_PARTITIONS: usize = 1;

/// Incremented by the partition after each successful event_wait.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(KClearsConfig, |tick, _k| {
    if tick == 2 {
        // NVIC::pend is a static method (no self receiver needed).
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("irq_kernel_clears_test: pended IRQ 60 at tick {}", tick);
    }
    if tick == 6 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count == 1 {
            hprintln!("irq_kernel_clears_test: PASS (wait_count={})", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 10 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        hprintln!(
            "irq_kernel_clears_test: FAIL (wait_count={}, expected ==1)",
            count
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionBody = p0_main_body;
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
                hprintln!("irq_kernel_clears_test: FAIL (event_wait err={:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_kernel_clears_test: Peripherals::take");
    hprintln!("irq_kernel_clears_test: start");

    let sched = ScheduleTable::<{ KClearsConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_kernel_clears_test: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    init_kernel(sched, &parts).expect("irq_kernel_clears_test: init_kernel");

    // Unmask IRQ 60 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, KClearsConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("irq_kernel_clears_test: boot") {}
}
