//! QEMU integration test: KernelClears dispatch path.
//!
//! Validates the KernelClears(WriteRegister) dispatch arm end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 5 → partition 0 with event bit 0x01
//!    using the 3-tuple form with `KernelClears(WriteRegister)`.
//!    The target register is UART0 ICR (0x4000_C044) on lm3s6965evb,
//!    which is safe to write in QEMU.
//! 2. Partition 0 loops on `event_wait(0x01)` — no SYS_IRQ_ACK needed
//!    because the kernel clears the interrupt source directly.
//! 3. The SysTick harness software-pends IRQ 5 at tick 2.
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
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::svc::{Kernel, SvcError};
use kernel::syscall::SYS_EVT_WAIT;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    KClearsConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Bind IRQ 5 → partition 0, event bit 0x01, KernelClears(WriteRegister).
// UART0 ICR at 0x4000_C044 on lm3s6965evb — safe to write in QEMU.
kernel::bind_interrupts!(KClearsConfig, 70,
    5 => (0, 0x01, IrqClearModel::KernelClears(
        ClearStrategy::WriteRegister { addr: 0x4000_C044, value: 0x7F0 },
    )),
);

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = KClearsConfig::STACK_WORDS;

/// Incremented by the partition after each successful event_wait.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(KClearsConfig, |tick, _k| {
    if tick == 2 {
        // NVIC::pend is a static method (no self receiver needed).
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(5));
        hprintln!("irq_kernel_clears_test: pended IRQ 5 at tick {}", tick);
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

extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x01u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("irq_kernel_clears_test: FAIL (event_wait rc=0x{:08X})", rc);
            debug::exit(debug::EXIT_FAILURE);
        }

        // No SYS_IRQ_ACK needed — kernel already cleared the source.
        WAIT_COUNT.fetch_add(1, Ordering::Release);
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_kernel_clears_test: Peripherals::take");
    hprintln!("irq_kernel_clears_test: start");

    let sched = ScheduleTable::<{ KClearsConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_kernel_clears_test: round_robin");

    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);

    let k = Kernel::<KClearsConfig>::create(sched, &cfgs)
        .expect("irq_kernel_clears_test: Kernel::create");

    // store_kernel, enable_bound_irqs, and boot are macro-generated.
    store_kernel(k);

    // Unmask IRQ 5 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, 0xC0);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0)];
    match boot(&parts, &mut p).expect("irq_kernel_clears_test: boot") {}
}
