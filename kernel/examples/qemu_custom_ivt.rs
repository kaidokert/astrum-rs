//! QEMU integration test for `bind_interrupts!` with a custom IVT.
//!
//! Validates the end-to-end flow:
//!
//! 1. `bind_interrupts!` generates a custom `__INTERRUPTS` vector table.
//! 2. The `custom-ivt` feature suppresses the default IVT (no duplicate symbol).
//! 3. The kernel boots normally with the macro-generated vector table.
//! 4. SysTick verifies the system is running and exits with `EXIT_SUCCESS`.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,custom-ivt --example qemu_custom_ivt
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

kernel::compose_kernel_config!(IvtConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Bind two IRQs to partition 0.  The handlers will never fire (no
// hardware triggers them in QEMU), but the vector table must link
// without duplicate-symbol or SIZEOF assertion errors.
kernel::bind_interrupts!(IvtConfig, 70,
    0 => (0, 0x01),
    1 => (0, 0x02),
);

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = IvtConfig::STACK_WORDS;
const PASS_TICK: u32 = 5;

/// Tick counter used by SysTick hook to verify liveness.
static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(IvtConfig, |tick, _k| {
    TICK_COUNT.store(tick, Ordering::Release);
    if tick >= PASS_TICK {
        hprintln!("qemu_custom_ivt: PASS (tick={})", tick);
        debug::exit(debug::EXIT_SUCCESS);
    }
});

extern "C" fn p0_main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("qemu_custom_ivt: Peripherals::take");
    hprintln!("qemu_custom_ivt: start");

    let sched = ScheduleTable::<{ IvtConfig::SCHED }>::round_robin(1, 3)
        .expect("qemu_custom_ivt: round_robin");

    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);

    let k = Kernel::<IvtConfig>::create(sched, &cfgs).expect("qemu_custom_ivt: Kernel::create");

    store_kernel(k);

    // Enable all IRQs bound by bind_interrupts! at the configured default priority.
    enable_bound_irqs(&mut p.NVIC, IvtConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0)];
    match boot(&parts, &mut p).expect("qemu_custom_ivt: boot") {}
}
