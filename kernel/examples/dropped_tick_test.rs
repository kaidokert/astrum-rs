//! Dropped-tick detection QEMU integration test.
//!
//! Uses an ultra-fast SysTick period (tick_period_us = 1, yielding only
//! 12 reload cycles at 12 MHz) so that the SysTick handler + PendSV
//! execution time naturally exceeds the tick period, causing SysTick to
//! re-pend before the current handler completes.  The harness-level
//! `_detect_dropped_ticks!` macro (called automatically in the SysTick
//! handler before tick processing) reads the ICSR PENDSTSET bit and
//! increments `ticks_dropped` when set.  The test verifies that
//! `ticks_dropped > 0` after sufficient ticks, proving end-to-end
//! dropped-tick detection under real timing pressure.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example dropped_tick_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

// Ultra-fast SysTick: 12 MHz * 1 µs / 1e6 = 12 cycles per tick.
// At only 12 cycles the SysTick handler body (dropped-tick detection,
// schedule advance, klog, user hook) takes far longer than one period,
// so SysTick naturally re-pends during execution — exactly the condition
// that _detect_dropped_ticks! is designed to catch.
kernel::compose_kernel_config!(
    TestConfig < Partitions1,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 1;
    }
);

/// Tick at which we first check for detected dropped ticks.
const CHECK_TICK: u32 = 10;
/// Hard timeout to avoid hanging if detection never fires.
const TIMEOUT_TICKS: u32 = 200;

kernel::define_unified_harness!(TestConfig, |tick, k| {
    // The harness already calls _detect_dropped_ticks!(k) in the SysTick
    // handler before this hook runs (harness.rs:408).  We just need to
    // check the counter — no manual ICSR manipulation required.
    if tick >= CHECK_TICK {
        let dropped = k.ticks_dropped();
        if dropped > 0 {
            hprintln!("dropped_tick_test: PASS (ticks_dropped={})", dropped);
            kernel::kexit!(success);
        }
    }
    if tick >= TIMEOUT_TICKS {
        hprintln!(
            "dropped_tick_test: FAIL (ticks_dropped=0 after {} ticks)",
            tick
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn partition_main() -> ! {
    // Busy work to add execution time pressure, increasing the chance
    // that SysTick re-pends before the handler completes.
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("dropped_tick_test: Peripherals::take");
    hprintln!("dropped_tick_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(1, 3)
        .expect("dropped_tick_test: round_robin");

    let k =
        Kernel::<TestConfig>::create_sentinels(sched).expect("dropped_tick_test: create_sentinels");

    // store_kernel and boot are provided by the define_unified_harness! macro.
    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); TestConfig::N] = [(partition_main, 0)];
    match boot(&parts, p).expect("dropped_tick_test: boot") {}
}
