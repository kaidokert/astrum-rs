//! QEMU test: event set / wait / clear lifecycle across two partitions.
//!
//! P0 sets bits 0-2 on P1 via plib::sys_event_set.
//! P1 waits for bit 2 only — event_wait returns all pending bits (0x07),
//! clears matched bit 2, leaving bits 0-1 pending.
//! P1 then clears bit 0 (event_clear returns previous flags 0x03),
//! and verifies bit 1 survived via a second wait.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example plib_event_test
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
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;
const NOT_YET: u32 = 0xDEAD_C0DE;

static SET_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static WAIT_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static CLEAR_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static REMAIN_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let set = SET_RC.load(Ordering::Acquire);
    let wait = WAIT_RC.load(Ordering::Acquire);
    let clear = CLEAR_RC.load(Ordering::Acquire);
    let remain = REMAIN_RC.load(Ordering::Acquire);

    if set == NOT_YET || wait == NOT_YET || clear == NOT_YET || remain == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_event_test: FAIL timeout (set={:#x}, wait={:#x}, clear={:#x}, remain={:#x})",
                set,
                wait,
                clear,
                remain
            );
            kernel::kexit!(failure);
        }
        return;
    }

    // event_set → 0 (success); event_wait(0x04) → 0x07 (all pending bits);
    // event_clear(0x01) → 0x03 (previous flags); event_wait(0x02) → 0x02 (bit 1 survived).
    // TODO: subtask originally specified wait(0x07)->clear(0x03)->verify bit 2,
    // but that sequence is internally inconsistent (wait(0x07) clears all bits).
    // Using wait(0x04)->clear(0x01)->verify bit 1 which is logically correct.
    if set == 0 && wait == 0x07 && clear == 0x03 && remain == 0x02 {
        hprintln!(
            "plib_event_test: PASS (set={}, wait={:#x}, clear={}, remain={:#x})",
            set,
            wait,
            clear,
            remain
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_event_test: FAIL (set={:#x}, wait={:#x}, clear={:#x}, remain={:#x})",
            set,
            wait,
            clear,
            remain
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn p0_main() -> ! {
    // Set bits 0-2 on P1 (partition index 1).
    match plib::sys_event_set(plib::PartitionId::new(1), plib::EventMask::new(0x07)) {
        Ok(rc) => SET_RC.store(rc, Ordering::Release),
        Err(_) => SET_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    // Wait for bit 2 only — event_wait clears matched bits, so bits 0-1
    // (0x03) remain in P1's pending flags after this call.
    match plib::sys_event_wait(plib::EventMask::new(0x04)) {
        Ok(rc) => WAIT_RC.store(rc.as_raw(), Ordering::Release),
        Err(_) => WAIT_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    // Clear bit 0 — bit 1 should survive.
    match plib::sys_event_clear(plib::EventMask::new(0x01)) {
        Ok(rc) => CLEAR_RC.store(rc.as_raw(), Ordering::Release),
        Err(_) => CLEAR_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    // Verify bit 1 survived by waiting on it — should return immediately.
    match plib::sys_event_wait(plib::EventMask::new(0x02)) {
        Ok(rc) => REMAIN_RC.store(rc.as_raw(), Ordering::Release),
        Err(_) => REMAIN_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_event_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_event_test: round_robin");

    let k = Kernel::<TestConfig>::create_sentinels(sched).expect("plib_event_test: kernel");
    store_kernel(k);

    match boot(&[(p0_main, 0), (p1_main, 0)], &mut p).expect("plib_event_test: boot") {}
}
