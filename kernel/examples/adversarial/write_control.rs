//! Adversarial test: unprivileged partition attempts to clear CONTROL.nPRIV.
//!
//! On Cortex-M3 in Thread mode with nPRIV=1, the nPRIV bit is sticky —
//! unprivileged code cannot set itself back to privileged mode. Writing
//! CONTROL with nPRIV=0 is silently ignored (no fault, but no effect).
//!
//! This test verifies:
//! 1. Partition boots in unprivileged mode (nPRIV=1)
//! 2. Partition attempts MSR CONTROL, r0 with nPRIV=0
//! 3. No fault occurs
//! 4. CONTROL.nPRIV still reads as 1 after the attempt
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example write_control

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::{
    kpanic as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    AlignedStack1K, DebugEnabled, MsgMinimal, Partitions1, PortsMinimal, StackStorage as _,
    SyncMinimal,
};

/// Test name for reporting.
const TEST_NAME: &str = "write_control";

kernel::compose_kernel_config!(TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsMinimal, DebugEnabled>);

const NUM_PARTITIONS: usize = TestConfig::N;

// 0 = pending, 1 = pass, 2 = fail (partition not unpriv), 3 = fail (escalated)
static RESULT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == 1 {
        hprintln!("{}: nPRIV unchanged (write ignored) — PASS", TEST_NAME);
        debug::exit(debug::EXIT_SUCCESS);
    } else if r == 2 {
        hprintln!("{}: FAIL - not unprivileged at start", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    } else if r == 3 {
        hprintln!("{}: FAIL - privilege escalation", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick > 100 {
        hprintln!("{}: FAIL - timeout", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition entry: attempts privilege escalation, stores result in atomic.
extern "C" fn partition_main() -> ! {
    let control_before: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control_before);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control_before = 3;
    }
    if control_before & 1 != 1 {
        RESULT.store(2, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }
    #[cfg(target_arch = "arm")]
    let control_write = control_before & !1u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("msr CONTROL, {0}", "isb", in(reg) control_write);
    }
    let control_after: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control_after);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control_after = 3;
    }
    RESULT.store(
        if control_after & 1 == 1 { 1 } else { 3 },
        Ordering::Release,
    );
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals already taken");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("static schedule entry must fit");

    let mut stack0 = AlignedStack1K::ZERO;
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mem0 = ExternalPartitionMemory::new(
        &mut stack0.0,
        partition_main as *const () as u32,
        sentinel_mpu,
        0,
    )
    .expect("ext mem");
    let mems: [ExternalPartitionMemory; NUM_PARTITIONS] = [mem0];

    let k = Kernel::<TestConfig>::new(sched, &mems).expect("kernel creation");

    store_kernel(k);

    match boot(p).expect("write_control: boot failed") {}
}
