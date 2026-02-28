//! QEMU smoke test: boot kernel, run one partition, execute SYS_YIELD, exit.
//!
//! This is the minimal CI-friendly integration test.  It verifies the full
//! boot → schedule → syscall → exit path works end-to-end:
//!
//! 1. Kernel boots with one partition via `define_unified_harness!`.
//! 2. The scheduler dispatches the partition.
//! 3. The partition issues `SYS_YIELD` and records the return code.
//! 4. The SysTick hook observes the return code and exits via semihosting.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example qemu_smoke
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::syscall::SYS_YIELD;
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

kernel::kernel_config!(SmokeConfig {
    partitions = 1;
    sampling_msg_size = 1;
    blackboard_msg_size = 1;
});

/// Stores the SYS_YIELD return code (0xFFFF_FFFF = not yet called).
static YIELD_RC: AtomicU32 = AtomicU32::new(u32::MAX);

// TODO: hprintln!/debug::exit in SysTick ISR is heavy for an interrupt handler;
// acceptable for QEMU smoke tests but must not be used in production code.
kernel::define_unified_harness!(SmokeConfig, NUM_PARTITIONS, STACK_WORDS, |_tick, _k| {
    let rc = YIELD_RC.load(Ordering::Acquire);
    if rc != u32::MAX {
        if rc == 0 {
            hprintln!("qemu_smoke: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("qemu_smoke: FAIL yield rc={}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
});

extern "C" fn partition_main() -> ! {
    let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    YIELD_RC.store(rc, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("qemu_smoke: Peripherals::take");
    hprintln!("qemu_smoke: start");

    let mut sched = ScheduleTable::<{ SmokeConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("qemu_smoke: sched add");

    let cfgs: [PartitionConfig; NUM_PARTITIONS] =
        core::array::from_fn(|i| PartitionConfig::sentinel(i as u8, (STACK_WORDS * 4) as u32));

    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<SmokeConfig>::new(sched, &cfgs).expect("qemu_smoke: Kernel::new");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<SmokeConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("qemu_smoke: Kernel::new");

    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    match boot(&parts, &mut p).expect("qemu_smoke: boot") {}
}
