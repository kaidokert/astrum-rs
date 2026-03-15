//! Minimal test for define_unified_harness!
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// TODO: reviewer false positive — NUM_PARTITIONS is used in the `parts` array type below.
const NUM_PARTITIONS: usize = 1;

kernel::define_unified_harness!(TestConfig);

// Atomic flag for partition to signal it ran (semihosting requires privileged mode)
static PARTITION_RAN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

extern "C" fn partition_main() -> ! {
    // Set flag instead of using semihosting (which requires privileged mode)
    PARTITION_RAN.store(true, core::sync::atomic::Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("unified_minimal: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");

    let k = Kernel::<TestConfig>::create_sentinels(sched).expect("kernel");

    store_kernel(k);
    hprintln!("unified_minimal: kernel stored");

    hprintln!("unified_minimal: calling boot");
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    match boot(&parts, p).expect("unified_minimal: boot failed") {}
}
