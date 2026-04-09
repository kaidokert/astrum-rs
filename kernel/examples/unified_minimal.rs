//! Minimal test for define_kernel!
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
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Atomic flag for partition to signal it ran (semihosting requires privileged mode)
static PARTITION_RAN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    // Set flag instead of using semihosting (which requires privileged mode)
    PARTITION_RAN.store(true, core::sync::atomic::Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

fn make_schedule() -> ScheduleTable<{ TestConfig::SCHED }> {
    let mut sched = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched
}

kernel::define_kernel!(
    TestConfig,
    make_schedule(),
    &[PartitionSpec::new(partition_main as PartitionEntry, 0)]
);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("unified_minimal: start");

    hprintln!("unified_minimal: calling boot");
    match boot(p).expect("unified_minimal: boot failed") {}
}
