//! QEMU integration example: two-partition context switch via PendSV.
//!
//! Sets up two partitions using `define_unified_harness!`. Each partition
//! stores its ID to a shared atomic; the SysTick hook observes which
//! partition ran and exits after enough interleaved switches.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(DemoConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

const NUM_PARTITIONS: usize = DemoConfig::N;
const STACK_WORDS: usize = DemoConfig::STACK_WORDS;
const TARGET_SWITCHES: u32 = 6;

static PARTITION_RUNNING: AtomicU32 = AtomicU32::new(u32::MAX);
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(DemoConfig, |tick, _k| {
    let who = PARTITION_RUNNING.load(Ordering::Acquire);
    if who != u32::MAX {
        let count = SWITCH_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
        hprintln!("switch {}: partition {} was running", count, who);
        if count >= TARGET_SWITCHES {
            hprintln!(
                "context_switch: {} switches observed -- PASS",
                TARGET_SWITCHES
            );
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick > 100 {
        hprintln!("context_switch: FAIL - timeout");
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn partition_0_entry() -> ! {
    loop {
        PARTITION_RUNNING.store(0, Ordering::Release);
        cortex_m::asm::nop();
    }
}
extern "C" fn partition_1_entry() -> ! {
    loop {
        PARTITION_RUNNING.store(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("context_switch: setting up two partitions");

    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched entry 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched entry 1");

    let cfgs: [PartitionConfig; NUM_PARTITIONS] =
        core::array::from_fn(|i| PartitionConfig::sentinel(i as u8, (STACK_WORDS * 4) as u32));

    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<DemoConfig>::new(sched, &cfgs).expect("kernel creation");

    store_kernel(k);
    hprintln!("context_switch: triggering first PendSV");

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        [(partition_0_entry, 0), (partition_1_entry, 0)];
    match boot(&parts, &mut p).expect("context_switch: boot failed") {}
}
