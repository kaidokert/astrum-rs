//! QEMU test: verify scheduler tick drives partition switches via Kernel.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncStandard,
};

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncStandard,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 100; // 10 kHz — matches original RELOAD for reproducibility
    }
);

static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);
/// Tracks the active partition across ticks. `u32::MAX` means no partition yet.
static ACTIVE_PID: AtomicU32 = AtomicU32::new(u32::MAX);
const MAX_SWITCHES: u32 = 6;

kernel::define_kernel!(TestConfig, |_tick, k| {
    use kernel::partition::PartitionState;
    // Verify post-switch state: harness already called advance_schedule_tick + set_next_partition.
    let pid = k.next_partition();
    let prev = ACTIVE_PID.swap(pid as u32, Ordering::AcqRel);
    if prev != pid as u32 {
        // (1) Verify outgoing partition transitioned to Ready (not still Running).
        if prev != u32::MAX {
            let out_state = k.partitions().get(prev as usize).map(|p| p.state());
            if out_state != Some(PartitionState::Ready) {
                hprintln!(
                    "FAIL: outgoing partition {} expected Ready, got {:?}",
                    prev,
                    out_state
                );
                kernel::kexit!(failure);
            }
        }
        // (2) Verify incoming partition is Running.
        let in_state = k.partitions().get(pid as usize).map(|p| p.state());
        if in_state != Some(PartitionState::Running) {
            hprintln!(
                "FAIL: partition {} expected Running, got {:?}",
                pid,
                in_state
            );
            kernel::kexit!(failure);
        }
        hprintln!("switch -> partition {}", pid);
        SWITCH_COUNT.fetch_add(1, Ordering::Release);
    }
    if SWITCH_COUNT.load(Ordering::Acquire) >= MAX_SWITCHES {
        hprintln!("done: {} switches, 0 failures", MAX_SWITCHES);
        kernel::kexit!(success);
    }
});

const _: PartitionEntry = partition_idle;
extern "C" fn partition_idle() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("scheduler_tick: Peripherals::take");
    hprintln!("scheduler_tick: started");

    let mut sched: ScheduleTable<{ TestConfig::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 3)).unwrap();
    sched.add(ScheduleEntry::new(1, 2)).unwrap();

    let entries: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(partition_idle as PartitionEntry, 0),
        PartitionSpec::new(partition_idle as PartitionEntry, 0),
    ];
    init_kernel(sched, &entries).expect("kernel creation");

    match boot(p).expect("scheduler_tick: boot") {}
}
