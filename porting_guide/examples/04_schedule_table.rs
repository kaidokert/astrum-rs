//! Deterministic time-slice scheduling with the kernel's ScheduleTable.
//! P0=4t, P1=2t, 1t system windows (major frame=8). PASS after 3 major
//! frames when P0 >= 12, P1 >= 6, P0 > P1.

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

const P0T: u32 = 4;
const P1T: u32 = 2;
const SYST: u32 = 1;
const MF: u32 = P0T + SYST + P1T + SYST;
const FRAMES: u32 = 3;
const TIMEOUT: u32 = 100;

kernel::kernel_config!(
    Cfg < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

static P0_TICKS: AtomicU32 = AtomicU32::new(0);
static P1_TICKS: AtomicU32 = AtomicU32::new(0);

kernel::define_harness!(Cfg, |tick, k| {
    match k.current_partition as usize {
        0 => {
            P0_TICKS.fetch_add(1, Ordering::Relaxed);
        }
        1 => {
            P1_TICKS.fetch_add(1, Ordering::Relaxed);
        }
        _ => {}
    }
    let p0 = P0_TICKS.load(Ordering::Relaxed);
    let p1 = P1_TICKS.load(Ordering::Relaxed);

    if tick >= MF * FRAMES {
        let mf = k.schedule().major_frame_count();
        klog!("tick {}: p0={} p1={} major_frames={}", tick, p0, p1, mf);
        if mf >= FRAMES {
            if p0 < P0T * FRAMES {
                klog!("FAIL: P0 ticks {} < expected {}", p0, P0T * FRAMES);
                kernel::kexit!(failure);
            }
            if p1 < P1T * FRAMES {
                klog!("FAIL: P1 ticks {} < expected {}", p1, P1T * FRAMES);
                kernel::kexit!(failure);
            }
            if p0 <= p1 {
                klog!("FAIL: P0 ({}) should be > P1 ({})", p0, p1);
                kernel::kexit!(failure);
            }
            klog!(
                "04_schedule_table: PASS (p0={} p1={} frames={})",
                p0,
                p1,
                mf
            );
            kernel::kexit!(success);
        }
    }
    if tick >= TIMEOUT {
        klog!("04_schedule_table: FAIL timeout (p0={} p1={})", p0, p1);
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = idle;
extern "C" fn idle() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("04_schedule_table: deterministic time-slice demo");
    klog!("schedule: P0={}t P1={}t major_frame={}t", P0T, P1T, MF);

    let mut sched: ScheduleTable<{ Cfg::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, P0T)).expect("sched P0");
    sched.add_system_window(SYST).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, P1T)).expect("sched P1");
    sched.add_system_window(SYST).expect("syswin 1");

    let entries: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(idle as PartitionEntry, 0),
        PartitionSpec::new(idle as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &entries).expect("kernel creation");
    store_kernel(&mut k);
    match boot(p).expect("04_schedule_table: boot") {}
}
