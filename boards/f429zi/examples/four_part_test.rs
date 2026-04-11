#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugEnabled, MsgMinimal, Partitions4, PortsTiny, SyncMinimal},
};
use f429zi as _;

static P: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0),
];

kernel::kernel_config!(Cfg4<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
});

kernel::define_kernel!(Cfg4, |tick, _k| {
    if tick % 500 == 0 {
        kernel::klog!("[{}ms] P0={} P1={} P2={} P3={}", tick,
            P[0].load(Ordering::Relaxed), P[1].load(Ordering::Relaxed),
            P[2].load(Ordering::Relaxed), P[3].load(Ordering::Relaxed));
        let all = P.iter().all(|p| p.load(Ordering::Relaxed) > 10);
        if all { kernel::klog!("SUCCESS: 4 partitions running!"); }
    }
});

extern "C" fn part_body(r0: u32) -> ! {
    loop {
        P[r0 as usize].fetch_add(1, Ordering::Relaxed);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(part_main => part_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let mut s = ScheduleTable::<{ Cfg4::SCHED }>::new();
    s.add(ScheduleEntry::new(0, 2)).expect("s0");
    s.add(ScheduleEntry::new(1, 2)).expect("s1");
    s.add(ScheduleEntry::new(2, 2)).expect("s2");
    s.add(ScheduleEntry::new(3, 2)).expect("s3");
    s.add_system_window(1).expect("sys_window");
    let parts: [PartitionSpec; 4] = [
        PartitionSpec::entry(part_main), PartitionSpec::new(part_main as kernel::PartitionEntry, 1), PartitionSpec::new(part_main as kernel::PartitionEntry, 2), PartitionSpec::new(part_main as kernel::PartitionEntry, 3),
    ];
    init_kernel(s, &parts).expect("init");
    match boot(p).expect("boot") {}
}
