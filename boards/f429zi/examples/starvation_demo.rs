//! Starvation Detection Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates the kernel's starvation tracking on real hardware.
//!
//! Architecture (3 partitions):
//!   P0 (runner): loops freely, incrementing a counter.
//!   P1 (blocker): blocks on an empty semaphore immediately → Waiting state.
//!   P2 (observer): loops freely, incrementing a counter.
//!
//! Schedule: P0(2), P1(2) — P2 is NOT in the schedule table.
//! P2 stays Ready but never gets a time slot. When P1's slot comes up,
//! the scheduler sees P1 is Waiting and calls
//! increment_starvation_for_ready_partitions(). This increments starvation
//! for all Ready partitions that are NOT the currently active partition.
//! P2 is Ready but never active → its starvation_count climbs monotonically.
//! After STARVATION_THRESHOLD (3) consecutive skips, P2 is marked starved.
//!
//! RTT output shows P2's starvation_count climbing and is_starved flag.
//!
//! Build: cd f429zi && cargo build --example starvation_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    partition::STARVATION_THRESHOLD,
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    {Partitions3, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;

const NUM_PARTITIONS: usize = 3;

static P0_COUNT: AtomicU32 = AtomicU32::new(0);
static P2_COUNT: AtomicU32 = AtomicU32::new(0);
static P1_SEM_ERR: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(StarvCfg<Partitions3, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(StarvCfg, |tick, k| {
    if tick % 500 == 0 {
        let p0_count = P0_COUNT.load(Ordering::Acquire);
        let p2_count = P2_COUNT.load(Ordering::Acquire);
        let sem_err  = P1_SEM_ERR.load(Ordering::Acquire);

        // Read starvation state for all partitions.
        let (p0_starv, p0_is) = k.partitions().get(0)
            .map(|p| (p.starvation_count(), p.is_starved()))
            .unwrap_or((0, false));
        let (p1_starv, p1_is) = k.partitions().get(1)
            .map(|p| (p.starvation_count(), p.is_starved()))
            .unwrap_or((0, false));
        let (p2_starv, p2_is) = k.partitions().get(2)
            .map(|p| (p.starvation_count(), p.is_starved()))
            .unwrap_or((0, false));

        rprintln!(
            "[{:5}ms] P0: n={} s={}  P1(blocked): s={}  P2: n={} s={} starved={}  sem_err={}",
            tick, p0_count, p0_starv, p1_starv, p2_count, p2_starv, p2_is, sem_err
        );

        if p2_is && p2_starv >= STARVATION_THRESHOLD {
            rprintln!(
                "STARVATION DETECTED on P2! count={} threshold={} is_starved=true",
                p2_starv, STARVATION_THRESHOLD
            );
            rprintln!("P0 running: P0_COUNT={}  P2 running: P2_COUNT={}", p0_count, p2_count);
            if p0_count > 0 && p2_count > 0 {
                rprintln!("SUCCESS: starvation detected — P1 blocking causes P2 to be skipped.");
            }
        }
    }
});

// P0: healthy partition, runs freely.
extern "C" fn p0_runner(_r0: u32) -> ! {
    loop {
        P0_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}

// P1: blocks on empty semaphore → Waiting forever.
extern "C" fn p1_blocker(_r0: u32) -> ! {
    // Semaphore 0 has count=0, so this blocks immediately.
    if let Err(e) = plib::sys_sem_wait(plib::SemaphoreId::new(0)) {
        P1_SEM_ERR.store(e.to_u32(), Ordering::Release);
    }
    // If sem_wait ever returns (shouldn't), just loop.
    loop {
        cortex_m::asm::nop();
    }
}

// P2: healthy partition, runs freely. Its starvation counter climbs
// because when P1's schedule slot is wasted (P1 is Waiting), P2 is
// Ready but not active → the kernel marks it as "skipped".
extern "C" fn p2_observer(_r0: u32) -> ! {
    loop {
        P2_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(p0_main => p0_runner);
kernel::partition_trampoline!(p1_main => p1_blocker);
kernel::partition_trampoline!(p2_main => p2_observer);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Starvation Detection Demo — STM32F429ZI ===");
    rprintln!("P0: runs freely  P1: blocks on semaphore  P2: runs freely");
    rprintln!("P2's starvation climbs when P1's slot is wasted (P1 Waiting).");
    rprintln!("Threshold: {} consecutive skips → is_starved=true\n", STARVATION_THRESHOLD);

    let p = cortex_m::Peripherals::take().unwrap();

    // P2 intentionally NOT scheduled — it stays Ready but never runs.
    let mut sched = ScheduleTable::<{ StarvCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(p0_main),
        PartitionSpec::entry(p1_main),
        PartitionSpec::entry(p2_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Add semaphore 0: count=0, max=1 — P1 blocks immediately on wait.
    kernel::state::with_kernel_mut::<StarvCfg, _, _>(|k| {
        k.semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        Ok::<(), ()>(())
    }).expect("semaphore setup");

    rprintln!("[INIT] Kernel created. Semaphore 0: count=0 max=1 (P1 will block).");
    rprintln!("[INIT] Schedule: P0(2) P1(2) — P2 unscheduled (stays Ready)");
    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
