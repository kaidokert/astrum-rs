//! Partition Debug Logging Demo for STM32F429ZI
//!
//! Demonstrates `plib`/`dprint!` — the partition-side debug logging facility:
//! - P0 (counter): increments a work counter, logs progress via `dprint!`
//! - P1 (monitor): observes the counter, logs its view via `debug_warn!`
//! - Kernel harness: auto-drains both ring buffers every tick via klog!/RTT
//!
//! The full path exercised:
//!   partition calls dprint!()
//!   → plib::debug_write() writes framed record to DebugRingBuffer
//!   → plib calls SYS_DEBUG_NOTIFY syscall
//!   → kernel sets pcb.debug_pending flag
//!   → harness SysTick auto-drain calls drain_debug_pending → klog! → RTT
//!
//! Success criterion: RTT shows interleaved [P0:INF] and [P1:WRN] lines
//! from both partitions, with WORK counter advancing.
//!
//! Build: cd f429zi && cargo build --example plib_demo --features partition-debug

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{debug_warn, define_partition_debug, dprint};
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 512;

// Shared atomic so the SysTick hook can report progress
static WORK: AtomicU32 = AtomicU32::new(0);
static P0_DPRINTLN: AtomicU32 = AtomicU32::new(0);
static P1_DPRINTLN: AtomicU32 = AtomicU32::new(0);

// Per-partition debug ring buffers (power-of-2 size, registered with PCBs in main())
define_partition_debug!(P0_DEBUG, 256);
define_partition_debug!(P1_DEBUG, 256);

kernel::kernel_config!(DemoConfig[AlignedStack2K]<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 4;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(DemoConfig, |tick, _k| {
    if tick % 500 == 0 {
        let work = WORK.load(Ordering::Acquire);
        let p0 = P0_DPRINTLN.load(Ordering::Acquire);
        let p1 = P1_DPRINTLN.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] WORK={} | dprint calls: P0={} P1={}",
            tick, work, p0, p1
        );
        if p0 > 5 && p1 > 5 {
            rprintln!("✓ SUCCESS: plib/dprint! working! Messages from both partitions.");
        }
    }
    // Partition debug output is drained automatically by the harness
    // (DEBUG_AUTO_DRAIN_BUDGET = 256 bytes/partition/tick via klog!/RTT).
});

/// P0: increments WORK counter, logs progress via dprint! every ~500 iterations.
extern "C" fn counter_main_body(_r0: u32) -> ! {
    let mut iter: u32 = 0;
    loop {
        iter = iter.wrapping_add(1);
        WORK.fetch_add(1, Ordering::Release);

        if iter % 500 == 0 {
            let _ = dprint!(&P0_DEBUG, "iter={} work={}", iter, WORK.load(Ordering::Relaxed));
            P0_DPRINTLN.fetch_add(1, Ordering::Release);
        }

        plib::sys_yield().ok();
    }
}

/// P1: watches WORK counter, logs via debug_warn! every ~300 iterations.
extern "C" fn monitor_main_body(_r0: u32) -> ! {
    let mut iter: u32 = 0;
    loop {
        iter = iter.wrapping_add(1);

        if iter % 300 == 0 {
            let w = WORK.load(Ordering::Acquire);
            let _ = debug_warn!(&P1_DEBUG, "monitor: work={} my_iter={}", w, iter);
            P1_DPRINTLN.fetch_add(1, Ordering::Release);
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(counter_main => counter_main_body);
kernel::partition_trampoline!(monitor_main => monitor_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Partition Debug Logging Demo ===");
    rprintln!("STM32F429ZI - plib/dprint! path");
    rprintln!("P0(counter) P1(monitor)");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] =
        [PartitionSpec::entry(counter_main), PartitionSpec::entry(monitor_main)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Register debug ring buffers with each partition's PCB
    kernel::state::with_kernel_mut::<DemoConfig, _, _>(|k| {
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&P0_DEBUG);
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .set_debug_buffer(&P1_DEBUG);
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Debug buffers registered: P0=256B P1=256B");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
