//! Events + dprint! Demo — Instrumented EventWait Path on STM32F429ZI
//!
//! Extends the events_demo ping-pong pattern with per-partition dprint! logging
//! so we can observe the full EventWait blocking/unblocking path via RTT narration.
//!
//! Pattern:
//! - P0 (setter, pid=0): sets event bit 0x1 on P1 every iteration, logs every 50 sets
//! - P1 (waiter, pid=1): calls event_wait, logs "wait...", "blocked→wake", or "recv=0x01"
//! - SysTick hook: kernel auto-drains both ring buffers to RTT every tick
//!
//! Success criterion:
//!   RTT shows [P0:INF] "set#N" and [P1:INF] "recv=0x1 cnt=N" interleaved,
//!   plus occasional [P1:INF] "blocked→wake" lines proving the blocking path fires.
//!
//! Build: cd f429zi && cargo build --example events_dprint_demo --features partition-debug

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use plib::{define_partition_debug, dprint, EventMask, PartitionId};
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 512;

// Hardcoded PIDs — avoids unpack_r0!() fragility (non-zero constants clobber r0).
const WAITER_PID: PartitionId = PartitionId::new(1);
const EVENT_BIT: EventMask = EventMask::new(0x1);

// Shared atomics for kernel-side progress reporting.
static SET_COUNT: AtomicU32 = AtomicU32::new(0);
static RECV_COUNT: AtomicU32 = AtomicU32::new(0);
static BLOCK_COUNT: AtomicU32 = AtomicU32::new(0);

// Per-partition debug ring buffers.
define_partition_debug!(P0_DEBUG, 256);
define_partition_debug!(P1_DEBUG, 512);

kernel::kernel_config!(EventDprintConfig[AlignedStack2K]<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(EventDprintConfig, |tick, k| {
    // Log kernel-side counters and partition states every 500 ms.
    if tick % 500 == 0 {
        let p1_state = k.partitions().get(1).map(|p| p.state());
        let p1_flags = k.partitions().get(1).map(|p| p.event_flags()).unwrap_or(0);
        let sets = SET_COUNT.load(Ordering::Acquire);
        let recvs = RECV_COUNT.load(Ordering::Acquire);
        let blocks = BLOCK_COUNT.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] SET={} RECV={} BLOCK={} | P1={:?} flags={:#010b}",
            tick, sets, recvs, blocks, p1_state, p1_flags
        );
        if recvs > 10 {
            rprintln!("✓ SUCCESS: events + dprint! working, RECV={}", recvs);
        }
    }
    // Partition debug output is drained automatically by the harness
    // (DEBUG_AUTO_DRAIN_BUDGET = 256 bytes/partition/tick via klog!/RTT).
});

/// P0 (setter): sets event bit 0x1 on P1 repeatedly, logs every 50 sets.
extern "C" fn setter_main_body(_r0: u32) -> ! {
    let mut iter: u32 = 0;

    loop {
        if plib::sys_event_set(WAITER_PID, EVENT_BIT).is_ok() {
            iter = iter.wrapping_add(1);
            SET_COUNT.fetch_add(1, Ordering::Release);

            if iter % 50 == 0 {
                let _ = dprint!(&P0_DEBUG, "set#{}", iter);
            }
        }

        // Brief spin to let P1 run before we set again.
        for _ in 0..5000 {
            core::hint::spin_loop();
        }
        plib::sys_yield().ok();
    }
}

/// P1 (waiter): waits for event bit 0x1, narrates each outcome via dprint!.
extern "C" fn waiter_main_body(_r0: u32) -> ! {
    let mut recv_count: u32 = 0;

    loop {
        // Call event_wait.  Two outcomes:
        //   rc == 0  → we were put to Waiting; kernel will reschedule us when P0 signals.
        //              trigger_deschedule() in svc.rs pends PendSV → we truly block.
        //   rc != 0  → event was already set; bits returned (should equal EVENT_BIT).
        let bits = match plib::sys_event_wait(EVENT_BIT) {
            Err(_) => continue,
            Ok(b) => b,
        };

        if bits.as_raw() == 0 {
            // We were blocked and just woke up.  Log and loop to consume.
            BLOCK_COUNT.fetch_add(1, Ordering::Release);
            let _ = dprint!(&P1_DEBUG, "blocked->wake");
            continue;
        }

        // bits != 0: event consumed successfully.
        recv_count = recv_count.wrapping_add(1);
        RECV_COUNT.store(recv_count, Ordering::Release);

        if recv_count % 50 == 1 {
            let _ = dprint!(&P1_DEBUG, "recv=0x{:x} cnt={}", bits.as_raw(), recv_count);
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(setter_main => setter_main_body);
kernel::partition_trampoline!(waiter_main => waiter_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Events + dprint! Demo ===");
    rprintln!("STM32F429ZI — EventWait blocking path narrated via partition dprint!");
    rprintln!("P0(setter) P1(waiter) event_bit=0x{:x}", EVENT_BIT.as_raw());

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ EventDprintConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    // No hints passed — PIDs are hardcoded in each partition function.
    let parts: [PartitionSpec; NUM_PARTITIONS] =
        [PartitionSpec::entry(setter_main), PartitionSpec::entry(waiter_main)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Register debug ring buffers with each partition's PCB.
    kernel::state::with_kernel_mut::<EventDprintConfig, _, _>(|k| {
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

    rprintln!("Debug buffers registered: P0=256B P1=512B");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
