//! Events Hardware Demo — Ping-Pong Synchronization
//!
//! Demonstrates inter-partition event signaling on nRF52833.
//!
//! Pattern:
//! - P0 (setter, pid=0): sets event bit 0x1 on P1 every iteration, then yields
//! - P1 (waiter, pid=1): waits for event bit 0x1, verifies return value is 0x1, yields
//! - Kernel tick handler: logs partition states and local counters via RTT
//!
//! Events do not use pointer validation (no buffer passed), so this test
//! exercises the event_wait / event_set / event_clear syscall path directly.
//!
//! Build:  cd nrf52 && cargo build --example events_demo --features kernel-example
//! Flash:  (via GDB + OpenOCD)

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use plib::{self, EventMask, PartitionId};
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here, it would cause a duplicate panic handler link error.
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;

static RECV_COUNT: AtomicU32 = AtomicU32::new(0);

/// P1_PID is the partition ID of the waiter — passed as hint to both partitions
/// so P0 knows who to signal, and P1 knows its own id for event_wait.
const P1_PID: PartitionId = PartitionId::new(1);

kernel::kernel_config!(EventConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(EventConfig, |tick, k| {
    // Kernel tick handler — privileged context, has access to partition table.
    // Log partition states every 500 ticks to observe blocking / wake-up behaviour.
    if tick % 500 == 0 {
        let p0_state = k.partitions().get(0).map(|p| p.state());
        let p1_state = k.partitions().get(1).map(|p| p.state());
        let p1_flags = k.partitions().get(1).map(|p| p.event_flags());
        let recvs = RECV_COUNT.load(Ordering::Acquire);
        rprintln!(
            "[KERNEL] tick={} P0={:?} P1={:?} P1.flags={:#010b} RECV={}",
            tick, p0_state, p1_state, p1_flags.unwrap_or(0), recvs
        );
        if recvs > 20 {
            rprintln!("✓ SUCCESS: Events working! RECV={}", recvs);
        }
    }
});

/// P0: sets event bit 0x1 on P1 in a loop.
extern "C" fn setter_main_body(r0: u32) -> ! {
    let target_pid = r0.into();
    let event_bit = EventMask::new(0x1);
    let mut set_count: u32 = 0;

    loop {
        // Set event bit on P1.
        if plib::sys_event_set(target_pid, event_bit).is_ok() {
            set_count = set_count.wrapping_add(1);
        }

        // Brief spin before yielding so P1 has time to process.
        for _ in 0..3000 {
            core::hint::spin_loop();
        }
        plib::sys_yield().ok();
    }
}

/// P1: waits for event bit 0x1 from P0, verifies the return value.
extern "C" fn waiter_main_body(_r0: u32) -> ! {
    let event_bit = EventMask::new(0x1);
    let mut recv_count: u32 = 0;

    loop {
        // event_wait semantics:
        //   Ok(0)    = was put to Waiting; just woke up — loop and re-wait to consume.
        //   Ok(bits) = matched bits returned, event consumed.
        //   Err(_)   = error — should never happen.
        match plib::sys_event_wait(event_bit) {
            Err(_) => continue,
            Ok(bits) if bits.as_raw() == 0 => continue,
            Ok(bits) => {
                if (bits & event_bit).as_raw() != 0 {
                    recv_count = recv_count.wrapping_add(1);
                    RECV_COUNT.store(recv_count, Ordering::Release);
                }
            }
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(setter_main => setter_main_body);
kernel::partition_trampoline!(waiter_main => waiter_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Events Hardware Demo — Ping-Pong via SYS_EVT_SET / SYS_EVT_WAIT ===");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ EventConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    // P0 gets P1's pid as hint (so it knows who to signal).
    // P1 gets its own pid as hint (so it can pass it to event_wait).
    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(setter_main as kernel::PartitionEntry, P1_PID.as_raw()),      // P0: target = P1
        PartitionSpec::new(waiter_main as kernel::PartitionEntry, P1_PID.as_raw()),      // P1: own_pid = 1
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("[INIT] Kernel created");

    rprintln!("[INIT] Booting: P0=setter P1=waiter event_bit=0x1\n");
    match boot(p).expect("boot") {}
}
