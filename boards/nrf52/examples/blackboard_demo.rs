//! Blackboard Demo for nRF52833 (PCA10100)
//!
//! Demonstrates shared memory communication via blackboards:
//! - Publisher partition writes status updates to blackboard
//! - Multiple subscribers read the shared data
//! - Last-value semantics (readers always see most recent write)
//!
//! Three partitions:
//! - P0 (Publisher): Writes status values (counter, state) to blackboard
//! - P1 (Subscriber A): Reads status and tracks changes
//! - P2 (Subscriber B): Reads status independently

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
use plib;
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here, it would cause a duplicate panic handler link error.
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 3;
const BB_MSG_SIZE: usize = 8; // 2x u32: counter + state

// Status states
const STATE_IDLE: u32 = 0;
const STATE_ACTIVE: u32 = 1;
const STATE_BUSY: u32 = 2;


// Progress tracking
static PUB_WRITES: AtomicU32 = AtomicU32::new(0);
static PUB_COUNTER: AtomicU32 = AtomicU32::new(0);
static SUB_A_READS: AtomicU32 = AtomicU32::new(0);
static SUB_A_LAST: AtomicU32 = AtomicU32::new(0);
static SUB_B_READS: AtomicU32 = AtomicU32::new(0);
static SUB_B_LAST: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(BlackboardConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = BB_MSG_SIZE; BW = 4;
});

kernel::define_kernel!(BlackboardConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let pub_w = PUB_WRITES.load(Ordering::Acquire);
        let pub_c = PUB_COUNTER.load(Ordering::Acquire);
        let sub_a_r = SUB_A_READS.load(Ordering::Acquire);
        let sub_a_l = SUB_A_LAST.load(Ordering::Acquire);
        let sub_b_r = SUB_B_READS.load(Ordering::Acquire);
        let sub_b_l = SUB_B_LAST.load(Ordering::Acquire);

        rprintln!(
            "[{:5}ms] PUB: writes={:4} cnt={:3} | SUB_A: reads={:4} last={:3} | SUB_B: reads={:4} last={:3}",
            tick, pub_w, pub_c, sub_a_r, sub_a_l, sub_b_r, sub_b_l
        );

        if pub_w > 10 && sub_a_r > 5 && sub_b_r > 5 {
            rprintln!("✓ SUCCESS: Blackboard working! Publisher writing, subscribers reading.");
        }
    }
});

/// Publisher partition - writes status updates to blackboard
extern "C" fn publisher_main_body(r0: u32) -> ! {
    let bb_id = r0.into();
    let mut counter: u8 = 0;
    let states = [STATE_IDLE, STATE_ACTIVE, STATE_BUSY];
    let mut state_idx = 0;

    loop {
        counter = counter.wrapping_add(1);
        let state = states[state_idx];
        state_idx = (state_idx + 1) % states.len();

        // Write status to blackboard: [counter_low, counter_high, state_low, state_high]
        // Note: u32 values stored as 4 bytes each (little-endian)
        let buf = [
            counter, 0, 0, 0,           // counter as u32 (low byte only used)
            state as u8, 0, 0, 0,       // state as u32 (low byte only used)
        ];

        if plib::sys_bb_display(bb_id, &buf).is_ok() {
            PUB_WRITES.fetch_add(1, Ordering::Release);
            PUB_COUNTER.store(counter as u32, Ordering::Release);
        }

        // Delay
        for _ in 0..20000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

/// Subscriber A partition - reads status from blackboard
extern "C" fn subscriber_a_main_body(r0: u32) -> ! {
    let bb_id = r0.into();

    loop {
        let mut buf = [0u8; BB_MSG_SIZE];
        if let Ok(sz) = plib::sys_bb_read(bb_id, &mut buf) {
            if sz > 0 {
                SUB_A_READS.fetch_add(1, Ordering::Release);
                // Extract counter from first u32
                let counter = buf[0] as u32;
                SUB_A_LAST.store(counter, Ordering::Release);
            }
        }

        // Delay
        for _ in 0..15000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

/// Subscriber B partition - reads status from blackboard independently
extern "C" fn subscriber_b_main_body(r0: u32) -> ! {
    let bb_id = r0.into();

    loop {
        let mut buf = [0u8; BB_MSG_SIZE];
        if let Ok(sz) = plib::sys_bb_read(bb_id, &mut buf) {
            if sz > 0 {
                SUB_B_READS.fetch_add(1, Ordering::Release);
                // Extract counter from first u32
                let counter = buf[0] as u32;
                SUB_B_LAST.store(counter, Ordering::Release);
            }
        }

        // Delay
        for _ in 0..12000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(publisher_main => publisher_main_body);
kernel::partition_trampoline!(subscriber_a_main => subscriber_a_main_body);
kernel::partition_trampoline!(subscriber_b_main => subscriber_b_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Blackboard Demo ===");
    rprintln!("nRF52833 - Shared Status Board Pattern");

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: Equal time slices for all partitions
    let mut sched = ScheduleTable::<{ BlackboardConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add(ScheduleEntry::new(2, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");
    rprintln!("Schedule: P0(Publisher) P1(SubA) P2(SubB)");

    let bb: u32 = 0;

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(publisher_main as kernel::PartitionEntry, bb),
        PartitionSpec::new(subscriber_a_main as kernel::PartitionEntry, bb),
        PartitionSpec::new(subscriber_b_main as kernel::PartitionEntry, bb),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("Kernel created");

    kernel::state::with_kernel_mut::<BlackboardConfig, _, _>(|k| {
        k.blackboards_mut()
            .create()
            .expect("blackboard");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Blackboard created: {}", bb);

    rprintln!("Booting...\n");

    match boot(p).expect("boot") {}
}
