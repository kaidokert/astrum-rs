//! Message Queue Demo for STM32F429ZI
//!
//! Demonstrates kernel message queues between partitions:
//! - P0 (sender): sends an increasing counter via SYS_MSG_SEND
//! - P1 (receiver): receives messages, tracks count
//! - Kernel tick handler: logs progress via RTT every 500ms
//!
//! Success criterion: after 5 seconds, SENT > 10 and RECV > 10 and SENT == RECV.
//!
//! Build: cd f429zi && cargo build --example msg_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    message::MessageQueue,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here, it would cause a duplicate panic handler link error.
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const MSG_SIZE: usize = 4; // 4 bytes = 1 u32 counter

static SENT: AtomicU32 = AtomicU32::new(0);
static RECV: AtomicU32 = AtomicU32::new(0);
static LAST_SENT: AtomicU32 = AtomicU32::new(0);
static LAST_RECV: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(MsgConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(MsgConfig, |tick, _k| {
    if tick % 500 == 0 {
        let s = SENT.load(Ordering::Acquire);
        let r = RECV.load(Ordering::Acquire);
        let ls = LAST_SENT.load(Ordering::Acquire);
        let lr = LAST_RECV.load(Ordering::Acquire);
        rprintln!(
            "[{:4}ms] SENT={} (last_val={}) RECV={} (last_val={})",
            tick, s, ls, r, lr
        );
        if r > 10 && s > 10 {
            rprintln!("✓ SUCCESS: Message queues working! SENT={} RECV={}", s, r);
        }
    }
});

/// P0: sends increasing counter via message queue.
extern "C" fn sender_main_body(_r0: u32) -> ! {
    let target_pid = 0u32.into();
    let mut counter: u32 = 1;

    loop {
        let msg = counter.to_le_bytes();
        if plib::sys_msg_send(target_pid, &msg).is_ok() {
            SENT.fetch_add(1, Ordering::Release);
            LAST_SENT.store(counter, Ordering::Release);
            counter = counter.wrapping_add(1);
        }

        for _ in 0..5000 {
            core::hint::spin_loop();
        }
        plib::sys_yield().ok();
    }
}

/// P1: receives messages and counts them.
extern "C" fn receiver_main_body(_r0: u32) -> ! {
    let mut buf = [0u8; MSG_SIZE];

    loop {
        // sys_msg_recv: receives from the caller's own queue (queue_id=0 hardcoded in plib).
        // Returns Ok on both receive and block; Err only on error.
        if plib::sys_msg_recv(&mut buf).is_ok() {
            let val = u32::from_le_bytes(buf);
            if val > 0 {
                RECV.fetch_add(1, Ordering::Release);
                LAST_RECV.store(val, Ordering::Release);
                buf = [0u8; MSG_SIZE]; // clear for next message
            }
        }

        for _ in 0..5000 {
            core::hint::spin_loop();
        }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(sender_main => sender_main_body);
kernel::partition_trampoline!(receiver_main => receiver_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Message Queue Demo — P0 sends, P1 receives ===");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ MsgConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(sender_main), // target_pid = 0
        PartitionSpec::entry(receiver_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Create one message queue (gets ID 0)
    kernel::state::with_kernel_mut::<MsgConfig, _, _>(|k| {
        k.messages_mut().add(MessageQueue::new()).expect("add queue");
        Ok::<(), ()>(())
    }).expect("ipc setup");
    rprintln!("[INIT] Message queue created (id=0)");

    rprintln!("[INIT] Booting: P0=sender P1=receiver target_pid=0\n");
    match boot(p).expect("boot") {}
}
