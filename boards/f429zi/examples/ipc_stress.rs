//! Sustained High-Rate IPC Stress Test
//!
//! Same as msg_demo but with no artificial delays — sender and receiver
//! blast as fast as possible. Target: >10K msg/s sustained.
//!
//! Build: cd f429zi && cargo build --example ipc_stress \
//!            --features kernel-example --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    message::MessageQueue,
    scheduler::{ScheduleEntry, ScheduleTable},
    PartitionEntry, PartitionSpec,
    {DebugEnabled, MsgSmall, Partitions4, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

const MSG_SIZE: usize = 4;

static SENT: AtomicU32 = AtomicU32::new(0);
static RECV: AtomicU32 = AtomicU32::new(0);
static SEND_FAIL: AtomicU32 = AtomicU32::new(0);
static PREV_SENT: AtomicU32 = AtomicU32::new(0);
static PREV_RECV: AtomicU32 = AtomicU32::new(0);
static SUSTAINED_OK: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(StressCfg<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(StressCfg, |tick, _k| {
    if tick % 1000 == 0 && tick > 0 {
        let sent = SENT.load(Ordering::Acquire);
        let recv = RECV.load(Ordering::Acquire);
        let fails = SEND_FAIL.load(Ordering::Acquire);
        let prev_s = PREV_SENT.swap(sent, Ordering::Relaxed);
        let prev_r = PREV_RECV.swap(recv, Ordering::Relaxed);
        let rate_s = sent - prev_s;
        let rate_r = recv - prev_r;

        rprintln!(
            "[{:5}ms] sent={} recv={} rate_s={}/s rate_r={}/s fails={}",
            tick, sent, recv, rate_s, rate_r, fails
        );

        if rate_s >= 10000 && rate_r >= 10000 {
            let ok = SUSTAINED_OK.fetch_add(1, Ordering::Relaxed) + 1;
            if ok >= 3 {
                rprintln!(
                    "SUCCESS: sustained IPC stress! {}/s send, {}/s recv for {}s",
                    rate_s, rate_r, ok
                );
            }
        } else if tick > 2000 {
            SUSTAINED_OK.store(0, Ordering::Relaxed);
        }
    }
});

/// P0: sender — batch sends before yielding for higher throughput
extern "C" fn sender_body(_r0: u32) -> ! {
    let target_pid = 0u32.into();
    let mut counter: u32 = 1;

    loop {
        // Send a batch before yielding
        for _ in 0..4 {
            let msg = counter.to_le_bytes();
            if plib::sys_msg_send(target_pid, &msg).is_ok() {
                SENT.fetch_add(1, Ordering::Relaxed);
                counter = counter.wrapping_add(1);
            } else {
                SEND_FAIL.fetch_add(1, Ordering::Relaxed);
                break; // queue full, yield to let receiver drain
            }
        }
        plib::sys_yield().ok();
    }
}

/// P1: receiver — batch recvs before yielding
extern "C" fn receiver_body(_r0: u32) -> ! {
    let mut buf = [0u8; MSG_SIZE];

    loop {
        for _ in 0..4 {
            if plib::sys_msg_recv(&mut buf).is_ok() {
                let val = u32::from_le_bytes(buf);
                if val > 0 {
                    RECV.fetch_add(1, Ordering::Relaxed);
                    buf = [0u8; MSG_SIZE];
                }
            }
        }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(sender_main => sender_body);
kernel::partition_trampoline!(receiver_main => receiver_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Sustained IPC Stress Test ===");
    rprintln!("P0 blasts messages, P1 drains. Target: >10K msg/s sustained.\n");

    let mut sched = ScheduleTable::<{ StressCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("P0");
    sched.add_system_window(1).expect("SW");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW");

    let parts: [PartitionSpec; 2] = [
        PartitionSpec::entry(sender_main),
        PartitionSpec::entry(receiver_main),
    ];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    kernel::state::with_kernel_mut::<StressCfg, _, _>(|k| {
        k.messages_mut().add(MessageQueue::new()).expect("add queue");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
