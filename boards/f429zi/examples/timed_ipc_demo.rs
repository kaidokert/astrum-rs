//! Timed IPC Demo — STM32F429ZI (3-partition)
//!
//! Demonstrates both SYS_QUEUING_SEND_TIMED and SYS_QUEUING_RECV_TIMED
//! timeout paths on real hardware.
//!
//! 3 partitions:
//!   P0 (sender):   sends with SHORT timeout — times out when queue full
//!   P1 (CPU hog):  busy-loops, eating schedule slots
//!   P2 (receiver): rarely runs, drains queue when scheduled
//!
//! Schedule: P0(2) → P1(10) → P2(2) → P1(10) → repeat
//! P1 gets 5x more CPU than P0/P2. The queue fills while P1 hogs,
//! causing P0's timed sends to expire. P2's timed recvs also expire
//! when the queue is empty between drains.
//!
//! Success: SEND_TIMEOUT > 0 AND RECV_TIMEOUT > 0 AND SEND_OK > 0 AND RECV_OK > 0
//!
//! Build: cd f429zi && cargo build --example timed_ipc_demo \
//!            --features kernel-example --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugEnabled, MsgMinimal, Partitions3, PortsTiny, SyncMinimal},
};
use plib::QueuingPortId;
use f429zi as _;

const NUM_PARTITIONS: usize = 3;
const SHORT_TIMEOUT: u16 = 0;  // 10ms — shorter than schedule period (24ms)

static SEND_OK: AtomicU32 = AtomicU32::new(0);
static SEND_TIMEOUT: AtomicU32 = AtomicU32::new(0);
static RECV_OK: AtomicU32 = AtomicU32::new(0);
static RECV_TIMEOUT: AtomicU32 = AtomicU32::new(0);
static HOG_LOOPS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(TimedCfg<Partitions3, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    QS = 4; QD = 2; QM = 4; QW = 4;
});

kernel::define_kernel!(TimedCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let sok = SEND_OK.load(Ordering::Acquire);
        let sto = SEND_TIMEOUT.load(Ordering::Acquire);
        let rok = RECV_OK.load(Ordering::Acquire);
        let rto = RECV_TIMEOUT.load(Ordering::Acquire);
        let hog = HOG_LOOPS.load(Ordering::Acquire);
        kernel::klog!("[{:5}ms] SEND ok={} timeout={} | RECV ok={} timeout={} | HOG={}",
            tick, sok, sto, rok, rto, hog);
        if sok > 0 && sto > 0 && rok > 0 && rto > 0 {
            kernel::klog!("SUCCESS: full timed IPC! send_ok={} send_timeout={} recv_ok={} recv_timeout={}",
                sok, sto, rok, rto);
        }
    }
});

/// P0: sender — sends with SHORT timeout. When queue is full (P2 not
/// draining because P1 hogs CPU), the timed send expires.
extern "C" fn sender_body(r0: u32) -> ! {
    let port = QueuingPortId::new(r0);
    let mut seq: u8 = 0;
    loop {
        let msg = [b'Q', seq];
        match plib::sys_queuing_send_timed(port, &msg, SHORT_TIMEOUT) {
            Ok(_) => { SEND_OK.fetch_add(1, Ordering::Release); seq = seq.wrapping_add(1); }
            Err(_) => { SEND_TIMEOUT.fetch_add(1, Ordering::Release); }
        }
    }
}

/// P1: CPU hog — busy-loops without yielding, consuming schedule slots.
extern "C" fn hog_body(_r0: u32) -> ! {
    loop {
        for _ in 0..10_000u32 { core::hint::spin_loop(); }
        HOG_LOOPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// P2: receiver — drains queue with SHORT timeout when scheduled.
extern "C" fn receiver_body(r0: u32) -> ! {
    let port = QueuingPortId::new(r0);
    loop {
        let mut buf = [0u8; 8];
        match plib::sys_queuing_recv_timed(port, &mut buf, SHORT_TIMEOUT) {
            Ok(n) if n > 0 => { RECV_OK.fetch_add(1, Ordering::Release); }
            _ => { RECV_TIMEOUT.fetch_add(1, Ordering::Release); }
        }
    }
}

kernel::partition_trampoline!(sender_main => sender_body);
kernel::partition_trampoline!(hog_main => hog_body);
kernel::partition_trampoline!(receiver_main => receiver_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ TimedCfg::SCHED }>::new();
    // P0 gets 20 ticks (sends many messages), P1 gets 50 (hog), P2 gets 2
    // Sender fills QD=2 queue in first 2 sends, then 3rd send blocks.
    // SHORT_TIMEOUT=10ms < hog slot (50ms) → timed send expires.
    sched.add(ScheduleEntry::new(0, 20)).expect("P0 sender");
    sched.add(ScheduleEntry::new(1, 50)).expect("P1 hog");
    sched.add(ScheduleEntry::new(2, 2)).expect("P2 receiver");
    sched.add_system_window(1).expect("sys_window");

    let cmd_src: u32 = 0;
    let cmd_dst: u32 = 1;

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(sender_main as kernel::PartitionEntry, cmd_src),
        PartitionSpec::entry(hog_main),
        PartitionSpec::new(receiver_main as kernel::PartitionEntry, cmd_dst),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    kernel::state::with_kernel_mut::<TimedCfg, _, _>(|k| {
        let s = k.queuing_mut().create_port(PortDirection::Source).expect("src");
        let d = k.queuing_mut().create_port(PortDirection::Destination).expect("dst");
        k.queuing_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    match boot(p).expect("boot") {}
}
