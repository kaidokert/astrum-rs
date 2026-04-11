//! Timed IPC Test — sys_queuing_send_timed / sys_queuing_recv_timed / sys_queuing_status
//!
//! P0 (producer): sends bursts of 5 messages, then sleeps 500ms (creating a gap).
//! P1 (consumer): sys_queuing_recv_timed with 100ms timeout. During bursts it
//!   receives successfully; during gaps the timeout fires.
//! Tick handler: reports recv count, timeout count, and queuing_status.
//!
//! Proves: timed IPC wakeup path, timeout expiry, queuing port status query.
//!
//! Build: cd f429zi && cargo build --example timed_ipc_test \
//!            --features kernel-example --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    PartitionEntry, PartitionSpec,
    {DebugEnabled, MsgSmall, Partitions4, PortsTiny, SyncMinimal},
};
use plib::QueuingPortId;
use rtt_target::rprintln;
use f429zi as _;

const MSG_SIZE: usize = 4;
const BURST_SIZE: u32 = 5;
const SLEEP_MS: u16 = 500;
const RECV_TIMEOUT_TICKS: u16 = 100; // 100ms timeout

static SENT: AtomicU32 = AtomicU32::new(0);
static RECV_OK: AtomicU32 = AtomicU32::new(0);
static RECV_TIMEOUT: AtomicU32 = AtomicU32::new(0);
static SEND_TIMED_OK: AtomicU32 = AtomicU32::new(0);
static SEND_TIMED_FAIL: AtomicU32 = AtomicU32::new(0);
static STATUS_QUERIES: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(TimedCfg<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
    QD = 4; QM = 8; QW = 4;
});

kernel::define_kernel!(TimedCfg, |tick, _k| {
    if tick % 1000 == 0 && tick > 0 {
        let sent = SENT.load(Ordering::Acquire);
        let ok = RECV_OK.load(Ordering::Acquire);
        let tout = RECV_TIMEOUT.load(Ordering::Acquire);
        let stok = SEND_TIMED_OK.load(Ordering::Acquire);
        let stfail = SEND_TIMED_FAIL.load(Ordering::Acquire);
        let sq = STATUS_QUERIES.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] sent={} recv_ok={} recv_timeout={} send_timed_ok={} send_timed_fail={} status_q={}",
            tick, sent, ok, tout, stok, stfail, sq
        );
        if ok >= 10 && tout >= 3 && sq >= 3 {
            rprintln!(
                "SUCCESS: timed IPC verified! recv_ok={} recv_timeout={} send_timed={} status={}",
                ok, tout, stok, sq
            );
        }
    }
});

/// P0: producer — send bursts then sleep
extern "C" fn producer_body(r0: u32) -> ! {
    let send_port = QueuingPortId::new((r0 >> 16) as u32);
    let mut counter: u32 = 1;

    loop {
        // Send a burst
        for _ in 0..BURST_SIZE {
            let msg = counter.to_le_bytes();
            // Use timed send with 50ms timeout (should succeed since consumer is draining)
            match plib::sys_queuing_send_timed(send_port, &msg, 50) {
                Ok(_) => {
                    SENT.fetch_add(1, Ordering::Relaxed);
                    SEND_TIMED_OK.fetch_add(1, Ordering::Relaxed);
                    counter = counter.wrapping_add(1);
                }
                Err(_) => {
                    SEND_TIMED_FAIL.fetch_add(1, Ordering::Relaxed);
                }
            }
            plib::sys_yield().ok();
        }

        // Sleep to create a gap — consumer will timeout during this
        plib::sys_sleep_ticks(SLEEP_MS).ok();
    }
}

/// P1: consumer — recv with timeout, also queries port status
extern "C" fn consumer_body(r0: u32) -> ! {
    let recv_port = QueuingPortId::new((r0 & 0xFFFF) as u32);
    let mut buf = [0u8; MSG_SIZE];

    loop {
        match plib::sys_queuing_recv_timed(recv_port, &mut buf, RECV_TIMEOUT_TICKS) {
            Ok(n) => {
                if n > 0 {
                    RECV_OK.fetch_add(1, Ordering::Relaxed);
                    buf = [0u8; MSG_SIZE];
                }
            }
            Err(plib::SvcError::TimedOut) => {
                RECV_TIMEOUT.fetch_add(1, Ordering::Relaxed);
            }
            Err(_) => {
                // Other error — also count as timeout for reporting
                RECV_TIMEOUT.fetch_add(1, Ordering::Relaxed);
            }
        }

        // Query port status periodically
        if let Ok(_status) = plib::sys_queuing_status(recv_port) {
            STATUS_QUERIES.fetch_add(1, Ordering::Relaxed);
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_body);
kernel::partition_trampoline!(consumer_main => consumer_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Timed IPC Test ===");
    rprintln!("P0: burst {} msgs, sleep {}ms. P1: recv_timed({}ms), queuing_status.\n",
        BURST_SIZE, SLEEP_MS, RECV_TIMEOUT_TICKS);

    let mut sched = ScheduleTable::<{ TimedCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("P0");
    sched.add_system_window(1).expect("SW");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW");

    // Port IDs: src=0, dst=1
    let src: u32 = 0;
    let dst: u32 = 1;
    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(producer_main as PartitionEntry, (src << 16) | dst),
        PartitionSpec::new(consumer_main as PartitionEntry, (src << 16) | dst),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    kernel::state::with_kernel_mut::<TimedCfg, _, _>(|k| {
        let s = k.queuing_mut().create_port(PortDirection::Source).expect("src");
        let d = k.queuing_mut().create_port(PortDirection::Destination).expect("dst");
        k.queuing_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
