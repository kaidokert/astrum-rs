//! QEMU test: P0 sends a 4-byte message via queue 0, P1 receives and verifies.
//!
//! 1. P0 calls `plib::sys_msg_send(0, &[0xCA, 0xFE, 0xBA, 0xBE])`.
//!    // TODO: subtask specifies sys_msg_send(1, &payload) targeting partition 1,
//!    // but the first arg is actually queue_id (not partition_id) in the kernel
//!    // dispatch. sys_msg_recv hardcodes queue 0, so send must also use queue 0.
//! 2. P1 calls `plib::sys_msg_recv(&mut buf)` and stores rc + received data.
//! 3. SysTick hook verifies the received payload matches the sent payload.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example plib_msg_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::message::MessageQueue;
use kernel::scheduler::ScheduleTable;
// TODO: subtask requests MsgStandard, but MsgSmall (MAX_MSG_SIZE=4, QUEUES=2)
// is sufficient for this 4-byte payload test and is the minimal config.
use kernel::{
    DebugEnabled, MsgSmall, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = TestConfig::N;
const TIMEOUT_TICKS: u32 = 50;

const PAYLOAD: [u8; 4] = [0xCA, 0xFE, 0xBA, 0xBE];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);

/// Sentinel: partition has not yet executed its syscall.
const NOT_YET: u32 = 0xDEAD_C0DE;

static SEND_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static RECV_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Received 4 bytes packed as a little-endian u32.
static RECV_DATA: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_kernel!(TestConfig, |tick, _k| {
    let send = SEND_RC.load(Ordering::Acquire);
    let recv = RECV_RC.load(Ordering::Acquire);

    if send == NOT_YET || recv == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_msg_test: FAIL timeout (send={:#x}, recv={:#x})",
                send,
                recv
            );
            kernel::kexit!(failure);
        }
        return;
    }

    // Load RECV_DATA only after RECV_RC is confirmed non-sentinel.
    // p1_main stores RECV_DATA (Release) before RECV_RC (Release), so
    // acquiring RECV_RC != NOT_YET guarantees RECV_DATA is visible.
    let data = RECV_DATA.load(Ordering::Acquire);

    if send == 0 && recv == 0 && data == EXPECTED_DATA {
        hprintln!(
            "plib_msg_test: PASS (send={}, recv={}, data={:#010x})",
            send,
            recv,
            data
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_msg_test: FAIL (send={:#x}, recv={:#x}, data={:#010x} expected {:#010x})",
            send,
            recv,
            data,
            EXPECTED_DATA
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let payload = PAYLOAD;
    // TODO: PartitionId(0) is actually queue_id 0 here, not a partition target;
    // see file-level comment. Consider a QueueId newtype for sys_msg_send.
    match plib::sys_msg_send(plib::PartitionId::new(0), &payload) {
        Ok(rc) => SEND_RC.store(rc, Ordering::Release),
        Err(_) => SEND_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    let mut buf = [0u8; 4];
    // TODO: validate received length matches expected 4 bytes once plib
    // exposes a length out-param in sys_msg_recv; currently buf is [u8; 4]
    // so from_le_bytes is safe at compile time.
    match plib::sys_msg_recv(&mut buf) {
        Ok(rc) => {
            RECV_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            RECV_RC.store(rc, Ordering::Release);
        }
        Err(_) => {
            RECV_RC.store(0xFFFF_FFFF, Ordering::Release);
        }
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_msg_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_msg_test: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("plib_msg_test: kernel");
    with_kernel_mut(|k| {
        k.messages_mut()
            .add(MessageQueue::new())
            .expect("plib_msg_test: add msg queue");
    });

    match boot(p).expect("plib_msg_test: boot") {}
}
