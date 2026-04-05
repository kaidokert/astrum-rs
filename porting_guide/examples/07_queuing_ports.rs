//! Two-partition IPC via kernel queuing ports.
//!
//! Producer (P0) sends 32-byte messages with embedded sequence numbers.
//! Consumer (P1) receives each message and validates ordering and content.
//! The SysTick hook monitors message counts and exits with PASS after 50+
//! successful exchanges, or FAIL on timeout, data corruption, or reordering.
//!
//! This demonstrates the kernel's queuing port API: port creation, routing
//! (`connect_ports`), and the plib `sys_queuing_send` / `sys_queuing_recv`
//! syscall wrappers.
//!
//! # Run
//!
//! ```text
//! cargo run --target thumbv7m-none-eabi \
//!     --features board-qemu,log-semihosting \
//!     --example 07_queuing_ports
//! ```

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::sampling::PortDirection;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgRich, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

// ── Configuration ──────────────────────────────────────────────────
// MsgRich gives QS=4, QD=4, QM=64, QW=4 — enough for 32-byte messages.

kernel::kernel_config!(
    Cfg < Partitions2,
    SyncMinimal,
    MsgRich,
    PortsTiny,
    DebugEnabled > {}
);

// ── Shared state ───────────────────────────────────────────────────

const MSG_SIZE: usize = 32;
/// Receive buffer must match the kernel's QM (max message size) because the
/// QueuingRecv syscall handler validates `check_user_ptr(buf, QM)`.
const RECV_BUF_SIZE: usize = 64;
const GOAL: u32 = 50;
const TIMEOUT: u32 = 2000;

/// Count of messages the consumer has successfully validated.
static RECV_COUNT: AtomicU32 = AtomicU32::new(0);
/// Set to non-zero if the consumer detects bad ordering or data.
static DATA_ERROR: AtomicU32 = AtomicU32::new(0);
/// Count of messages the producer has successfully sent.
static SEND_COUNT: AtomicU32 = AtomicU32::new(0);

// ── SysTick hook ───────────────────────────────────────────────────

kernel::define_harness!(Cfg, |tick, _k| {
    let err = DATA_ERROR.load(Ordering::Acquire);
    if err != 0 {
        klog!("07_queuing_ports: FAIL data error (code={:#x})", err);
        kernel::kexit!(failure);
    }

    let recv = RECV_COUNT.load(Ordering::Acquire);
    let send = SEND_COUNT.load(Ordering::Acquire);

    if recv >= GOAL {
        klog!("07_queuing_ports: PASS (sent={}, recv={})", send, recv);
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT {
        klog!(
            "07_queuing_ports: FAIL timeout (sent={}, recv={})",
            send,
            recv
        );
        kernel::kexit!(failure);
    }

    // Suppress unused-variable warnings when klog is a no-op.
    let _ = (send, recv);
});

// ── Message format ─────────────────────────────────────────────────
// Bytes 0..4  : sequence number (little-endian u32)
// Bytes 4..8  : bitwise NOT of sequence number (integrity check)
// Bytes 8..32 : filled with (seq & 0xFF) as a pattern byte

fn build_message(seq: u32, buf: &mut [u8; MSG_SIZE]) {
    let seq_bytes = seq.to_le_bytes();
    let inv_bytes = (!seq).to_le_bytes();
    let mut i = 0;
    while i < 4 {
        if let (Some(s), Some(v)) = (buf.get_mut(i), seq_bytes.get(i)) {
            *s = *v;
        }
        if let (Some(s), Some(v)) = (buf.get_mut(i + 4), inv_bytes.get(i)) {
            *s = *v;
        }
        i += 1;
    }
    let pattern = (seq & 0xFF) as u8;
    i = 8;
    while i < MSG_SIZE {
        if let Some(slot) = buf.get_mut(i) {
            *slot = pattern;
        }
        i += 1;
    }
}

/// Returns the sequence number if the message is valid, or None.
fn validate_message(buf: &[u8]) -> Option<u32> {
    if buf.len() < MSG_SIZE {
        return None;
    }
    let seq = u32::from_le_bytes([*buf.first()?, *buf.get(1)?, *buf.get(2)?, *buf.get(3)?]);
    let inv = u32::from_le_bytes([*buf.get(4)?, *buf.get(5)?, *buf.get(6)?, *buf.get(7)?]);
    if inv != !seq {
        return None;
    }
    let pattern = (seq & 0xFF) as u8;
    let mut i = 8;
    while i < MSG_SIZE {
        if *buf.get(i)? != pattern {
            return None;
        }
        i += 1;
    }
    Some(seq)
}

// ── Partition entry points ─────────────────────────────────────────

const _: PartitionEntry = producer;

/// Producer: sends sequenced 32-byte messages on queuing port 0.
/// Sends one message per schedule cycle and yields, so the queue (depth 4)
/// never overflows and every send is a successful delivery.
extern "C" fn producer() -> ! {
    let mut seq: u32 = 1;
    let mut msg = [0u8; MSG_SIZE];
    loop {
        build_message(seq, &mut msg);
        match plib::sys_queuing_send(plib::QueuingPortId::new(0), &msg) {
            Ok(_) => {
                SEND_COUNT.store(seq, Ordering::Release);
                seq += 1;
            }
            Err(e) => {
                // Send failed (queue full, invalid port, etc.) — report and halt.
                DATA_ERROR.store(0xBAD1_0000 | e.to_u32(), Ordering::Release);
                loop {
                    cortex_m::asm::nop();
                }
            }
        }
        // Yield remainder of time window so consumer can drain before next send.
        if plib::sys_yield().is_err() {
            DATA_ERROR.store(0xBAD1_00FF, Ordering::Release);
            loop {
                cortex_m::asm::nop();
            }
        }
    }
}

/// Consumer: receives messages on queuing port 1 and validates ordering.
extern "C" fn consumer() -> ! {
    let mut expected_seq: u32 = 1;
    let mut buf = [0u8; RECV_BUF_SIZE];
    loop {
        match plib::sys_queuing_recv(plib::QueuingPortId::new(1), &mut buf) {
            Ok(len) if len as usize >= MSG_SIZE => {
                let msg_slice = match buf.get(..len as usize) {
                    Some(s) => s,
                    None => {
                        DATA_ERROR.store(0xBAD0_0004, Ordering::Release);
                        loop {
                            cortex_m::asm::nop();
                        }
                    }
                };
                match validate_message(msg_slice) {
                    Some(seq) if seq == expected_seq => {
                        expected_seq += 1;
                        RECV_COUNT.store(seq, Ordering::Release);
                    }
                    Some(_seq) => {
                        // Out-of-order message.
                        DATA_ERROR.store(0xBAD0_0002, Ordering::Release);
                        loop {
                            cortex_m::asm::nop();
                        }
                    }
                    None => {
                        // Corrupt message payload.
                        DATA_ERROR.store(0xBAD0_0003, Ordering::Release);
                        loop {
                            cortex_m::asm::nop();
                        }
                    }
                }
            }
            Ok(0) => {
                // Queue empty — yield and retry.
                if plib::sys_yield().is_err() {
                    DATA_ERROR.store(0xBAD0_00FF, Ordering::Release);
                    loop {
                        cortex_m::asm::nop();
                    }
                }
            }
            Ok(_short) => {
                // Unexpected short message.
                DATA_ERROR.store(0xBAD0_0001, Ordering::Release);
                loop {
                    cortex_m::asm::nop();
                }
            }
            Err(e) => {
                // Unexpected kernel error — report and halt.
                DATA_ERROR.store(0xBAD0_E000 | e.to_u32(), Ordering::Release);
                loop {
                    cortex_m::asm::nop();
                }
            }
        }
    }
}

// ── main: build kernel with queuing ports, boot ────────────────────

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("07_queuing_ports: queuing port IPC between two partitions");

    // Schedule: P0 (producer) and P1 (consumer) each get 4-tick windows,
    // separated by 1-tick system windows.
    let mut sched: ScheduleTable<{ Cfg::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");
    sched.add_system_window(1).expect("syswin 1");

    let entries: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(producer as PartitionEntry, 0),
        PartitionSpec::new(consumer as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &entries).expect("init_kernel");

    // Create queuing ports: source (id=0) for producer, destination (id=1) for consumer.
    let src = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("src port");
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("dst port");
    k.queuing_mut().connect_ports(src, dst).expect("connect");

    store_kernel(&mut k);
    match boot(p).expect("07_queuing_ports: boot") {}
}
