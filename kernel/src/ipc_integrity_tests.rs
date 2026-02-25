//! IPC data-integrity round-trip tests (subtask 274).
//!
//! Each test sends a known byte pattern through an IPC channel and verifies
//! byte-exact integrity on the receive side.

#[cfg(feature = "ipc-blackboard")]
use crate::blackboard::{Blackboard, ReadBlackboardOutcome};
use crate::message::{MessageQueue, RecvOutcome, SendOutcome};
use crate::queuing::{QueuingPortPool, RecvQueuingOutcome, SendQueuingOutcome};
use crate::sampling::PortDirection;

/// Single-word (4-byte) message-queue round-trip.
#[test]
fn msg_single_word_integrity() {
    let mut q = MessageQueue::<4, 4, 4>::new();
    let pattern = [0xDE, 0xAD, 0xBE, 0xEF];
    let send = q.send(0, &pattern).unwrap();
    assert_eq!(
        send,
        SendOutcome::Delivered {
            wake_receiver: None
        }
    );
    let mut buf = [0u8; 4];
    let recv = q.recv(1, &mut buf).unwrap();
    assert_eq!(recv, RecvOutcome::Received { wake_sender: None });
    assert_eq!(buf, pattern, "single-word data corrupted in transit");
}

/// Multi-word (16-byte) message-queue round-trip with distinct per-byte pattern.
#[test]
fn msg_multi_word_integrity() {
    let mut q = MessageQueue::<4, 16, 4>::new();
    let pattern: [u8; 16] = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
        0xFF,
    ];
    let send = q.send(0, &pattern).unwrap();
    assert_eq!(
        send,
        SendOutcome::Delivered {
            wake_receiver: None
        }
    );
    let mut buf = [0u8; 16];
    let recv = q.recv(1, &mut buf).unwrap();
    assert_eq!(recv, RecvOutcome::Received { wake_sender: None });
    for i in 0..16 {
        assert_eq!(buf[i], pattern[i], "byte {i} corrupted");
    }
}

/// Queuing port: max-size buffer round-trip via pool routing (Source → Destination).
#[test]
fn queuing_max_size_routed_integrity() {
    let mut pool = QueuingPortPool::<4, 4, 32, 4>::new();
    let src = pool.create_port(PortDirection::Source).unwrap();
    let dst = pool.create_port(PortDirection::Destination).unwrap();
    pool.connect_ports(src, dst).unwrap();
    let mut pattern = [0u8; 32];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = i as u8;
    }
    let send = pool.send_routed(src, 0, &pattern, 0, 0).unwrap();
    assert_eq!(
        send,
        SendQueuingOutcome::Delivered {
            wake_receiver: None
        }
    );
    let mut buf = [0u8; 32];
    let recv = pool
        .receive_queuing_message(dst, 1, &mut buf, 0, 0)
        .unwrap();
    assert_eq!(
        recv,
        RecvQueuingOutcome::Received {
            msg_len: 32,
            wake_sender: None
        }
    );
    assert_eq!(buf, pattern, "max-size queuing data corrupted");
}

/// Blackboard display/read round-trip with structured payload.
#[cfg(feature = "ipc-blackboard")]
#[test]
fn blackboard_display_read_integrity() {
    let mut bb = Blackboard::<16, 4>::new(0);
    let pattern = [0xCA, 0xFE, 0xBA, 0xBE, 0x01, 0x02, 0x03];
    let woken: heapless::Vec<u8, 4> = bb.display(&pattern).unwrap();
    assert!(woken.is_empty());
    let mut buf = [0u8; 16];
    let outcome = bb.read_timed(0, &mut buf, 0, 0).unwrap();
    assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 7 });
    assert_eq!(&buf[..7], &pattern, "blackboard data corrupted");
}

/// Queuing port: zero-length message round-trip (edge case).
#[test]
fn queuing_zero_length_integrity() {
    let mut pool = QueuingPortPool::<4, 4, 16, 4>::new();
    let src = pool.create_port(PortDirection::Source).unwrap();
    let dst = pool.create_port(PortDirection::Destination).unwrap();
    pool.connect_ports(src, dst).unwrap();
    let send = pool.send_routed(src, 0, &[], 0, 0).unwrap();
    assert_eq!(
        send,
        SendQueuingOutcome::Delivered {
            wake_receiver: None
        }
    );
    let mut buf = [0xFFu8; 16];
    let recv = pool
        .receive_queuing_message(dst, 1, &mut buf, 0, 0)
        .unwrap();
    assert_eq!(
        recv,
        RecvQueuingOutcome::Received {
            msg_len: 0,
            wake_sender: None
        }
    );
    assert_eq!(buf, [0xFF; 16], "zero-length recv must not touch buffer");
}
