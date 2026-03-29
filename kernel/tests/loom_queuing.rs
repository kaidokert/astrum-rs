#![cfg(loom)]

//! Loom concurrency tests for QueuingPort send/recv races.
//!
//! Each test wraps a `QueuingPort` or `QueuingPortPool` in `Arc<Mutex<_>>`
//! and spawns threads that perform concurrent send/recv operations. Loom
//! exhaustively explores all thread interleavings to verify no message loss
//! or duplication.

use kernel::loom_compat::{model, thread, Arc, Mutex};
use kernel::queuing::{
    QueuingError, QueuingPort, QueuingPortPool, RecvQueuingOutcome, SendQueuingOutcome,
};
use kernel::sampling::PortDirection;

/// Helper: lock a loom Mutex (returns MutexGuard, panics on poison).
fn lock<T>(m: &Mutex<T>) -> loom::sync::MutexGuard<'_, T> {
    m.lock().unwrap()
}

/// Two concurrent senders on a depth-1 port: exactly one succeeds, one gets
/// QueueFull, and the total number of enqueued messages is exactly 1.
/// Verify the delivered payload matches the successful sender.
#[test]
fn two_senders_depth1_one_succeeds_one_full() {
    model(|| {
        // Pool with depth-1 destination so we can send via source, recv via destination.
        let pool = Arc::new(Mutex::new(QueuingPortPool::<4, 1, 4, 2>::new()));

        let (src, dst) = {
            let mut g = lock(&pool);
            let s = g.create_port(PortDirection::Source).unwrap();
            let d = g.create_port(PortDirection::Destination).unwrap();
            g.connect_ports(s, d).unwrap();
            (s, d)
        };

        let p1 = pool.clone();
        let t1 = thread::spawn(move || lock(&p1).send_routed(src, 1, &[0xAA], 0, 0));

        let p2 = pool.clone();
        let t2 = thread::spawn(move || lock(&p2).send_routed(src, 2, &[0xBB], 0, 0));

        let r1 = t1.join().unwrap();
        let r2 = t2.join().unwrap();

        // Exactly one must succeed (Delivered), the other must get QueueFull.
        let ok1 = r1.is_ok();
        let ok2 = r2.is_ok();
        let full1 = matches!(r1, Err(QueuingError::QueueFull));
        let full2 = matches!(r2, Err(QueuingError::QueueFull));
        assert!(
            (ok1 && full2) || (full1 && ok2),
            "exactly one Delivered + one QueueFull, got r1={:?} r2={:?}",
            r1,
            r2
        );

        // Destination must contain exactly 1 message.
        let g = lock(&pool);
        assert_eq!(g.get(dst).unwrap().nb_messages(), 1);
        drop(g);

        // Verify which message is in the port matches the successful sender.
        let mut buf = [0u8; 4];
        let recv_out = lock(&pool)
            .receive_queuing_message(dst, 99, &mut buf, 0, 0)
            .unwrap();
        match recv_out {
            RecvQueuingOutcome::Received { msg_len, .. } => {
                assert_eq!(msg_len, 1);
                let expected: u8 = if ok1 { 0xAA } else { 0xBB };
                assert_eq!(
                    buf[0], expected,
                    "buffer must contain the successful sender's data"
                );
            }
            other => panic!("expected Received, got {:?}", other),
        }
    });
}

/// Concurrent send + recv via a connected port pair: one thread sends on the
/// source (routed to destination), another thread receives from the destination.
/// Pre-load one message so recv always has data. After both complete, verify
/// total messages = pre-loaded + sent - received = consistent (no loss/dup).
#[test]
fn concurrent_send_recv_no_message_loss() {
    model(|| {
        // Pool: 4 ports max, depth 2, msg size 4, wait-queue 2.
        let pool = Arc::new(Mutex::new(QueuingPortPool::<4, 2, 4, 2>::new()));

        let (src, dst) = {
            let mut g = lock(&pool);
            let s = g.create_port(PortDirection::Source).unwrap();
            let d = g.create_port(PortDirection::Destination).unwrap();
            g.connect_ports(s, d).unwrap();
            (s, d)
        };

        // Pre-load one message via send_routed.
        {
            let mut g = lock(&pool);
            let outcome = g.send_routed(src, 10, &[0xDE, 0xAD], 0, 0).unwrap();
            assert!(matches!(outcome, SendQueuingOutcome::Delivered { .. }));
        }

        // Thread A: send a second message.
        let pool1 = pool.clone();
        let t_send = thread::spawn(move || lock(&pool1).send_routed(src, 11, &[0xCA, 0xFE], 0, 0));

        // Thread B: recv from destination, returning the buffer for content verification.
        let pool2 = pool.clone();
        let t_recv = thread::spawn(move || {
            let mut buf = [0u8; 4];
            let res = lock(&pool2).receive_queuing_message(dst, 20, &mut buf, 0, 0);
            (res, buf)
        });

        let send_res = t_send.join().unwrap();
        let (recv_res, recv_buf) = t_recv.join().unwrap();

        // Send must succeed (depth=2, only 1 pre-loaded).
        assert!(send_res.is_ok(), "send must succeed: {:?}", send_res);

        // Recv must succeed (at least 1 message was available).
        assert!(recv_res.is_ok(), "recv must succeed: {:?}", recv_res);
        match recv_res.unwrap() {
            RecvQueuingOutcome::Received { msg_len, .. } => {
                assert_eq!(msg_len, 2, "received message must be 2 bytes");
                // Buffer must contain one of the two sent payloads (no corruption).
                let payload = &recv_buf[..msg_len];
                assert!(
                    payload == [0xDE, 0xAD] || payload == [0xCA, 0xFE],
                    "received payload must be [DE,AD] or [CA,FE], got {:?}",
                    payload
                );
            }
            other => panic!("expected Received, got {:?}", other),
        }

        // Final: started with 1, sent 1 more (+1), received 1 (-1) = 1 remaining.
        let g = lock(&pool);
        assert_eq!(
            g.get(dst).unwrap().nb_messages(),
            1,
            "exactly 1 message must remain"
        );
    });
}

/// Two senders on a full depth-1 port both get QueueFull and are added to the
/// sender wait queue. Drain via recv and verify wake_sender contains a valid
/// task ID from the blocked senders.
#[test]
fn sender_blocked_then_receiver_drains_wake_sender() {
    model(|| {
        // Pool with depth-1 destination, wait-queue capacity 4.
        let pool = Arc::new(Mutex::new(QueuingPortPool::<4, 1, 4, 4>::new()));

        let (src, dst) = {
            let mut g = lock(&pool);
            let s = g.create_port(PortDirection::Source).unwrap();
            let d = g.create_port(PortDirection::Destination).unwrap();
            g.connect_ports(s, d).unwrap();
            (s, d)
        };

        // Fill the queue via source.
        lock(&pool).send_routed(src, 0, &[0x01], 0, 0).unwrap();

        // Two threads try to send on the full port with a timeout so they
        // get added to the sender wait queue instead of immediate QueueFull.
        let p1 = pool.clone();
        let t1 = thread::spawn(move || lock(&p1).send_routed(src, 1, &[0x02], 100, 0));

        let p2 = pool.clone();
        let t2 = thread::spawn(move || lock(&p2).send_routed(src, 2, &[0x03], 100, 0));

        let r1 = t1.join().unwrap();
        let r2 = t2.join().unwrap();

        // Both must be SenderBlocked (queue was full, timeout > 0).
        assert!(
            matches!(r1, Ok(SendQueuingOutcome::SenderBlocked { .. })),
            "sender 1 must be blocked, got {:?}",
            r1
        );
        assert!(
            matches!(r2, Ok(SendQueuingOutcome::SenderBlocked { .. })),
            "sender 2 must be blocked, got {:?}",
            r2
        );

        {
            let g = lock(&pool);
            let dst_port = g.get(dst).unwrap();
            // Still 1 message (neither send succeeded).
            assert_eq!(dst_port.nb_messages(), 1);
            // Both callers are now in the sender wait queue on the destination.
            assert_eq!(dst_port.pending_senders(), 2);
        }

        // Drain the queue via recv on the destination: wake_sender must be a
        // valid task ID from one of the two blocked senders (task 1 or task 2).
        let mut buf = [0u8; 4];
        let recv_out = lock(&pool)
            .receive_queuing_message(dst, 99, &mut buf, 0, 0)
            .unwrap();
        match recv_out {
            RecvQueuingOutcome::Received {
                msg_len,
                wake_sender,
            } => {
                assert_eq!(msg_len, 1);
                assert_eq!(buf[0], 0x01, "drained message must be the original payload");
                let woken =
                    wake_sender.expect("recv must return a wake_sender when senders are queued");
                assert!(
                    woken == 1 || woken == 2,
                    "wake_sender must be task 1 or 2, got {}",
                    woken
                );
            }
            other => panic!("expected Received, got {:?}", other),
        }
    });
}

/// Two concurrent recv calls on an empty Destination port: both get
/// QueueEmpty without panic, and both callers end up in the receiver
/// wait queue.
#[test]
fn concurrent_recv_empty_both_get_queue_empty() {
    model(|| {
        let dst = Arc::new(Mutex::new(QueuingPort::<2, 4, 4>::new(
            PortDirection::Destination,
        )));

        let d1 = dst.clone();
        let t1 = thread::spawn(move || {
            let mut buf = [0u8; 4];
            lock(&d1).recv(1, &mut buf)
        });

        let d2 = dst.clone();
        let t2 = thread::spawn(move || {
            let mut buf = [0u8; 4];
            lock(&d2).recv(2, &mut buf)
        });

        let r1 = t1.join().unwrap();
        let r2 = t2.join().unwrap();

        // Both must get QueueEmpty.
        assert_eq!(r1, Err(QueuingError::QueueEmpty));
        assert_eq!(r2, Err(QueuingError::QueueEmpty));

        let guard = lock(&dst);
        assert_eq!(guard.nb_messages(), 0);
        // Both callers are in the receiver wait queue.
        assert_eq!(guard.pending_receivers(), 2);
    });
}
