#![cfg(loom)]

//! Loom concurrency tests for WaitQueue and TimedWaitQueue.
//!
//! Each test wraps a queue in `Arc<Mutex<_>>` and spawns threads that
//! perform concurrent push/pop/remove operations. Loom exhaustively
//! explores all thread interleavings to verify FIFO invariants hold.

use kernel::loom_compat::{model, thread, Arc, Mutex};
use kernel::waitqueue::{TimedWaitQueue, WaitQueue};

/// Helper: lock a loom Mutex (returns MutexGuard, panics on poison).
fn lock<T>(m: &Mutex<T>) -> loom::sync::MutexGuard<'_, T> {
    m.lock().unwrap()
}

/// Two threads each push one element; total length must be 2 and both
/// elements are retrievable.
#[test]
fn concurrent_pushes_both_succeed() {
    model(|| {
        let q = Arc::new(Mutex::new(WaitQueue::<4>::new()));

        let q1 = q.clone();
        let t1 = thread::spawn(move || {
            lock(&q1).push(1).unwrap();
        });

        let q2 = q.clone();
        let t2 = thread::spawn(move || {
            lock(&q2).push(2).unwrap();
        });

        t1.join().unwrap();
        t2.join().unwrap();

        let mut guard = lock(&q);
        assert_eq!(guard.len(), 2);

        let a = guard.pop_front().unwrap();
        let b = guard.pop_front().unwrap();
        assert!(guard.is_empty());

        let mut got = [a, b];
        got.sort();
        assert_eq!(got, [1, 2]);
    });
}

/// One thread pushes an element, another pops. FIFO order is preserved:
/// the popped element is always the first one pushed (pre-loaded head).
#[test]
fn push_pop_preserves_fifo_order() {
    model(|| {
        let q = Arc::new(Mutex::new(WaitQueue::<4>::new()));

        // Pre-load two entries so pop always has something.
        {
            let mut guard = lock(&q);
            guard.push(10).unwrap();
            guard.push(20).unwrap();
        }

        let q1 = q.clone();
        let t_push = thread::spawn(move || {
            lock(&q1).push(30).unwrap();
        });

        let q2 = q.clone();
        let t_pop = thread::spawn(move || -> u8 { lock(&q2).pop_front().unwrap() });

        t_push.join().unwrap();
        let popped = t_pop.join().unwrap();

        let mut guard = lock(&q);

        // The popped value must be 10 (FIFO head) since push(30) only
        // appends to the back and cannot reorder existing entries.
        assert_eq!(popped, 10);

        // Remaining: 20 then 30.
        assert_eq!(guard.pop_front(), Some(20));
        assert_eq!(guard.pop_front(), Some(30));
        assert!(guard.is_empty());
    });
}

/// One thread pushes an element, another removes a pre-existing element
/// by ID. The remove always finds its target, and the pushed element
/// survives.
#[test]
fn push_and_remove_by_id() {
    model(|| {
        let q = Arc::new(Mutex::new(WaitQueue::<4>::new()));

        // Pre-load entries: 5, 6, 7
        {
            let mut guard = lock(&q);
            guard.push(5).unwrap();
            guard.push(6).unwrap();
            guard.push(7).unwrap();
        }

        let q1 = q.clone();
        let t_push = thread::spawn(move || {
            lock(&q1).push(8).unwrap();
        });

        let q2 = q.clone();
        let t_remove = thread::spawn(move || -> bool { lock(&q2).remove_by_id(6) });

        t_push.join().unwrap();
        let removed = t_remove.join().unwrap();
        assert!(removed);

        let mut guard = lock(&q);
        // 6 was removed; 5, 7, 8 remain in FIFO order.
        assert_eq!(guard.len(), 3);
        assert_eq!(guard.pop_front(), Some(5));
        assert_eq!(guard.pop_front(), Some(7));
        assert_eq!(guard.pop_front(), Some(8));
    });
}

/// TimedWaitQueue: one thread pushes a non-expired entry while another
/// drains expired entries. The non-expired entry must never be lost.
#[test]
fn timed_drain_expired_concurrent_push_preserves_non_expired() {
    model(|| {
        let q = Arc::new(Mutex::new(TimedWaitQueue::<4>::new()));

        // Pre-load: pid 1 expired (expiry 50), pid 2 not expired (expiry 500).
        {
            let mut guard = lock(&q);
            guard.push(1, 50).unwrap();
            guard.push(2, 500).unwrap();
        }

        let q1 = q.clone();
        let t_push = thread::spawn(move || {
            // Push another non-expired entry.
            lock(&q1).push(3, 999).unwrap();
        });

        let q2 = q.clone();
        let t_drain = thread::spawn(move || -> heapless::Vec<u8, 4> {
            let mut guard = lock(&q2);
            let mut expired = heapless::Vec::<u8, 4>::new();
            guard.drain_expired(100, &mut expired);
            expired
        });

        t_push.join().unwrap();
        let expired = t_drain.join().unwrap();

        // pid 1 (expiry 50) must have been drained.
        assert_eq!(expired.as_slice(), &[1]);

        let mut guard = lock(&q);
        // pid 2 and pid 3 must both survive (non-expired).
        assert_eq!(guard.len(), 2);
        let a = guard.pop_front().unwrap().0;
        let b = guard.pop_front().unwrap().0;
        let mut remaining = [a, b];
        remaining.sort();
        assert_eq!(remaining, [2, 3]);
        assert!(guard.is_empty());
    });
}
