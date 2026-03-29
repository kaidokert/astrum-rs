#![cfg(loom)]

//! Loom concurrency tests for MutexPool lock contention.
//!
//! Each test wraps a `MutexPool` and `PartitionTable` together in
//! `Arc<Mutex<_>>` and spawns threads that race to lock/unlock/cleanup.
//! Loom exhaustively explores all interleavings to verify ownership
//! invariants hold.

use kernel::loom_compat::{model, thread, Arc, Mutex};
use kernel::mutex::{MutexError, MutexPool};
use kernel::partition::{MpuRegion, PartitionControlBlock as PCB, PartitionState, PartitionTable};

/// Helper: lock a loom Mutex (returns MutexGuard, panics on poison).
fn lock<T>(m: &Mutex<T>) -> loom::sync::MutexGuard<'_, T> {
    m.lock().unwrap()
}

const R: MpuRegion = MpuRegion::new(0, 4096, 0);

/// Build a minimal PartitionTable with `n` partitions, all in Running state.
fn make_parts<const N: usize>(n: u8) -> PartitionTable<N> {
    let mut t = PartitionTable::new();
    for i in 0..n {
        t.add(PCB::new(i, 0x800_0000, 0x2000_0000, 0x2000_0400, R))
            .unwrap();
        t.get_mut(i as usize)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
    }
    t
}

/// Shared state: MutexPool + PartitionTable bundled together.
struct State {
    pool: MutexPool<4, 4>,
    parts: PartitionTable<4>,
}

/// Two partitions racing to lock the same mutex — exactly one acquires,
/// the other blocks (transitions to Waiting).
#[test]
fn two_partitions_race_to_lock() {
    model(|| {
        let state = Arc::new(Mutex::new(State {
            pool: MutexPool::new(1),
            parts: make_parts(2),
        }));

        // Thread A: partition 0 tries to lock mutex 0.
        let s1 = state.clone();
        let t0 = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.lock(parts, 0, 0)
        });

        // Thread B: partition 1 tries to lock mutex 0.
        let s2 = state.clone();
        let t1 = thread::spawn(move || {
            let mut g = lock(&s2);
            let State { pool, parts } = &mut *g;
            pool.lock(parts, 0, 1)
        });

        let r0 = t0.join().unwrap();
        let r1 = t1.join().unwrap();

        // Both must succeed. Exactly one acquired (Ok(true)), other blocked (Ok(false)).
        assert!(r0.is_ok(), "P0 lock must not error: {:?}", r0);
        assert!(r1.is_ok(), "P1 lock must not error: {:?}", r1);

        let acquired_0 = r0.unwrap();
        let acquired_1 = r1.unwrap();
        assert_ne!(
            acquired_0, acquired_1,
            "exactly one must acquire (true), the other must block (false)"
        );

        // Verify ownership invariant: exactly one owner.
        let g = lock(&state);
        let owner = g.pool.owner(0).unwrap();
        assert!(owner.is_some(), "mutex must have an owner");

        // The blocker must be in Waiting state.
        if acquired_0 {
            assert_eq!(owner, Some(0));
            assert_eq!(g.parts.get(1).unwrap().state(), PartitionState::Waiting);
        } else {
            assert_eq!(owner, Some(1));
            assert_eq!(g.parts.get(0).unwrap().state(), PartitionState::Waiting);
        }
    });
}

/// Lock then unlock with waiter — ownership transfers and waiter is woken.
#[test]
fn unlock_transfers_ownership_to_waiter() {
    model(|| {
        let state = Arc::new(Mutex::new(State {
            pool: MutexPool::new(1),
            parts: make_parts(2),
        }));

        // Setup: P0 acquires, P1 blocks.
        {
            let mut g = lock(&state);
            let State { pool, parts } = &mut *g;
            let r0 = pool.lock(parts, 0, 0);
            assert_eq!(r0, Ok(true));
            let r1 = pool.lock(parts, 0, 1);
            assert_eq!(r1, Ok(false));
            assert_eq!(parts.get(1).unwrap().state(), PartitionState::Waiting);
        }

        // Thread A: P0 unlocks mutex 0.
        let s1 = state.clone();
        let t_unlock = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.unlock(parts, 0, 0)
        });

        // Thread B: checks state after potential unlock.
        let s2 = state.clone();
        let t_check = thread::spawn(move || {
            let g = lock(&s2);
            (g.pool.owner(0).unwrap(), g.parts.get(1).unwrap().state())
        });

        let unlock_res = t_unlock.join().unwrap();
        assert!(unlock_res.is_ok(), "unlock must succeed: {:?}", unlock_res);

        let (owner, p1_state) = t_check.join().unwrap();

        // Checker saw either pre-unlock or post-unlock state.
        match owner {
            Some(0) => {
                assert_eq!(
                    p1_state,
                    PartitionState::Waiting,
                    "pre-unlock: P1 still waiting"
                );
            }
            Some(1) => {
                assert_eq!(p1_state, PartitionState::Ready, "post-unlock: P1 woken");
            }
            None => panic!("mutex should not be unowned when there was a waiter"),
            Some(other) => panic!("unexpected owner: {}", other),
        }
    });
}

/// Double-lock by same partition returns AlreadyOwned under all interleavings.
#[test]
fn double_lock_returns_already_owned() {
    model(|| {
        let state = Arc::new(Mutex::new(State {
            pool: MutexPool::new(1),
            parts: make_parts(2),
        }));

        // Setup: P0 acquires mutex 0.
        {
            let mut g = lock(&state);
            let State { pool, parts } = &mut *g;
            let r = pool.lock(parts, 0, 0);
            assert_eq!(r, Ok(true));
        }

        // Thread A: P0 tries to lock again (double-lock).
        let s1 = state.clone();
        let t_double = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.lock(parts, 0, 0)
        });

        // Thread B: P1 tries to lock (should block).
        let s2 = state.clone();
        let t_other = thread::spawn(move || {
            let mut g = lock(&s2);
            let State { pool, parts } = &mut *g;
            pool.lock(parts, 0, 1)
        });

        let double_res = t_double.join().unwrap();
        let other_res = t_other.join().unwrap();

        // P0's double-lock must always return AlreadyOwned.
        assert_eq!(
            double_res,
            Err(MutexError::AlreadyOwned),
            "double-lock must return AlreadyOwned"
        );

        // P1 must block (Ok(false)) since P0 still owns the mutex.
        assert_eq!(other_res, Ok(false), "P1 must block on mutex held by P0");

        // Ownership invariant: P0 is still the owner.
        let g = lock(&state);
        assert_eq!(g.pool.owner(0).unwrap(), Some(0));
        assert_eq!(g.parts.get(1).unwrap().state(), PartitionState::Waiting);
    });
}

/// cleanup_partition releases ownership and removes from wait queues.
#[test]
fn cleanup_partition_releases_and_removes() {
    model(|| {
        let state = Arc::new(Mutex::new(State {
            pool: MutexPool::new(2),
            parts: make_parts(2),
        }));

        // Setup: P0 owns mutex 0 with P1 waiting; P1 owns mutex 1.
        {
            let mut g = lock(&state);
            let State { pool, parts } = &mut *g;
            assert_eq!(pool.lock(parts, 0, 0), Ok(true));
            assert_eq!(pool.lock(parts, 0, 1), Ok(false));
            // P1 is now Waiting; transition back to Running so it can lock mutex 1.
            parts
                .get_mut(1)
                .unwrap()
                .transition(PartitionState::Ready)
                .unwrap();
            parts
                .get_mut(1)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            assert_eq!(pool.lock(parts, 1, 1), Ok(true));
        }

        // Thread A: cleanup P1.
        let s1 = state.clone();
        let t_cleanup = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.release_mutexes_for_partition(1, parts);
        });

        // Thread B: check ownership state.
        let s2 = state.clone();
        let t_check = thread::spawn(move || {
            let g = lock(&s2);
            (g.pool.owner(0).unwrap(), g.pool.owner(1).unwrap())
        });

        t_cleanup.join().unwrap();
        let (owner0, owner1) = t_check.join().unwrap();

        // Intermediate check saw either pre-cleanup or post-cleanup state.
        assert_eq!(owner0, Some(0), "mutex 0 owned by P0 in all interleavings");
        assert!(
            owner1 == Some(1) || owner1 == None,
            "mutex 1 must be Some(1) or None, got {:?}",
            owner1
        );

        // Final state after both threads complete.
        let mut g = lock(&state);
        let final_owner0 = g.pool.owner(0).unwrap();
        let final_owner1 = g.pool.owner(1).unwrap();
        assert_eq!(final_owner0, Some(0), "P0 still owns mutex 0");
        assert_eq!(final_owner1, None, "mutex 1 released after P1 cleanup");

        // Verify P1 was removed from mutex 0's wait queue:
        // unlocking mutex 0 should leave it unowned (no waiters).
        let State { pool, parts } = &mut *g;
        pool.unlock(parts, 0, 0).unwrap();
        assert_eq!(
            pool.owner(0).unwrap(),
            None,
            "no waiters remain after P1 cleanup"
        );
    });
}
