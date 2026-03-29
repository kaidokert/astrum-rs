#![cfg(loom)]

//! Loom concurrency tests for SemaphorePool wait/signal contention.
//! Verifies count invariant: 0 <= count <= max_count across all interleavings.

use kernel::loom_compat::{model, thread, Arc, Mutex};
use kernel::partition::{MpuRegion, PartitionControlBlock as PCB, PartitionState, PartitionTable};
use kernel::semaphore::{Semaphore, SemaphoreError, SemaphorePool};

fn lock<T>(m: &Mutex<T>) -> loom::sync::MutexGuard<'_, T> {
    m.lock().unwrap()
}

const R: MpuRegion = MpuRegion::new(0, 4096, 0);

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

struct State {
    pool: SemaphorePool<4, 4>,
    parts: PartitionTable<4>,
}

#[test]
fn two_partitions_race_to_wait_count1() {
    model(|| {
        let mut pool = SemaphorePool::<4, 4>::new();
        pool.add(Semaphore::new(1, 1)).unwrap();
        let state = Arc::new(Mutex::new(State {
            pool,
            parts: make_parts(2),
        }));

        // Thread A: partition 0 waits on semaphore 0.
        let s1 = state.clone();
        let t0 = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.wait(parts, 0, 0)
        });

        // Thread B: partition 1 waits on semaphore 0.
        let s2 = state.clone();
        let t1 = thread::spawn(move || {
            let mut g = lock(&s2);
            let State { pool, parts } = &mut *g;
            pool.wait(parts, 0, 1)
        });

        let r0 = t0.join().unwrap();
        let r1 = t1.join().unwrap();

        // Both must succeed. Exactly one acquired (Ok(true)), other blocked (Ok(false)).
        assert!(r0.is_ok(), "P0 wait must not error: {:?}", r0);
        assert!(r1.is_ok(), "P1 wait must not error: {:?}", r1);

        let acquired_0 = r0.unwrap();
        let acquired_1 = r1.unwrap();
        assert_ne!(
            acquired_0, acquired_1,
            "exactly one must acquire (true), the other must block (false)"
        );

        // Count invariant: count must be 0.
        let g = lock(&state);
        let sem = g.pool.get(0).unwrap();
        assert_eq!(sem.count(), 0, "count must be 0 after both operations");

        // The blocker must be in Waiting state.
        if acquired_0 {
            assert_eq!(
                g.parts.get(1).unwrap().state(),
                PartitionState::Waiting,
                "P1 must be waiting"
            );
            assert_eq!(
                g.parts.get(0).unwrap().state(),
                PartitionState::Running,
                "P0 acquired, still running"
            );
        } else {
            assert_eq!(
                g.parts.get(0).unwrap().state(),
                PartitionState::Waiting,
                "P0 must be waiting"
            );
            assert_eq!(
                g.parts.get(1).unwrap().state(),
                PartitionState::Running,
                "P1 acquired, still running"
            );
        }
    });
}

#[test]
fn signal_wakes_blocked_waiter() {
    model(|| {
        let mut pool = SemaphorePool::<4, 4>::new();
        pool.add(Semaphore::new(0, 1)).unwrap();
        let state = Arc::new(Mutex::new(State {
            pool,
            parts: make_parts(2),
        }));

        // Setup: P0 waits on empty semaphore → blocks.
        {
            let mut g = lock(&state);
            let State { pool, parts } = &mut *g;
            let r = pool.wait(parts, 0, 0);
            assert_eq!(r, Ok(false), "P0 must block on zero-count semaphore");
            assert_eq!(parts.get(0).unwrap().state(), PartitionState::Waiting);
        }

        // Thread A: P1 signals semaphore 0.
        let s1 = state.clone();
        let t_signal = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.signal(parts, 0)
        });

        // Thread B: check state after potential signal.
        let s2 = state.clone();
        let t_check = thread::spawn(move || {
            let g = lock(&s2);
            (
                g.pool.get(0).unwrap().count(),
                g.parts.get(0).unwrap().state(),
            )
        });

        let signal_res = t_signal.join().unwrap();
        assert!(signal_res.is_ok(), "signal must succeed: {:?}", signal_res);

        let (count, p0_state) = t_check.join().unwrap();

        // Checker saw either pre-signal or post-signal state.
        match p0_state {
            PartitionState::Waiting => {
                // Pre-signal: count still 0, P0 still waiting.
                assert_eq!(count, 0, "pre-signal: count must be 0");
            }
            PartitionState::Ready => {
                // Post-signal: waiter woken, count stays 0 (signal went to waiter).
                assert_eq!(count, 0, "post-signal: count must be 0 (went to waiter)");
            }
            other => panic!("unexpected P0 state: {:?}", other),
        }

        // Final state: P0 must be Ready (woken), count must be 0.
        let g = lock(&state);
        assert_eq!(g.parts.get(0).unwrap().state(), PartitionState::Ready);
        assert_eq!(g.pool.get(0).unwrap().count(), 0);
    });
}

#[test]
fn signal_at_max_count_returns_overflow() {
    model(|| {
        let mut pool = SemaphorePool::<4, 4>::new();
        // Semaphore at max count (count == max_count == 2).
        pool.add(Semaphore::new(2, 2)).unwrap();
        let state = Arc::new(Mutex::new(State {
            pool,
            parts: make_parts(2),
        }));

        // Thread A: P0 signals (should overflow).
        let s1 = state.clone();
        let t0 = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            pool.signal(parts, 0)
        });

        // Thread B: P1 also signals (should also overflow).
        let s2 = state.clone();
        let t1 = thread::spawn(move || {
            let mut g = lock(&s2);
            let State { pool, parts } = &mut *g;
            pool.signal(parts, 0)
        });

        let r0 = t0.join().unwrap();
        let r1 = t1.join().unwrap();

        // Both must return CountOverflow since count == max_count.
        assert_eq!(
            r0,
            Err(SemaphoreError::CountOverflow),
            "signal at max must return CountOverflow"
        );
        assert_eq!(
            r1,
            Err(SemaphoreError::CountOverflow),
            "signal at max must return CountOverflow"
        );

        // Count invariant: count must not exceed max_count.
        let g = lock(&state);
        let sem = g.pool.get(0).unwrap();
        assert_eq!(sem.count(), 2, "count must remain at max_count");
        assert!(
            sem.count() <= sem.max_count(),
            "count {} must not exceed max_count {}",
            sem.count(),
            sem.max_count()
        );
    });
}

#[test]
fn concurrent_signal_and_wait_count_invariant() {
    model(|| {
        let mut pool = SemaphorePool::<4, 4>::new();
        // Start with count=1, max=2.
        pool.add(Semaphore::new(1, 2)).unwrap();
        let state = Arc::new(Mutex::new(State {
            pool,
            parts: make_parts(2),
        }));

        // Thread A: P0 waits on semaphore 0.
        let s1 = state.clone();
        let t_wait = thread::spawn(move || {
            let mut g = lock(&s1);
            let State { pool, parts } = &mut *g;
            let result = pool.wait(parts, 0, 0);
            // After wait, check count invariant under the same lock.
            let sem = pool.get(0).unwrap();
            assert!(
                sem.count() <= sem.max_count(),
                "after wait: count {} exceeds max_count {}",
                sem.count(),
                sem.max_count()
            );
            result
        });

        // Thread B: P1 signals semaphore 0.
        let s2 = state.clone();
        let t_signal = thread::spawn(move || {
            let mut g = lock(&s2);
            let State { pool, parts } = &mut *g;
            let result = pool.signal(parts, 0);
            // After signal, check count invariant under the same lock.
            let sem = pool.get(0).unwrap();
            assert!(
                sem.count() <= sem.max_count(),
                "after signal: count {} exceeds max_count {}",
                sem.count(),
                sem.max_count()
            );
            result
        });

        let wait_res = t_wait.join().unwrap();
        let signal_res = t_signal.join().unwrap();

        // Wait must succeed (count was 1, or signal happened first making it 2).
        assert!(wait_res.is_ok(), "wait must not error: {:?}", wait_res);
        // Wait must have acquired (count >= 1 in all orderings).
        assert_eq!(wait_res, Ok(true), "wait must acquire (count >= 1)");

        assert!(signal_res.is_ok(), "signal must succeed: {:?}", signal_res);

        // Final count invariant.
        let g = lock(&state);
        let sem = g.pool.get(0).unwrap();
        let final_count = sem.count();
        assert!(
            final_count <= sem.max_count(),
            "final count {} exceeds max_count {}",
            final_count,
            sem.max_count()
        );
        // count must be 1 regardless of ordering:
        // wait first: 1→0, signal: 0→1 = 1
        // signal first: 1→2, wait: 2→1 = 1
        assert_eq!(final_count, 1, "final count must be 1");
    });
}
