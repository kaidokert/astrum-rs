//! Host-mode integration tests for the multi-threaded partition lifecycle.
//!
//! These tests exercise thread creation, scheduling (round-robin and static
//! priority), suspend/resume, and stop — all through the syscall handler
//! functions in `svc::thread`.

use super::*;
use crate::svc::thread as svc_thread;
use rtos_traits::ids::{PartitionId, ThreadId};
use rtos_traits::thread::{SchedulingPolicy, ThreadState};

fn pid(v: u32) -> PartitionId {
    PartitionId::new(v)
}

/// Build a partition table with one partition whose stack is 4096 bytes and
/// max_threads = 4 (the default ThreadTable capacity).  The partition is
/// transitioned to Running so syscall handlers accept it.
fn make_pt() -> PartitionTable<4> {
    let mut t = PartitionTable::new();
    let base = 0x2000_0000u32;
    let size = 4096u32;
    t.add(PartitionControlBlock::new(
        0,
        0x0800_0000u32,
        base,
        base + size,
        MpuRegion::new(base, size, 0),
    ))
    .unwrap();
    t.get_mut(0)
        .unwrap()
        .transition(crate::partition::PartitionState::Running)
        .unwrap();
    t
}

// ── AC-1: create partition with max_threads=4, verify main thread ────

#[test]
fn partition_has_main_thread_on_creation() {
    let pt = make_pt();
    let pcb = pt.get(0).unwrap();
    let table = pcb.thread_table();

    assert_eq!(table.capacity(), 4, "max_threads must be 4");
    assert_eq!(table.thread_count(), 1, "main thread must exist");

    let main = table.get(ThreadId::new(0)).unwrap();
    assert_eq!(main.state, ThreadState::Running);
    assert_eq!(main.priority, 0);
    assert_eq!(main.entry_point, 0x0800_0000);
}

// ── AC-2: dispatch SYS_THREAD_CREATE twice, verify thread_count=3 ───

#[test]
fn create_two_threads_via_dispatch_yields_count_3() {
    let mut pt = make_pt();

    // NOTE: stack_size_hint=0 is valid — the parameter is currently ignored
    // and stacks are equally partitioned. TODO: update when variable-size
    // sub-stacks are implemented.
    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1), "first create failed: {t1:#x}");
    assert_eq!(t1, 1, "first created thread id must be 1");

    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2), "second create failed: {t2:#x}");
    assert_eq!(t2, 2, "second created thread id must be 2");

    assert_eq!(
        pt.get(0).unwrap().thread_table().thread_count(),
        3,
        "thread_count must be 3 after creating 2 additional threads"
    );
}

// ── AC-3: round-robin advance cycles through 3 threads ──────────────

#[test]
fn round_robin_cycles_three_threads() {
    let mut pt = make_pt();

    // Create 2 more threads (total 3: T0=Running, T1=Ready, T2=Ready).
    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2));

    let table = pt.get_mut(0).unwrap().thread_table_mut();
    assert_eq!(table.thread_count(), 3);
    assert_eq!(table.scheduling_policy(), SchedulingPolicy::RoundRobin);

    // Advance 1: T0 -> T1
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(1));

    // Advance 2: T1 -> T2
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2));

    // Advance 3: T2 -> T0 (wraps around)
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(0));

    // Advance 4: T0 -> T1 (cycle repeats)
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(1));
}

// ── AC-4: static-priority advance always picks highest priority ──────

#[test]
fn static_priority_always_picks_highest() {
    let mut pt = make_pt();

    // T0 (main) has priority 0. Create T1 with priority 5 and T2 with priority 3.
    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 5, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 3, 0);
    assert!(!SvcError::is_error(t2));

    {
        let table = pt.get_mut(0).unwrap().thread_table_mut();
        table.set_scheduling_policy(SchedulingPolicy::StaticPriority);

        // T0 is Running (priority 0 = highest). After demoting T0 to Ready,
        // priority picks T0 again (lowest number wins).
        let next = table.advance_intra_schedule().unwrap();
        assert_eq!(next, ThreadId::new(0), "T0 (prio 0) must win");
    }

    // Suspend T0 via the syscall handler so it's no longer schedulable.
    let r = svc_thread::handle_thread_suspend(&mut pt, pid(0), 0);
    assert_eq!(r, 0, "suspend T0 must succeed");
    let table = pt.get_mut(0).unwrap().thread_table_mut();

    // Now T2 (prio 3) beats T1 (prio 5).
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2), "T2 (prio 3) must beat T1 (prio 5)");

    // Repeatedly: T2 keeps winning.
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2), "T2 must keep winning");
}

// ── AC-5: suspend middle thread, verify skipped in scheduling ────────

#[test]
fn suspended_thread_skipped_in_round_robin() {
    let mut pt = make_pt();

    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2));

    // Suspend T1 (the "middle" thread).
    let r = svc_thread::handle_thread_suspend(&mut pt, pid(0), t1);
    assert_eq!(r, 0, "suspend must succeed");

    let table = pt.get_mut(0).unwrap().thread_table_mut();

    // Advance from T0: should skip suspended T1 and pick T2.
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2), "must skip suspended T1");

    // Advance from T2: should skip suspended T1 and pick T0.
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(0), "must skip suspended T1 again");

    // Full cycle: T0 -> T2 -> T0 (T1 never scheduled).
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2));
}

// ── AC-6: resume suspended thread, verify re-enters rotation ────────

#[test]
fn resumed_thread_re_enters_round_robin() {
    let mut pt = make_pt();

    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2));

    // Suspend T1, advance once (T0 -> T2, skipping T1).
    assert_eq!(svc_thread::handle_thread_suspend(&mut pt, pid(0), t1), 0);
    let table = pt.get_mut(0).unwrap().thread_table_mut();
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2));

    // Resume T1 via syscall handler.
    assert_eq!(svc_thread::handle_thread_resume(&mut pt, pid(0), t1), 0);
    assert_eq!(
        pt.get(0)
            .unwrap()
            .thread_table()
            .get(ThreadId::new(t1 as u8))
            .unwrap()
            .state,
        ThreadState::Ready,
        "resumed thread must be Ready"
    );

    let table = pt.get_mut(0).unwrap().thread_table_mut();
    // Current is T2. Advance should now find T0 (next in RR from T2).
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(0));

    // From T0, next should be T1 (re-entered rotation).
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(1), "resumed T1 must re-enter rotation");

    // Full 3-thread cycle works again: T1 -> T2
    let next = table.advance_intra_schedule().unwrap();
    assert_eq!(next, ThreadId::new(2));
}

// ── AC-7: stop thread, verify count decreases and never scheduled ───

#[test]
fn stopped_thread_excluded_from_scheduling() {
    let mut pt = make_pt();

    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2));

    assert_eq!(pt.get(0).unwrap().thread_table().thread_count(), 3);
    assert_eq!(pt.get(0).unwrap().thread_table().runnable_count(), 3);

    // Stop T1.
    let r = svc_thread::handle_thread_stop(&mut pt, pid(0), t1);
    assert_eq!(r, 0, "stop must succeed");

    // thread_count still 3: stopped threads retain their slot in the table
    // (slots are freed only on join/delete, which is not yet implemented).
    // The scheduler correctness is verified via runnable_count instead.
    assert_eq!(pt.get(0).unwrap().thread_table().thread_count(), 3);
    assert_eq!(
        pt.get(0).unwrap().thread_table().runnable_count(),
        2,
        "runnable_count must decrease after stop"
    );

    let table = pt.get_mut(0).unwrap().thread_table_mut();

    // Cycle multiple times: T1 must never appear.
    for _ in 0..6 {
        let next = table.advance_intra_schedule().unwrap();
        assert_ne!(
            next,
            ThreadId::new(t1 as u8),
            "stopped thread must never be scheduled"
        );
    }
}

/// Verify that stopping a second thread further reduces runnable_count
/// and that the last remaining thread keeps running.
#[test]
fn stop_two_threads_leaves_sole_runner() {
    let mut pt = make_pt();

    let t1 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_1000, 1, 0);
    assert!(!SvcError::is_error(t1));
    let t2 = svc_thread::handle_thread_create(&mut pt, pid(0), 0x0800_2000, 2, 0);
    assert!(!SvcError::is_error(t2));

    assert_eq!(svc_thread::handle_thread_stop(&mut pt, pid(0), t1), 0);
    assert_eq!(svc_thread::handle_thread_stop(&mut pt, pid(0), t2), 0);

    assert_eq!(pt.get(0).unwrap().thread_table().runnable_count(), 1);

    // With only one runnable thread, advance keeps selecting T0 (no actual switch).
    let table = pt.get_mut(0).unwrap().thread_table_mut();
    let next = table
        .advance_intra_schedule()
        .expect("scheduler must return the sole runnable thread");
    assert_eq!(next, ThreadId::new(0), "only T0 is runnable");
    assert_eq!(table.runnable_count(), 1);
}
