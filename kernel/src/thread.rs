//! Thread table for managing fixed-capacity thread pools.
//!
//! `ThreadTable<MAX>` stores up to `MAX` threads in a fixed-size array,
//! supporting O(1) lookup by `ThreadId` and O(N) insertion (linear scan for
//! the first free slot).

use rtos_traits::ids::ThreadId;
use rtos_traits::thread::{SchedulingPolicy, ThreadControlBlock, ThreadState};

/// Errors returned by [`ThreadTable`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadError {
    /// The thread table is full; no more threads can be added.
    TableFull,
    /// The given thread ID does not refer to a valid thread.
    InvalidThread,
}

/// Fixed-capacity table of thread control blocks.
///
/// `MAX` is a const-generic that determines the maximum number of threads.
/// The table uses a flat `[Option<ThreadControlBlock>; MAX]` array so that
/// `ThreadId::as_raw()` is a direct index — no searching required.
pub struct ThreadTable<const MAX: usize> {
    threads: [Option<ThreadControlBlock>; MAX],
    current_thread: u8,
    scheduling_policy: SchedulingPolicy,
}

impl<const MAX: usize> ThreadTable<MAX> {
    const _ASSERT_MAX_FITS_U8: () =
        assert!(MAX <= 256, "ThreadTable capacity must fit in u8 (max 256)");

    /// Create a new, empty thread table with the given scheduling policy.
    #[allow(path_statements)]
    pub fn new(policy: SchedulingPolicy) -> Self {
        Self::_ASSERT_MAX_FITS_U8;
        Self {
            threads: [None; MAX],
            current_thread: 0,
            scheduling_policy: policy,
        }
    }

    /// Insert a thread control block into the first available slot.
    ///
    /// Returns the `ThreadId` assigned to the slot on success, or
    /// `ThreadError::TableFull` if all `MAX` slots are occupied.
    pub fn add_thread(&mut self, mut tcb: ThreadControlBlock) -> Result<ThreadId, ThreadError> {
        for (i, slot) in self.threads.iter_mut().enumerate() {
            if slot.is_none() {
                let id = ThreadId::new(i as u8);
                tcb.id = id;
                *slot = Some(tcb);
                return Ok(id);
            }
        }
        Err(ThreadError::TableFull)
    }

    /// Look up a thread control block by its ID.
    pub fn get(&self, id: ThreadId) -> Option<&ThreadControlBlock> {
        let idx = id.as_raw() as usize;
        self.threads.get(idx)?.as_ref()
    }

    /// Look up a thread control block by its ID (mutable).
    pub fn get_mut(&mut self, id: ThreadId) -> Option<&mut ThreadControlBlock> {
        let idx = id.as_raw() as usize;
        self.threads.get_mut(idx)?.as_mut()
    }

    /// Return the ID of the currently executing thread, or `None` if no
    /// thread occupies the current slot.
    pub fn current_thread_id(&self) -> Option<ThreadId> {
        let idx = self.current_thread as usize;
        if self.threads.get(idx).and_then(|s| s.as_ref()).is_some() {
            Some(ThreadId::new(self.current_thread))
        } else {
            None
        }
    }

    /// Number of threads currently stored in the table.
    pub fn thread_count(&self) -> usize {
        self.threads.iter().filter(|s| s.is_some()).count()
    }

    /// The scheduling policy configured for this table.
    pub fn scheduling_policy(&self) -> SchedulingPolicy {
        self.scheduling_policy
    }

    /// Pick the next Ready thread using round-robin order.
    ///
    /// Starting from the thread after `current_thread`, scan forward (wrapping
    /// around) and return the first thread in the `Ready` state.  Returns
    /// `None` when no Ready thread exists.
    pub fn pick_next_thread(&self) -> Option<ThreadId> {
        self.find_next_ready_index().and_then(|idx| {
            self.threads
                .get(idx)
                .and_then(|slot| slot.as_ref().map(|tcb| tcb.id))
        })
    }

    /// Return the slot index of the next Ready thread (round-robin scan).
    fn find_next_ready_index(&self) -> Option<usize> {
        let count = self.threads.len();
        if count == 0 {
            return None;
        }
        let start = self.current_thread as usize;
        for offset in 1..=count {
            let idx = (start + offset) % count;
            if let Some(Some(ref tcb)) = self.threads.get(idx) {
                if tcb.state == ThreadState::Ready {
                    return Some(idx);
                }
            }
        }
        None
    }

    /// Advance the round-robin schedule by one step.
    ///
    /// 1. Transition the current Running thread to Ready.
    /// 2. Pick the next Ready thread via [`pick_next_thread`].
    /// 3. Transition that thread to Running and update `current_thread`.
    ///
    /// Returns the `ThreadId` of the newly running thread, or `None` if no
    /// Ready thread was found (in which case no state changes are made to the
    /// current thread).
    pub fn advance_round_robin(&mut self) -> Option<ThreadId> {
        // Demote current thread from Running to Ready (if it is Running).
        let cur = self.current_thread as usize;
        if let Some(ref mut tcb) = self.threads.get_mut(cur).and_then(|s| s.as_mut()) {
            if tcb.state == ThreadState::Running {
                tcb.state = ThreadState::Ready;
            }
        }

        if let Some(idx) = self.find_next_ready_index() {
            if let Some(Some(ref mut tcb)) = self.threads.get_mut(idx) {
                tcb.state = ThreadState::Running;
                self.current_thread = tcb.id.as_raw();
                return Some(tcb.id);
            }
        }

        // No ready thread — restore current thread to Running if it was demoted.
        if let Some(ref mut tcb) = self.threads.get_mut(cur).and_then(|s| s.as_mut()) {
            if tcb.state == ThreadState::Ready {
                tcb.state = ThreadState::Running;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtos_traits::thread::ThreadState;

    /// Helper to build a TCB with sensible defaults.
    fn make_tcb(id: u8, priority: u8) -> ThreadControlBlock {
        ThreadControlBlock {
            stack_pointer: 0x2000_0000 + u32::from(id) * 0x1000,
            id: ThreadId::new(id),
            state: ThreadState::Ready,
            priority,
            stack_base: 0x2000_0000 + u32::from(id) * 0x1000 - 1024,
            stack_size: 1024,
            entry_point: 0x0800_0000 + u32::from(id) * 4,
            r0_arg: u32::from(id),
        }
    }

    #[test]
    fn new_table_is_empty() {
        let table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(table.thread_count(), 0);
        assert_eq!(table.scheduling_policy(), SchedulingPolicy::RoundRobin);
        assert_eq!(table.current_thread_id(), None);
    }

    #[test]
    fn add_up_to_max_then_reject() {
        let mut table = ThreadTable::<2>::new(SchedulingPolicy::RoundRobin);
        let id0 = table.add_thread(make_tcb(0, 1)).unwrap();
        assert_eq!(id0.as_raw(), 0);
        let id1 = table.add_thread(make_tcb(1, 2)).unwrap();
        assert_eq!(id1.as_raw(), 1);
        assert_eq!(table.thread_count(), 2);
        assert_eq!(
            table.add_thread(make_tcb(2, 3)),
            Err(ThreadError::TableFull)
        );
        assert_eq!(table.thread_count(), 2);
    }

    #[test]
    fn add_thread_zero_capacity() {
        let mut table = ThreadTable::<0>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(
            table.add_thread(make_tcb(0, 1)),
            Err(ThreadError::TableFull)
        );
    }

    #[test]
    fn get_returns_correct_data() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        let id = table.add_thread(make_tcb(7, 10)).unwrap();
        let got = table.get(id).unwrap();
        assert_eq!(got.priority, 10);
        assert_eq!(got.r0_arg, 7);
        assert_eq!(got.state, ThreadState::Ready);
    }

    #[test]
    fn add_thread_synchronizes_tcb_id() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        // Pass a TCB with a placeholder id; add_thread must overwrite it with the slot index.
        let id = table.add_thread(make_tcb(99, 1)).unwrap();
        assert_eq!(id.as_raw(), 0);
        assert_eq!(table.get(id).unwrap().id.as_raw(), 0);
    }

    #[test]
    fn get_empty_and_oob_returns_none() {
        let table = ThreadTable::<2>::new(SchedulingPolicy::RoundRobin);
        assert!(table.get(ThreadId::new(0)).is_none());
        assert!(table.get(ThreadId::new(5)).is_none());
        assert!(table.get(ThreadId::new(255)).is_none());
    }

    #[test]
    fn get_mut_modifies_in_place() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        let id = table.add_thread(make_tcb(0, 5)).unwrap();
        let tcb = table.get_mut(id).unwrap();
        assert_eq!(tcb.state, ThreadState::Ready);
        tcb.state = ThreadState::Running;
        tcb.priority = 20;
        // Read back via immutable get
        let tcb = table.get(id).unwrap();
        assert_eq!(tcb.state, ThreadState::Running);
        assert_eq!(tcb.priority, 20);
    }

    #[test]
    fn get_mut_empty_and_oob_returns_none() {
        let mut table = ThreadTable::<2>::new(SchedulingPolicy::RoundRobin);
        assert!(table.get_mut(ThreadId::new(0)).is_none());
        assert!(table.get_mut(ThreadId::new(10)).is_none());
    }

    #[test]
    fn current_thread_id_tracks_slot() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        table.add_thread(make_tcb(0, 1)).unwrap();
        table.add_thread(make_tcb(1, 2)).unwrap();
        assert_eq!(table.current_thread_id(), Some(ThreadId::new(0)));
        table.current_thread = 1;
        assert_eq!(table.current_thread_id(), Some(ThreadId::new(1)));
    }

    #[test]
    fn scheduling_policy_preserved() {
        let rr = ThreadTable::<2>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(rr.scheduling_policy(), SchedulingPolicy::RoundRobin);
        let sp = ThreadTable::<2>::new(SchedulingPolicy::StaticPriority);
        assert_eq!(sp.scheduling_policy(), SchedulingPolicy::StaticPriority);
    }

    #[test]
    fn thread_error_variants() {
        assert!(format!("{:?}", ThreadError::TableFull).contains("TableFull"));
        assert!(format!("{:?}", ThreadError::InvalidThread).contains("InvalidThread"));
        assert_ne!(ThreadError::TableFull, ThreadError::InvalidThread);
    }

    #[test]
    fn single_capacity_table() {
        let mut table = ThreadTable::<1>::new(SchedulingPolicy::RoundRobin);
        let id = table.add_thread(make_tcb(0, 5)).unwrap();
        assert_eq!(table.thread_count(), 1);
        assert_eq!(
            table.add_thread(make_tcb(1, 1)),
            Err(ThreadError::TableFull)
        );
        assert_eq!(table.get(id).unwrap().priority, 5);
    }

    #[test]
    fn all_slots_retain_correct_data() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::StaticPriority);
        for i in 0..4u8 {
            table.add_thread(make_tcb(i, i * 10)).unwrap();
        }
        for i in 0..4u8 {
            let tcb = table.get(ThreadId::new(i)).unwrap();
            assert_eq!(tcb.priority, i * 10);
            assert_eq!(tcb.r0_arg, u32::from(i));
        }
    }

    // ── Round-robin scheduler tests ────────────────────────────────

    /// Helper: build a table with N Ready threads, thread 0 set to Running.
    fn make_rr_table<const N: usize>() -> ThreadTable<N> {
        let mut table = ThreadTable::<N>::new(SchedulingPolicy::RoundRobin);
        for i in 0..N as u8 {
            table.add_thread(make_tcb(i, 1)).unwrap();
        }
        // Mark thread 0 as Running (the "current" thread).
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Running;
        table.current_thread = 0;
        table
    }

    #[test]
    fn pick_next_thread_single_thread_returns_none() {
        // One thread that is Running — no Ready threads to pick.
        let table = make_rr_table::<1>();
        assert_eq!(table.pick_next_thread(), None);
    }

    #[test]
    fn pick_next_thread_two_threads() {
        let table = make_rr_table::<2>();
        // Thread 0 is Running, thread 1 is Ready.
        assert_eq!(table.pick_next_thread(), Some(ThreadId::new(1)));
    }

    #[test]
    fn pick_next_thread_wraps_around() {
        let mut table = make_rr_table::<3>();
        // Move current to thread 2 (Running), others Ready.
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Ready;
        table.get_mut(ThreadId::new(2)).unwrap().state = ThreadState::Running;
        table.current_thread = 2;
        // Should wrap to thread 0.
        assert_eq!(table.pick_next_thread(), Some(ThreadId::new(0)));
    }

    #[test]
    fn pick_next_thread_skips_suspended_and_stopped() {
        let mut table = make_rr_table::<4>();
        // Thread 0: Running (current), 1: Suspended, 2: Stopped, 3: Ready
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Suspended;
        table.get_mut(ThreadId::new(2)).unwrap().state = ThreadState::Stopped;
        assert_eq!(table.pick_next_thread(), Some(ThreadId::new(3)));
    }

    #[test]
    fn pick_next_thread_all_stopped_returns_none() {
        let mut table = make_rr_table::<3>();
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Stopped;
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Stopped;
        table.get_mut(ThreadId::new(2)).unwrap().state = ThreadState::Stopped;
        assert_eq!(table.pick_next_thread(), None);
    }

    #[test]
    fn advance_rr_single_thread_stays_running() {
        let mut table = make_rr_table::<1>();
        // Single thread: demoted to Ready, then picked as the only Ready thread.
        let result = table.advance_round_robin();
        assert_eq!(result, Some(ThreadId::new(0)));
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Running
        );
        assert_eq!(table.current_thread, 0);

        // Repeated advances keep the same thread Running.
        let result2 = table.advance_round_robin();
        assert_eq!(result2, Some(ThreadId::new(0)));
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Running
        );
    }

    #[test]
    fn advance_rr_two_threads_alternate() {
        let mut table = make_rr_table::<2>();
        // Initial: T0=Running, T1=Ready, current=0
        let next = table.advance_round_robin().unwrap();
        assert_eq!(next, ThreadId::new(1));
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Ready
        );
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Running
        );
        assert_eq!(table.current_thread, 1);

        // Advance again: T1=Running→Ready, T0=Ready→Running
        let next = table.advance_round_robin().unwrap();
        assert_eq!(next, ThreadId::new(0));
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Running
        );
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Ready
        );
        assert_eq!(table.current_thread, 0);
    }

    #[test]
    fn advance_rr_three_threads_cycle() {
        let mut table = make_rr_table::<3>();
        // T0=Running, T1=Ready, T2=Ready

        let n1 = table.advance_round_robin().unwrap();
        assert_eq!(n1, ThreadId::new(1));
        assert_eq!(table.current_thread, 1);

        let n2 = table.advance_round_robin().unwrap();
        assert_eq!(n2, ThreadId::new(2));
        assert_eq!(table.current_thread, 2);

        let n3 = table.advance_round_robin().unwrap();
        assert_eq!(n3, ThreadId::new(0));
        assert_eq!(table.current_thread, 0);

        // Full cycle completed — verify states.
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Running
        );
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Ready
        );
        assert_eq!(
            table.get(ThreadId::new(2)).unwrap().state,
            ThreadState::Ready
        );
    }

    #[test]
    fn advance_rr_skips_suspended() {
        let mut table = make_rr_table::<3>();
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Suspended;
        // T0=Running, T1=Suspended, T2=Ready
        let next = table.advance_round_robin().unwrap();
        assert_eq!(next, ThreadId::new(2));
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Suspended
        );
    }

    #[test]
    fn advance_rr_all_stopped_returns_none() {
        let mut table = make_rr_table::<3>();
        // Stop all threads.
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Stopped;
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Stopped;
        table.get_mut(ThreadId::new(2)).unwrap().state = ThreadState::Stopped;
        let result = table.advance_round_robin();
        assert_eq!(result, None);
        // All remain Stopped (no spurious state change).
        for i in 0..3u8 {
            assert_eq!(
                table.get(ThreadId::new(i)).unwrap().state,
                ThreadState::Stopped
            );
        }
    }

    #[test]
    fn advance_rr_empty_table_returns_none() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(table.advance_round_robin(), None);
    }
}
