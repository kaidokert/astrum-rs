//! Thread table for managing fixed-capacity thread pools.
//!
//! `ThreadTable<MAX>` stores up to `MAX` threads in a fixed-size array,
//! supporting O(1) lookup by `ThreadId` and O(N) insertion (linear scan for
//! the first free slot).

use crate::context;
use crate::partition::EntryAddr;
use rtos_traits::ids::ThreadId;
use rtos_traits::thread::{SchedulingPolicy, ThreadControlBlock, ThreadState};

/// Divide a partition's stack into `max_threads` equal sub-stacks, each
/// aligned down to 8 bytes. Thread index 0 (the main thread) occupies the
/// top (highest addresses) of the partition stack.
///
/// The top of the partition stack is aligned down to 8 bytes before dividing,
/// so sub-stack boundaries are correct even when the input range is not
/// pre-aligned.
///
/// Returns `Some((sub_stack_base, sub_stack_size))` for the requested thread,
/// or `None` if `max_threads == 0`, `thread_index >= max_threads`, or
/// arithmetic overflows.
pub fn split_thread_stack(
    partition_stack_base: u32,
    partition_stack_size: u32,
    thread_index: u32,
    max_threads: u32,
) -> Option<(u32, u32)> {
    if max_threads == 0 || thread_index >= max_threads {
        return None;
    }
    let top = partition_stack_base.checked_add(partition_stack_size)?;
    let aligned_top = top & !7;
    let usable = aligned_top.saturating_sub(partition_stack_base);
    let sub_size = (usable / max_threads) & !7;
    let offset = (thread_index.checked_add(1)?).checked_mul(sub_size)?;
    let base = aligned_top.checked_sub(offset)?;
    Some((base, sub_size))
}

/// Initialize a context-switch frame on a thread's sub-stack.
///
/// This is a thin wrapper around [`context::init_stack_frame`] that converts
/// the returned word-index into an absolute stack pointer address.
///
/// `sub_stack` is a `&mut [u32]` slice covering the sub-stack region, and
/// `sub_stack_base` is its absolute base address (as returned by
/// [`split_thread_stack`]).
///
/// Returns the absolute stack pointer (byte address), or `None` if the
/// sub-stack is too small for a context frame.
pub fn init_thread_stack_frame(
    sub_stack: &mut [u32],
    sub_stack_base: u32,
    entry_point: impl Into<EntryAddr>,
    r0_arg: Option<u32>,
) -> Option<u32> {
    let frame_index = context::init_stack_frame(sub_stack, entry_point, r0_arg)?;
    let byte_offset = (frame_index as u32).checked_mul(4)?;
    sub_stack_base.checked_add(byte_offset)
}

/// Errors returned by [`ThreadTable`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadError {
    /// The thread table is full; no more threads can be added.
    TableFull,
    /// The given thread ID does not refer to a valid thread.
    InvalidThread,
    /// Stack base + size overflows the address space.
    StackOverflow,
}

/// Fixed-capacity table of thread control blocks.
///
/// `MAX` is a const-generic that determines the maximum number of threads.
/// The table uses a flat `[Option<ThreadControlBlock>; MAX]` array so that
/// `ThreadId::as_raw()` is a direct index — no searching required.
#[derive(Debug, Clone, PartialEq, Eq)]
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

    /// Number of threads in the `Ready` or `Running` state (schedulable).
    pub fn runnable_count(&self) -> usize {
        self.threads
            .iter()
            .flatten()
            .filter(|tcb| matches!(tcb.state, ThreadState::Ready | ThreadState::Running))
            .count()
    }

    /// Maximum number of threads this table can hold.
    pub const fn capacity(&self) -> usize {
        MAX
    }

    /// The scheduling policy configured for this table.
    pub fn scheduling_policy(&self) -> SchedulingPolicy {
        self.scheduling_policy
    }

    /// Update the scheduling policy without disturbing existing threads.
    pub fn set_scheduling_policy(&mut self, policy: SchedulingPolicy) {
        self.scheduling_policy = policy;
    }

    /// Pick the highest-priority Ready thread (static priority scheduling).
    ///
    /// Lowest priority number = highest priority.  On tie, the thread with
    /// the lower `ThreadId` wins (deterministic tie-breaking).
    /// Returns `None` when no Ready thread exists.
    pub fn pick_next_priority(&self) -> Option<ThreadId> {
        let mut best: Option<&ThreadControlBlock> = None;
        for tcb in self.threads.iter().flatten() {
            if tcb.state != ThreadState::Ready {
                continue;
            }
            best = Some(match best {
                None => tcb,
                Some(prev) => {
                    if tcb.priority < prev.priority
                        || (tcb.priority == prev.priority && tcb.id.as_raw() < prev.id.as_raw())
                    {
                        tcb
                    } else {
                        prev
                    }
                }
            });
        }
        best.map(|tcb| tcb.id)
    }

    /// Advance the static-priority schedule by one step.
    ///
    /// 1. Transition the current Running thread to Ready.
    /// 2. Pick the highest-priority Ready thread via [`pick_next_priority`].
    /// 3. Transition that thread to Running and update `current_thread`.
    ///
    /// Returns the `ThreadId` of the newly running thread, or `None` if no
    /// Ready thread was found (in which case the current thread is restored
    /// to Running).
    pub fn advance_priority(&mut self) -> Option<ThreadId> {
        // Demote current thread from Running to Ready (if it is Running).
        let cur = self.current_thread as usize;
        if let Some(ref mut tcb) = self.threads.get_mut(cur).and_then(|s| s.as_mut()) {
            if tcb.state == ThreadState::Running {
                tcb.state = ThreadState::Ready;
            }
        }

        if let Some(next_id) = self.pick_next_priority() {
            let idx = next_id.as_raw() as usize;
            if let Some(Some(ref mut tcb)) = self.threads.get_mut(idx) {
                tcb.state = ThreadState::Running;
                self.current_thread = next_id.as_raw();
                return Some(next_id);
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

    /// Advance the intra-partition schedule by one step, dispatching to the
    /// correct policy implementation based on `self.scheduling_policy`.
    ///
    /// Returns the `ThreadId` of the newly running thread, or `None` if no
    /// Ready thread was found.
    pub fn advance_intra_schedule(&mut self) -> Option<ThreadId> {
        match self.scheduling_policy {
            SchedulingPolicy::RoundRobin => self.advance_round_robin(),
            SchedulingPolicy::StaticPriority => self.advance_priority(),
        }
    }

    /// Create thread 0 as the partition's main thread.
    ///
    /// This is the default thread that exists for backward compatibility —
    /// partitions with no explicit thread creation still work.  The main
    /// thread is placed in slot 0 with priority 0 and `Running` state.
    /// `stack_pointer` is set to `stack_base + stack_size` (a sentinel value
    /// that is patched at boot).
    ///
    /// # Errors
    ///
    /// Returns `ThreadError::TableFull` if `MAX == 0` (no room for a thread).
    /// Returns `ThreadError::StackOverflow` if `stack_base + stack_size` overflows.
    pub fn init_main_thread(
        &mut self,
        entry_point: u32,
        stack_base: u32,
        stack_size: u32,
        r0: u32,
    ) -> Result<(), ThreadError> {
        let slot = self.threads.get_mut(0).ok_or(ThreadError::TableFull)?;
        let stack_pointer = stack_base
            .checked_add(stack_size)
            .ok_or(ThreadError::StackOverflow)?;
        let tcb = ThreadControlBlock {
            stack_pointer,
            id: ThreadId::new(0),
            state: ThreadState::Running,
            priority: 0,
            stack_base,
            stack_size,
            entry_point,
            r0_arg: r0,
        };
        *slot = Some(tcb);
        self.current_thread = 0;
        Ok(())
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

    // ── Static-priority scheduler tests ────────────────────────────

    /// Helper: build a table with threads at given priorities, thread 0 Running.
    fn make_priority_table(priorities: &[u8]) -> ThreadTable<8> {
        let mut table = ThreadTable::<8>::new(SchedulingPolicy::StaticPriority);
        for (i, &prio) in priorities.iter().enumerate() {
            table.add_thread(make_tcb(i as u8, prio)).unwrap();
        }
        if !priorities.is_empty() {
            table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Running;
            table.current_thread = 0;
        }
        table
    }

    #[test]
    fn pick_priority_high_over_low() {
        // Thread 0: Running (prio 5), Thread 1: Ready (prio 1), Thread 2: Ready (prio 3)
        let table = make_priority_table(&[5, 1, 3]);
        // Lowest number = highest priority → thread 1 (prio 1).
        assert_eq!(table.pick_next_priority(), Some(ThreadId::new(1)));
    }

    #[test]
    fn pick_priority_tiebreak_lower_id() {
        // All Ready threads at same priority — lower ID wins.
        let mut table = make_priority_table(&[2, 2, 2]);
        // Thread 0 is Running, threads 1 and 2 are Ready with equal prio.
        assert_eq!(table.pick_next_priority(), Some(ThreadId::new(1)));

        // Make thread 0 Ready too (simulate no current running).
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Ready;
        assert_eq!(table.pick_next_priority(), Some(ThreadId::new(0)));
    }

    #[test]
    fn pick_priority_suspended_high_falls_through() {
        // Thread 0: Running (prio 5), Thread 1: Suspended (prio 0), Thread 2: Ready (prio 3)
        let mut table = make_priority_table(&[5, 0, 3]);
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Suspended;
        // Thread 1 has highest priority but is Suspended → thread 2 selected.
        assert_eq!(table.pick_next_priority(), Some(ThreadId::new(2)));
    }

    #[test]
    fn pick_priority_empty_returns_none() {
        let table = ThreadTable::<4>::new(SchedulingPolicy::StaticPriority);
        assert_eq!(table.pick_next_priority(), None);
    }

    #[test]
    fn pick_priority_all_stopped_returns_none() {
        let mut table = make_priority_table(&[1, 2, 3]);
        for i in 0..3u8 {
            table.get_mut(ThreadId::new(i)).unwrap().state = ThreadState::Stopped;
        }
        assert_eq!(table.pick_next_priority(), None);
    }

    #[test]
    fn advance_priority_selects_highest() {
        // Thread 0: Running (prio 5), Thread 1: Ready (prio 1), Thread 2: Ready (prio 3)
        let mut table = make_priority_table(&[5, 1, 3]);
        let next = table.advance_priority().unwrap();
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
    }

    #[test]
    fn advance_priority_preemption() {
        // High-priority thread always preempts: repeated advances keep it running.
        let mut table = make_priority_table(&[5, 1, 3]);
        // First advance: thread 1 (prio 1) takes over.
        table.advance_priority();
        assert_eq!(table.current_thread, 1);

        // Second advance: thread 1 demoted to Ready, but it's still highest prio.
        let next = table.advance_priority().unwrap();
        assert_eq!(next, ThreadId::new(1));
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Running
        );
    }

    #[test]
    fn advance_priority_tiebreak() {
        // Threads 1 and 2 both prio 0 (highest), thread 0 Running prio 5.
        let mut table = make_priority_table(&[5, 0, 0]);
        let next = table.advance_priority().unwrap();
        // Lower ID wins the tie → thread 1.
        assert_eq!(next, ThreadId::new(1));
    }

    #[test]
    fn advance_priority_all_stopped_returns_none() {
        let mut table = make_priority_table(&[1, 2]);
        table.get_mut(ThreadId::new(0)).unwrap().state = ThreadState::Stopped;
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Stopped;
        assert_eq!(table.advance_priority(), None);
    }

    #[test]
    fn advance_priority_empty_returns_none() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::StaticPriority);
        assert_eq!(table.advance_priority(), None);
    }

    #[test]
    fn advance_priority_suspended_high_falls_to_lower() {
        // Thread 0: Running prio 10, Thread 1: Suspended prio 0, Thread 2: Ready prio 5
        let mut table = make_priority_table(&[10, 0, 5]);
        table.get_mut(ThreadId::new(1)).unwrap().state = ThreadState::Suspended;
        let next = table.advance_priority().unwrap();
        assert_eq!(next, ThreadId::new(2));
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().state,
            ThreadState::Ready
        );
        assert_eq!(
            table.get(ThreadId::new(1)).unwrap().state,
            ThreadState::Suspended
        );
        assert_eq!(
            table.get(ThreadId::new(2)).unwrap().state,
            ThreadState::Running
        );
    }

    // ── advance_intra_schedule dispatch tests ─────────────────────

    #[test]
    fn advance_intra_schedule_rr_uses_round_robin() {
        let mut table = make_rr_table::<3>();
        // T0=Running, T1=Ready, T2=Ready — RR should pick T1 next.
        let next = table.advance_intra_schedule().unwrap();
        assert_eq!(next, ThreadId::new(1));
        assert_eq!(table.current_thread, 1);

        // Next advance picks T2 (round-robin order, not priority).
        let next = table.advance_intra_schedule().unwrap();
        assert_eq!(next, ThreadId::new(2));
        assert_eq!(table.current_thread, 2);
    }

    #[test]
    fn advance_intra_schedule_priority_uses_priority_logic() {
        // Thread 0: Running prio 5, Thread 1: Ready prio 1, Thread 2: Ready prio 3
        let mut table = make_priority_table(&[5, 1, 3]);
        let next = table.advance_intra_schedule().unwrap();
        // Highest priority (lowest number) = thread 1 (prio 1).
        assert_eq!(next, ThreadId::new(1));
        assert_eq!(table.current_thread, 1);

        // Thread 1 keeps winning because it has the highest priority.
        let next = table.advance_intra_schedule().unwrap();
        assert_eq!(next, ThreadId::new(1));
    }

    #[test]
    fn advance_intra_schedule_empty_returns_none() {
        let mut rr = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(rr.advance_intra_schedule(), None);

        let mut sp = ThreadTable::<4>::new(SchedulingPolicy::StaticPriority);
        assert_eq!(sp.advance_intra_schedule(), None);
    }

    // ── init_main_thread tests ────────────────────────────────────

    #[test]
    fn init_main_thread_creates_running_thread_at_index_0() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        table
            .init_main_thread(0x0800_0000, 0x2000_0000, 1024, 42)
            .unwrap();

        assert_eq!(table.thread_count(), 1);
        let tcb = table.get(ThreadId::new(0)).unwrap();
        assert_eq!(tcb.id, ThreadId::new(0));
        assert_eq!(tcb.state, ThreadState::Running);
        assert_eq!(tcb.priority, 0);
        assert_eq!(tcb.entry_point, 0x0800_0000);
        assert_eq!(tcb.stack_base, 0x2000_0000);
        assert_eq!(tcb.stack_size, 1024);
        assert_eq!(tcb.stack_pointer, 0x2000_0000 + 1024);
        assert_eq!(tcb.r0_arg, 42);
        assert_eq!(table.current_thread, 0);
    }

    #[test]
    fn init_main_thread_only_slot_0_occupied() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::StaticPriority);
        table
            .init_main_thread(0x0800_0000, 0x2000_0000, 512, 0)
            .unwrap();

        // Exactly one thread at index 0; slots 1-3 are empty.
        assert_eq!(table.thread_count(), 1);
        assert!(table.get(ThreadId::new(0)).is_some());
        assert!(table.get(ThreadId::new(1)).is_none());
        assert!(table.get(ThreadId::new(2)).is_none());
        assert!(table.get(ThreadId::new(3)).is_none());
    }

    #[test]
    fn init_main_thread_stack_pointer_sentinel() {
        let mut table = ThreadTable::<2>::new(SchedulingPolicy::RoundRobin);
        let base = 0x2000_4000u32;
        let size = 2048u32;
        table.init_main_thread(0x0800_0000, base, size, 0).unwrap();
        assert_eq!(
            table.get(ThreadId::new(0)).unwrap().stack_pointer,
            base + size
        );
    }

    #[test]
    fn init_main_thread_returns_err_on_zero_capacity() {
        let mut table = ThreadTable::<0>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(
            table.init_main_thread(0x0800_0000, 0x2000_0000, 1024, 0),
            Err(ThreadError::TableFull)
        );
    }

    #[test]
    fn init_main_thread_returns_err_on_stack_overflow() {
        let mut table = ThreadTable::<4>::new(SchedulingPolicy::RoundRobin);
        assert_eq!(
            table.init_main_thread(0x0800_0000, u32::MAX, 1, 0),
            Err(ThreadError::StackOverflow)
        );
    }

    // ── split_thread_stack tests ──────────────────────────────────

    #[test]
    fn split_1_thread_gets_full_stack() {
        let base = 0x2000_0000u32;
        let size = 1024u32;
        let (sub_base, sub_size) = super::split_thread_stack(base, size, 0, 1).unwrap();
        assert_eq!(sub_size, 1024);
        assert_eq!(sub_base, base);
    }

    #[test]
    fn split_2_threads_even() {
        let base = 0x2000_0000u32;
        let size = 1024u32;
        let (b0, s0) = super::split_thread_stack(base, size, 0, 2).unwrap();
        let (b1, s1) = super::split_thread_stack(base, size, 1, 2).unwrap();
        // Each thread gets 512 bytes.
        assert_eq!(s0, 512);
        assert_eq!(s1, 512);
        // Thread 0 (main) is at the top.
        assert_eq!(b0, base + 512);
        assert_eq!(b1, base);
        // No overlap: b1 + s1 == b0.
        assert_eq!(b1 + s1, b0);
    }

    #[test]
    fn split_4_threads_quarters() {
        let base = 0x2000_0000u32;
        let size = 2048u32;
        for i in 0..4u32 {
            let (bi, si) = super::split_thread_stack(base, size, i, 4).unwrap();
            assert_eq!(si, 512);
            assert_eq!(bi, base + 2048 - (i + 1) * 512);
            assert_eq!(bi % 8, 0, "sub-stack base not 8-byte aligned");
        }
        // Total sub-stack space == partition_stack_size.
        assert_eq!(512 * 4, size);
    }

    #[test]
    fn split_alignment_maintained_for_unaligned_size() {
        let base = 0x2000_0000u32;
        // Use truly unaligned sizes (not multiples of 8).
        for &size in &[1001u32, 1007, 997] {
            for i in 0..3u32 {
                let (bi, si) = super::split_thread_stack(base, size, i, 3).unwrap();
                assert_eq!(
                    si % 8,
                    0,
                    "sub-stack size not 8-byte aligned for size={size}"
                );
                assert_eq!(
                    bi % 8,
                    0,
                    "sub-stack base not 8-byte aligned for size={size}"
                );
            }
            // Sub-stacks must not overlap.
            let (b0, s0) = super::split_thread_stack(base, size, 0, 3).unwrap();
            let (b1, s1) = super::split_thread_stack(base, size, 1, 3).unwrap();
            let (b2, _s2) = super::split_thread_stack(base, size, 2, 3).unwrap();
            assert!(
                b2 + s0 <= b1,
                "thread 2 and thread 1 overlap for size={size}"
            );
            assert!(
                b1 + s1 <= b0,
                "thread 1 and thread 0 overlap for size={size}"
            );
            // Total sub-stack space <= partition_stack_size.
            assert!(s0 * 3 <= size);
        }
    }

    #[test]
    fn split_alignment_with_unaligned_base() {
        // Base that is not 8-byte aligned: sub-stack tops must still be aligned.
        let base = 0x2000_0003u32;
        let size = 1024u32;
        for i in 0..2u32 {
            let (bi, si) = super::split_thread_stack(base, size, i, 2).unwrap();
            assert_eq!(si % 8, 0, "sub-stack size not 8-byte aligned");
            assert_eq!(bi % 8, 0, "sub-stack base not 8-byte aligned");
            assert!(bi >= base, "sub-stack below partition base");
            assert!(bi + si <= base + size, "sub-stack exceeds partition top");
        }
    }

    #[test]
    fn split_no_overlap_any_count() {
        let base = 0x2000_0000u32;
        let size = 4096u32;
        for max in 1..=8u32 {
            let mut prev_base = base + size;
            for i in 0..max {
                let (bi, si) = super::split_thread_stack(base, size, i, max).unwrap();
                // Each sub-stack ends at or before the previous one starts.
                assert!(bi + si <= prev_base, "overlap at max={max}, i={i}");
                // All sub-stacks are within the partition stack.
                assert!(bi >= base, "sub-stack below partition base");
                assert!(bi + si <= base + size, "sub-stack exceeds partition top");
                prev_base = bi;
            }
        }
    }

    #[test]
    fn split_returns_none_on_zero_threads() {
        assert!(super::split_thread_stack(0x2000_0000, 1024, 0, 0).is_none());
    }

    #[test]
    fn split_returns_none_on_oob_index() {
        assert!(super::split_thread_stack(0x2000_0000, 1024, 2, 2).is_none());
    }

    #[test]
    fn split_returns_none_on_overflow() {
        // base + size would exceed u32::MAX.
        assert!(super::split_thread_stack(u32::MAX - 10, 100, 0, 1).is_none());
    }

    // ── init_thread_stack_frame tests ─────────────────────────────

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_thread_stack_frame_returns_absolute_sp() {
        let mut stack = [0u32; 32];
        let base = 0x2000_0000u32;
        let sp = super::init_thread_stack_frame(&mut stack, base, 0x0800_0101u32, None).unwrap();
        // init_stack_frame returns word index 16 (32 - 16 frame words).
        // Absolute SP = base + 16 * 4 = base + 64.
        assert_eq!(sp, base + 16 * 4);
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_thread_stack_frame_too_small_returns_none() {
        let mut stack = [0u32; 4];
        let result = super::init_thread_stack_frame(&mut stack, 0x2000_0000, 0x0800_0000u32, None);
        assert!(result.is_none());
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_thread_stack_frame_writes_entry_and_r0() {
        use crate::context::{SAVED_CONTEXT_WORDS, XPSR_THUMB};
        let mut stack = [0xDEADu32; 32];
        let base = 0x2000_1000u32;
        let entry = 0x0800_0201u32;
        let sp = super::init_thread_stack_frame(&mut stack, base, entry, Some(99)).unwrap();
        let idx = ((sp - base) / 4) as usize;
        // r0 = 99
        assert_eq!(stack[idx + SAVED_CONTEXT_WORDS], 99);
        // pc = entry
        assert_eq!(stack[idx + SAVED_CONTEXT_WORDS + 6], entry);
        // xpsr has Thumb bit
        assert_eq!(stack[idx + SAVED_CONTEXT_WORDS + 7], XPSR_THUMB);
    }
}
