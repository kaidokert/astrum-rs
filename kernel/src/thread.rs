//! Thread table for managing fixed-capacity thread pools.
//!
//! `ThreadTable<MAX>` stores up to `MAX` threads in a fixed-size array,
//! supporting O(1) lookup by `ThreadId` and O(N) insertion (linear scan for
//! the first free slot).

use rtos_traits::ids::ThreadId;
use rtos_traits::thread::{SchedulingPolicy, ThreadControlBlock};

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
}
