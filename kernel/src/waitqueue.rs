//! A fixed-capacity FIFO queue of partition IDs used by synchronization
//! primitives (semaphores, mutexes, message queues) that do not need
//! timeout support. `W` is the compile-time maximum number of waiters.

/// Error returned when a [`WaitQueue`] is full and cannot accept another entry.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct WaitQueueFull;

/// Fixed-capacity FIFO wait queue storing partition IDs (`u8`).
///
/// Wraps [`heapless::Deque`] and exposes only the subset of operations
/// needed by kernel synchronization primitives.
#[derive(Debug)]
pub struct WaitQueue<const W: usize> {
    inner: heapless::Deque<u8, W>,
}

impl<const W: usize> Default for WaitQueue<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: usize> WaitQueue<W> {
    /// Create an empty wait queue.
    pub const fn new() -> Self {
        Self {
            inner: heapless::Deque::new(),
        }
    }

    /// Enqueue a partition ID at the back of the queue.
    ///
    /// Returns `Err(WaitQueueFull)` if the queue has reached capacity.
    pub fn push(&mut self, pid: u8) -> Result<(), WaitQueueFull> {
        self.inner.push_back(pid).map_err(|_| WaitQueueFull)
    }

    /// Dequeue the partition ID at the front (oldest entry).
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front(&mut self) -> Option<u8> {
        self.inner.pop_front()
    }

    /// Returns `true` if the queue contains no entries.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Returns the number of entries currently in the queue.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Returns `true` if the queue has reached its compile-time capacity.
    pub fn is_full(&self) -> bool {
        self.inner.is_full()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_queue_is_empty() {
        let q = WaitQueue::<4>::new();
        assert!(q.is_empty());
        assert!(!q.is_full());
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn push_and_pop_single() {
        let mut q = WaitQueue::<4>::new();
        q.push(7).unwrap();
        assert_eq!(q.len(), 1);
        assert!(!q.is_empty());
        assert_eq!(q.pop_front(), Some(7));
        assert!(q.is_empty());
    }

    #[test]
    fn fifo_ordering() {
        let mut q = WaitQueue::<8>::new();
        for pid in 0..5u8 {
            q.push(pid).unwrap();
        }
        for pid in 0..5u8 {
            assert_eq!(q.pop_front(), Some(pid));
        }
        assert!(q.is_empty());
    }

    #[test]
    fn capacity_limit_returns_error() {
        let mut q = WaitQueue::<3>::new();
        q.push(0).unwrap();
        q.push(1).unwrap();
        q.push(2).unwrap();
        assert!(q.is_full());
        assert_eq!(q.push(3), Err(WaitQueueFull));
        // Queue contents unchanged after failed push
        assert_eq!(q.len(), 3);
        assert_eq!(q.pop_front(), Some(0));
    }

    #[test]
    fn pop_from_empty_returns_none() {
        let mut q = WaitQueue::<4>::new();
        assert_eq!(q.pop_front(), None);
    }

    #[test]
    fn fill_drain_refill() {
        let mut q = WaitQueue::<2>::new();
        q.push(10).unwrap();
        q.push(20).unwrap();
        assert!(q.is_full());

        assert_eq!(q.pop_front(), Some(10));
        assert_eq!(q.pop_front(), Some(20));
        assert!(q.is_empty());

        // Refill after drain
        q.push(30).unwrap();
        q.push(40).unwrap();
        assert!(q.is_full());
        assert_eq!(q.pop_front(), Some(30));
        assert_eq!(q.pop_front(), Some(40));
    }

    #[test]
    fn len_tracks_pushes_and_pops() {
        let mut q = WaitQueue::<4>::new();
        assert_eq!(q.len(), 0);
        q.push(1).unwrap();
        assert_eq!(q.len(), 1);
        q.push(2).unwrap();
        assert_eq!(q.len(), 2);
        q.pop_front();
        assert_eq!(q.len(), 1);
        q.pop_front();
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn capacity_one_queue() {
        let mut q = WaitQueue::<1>::new();
        q.push(42).unwrap();
        assert!(q.is_full());
        assert_eq!(q.push(43), Err(WaitQueueFull));
        assert_eq!(q.pop_front(), Some(42));
        assert!(q.is_empty());
    }

    #[test]
    fn duplicate_pids_allowed() {
        let mut q = WaitQueue::<4>::new();
        q.push(5).unwrap();
        q.push(5).unwrap();
        q.push(5).unwrap();
        assert_eq!(q.len(), 3);
        assert_eq!(q.pop_front(), Some(5));
        assert_eq!(q.pop_front(), Some(5));
        assert_eq!(q.pop_front(), Some(5));
    }
}
