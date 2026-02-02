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

/// Fixed-capacity FIFO wait queue storing `(partition_id, expiry_tick)` pairs.
///
/// Used by synchronization primitives that support blocking with timeouts
/// (queuing ports, blackboards). `W` is the compile-time maximum number of
/// waiters.
#[derive(Debug)]
pub struct TimedWaitQueue<const W: usize> {
    inner: heapless::Deque<(u8, u64), W>,
}

impl<const W: usize> Default for TimedWaitQueue<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: usize> TimedWaitQueue<W> {
    /// Create an empty timed wait queue.
    pub const fn new() -> Self {
        Self {
            inner: heapless::Deque::new(),
        }
    }

    /// Enqueue a partition ID with an expiry tick at the back of the queue.
    ///
    /// Returns `Err(WaitQueueFull)` if the queue has reached capacity.
    pub fn push(&mut self, pid: u8, expiry: u64) -> Result<(), WaitQueueFull> {
        self.inner
            .push_back((pid, expiry))
            .map_err(|_| WaitQueueFull)
    }

    /// Dequeue the oldest `(pid, expiry)` pair from the front.
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front(&mut self) -> Option<(u8, u64)> {
        self.inner.pop_front()
    }

    /// Dequeue the oldest entry, returning only the partition ID and
    /// discarding the expiry tick.
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front_pid(&mut self) -> Option<u8> {
        self.inner.pop_front().map(|(pid, _)| pid)
    }

    /// Remove all entries whose expiry tick has been reached, appending
    /// their partition IDs to `out`. Non-expired entries are preserved
    /// in their original FIFO order.
    ///
    /// # Panics
    ///
    /// Debug-asserts if `out` cannot hold all expired entries.
    pub fn drain_expired<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        // TODO: pop-and-requeue is a brute-force workaround for heapless::Deque
        // lacking retain/partition. Revisit if heapless gains such an API.
        let len = self.inner.len();
        for _ in 0..len {
            if let Some((pid, expiry)) = self.inner.pop_front() {
                if current_tick >= expiry {
                    debug_assert!(out.push(pid).is_ok(), "drain_expired: output buffer full");
                } else {
                    // Re-enqueue non-expired entry to preserve FIFO order.
                    let _ = self.inner.push_back((pid, expiry));
                }
            }
        }
    }

    /// Remove all entries, returning their partition IDs in FIFO order.
    ///
    /// Used for wake-all semantics (e.g., blackboard display waking all
    /// blocked readers).
    pub fn drain_all(&mut self) -> heapless::Vec<u8, W> {
        core::mem::take(&mut self.inner)
            .into_iter()
            .map(|(pid, _)| pid)
            .collect()
    }

    /// Returns `true` if the queue contains no entries.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Returns the number of entries currently in the queue.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Non-destructive snapshot of the partition IDs currently in the queue,
    /// in FIFO order. Intended for test assertions that need to observe queue
    /// contents without mutating the queue.
    #[cfg(test)]
    pub fn waiting_pids(&self) -> std::vec::Vec<u8> {
        self.inner.iter().map(|&(pid, _)| pid).collect()
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

    // ---- TimedWaitQueue tests ----

    #[test]
    fn timed_new_and_default_are_empty() {
        let q = TimedWaitQueue::<4>::new();
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
        let q2 = TimedWaitQueue::<4>::default();
        assert!(q2.is_empty());
    }

    #[test]
    fn timed_push_pop_and_pop_pid() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(7, 100).unwrap();
        assert_eq!(q.len(), 1);
        assert!(!q.is_empty());
        assert_eq!(q.pop_front(), Some((7, 100)));
        assert!(q.is_empty());
        // pop_front_pid discards the expiry
        q.push(3, 999).unwrap();
        assert_eq!(q.pop_front_pid(), Some(3));
        // pop on empty
        assert_eq!(q.pop_front(), None);
        assert_eq!(q.pop_front_pid(), None);
    }

    #[test]
    fn timed_fifo_ordering() {
        let mut q = TimedWaitQueue::<8>::new();
        for pid in 0..5u8 {
            q.push(pid, pid as u64 * 10).unwrap();
        }
        for pid in 0..5u8 {
            assert_eq!(q.pop_front(), Some((pid, pid as u64 * 10)));
        }
        assert!(q.is_empty());
    }

    #[test]
    fn timed_capacity_and_len() {
        let mut q = TimedWaitQueue::<2>::new();
        assert_eq!(q.len(), 0);
        q.push(0, 10).unwrap();
        assert_eq!(q.len(), 1);
        q.push(1, 20).unwrap();
        assert_eq!(q.len(), 2);
        assert_eq!(q.push(2, 30), Err(WaitQueueFull));
        assert_eq!(q.len(), 2);
        q.pop_front();
        assert_eq!(q.len(), 1);
    }

    #[test]
    fn timed_drain_expired_mixed() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(3, 50).unwrap();

        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut expired);
        assert_eq!(expired.as_slice(), &[1, 3]);
        assert_eq!(q.len(), 1);
        assert_eq!(q.pop_front(), Some((2, 200)));
    }

    #[test]
    fn timed_drain_expired_none_and_all() {
        // None expired
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert!(expired.is_empty());
        assert_eq!(q.len(), 2);

        // All expired
        expired.clear();
        q.push(3, 30).unwrap();
        q.drain_expired(200, &mut expired);
        assert_eq!(expired.as_slice(), &[1, 2, 3]);
        assert!(q.is_empty());

        // Empty queue
        expired.clear();
        q.drain_expired(100, &mut expired);
        assert!(expired.is_empty());
    }

    #[test]
    fn timed_drain_expired_preserves_fifo_order() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(10, 200).unwrap(); // survives
        q.push(20, 50).unwrap(); // expires
        q.push(30, 300).unwrap(); // survives
        q.push(40, 50).unwrap(); // expires

        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert_eq!(expired.as_slice(), &[20, 40]);
        assert_eq!(q.pop_front_pid(), Some(10));
        assert_eq!(q.pop_front_pid(), Some(30));
    }

    #[test]
    fn timed_drain_expired_tick_boundary() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        // Exact boundary: current_tick == expiry => expired
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut expired);
        assert_eq!(expired.as_slice(), &[1]);
        // One tick before: not expired
        q.push(2, 100).unwrap();
        expired.clear();
        q.drain_expired(99, &mut expired);
        assert!(expired.is_empty());
        assert_eq!(q.len(), 1);
    }

    #[test]
    fn timed_drain_all_and_wake_all() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(5, 10).unwrap();
        q.push(3, 20).unwrap();
        q.push(7, 30).unwrap();
        let pids = q.drain_all();
        assert_eq!(pids.as_slice(), &[5, 3, 7]);
        assert!(q.is_empty());
        // drain_all on empty
        let pids = q.drain_all();
        assert!(pids.is_empty());
    }

    #[test]
    fn timed_fill_drain_refill() {
        let mut q = TimedWaitQueue::<2>::new();
        q.push(10, 50).unwrap();
        q.push(20, 60).unwrap();
        assert_eq!(q.pop_front(), Some((10, 50)));
        assert_eq!(q.pop_front(), Some((20, 60)));
        q.push(30, 70).unwrap();
        q.push(40, 80).unwrap();
        assert_eq!(q.pop_front(), Some((30, 70)));
        assert_eq!(q.pop_front(), Some((40, 80)));
    }
}
