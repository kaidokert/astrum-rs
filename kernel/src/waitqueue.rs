//! A fixed-capacity FIFO queue of partition IDs used by synchronization
//! primitives (semaphores, mutexes, message queues) that do not need
//! timeout support. `W` is the compile-time maximum number of waiters.

use rtos_traits::ids::PartitionId;

/// Error returned when a [`WaitQueue`] is full and cannot accept another entry.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct WaitQueueFull;

/// Fixed-capacity FIFO wait queue storing [`PartitionId`] values.
///
/// Wraps [`heapless::Deque`] and exposes only the subset of operations
/// needed by kernel synchronization primitives.
#[derive(Debug)]
pub struct WaitQueue<const W: usize> {
    inner: heapless::Deque<PartitionId, W>,
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
    pub fn push(&mut self, pid: PartitionId) -> Result<(), WaitQueueFull> {
        self.inner.push_back(pid).map_err(|_| WaitQueueFull)
    }

    /// Dequeue the partition ID at the front (oldest entry).
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front(&mut self) -> Option<PartitionId> {
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

    /// Remove the first entry matching `pid` and return `true`.
    ///
    /// Returns `false` if no entry with that partition ID exists.
    /// Remaining entries preserve their original FIFO order.
    pub fn remove_by_id(&mut self, pid: PartitionId) -> bool {
        let len = self.inner.len();
        let mut found = false;
        for _ in 0..len {
            if let Some(entry) = self.inner.pop_front() {
                if entry == pid && !found {
                    found = true; // skip first match
                } else {
                    // Cannot fail: we popped one element before pushing.
                    let _ = self.inner.push_back(entry);
                }
            }
        }
        found
    }
}

/// Compute a safe absolute expiry tick from the current tick and a timeout.
///
/// Enforces a minimum 2-tick gap (`current_tick + max(timeout_ticks, 2)`) to
/// prevent same-tick or next-tick expiry races: if the SysTick handler fires
/// between the syscall and the partition's transition to `Waiting`, a 0- or
/// 1-tick timeout could already appear expired.
///
/// Uses saturating arithmetic to handle `u64` overflow gracefully.
pub const fn safe_expiry(current_tick: u64, timeout_ticks: u64) -> u64 {
    let effective = if timeout_ticks < 2 { 2 } else { timeout_ticks };
    current_tick.saturating_add(effective)
}

/// Fixed-capacity FIFO wait queue storing `(partition_id, expiry_tick)` pairs.
///
/// Used by synchronization primitives that support blocking with timeouts
/// (queuing ports, blackboards). `W` is the compile-time maximum number of
/// waiters.
#[derive(Debug)]
pub struct TimedWaitQueue<const W: usize> {
    inner: heapless::Deque<(PartitionId, u64), W>,
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
    pub fn push(&mut self, pid: PartitionId, expiry: u64) -> Result<(), WaitQueueFull> {
        self.inner
            .push_back((pid, expiry))
            .map_err(|_| WaitQueueFull)
    }

    /// Dequeue the oldest `(pid, expiry)` pair from the front.
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front(&mut self) -> Option<(PartitionId, u64)> {
        self.inner.pop_front()
    }

    /// Dequeue the oldest entry, returning only the partition ID and
    /// discarding the expiry tick.
    ///
    /// Returns `None` if the queue is empty.
    pub fn pop_front_pid(&mut self) -> Option<PartitionId> {
        self.inner.pop_front().map(|(pid, _)| pid)
    }

    /// Remove all entries whose expiry tick has been reached, appending
    /// their partition IDs to `out`. Non-expired entries are preserved
    /// in their original FIFO order.
    ///
    /// Performs a single in-place pass over the deque with O(1) extra
    /// space: each element is popped from the front and either collected
    /// into `out` (expired) or pushed back (non-expired). Because
    /// `heapless::Deque` lacks indexed access and interior removal,
    /// pop-front/push-back is the only way to compact in-place without
    /// allocating a temporary buffer.
    ///
    /// If `out` is full, expired entries are re-enqueued and will be
    /// retried on the next tick (graceful degradation instead of panic
    /// or silent loss).
    pub fn drain_expired<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<PartitionId, E>,
    ) {
        let len = self.inner.len();
        for _ in 0..len {
            if let Some((pid, expiry)) = self.inner.pop_front() {
                if current_tick >= expiry {
                    if out.push(pid).is_err() {
                        // Output buffer full; re-enqueue so this entry is
                        // retried next tick rather than silently lost.
                        let _ = self.inner.push_back((pid, expiry));
                    }
                } else {
                    let _ = self.inner.push_back((pid, expiry));
                }
            }
        }
    }

    /// Remove all entries, returning their partition IDs in FIFO order.
    ///
    /// Used for wake-all semantics (e.g., blackboard display waking all
    /// blocked readers).
    pub fn drain_all(&mut self) -> heapless::Vec<PartitionId, W> {
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

    /// Returns `true` if the queue has reached its compile-time capacity.
    pub fn is_full(&self) -> bool {
        self.inner.is_full()
    }

    /// Remove the first entry matching `pid` and return `true`.
    ///
    /// Returns `false` if no entry with that partition ID exists.
    /// Remaining entries preserve their original FIFO order.
    pub fn remove_by_id(&mut self, pid: PartitionId) -> bool {
        let len = self.inner.len();
        let mut found = false;
        for _ in 0..len {
            if let Some((p, expiry)) = self.inner.pop_front() {
                if p == pid && !found {
                    found = true; // skip first match
                } else {
                    // Cannot fail: we popped one element before pushing.
                    let _ = self.inner.push_back((p, expiry));
                }
            }
        }
        found
    }

    /// Non-destructive snapshot of the partition IDs currently in the queue,
    /// in FIFO order. Intended for test assertions that need to observe queue
    /// contents without mutating the queue.
    #[cfg(test)]
    pub fn waiting_pids(&self) -> std::vec::Vec<u8> {
        self.inner.iter().map(|&(pid, _)| pid).collect()
    }
}

/// Fixed-capacity FIFO wait queue for partitions blocked on device reads.
///
/// Wraps [`TimedWaitQueue`] with device-oriented method names. Each entry
/// records a partition ID and an optional expiry tick. `W` is the
/// compile-time maximum number of blocked readers.
#[derive(Debug)]
pub struct DeviceWaitQueue<const W: usize> {
    inner: TimedWaitQueue<W>,
}

impl<const W: usize> Default for DeviceWaitQueue<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: usize> DeviceWaitQueue<W> {
    /// Create an empty device wait queue.
    pub const fn new() -> Self {
        Self {
            inner: TimedWaitQueue::new(),
        }
    }

    /// Block a partition on a device read with an optional expiry tick.
    ///
    /// Returns `Err(WaitQueueFull)` if the queue has reached capacity.
    pub fn block_reader(&mut self, pid: u8, expiry: u64) -> Result<(), WaitQueueFull> {
        self.inner.push(pid, expiry)
    }

    /// Wake the oldest blocked reader, returning its partition ID.
    ///
    /// Returns `None` if no readers are blocked.
    pub fn wake_one_reader(&mut self) -> Option<u8> {
        self.inner.pop_front_pid()
    }

    /// Remove all entries whose expiry tick has been reached, appending
    /// their partition IDs to `out`.
    pub fn drain_expired<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        self.inner.drain_expired(current_tick, out);
    }

    /// Returns `true` if no readers are blocked.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Returns the number of blocked readers.
    pub fn len(&self) -> usize {
        self.inner.len()
    }
}

/// Sorted sleep timer queue for O(1) amortised wakeup.
///
/// Entries are kept sorted by expiry tick (ascending). [`drain_expired`]
/// pops from the front while expired, giving O(k) drain where k is
/// the number of expired entries. Insertion is O(N) due to maintaining
/// sorted order, but insertions happen per-syscall (rare), not per-tick.
///
/// [`drain_expired`]: SleepQueue::drain_expired
#[derive(Debug)]
pub struct SleepQueue<const W: usize> {
    inner: heapless::Vec<(u8, u64), W>,
}

impl<const W: usize> Default for SleepQueue<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: usize> SleepQueue<W> {
    /// Create an empty sleep queue.
    pub const fn new() -> Self {
        Self {
            inner: heapless::Vec::new(),
        }
    }

    /// Insert a sleep entry, maintaining sorted order by expiry tick.
    ///
    /// Returns `Err(WaitQueueFull)` if the queue has reached capacity.
    pub fn push(&mut self, pid: u8, expiry: u64) -> Result<(), WaitQueueFull> {
        if self.inner.is_full() {
            return Err(WaitQueueFull);
        }
        // Append and bubble into sorted position (insertion sort).
        let _ = self.inner.push((pid, expiry));
        let mut i = self.inner.len() - 1;
        while i > 0 && self.inner[i - 1].1 > expiry {
            self.inner.swap(i, i - 1);
            i -= 1;
        }
        Ok(())
    }

    /// Remove all entries whose expiry tick has been reached, appending
    /// their partition IDs to `out`.
    ///
    /// Because entries are sorted by expiry tick, this pops from the front
    /// until a non-expired entry is found, giving O(k) where k is the
    /// number of expired entries.
    ///
    /// If `out` is full, draining stops and remaining expired entries stay
    /// in the queue for retry on the next tick.
    pub fn drain_expired<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        let mut drained = 0usize;
        for &(pid, expiry) in self.inner.iter() {
            if current_tick >= expiry {
                if out.push(pid).is_err() {
                    break;
                }
                drained += 1;
            } else {
                break; // Sorted: no more expired entries past this point.
            }
        }
        if drained > 0 {
            let new_len = self.inner.len() - drained;
            for i in 0..new_len {
                self.inner[i] = self.inner[i + drained];
            }
            self.inner.truncate(new_len);
        }
    }

    /// Returns `true` if the queue contains no entries.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Returns the number of entries currently in the queue.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Remove the first entry matching `pid` and return `true`.
    ///
    /// Returns `false` if no entry with that partition ID exists.
    /// Remaining entries preserve their sorted order.
    pub fn remove_by_id(&mut self, pid: u8) -> bool {
        if let Some(idx) = self.inner.iter().position(|&(p, _)| p == pid) {
            self.inner.remove(idx);
            true
        } else {
            false
        }
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

    #[test]
    fn timed_drain_expired_empty_queue() {
        let mut q = TimedWaitQueue::<4>::new();
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(1000, &mut expired);
        assert!(expired.is_empty());
        assert!(q.is_empty());
    }

    #[test]
    fn timed_drain_expired_single_expired() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(42, 50).unwrap();
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert_eq!(expired.as_slice(), &[42]);
        assert!(q.is_empty());
    }

    #[test]
    fn timed_drain_expired_single_not_expired() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(42, 100).unwrap();
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert!(expired.is_empty());
        assert_eq!(q.len(), 1);
        assert_eq!(q.pop_front(), Some((42, 100)));
    }

    #[test]
    fn timed_drain_expired_interleaved_pattern() {
        // Alternating expired/not-expired entries
        let mut q = TimedWaitQueue::<8>::new();
        q.push(1, 10).unwrap(); // expired
        q.push(2, 200).unwrap(); // kept
        q.push(3, 20).unwrap(); // expired
        q.push(4, 300).unwrap(); // kept
        q.push(5, 30).unwrap(); // expired
        q.push(6, 400).unwrap(); // kept

        let mut expired: heapless::Vec<u8, 8> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert_eq!(expired.as_slice(), &[1, 3, 5]);
        // Survivors preserve FIFO order
        assert_eq!(q.waiting_pids(), vec![2, 4, 6]);
    }

    #[test]
    fn timed_drain_expired_all_at_full_capacity() {
        // Fill to capacity and expire everything
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 10).unwrap();
        q.push(2, 20).unwrap();
        q.push(3, 30).unwrap();
        q.push(4, 40).unwrap();

        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut expired);
        assert_eq!(expired.as_slice(), &[1, 2, 3, 4]);
        assert!(q.is_empty());
    }

    #[test]
    fn timed_drain_expired_none_at_full_capacity() {
        // Fill to capacity and expire nothing
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(3, 300).unwrap();
        q.push(4, 400).unwrap();

        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut expired);
        assert!(expired.is_empty());
        assert_eq!(q.len(), 4);
        assert_eq!(q.waiting_pids(), vec![1, 2, 3, 4]);
    }

    #[test]
    fn timed_drain_expired_successive_drains() {
        // Multiple successive drain_expired calls with advancing ticks
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(3, 300).unwrap();

        // First drain at tick 150: only pid 1 expires
        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(150, &mut expired);
        assert_eq!(expired.as_slice(), &[1]);
        assert_eq!(q.waiting_pids(), vec![2, 3]);

        // Second drain at tick 250: pid 2 expires
        expired.clear();
        q.drain_expired(250, &mut expired);
        assert_eq!(expired.as_slice(), &[2]);
        assert_eq!(q.waiting_pids(), vec![3]);

        // Third drain at tick 300: pid 3 expires
        expired.clear();
        q.drain_expired(300, &mut expired);
        assert_eq!(expired.as_slice(), &[3]);
        assert!(q.is_empty());
    }

    #[test]
    fn timed_drain_expired_wrapping_ring_buffer() {
        // Exercise the ring buffer wrap-around by pushing/popping
        // before filling, so the internal head pointer is non-zero.
        let mut q = TimedWaitQueue::<4>::new();
        // Push and pop two entries to advance the ring buffer head
        q.push(99, 1).unwrap();
        q.push(98, 2).unwrap();
        q.pop_front();
        q.pop_front();

        // Now head is at offset 2; push 4 entries that wrap around
        q.push(1, 50).unwrap(); // expired
        q.push(2, 200).unwrap(); // kept
        q.push(3, 60).unwrap(); // expired
        q.push(4, 300).unwrap(); // kept

        let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut expired);
        assert_eq!(expired.as_slice(), &[1, 3]);
        assert_eq!(q.waiting_pids(), vec![2, 4]);
    }

    // ---- DeviceWaitQueue tests ----
    // Only verify that the wrapper forwards to TimedWaitQueue correctly.
    // FIFO ordering, expiry logic, and capacity semantics are covered by
    // the TimedWaitQueue tests above.

    mod device_wait_queue {
        use super::super::*;

        #[test]
        fn forwards_block_and_wake() {
            let mut q = DeviceWaitQueue::<4>::new();
            assert!(q.is_empty());
            assert_eq!(q.len(), 0);
            q.block_reader(3, 100).unwrap();
            assert_eq!(q.len(), 1);
            assert!(!q.is_empty());
            assert_eq!(q.wake_one_reader(), Some(3));
            assert!(q.is_empty());
        }

        #[test]
        fn forwards_drain_expired() {
            let mut q = DeviceWaitQueue::<4>::new();
            q.block_reader(1, 50).unwrap();
            q.block_reader(2, 200).unwrap();
            let mut expired: heapless::Vec<u8, 4> = heapless::Vec::new();
            q.drain_expired(100, &mut expired);
            assert_eq!(expired.as_slice(), &[1]);
            assert_eq!(q.len(), 1);
        }

        #[test]
        fn forwards_capacity_error() {
            let mut q = DeviceWaitQueue::<2>::new();
            q.block_reader(0, 100).unwrap();
            q.block_reader(1, 200).unwrap();
            assert_eq!(q.block_reader(2, 300), Err(WaitQueueFull));
        }
    }

    // ---- SleepQueue tests ----

    #[test]
    fn sleep_queue_new_is_empty() {
        let q = SleepQueue::<4>::new();
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn sleep_queue_push_maintains_sorted_order() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 300).unwrap();
        q.push(2, 100).unwrap();
        q.push(3, 200).unwrap();
        // Drain all: should come out in expiry order (100, 200, 300).
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(300, &mut out);
        assert_eq!(out.as_slice(), &[2, 3, 1]);
        assert!(q.is_empty());
    }

    #[test]
    fn sleep_queue_drain_expired_stops_at_non_expired() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 50).unwrap();
        q.push(2, 100).unwrap();
        q.push(3, 200).unwrap();
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut out);
        assert_eq!(out.as_slice(), &[1, 2]);
        assert_eq!(q.len(), 1);
    }

    #[test]
    fn sleep_queue_drain_expired_none() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 100).unwrap();
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(50, &mut out);
        assert!(out.is_empty());
        assert_eq!(q.len(), 1);
    }

    #[test]
    fn sleep_queue_drain_expired_empty_queue() {
        let mut q = SleepQueue::<4>::new();
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(1000, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn sleep_queue_capacity_error() {
        let mut q = SleepQueue::<2>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        assert_eq!(q.push(3, 300), Err(WaitQueueFull));
    }

    #[test]
    fn sleep_queue_drain_stops_when_out_full() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 10).unwrap();
        q.push(2, 20).unwrap();
        q.push(3, 30).unwrap();
        // Output buffer only has room for 2.
        let mut out: heapless::Vec<u8, 2> = heapless::Vec::new();
        q.drain_expired(100, &mut out);
        assert_eq!(out.as_slice(), &[1, 2]);
        // Pid 3 remains in queue for retry next tick.
        assert_eq!(q.len(), 1);
    }

    #[test]
    fn sleep_queue_equal_expiry_preserves_push_order() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 100).unwrap();
        q.push(3, 100).unwrap();
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(100, &mut out);
        assert_eq!(out.as_slice(), &[1, 2, 3]);
    }

    // ---- safe_expiry tests ----

    #[test]
    fn safe_expiry_zero_timeout_bumped_to_two() {
        assert_eq!(safe_expiry(100, 0), 102);
    }

    #[test]
    fn safe_expiry_one_tick_bumped_to_two() {
        assert_eq!(safe_expiry(100, 1), 102);
    }

    #[test]
    fn safe_expiry_two_tick_unchanged() {
        assert_eq!(safe_expiry(100, 2), 102);
    }

    #[test]
    fn safe_expiry_large_timeout_unchanged() {
        assert_eq!(safe_expiry(100, 50), 150);
        assert_eq!(safe_expiry(100, 100), 200);
    }

    #[test]
    fn safe_expiry_overflow_saturates() {
        assert_eq!(safe_expiry(u64::MAX, 5), u64::MAX);
        assert_eq!(safe_expiry(u64::MAX - 1, 100), u64::MAX);
    }

    // ---- WaitQueue::remove_by_id tests ----

    #[test]
    fn wq_remove_by_id_found() {
        let mut q = WaitQueue::<4>::new();
        q.push(1).unwrap();
        q.push(2).unwrap();
        q.push(3).unwrap();
        assert!(q.remove_by_id(2));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some(1));
        assert_eq!(q.pop_front(), Some(3));
    }

    #[test]
    fn wq_remove_by_id_not_found() {
        let mut q = WaitQueue::<4>::new();
        q.push(1).unwrap();
        q.push(2).unwrap();
        assert!(!q.remove_by_id(99));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some(1));
        assert_eq!(q.pop_front(), Some(2));
    }

    #[test]
    fn wq_remove_by_id_empty() {
        let mut q = WaitQueue::<4>::new();
        assert!(!q.remove_by_id(1));
        assert!(q.is_empty());
    }

    #[test]
    fn wq_remove_by_id_preserves_order() {
        let mut q = WaitQueue::<8>::new();
        for pid in 0..6u8 {
            q.push(pid).unwrap();
        }
        assert!(q.remove_by_id(3));
        assert_eq!(q.len(), 5);
        assert_eq!(q.pop_front(), Some(0));
        assert_eq!(q.pop_front(), Some(1));
        assert_eq!(q.pop_front(), Some(2));
        assert_eq!(q.pop_front(), Some(4));
        assert_eq!(q.pop_front(), Some(5));
    }

    #[test]
    fn wq_remove_by_id_duplicate_removes_first_only() {
        let mut q = WaitQueue::<4>::new();
        q.push(5).unwrap();
        q.push(5).unwrap();
        q.push(5).unwrap();
        assert!(q.remove_by_id(5));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some(5));
        assert_eq!(q.pop_front(), Some(5));
    }

    // ---- TimedWaitQueue::remove_by_id tests ----

    #[test]
    fn twq_remove_by_id_found() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(3, 300).unwrap();
        assert!(q.remove_by_id(2));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some((1, 100)));
        assert_eq!(q.pop_front(), Some((3, 300)));
    }

    #[test]
    fn twq_remove_by_id_not_found() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        assert!(!q.remove_by_id(99));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some((1, 100)));
        assert_eq!(q.pop_front(), Some((2, 200)));
    }

    #[test]
    fn twq_remove_by_id_empty() {
        let mut q = TimedWaitQueue::<4>::new();
        assert!(!q.remove_by_id(1));
        assert!(q.is_empty());
    }

    #[test]
    fn twq_remove_by_id_preserves_order() {
        let mut q = TimedWaitQueue::<8>::new();
        for pid in 0..6u8 {
            q.push(pid, pid as u64 * 10).unwrap();
        }
        assert!(q.remove_by_id(3));
        assert_eq!(q.len(), 5);
        assert_eq!(q.pop_front(), Some((0, 0)));
        assert_eq!(q.pop_front(), Some((1, 10)));
        assert_eq!(q.pop_front(), Some((2, 20)));
        assert_eq!(q.pop_front(), Some((4, 40)));
        assert_eq!(q.pop_front(), Some((5, 50)));
    }

    #[test]
    fn twq_remove_by_id_duplicate_removes_first_only() {
        let mut q = TimedWaitQueue::<4>::new();
        q.push(5, 100).unwrap();
        q.push(5, 200).unwrap();
        q.push(5, 300).unwrap();
        assert!(q.remove_by_id(5));
        assert_eq!(q.len(), 2);
        assert_eq!(q.pop_front(), Some((5, 200)));
        assert_eq!(q.pop_front(), Some((5, 300)));
    }

    // ---- SleepQueue::remove_by_id tests ----

    #[test]
    fn sq_remove_by_id_found() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(3, 300).unwrap();
        assert!(q.remove_by_id(2));
        assert_eq!(q.len(), 2);
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(300, &mut out);
        assert_eq!(out.as_slice(), &[1, 3]);
    }

    #[test]
    fn sq_remove_by_id_not_found() {
        let mut q = SleepQueue::<4>::new();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        assert!(!q.remove_by_id(99));
        assert_eq!(q.len(), 2);
    }

    #[test]
    fn sq_remove_by_id_empty() {
        let mut q = SleepQueue::<4>::new();
        assert!(!q.remove_by_id(1));
        assert!(q.is_empty());
    }

    #[test]
    fn sq_remove_by_id_preserves_sorted_order() {
        let mut q = SleepQueue::<8>::new();
        // Insert out of order; SleepQueue sorts by expiry.
        q.push(3, 300).unwrap();
        q.push(1, 100).unwrap();
        q.push(2, 200).unwrap();
        q.push(4, 400).unwrap();
        // Remove pid 2 (expiry 200)
        assert!(q.remove_by_id(2));
        assert_eq!(q.len(), 3);
        let mut out: heapless::Vec<u8, 8> = heapless::Vec::new();
        q.drain_expired(400, &mut out);
        // Should drain in sorted expiry order: 100, 300, 400
        assert_eq!(out.as_slice(), &[1, 3, 4]);
    }

    #[test]
    fn sq_remove_by_id_duplicate_removes_first_only() {
        let mut q = SleepQueue::<4>::new();
        q.push(5, 100).unwrap();
        q.push(5, 200).unwrap();
        q.push(5, 300).unwrap();
        assert!(q.remove_by_id(5));
        assert_eq!(q.len(), 2);
        let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
        q.drain_expired(300, &mut out);
        assert_eq!(out.as_slice(), &[5, 5]);
    }
}
