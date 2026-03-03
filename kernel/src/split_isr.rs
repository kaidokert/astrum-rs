//! Split-ISR ring buffer for top-half to bottom-half communication.
//!
//! `IsrRingBuffer<D, M>` is a fixed-depth ring of fixed-size event records.
//! `D` is the max pending record count; `M` is payload bytes per record.
//! All storage is inline (no heap). Single-core Cortex-M only.
//!
//! # Single-producer / single-consumer protocol
//!
//! The ring buffer follows an SPSC (single-producer, single-consumer)
//! discipline:
//!
//! - **Producer (ISR):** The hardware interrupt handler calls
//!   [`StaticIsrRing::push_from_isr`] to enqueue an event record.
//! - **Consumer (partition):** The partition's bottom-half calls
//!   [`StaticIsrRing::pop_with`] to dequeue and process records.
//!
//! Because both sides share mutable state (`head`, `count`) without
//! atomics, access must be **mutually exclusive**.
//!
//! # IRQ masking requirement
//!
//! On single-core Cortex-M the only way a partition can race with the
//! ISR is if the bound IRQ fires while the partition is mid-operation.
//! The partition **must mask the bound IRQ** before calling any reader
//! method ([`pop_with`](StaticIsrRing::pop_with),
//! [`reset_overflow_count`](StaticIsrRing::reset_overflow_count),
//! [`len`](StaticIsrRing::len), [`is_empty`](StaticIsrRing::is_empty),
//! [`is_full`](StaticIsrRing::is_full)). Failure to mask allows the
//! ISR to preempt mid-read, corrupting the head/count state.
//!
//! # Drain-then-ack pattern
//!
//! The typical partition-side loop looks like this:
//!
//! ```rust,ignore
//! loop {
//!     // Block until the kernel signals that the ISR fired.
//!     plib::sys_event_wait(EVT_IRQ);
//!
//!     // The IRQ is already masked by the dispatch handler (PartitionAcks
//!     // model), so it is safe to drain without explicit masking.
//!     loop {
//!         // SAFETY: sole consumer; IRQ is masked (ack not yet called).
//!         let popped = unsafe { RING.pop_with(|tag, payload| {
//!             handle_event(tag, payload);
//!         })};
//!         if !popped { break; }
//!     }
//!
//!     // Check for overflow (events dropped while the ring was full).
//!     // SAFETY: IRQ still masked.
//!     let dropped = unsafe { RING.reset_overflow_count() };
//!     if dropped > 0 {
//!         report_overflow(dropped);
//!     }
//!
//!     // Unmask the IRQ so the ISR can fire again.
//!     plib::sys_irq_ack(IRQ_NUM);
//! }
//! ```
//!
//! # `PartitionAcks` and implicit masking
//!
//! When an IRQ binding uses the default [`PartitionAcks`] clear model,
//! the kernel's dispatch handler masks the IRQ line in the NVIC before
//! signalling the partition.  The line stays masked until the partition
//! calls `SYS_IRQ_ACK`.  This means the partition does **not** need to
//! mask the IRQ explicitly — the entire window between `event_wait`
//! returning and `sys_irq_ack` completing is implicitly protected.
//!
//! [`PartitionAcks`]: crate::irq_dispatch::IrqClearModel::PartitionAcks

/// Error returned when the ring buffer is full.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RingBufferFull;

/// A single event record in the ring buffer.
#[derive(Clone)]
struct EventRecord<const M: usize> {
    tag: u8,
    len: usize,
    payload: [u8; M],
}

impl<const M: usize> EventRecord<M> {
    const fn empty() -> Self {
        Self {
            tag: 0,
            len: 0,
            payload: [0u8; M],
        }
    }
}

/// `Sync` wrapper around [`IsrRingBuffer`] for `static` placement.
///
/// # Safety invariant (`Sync`)
/// Single-core Cortex-M only. ISR masks its IRQ before returning;
/// partition masks IRQ before draining — no concurrent access.
pub struct StaticIsrRing<const D: usize, const M: usize> {
    inner: core::cell::UnsafeCell<IsrRingBuffer<D, M>>,
}

// SAFETY: On single-core Cortex-M the ISR/partition mutual-exclusion
// protocol (see struct-level doc) prevents concurrent access.
// Gated via feature flags: the Sync impl is only sound when a single core
// guarantees that IRQ-masking prevents concurrency.  When `multi-core` is
// enabled (without `single-core`) the impl is absent, so any attempt to
// place a StaticIsrRing in a `static` will fail with a missing-Sync error.
// `single-core` exists as an explicit override for `--all-features` builds.
#[cfg(any(feature = "single-core", not(feature = "multi-core")))]
unsafe impl<const D: usize, const M: usize> Sync for StaticIsrRing<D, M> {}

#[cfg(all(feature = "multi-core", not(feature = "single-core")))]
compile_error!(
    "StaticIsrRing requires external synchronization on multi-core targets. \
     Either remove the `multi-core` feature or add `single-core` to override."
);

impl<const D: usize, const M: usize> Default for StaticIsrRing<D, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const D: usize, const M: usize> StaticIsrRing<D, M> {
    /// Create an empty ring, usable in `static` items.
    pub const fn new() -> Self {
        Self {
            inner: core::cell::UnsafeCell::new(IsrRingBuffer::new()),
        }
    }

    /// Push an event from ISR context.
    /// # Safety
    /// Caller must guarantee no concurrent access.
    pub unsafe fn push_from_isr(&self, tag: u8, data: &[u8]) -> Result<(), RingBufferFull> {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &mut *self.inner.get() }.push_from_isr(tag, data)
    }

    /// Pop oldest record, passing `(tag, payload)` to `f`.
    /// # Safety
    /// Caller must guarantee no concurrent access.
    pub unsafe fn pop_with<F>(&self, f: F) -> bool
    where
        F: FnOnce(u8, &[u8]),
    {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &mut *self.inner.get() }.pop_with(f)
    }

    /// Number of events dropped due to a full buffer.
    /// # Safety
    /// Caller must guarantee no concurrent access (e.g. mask the bound IRQ).
    pub unsafe fn overflow_count(&self) -> usize {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &*self.inner.get() }.overflow_count()
    }

    /// Reset overflow count to zero, returning previous value.
    /// # Safety
    /// Caller must guarantee no concurrent access.
    pub unsafe fn reset_overflow_count(&self) -> usize {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &mut *self.inner.get() }.reset_overflow_count()
    }

    /// Number of pending event records.
    /// # Safety
    /// Caller must guarantee no concurrent access (e.g. mask the bound IRQ).
    pub unsafe fn len(&self) -> usize {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &*self.inner.get() }.len()
    }

    /// `true` if no event records are pending.
    /// # Safety
    /// Caller must guarantee no concurrent access (e.g. mask the bound IRQ).
    pub unsafe fn is_empty(&self) -> bool {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &*self.inner.get() }.is_empty()
    }

    /// `true` if the buffer has no free slots.
    /// # Safety
    /// Caller must guarantee no concurrent access (e.g. mask the bound IRQ).
    pub unsafe fn is_full(&self) -> bool {
        // SAFETY: caller guarantees single-core mutual exclusion.
        unsafe { &*self.inner.get() }.is_full()
    }
}

/// Fixed-depth ring buffer for ISR top-half to bottom-half communication.
///
/// `D` — max pending records. `M` — max payload bytes per record.
/// `push_from_isr` is O(1), allocation-free, ISR-safe on single-core
/// Cortex-M (caller ensures mutual exclusion).
pub struct IsrRingBuffer<const D: usize, const M: usize> {
    buf: [EventRecord<M>; D],
    head: usize,
    count: usize,
    overflow_count: usize,
}

impl<const D: usize, const M: usize> Default for IsrRingBuffer<D, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const D: usize, const M: usize> IsrRingBuffer<D, M> {
    /// Create an empty ring buffer.
    pub const fn new() -> Self {
        Self {
            buf: [const { EventRecord::empty() }; D],
            head: 0,
            count: 0,
            overflow_count: 0,
        }
    }

    /// Push an event from ISR context. Truncates `data` to `M` bytes.
    /// Returns `Err(RingBufferFull)` when no slots are free.
    pub fn push_from_isr(&mut self, tag: u8, data: &[u8]) -> Result<(), RingBufferFull> {
        if self.count >= D {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(RingBufferFull);
        }
        let tail = (self.head + self.count) % D;
        let copy_len = data.len().min(M);
        let slot = self.buf.get_mut(tail).ok_or(RingBufferFull)?;
        slot.tag = tag;
        slot.len = copy_len;
        slot.payload[..copy_len].copy_from_slice(&data[..copy_len]);
        self.count += 1;
        Ok(())
    }

    /// Pop the oldest record, passing it to `f` for in-place processing.
    /// The closure receives `(tag, payload_slice)`. Returns `false` if
    /// the buffer was empty (closure not called), `true` otherwise.
    pub fn pop_with<F>(&mut self, f: F) -> bool
    where
        F: FnOnce(u8, &[u8]),
    {
        if self.count == 0 {
            return false;
        }
        let idx = self.head;
        let slot = match self.buf.get(idx) {
            Some(s) => s,
            None => return false,
        };
        let payload = match slot.payload.get(..slot.len) {
            Some(p) => p,
            None => return false,
        };
        f(slot.tag, payload);
        self.head = (self.head + 1) % D;
        self.count -= 1;
        true
    }

    /// Returns the number of pending event records.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns `true` if no event records are pending.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Returns `true` if the buffer has no free slots.
    pub fn is_full(&self) -> bool {
        self.count >= D
    }

    /// Returns the number of events dropped due to a full buffer.
    pub fn overflow_count(&self) -> usize {
        self.overflow_count
    }

    /// Returns the current overflow count and resets it to zero.
    pub fn reset_overflow_count(&mut self) -> usize {
        let prev = self.overflow_count;
        self.overflow_count = 0;
        prev
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ===== Empty buffer tests =====

    #[test]
    fn new_buffer_is_empty() {
        let rb = IsrRingBuffer::<4, 8>::new();
        assert!(rb.is_empty());
        assert!(!rb.is_full());
        assert_eq!(rb.len(), 0);
    }

    #[test]
    fn pop_empty_returns_false() {
        let mut rb = IsrRingBuffer::<4, 8>::new();
        assert!(!rb.pop_with(|_, _| panic!("should not be called")));
    }

    // ===== Basic push/pop tests =====

    #[test]
    fn push_and_pop_single() {
        let mut rb = IsrRingBuffer::<4, 8>::new();
        rb.push_from_isr(42, &[1, 2, 3]).unwrap();
        assert_eq!(rb.len(), 1);

        rb.pop_with(|tag, data| {
            assert_eq!(tag, 42);
            assert_eq!(data, &[1, 2, 3]);
        });
        assert!(rb.is_empty());
    }

    #[test]
    fn push_empty_payload() {
        let mut rb = IsrRingBuffer::<4, 8>::new();
        rb.push_from_isr(7, &[]).unwrap();
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 7);
            assert!(data.is_empty());
        });
    }

    #[test]
    fn push_truncates_oversized_payload() {
        let mut rb = IsrRingBuffer::<2, 3>::new();
        rb.push_from_isr(5, &[1, 2, 3, 4, 5]).unwrap();
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 5);
            assert_eq!(data, &[1, 2, 3]);
        });
    }

    // ===== FIFO ordering tests =====

    #[test]
    fn fifo_ordering() {
        let mut rb = IsrRingBuffer::<4, 8>::new();
        rb.push_from_isr(1, &[10]).unwrap();
        rb.push_from_isr(2, &[20]).unwrap();
        rb.push_from_isr(3, &[30]).unwrap();

        rb.pop_with(|tag, data| assert_eq!((tag, data), (1, &[10][..])));
        rb.pop_with(|tag, data| assert_eq!((tag, data), (2, &[20][..])));
        rb.pop_with(|tag, data| assert_eq!((tag, data), (3, &[30][..])));
        assert!(rb.is_empty());
    }

    // ===== Full buffer rejection tests =====

    #[test]
    fn full_buffer_rejects_push() {
        let mut rb = IsrRingBuffer::<2, 4>::new();
        rb.push_from_isr(1, &[1]).unwrap();
        rb.push_from_isr(2, &[2]).unwrap();
        assert!(rb.is_full());
        assert_eq!(rb.push_from_isr(3, &[3]), Err(RingBufferFull));
        assert_eq!(rb.len(), 2);
    }

    #[test]
    fn depth_one_buffer() {
        let mut rb = IsrRingBuffer::<1, 4>::new();
        rb.push_from_isr(1, &[10]).unwrap();
        assert!(rb.is_full());
        assert_eq!(rb.push_from_isr(2, &[20]), Err(RingBufferFull));
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 1);
            assert_eq!(data, &[10]);
        });
        assert!(rb.is_empty());
    }

    // ===== Wrap-around tests =====

    #[test]
    fn wrap_around_maintains_fifo() {
        let mut rb = IsrRingBuffer::<3, 4>::new();
        // Advance head past the start
        rb.push_from_isr(1, &[10]).unwrap();
        rb.push_from_isr(2, &[20]).unwrap();
        rb.pop_with(|_, _| {});
        rb.pop_with(|_, _| {});

        // head=2, count=0. Push 3 items that wrap around.
        rb.push_from_isr(3, &[30]).unwrap();
        rb.push_from_isr(4, &[40]).unwrap();
        rb.push_from_isr(5, &[50]).unwrap();
        assert!(rb.is_full());

        rb.pop_with(|tag, data| assert_eq!((tag, data), (3, &[30][..])));
        rb.pop_with(|tag, data| assert_eq!((tag, data), (4, &[40][..])));
        rb.pop_with(|tag, data| assert_eq!((tag, data), (5, &[50][..])));
        assert!(rb.is_empty());
    }

    #[test]
    fn interleaved_push_pop_with_wrap() {
        let mut rb = IsrRingBuffer::<2, 4>::new();
        for i in 0u8..10 {
            rb.push_from_isr(i, &[i * 10]).unwrap();
            rb.pop_with(|tag, data| {
                assert_eq!(tag, i);
                assert_eq!(data, &[i * 10]);
            });
        }
        assert!(rb.is_empty());
    }

    // ===== Reuse after drain =====

    #[test]
    fn reuse_after_full_drain() {
        let mut rb = IsrRingBuffer::<2, 4>::new();
        rb.push_from_isr(1, &[1]).unwrap();
        rb.push_from_isr(2, &[2]).unwrap();
        rb.pop_with(|_, _| {});
        rb.pop_with(|_, _| {});
        assert!(rb.is_empty());

        rb.push_from_isr(3, &[3]).unwrap();
        rb.push_from_isr(4, &[4]).unwrap();
        rb.pop_with(|t, _| assert_eq!(t, 3));
        rb.pop_with(|t, _| assert_eq!(t, 4));
    }

    // ===== Overflow count tests =====

    #[test]
    fn overflow_count_increments_on_each_rejected_push() {
        let mut rb = IsrRingBuffer::<2, 4>::new();
        rb.push_from_isr(1, &[1]).unwrap();
        rb.push_from_isr(2, &[2]).unwrap();
        assert_eq!(rb.overflow_count(), 0);

        assert_eq!(rb.push_from_isr(3, &[3]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 1);
        assert_eq!(rb.push_from_isr(4, &[4]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 2);
        assert_eq!(rb.push_from_isr(5, &[5]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 3);
    }

    #[test]
    fn overflow_count_stays_zero_when_all_pushes_succeed() {
        let mut rb = IsrRingBuffer::<4, 4>::new();
        rb.push_from_isr(1, &[1]).unwrap();
        rb.push_from_isr(2, &[2]).unwrap();
        rb.push_from_isr(3, &[3]).unwrap();
        rb.push_from_isr(4, &[4]).unwrap();
        assert_eq!(rb.overflow_count(), 0);
    }

    #[test]
    fn reset_overflow_count_returns_previous_and_resets() {
        let mut rb = IsrRingBuffer::<1, 4>::new();
        rb.push_from_isr(1, &[1]).unwrap();

        assert_eq!(rb.push_from_isr(2, &[2]), Err(RingBufferFull));
        assert_eq!(rb.push_from_isr(3, &[3]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 2);

        let prev = rb.reset_overflow_count();
        assert_eq!(prev, 2);
        assert_eq!(rb.overflow_count(), 0);

        // Accumulate again after reset
        assert_eq!(rb.push_from_isr(4, &[4]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 1);
    }

    // ===== Zero-depth (D=0) edge case =====

    #[test]
    fn zero_depth_buffer_is_simultaneously_empty_and_full() {
        let rb = IsrRingBuffer::<0, 8>::new();
        assert!(rb.is_empty());
        assert!(rb.is_full());
        assert_eq!(rb.len(), 0);
    }

    #[test]
    fn zero_depth_buffer_push_always_fails() {
        let mut rb = IsrRingBuffer::<0, 8>::new();
        assert_eq!(rb.push_from_isr(1, &[0xAA]), Err(RingBufferFull));
        assert_eq!(rb.push_from_isr(2, &[]), Err(RingBufferFull));
        assert_eq!(rb.overflow_count(), 2);
    }

    #[test]
    fn zero_depth_buffer_pop_returns_false() {
        let mut rb = IsrRingBuffer::<0, 8>::new();
        assert!(!rb.pop_with(|_, _| panic!("should not be called")));
    }

    // ===== Default trait =====

    #[test]
    fn default_matches_new() {
        let from_new = IsrRingBuffer::<4, 8>::new();
        let from_default = IsrRingBuffer::<4, 8>::default();
        assert_eq!(from_new.len(), from_default.len());
        assert_eq!(from_new.is_empty(), from_default.is_empty());
        assert_eq!(from_new.is_full(), from_default.is_full());
        assert_eq!(from_new.overflow_count(), from_default.overflow_count());
    }

    // ===== Zero-payload (M=0) edge case =====

    #[test]
    fn zero_payload_push_truncates_data_to_empty() {
        let mut rb = IsrRingBuffer::<4, 0>::new();
        rb.push_from_isr(42, &[1, 2, 3]).unwrap();
        assert_eq!(rb.len(), 1);

        let mut called = false;
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 42);
            assert!(data.is_empty());
            called = true;
        });
        assert!(called);
    }

    #[test]
    fn zero_payload_empty_push_succeeds() {
        let mut rb = IsrRingBuffer::<4, 0>::new();
        rb.push_from_isr(7, &[]).unwrap();
        let mut called = false;
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 7);
            assert!(data.is_empty());
            called = true;
        });
        assert!(called);
    }

    // ===== Single-byte payload (M=1) edge case =====

    #[test]
    fn single_byte_payload_truncates_oversized_data() {
        let mut rb = IsrRingBuffer::<4, 1>::new();
        rb.push_from_isr(99, &[0xAA, 0xBB, 0xCC]).unwrap();

        let mut called = false;
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 99);
            assert_eq!(data, &[0xAA]);
            called = true;
        });
        assert!(called);
    }

    #[test]
    fn single_byte_payload_exact_fit() {
        let mut rb = IsrRingBuffer::<4, 1>::new();
        rb.push_from_isr(1, &[0xFF]).unwrap();
        let mut called = false;
        rb.pop_with(|tag, data| {
            assert_eq!(tag, 1);
            assert_eq!(data, &[0xFF]);
            called = true;
        });
        assert!(called);
    }

    // ===== StaticIsrRing tests =====

    #[test]
    fn static_ring_construction_is_empty() {
        let ring = StaticIsrRing::<4, 8>::new();
        // SAFETY: test is single-threaded; no concurrent access.
        unsafe {
            assert!(ring.is_empty());
            assert!(!ring.is_full());
            assert_eq!(ring.len(), 0);
            assert_eq!(ring.overflow_count(), 0);
        }
    }

    #[test]
    fn static_ring_push_pop_round_trip() {
        let ring = StaticIsrRing::<4, 8>::new();
        // SAFETY: test is single-threaded; no concurrent access.
        unsafe {
            ring.push_from_isr(42, &[0xDE, 0xAD]).unwrap();
            assert_eq!(ring.len(), 1);
        };
        let mut got = (0u8, [0u8; 2]);
        // SAFETY: test is single-threaded.
        let popped = unsafe {
            ring.pop_with(|tag, data| {
                got.0 = tag;
                got.1[..data.len()].copy_from_slice(data);
            })
        };
        assert!(popped);
        assert_eq!(got, (42, [0xDE, 0xAD]));
        // SAFETY: test is single-threaded.
        assert!(unsafe { ring.is_empty() });
    }

    #[test]
    fn static_ring_overflow_count_and_reset() {
        let ring = StaticIsrRing::<1, 4>::new();
        // SAFETY: test is single-threaded; no concurrent access.
        unsafe {
            ring.push_from_isr(1, &[1]).unwrap();
            assert!(ring.is_full());
            assert_eq!(ring.push_from_isr(2, &[2]), Err(RingBufferFull));
            assert_eq!(ring.push_from_isr(3, &[3]), Err(RingBufferFull));
            assert_eq!(ring.overflow_count(), 2);
            let prev = ring.reset_overflow_count();
            assert_eq!(prev, 2);
            assert_eq!(ring.overflow_count(), 0);
        }
    }

    #[test]
    #[cfg(not(feature = "multi-core"))]
    fn static_ring_implements_sync() {
        fn requires_sync<T: Sync>() {}
        requires_sync::<StaticIsrRing<4, 4>>();
    }
}
