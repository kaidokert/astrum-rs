//! Split-ISR ring buffer for top-half to bottom-half communication.
//!
//! `IsrRingBuffer<D, M>` is a fixed-depth ring of fixed-size event records.
//! `D` is the max pending record count; `M` is payload bytes per record.
//! All storage is inline (no heap). Single-core Cortex-M only.

/// Error returned when the ring buffer is full.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RingBufferFull;

/// Notification type delivered to a partition on a device event.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DeviceNotification {
    DataAvailable,
    TxComplete,
    Error,
}

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

/// Fixed-depth ring buffer for ISR top-half to bottom-half communication.
///
/// `D` — max pending records. `M` — max payload bytes per record.
/// `push_from_isr` is O(1), allocation-free, ISR-safe on single-core
/// Cortex-M (caller ensures mutual exclusion).
pub struct IsrRingBuffer<const D: usize, const M: usize> {
    buf: [EventRecord<M>; D],
    head: usize,
    count: usize,
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
        }
    }

    /// Push an event from ISR context. Truncates `data` to `M` bytes.
    /// Returns `Err(RingBufferFull)` when no slots are free.
    pub fn push_from_isr(&mut self, tag: u8, data: &[u8]) -> Result<(), RingBufferFull> {
        if self.count >= D {
            return Err(RingBufferFull);
        }
        let tail = (self.head + self.count) % D;
        let copy_len = data.len().min(M);
        let slot = &mut self.buf[tail];
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
        let slot = &self.buf[idx];
        f(slot.tag, &slot.payload[..slot.len]);
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
}

#[cfg(test)]
mod tests {
    use super::*;

    // ===== DeviceNotification tests =====

    #[test]
    fn device_notification_variants_are_distinct() {
        assert_ne!(
            DeviceNotification::DataAvailable,
            DeviceNotification::TxComplete
        );
        assert_ne!(DeviceNotification::TxComplete, DeviceNotification::Error);
        assert_ne!(DeviceNotification::DataAvailable, DeviceNotification::Error);
        // Verify Clone + Copy + Debug
        let n = DeviceNotification::DataAvailable;
        assert_eq!(n, n.clone());
        assert!(format!("{:?}", DeviceNotification::Error).contains("Error"));
    }

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
}
