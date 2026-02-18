//! Debug ring buffer: lock-free SPSC ring buffer (N must be power of 2).

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

/// Debug ring buffer for partition-to-kernel communication.
pub struct DebugRingBuffer<const N: usize> {
    write_idx: AtomicU32,
    read_idx: AtomicU32,
    dropped: AtomicU32,
    data: UnsafeCell<[u8; N]>,
}

// SAFETY: DebugRingBuffer is designed for SPSC use where the producer (partition)
// calls write() and the consumer (kernel) calls drain(). The atomics provide
// synchronization, and UnsafeCell is only accessed through properly synchronized
// operations.
unsafe impl<const N: usize> Sync for DebugRingBuffer<N> {}

impl<const N: usize> DebugRingBuffer<N> {
    const MASK: u32 = (N - 1) as u32;

    // Compile-time check: N must be a power of 2 and greater than 0
    const _POWER_OF_TWO_CHECK: () = {
        assert!(N > 0 && (N & (N - 1)) == 0, "N must be a power of 2");
    };

    /// Create a new empty ring buffer.
    #[allow(clippy::let_unit_value)]
    pub const fn new() -> Self {
        // Force compile-time evaluation of the power-of-two check
        let _ = Self::_POWER_OF_TWO_CHECK;
        Self {
            write_idx: AtomicU32::new(0),
            read_idx: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
            data: UnsafeCell::new([0u8; N]),
        }
    }

    /// Write data (partition-side). Returns false on overflow.
    pub fn write(&self, data: &[u8]) -> bool {
        let write_pos = self.write_idx.load(Ordering::Relaxed);
        let read_pos = self.read_idx.load(Ordering::Relaxed);
        let used = write_pos.wrapping_sub(read_pos);
        let available = (N as u32).saturating_sub(used);

        if data.len() as u32 > available {
            self.dropped.fetch_add(data.len() as u32, Ordering::Relaxed);
            return false;
        }
        let start = (write_pos & Self::MASK) as usize;
        let first_chunk = core::cmp::min(data.len(), N - start);

        // SAFETY: We are the only writer (SPSC invariant). The read_idx check above
        // ensures we don't overwrite unread data. The kernel (reader) only reads
        // indices before read_idx, and we only write at write_idx and beyond.
        unsafe {
            let buf_ptr = self.data.get();
            let buf = &mut *buf_ptr;
            buf[start..start + first_chunk].copy_from_slice(&data[..first_chunk]);
            if first_chunk < data.len() {
                buf[..data.len() - first_chunk].copy_from_slice(&data[first_chunk..]);
            }
        }

        self.write_idx
            .store(write_pos.wrapping_add(data.len() as u32), Ordering::Release);
        true
    }

    /// Drain up to `budget` bytes (kernel-side). Returns bytes read.
    pub fn drain(&self, output: &mut [u8], budget: usize) -> usize {
        let write_pos = self.write_idx.load(Ordering::Acquire);
        let read_pos = self.read_idx.load(Ordering::Relaxed);
        let available = write_pos.wrapping_sub(read_pos) as usize;
        let to_read = available.min(budget).min(output.len());
        if to_read == 0 {
            return 0;
        }

        let start = (read_pos & Self::MASK) as usize;
        let first_chunk = to_read.min(N - start);

        // SAFETY: We are the only reader (SPSC invariant). The write_idx check above
        // ensures data is available. The producer only writes at write_idx and beyond,
        // and we only read indices from read_idx up to write_idx.
        unsafe {
            let buf_ptr = self.data.get();
            let buf = &*buf_ptr;
            output[..first_chunk].copy_from_slice(&buf[start..start + first_chunk]);
            if first_chunk < to_read {
                let second = to_read - first_chunk;
                output[first_chunk..first_chunk + second].copy_from_slice(&buf[..second]);
            }
        }

        self.read_idx
            .store(read_pos.wrapping_add(to_read as u32), Ordering::Release);
        to_read
    }

    /// Returns bytes dropped due to overflow.
    pub fn dropped(&self) -> u32 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Returns bytes available to read.
    pub fn available(&self) -> usize {
        self.write_idx
            .load(Ordering::Acquire)
            .wrapping_sub(self.read_idx.load(Ordering::Relaxed)) as usize
    }

    /// Returns true if empty.
    pub fn is_empty(&self) -> bool {
        self.available() == 0
    }
}

impl<const N: usize> Default for DebugRingBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_single_write() {
        let rb = DebugRingBuffer::<16>::new();
        assert!(rb.is_empty() && rb.dropped() == 0 && rb.drain(&mut [0u8; 16], 16) == 0);
        assert!(rb.write(&[1, 2, 3, 4]) && rb.available() == 4);
        let mut out = [0u8; 8];
        assert!(rb.drain(&mut out, 8) == 4 && out[..4] == [1, 2, 3, 4]);
    }

    #[test]
    fn wrap_and_overflow() {
        let rb = DebugRingBuffer::<8>::new();
        let mut out = [0u8; 8];
        rb.write(&[1, 2, 3, 4, 5, 6]);
        rb.drain(&mut out, 6);
        rb.write(&[7, 8, 9, 10, 11, 12]);
        assert_eq!(rb.drain(&mut out, 8), 6);
        rb.write(&[1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(!rb.write(&[9]) && rb.dropped() == 1);
    }
}
