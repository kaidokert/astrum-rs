//! Debug ring buffer: lock-free SPSC ring buffer (N must be power of 2).

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

/// Trait for type-erased access to debug ring buffers.
///
/// This provides safe, polymorphic access to `DebugRingBuffer<N>` without
/// exposing the const generic `N` to callers.
pub trait DebugBuffer: Sync {
    /// Drain up to `budget` bytes (kernel-side). Returns bytes read.
    fn drain(&self, output: &mut [u8], budget: usize) -> usize;
    /// Returns bytes available to read.
    fn available(&self) -> usize;
    /// Returns true if empty.
    fn is_empty(&self) -> bool;
    /// Returns bytes dropped due to overflow.
    fn dropped(&self) -> u32;
}

// Log levels: ERROR=0 (highest) to TRACE=4 (lowest)
pub const LOG_ERROR: u8 = 0;
pub const LOG_WARN: u8 = 1;
pub const LOG_INFO: u8 = 2;
pub const LOG_DEBUG: u8 = 3;
pub const LOG_TRACE: u8 = 4;
// Record kinds
pub const KIND_TEXT: u8 = 0;
pub const KIND_DEFMT: u8 = 1;
pub const KIND_BINARY: u8 = 2;
pub const KIND_EVENT_ID: u8 = 3;

/// Debug record header (4 bytes): [len, level, kind, flags].
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DebugRecordHeader {
    pub len: u8,
    pub level: u8,
    pub kind: u8,
    pub flags: u8,
}
impl DebugRecordHeader {
    pub const SIZE: usize = 4;
    pub const fn new(len: u8, level: u8, kind: u8) -> Self {
        Self {
            len,
            level,
            kind,
            flags: 0,
        }
    }
    pub const fn to_bytes(self) -> [u8; 4] {
        [self.len, self.level, self.kind, self.flags]
    }
    pub const fn from_bytes(b: [u8; 4]) -> Self {
        Self {
            len: b[0],
            level: b[1],
            kind: b[2],
            flags: b[3],
        }
    }
}

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
    /// Write a framed record (header + payload) atomically. Returns false on overflow.
    pub fn write_record(&self, level: u8, kind: u8, payload: &[u8]) -> bool {
        let len = payload.len();
        if len > 255 {
            self.dropped.fetch_add((4 + len) as u32, Ordering::Relaxed);
            return false;
        }
        let total = 4 + len;
        let w = self.write_idx.load(Ordering::Relaxed);
        let r = self.read_idx.load(Ordering::Relaxed);
        if total as u32 > (N as u32).saturating_sub(w.wrapping_sub(r)) {
            self.dropped.fetch_add(total as u32, Ordering::Relaxed);
            return false;
        }
        let hdr = DebugRecordHeader::new(len as u8, level, kind).to_bytes();
        // SAFETY: Same SPSC invariant as write().
        unsafe {
            let buf = &mut *self.data.get();
            let mut p = (w & Self::MASK) as usize;
            for &b in &hdr {
                buf[p] = b;
                p = (p + 1) & (N - 1);
            }
            for &b in payload {
                buf[p] = b;
                p = (p + 1) & (N - 1);
            }
        }
        self.write_idx
            .store(w.wrapping_add(total as u32), Ordering::Release);
        true
    }
}

impl<const N: usize> Default for DebugRingBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DebugBuffer for DebugRingBuffer<N> {
    fn drain(&self, output: &mut [u8], budget: usize) -> usize {
        DebugRingBuffer::drain(self, output, budget)
    }
    fn available(&self) -> usize {
        DebugRingBuffer::available(self)
    }
    fn is_empty(&self) -> bool {
        DebugRingBuffer::is_empty(self)
    }
    fn dropped(&self) -> u32 {
        DebugRingBuffer::dropped(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn basic_ring_buffer() {
        let rb = DebugRingBuffer::<16>::new();
        assert!(rb.is_empty() && rb.dropped() == 0 && rb.write(&[1, 2, 3, 4]));
        let mut out = [0u8; 8];
        assert_eq!(rb.drain(&mut out, 8), 4);
        let rb2 = DebugRingBuffer::<8>::new();
        rb2.write(&[1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(!rb2.write(&[9]) && rb2.dropped() == 1);
    }
    #[test]
    fn header_packing_and_constants() {
        assert_eq!(core::mem::size_of::<DebugRecordHeader>(), 4);
        let h = DebugRecordHeader::new(42, LOG_WARN, KIND_TEXT);
        assert_eq!((h.len, h.level, h.kind, h.flags), (42, 1, 0, 0));
        assert_eq!(h, DebugRecordHeader::from_bytes(h.to_bytes()));
        assert!((LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_TRACE) == (0, 1, 2, 3, 4));
        assert!((KIND_TEXT, KIND_DEFMT, KIND_BINARY, KIND_EVENT_ID) == (0, 1, 2, 3));
    }
    #[test]
    fn record_write_roundtrip() {
        let rb = DebugRingBuffer::<64>::new();
        assert!(rb.write_record(LOG_INFO, KIND_TEXT, b"hello") && rb.available() == 9);
        let mut out = [0u8; 32];
        assert_eq!(rb.drain(&mut out, 32), 9);
        let hdr = DebugRecordHeader::from_bytes([out[0], out[1], out[2], out[3]]);
        assert_eq!((hdr.len, hdr.level, hdr.kind), (5, LOG_INFO, KIND_TEXT));
        assert_eq!(&out[4..9], b"hello");
        let rb2 = DebugRingBuffer::<16>::new();
        rb2.write_record(LOG_INFO, KIND_TEXT, b"12345678");
        assert!(!rb2.write_record(LOG_DEBUG, KIND_TEXT, b"fail") && rb2.dropped() > 0);
    }
}
