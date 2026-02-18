//! Debug ring buffer: re-exports from rtos-traits for kernel use.
//!
//! The actual `DebugRingBuffer` implementation lives in `rtos-traits::debug`
//! to allow both kernel and plib to use it without circular dependencies.

// Re-export all debug types and constants from the shared traits crate
pub use rtos_traits::debug::{
    DebugBuffer, DebugRecordHeader, DebugRingBuffer, KIND_BINARY, KIND_DEFMT, KIND_EVENT_ID,
    KIND_TEXT, LOG_DEBUG, LOG_ERROR, LOG_INFO, LOG_TRACE, LOG_WARN,
};

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;
    use rtos_traits::fmt::FmtBuffer;
    #[test]
    fn basic_ring_buffer() {
        let rb = DebugRingBuffer::<16>::new();
        assert!(rb.is_empty() && rb.dropped() == 0 && rb.write(&[1, 2, 3, 4]) == 4);
        let mut out = [0u8; 8];
        assert_eq!(rb.drain(&mut out, 8), 4);
        let rb2 = DebugRingBuffer::<8>::new();
        rb2.write(&[1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(rb2.write(&[9]) == 0 && rb2.dropped() == 1);
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
    /// Tests FmtBuffer -> DebugRingBuffer path (same path dprint! macro uses in plib).
    #[test]
    fn fmt_buffer_to_ring_buffer() {
        let rb = DebugRingBuffer::<64>::new();
        let mut fb = FmtBuffer::<32>::new();
        write!(fb, "x={}", 42).unwrap();
        assert_eq!(fb.as_bytes(), b"x=42");
        assert!(rb.write_record(LOG_INFO, KIND_TEXT, fb.as_bytes()));
        let mut out = [0u8; 16];
        let n = rb.drain(&mut out, 16);
        assert_eq!(n, 8); // 4 header + 4 payload
        assert_eq!(&out[4..8], b"x=42");
    }

    // TODO: Integration tests for define_partition_debug! macro should be added
    // in a separate integration test crate that can import plib and test the
    // actual exported macro, including $crate resolution and symbol visibility.
}
