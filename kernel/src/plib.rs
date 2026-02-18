//! Partition library: user-space helpers for partitions.

#[cfg(feature = "partition-debug")]
use crate::debug::{DebugRingBuffer, KIND_TEXT};
#[cfg(feature = "partition-debug")]
use crate::syscall::SYS_DEBUG_NOTIFY;

/// Write a debug message to the ring buffer and notify the kernel.
///
/// Writes a framed text record and invokes `SYS_DEBUG_NOTIFY` to signal pending output.
///
/// Returns `true` on success, `false` if the buffer overflowed (message dropped).
#[cfg(feature = "partition-debug")]
pub fn debug_write<const N: usize>(buffer: &DebugRingBuffer<N>, level: u8, msg: &[u8]) -> bool {
    let ok = buffer.write_record(level, KIND_TEXT, msg);
    // Notify kernel even on overflow to drain any buffered data
    crate::svc!(SYS_DEBUG_NOTIFY, 0u32, 0u32, 0u32);
    ok
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "partition-debug")]
    use super::*;
    #[cfg(feature = "partition-debug")]
    use crate::debug::{DebugRecordHeader, LOG_DEBUG, LOG_ERROR, LOG_INFO, LOG_WARN};

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_framed_record_format() {
        let buffer = DebugRingBuffer::<64>::new();
        let msg = b"test message";

        assert!(debug_write(&buffer, LOG_INFO, msg));
        assert_eq!(buffer.available(), 4 + msg.len());

        let mut out = [0u8; 32];
        let drained = buffer.drain(&mut out, 32);
        assert_eq!(drained, 4 + msg.len());

        let hdr = DebugRecordHeader::from_bytes([out[0], out[1], out[2], out[3]]);
        assert_eq!(
            (hdr.len, hdr.level, hdr.kind, hdr.flags),
            (msg.len() as u8, LOG_INFO, KIND_TEXT, 0)
        );
        assert_eq!(&out[4..4 + msg.len()], msg);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_various_levels_and_edge_cases() {
        // Test all log levels
        let buffer = DebugRingBuffer::<128>::new();
        for &level in &[LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG] {
            assert!(debug_write(&buffer, level, b"lvl"));
        }
        let mut out = [0u8; 128];
        assert_eq!(buffer.drain(&mut out, 128), 28); // 4 records * 7 bytes each
        for (i, &level) in [LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG]
            .iter()
            .enumerate()
        {
            assert_eq!(out[i * 7 + 1], level);
        }

        // Empty message
        let buffer2 = DebugRingBuffer::<32>::new();
        assert!(debug_write(&buffer2, LOG_INFO, b""));
        assert_eq!(buffer2.available(), 4);

        // Max length (255 bytes)
        let buffer3 = DebugRingBuffer::<512>::new();
        let msg = [b'X'; 255];
        assert!(debug_write(&buffer3, LOG_DEBUG, &msg));
        let mut out3 = [0u8; 260];
        assert_eq!(buffer3.drain(&mut out3, 260), 259);
        assert_eq!(&out3[4..259], &msg[..]);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_overflow_returns_false() {
        let buffer = DebugRingBuffer::<16>::new();
        assert!(debug_write(&buffer, LOG_INFO, b"12345678")); // 12 bytes used
        assert!(!debug_write(&buffer, LOG_INFO, b"fail")); // 8 bytes needed, only 4 left
        assert!(buffer.dropped() > 0);
    }
}
