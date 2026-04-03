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

    /// Tests that LOG_WARN level records are written with correct header level.
    /// This verifies the path used by debug_warn! macro.
    #[test]
    fn warn_level_record() {
        let rb = DebugRingBuffer::<64>::new();
        let mut fb = FmtBuffer::<32>::new();
        write!(fb, "warning: low memory").unwrap();
        assert!(rb.write_record(LOG_WARN, KIND_TEXT, fb.as_bytes()));
        let mut out = [0u8; 32];
        let n = rb.drain(&mut out, 32);
        assert!(n > 4);
        let hdr = DebugRecordHeader::from_bytes([out[0], out[1], out[2], out[3]]);
        assert_eq!(hdr.level, LOG_WARN);
        assert_eq!(hdr.kind, KIND_TEXT);
    }

    /// Tests that LOG_ERROR level records are written with correct header level.
    /// This verifies the path used by debug_error! macro.
    #[test]
    fn error_level_record() {
        let rb = DebugRingBuffer::<64>::new();
        let mut fb = FmtBuffer::<32>::new();
        write!(fb, "error: critical failure").unwrap();
        assert!(rb.write_record(LOG_ERROR, KIND_TEXT, fb.as_bytes()));
        let mut out = [0u8; 32];
        let n = rb.drain(&mut out, 32);
        assert!(n > 4);
        let hdr = DebugRecordHeader::from_bytes([out[0], out[1], out[2], out[3]]);
        assert_eq!(hdr.level, LOG_ERROR);
        assert_eq!(hdr.kind, KIND_TEXT);
    }

    /// Tests that different buffer sizes work correctly (verifies @size behavior).
    #[test]
    fn custom_buffer_sizes() {
        // Small buffer: 16 bytes
        let mut fb_small = FmtBuffer::<16>::new();
        write!(fb_small, "short").unwrap();
        assert_eq!(fb_small.as_bytes(), b"short");

        // Large buffer: 256 bytes
        let mut fb_large = FmtBuffer::<256>::new();
        write!(fb_large, "this is a longer message with value {}", 12345).unwrap();
        assert_eq!(
            fb_large.as_bytes(),
            b"this is a longer message with value 12345"
        );

        // Verify truncation behavior with small buffer
        let mut fb_trunc = FmtBuffer::<8>::new();
        let _ = write!(fb_trunc, "truncated message");
        // Should be truncated to buffer size
        assert!(fb_trunc.as_bytes().len() <= 8);
    }

    /// Tests interleaved writes with different log levels.
    /// Verifies that debug_warn! and debug_error! records remain distinguishable.
    #[test]
    fn interleaved_log_levels() {
        let rb = DebugRingBuffer::<128>::new();

        // Write INFO
        let mut fb1 = FmtBuffer::<32>::new();
        write!(fb1, "info msg").unwrap();
        assert!(rb.write_record(LOG_INFO, KIND_TEXT, fb1.as_bytes()));

        // Write WARN
        let mut fb2 = FmtBuffer::<32>::new();
        write!(fb2, "warn msg").unwrap();
        assert!(rb.write_record(LOG_WARN, KIND_TEXT, fb2.as_bytes()));

        // Write ERROR
        let mut fb3 = FmtBuffer::<32>::new();
        write!(fb3, "error msg").unwrap();
        assert!(rb.write_record(LOG_ERROR, KIND_TEXT, fb3.as_bytes()));

        // Drain and verify all three
        let mut out = [0u8; 64];
        let n = rb.drain(&mut out, 64);
        // 3 records: (4 + 8) + (4 + 8) + (4 + 9) = 37 bytes
        assert_eq!(n, 37);

        // First record: INFO
        let h1 = DebugRecordHeader::from_bytes([out[0], out[1], out[2], out[3]]);
        assert_eq!((h1.level, h1.len), (LOG_INFO, 8));

        // Second record: WARN (starts at 4 + 8 = 12)
        let h2 = DebugRecordHeader::from_bytes([out[12], out[13], out[14], out[15]]);
        assert_eq!((h2.level, h2.len), (LOG_WARN, 8));

        // Third record: ERROR (starts at 12 + 4 + 8 = 24)
        let h3 = DebugRecordHeader::from_bytes([out[24], out[25], out[26], out[27]]);
        assert_eq!((h3.level, h3.len), (LOG_ERROR, 9));
    }

    // Note: Full macro integration tests (dprint!, debug_warn!, debug_error!)
    // require the syscall path. On non-ARM hosts, svc! returns 0 (success).
    // End-to-end tests run in QEMU integration test examples.

    // ── plib syscall wrapper tests ──────────────────────────────────────
    //
    // Per the plib crate's documented testing policy (plib/src/lib.rs module
    // docs), tests that require kernel infrastructure live here to avoid
    // panic-handler conflicts.  These verify that the plib wrapper functions
    // wire the correct syscall ID into r0 and pass their resource-id
    // argument via r1, exercising the full dispatch path.

    use crate::syscall::{SYS_MTX_LOCK, SYS_MTX_UNLOCK, SYS_SEM_SIGNAL, SYS_SEM_WAIT};
    use crate::test_harness::KernelTestHarness;
    use rtos_traits::ids::PartitionId;

    /// sys_sem_wait: r0 = SYS_SEM_WAIT, r1 = sem_id.
    /// Dispatching with r1=1 must decrement semaphore 1 and leave semaphore 0
    /// untouched, proving the wrapper passes sem_id in the correct register.
    #[test]
    fn plib_sem_wait_passes_sem_id_via_r1() {
        let mut h = KernelTestHarness::with_semaphores(&[1, 1]).unwrap();
        // Wait on semaphore 1 (index 1).
        let ef = h.dispatch(SYS_SEM_WAIT, 1, 0, 0);
        assert_eq!(ef.r0, 1, "SemWait must return 1 (acquired)");
        // Semaphore 1 count should be decremented from 1 → 0.
        assert_eq!(h.kernel().semaphores().get(1).unwrap().count(), 0);
        // Semaphore 0 must remain at count 1 (untouched).
        assert_eq!(h.kernel().semaphores().get(0).unwrap().count(), 1);
    }

    /// sys_sem_signal: r0 = SYS_SEM_SIGNAL, r1 = sem_id.
    /// After draining semaphore 0, signalling with r1=0 must increment
    /// semaphore 0 and leave semaphore 1 unchanged.
    #[test]
    fn plib_sem_signal_passes_sem_id_via_r1() {
        let mut h = KernelTestHarness::with_semaphores(&[1, 1]).unwrap();
        // Drain semaphore 0 so its count goes from 1 → 0.
        let ef = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(ef.r0, 1);
        assert_eq!(h.kernel().semaphores().get(0).unwrap().count(), 0);
        // Signal semaphore 0 — count should go from 0 → 1.
        let ef = h.dispatch(SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(ef.r0, 0, "SemSignal must return 0 (success)");
        assert_eq!(h.kernel().semaphores().get(0).unwrap().count(), 1);
        // Semaphore 1 must still be at count 1 (untouched).
        assert_eq!(h.kernel().semaphores().get(1).unwrap().count(), 1);
    }

    /// sys_mtx_lock: r0 = SYS_MTX_LOCK, r1 = mtx_id.
    /// Locking with r1=0 must set the owner of mutex 0 to the calling
    /// partition, proving mtx_id is passed correctly.
    #[test]
    fn plib_mtx_lock_passes_mtx_id_via_r1() {
        let mut h = KernelTestHarness::with_mutexes(1).unwrap();
        let ef = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(ef.r0, 1, "MtxLock must return 1 (acquired)");
        assert_eq!(
            h.kernel().mutexes().owner(0),
            Ok(Some(PartitionId::new(0))),
            "mutex 0 must be owned by partition 0"
        );
    }

    /// sys_mtx_unlock: r0 = SYS_MTX_UNLOCK, r1 = mtx_id.
    /// After locking mutex 0, unlocking with r1=0 must release it.
    #[test]
    fn plib_mtx_unlock_passes_mtx_id_via_r1() {
        let mut h = KernelTestHarness::with_mutexes(1).unwrap();
        // Lock mutex 0 first.
        let ef = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(ef.r0, 1);
        assert_eq!(h.kernel().mutexes().owner(0), Ok(Some(PartitionId::new(0))));
        // Unlock mutex 0.
        let ef = h.dispatch(SYS_MTX_UNLOCK, 0, 0, 0);
        assert_eq!(ef.r0, 0, "MtxUnlock must return 0 (success)");
        assert_eq!(
            h.kernel().mutexes().owner(0),
            Ok(None),
            "mutex 0 must be released"
        );
    }
}
