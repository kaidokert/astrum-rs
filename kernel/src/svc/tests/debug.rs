// -------------------------------------------------------------------------
// Debug dispatch and drain tests
// -------------------------------------------------------------------------

#[cfg(feature = "partition-debug")]
use super::*;

#[cfg(feature = "partition-debug")]
#[test]
fn debug_notify_sets_debug_pending_flag() {
    use crate::syscall::SYS_DEBUG_NOTIFY;
    let mut k = kernel(0, 0, 0);
    // Verify debug_pending is initially false
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    // Dispatch SYS_DEBUG_NOTIFY syscall
    let mut ef = frame(SYS_DEBUG_NOTIFY, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    // Should return 0 on success
    assert_eq!(ef.r0, 0);
    // debug_pending flag should now be set on current partition (0)
    assert!(k.partitions().get(0).unwrap().debug_pending());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_pending_iterates_and_clears() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};
    use crate::partition_debug::DrainContext;

    static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
    static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

    let mut k = kernel(0, 0, 0);

    // Set up debug buffers on both partitions
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF0);
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .set_debug_buffer(&BUF1);

    // Write records to both buffers
    BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
    BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");

    // Signal debug pending on both partitions
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .signal_debug_pending();

    // Verify both flags are set
    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(k.partitions().get(1).unwrap().debug_pending());

    // Drain with sufficient budget
    let mut ctx = DrainContext::new();
    let drained = k.drain_debug_pending(&mut ctx, 256);

    // Should have drained records from both partitions (4 + 2 bytes each)
    assert_eq!(drained, 12);

    // Both debug_pending flags should be cleared
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(!k.partitions().get(1).unwrap().debug_pending());

    // Buffers should be empty
    assert!(BUF0.is_empty());
    assert!(BUF1.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_pending_skips_non_pending() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};
    use crate::partition_debug::DrainContext;

    static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
    static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

    let mut k = kernel(0, 0, 0);

    // Set up debug buffers on both partitions
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF0);
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .set_debug_buffer(&BUF1);

    // Write records to both buffers but only signal partition 1
    BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
    BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .signal_debug_pending();

    // Only partition 1 has debug_pending set
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(k.partitions().get(1).unwrap().debug_pending());

    let mut ctx = DrainContext::new();
    let drained = k.drain_debug_pending(&mut ctx, 256);

    // Should only drain partition 1's buffer (4 + 2 = 6 bytes)
    assert_eq!(drained, 6);

    // Partition 0's buffer should still have data
    assert!(!BUF0.is_empty());
    // Partition 1's buffer should be empty
    assert!(BUF1.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_auto_drains_pending_output() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
    let mut k = kernel(0, 0, 0);

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);
    BUF.write_record(LOG_INFO, KIND_TEXT, b"hello");
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();

    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(!BUF.is_empty());

    k.drain_debug_auto();

    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(BUF.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_auto_budget_zero_is_noop() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    struct NoDrainConfig;
    impl KernelConfig for NoDrainConfig {
        const N: usize = 4;
        const S: usize = 4;
        const SW: usize = 4;
        const MS: usize = 4;
        const MW: usize = 4;
        const QS: usize = 4;
        const QD: usize = 4;
        const QM: usize = 4;
        const QW: usize = 4;
        const SP: usize = 4;
        const BS: usize = 4;
        const BW: usize = 4;
        const DEBUG_AUTO_DRAIN_BUDGET: usize = 0;
        const BP: usize = 4;

        kernel_config_types!();
    }

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
    let mut k: Kernel<'static, NoDrainConfig> = Kernel::default();
    k.partitions_mut().add(pcb(0)).unwrap();

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);
    BUF.write_record(LOG_INFO, KIND_TEXT, b"stay");
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();

    assert!(k.partitions().get(0).unwrap().debug_pending());

    k.drain_debug_auto();

    // Budget is 0 so buffer must remain untouched.
    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(!BUF.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn systick_handler_drains_debug_output() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
    let mut k = kernel(0, 0, 0);

    // Set up debug buffer on partition 0
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);
    BUF.write_record(LOG_INFO, KIND_TEXT, b"test");
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();
    assert!(k.partitions().get(0).unwrap().debug_pending());

    // Call systick_handler - should drain debug output
    crate::tick::systick_handler::<TestConfig>(&mut k);

    // debug_pending should be cleared and buffer empty
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(BUF.is_empty());
}

/// Mirrors the harness yield closure: yield_current_slot then
/// drain_debug_auto clears pending debug output.
#[cfg(feature = "partition-debug")]
#[test]
fn yield_then_drain_debug_auto_clears_pending() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
    let mut k = kernel_with_schedule();

    // Attach debug buffer to partition 0 and write a record.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);
    BUF.write_record(LOG_INFO, KIND_TEXT, b"yield-drain");
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();

    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(!BUF.is_empty());

    // Execute the same sequence as the harness yield closure:
    // yield_current_slot followed by drain_debug_auto.
    let result = k.yield_current_slot();
    assert!(result.partition_id().is_some());
    k.drain_debug_auto();

    // Debug buffer must be fully drained.
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(BUF.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_auto_multi_partition_clears_all() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
    static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

    let mut k = kernel(0, 0, 0);

    // Attach debug buffers to both partitions.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF0);
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .set_debug_buffer(&BUF1);

    // Write distinct records so we can verify per-partition drain.
    BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
    BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");

    // Signal pending on both partitions.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .signal_debug_pending();

    // Pre-conditions: both pending, both non-empty.
    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(k.partitions().get(1).unwrap().debug_pending());
    assert!(!BUF0.is_empty());
    assert!(!BUF1.is_empty());

    // Exercise the auto-drain entry point (not drain_debug_pending directly).
    k.drain_debug_auto();

    // Both debug_pending flags must be cleared.
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(!k.partitions().get(1).unwrap().debug_pending());

    // Both buffers must be fully drained.
    assert!(BUF0.is_empty());
    assert!(BUF1.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn drain_debug_auto_idempotent_second_call_noop() {
    use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();

    let mut k = kernel(0, 0, 0);

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);
    BUF.write_record(LOG_INFO, KIND_TEXT, b"once");
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .signal_debug_pending();

    assert!(k.partitions().get(0).unwrap().debug_pending());
    assert!(!BUF.is_empty());

    // First call drains the buffer and clears pending.
    k.drain_debug_auto();
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(BUF.is_empty());

    // Second call is a no-op: pending is already false so nothing runs.
    k.drain_debug_auto();
    assert!(!k.partitions().get(0).unwrap().debug_pending());
    assert!(BUF.is_empty());
}

#[cfg(feature = "partition-debug")]
#[test]
fn debug_write_rejects_invalid_pointer() {
    use crate::syscall::SYS_DEBUG_WRITE;
    let mut k = kernel(0, 0, 0);
    let mut ef = frame(SYS_DEBUG_WRITE, 0xDEAD_BEEF, 5);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
}

/// Allocate a page at a fixed low address for partition-debug tests.
/// Page 0 corresponds to partition 0's MPU data region at 0x2000_0000.
#[cfg(feature = "partition-debug")]
fn debug_test_buf(page: usize) -> *mut u8 {
    extern "C" {
        fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
    }
    // Partition 0 has MPU data region at 0x2000_0000 with size 4096.
    // Use page 0 for all partition-debug tests (shared with dynamic-mpu tests).
    let _ = page; // All tests use partition 0's region.
    let addr = 0x2000_0000_usize;
    // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
    // low address. The mapping is intentionally leaked (test-only).
    let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
    assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
    ptr
}

#[cfg(feature = "partition-debug")]
#[test]
fn debug_write_success_writes_data_to_buffer() {
    use crate::debug::DebugRingBuffer;
    use crate::syscall::SYS_DEBUG_WRITE;

    static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
    let mut k = kernel(0, 0, 0);

    // Configure debug buffer for partition 0.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);

    // Write a known pattern into partition memory.
    let ptr = debug_test_buf(0);
    let pattern = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE];
    // SAFETY: `ptr` is a valid mmap'd page from `debug_test_buf`, `pattern`
    // is a stack-local array, and `pattern.len()` (6) fits within the 4096-byte page.
    unsafe { core::ptr::copy_nonoverlapping(pattern.as_ptr(), ptr, pattern.len()) };

    // Invoke SYS_DEBUG_WRITE with ptr and len.
    let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, pattern.len() as u32);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };

    // Verify success: r0 == bytes written.
    assert_eq!(ef.r0, pattern.len() as u32);

    // Verify data integrity: drain buffer and compare.
    let mut out = [0u8; 64];
    let n = BUF.drain(&mut out, 64);
    assert_eq!(n, pattern.len());
    assert_eq!(&out[..n], &pattern);
}

#[cfg(feature = "partition-debug")]
#[test]
fn debug_write_returns_not_supported_when_no_buffer() {
    use crate::syscall::SYS_DEBUG_WRITE;

    let mut k = kernel(0, 0, 0);
    // Do NOT set a debug buffer.

    let ptr = debug_test_buf(1);
    let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, 4);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };

    assert_eq!(ef.r0, SvcError::NotSupported.to_u32());
}

#[cfg(feature = "partition-debug")]
#[test]
fn debug_write_returns_zero_on_overflow() {
    use crate::debug::DebugRingBuffer;
    use crate::syscall::SYS_DEBUG_WRITE;

    // Small buffer that will overflow.
    static BUF: DebugRingBuffer<16> = DebugRingBuffer::new();
    let mut k = kernel(0, 0, 0);

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_debug_buffer(&BUF);

    // Try to write more than the buffer can hold — returns 0 (no bytes written).
    let ptr = debug_test_buf(2);
    let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, 32);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };

    assert_eq!(ef.r0, 0);
}
