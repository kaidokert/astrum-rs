//! Partition library: user-space helpers for partitions.
//!
//! This crate provides convenient wrappers around kernel syscalls for use
//! by partition code running in unprivileged (user) mode.
//!
//! # Testing
//!
//! Tests for this crate live in `kernel/src/debug.rs` to avoid panic handler
//! conflicts between std (used in tests) and kernel's optional panic-halt dependency.

#![no_std]

use kernel::api::decode_rc;
pub use kernel::api::SvcError;
use kernel::syscall::{
    SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_GET_TIME, SYS_IRQ_ACK, SYS_YIELD,
};

#[cfg(feature = "partition-debug")]
use rtos_traits::debug::{DebugRingBuffer, KIND_TEXT};
#[cfg(feature = "partition-debug")]
use rtos_traits::syscall::SYS_DEBUG_NOTIFY;

// Re-export FmtBuffer from shared traits crate for use with dprint! macro
#[cfg(feature = "partition-debug")]
pub use rtos_traits::fmt::FmtBuffer;

// Re-export DebugRingBuffer for use with define_partition_debug! macro
#[cfg(feature = "partition-debug")]
#[doc(hidden)]
pub use rtos_traits::debug::DebugRingBuffer as __DebugRingBuffer;

/// Define a static debug ring buffer for a partition.
///
/// This macro declares a static [`DebugRingBuffer`] with the specified size,
/// suitable for use with [`dprint!`] and kernel debug output draining.
///
/// # Usage
///
/// ```ignore
/// // Define a 256-byte debug buffer named DEBUG_BUF
/// define_partition_debug!(DEBUG_BUF, 256);
///
/// // Use it with dprint!
/// dprint!(&DEBUG_BUF, "value = {}", 42);
/// ```
///
/// # Requirements
///
/// - `SIZE` must be a power of 2 (enforced at compile time)
/// - The buffer is initialized to empty state
/// - The buffer has correct alignment for atomic operations
///
/// # Linkage
///
/// The generated static has internal linkage. To register it with the kernel,
/// pass a reference to `PartitionControlBlock::set_debug_buffer` during
/// partition initialization.
#[cfg(feature = "partition-debug")]
#[macro_export]
macro_rules! define_partition_debug {
    ($name:ident, $size:expr) => {
        // Compile-time assertion: SIZE must be a power of 2
        const _: () = {
            assert!(
                $size > 0 && ($size & ($size - 1)) == 0,
                "define_partition_debug!: SIZE must be a power of 2"
            );
        };
        // TODO: Linkage is currently private. If partition harness or kernel needs
        // to discover this buffer via linker symbols, add #[no_mangle] and pub.
        #[allow(dead_code)]
        static $name: $crate::__DebugRingBuffer<$size> = $crate::__DebugRingBuffer::new();
    };
}

/// Format and write a debug message to the partition's debug ring buffer.
///
/// # Forms
/// - `dprint!(buffer, "format", args...)` - uses default 128-byte stack buffer
/// - `dprint!(buffer, @size SIZE, "format", args...)` - uses SIZE-byte stack buffer
///
/// # Stack Safety
/// The default 128-byte buffer is chosen as a reasonable balance between message
/// length and stack usage. For very constrained partitions, use the explicit size form.
///
/// # Truncation Behavior
/// Messages that exceed the buffer size are silently truncated. This is intentional
/// for best-effort debug logging where partial output is preferable to panicking.
// TODO: Consider making the default size configurable via KernelConfig trait
// for system-wide stack budget control.
#[cfg(feature = "partition-debug")]
#[macro_export]
macro_rules! dprint {
    // Form with explicit buffer size (uses @size keyword to disambiguate from format string)
    ($buf:expr, @size $size:literal, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_INFO, @size $size, $($arg)*)
    };
    // Form with default 128-byte buffer
    ($buf:expr, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_INFO, $($arg)*)
    };
}

#[cfg(feature = "partition-debug")]
#[doc(hidden)]
pub use rtos_traits::debug::LOG_ERROR as __LOG_ERROR;
#[cfg(feature = "partition-debug")]
#[doc(hidden)]
pub use rtos_traits::debug::LOG_INFO as __LOG_INFO;
#[cfg(feature = "partition-debug")]
#[doc(hidden)]
pub use rtos_traits::debug::LOG_WARN as __LOG_WARN;

/// Internal macro that implements the common formatting and writing logic.
/// Used by `dprint!`, `debug_warn!`, and `debug_error!` to avoid code duplication.
#[cfg(feature = "partition-debug")]
#[doc(hidden)]
#[macro_export]
macro_rules! __debug_write_impl {
    ($buf:expr, $level:expr, @size $size:literal, $($arg:tt)*) => {{
        use core::fmt::Write;
        let mut fmt_buf = $crate::FmtBuffer::<$size>::new();
        // Silent truncation is intentional for best-effort debug logging
        let _ = write!(fmt_buf, $($arg)*);
        $crate::debug_write($buf, $level, fmt_buf.as_bytes())
    }};
    ($buf:expr, $level:expr, $($arg:tt)*) => {{
        use core::fmt::Write;
        let mut fmt_buf = $crate::FmtBuffer::<128>::new();
        // Silent truncation is intentional for best-effort debug logging
        let _ = write!(fmt_buf, $($arg)*);
        $crate::debug_write($buf, $level, fmt_buf.as_bytes())
    }};
}

/// Format and write a warning-level debug message to the partition's debug ring buffer.
///
/// Similar to [`dprint!`] but uses `LOG_WARN` level instead of `LOG_INFO`.
///
/// # Forms
/// - `debug_warn!(buffer, "format", args...)` - uses default 128-byte stack buffer
/// - `debug_warn!(buffer, @size SIZE, "format", args...)` - uses SIZE-byte stack buffer
#[cfg(feature = "partition-debug")]
#[macro_export]
macro_rules! debug_warn {
    ($buf:expr, @size $size:literal, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_WARN, @size $size, $($arg)*)
    };
    ($buf:expr, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_WARN, $($arg)*)
    };
}

/// Format and write an error-level debug message to the partition's debug ring buffer.
///
/// Similar to [`dprint!`] but uses `LOG_ERROR` level instead of `LOG_INFO`.
///
/// # Forms
/// - `debug_error!(buffer, "format", args...)` - uses default 128-byte stack buffer
/// - `debug_error!(buffer, @size SIZE, "format", args...)` - uses SIZE-byte stack buffer
#[cfg(feature = "partition-debug")]
#[macro_export]
macro_rules! debug_error {
    ($buf:expr, @size $size:literal, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_ERROR, @size $size, $($arg)*)
    };
    ($buf:expr, $($arg:tt)*) => {
        $crate::__debug_write_impl!($buf, $crate::__LOG_ERROR, $($arg)*)
    };
}

/// Error type for debug_write operations.
#[cfg(feature = "partition-debug")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugWriteError {
    /// The ring buffer overflowed and the message was dropped.
    BufferOverflow,
    /// The kernel syscall failed with the given error code.
    SyscallFailed(u32),
}

/// Write a debug message to the ring buffer and notify the kernel.
///
/// Writes a framed text record and invokes `SYS_DEBUG_NOTIFY` to signal pending output.
///
/// # Returns
///
/// - `Ok(())` on success (message written and kernel notified)
/// - `Err(DebugWriteError::BufferOverflow)` if the buffer overflowed (message dropped,
///   but kernel was still notified to drain any existing data)
/// - `Err(DebugWriteError::SyscallFailed(code))` if the syscall returned an error
#[cfg(feature = "partition-debug")]
pub fn debug_write<const N: usize>(
    buffer: &DebugRingBuffer<N>,
    level: u8,
    msg: &[u8],
) -> Result<(), DebugWriteError> {
    let write_ok = buffer.write_record(level, KIND_TEXT, msg);

    // Notify kernel even on overflow to drain any buffered data
    let syscall_result = kernel::svc!(SYS_DEBUG_NOTIFY, 0u32, 0u32, 0u32);

    // Check syscall return value per mandate
    if SvcError::is_error(syscall_result) {
        return Err(DebugWriteError::SyscallFailed(syscall_result));
    }

    if write_ok {
        Ok(())
    } else {
        Err(DebugWriteError::BufferOverflow)
    }
}

/// Block the calling partition until at least one event in `mask` is pending.
///
/// The kernel clears the matched bits atomically before returning.
///
/// # Returns
///
/// `Ok(bits)` with the bitmask of events that were pending, or
/// `Err(SvcError)` if the syscall failed.
pub fn sys_event_wait(mask: u32) -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_EVT_WAIT, mask, 0u32, 0u32))
}

/// Set event bits on another partition.
///
/// Delivers `mask` to the target partition's pending-event word, potentially
/// waking it from an `sys_event_wait` call.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_event_set(target_partition: u32, mask: u32) -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_EVT_SET, target_partition, mask, 0u32))
}

/// Clear event bits in the calling partition's pending-event word.
///
/// Atomically clears the bits specified in `mask`.
///
/// # Returns
///
/// `Ok(prev)` with the previous pending-event word value, or
/// `Err(SvcError)` if the syscall failed.
pub fn sys_event_clear(mask: u32) -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_EVT_CLEAR, mask, 0u32, 0u32))
}

/// Acknowledge a hardware IRQ after the partition has handled it.
///
/// This re-enables the IRQ in the NVIC so subsequent interrupts can be
/// delivered.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_irq_ack(irq_num: u8) -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_IRQ_ACK, irq_num as u32, 0u32, 0u32))
}

/// Yield the calling partition's remaining time slice.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_yield() -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32))
}

/// Get the current kernel tick count.
///
/// # Returns
///
/// `Ok(ticks)` with the current tick count, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_get_time() -> Result<u32, SvcError> {
    decode_rc(kernel::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_wait_returns_ok_zero_on_host() {
        assert_eq!(sys_event_wait(0x01), Ok(0));
    }

    #[test]
    fn event_set_returns_ok_zero_on_host() {
        assert_eq!(sys_event_set(1, 0xFF), Ok(0));
    }

    #[test]
    fn event_clear_returns_ok_zero_on_host() {
        assert_eq!(sys_event_clear(0x03), Ok(0));
    }

    #[test]
    fn irq_ack_returns_ok_zero_on_host() {
        assert_eq!(sys_irq_ack(5), Ok(0));
    }

    #[test]
    fn yield_returns_ok_zero_on_host() {
        assert_eq!(sys_yield(), Ok(0));
    }

    #[test]
    fn get_time_returns_ok_zero_on_host() {
        assert_eq!(sys_get_time(), Ok(0));
    }
}
