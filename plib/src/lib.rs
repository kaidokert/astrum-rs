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

pub use rtos_traits::api::decode_rc;
pub use rtos_traits::api::SvcError;

/// Status information for a queuing port, returned by [`sys_queuing_status`].
///
/// Layout must match the kernel's `QueuingPortStatus` (both are `#[repr(C)]`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueuingPortStatus {
    pub nb_messages: u32,
    pub max_nb_messages: u32,
    pub max_message_size: u32,
    /// 0 = Source, 1 = Destination.
    pub direction: u32,
}

#[cfg(feature = "dynamic-mpu")]
pub use rtos_traits::buf_syscall;

// ── Re-exported syscall constants (from rtos-traits) ──────────────────

// Control
pub use rtos_traits::syscall::{SYS_GET_PARTITION_ID, SYS_GET_TIME, SYS_IRQ_ACK, SYS_YIELD};
// Events
pub use rtos_traits::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT};
// Sync
pub use rtos_traits::syscall::{SYS_MTX_LOCK, SYS_MTX_UNLOCK, SYS_SEM_SIGNAL, SYS_SEM_WAIT};
// Messaging
pub use rtos_traits::syscall::{SYS_MSG_RECV, SYS_MSG_SEND};
// Ports
pub use rtos_traits::syscall::{
    SYS_QUEUING_RECV, SYS_QUEUING_RECV_TIMED, SYS_QUEUING_SEND, SYS_QUEUING_SEND_TIMED,
    SYS_QUEUING_STATUS, SYS_SAMPLING_READ, SYS_SAMPLING_WRITE,
};
// Device driver & query (dynamic-mpu only, defined in rtos-traits)
#[cfg(feature = "dynamic-mpu")]
pub use rtos_traits::syscall::{
    SYS_DEV_CLOSE, SYS_DEV_IOCTL, SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_READ_TIMED, SYS_DEV_WRITE,
    SYS_QUERY_BOTTOM_HALF,
};
// Buffer pool (dynamic-mpu only, defined in rtos-traits)
#[cfg(feature = "dynamic-mpu")]
pub use rtos_traits::syscall::lend_flags;
#[cfg(feature = "dynamic-mpu")]
pub use rtos_traits::syscall::{
    SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_READ, SYS_BUF_RELEASE, SYS_BUF_REVOKE, SYS_BUF_TRANSFER,
    SYS_BUF_WRITE,
};
// Blackboard
#[cfg(feature = "ipc-blackboard")]
pub use rtos_traits::syscall::{SYS_BB_CLEAR, SYS_BB_DISPLAY, SYS_BB_READ};
// Debug – SYS_DEBUG_PRINT/EXIT are semihosting ops (unconditional in rtos-traits);
// SYS_DEBUG_NOTIFY/WRITE are partition ring-buffer ops (gated on partition-debug).
pub use rtos_traits::syscall::{SYS_DEBUG_EXIT, SYS_DEBUG_PRINT};
#[cfg(feature = "partition-debug")]
pub use rtos_traits::syscall::{SYS_DEBUG_NOTIFY, SYS_DEBUG_WRITE};

// ── Re-exported resource ID newtypes (from rtos-traits) ─────────────
#[cfg(feature = "ipc-blackboard")]
pub use rtos_traits::ids::BlackboardId;
#[cfg(feature = "dynamic-mpu")]
pub use rtos_traits::ids::DeviceId;
pub use rtos_traits::ids::{
    EventMask, MutexId, PartitionId, QueuingPortId, SamplingPortId, SemaphoreId,
};

#[cfg(feature = "partition-debug")]
use rtos_traits::debug::{DebugRingBuffer, KIND_TEXT};

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
    let syscall_result = rtos_traits::svc!(SYS_DEBUG_NOTIFY, 0u32, 0u32, 0u32);

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
pub fn sys_event_wait(mask: EventMask) -> Result<EventMask, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_EVT_WAIT, 0u32, mask.as_raw(), 0u32)).map(EventMask::new)
}

/// Set event bits on another partition.
///
/// Delivers `mask` to the target partition's pending-event word, potentially
/// waking it from an `sys_event_wait` call.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_event_set(target_partition: PartitionId, mask: EventMask) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_EVT_SET,
        target_partition.as_raw(),
        mask.as_raw(),
        0u32
    ))
}

/// Clear event bits in the calling partition's pending-event word.
///
/// Atomically clears the bits specified in `mask`.
///
/// # Returns
///
/// `Ok(prev)` with the previous pending-event word value, or
/// `Err(SvcError)` if the syscall failed.
pub fn sys_event_clear(mask: EventMask) -> Result<EventMask, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_EVT_CLEAR, 0u32, mask.as_raw(), 0u32)).map(EventMask::new)
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
    decode_rc(rtos_traits::svc!(SYS_IRQ_ACK, irq_num as u32, 0u32, 0u32))
}

/// Yield the calling partition's remaining time slice.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_yield() -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_YIELD, 0u32, 0u32, 0u32))
}

/// Get the partition ID of the calling partition.
///
/// # Returns
///
/// `Ok(id)` with the caller's partition index, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_get_partition_id() -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_GET_PARTITION_ID, 0u32, 0u32, 0u32))
}

/// Get the current kernel tick count.
///
/// # Returns
///
/// `Ok(ticks)` with the current tick count, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_get_time() -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32))
}

/// Wait (decrement) on a semaphore.
///
/// If the semaphore count is greater than zero it is decremented and the
/// call returns immediately.  Otherwise the calling partition blocks until
/// another partition signals the semaphore.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_sem_wait(sem_id: SemaphoreId) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_SEM_WAIT, sem_id.as_raw(), 0u32, 0u32))
}

/// Signal (increment) a semaphore.
///
/// Increments the semaphore count and wakes one waiting partition, if any.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_sem_signal(sem_id: SemaphoreId) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_SEM_SIGNAL,
        sem_id.as_raw(),
        0u32,
        0u32
    ))
}

/// Lock a mutex.
///
/// If the mutex is free it is acquired by the calling partition.  If it is
/// already held by another partition the caller blocks until it is released.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_mtx_lock(mtx_id: MutexId) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_MTX_LOCK, mtx_id.as_raw(), 0u32, 0u32))
}

/// Unlock a mutex.
///
/// Releases the mutex and wakes one waiting partition, if any.  Only the
/// partition that holds the mutex may unlock it.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_mtx_unlock(mtx_id: MutexId) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_MTX_UNLOCK,
        mtx_id.as_raw(),
        0u32,
        0u32
    ))
}

/// Write data to a sampling port.
///
/// Copies `data` into the sampling port identified by `port_id`.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_sampling_write(port_id: u32, data: &[u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_SAMPLING_WRITE,
        port_id,
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Read data from a sampling port.
///
/// Copies the current message from the sampling port identified by
/// `port_id` into `buf`.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes read, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_sampling_read(port_id: u32, buf: &mut [u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_SAMPLING_READ,
        port_id,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    ))
}

/// Send a message to another partition.
///
/// Copies `data` into the target partition's message buffer.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_msg_send(target_partition: PartitionId, data: &[u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_MSG_SEND,
        target_partition.as_raw(),
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Send a message to a queuing port.
///
/// Copies `data` into the queuing port identified by `port_id`.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_queuing_send(port_id: u32, data: &[u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_QUEUING_SEND,
        port_id,
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Receive a message from a queuing port.
///
/// Copies the next queued message from `port_id` into `buf`.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes received, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_queuing_recv(port_id: u32, buf: &mut [u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_QUEUING_RECV,
        port_id,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    ))
}

/// Send a message to a queuing port with a timeout.
///
/// ABI: r2 = (timeout_ticks_hi16 << 16 | data_len_lo16), r3 = data_ptr.
///
/// # Returns
///
/// `Ok(0)` on success, `Err(SvcError::InvalidParameter)` if `data` exceeds
/// 65 535 bytes, or `Err(SvcError)` if the syscall failed.
pub fn sys_queuing_send_timed(
    port_id: u32,
    data: &[u8],
    timeout_ticks: u16,
) -> Result<u32, SvcError> {
    if data.len() > u16::MAX as usize {
        return Err(SvcError::InvalidParameter);
    }
    let r2 = ((timeout_ticks as u32) << 16) | (data.len() as u32);
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The data pointer is valid for data.len() bytes and the
    // packed r2 encodes the length so the kernel can bounds-check it.
    decode_rc(rtos_traits::svc!(
        SYS_QUEUING_SEND_TIMED,
        port_id,
        r2,
        data.as_ptr() as u32
    ))
}

/// Receive a message from a queuing port with a timeout.
///
/// ABI: r2 = (timeout_ticks_hi16 << 16 | buf_len_lo16), r3 = buf_ptr.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes received, `Err(SvcError::InvalidParameter)`
/// if `buf` exceeds 65 535 bytes, or `Err(SvcError)` if the syscall failed.
pub fn sys_queuing_recv_timed(
    port_id: u32,
    buf: &mut [u8],
    timeout_ticks: u16,
) -> Result<u32, SvcError> {
    if buf.len() > u16::MAX as usize {
        return Err(SvcError::InvalidParameter);
    }
    let r2 = ((timeout_ticks as u32) << 16) | (buf.len() as u32);
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The buf pointer is valid for buf.len() bytes and the
    // packed r2 encodes the length so the kernel can bounds-check it.
    decode_rc(rtos_traits::svc!(
        SYS_QUEUING_RECV_TIMED,
        port_id,
        r2,
        buf.as_mut_ptr() as u32
    ))
}

/// Query the status of a queuing port.
///
/// The kernel writes the full [`QueuingPortStatus`] struct to a caller-provided
/// buffer via the `r2` pointer register.
///
/// # Returns
///
/// `Ok(status)` with the full port status, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_queuing_status(port_id: u32) -> Result<QueuingPortStatus, SvcError> {
    let mut status = core::mem::MaybeUninit::<QueuingPortStatus>::zeroed();
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The pointer is valid for writes of size_of::<QueuingPortStatus>()
    // and the kernel writes the full struct before returning 0.
    let rc = rtos_traits::svc!(
        SYS_QUEUING_STATUS,
        port_id,
        status.as_mut_ptr() as u32,
        0u32
    );
    decode_rc(rc)?;
    // SAFETY: On success (rc == 0) the kernel has fully initialised the struct.
    Ok(unsafe { status.assume_init() })
}

/// Receive a message from another partition.
///
/// Copies the pending message into `buf`.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes received, or `Err(SvcError)` if the
/// syscall failed.
pub fn sys_msg_recv(buf: &mut [u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_MSG_RECV,
        0u32,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    ))
}

/// Display (write) data to a blackboard.
///
/// Copies `data` into the blackboard identified by `board_id`, replacing
/// any previously displayed value and waking all blocked readers.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "ipc-blackboard")]
pub fn sys_bb_display(board_id: u32, data: &[u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_BB_DISPLAY,
        board_id,
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Read the current value from a blackboard.
///
/// Copies the blackboard contents into `buf`.  This is a non-blocking
/// read; if the blackboard is empty, the kernel returns an error.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes read, or `Err(SvcError)` if the
/// syscall failed.
#[cfg(feature = "ipc-blackboard")]
pub fn sys_bb_read(board_id: u32, buf: &mut [u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_BB_READ,
        board_id,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    ))
}

/// Clear a blackboard, removing any displayed data.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "ipc-blackboard")]
pub fn sys_bb_clear(board_id: u32) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_BB_CLEAR, board_id, 0u32, 0u32))
}

/// Open a device by its ID.
///
/// ABI: r0 = SYS_DEV_OPEN, r1 = device_id.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_open(device_id: u8) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_DEV_OPEN,
        device_id as u32,
        0u32,
        0u32
    ))
}

/// Close a previously opened device.
///
/// ABI: r0 = SYS_DEV_CLOSE, r1 = device_id.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_close(device_id: u8) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_DEV_CLOSE,
        device_id as u32,
        0u32,
        0u32
    ))
}

/// Send an I/O control command to a device.
///
/// ABI: r0 = SYS_DEV_IOCTL, r1 = device_id, r2 = cmd, r3 = arg.
///
/// # Returns
///
/// `Ok(value)` with a device-specific result, or `Err(SvcError)` if the
/// syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_ioctl(device_id: u8, cmd: u32, arg: u32) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_DEV_IOCTL, device_id as u32, cmd, arg))
}

/// Read data from a device into `buf`.
///
/// ABI: r1 = device_id, r2 = buf_len, r3 = buf_ptr.
/// Returns `Ok(n)` bytes read or `Err(SvcError)`.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_read(device_id: u8, buf: &mut [u8]) -> Result<u32, SvcError> {
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The buf pointer is valid for buf.len() bytes and the
    // kernel uses the length to bounds-check the write into user memory.
    decode_rc(rtos_traits::svc!(
        SYS_DEV_READ,
        device_id as u32,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    ))
}

/// Write data to a device.
///
/// ABI: r1 = device_id, r2 = data_len, r3 = data_ptr.
/// Returns `Ok(0)` on success or `Err(SvcError)`.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_write(device_id: u8, data: &[u8]) -> Result<u32, SvcError> {
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The data pointer is valid for data.len() bytes and the
    // kernel uses the length to bounds-check the read from user memory.
    decode_rc(rtos_traits::svc!(
        SYS_DEV_WRITE,
        device_id as u32,
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Read data from a device with a timeout.
///
/// ABI: r1 = device_id, r2 = (timeout_ticks_hi16 << 16 | buf_len_lo16), r3 = buf_ptr.
/// Returns `Ok(n)` bytes read, `Err(OperationFailed)` if buf > 65 535 bytes.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_dev_read_timed(
    device_id: u8,
    buf: &mut [u8],
    timeout_ticks: u16,
) -> Result<u32, SvcError> {
    if buf.len() > u16::MAX as usize {
        return Err(SvcError::InvalidParameter);
    }
    let r2 = ((timeout_ticks as u32) << 16) | (buf.len() as u32);
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The buf pointer is valid for buf.len() bytes and the
    // packed r2 encodes the length so the kernel can bounds-check it.
    decode_rc(rtos_traits::svc!(
        SYS_DEV_READ_TIMED,
        device_id as u32,
        r2,
        buf.as_mut_ptr() as u32
    ))
}

/// Query whether a bottom-half handler is pending for a device.
///
/// ABI: r1 = device_id. Returns `Ok(status)` or `Err(SvcError)`.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_query_bottom_half(device_id: u8) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_QUERY_BOTTOM_HALF,
        device_id as u32,
        0u32,
        0u32
    ))
}

/// Allocate a buffer slot from the shared buffer pool.
///
/// `writable`: request a writable (`true`) or read-only (`false`) borrow.
/// `max_ticks`: deadline in ticks (0 = no deadline).
///
/// ABI: r1 = mode (0 read-only, 1 writable), r2 = max_ticks.
///
/// # Returns
///
/// `Ok(slot)` with the allocated slot index, or `Err(SvcError)` if the
/// syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_alloc(writable: bool, max_ticks: u16) -> Result<u32, SvcError> {
    let mode = if writable { 1u32 } else { 0u32 };
    decode_rc(rtos_traits::svc!(
        SYS_BUF_ALLOC,
        mode,
        max_ticks as u32,
        0u32
    ))
}

/// Release a buffer slot back to the shared pool.
///
/// ABI: r1 = slot.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_release(slot: u8) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_BUF_RELEASE, slot as u32, 0u32, 0u32))
}

/// Read data from a buffer slot into `dst`.
///
/// ABI: r1 = slot, r2 = dst_len, r3 = dst_ptr.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes read, `Err(SvcError::OperationFailed)`
/// if `dst` exceeds 65 535 bytes, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_read(slot: u8, dst: &mut [u8]) -> Result<u32, SvcError> {
    if dst.len() > u16::MAX as usize {
        return Err(SvcError::InvalidParameter);
    }
    decode_rc(rtos_traits::svc!(
        SYS_BUF_READ,
        slot as u32,
        dst.len() as u32,
        dst.as_mut_ptr() as u32
    ))
}

/// Write data to a buffer slot.
///
/// ABI: r1 = slot, r2 = data_len, r3 = data_ptr.
///
/// # Returns
///
/// `Ok(n)` with the number of bytes written, `Err(SvcError::OperationFailed)`
/// if `data` exceeds 65 535 bytes, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_write(slot: u8, data: &[u8]) -> Result<u32, SvcError> {
    if data.len() > u16::MAX as usize {
        return Err(SvcError::InvalidParameter);
    }
    decode_rc(rtos_traits::svc!(
        SYS_BUF_WRITE,
        slot as u32,
        data.len() as u32,
        data.as_ptr() as u32
    ))
}

/// Lend a buffer slot to a target partition.
///
/// ABI: r1 = slot, r2 = (writable_bit8 | target), r3 = 0.
///
/// # Returns
///
/// `Ok(region_id)` with the MPU region assigned to the target, or
/// `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_lend(slot: u8, target: u8, writable: bool) -> Result<u32, SvcError> {
    let flags: u32 = if writable { lend_flags::WRITABLE } else { 0 };
    let r2 = (target as u32) | flags;
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  The slot and packed r2 contain only small integer values;
    // no pointers are passed.
    decode_rc(rtos_traits::svc!(SYS_BUF_LEND, slot as u32, r2, 0u32))
}

/// Revoke a previously lent buffer slot from a target partition.
///
/// ABI: r1 = slot, r2 = target.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_revoke(slot: u8, target: u8) -> Result<u32, SvcError> {
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  Only small integer values are passed; no pointers.
    decode_rc(rtos_traits::svc!(
        SYS_BUF_REVOKE,
        slot as u32,
        target as u32,
        0u32
    ))
}

/// Transfer ownership of a buffer slot to another partition.
///
/// ABI: r1 = slot, r2 = new_owner.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
#[cfg(feature = "dynamic-mpu")]
pub fn sys_buf_transfer(slot: u8, new_owner: u8) -> Result<u32, SvcError> {
    // SAFETY: svc! triggers a supervisor call whose handler validates all
    // arguments.  Only small integer values are passed; no pointers.
    decode_rc(rtos_traits::svc!(
        SYS_BUF_TRANSFER,
        slot as u32,
        new_owner as u32,
        0u32
    ))
}

/// Print a debug message via semihosting.
///
/// ABI: r1 = ptr, r2 = len.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_debug_print(msg: &[u8]) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(
        SYS_DEBUG_PRINT,
        msg.as_ptr() as u32,
        msg.len() as u32,
        0u32
    ))
}

/// Terminate the partition/system via semihosting with an exit code.
///
/// ABI: r1 = exit_code.
///
/// # Returns
///
/// `Ok(0)` on success, or `Err(SvcError)` if the syscall failed.
pub fn sys_debug_exit(code: u32) -> Result<u32, SvcError> {
    decode_rc(rtos_traits::svc!(SYS_DEBUG_EXIT, code, 0u32, 0u32))
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use alloc::vec;

    #[test]
    fn event_wait_returns_ok_zero_on_host() {
        assert_eq!(sys_event_wait(EventMask::new(0x01)), Ok(EventMask::new(0)));
    }

    #[test]
    fn event_set_returns_ok_zero_on_host() {
        assert_eq!(
            sys_event_set(PartitionId::new(1), EventMask::new(0xFF)),
            Ok(0)
        );
    }

    #[test]
    fn event_clear_returns_ok_zero_on_host() {
        assert_eq!(sys_event_clear(EventMask::new(0x03)), Ok(EventMask::new(0)));
    }

    #[test]
    fn irq_ack_min_returns_ok_zero_on_host() {
        assert_eq!(sys_irq_ack(0), Ok(0));
    }

    #[test]
    fn irq_ack_max_returns_ok_zero_on_host() {
        assert_eq!(sys_irq_ack(255), Ok(0));
    }

    #[test]
    fn yield_returns_ok_zero_on_host() {
        assert_eq!(sys_yield(), Ok(0));
    }

    #[test]
    fn get_partition_id_returns_ok_zero_on_host() {
        assert_eq!(sys_get_partition_id(), Ok(0));
    }

    #[test]
    fn get_time_returns_ok_zero_on_host() {
        assert_eq!(sys_get_time(), Ok(0));
    }

    #[test]
    fn sem_wait_returns_ok_zero_on_host() {
        assert_eq!(sys_sem_wait(SemaphoreId::new(0)), Ok(0));
    }

    #[test]
    fn sem_signal_returns_ok_zero_on_host() {
        assert_eq!(sys_sem_signal(SemaphoreId::new(0)), Ok(0));
    }

    #[test]
    fn mtx_lock_returns_ok_zero_on_host() {
        assert_eq!(sys_mtx_lock(MutexId::new(0)), Ok(0));
    }

    #[test]
    fn mtx_unlock_returns_ok_zero_on_host() {
        assert_eq!(sys_mtx_unlock(MutexId::new(0)), Ok(0));
    }

    #[test]
    fn sampling_write_returns_ok_zero_on_host() {
        assert_eq!(sys_sampling_write(0, &[1, 2, 3]), Ok(0));
    }

    #[test]
    fn sampling_read_returns_ok_zero_on_host() {
        let mut buf = [0u8; 8];
        assert_eq!(sys_sampling_read(0, &mut buf), Ok(0));
    }

    #[test]
    fn msg_send_returns_ok_zero_on_host() {
        assert_eq!(
            sys_msg_send(PartitionId::new(1), &[0xAA, 0xBB, 0xCC]),
            Ok(0)
        );
    }

    #[test]
    fn msg_send_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_msg_send(PartitionId::new(0), &[]), Ok(0));
    }

    #[test]
    fn msg_recv_returns_ok_zero_on_host() {
        let mut buf = [0u8; 16];
        assert_eq!(sys_msg_recv(&mut buf), Ok(0));
    }

    #[test]
    fn msg_recv_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_msg_recv(&mut buf), Ok(0));
    }

    #[test]
    fn queuing_send_returns_ok_zero_on_host() {
        assert_eq!(sys_queuing_send(0, &[0xDE, 0xAD]), Ok(0));
    }

    #[test]
    fn queuing_send_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_queuing_send(1, &[]), Ok(0));
    }

    #[test]
    fn queuing_recv_returns_ok_zero_on_host() {
        let mut buf = [0u8; 8];
        assert_eq!(sys_queuing_recv(0, &mut buf), Ok(0));
    }

    #[test]
    fn queuing_recv_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_queuing_recv(1, &mut buf), Ok(0));
    }

    #[test]
    fn queuing_status_returns_ok_zeroed_on_host() {
        let expected = QueuingPortStatus {
            nb_messages: 0,
            max_nb_messages: 0,
            max_message_size: 0,
            direction: 0,
        };
        assert_eq!(sys_queuing_status(0), Ok(expected));
    }

    #[test]
    fn queuing_send_timed_returns_ok_zero_on_host() {
        assert_eq!(sys_queuing_send_timed(0, &[0xAA, 0xBB], 100), Ok(0));
    }

    #[test]
    fn queuing_send_timed_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_queuing_send_timed(1, &[], 50), Ok(0));
    }

    #[test]
    fn queuing_send_timed_rejects_oversized_data() {
        // A slice longer than u16::MAX must be rejected before the syscall.
        let big = vec![0u8; u16::MAX as usize + 1];
        assert_eq!(
            sys_queuing_send_timed(0, &big, 100),
            Err(SvcError::InvalidParameter)
        );
    }

    #[test]
    fn queuing_recv_timed_returns_ok_zero_on_host() {
        let mut buf = [0u8; 8];
        assert_eq!(sys_queuing_recv_timed(0, &mut buf, 200), Ok(0));
    }

    #[test]
    fn queuing_recv_timed_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_queuing_recv_timed(1, &mut buf, 0), Ok(0));
    }

    #[test]
    fn queuing_recv_timed_max_timeout_returns_ok_zero_on_host() {
        let mut buf = [0u8; 4];
        assert_eq!(sys_queuing_recv_timed(0, &mut buf, u16::MAX), Ok(0));
    }

    #[test]
    fn queuing_recv_timed_rejects_oversized_buf() {
        // A buffer longer than u16::MAX must be rejected before the syscall.
        let mut big = vec![0u8; u16::MAX as usize + 1];
        assert_eq!(
            sys_queuing_recv_timed(0, &mut big, 100),
            Err(SvcError::InvalidParameter)
        );
    }

    #[test]
    fn queuing_send_timed_packing_boundary() {
        // Verify the largest valid data length (u16::MAX) is accepted.
        let data = vec![0u8; u16::MAX as usize];
        assert_eq!(sys_queuing_send_timed(0, &data, u16::MAX), Ok(0));
    }

    #[test]
    fn queuing_recv_timed_packing_boundary() {
        // Verify the largest valid buffer length (u16::MAX) is accepted.
        let mut buf = vec![0u8; u16::MAX as usize];
        assert_eq!(sys_queuing_recv_timed(0, &mut buf, u16::MAX), Ok(0));
    }

    // TODO: Host stub returns 0 unconditionally so we cannot verify that
    // the packed r2 value (timeout_hi16 | len_lo16) is correctly formed
    // from these tests.  Register-level verification requires an on-target
    // (QEMU) integration test that inspects the actual SVC arguments.

    // Parameter-verification tests live in kernel/src/debug.rs per the
    // crate's documented testing policy (see module docs).

    /// Verify the buf_syscall re-export is accessible (ABI helpers).
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_syscall_reexport_abi_helpers_accessible() {
        assert_eq!(crate::buf_syscall::pack_lend_r2(1, false), 1);
        assert_eq!(crate::buf_syscall::parse_result(0), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_display_returns_ok_zero_on_host() {
        assert_eq!(sys_bb_display(0, &[0xCA, 0xFE]), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_display_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_bb_display(0, &[]), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_read_returns_ok_zero_on_host() {
        let mut buf = [0u8; 16];
        assert_eq!(sys_bb_read(0, &mut buf), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_read_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_bb_read(0, &mut buf), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_clear_returns_ok_zero_on_host() {
        assert_eq!(sys_bb_clear(0), Ok(0));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bb_clear_max_board_id_returns_ok_zero_on_host() {
        assert_eq!(sys_bb_clear(u32::MAX), Ok(0));
    }

    // ── Re-export verification tests ──────────────────────────────────

    #[test]
    fn decode_rc_reexported_success() {
        assert_eq!(crate::decode_rc(0), Ok(0));
        assert_eq!(crate::decode_rc(42), Ok(42));
    }

    #[test]
    fn decode_rc_reexported_error() {
        assert_eq!(
            crate::decode_rc(SvcError::InvalidResource.to_u32()),
            Err(SvcError::InvalidResource)
        );
    }

    /// Every [`SvcError`] variant round-trips through `to_u32()` → `decode_rc()`.
    ///
    /// The wildcard-free match ensures a compile error when a new variant is
    /// added without updating this list.
    #[test]
    fn svc_error_round_trip_exhaustive() {
        use SvcError::*;
        let variants: [SvcError; 12] = [
            InvalidSyscall,
            InvalidResource,
            WaitQueueFull,
            TransitionFailed,
            InvalidPartition,
            OperationFailed,
            InvalidPointer,
            NotImplemented,
            BufferFull,
            NotSupported,
            PermissionDenied,
            InvalidParameter,
        ];
        for v in variants {
            // Compile-time exhaustiveness guard (no wildcard).
            match v {
                InvalidSyscall | InvalidResource | WaitQueueFull | TransitionFailed
                | InvalidPartition | OperationFailed | InvalidPointer | NotImplemented
                | BufferFull | NotSupported | PermissionDenied | InvalidParameter => {}
            }
            let code = v.to_u32();
            assert!(SvcError::is_error(code), "{v:?} must have error bit set");
            assert_eq!(decode_rc(code), Err(v), "{v:?} round-trip failed");
        }
        // Non-error values decode to Ok(value).
        for ok_val in [0u32, 1, 42, 0x7FFF_FFFF] {
            assert_eq!(decode_rc(ok_val), Ok(ok_val));
        }
    }

    /// Verify that plib re-exports map transparently to the rtos-traits
    /// source of truth (no hardcoded ABI values).
    #[test]
    fn syscall_constants_reexported_match_source() {
        use rtos_traits::syscall as src;
        // Control
        assert_eq!(crate::SYS_YIELD, src::SYS_YIELD);
        assert_eq!(crate::SYS_GET_PARTITION_ID, src::SYS_GET_PARTITION_ID);
        assert_eq!(crate::SYS_GET_TIME, src::SYS_GET_TIME);
        assert_eq!(crate::SYS_IRQ_ACK, src::SYS_IRQ_ACK);
        // Events
        assert_eq!(crate::SYS_EVT_WAIT, src::SYS_EVT_WAIT);
        assert_eq!(crate::SYS_EVT_SET, src::SYS_EVT_SET);
        assert_eq!(crate::SYS_EVT_CLEAR, src::SYS_EVT_CLEAR);
        // Sync
        assert_eq!(crate::SYS_SEM_WAIT, src::SYS_SEM_WAIT);
        assert_eq!(crate::SYS_SEM_SIGNAL, src::SYS_SEM_SIGNAL);
        assert_eq!(crate::SYS_MTX_LOCK, src::SYS_MTX_LOCK);
        assert_eq!(crate::SYS_MTX_UNLOCK, src::SYS_MTX_UNLOCK);
        // Messaging
        assert_eq!(crate::SYS_MSG_SEND, src::SYS_MSG_SEND);
        assert_eq!(crate::SYS_MSG_RECV, src::SYS_MSG_RECV);
        // Ports
        assert_eq!(crate::SYS_SAMPLING_WRITE, src::SYS_SAMPLING_WRITE);
        assert_eq!(crate::SYS_SAMPLING_READ, src::SYS_SAMPLING_READ);
        assert_eq!(crate::SYS_QUEUING_SEND, src::SYS_QUEUING_SEND);
        assert_eq!(crate::SYS_QUEUING_RECV, src::SYS_QUEUING_RECV);
        assert_eq!(crate::SYS_QUEUING_STATUS, src::SYS_QUEUING_STATUS);
        assert_eq!(crate::SYS_QUEUING_SEND_TIMED, src::SYS_QUEUING_SEND_TIMED);
        assert_eq!(crate::SYS_QUEUING_RECV_TIMED, src::SYS_QUEUING_RECV_TIMED);
        // Debug (unconditional)
        assert_eq!(crate::SYS_DEBUG_PRINT, src::SYS_DEBUG_PRINT);
        assert_eq!(crate::SYS_DEBUG_EXIT, src::SYS_DEBUG_EXIT);
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn syscall_bb_constants_reexported_match_source() {
        use rtos_traits::syscall as src;
        assert_eq!(crate::SYS_BB_DISPLAY, src::SYS_BB_DISPLAY);
        assert_eq!(crate::SYS_BB_READ, src::SYS_BB_READ);
        assert_eq!(crate::SYS_BB_CLEAR, src::SYS_BB_CLEAR);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn syscall_debug_gated_constants_reexported_match_source() {
        use rtos_traits::syscall as src;
        assert_eq!(crate::SYS_DEBUG_NOTIFY, src::SYS_DEBUG_NOTIFY);
        assert_eq!(crate::SYS_DEBUG_WRITE, src::SYS_DEBUG_WRITE);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn syscall_dev_constants_reexported_match_traits_source() {
        use rtos_traits::syscall as src;
        assert_eq!(crate::SYS_DEV_OPEN, src::SYS_DEV_OPEN);
        assert_eq!(crate::SYS_DEV_READ, src::SYS_DEV_READ);
        assert_eq!(crate::SYS_DEV_WRITE, src::SYS_DEV_WRITE);
        assert_eq!(crate::SYS_DEV_IOCTL, src::SYS_DEV_IOCTL);
        assert_eq!(crate::SYS_DEV_CLOSE, src::SYS_DEV_CLOSE);
        assert_eq!(crate::SYS_DEV_READ_TIMED, src::SYS_DEV_READ_TIMED);
        assert_eq!(crate::SYS_QUERY_BOTTOM_HALF, src::SYS_QUERY_BOTTOM_HALF);
        assert_eq!(crate::SYS_BUF_READ, src::SYS_BUF_READ);
        assert_eq!(crate::SYS_BUF_WRITE, src::SYS_BUF_WRITE);
        assert_eq!(crate::SYS_BUF_LEND, src::SYS_BUF_LEND);
        assert_eq!(crate::SYS_BUF_REVOKE, src::SYS_BUF_REVOKE);
        assert_eq!(crate::SYS_BUF_TRANSFER, src::SYS_BUF_TRANSFER);
    }

    /// Verify that plib re-exports of resource ID newtypes resolve to the
    /// same types as `rtos_traits::ids` (construct via both paths, compare).
    #[test]
    fn resource_id_newtypes_reexported_match_source() {
        use rtos_traits::ids as src;
        // Unconditional re-exports
        assert_eq!(crate::SemaphoreId::new(1), src::SemaphoreId::new(1));
        assert_eq!(crate::MutexId::new(2), src::MutexId::new(2));
        assert_eq!(crate::SamplingPortId::new(3), src::SamplingPortId::new(3));
        assert_eq!(crate::QueuingPortId::new(4), src::QueuingPortId::new(4));
        assert_eq!(crate::PartitionId::new(5), src::PartitionId::new(5));
        assert_eq!(crate::EventMask::new(0xFF), src::EventMask::new(0xFF));
    }

    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn blackboard_id_reexported_matches_source() {
        use rtos_traits::ids as src;
        assert_eq!(crate::BlackboardId::new(6), src::BlackboardId::new(6));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn device_id_reexported_matches_source() {
        use rtos_traits::ids as src;
        assert_eq!(crate::DeviceId::new(7), src::DeviceId::new(7));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_open_returns_ok_zero_on_host() {
        assert_eq!(sys_dev_open(0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_close_returns_ok_zero_on_host() {
        assert_eq!(sys_dev_close(0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_ioctl_returns_ok_zero_on_host() {
        assert_eq!(sys_dev_ioctl(0, 1, 2), Ok(0));
    }

    // TODO: dev_read / dev_write host tests are liveness-only smoke tests.
    // The host stub always returns Ok(0) without populating buffers or
    // verifying written data, so data-integrity assertions are not feasible
    // here.  Integration tests on target hardware should cover payload
    // correctness.

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_returns_ok_zero_on_host() {
        let mut buf = [0u8; 8];
        assert_eq!(sys_dev_read(0, &mut buf), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_dev_read(1, &mut buf), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_write_returns_ok_zero_on_host() {
        assert_eq!(sys_dev_write(0, &[0xDE, 0xAD]), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_write_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_dev_write(1, &[]), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_returns_ok_zero_on_host() {
        let mut buf = [0u8; 8];
        assert_eq!(sys_dev_read_timed(0, &mut buf, 100), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_empty_buf_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_dev_read_timed(1, &mut buf, 50), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_rejects_oversized_buf() {
        let mut big = vec![0u8; u16::MAX as usize + 1];
        assert_eq!(
            sys_dev_read_timed(0, &mut big, 100),
            Err(SvcError::InvalidParameter)
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_packing_boundary() {
        let mut buf = vec![0u8; u16::MAX as usize];
        assert_eq!(sys_dev_read_timed(0, &mut buf, u16::MAX), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn query_bottom_half_returns_ok_zero_on_host() {
        assert_eq!(sys_query_bottom_half(0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn query_bottom_half_max_device_id_returns_ok_zero_on_host() {
        assert_eq!(sys_query_bottom_half(u8::MAX), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_readonly_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_alloc(false, 0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_writable_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_alloc(true, 100), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_max_ticks_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_alloc(false, u16::MAX), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_release_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_release(0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_release_max_slot_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_release(u8::MAX), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_returns_ok_zero_on_host() {
        let mut buf = [0u8; 16];
        assert_eq!(sys_buf_read(0, &mut buf), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_empty_dst_returns_ok_zero_on_host() {
        let mut buf = [0u8; 0];
        assert_eq!(sys_buf_read(1, &mut buf), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_rejects_oversized_dst() {
        let mut big = vec![0u8; u16::MAX as usize + 1];
        assert_eq!(sys_buf_read(0, &mut big), Err(SvcError::InvalidParameter));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_boundary_len_accepted() {
        let mut buf = vec![0u8; u16::MAX as usize];
        assert_eq!(sys_buf_read(0, &mut buf), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_write_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_write(0, &[0xDE, 0xAD]), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_write_empty_data_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_write(1, &[]), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_write_rejects_oversized_data() {
        let big = vec![0u8; u16::MAX as usize + 1];
        assert_eq!(sys_buf_write(0, &big), Err(SvcError::InvalidParameter));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_write_boundary_len_accepted() {
        let data = vec![0u8; u16::MAX as usize];
        assert_eq!(sys_buf_write(0, &data), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn lend_r2_packing_readonly_sets_target_only() {
        // Verify the inline packing in sys_buf_lend: r2 = target | flags.
        fn pack(target: u8, writable: bool) -> u32 {
            let flags: u32 = if writable { lend_flags::WRITABLE } else { 0 };
            (target as u32) | flags
        }
        assert_eq!(pack(0, false), 0);
        assert_eq!(pack(1, false), 1);
        assert_eq!(pack(0xFF, false), 0xFF);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn lend_r2_packing_writable_sets_bit8_and_target() {
        fn pack(target: u8, writable: bool) -> u32 {
            let flags: u32 = if writable { lend_flags::WRITABLE } else { 0 };
            (target as u32) | flags
        }
        assert_eq!(pack(0, true), lend_flags::WRITABLE);
        assert_eq!(pack(1, true), 1 | lend_flags::WRITABLE);
        assert_eq!(pack(0xFF, true), 0xFF | lend_flags::WRITABLE);
    }

    // TODO: Host stub returns 0 unconditionally so we cannot verify that
    // sys_buf_lend propagates the kernel's region_id return value, or that
    // the packed r2 reaches the SVC handler.  Register-level verification
    // requires an on-target (QEMU) integration test.

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_readonly_returns_ok_on_host() {
        assert_eq!(sys_buf_lend(0, 1, false), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_writable_returns_ok_on_host() {
        assert_eq!(sys_buf_lend(2, 3, true), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_revoke_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_revoke(0, 1), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_revoke_max_slot_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_revoke(u8::MAX, 0), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_transfer_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_transfer(0, 1), Ok(0));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_transfer_max_slot_returns_ok_zero_on_host() {
        assert_eq!(sys_buf_transfer(u8::MAX, u8::MAX), Ok(0));
    }

    #[test]
    fn debug_print_empty_msg_returns_ok_zero_on_host() {
        assert_eq!(sys_debug_print(b""), Ok(0));
    }

    #[test]
    fn debug_print_nonempty_msg_returns_ok_zero_on_host() {
        assert_eq!(sys_debug_print(b"hello world"), Ok(0));
    }

    #[test]
    fn debug_exit_zero_code_returns_ok_zero_on_host() {
        assert_eq!(sys_debug_exit(0), Ok(0));
    }

    #[test]
    fn debug_exit_nonzero_code_returns_ok_zero_on_host() {
        assert_eq!(sys_debug_exit(1), Ok(0));
    }
}
