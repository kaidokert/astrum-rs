//! Syscall constants and error types shared between kernel and partition code.
//!
//! These types define the ABI for syscall invocations and are used by both
//! the kernel (callee) and partitions (callers).

// ── Base (unconditional) syscall numbers ──────────────────────────────
pub const SYS_YIELD: u32 = 0;
pub const SYS_GET_PARTITION_ID: u32 = 1;
pub const SYS_EVT_WAIT: u32 = 2;
pub const SYS_EVT_SET: u32 = 3;
pub const SYS_EVT_CLEAR: u32 = 4;
pub const SYS_SEM_WAIT: u32 = 5;
pub const SYS_SEM_SIGNAL: u32 = 6;
pub const SYS_MTX_LOCK: u32 = 7;
pub const SYS_MTX_UNLOCK: u32 = 8;
pub const SYS_MSG_SEND: u32 = 9;
pub const SYS_MSG_RECV: u32 = 10;
pub const SYS_GET_TIME: u32 = 11;
pub const SYS_SAMPLING_WRITE: u32 = 12;
pub const SYS_SAMPLING_READ: u32 = 13;
pub const SYS_QUEUING_SEND: u32 = 14;
pub const SYS_QUEUING_RECV: u32 = 15;
pub const SYS_QUEUING_STATUS: u32 = 16;
pub const SYS_BB_DISPLAY: u32 = 17;
pub const SYS_BB_READ: u32 = 18;
pub const SYS_BB_CLEAR: u32 = 19;
/// Timed queuing send: r1=port_id, r2=(timeout_ticks_hi16 << 16 | data_len_lo16), r3=data_ptr
pub const SYS_QUEUING_SEND_TIMED: u32 = 27;
/// Timed queuing recv: r1=port_id, r2=(timeout_ticks_hi16 << 16 | buf_len_lo16), r3=buf_ptr
pub const SYS_QUEUING_RECV_TIMED: u32 = 28;
/// Debug print: r1=string_ptr, r2=string_len. Outputs via semihosting (privileged).
pub const SYS_DEBUG_PRINT: u32 = 31;
/// Debug exit: r1=exit_code (0=success, nonzero=failure). Exits via semihosting.
pub const SYS_DEBUG_EXIT: u32 = 32;
/// IRQ acknowledge: r1=irq_number. Re-enables the masked IRQ after partition handles it.
pub const SYS_IRQ_ACK: u32 = 38;

// ── Feature-gated syscall numbers ─────────────────────────────────────

/// Debug notify syscall number: sets a per-partition 'debug pending' flag.
#[cfg(feature = "partition-debug")]
pub const SYS_DEBUG_NOTIFY: u32 = 0x40;

/// Debug write syscall number: writes data to partition's debug ring buffer.
/// r1=ptr, r2=len. Returns bytes written in r0 or error code.
#[cfg(feature = "partition-debug")]
pub const SYS_DEBUG_WRITE: u32 = 0x41;

// ── Dynamic-MPU syscall numbers ──────────────────────────────────────

#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_ALLOC: u32 = 20;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_RELEASE: u32 = 21;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_OPEN: u32 = 22;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_READ: u32 = 23;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_WRITE: u32 = 24;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_IOCTL: u32 = 25;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_WRITE: u32 = 26;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_CLOSE: u32 = 29;
/// Timed device read: r1=device_id, r2=timeout_ticks (0=non-blocking), r3=buf_ptr
#[cfg(feature = "dynamic-mpu")]
pub const SYS_DEV_READ_TIMED: u32 = 30;
/// Query bottom-half status: returns ticks_since_bottom_half in r0, stale flag in r1.
// TODO: Currently gated behind dynamic-mpu because the underlying ticks_since_bottom_half
// and is_bottom_half_stale mechanisms are feature-gated. A future refactor should make
// bottom-half health monitoring unconditional as it's a core architectural concern.
#[cfg(feature = "dynamic-mpu")]
pub const SYS_QUERY_BOTTOM_HALF: u32 = 33;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_LEND: u32 = 34;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_REVOKE: u32 = 35;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_TRANSFER: u32 = 36;
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_READ: u32 = 37;

/// Flags for `SYS_BUF_LEND`, packed into upper bits of r2.
#[cfg(feature = "dynamic-mpu")]
pub mod lend_flags {
    /// Grant AP_FULL_ACCESS instead of AP_RO_RO to the target.
    pub const WRITABLE: u32 = 1 << 8;
}

/// Typed SVC error codes returned to user-space via r0.
///
/// Each variant maps to a unique `u32` with the high bit set (>= 0x8000_0000),
/// making them distinguishable from success values (which are small non-negative
/// integers such as byte counts or zero).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SvcError {
    /// The syscall number in r0 was not a recognized `SyscallId`.
    InvalidSyscall,
    /// The resource ID (semaphore, mutex, queue, port, blackboard) was out of
    /// range or does not refer to an allocated resource.
    InvalidResource,
    /// A wait queue on the target resource is full and cannot accept another
    /// blocked partition.
    WaitQueueFull,
    /// A partition state transition (e.g. Running → Waiting) failed because the
    /// current state does not permit it.
    TransitionFailed,
    /// The partition index supplied as caller or target is out of range.
    InvalidPartition,
    /// A catch-all for operation-specific failures (e.g. direction violation,
    /// message too large, board empty on non-blocking read).
    OperationFailed,
    /// A user-supplied pointer (and length) does not lie within the calling
    /// partition's MPU data region or the arithmetic overflows `u32`.
    InvalidPointer,
    /// The syscall number is recognised but the handler is not yet
    /// implemented.
    NotImplemented,
    /// A write to a debug buffer failed because the buffer is full.
    BufferFull,
    /// The requested operation is not supported (e.g., debug buffer not configured).
    NotSupported,
    /// The caller lacks the required ownership or permission.
    PermissionDenied,
}

impl SvcError {
    /// Bit mask shared by all error codes.  Every `SvcError` variant has this
    /// bit set in its `u32` representation, while success values (small
    /// non-negative integers) never do.
    pub const ERROR_BIT: u32 = 0x8000_0000;

    /// Return `true` when the raw `u32` returned by an SVC call indicates an
    /// error (i.e. has the high bit set).
    #[inline]
    pub const fn is_error(code: u32) -> bool {
        code & Self::ERROR_BIT != 0
    }

    /// Convert a raw `u32` error code back to a typed [`SvcError`].
    ///
    /// Returns `None` if the code does not match any known variant.
    pub const fn from_u32(code: u32) -> Option<Self> {
        match code {
            0xFFFF_FFFF => Some(Self::InvalidSyscall),
            0xFFFF_FFFE => Some(Self::InvalidResource),
            0xFFFF_FFFD => Some(Self::WaitQueueFull),
            0xFFFF_FFFC => Some(Self::TransitionFailed),
            0xFFFF_FFFB => Some(Self::InvalidPartition),
            0xFFFF_FFFA => Some(Self::OperationFailed),
            0xFFFF_FFF9 => Some(Self::InvalidPointer),
            0xFFFF_FFF8 => Some(Self::NotImplemented),
            0xFFFF_FFF7 => Some(Self::BufferFull),
            0xFFFF_FFF6 => Some(Self::NotSupported),
            0xFFFF_FFF5 => Some(Self::PermissionDenied),
            _ => None,
        }
    }

    /// Map this error to a unique `u32` value with the high bit set.
    ///
    /// The values count down from `0xFFFF_FFFF` so they are easy to inspect in
    /// a debugger and leave room for future variants without renumbering.
    pub const fn to_u32(self) -> u32 {
        match self {
            Self::InvalidSyscall => 0xFFFF_FFFF,
            Self::InvalidResource => 0xFFFF_FFFE,
            Self::WaitQueueFull => 0xFFFF_FFFD,
            Self::TransitionFailed => 0xFFFF_FFFC,
            Self::InvalidPartition => 0xFFFF_FFFB,
            Self::OperationFailed => 0xFFFF_FFFA,
            Self::InvalidPointer => 0xFFFF_FFF9,
            Self::NotImplemented => 0xFFFF_FFF8,
            Self::BufferFull => 0xFFFF_FFF7,
            Self::NotSupported => 0xFFFF_FFF6,
            Self::PermissionDenied => 0xFFFF_FFF5,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// All base (unconditional) syscall constants with their expected values.
    const BASE_SYSCALLS: &[(&str, u32, u32)] = &[
        ("SYS_YIELD", SYS_YIELD, 0),
        ("SYS_GET_PARTITION_ID", SYS_GET_PARTITION_ID, 1),
        ("SYS_EVT_WAIT", SYS_EVT_WAIT, 2),
        ("SYS_EVT_SET", SYS_EVT_SET, 3),
        ("SYS_EVT_CLEAR", SYS_EVT_CLEAR, 4),
        ("SYS_SEM_WAIT", SYS_SEM_WAIT, 5),
        ("SYS_SEM_SIGNAL", SYS_SEM_SIGNAL, 6),
        ("SYS_MTX_LOCK", SYS_MTX_LOCK, 7),
        ("SYS_MTX_UNLOCK", SYS_MTX_UNLOCK, 8),
        ("SYS_MSG_SEND", SYS_MSG_SEND, 9),
        ("SYS_MSG_RECV", SYS_MSG_RECV, 10),
        ("SYS_GET_TIME", SYS_GET_TIME, 11),
        ("SYS_SAMPLING_WRITE", SYS_SAMPLING_WRITE, 12),
        ("SYS_SAMPLING_READ", SYS_SAMPLING_READ, 13),
        ("SYS_QUEUING_SEND", SYS_QUEUING_SEND, 14),
        ("SYS_QUEUING_RECV", SYS_QUEUING_RECV, 15),
        ("SYS_QUEUING_STATUS", SYS_QUEUING_STATUS, 16),
        ("SYS_BB_DISPLAY", SYS_BB_DISPLAY, 17),
        ("SYS_BB_READ", SYS_BB_READ, 18),
        ("SYS_BB_CLEAR", SYS_BB_CLEAR, 19),
        ("SYS_QUEUING_SEND_TIMED", SYS_QUEUING_SEND_TIMED, 27),
        ("SYS_QUEUING_RECV_TIMED", SYS_QUEUING_RECV_TIMED, 28),
        ("SYS_DEBUG_PRINT", SYS_DEBUG_PRINT, 31),
        ("SYS_DEBUG_EXIT", SYS_DEBUG_EXIT, 32),
        ("SYS_IRQ_ACK", SYS_IRQ_ACK, 38),
    ];

    #[test]
    fn base_constants_have_correct_values() {
        for &(name, actual, expected) in BASE_SYSCALLS {
            assert_eq!(actual, expected, "{name} should be {expected}");
        }
    }

    #[test]
    fn base_constants_are_unique() {
        for (i, &(name_a, val_a, _)) in BASE_SYSCALLS.iter().enumerate() {
            for &(name_b, val_b, _) in &BASE_SYSCALLS[i + 1..] {
                assert_ne!(val_a, val_b, "{name_a} and {name_b} must differ");
            }
        }
    }

    #[test]
    fn base_constant_count() {
        assert_eq!(BASE_SYSCALLS.len(), 25);
    }

    /// Dynamic-MPU syscall constants: (name, actual, expected).
    #[cfg(feature = "dynamic-mpu")]
    const DYN_SYSCALLS: &[(&str, u32, u32)] = &[
        ("SYS_BUF_ALLOC", SYS_BUF_ALLOC, 20),
        ("SYS_BUF_RELEASE", SYS_BUF_RELEASE, 21),
        ("SYS_DEV_OPEN", SYS_DEV_OPEN, 22),
        ("SYS_DEV_READ", SYS_DEV_READ, 23),
        ("SYS_DEV_WRITE", SYS_DEV_WRITE, 24),
        ("SYS_DEV_IOCTL", SYS_DEV_IOCTL, 25),
        ("SYS_BUF_WRITE", SYS_BUF_WRITE, 26),
        ("SYS_DEV_CLOSE", SYS_DEV_CLOSE, 29),
        ("SYS_DEV_READ_TIMED", SYS_DEV_READ_TIMED, 30),
        ("SYS_QUERY_BOTTOM_HALF", SYS_QUERY_BOTTOM_HALF, 33),
        ("SYS_BUF_LEND", SYS_BUF_LEND, 34),
        ("SYS_BUF_REVOKE", SYS_BUF_REVOKE, 35),
        ("SYS_BUF_TRANSFER", SYS_BUF_TRANSFER, 36),
        ("SYS_BUF_READ", SYS_BUF_READ, 37),
    ];

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dynamic_mpu_constants_values_unique_no_overlap() {
        assert_eq!(DYN_SYSCALLS.len(), 14);
        for &(name, actual, expected) in DYN_SYSCALLS {
            assert_eq!(actual, expected, "{name} should be {expected}");
        }
        for (i, &(a, va, _)) in DYN_SYSCALLS.iter().enumerate() {
            for &(b, vb, _) in &DYN_SYSCALLS[i + 1..] {
                assert_ne!(va, vb, "{a} and {b} must differ");
            }
            for &(bn, bv, _) in BASE_SYSCALLS {
                assert_ne!(va, bv, "{a} and {bn} must not overlap");
            }
        }
    }
}
