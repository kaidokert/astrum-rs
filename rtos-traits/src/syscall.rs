//! Syscall constants and error types shared between kernel and partition code.
//!
//! These types define the ABI for syscall invocations and are used by both
//! the kernel (callee) and partitions (callers).

/// Debug notify syscall number: sets a per-partition 'debug pending' flag.
#[cfg(feature = "partition-debug")]
pub const SYS_DEBUG_NOTIFY: u32 = 0x40;

/// Debug write syscall number: writes data to partition's debug ring buffer.
/// r1=ptr, r2=len. Returns bytes written in r0 or error code.
#[cfg(feature = "partition-debug")]
pub const SYS_DEBUG_WRITE: u32 = 0x41;

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
