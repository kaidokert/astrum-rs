//! Safe typed wrapper layer for syscall return codes.
//!
//! This module provides ergonomic Rust types on top of the raw `u32`
//! return codes produced by SVC calls, converting the ABI-level error
//! bit convention into `Result<u32, SvcError>`.

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
    InvalidParameter,
    /// A timed operation expired before the resource became available.
    TimedOut,
    /// A buffer slot index is out of range or the slot has no valid base
    /// address (e.g. the address lookup failed after a successful lend).
    InvalidBuffer,
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
            0xFFFF_FFF4 => Some(Self::InvalidParameter),
            0xFFFF_FFF3 => Some(Self::TimedOut),
            0xFFFF_FFF2 => Some(Self::InvalidBuffer),
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
            Self::InvalidParameter => 0xFFFF_FFF4,
            Self::TimedOut => 0xFFFF_FFF3,
            Self::InvalidBuffer => 0xFFFF_FFF2,
        }
    }
}

/// Decode a raw SVC return code into a typed `Result`.
///
/// Success values (high bit clear) are returned as `Ok(rc)`.
/// Error values (high bit set) are mapped to `Err(SvcError)` via
/// [`SvcError::from_u32`], falling back to [`SvcError::InvalidSyscall`]
/// for unrecognised error codes.
#[inline(always)]
pub fn decode_rc(rc: u32) -> Result<u32, SvcError> {
    if SvcError::is_error(rc) {
        Err(SvcError::from_u32(rc).unwrap_or(SvcError::InvalidSyscall))
    } else {
        Ok(rc)
    }
}

/// Like [`decode_rc`] but preserves a second return register (`r1`).
///
/// On success returns `Ok((r0, r1))`; on error returns the decoded
/// [`SvcError`] from `r0`.
#[inline(always)]
pub fn decode_rc_r01(r0r1: (u32, u32)) -> Result<(u32, u32), SvcError> {
    if SvcError::is_error(r0r1.0) {
        Err(SvcError::from_u32(r0r1.0).unwrap_or(SvcError::InvalidSyscall))
    } else {
        Ok(r0r1)
    }
}

#[cfg(test)]
mod tests {
    /// Return an array of all [`SvcError`] variants.
    ///
    /// This function uses a wildcard-free match so the compiler will emit
    /// an error when a new variant is added, forcing the test suite to be
    /// updated.
    const fn all_variants() -> [SvcError; 14] {
        use SvcError::*;
        [
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
            TimedOut,
            InvalidBuffer,
        ]
    }

    use super::*;

    #[test]
    fn success_zero() {
        assert_eq!(decode_rc(0), Ok(0));
    }

    #[test]
    fn success_nonzero() {
        assert_eq!(decode_rc(42), Ok(42));
    }

    #[test]
    fn success_max() {
        assert_eq!(decode_rc(0x7FFF_FFFF), Ok(0x7FFF_FFFF));
    }

    #[test]
    fn known_error_invalid_resource() {
        let code = SvcError::InvalidResource.to_u32();
        assert_eq!(decode_rc(code), Err(SvcError::InvalidResource));
    }

    #[test]
    fn unknown_error_maps_to_invalid_syscall() {
        // 0x8000_0001 has the error bit set but doesn't match any known variant
        let code: u32 = 0x8000_0001;
        assert!(SvcError::is_error(code));
        assert_eq!(decode_rc(code), Err(SvcError::InvalidSyscall));
    }

    /// Verify that every [`SvcError`] variant round-trips through
    /// `to_u32` → `decode_rc` → `Err(variant)`.
    ///
    /// Uses [`all_variants`] which is compiler-enforced exhaustive: adding
    /// a new variant without updating it will cause a build error.
    #[test]
    fn round_trip_all_variants() {
        for variant in all_variants() {
            let code = variant.to_u32();
            assert!(
                SvcError::is_error(code),
                "{variant:?} should have error bit"
            );
            assert_eq!(
                decode_rc(code),
                Err(variant),
                "{variant:?} round-trip failed"
            );
        }
    }

    /// Ensure [`all_variants`] covers every arm of [`SvcError`].
    ///
    /// The exhaustive match here (no wildcard `_`) guarantees a compile
    /// error when a new variant is added without updating this test.
    #[test]
    fn all_variants_is_exhaustive() {
        let variants = all_variants();
        for v in variants {
            match v {
                SvcError::InvalidSyscall
                | SvcError::InvalidResource
                | SvcError::WaitQueueFull
                | SvcError::TransitionFailed
                | SvcError::InvalidPartition
                | SvcError::OperationFailed
                | SvcError::InvalidPointer
                | SvcError::NotImplemented
                | SvcError::BufferFull
                | SvcError::NotSupported
                | SvcError::PermissionDenied
                | SvcError::InvalidParameter
                | SvcError::TimedOut
                | SvcError::InvalidBuffer => {}
            }
        }
        // If a variant is added to the enum but not to all_variants(),
        // this match will fail to compile.
    }

    #[test]
    fn decode_rc_r01_success_preserves_both() {
        assert_eq!(decode_rc_r01((5, 0x2000_0000)), Ok((5, 0x2000_0000)));
    }

    #[test]
    fn decode_rc_r01_success_zero_pair() {
        assert_eq!(decode_rc_r01((0, 0)), Ok((0, 0)));
    }

    #[test]
    fn decode_rc_r01_error_discards_r1() {
        let code = SvcError::InvalidResource.to_u32();
        assert_eq!(
            decode_rc_r01((code, 0x1234)),
            Err(SvcError::InvalidResource)
        );
    }
}
