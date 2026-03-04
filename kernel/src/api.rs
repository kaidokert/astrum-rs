//! Safe typed wrapper layer for partition syscalls.
//!
//! This module provides ergonomic Rust types on top of the raw `u32`
//! return codes produced by SVC calls, converting the ABI-level error
//! bit convention into `Result<u32, SvcError>`.

pub use rtos_traits::syscall::SvcError;

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

#[cfg(test)]
mod tests {
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
}
