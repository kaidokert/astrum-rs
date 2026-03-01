//! User-space convenience wrappers for buffer-pool lending syscalls.

use crate::buffer_pool::lend_flags;
use crate::syscall::{SYS_BUF_LEND, SYS_BUF_READ, SYS_BUF_REVOKE, SYS_BUF_TRANSFER};
use rtos_traits::syscall::SvcError;

/// Pack target partition ID and writable flag into r2.
/// Layout: bits\[7:0\] = target, bit\[8\] = WRITABLE.
#[inline]
pub const fn pack_lend_r2(target: u8, writable: bool) -> u32 {
    let flags = if writable { lend_flags::WRITABLE } else { 0 };
    (target as u32) | flags
}

/// Convert a raw SVC return value to `Result<u32, SvcError>`.
#[inline]
fn parse_result(raw: u32) -> Result<u32, SvcError> {
    if SvcError::is_error(raw) {
        Err(SvcError::from_u32(raw).unwrap_or(SvcError::InvalidSyscall))
    } else {
        Ok(raw)
    }
}

/// Lend a buffer slot to a target partition.  Returns MPU region ID.
#[inline]
pub fn buf_lend(slot: u8, target: u8, writable: bool) -> Result<u8, SvcError> {
    let r2 = pack_lend_r2(target, writable);
    let raw = crate::svc!(SYS_BUF_LEND, slot as u32, r2, 0u32);
    parse_result(raw).map(|v| v as u8)
}

/// Revoke a previously lent buffer slot from a target partition.
#[inline]
pub fn buf_revoke(slot: u8, target: u8) -> Result<(), SvcError> {
    let raw = crate::svc!(SYS_BUF_REVOKE, slot as u32, target as u32, 0u32);
    parse_result(raw).map(|_| ())
}

/// Transfer buffer slot ownership to a new partition.
#[inline]
pub fn buf_transfer(slot: u8, new_owner: u8) -> Result<(), SvcError> {
    let raw = crate::svc!(SYS_BUF_TRANSFER, slot as u32, new_owner as u32, 0u32);
    parse_result(raw).map(|_| ())
}

/// Read data from a buffer slot into `dst`.  Returns bytes copied.
#[inline]
pub fn buf_read(slot: u8, dst: &mut [u8]) -> Result<usize, SvcError> {
    let raw = crate::svc!(
        SYS_BUF_READ,
        slot as u32,
        dst.len() as u32,
        dst.as_mut_ptr() as u32
    );
    parse_result(raw).map(|v| v as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_lend_r2_readonly() {
        let r2 = pack_lend_r2(3, false);
        assert_eq!(r2 & 0xFF, 3);
        assert_eq!(r2 & lend_flags::WRITABLE, 0);
    }

    #[test]
    fn pack_lend_r2_writable() {
        let r2 = pack_lend_r2(5, true);
        assert_eq!(r2 & 0xFF, 5);
        assert_eq!(r2, 5 | (1 << 8));
    }

    #[test]
    fn pack_lend_r2_boundary() {
        assert_eq!(pack_lend_r2(0, false), 0);
        assert_eq!(pack_lend_r2(255, true), 255 | lend_flags::WRITABLE);
    }

    #[test]
    fn parse_result_success() {
        assert_eq!(parse_result(0), Ok(0));
        assert_eq!(parse_result(42), Ok(42));
    }

    #[test]
    fn parse_result_known_error() {
        let raw = SvcError::InvalidResource.to_u32();
        assert_eq!(parse_result(raw), Err(SvcError::InvalidResource));
    }

    #[test]
    fn parse_result_unknown_error() {
        assert_eq!(parse_result(0x8000_0001), Err(SvcError::InvalidSyscall));
    }

    #[test]
    fn wrappers_return_ok_on_host() {
        assert_eq!(buf_lend(0, 1, false), Ok(0));
        assert_eq!(buf_transfer(0, 1), Ok(()));
        assert_eq!(buf_revoke(0, 1), Ok(()));
        let mut dst = [0u8; 16];
        assert_eq!(buf_read(0, &mut dst), Ok(0));
    }

    #[test]
    fn buf_revoke_returns_ok_on_host() {
        assert_eq!(buf_revoke(0, 1), Ok(()));
        assert_eq!(buf_revoke(255, 0), Ok(()));
        assert_eq!(buf_revoke(3, 7), Ok(()));
    }

    #[test]
    fn buf_revoke_parse_result_errors() {
        // Verify parse_result correctly converts error codes that buf_revoke would encounter
        let raw_err = SvcError::InvalidResource.to_u32();
        let result: Result<(), SvcError> = parse_result(raw_err).map(|_| ());
        assert_eq!(result, Err(SvcError::InvalidResource));

        let raw_op = SvcError::OperationFailed.to_u32();
        let result: Result<(), SvcError> = parse_result(raw_op).map(|_| ());
        assert_eq!(result, Err(SvcError::OperationFailed));
    }
}
