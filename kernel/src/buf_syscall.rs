// TODO: buf_syscall.rs uses raw u8/u32 for partition IDs at the syscall ABI boundary;
// PartitionId conversion happens kernel-side in svc/mod.rs dispatch. No change needed here.
//! User-space convenience wrappers for buffer-pool lending syscalls.

use crate::syscall::{
    SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_READ, SYS_BUF_RELEASE, SYS_BUF_REVOKE, SYS_BUF_TRANSFER,
    SYS_BUF_WRITE,
};
// ABI helpers re-exported from rtos-traits.
pub use rtos_traits::buf_syscall::{pack_lend_r2, parse_result, LendResult};
#[cfg(test)]
use rtos_traits::syscall::lend_flags;
use rtos_traits::syscall::SvcError;

/// Allocate a buffer slot.  Returns the slot index on success.
///
/// `writable`: `true` = `BorrowedWrite`, `false` = `BorrowedRead`.
/// `max_ticks`: deadline in ticks (0 = no deadline).
#[inline]
pub fn buf_alloc(writable: bool, max_ticks: u32) -> Result<u8, SvcError> {
    let mode = if writable { 1u32 } else { 0u32 };
    let raw = crate::svc!(SYS_BUF_ALLOC, mode, max_ticks, 0u32);
    parse_result(raw).map(|v| v as u8)
}

/// Write `data` into a buffer slot.  Returns bytes written on success.
#[inline]
pub fn buf_write(slot: u8, data: &[u8]) -> Result<usize, SvcError> {
    let raw = crate::svc!(
        SYS_BUF_WRITE,
        slot as u32,
        data.len() as u32,
        data.as_ptr() as u32
    );
    parse_result(raw).map(|v| v as usize)
}

/// Lend a buffer slot to a target partition.  Returns MPU region ID.
#[inline]
pub fn buf_lend(slot: u8, target: u8, writable: bool) -> Result<u8, SvcError> {
    buf_lend_with_deadline(slot, target, writable, 0)
}

/// Lend a buffer slot to a target partition, returning both the MPU region
/// ID and the buffer base address.
///
/// This variant uses [`svc_r01!`] to capture both `r0` (region ID) and
/// `r1` (base address) from the kernel's `SYS_BUF_LEND` handler.
#[inline]
pub fn buf_lend_with_addr(slot: u8, target: u8, writable: bool) -> Result<LendResult, SvcError> {
    buf_lend_with_addr_deadline(slot, target, writable, 0)
}

/// Lend a buffer slot to a target partition with a deadline.
/// Returns MPU region ID.
///
/// `max_ticks`: relative tick offset for auto-revoke (0 = no deadline).
#[inline]
pub fn buf_lend_with_deadline(
    slot: u8,
    target: u8,
    writable: bool,
    max_ticks: u32,
) -> Result<u8, SvcError> {
    let r2 = pack_lend_r2(target, writable);
    let raw = crate::svc!(SYS_BUF_LEND, slot as u32, r2, max_ticks);
    parse_result(raw).map(|v| v as u8)
}

/// Lend a buffer slot to a target partition with a deadline, returning both
/// the MPU region ID and the buffer base address.
///
/// `max_ticks`: relative tick offset for auto-revoke (0 = no deadline).
#[inline]
pub fn buf_lend_with_addr_deadline(
    slot: u8,
    target: u8,
    writable: bool,
    max_ticks: u32,
) -> Result<LendResult, SvcError> {
    let r2 = pack_lend_r2(target, writable);
    let (r0, r1) = crate::svc_r01!(SYS_BUF_LEND, slot as u32, r2, max_ticks);
    parse_result(r0).map(|v| LendResult {
        region_id: v as u8,
        base_addr: r1,
    })
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

/// Release a buffer slot back to the pool.
#[inline]
pub fn buf_release(slot: u8) -> Result<(), SvcError> {
    let raw = crate::svc!(SYS_BUF_RELEASE, slot as u32, 0u32, 0u32);
    parse_result(raw).map(|_| ())
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
        assert_eq!(buf_alloc(true, 0), Ok(0));
        assert_eq!(buf_alloc(false, 50), Ok(0));
        assert_eq!(buf_write(0, &[1, 2, 3]), Ok(0));
        assert_eq!(buf_lend(0, 1, false), Ok(0));
        assert_eq!(buf_transfer(0, 1), Ok(()));
        assert_eq!(buf_revoke(0, 1), Ok(()));
        let mut dst = [0u8; 16];
        assert_eq!(buf_read(0, &mut dst), Ok(0));
    }

    #[test]
    fn lend_result_fields() {
        let lr = LendResult {
            region_id: 3,
            base_addr: 0x2000_1000,
        };
        assert_eq!(lr.region_id, 3);
        assert_eq!(lr.base_addr, 0x2000_1000);
    }

    #[test]
    fn buf_lend_with_addr_success_on_host() {
        // On host svc_r01! returns (0, 0) — both are valid success values.
        let result = buf_lend_with_addr(0, 1, false);
        let lr = result.expect("should be Ok on host");
        assert_eq!(lr.region_id, 0);
        assert_eq!(lr.base_addr, 0);
    }

    #[test]
    fn buf_lend_with_addr_error_path() {
        // Verify the error-parsing branch with known error codes.
        let raw_err = SvcError::InvalidResource.to_u32();
        assert!(SvcError::is_error(raw_err));
        let parsed = SvcError::from_u32(raw_err);
        assert_eq!(parsed, Some(SvcError::InvalidResource));
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

    #[test]
    fn buf_release_returns_ok_on_host() {
        assert_eq!(buf_release(0), Ok(()));
        assert_eq!(buf_release(255), Ok(()));
        assert_eq!(buf_release(3), Ok(()));
    }

    #[test]
    fn buf_release_parse_result_errors() {
        let raw_err = SvcError::InvalidResource.to_u32();
        let result: Result<(), SvcError> = parse_result(raw_err).map(|_| ());
        assert_eq!(result, Err(SvcError::InvalidResource));

        let raw_op = SvcError::OperationFailed.to_u32();
        let result: Result<(), SvcError> = parse_result(raw_op).map(|_| ());
        assert_eq!(result, Err(SvcError::OperationFailed));
    }

    #[test]
    fn buf_release_round_trip_error_codes() {
        for &err in &[
            SvcError::InvalidResource,
            SvcError::OperationFailed,
            SvcError::InvalidSyscall,
        ] {
            let raw = err.to_u32();
            let result: Result<(), SvcError> = parse_result(raw).map(|_| ());
            assert_eq!(result, Err(err));
        }
    }

    #[test]
    fn buf_lend_with_deadline_zero() {
        // Zero deadline = no deadline, same as buf_lend.
        let result = buf_lend_with_deadline(0, 1, false, 0);
        assert_eq!(result, Ok(0));
    }

    #[test]
    fn buf_lend_with_deadline_nonzero() {
        // Non-zero deadline passes through r3; host stub returns Ok(0).
        let result = buf_lend_with_deadline(2, 3, true, 500);
        assert_eq!(result, Ok(0));
    }

    #[test]
    fn buf_lend_with_deadline_preserves_r2_packing() {
        // Verify r2 packing is identical to buf_lend (writable flag).
        let r2_ro = pack_lend_r2(7, false);
        assert_eq!(r2_ro, 7);
        let r2_rw = pack_lend_r2(7, true);
        assert_eq!(r2_rw, 7 | lend_flags::WRITABLE);
        // Both deadline variants use the same packing.
        assert_eq!(buf_lend_with_deadline(0, 7, false, 100), Ok(0));
        assert_eq!(buf_lend_with_deadline(0, 7, true, 100), Ok(0));
    }

    #[test]
    fn buf_lend_with_addr_deadline_zero() {
        // Zero deadline = no deadline, same as buf_lend_with_addr.
        let result = buf_lend_with_addr_deadline(0, 1, false, 0);
        let lr = result.expect("should be Ok on host");
        assert_eq!(lr.region_id, 0);
        assert_eq!(lr.base_addr, 0);
    }

    #[test]
    fn buf_lend_with_addr_deadline_nonzero() {
        // Non-zero deadline passes through r3; host stub returns Ok((0,0)).
        let result = buf_lend_with_addr_deadline(2, 3, true, 1000);
        let lr = result.expect("should be Ok on host");
        assert_eq!(lr.region_id, 0);
        assert_eq!(lr.base_addr, 0);
    }

    #[test]
    fn buf_lend_with_addr_deadline_error_path() {
        // Exercise the same parse_result().map() chain that
        // buf_lend_with_addr_deadline uses for its error path.
        for &err in &[
            SvcError::InvalidResource,
            SvcError::OperationFailed,
            SvcError::InvalidSyscall,
        ] {
            let raw = err.to_u32();
            let result: Result<LendResult, SvcError> = parse_result(raw).map(|v| LendResult {
                region_id: v as u8,
                base_addr: 0,
            });
            assert_eq!(result, Err(err));
        }
    }
}
