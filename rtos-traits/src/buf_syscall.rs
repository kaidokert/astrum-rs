//! Buffer-pool ABI helpers shared by kernel and partition code.

use crate::syscall::lend_flags;
#[inline]
pub const fn pack_lend_r2(target: u8, writable: bool) -> u32 {
    let flags = if writable { lend_flags::WRITABLE } else { 0 };
    (target as u32) | flags
}
pub use crate::api::decode_rc as parse_result;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LendResult {
    pub region_id: u8,
    pub base_addr: u32,
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::SvcError;

    #[test]
    fn pack_lend_r2_values() {
        assert_eq!(pack_lend_r2(0, false), 0);
        assert_eq!(pack_lend_r2(3, false), 3);
        assert_eq!(pack_lend_r2(5, true), 5 | lend_flags::WRITABLE);
        assert_eq!(pack_lend_r2(255, true), 255 | lend_flags::WRITABLE);
    }

    #[test]
    fn parse_result_delegates() {
        assert_eq!(parse_result(0), Ok(0));
        assert_eq!(parse_result(42), Ok(42));
        let err = SvcError::InvalidResource.to_u32();
        assert_eq!(parse_result(err), Err(SvcError::InvalidResource));
        assert_eq!(parse_result(0x8000_0001), Err(SvcError::InvalidSyscall));
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
}
