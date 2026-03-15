use crate::sampling::{SamplingError, SamplingPortPool, Validity};
use crate::svc::SvcError;

/// Handle `SamplingWrite`.
///
/// # Safety
/// `ptr` must be valid for `len` bytes (validated by caller).
pub unsafe fn handle_sampling_write<const S: usize, const M: usize>(
    pool: &mut SamplingPortPool<S, M>,
    port_id: usize,
    ptr: *const u8,
    len: usize,
    tick: u64,
) -> u32 {
    // SAFETY: Caller (check_user_ptr) guarantees [ptr, ptr+len) is valid.
    let data = unsafe { core::slice::from_raw_parts(ptr, len) };
    pool.write_sampling_message(port_id, data, tick)
        .map_or_else(sampling_error_to_svc, |()| 0)
}

/// Handle `SamplingRead`.
///
/// # Safety
/// `ptr` must be valid for `buf_len` bytes (validated by caller).
pub unsafe fn handle_sampling_read<const S: usize, const M: usize>(
    pool: &mut SamplingPortPool<S, M>,
    port_id: usize,
    ptr: *mut u8,
    buf_len: usize,
    tick: u64,
) -> Result<(u32, Validity), u32> {
    // SAFETY: Caller (check_user_ptr) guarantees [ptr, ptr+buf_len) is valid.
    let buf = unsafe { core::slice::from_raw_parts_mut(ptr, buf_len) };
    pool.read_sampling_message(port_id, buf, tick)
        .map(|(sz, v)| (sz as u32, v))
        .map_err(sampling_error_to_svc)
}

pub fn sampling_error_to_svc(e: SamplingError) -> u32 {
    match e {
        SamplingError::InvalidPort => SvcError::InvalidResource.to_u32(),
        SamplingError::DirectionViolation => SvcError::OperationFailed.to_u32(),
        SamplingError::MessageTooLarge => SvcError::OperationFailed.to_u32(),
        SamplingError::PoolFull => SvcError::OperationFailed.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sampling::PortDirection::{Destination, Source};

    fn pool() -> SamplingPortPool<4, 16> {
        let mut p = SamplingPortPool::new();
        p.create_port(Source, 1000).unwrap();
        p.create_port(Destination, 1000).unwrap();
        p.connect_ports(0, 1).unwrap();
        p
    }

    /// # Safety
    /// Wraps handle_sampling_write with stack-local data.
    unsafe fn wr(p: &mut SamplingPortPool<4, 16>, id: usize, d: &[u8]) -> u32 {
        // SAFETY: d is a valid stack-local slice for the call duration.
        unsafe { handle_sampling_write(p, id, d.as_ptr(), d.len(), 1) }
    }

    /// # Safety
    /// Wraps handle_sampling_read with stack-local buffer.
    unsafe fn rd(
        p: &mut SamplingPortPool<4, 16>,
        id: usize,
        b: &mut [u8],
        tick: u64,
    ) -> Result<(u32, Validity), u32> {
        // SAFETY: b is a valid stack-local slice for the call duration.
        unsafe { handle_sampling_read(p, id, b.as_mut_ptr(), b.len(), tick) }
    }

    #[test]
    fn roundtrip_validity_and_errors() {
        let mut p = pool();
        let inv = SvcError::InvalidResource.to_u32();
        let op = SvcError::OperationFailed.to_u32();
        // SAFETY: all slices are stack-local and valid.
        unsafe {
            assert_eq!(wr(&mut p, 0, &[0xAA, 0xBB, 0xCC]), 0);
            let mut buf = [0u8; 16];
            assert_eq!(rd(&mut p, 1, &mut buf, 1), Ok((3, Validity::Valid)));
            assert_eq!(&buf[..3], &[0xAA, 0xBB, 0xCC]);
            assert_eq!(wr(&mut p, 99, &[1]), inv); // invalid port
            assert_eq!(rd(&mut p, 99, &mut buf, 1), Err(inv));
            assert_eq!(wr(&mut p, 1, &[1]), op); // direction violation
            assert_eq!(rd(&mut p, 1, &mut [0u8; 2], 1), Err(op)); // buf too small
            assert_eq!(wr(&mut p, 0, &[1]), 0);
            assert_eq!(rd(&mut p, 1, &mut buf, 500), Ok((1, Validity::Valid)));
            assert_eq!(rd(&mut p, 1, &mut buf, 2000), Ok((1, Validity::Invalid)));
        }
        assert_eq!(sampling_error_to_svc(SamplingError::InvalidPort), inv);
        assert_eq!(sampling_error_to_svc(SamplingError::DirectionViolation), op);
        assert_eq!(sampling_error_to_svc(SamplingError::MessageTooLarge), op);
        assert_eq!(sampling_error_to_svc(SamplingError::PoolFull), op);
    }
}
