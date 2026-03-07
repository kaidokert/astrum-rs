use crate::{sampling::SamplingPortPool, svc::SvcError};

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
    // SAFETY: Caller (validated_ptr!) guarantees [ptr, ptr+len) is valid.
    let data = unsafe { core::slice::from_raw_parts(ptr, len) };
    pool.write_sampling_message(port_id, data, tick)
        .map_or_else(|_| SvcError::InvalidResource.to_u32(), |()| 0)
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
) -> u32 {
    // SAFETY: Caller (validated_ptr!) guarantees [ptr, ptr+buf_len) is valid.
    let buf = unsafe { core::slice::from_raw_parts_mut(ptr, buf_len) };
    pool.read_sampling_message(port_id, buf, tick)
        .map_or_else(|_| SvcError::InvalidResource.to_u32(), |(sz, _)| sz as u32)
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
    unsafe fn rd(p: &mut SamplingPortPool<4, 16>, id: usize, b: &mut [u8]) -> u32 {
        // SAFETY: b is a valid stack-local slice for the call duration.
        unsafe { handle_sampling_read(p, id, b.as_mut_ptr(), b.len(), 1) }
    }

    #[test]
    fn roundtrip_and_errors() {
        let mut p = pool();
        let err = SvcError::InvalidResource.to_u32();
        // SAFETY: all slices are stack-local and valid.
        unsafe {
            assert_eq!(wr(&mut p, 0, &[0xAA, 0xBB, 0xCC]), 0);
            let mut buf = [0u8; 16];
            assert_eq!(rd(&mut p, 1, &mut buf), 3);
            assert_eq!(&buf[..3], &[0xAA, 0xBB, 0xCC]);
            assert_eq!(wr(&mut p, 99, &[1]), err); // invalid port
            assert_eq!(rd(&mut p, 99, &mut buf), err); // invalid port
            assert_eq!(wr(&mut p, 1, &[1]), err); // direction violation
        }
    }
}
