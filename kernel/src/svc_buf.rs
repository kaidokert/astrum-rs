use crate::buffer_pool::{BorrowMode, BorrowState, BufferPool};
use crate::svc::SvcError;

pub fn handle_buf_alloc<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    pid: u8,
    mode_raw: u32,
    max_ticks: u32,
    tick: u64,
) -> u32 {
    let mode = if mode_raw == 0 {
        BorrowMode::Read
    } else {
        BorrowMode::Write
    };
    match pool.alloc(pid, mode) {
        Some(slot) => {
            if max_ticks > 0 {
                let _ = pool.set_deadline(slot, Some(tick.wrapping_add(max_ticks as u64)));
            }
            slot as u32
        }
        None => SvcError::OperationFailed.to_u32(),
    }
}

pub fn handle_buf_release<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    slot: usize,
    pid: u8,
) -> u32 {
    pool.release(slot, pid)
        .map_or_else(|e| e.to_svc_error().to_u32(), |()| 0)
}

/// # Safety
///
/// `ptr` must be valid for `len` bytes.
pub unsafe fn handle_buf_write<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    idx: usize,
    pid: u8,
    ptr: *const u8,
    len: usize,
) -> u32 {
    match pool.get_mut(idx) {
        Some(s) if s.state() == (BorrowState::BorrowedWrite { owner: pid }) => {
            match s.data_mut().get_mut(..len) {
                Some(dst) => {
                    // SAFETY: caller guarantees [ptr, ptr+len) is valid.
                    dst.copy_from_slice(unsafe { core::slice::from_raw_parts(ptr, len) });
                    len as u32
                }
                None => SvcError::OperationFailed.to_u32(),
            }
        }
        Some(_) => SvcError::PermissionDenied.to_u32(),
        None => SvcError::InvalidResource.to_u32(),
    }
}

/// # Safety
///
/// `ptr` must be valid for `len` bytes.
pub unsafe fn handle_buf_read<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    idx: usize,
    pid: u8,
    ptr: *mut u8,
    len: usize,
) -> u32 {
    // SAFETY: caller guarantees [ptr, ptr+len) is valid.
    let dst = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
    pool.read_from_slot(idx, pid, dst)
        .map_or_else(|e| e.to_svc_error().to_u32(), |n| n as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    type P = BufferPool<2, 32>;
    fn alloc(p: &mut P, pid: u8, mode: u32) -> u32 {
        handle_buf_alloc(p, pid, mode, 0, 0)
    }

    #[test]
    fn alloc_release_deadline_exhaustion() {
        let mut p: BufferPool<1, 32> = BufferPool::new();
        assert_eq!(handle_buf_alloc(&mut p, 0, 0, 0, 0), 0);
        assert_eq!(
            handle_buf_alloc(&mut p, 1, 0, 0, 0),
            SvcError::OperationFailed.to_u32()
        );
        assert_eq!(handle_buf_release(&mut p, 0, 0), 0);
        assert_eq!(handle_buf_alloc(&mut p, 0, 1, 100, 50), 0);
        assert_eq!(p.deadline(0), Some(150));
        assert_ne!(handle_buf_release(&mut p, 0, 1), 0); // wrong owner
        assert_ne!(handle_buf_release(&mut p, 99, 0), 0); // bad slot
    }

    #[test]
    fn write_read_roundtrip() {
        let (mut p, data) = (P::new(), [0xAA, 0xBB, 0xCC, 0xDD]);
        alloc(&mut p, 0, 1);
        // SAFETY: stack-local slices valid for the call duration.
        unsafe {
            assert_eq!(handle_buf_write(&mut p, 0, 0, data.as_ptr(), 4), 4);
            let mut out = [0u8; 4];
            assert_eq!(handle_buf_read(&mut p, 0, 0, out.as_mut_ptr(), 4), 4);
            assert_eq!(out, data);
        }
    }

    #[test]
    fn write_read_errors() {
        let mut p = P::new();
        alloc(&mut p, 0, 1);
        alloc(&mut p, 1, 0);
        let perm = SvcError::PermissionDenied.to_u32();
        let inv = SvcError::InvalidResource.to_u32();
        // SAFETY: stack-local slices valid for the call duration.
        unsafe {
            assert_eq!(handle_buf_write(&mut p, 0, 1, [1u8].as_ptr(), 1), perm);
            assert_eq!(handle_buf_write(&mut p, 99, 0, [1u8].as_ptr(), 1), inv);
            assert_eq!(handle_buf_write(&mut p, 1, 1, [1u8].as_ptr(), 1), perm);
            assert_ne!(handle_buf_read(&mut p, 99, 0, [0u8; 4].as_mut_ptr(), 4), 4);
        }
    }
}
