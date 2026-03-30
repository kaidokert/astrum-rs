use crate::buffer_pool::{BorrowMode, BorrowState, BufferPool};
use crate::svc::SvcError;
use crate::PartitionId;
use rtos_traits::ids::BufferSlotId;

pub fn handle_buf_alloc<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    pid: PartitionId,
    mode_raw: u32,
    max_ticks: u32,
    tick: u64,
) -> (u32, Option<u32>) {
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
            (slot as u32, None)
        }
        None => (SvcError::OperationFailed.to_u32(), None),
    }
}

pub fn handle_buf_release<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    slot: BufferSlotId,
    pid: PartitionId,
) -> (u32, Option<u32>) {
    pool.release(slot.as_raw() as usize, pid).map_or_else(
        |e| (e.to_svc_error().to_u32(), Some(e.discriminant())),
        |()| (0, None),
    )
}

/// # Safety
///
/// `ptr` must be valid for `len` bytes.
pub unsafe fn handle_buf_write<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    idx: BufferSlotId,
    pid: PartitionId,
    ptr: *const u8,
    len: usize,
) -> (u32, Option<u32>) {
    match pool.get_mut(idx.as_raw() as usize) {
        Some(s) if s.state() == (BorrowState::BorrowedWrite { owner: pid }) => {
            match s.data_mut().get_mut(..len) {
                Some(dst) => {
                    // SAFETY: caller guarantees [ptr, ptr+len) is valid.
                    dst.copy_from_slice(unsafe { core::slice::from_raw_parts(ptr, len) });
                    (len as u32, None)
                }
                None => (SvcError::OperationFailed.to_u32(), None),
            }
        }
        Some(_) => (SvcError::PermissionDenied.to_u32(), None),
        None => (SvcError::InvalidResource.to_u32(), None),
    }
}

/// # Safety
///
/// `ptr` must be valid for `len` bytes.
pub unsafe fn handle_buf_read<const S: usize, const Z: usize>(
    pool: &mut BufferPool<S, Z>,
    idx: BufferSlotId,
    pid: PartitionId,
    ptr: *mut u8,
    len: usize,
) -> (u32, Option<u32>) {
    // SAFETY: caller guarantees [ptr, ptr+len) is valid.
    let dst = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
    pool.read_from_slot(idx.as_raw() as usize, pid, dst)
        .map_or_else(
            |e| (e.to_svc_error().to_u32(), Some(e.discriminant())),
            |n| (n as u32, None),
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer_pool::{BufferError, BufferPoolError};
    type P = BufferPool<2, 32>;
    fn pid(v: u8) -> PartitionId {
        PartitionId::new(v as u32)
    }
    fn alloc(p: &mut P, pid: PartitionId, mode: u32) -> (u32, Option<u32>) {
        handle_buf_alloc(p, pid, mode, 0, 0)
    }

    #[test]
    fn alloc_release_deadline_exhaustion() {
        let mut p: BufferPool<1, 32> = BufferPool::new();
        assert_eq!(handle_buf_alloc(&mut p, pid(0), 0, 0, 0), (0, None));
        assert_eq!(
            handle_buf_alloc(&mut p, pid(1), 0, 0, 0).0,
            SvcError::OperationFailed.to_u32()
        );
        assert_eq!(
            handle_buf_release(&mut p, BufferSlotId::new(0), pid(0)),
            (0, None)
        );
        assert_eq!(handle_buf_alloc(&mut p, pid(0), 1, 100, 50), (0, None));
        // TODO: reviewer false positive – deadline() takes a slot index (usize), not a PartitionId
        assert_eq!(p.deadline(0), Some(150));
        // wrong owner → NotOwner
        let (r0, r1) = handle_buf_release(&mut p, BufferSlotId::new(0), pid(1));
        assert_ne!(r0, 0);
        assert_eq!(r1, Some(BufferPoolError::NotOwner.discriminant()));
        // bad slot → InvalidSlot
        let (r0, r1) = handle_buf_release(&mut p, BufferSlotId::new(99), pid(0));
        assert_ne!(r0, 0);
        assert_eq!(r1, Some(BufferPoolError::InvalidSlot.discriminant()));
    }

    #[test]
    fn write_read_roundtrip() {
        let (mut p, data) = (P::new(), [0xAA, 0xBB, 0xCC, 0xDD]);
        alloc(&mut p, pid(0), 1);
        // SAFETY: stack-local slices valid for the call duration.
        unsafe {
            assert_eq!(
                handle_buf_write(&mut p, BufferSlotId::new(0), pid(0), data.as_ptr(), 4),
                (4, None)
            );
            let mut out = [0u8; 4];
            assert_eq!(
                handle_buf_read(&mut p, BufferSlotId::new(0), pid(0), out.as_mut_ptr(), 4),
                (4, None)
            );
            assert_eq!(out, data);
        }
    }

    #[test]
    fn slot_id_roundtrip() {
        let (mut p, data) = (P::new(), [0x11u8, 0x22, 0x33]);
        let id = BufferSlotId::new(handle_buf_alloc(&mut p, pid(0), 1, 0, 0).0 as u8);
        // SAFETY: stack-local slices valid for the call duration.
        unsafe { assert_eq!(handle_buf_write(&mut p, id, pid(0), data.as_ptr(), 3).0, 3) };
        let mut out = [0u8; 3];
        // SAFETY: stack-local slices valid for the call duration.
        unsafe {
            assert_eq!(
                handle_buf_read(&mut p, id, pid(0), out.as_mut_ptr(), 3).0,
                3
            )
        };
        assert_eq!(out, data);
        assert_eq!(handle_buf_release(&mut p, id, pid(0)), (0, None));
    }

    #[test]
    fn write_read_errors() {
        let mut p = P::new();
        alloc(&mut p, pid(0), 1);
        alloc(&mut p, pid(1), 0);
        let perm = SvcError::PermissionDenied.to_u32();
        let inv = SvcError::InvalidResource.to_u32();
        // SAFETY: stack-local slices valid for the call duration.
        unsafe {
            assert_eq!(
                handle_buf_write(&mut p, BufferSlotId::new(0), pid(1), [1u8].as_ptr(), 1).0,
                perm
            );
            assert_eq!(
                handle_buf_write(&mut p, BufferSlotId::new(99), pid(0), [1u8].as_ptr(), 1).0,
                inv
            );
            assert_eq!(
                handle_buf_write(&mut p, BufferSlotId::new(1), pid(1), [1u8].as_ptr(), 1).0,
                perm
            );
            // read from invalid slot → error with discriminant
            let (r0, r1) = handle_buf_read(
                &mut p,
                BufferSlotId::new(99),
                pid(0),
                [0u8; 4].as_mut_ptr(),
                4,
            );
            assert_eq!(r0, SvcError::InvalidResource.to_u32());
            assert_eq!(r1, Some(BufferError::InvalidSlot.discriminant()));
        }
    }
}
