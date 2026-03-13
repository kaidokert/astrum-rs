use crate::{buffer_pool::BufferPool, mpu_strategy::DynamicStrategy, svc::SvcError};
#[allow(clippy::too_many_arguments)]
pub fn handle_buffer_lend<const S: usize, const Z: usize>(
    buffers: &mut BufferPool<S, Z>,
    strategy: &DynamicStrategy,
    current_partition: u8,
    partition_count: usize,
    tick: u64,
    r1: u32,
    r2: u32,
    r3: u32,
    legacy_sentinel: u32,
) -> (u32, Option<u32>) {
    let slot = r1 as usize;
    if r2 & 0xFE00 != 0 {
        return (SvcError::OperationFailed.to_u32(), None);
    }
    let target_raw = (r2 & 0xFF) as usize;
    let writable = r2 & rtos_traits::syscall::lend_flags::WRITABLE != 0;
    if target_raw >= partition_count {
        return (SvcError::InvalidPartition.to_u32(), None);
    }
    match buffers.share_with_partition(
        slot,
        current_partition,
        target_raw as u8,
        writable,
        strategy,
    ) {
        Ok(rid) => match buffers.slot_base_address(slot) {
            Some(base) => {
                let r0 = if r3 > 0 && r3 != legacy_sentinel {
                    match buffers.set_deadline(slot, Some(tick.wrapping_add(r3 as u64))) {
                        Ok(()) => base,
                        Err(e) => e.to_svc_error().to_u32(),
                    }
                } else {
                    base
                };
                (r0, Some(rid as u32))
            }
            None => (SvcError::InvalidBuffer.to_u32(), None),
        },
        Err(e) => (e.to_svc_error().to_u32(), None),
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer_pool::BorrowMode;
    #[test]
    fn validate_and_lend() {
        let (mut p, d) = (BufferPool::<4, 32>::new(), DynamicStrategy::new());
        let eop = SvcError::OperationFailed.to_u32();
        let eip = SvcError::InvalidPartition.to_u32();
        let f = |p: &mut BufferPool<4, 32>, r1, r2, r3| {
            handle_buffer_lend(p, &d, 0, 2, 100, r1, r2, r3, 0xCC)
        };
        assert_eq!(f(&mut p, 0, 0x0200, 0), (eop, None));
        assert_eq!(f(&mut p, 0, 5, 0), (eip, None));
        let s = p.alloc(0, BorrowMode::Write).unwrap() as u32;
        let (r0, r1) = f(&mut p, s, 1, 50);
        assert_eq!(r0, p.slot_base_address(s as usize).unwrap());
        assert!(r1.is_some());
    }
}
