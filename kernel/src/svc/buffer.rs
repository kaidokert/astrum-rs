use crate::{buffer_pool::BufferPool, mpu_strategy::DynamicStrategy, svc::SvcError, PartitionId};
use rtos_traits::ids::BufferSlotId;
#[allow(clippy::too_many_arguments)]
pub fn handle_buffer_lend<const S: usize, const Z: usize>(
    buffers: &mut BufferPool<S, Z>,
    strategy: &DynamicStrategy,
    current_partition: PartitionId,
    partition_count: usize,
    tick: u64,
    slot_id: BufferSlotId,
    r2: u32,
    r3: u32,
    legacy_sentinel: u32,
) -> (u32, Option<u32>) {
    let slot = slot_id.as_raw() as usize;
    if r2 & 0xFE00 != 0 {
        return (SvcError::OperationFailed.to_u32(), Some(0xFE));
    }
    let target_raw = (r2 & 0xFF) as usize;
    let writable = r2 & rtos_traits::syscall::lend_flags::WRITABLE != 0;
    if target_raw >= partition_count {
        return (SvcError::InvalidPartition.to_u32(), Some(0xFD));
    }
    let target = PartitionId::new(target_raw as u32);
    match buffers.share_with_partition(slot, current_partition, target, writable, strategy) {
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
            None => (SvcError::InvalidBuffer.to_u32(), Some(0xFC)),
        },
        Err(e) => (e.to_svc_error().to_u32(), Some(e.discriminant())),
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer_pool::BorrowMode;
    fn pid(v: u8) -> PartitionId {
        PartitionId::new(v as u32)
    }
    #[test]
    fn validate_and_lend() {
        let (mut p, d) = (BufferPool::<4, 32>::new(), DynamicStrategy::new());
        let eop = SvcError::OperationFailed.to_u32();
        let eip = SvcError::InvalidPartition.to_u32();
        let f = |p: &mut BufferPool<4, 32>, slot: BufferSlotId, r2, r3| {
            handle_buffer_lend(p, &d, pid(0), 2, 100, slot, r2, r3, 0xCC)
        };
        assert_eq!(
            f(&mut p, BufferSlotId::new(0), 0x0200, 0),
            (eop, Some(0xFE))
        );
        assert_eq!(f(&mut p, BufferSlotId::new(0), 5, 0), (eip, Some(0xFD)));
        let s = p.alloc(pid(0), BorrowMode::Write).unwrap();
        let sid = BufferSlotId::new(s as u8);
        let (r0, r1) = f(&mut p, sid, 1, 50);
        assert_eq!(r0, p.slot_base_address(s).unwrap());
        assert!(r1.is_some());
        // Self-lend (partition 0 → partition 0) must return SelfLend discriminant
        use crate::buffer_pool::BufferError;
        let (r0, r1) = handle_buffer_lend(&mut p, &d, pid(0), 2, 100, sid, 0, 0, 0xCC);
        assert_eq!(r0, SvcError::OperationFailed.to_u32());
        assert_eq!(r1, Some(BufferError::SelfLend.discriminant()));
    }
}
