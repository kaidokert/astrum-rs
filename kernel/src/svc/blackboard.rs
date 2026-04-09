use crate::blackboard::{BlackboardError, BlackboardPool, ReadBlackboardOutcome};
use crate::partition::{PartitionState, PartitionTable};
use crate::svc::try_transition;
use rtos_traits::ids::{BlackboardId, PartitionId};

/// Outcome of a blackboard read at the SVC handler level.
#[derive(Debug, PartialEq, Eq)]
pub enum BbReadOutcome {
    /// Data was available; contains the byte count copied into the caller's buffer.
    Read { msg_len: u32 },
    /// Board was empty and the caller has been enqueued as a waiting reader.
    Blocked,
}

pub fn handle_bb_display<const N: usize, const S: usize, const M: usize, const W: usize>(
    bb: &mut BlackboardPool<S, M, W>,
    pt: &mut PartitionTable<N>,
    id: BlackboardId,
    data: &[u8],
) -> Result<(), BlackboardError> {
    let woken = bb.display_blackboard(id.as_raw() as usize, data)?;
    for &w in woken.iter() {
        // TODO: log wake-transition failure when kernel tracing is available
        let _ = try_transition(pt, w, PartitionState::Ready);
    }
    Ok(())
}

pub fn handle_bb_read<const N: usize, const S: usize, const M: usize, const W: usize>(
    bb: &mut BlackboardPool<S, M, W>,
    pt: &mut PartitionTable<N>,
    id: BlackboardId,
    pid: PartitionId,
    buf: &mut [u8],
    timeout: u32,
    tick: u64,
) -> Result<BbReadOutcome, BlackboardError> {
    match bb.read_blackboard_timed(id.as_raw() as usize, pid, buf, timeout, tick)? {
        ReadBlackboardOutcome::Read { msg_len } => Ok(BbReadOutcome::Read {
            msg_len: msg_len as u32,
        }),
        ReadBlackboardOutcome::ReaderBlocked => {
            // TODO: log transition failure when kernel tracing is available
            let _ = try_transition(pt, pid, PartitionState::Waiting);
            Ok(BbReadOutcome::Blocked)
        }
    }
}

pub fn handle_bb_clear<const S: usize, const M: usize, const W: usize>(
    bb: &mut BlackboardPool<S, M, W>,
    id: BlackboardId,
) -> Result<(), BlackboardError> {
    bb.clear_blackboard(id.as_raw() as usize)
}

/// Map a [`BlackboardError`] to its SVC return code, preserving diagnostic
/// granularity across the syscall boundary.
pub fn bb_error_to_svc(e: BlackboardError) -> u32 {
    use crate::svc::SvcError;
    match e {
        BlackboardError::InvalidBoard => SvcError::InvalidResource.to_u32(),
        BlackboardError::MessageTooLarge => SvcError::OperationFailed.to_u32(),
        BlackboardError::BoardEmpty => SvcError::OperationFailed.to_u32(),
        BlackboardError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        BlackboardError::PoolFull => SvcError::OperationFailed.to_u32(),
    }
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock};
    fn pid(v: u32) -> PartitionId { PartitionId::new(v) }
    fn setup() -> (BlackboardPool<4, 4, 4>, PartitionTable<4>) {
        let mut b = BlackboardPool::new();
        b.create().unwrap();
        let mut t = PartitionTable::new();
        t.add(PartitionControlBlock::new(0, 0x0800_0000, 0x2000_0000, 0x2000_0400, MpuRegion::new(0x2000_0000, 4096, 0))).unwrap();
        t.get_mut(0).unwrap().transition(PartitionState::Running).unwrap();
        (b, t)
    }
    #[test]
    fn display_read_clear_handlers() {
        let (mut b, mut p) = setup();
        let id0 = BlackboardId::new(0);
        let id99 = BlackboardId::new(99);
        assert_eq!(handle_bb_display(&mut b, &mut p, id0, &[0x11, 0x22, 0x33]), Ok(()));
        assert_eq!(handle_bb_display(&mut b, &mut p, id99, &[1]), Err(BlackboardError::InvalidBoard));
        let mut buf = [0u8; 4];
        assert_eq!(handle_bb_read(&mut b, &mut p, id0, pid(0), &mut buf, 0, 0), Ok(BbReadOutcome::Read { msg_len: 3 }));
        assert_eq!(&buf[..3], [0x11, 0x22, 0x33]);
        assert_eq!(handle_bb_read(&mut b, &mut p, id99, pid(0), &mut buf, 0, 0), Err(BlackboardError::InvalidBoard));
        assert_eq!(handle_bb_clear(&mut b, id0), Ok(()));
        assert_eq!(handle_bb_clear(&mut b, id99), Err(BlackboardError::InvalidBoard));
        assert_eq!(handle_bb_display(&mut b, &mut p, id0, &[1]), Ok(()));
        assert_eq!(handle_bb_clear(&mut b, id0), Ok(()));
        assert_eq!(handle_bb_read(&mut b, &mut p, id0, pid(0), &mut buf, 50, 0), Ok(BbReadOutcome::Blocked));
        assert_eq!(p.get(0).unwrap().state(), PartitionState::Waiting);
    }
    #[test]
    fn blackboard_id_round_trip() {
        let (mut b, mut p) = setup();
        // Construct BlackboardId from raw u32 and verify it indexes correctly
        let raw: u32 = 0;
        let id = BlackboardId::new(raw);
        assert_eq!(id.as_raw(), raw);
        // Display via BlackboardId, read back, verify data integrity
        assert_eq!(handle_bb_display(&mut b, &mut p, id, &[0xAA, 0xBB]), Ok(()));
        let mut buf = [0u8; 4];
        assert_eq!(
            handle_bb_read(&mut b, &mut p, id, pid(0), &mut buf, 0, 0),
            Ok(BbReadOutcome::Read { msg_len: 2 })
        );
        assert_eq!(&buf[..2], [0xAA, 0xBB]);
        // Clear via BlackboardId then verify read blocks (board is empty)
        assert_eq!(handle_bb_clear(&mut b, id), Ok(()));
        // Re-transition P0 back to Running for the blocking read
        p.get_mut(0).unwrap().transition(PartitionState::Ready).unwrap();
        p.get_mut(0).unwrap().transition(PartitionState::Running).unwrap();
        assert_eq!(
            handle_bb_read(&mut b, &mut p, id, pid(0), &mut buf, 10, 0),
            Ok(BbReadOutcome::Blocked)
        );
    }
    #[test]
    fn error_mapping() {
        use crate::svc::SvcError;
        assert_eq!(bb_error_to_svc(BlackboardError::InvalidBoard), SvcError::InvalidResource.to_u32());
        assert_eq!(bb_error_to_svc(BlackboardError::MessageTooLarge), SvcError::OperationFailed.to_u32());
        assert_eq!(bb_error_to_svc(BlackboardError::WaitQueueFull), SvcError::WaitQueueFull.to_u32());
    }
}
