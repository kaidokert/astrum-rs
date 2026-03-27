use crate::blackboard::{BlackboardError, BlackboardPool, ReadBlackboardOutcome};
use crate::partition::{PartitionState, PartitionTable};
use crate::svc::try_transition;

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
    id: usize,
    data: &[u8],
) -> Result<(), BlackboardError> {
    let woken = bb.display_blackboard(id, data)?;
    for &w in woken.iter() {
        // TODO: log wake-transition failure when kernel tracing is available
        let _ = try_transition(pt, w, PartitionState::Ready);
    }
    Ok(())
}

pub fn handle_bb_read<const N: usize, const S: usize, const M: usize, const W: usize>(
    bb: &mut BlackboardPool<S, M, W>,
    pt: &mut PartitionTable<N>,
    id: usize,
    pid: u8,
    buf: &mut [u8],
    timeout: u32,
    tick: u64,
) -> Result<BbReadOutcome, BlackboardError> {
    match bb.read_blackboard_timed(id, pid, buf, timeout, tick)? {
        ReadBlackboardOutcome::Read { msg_len } => Ok(BbReadOutcome::Read {
            msg_len: msg_len as u32,
        }),
        ReadBlackboardOutcome::ReaderBlocked => {
            try_transition(pt, pid, PartitionState::Waiting);
            Ok(BbReadOutcome::Blocked)
        }
    }
}

pub fn handle_bb_clear<const S: usize, const M: usize, const W: usize>(
    bb: &mut BlackboardPool<S, M, W>,
    id: usize,
) -> Result<(), BlackboardError> {
    bb.clear_blackboard(id)
}

/// Map a [`BlackboardError`] to its SVC return code, preserving diagnostic
/// granularity across the syscall boundary.
pub fn bb_error_to_svc(e: BlackboardError) -> u32 {
    use crate::svc::SvcError;
    match e {
        BlackboardError::InvalidBoard => SvcError::InvalidResource.to_u32(),
        BlackboardError::MessageTooLarge => {
            crate::klog!("bb_error_to_svc: MessageTooLarge");
            SvcError::OperationFailed.to_u32()
        }
        BlackboardError::BoardEmpty => {
            crate::klog!("bb_error_to_svc: BoardEmpty");
            SvcError::OperationFailed.to_u32()
        }
        BlackboardError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        BlackboardError::PoolFull => {
            crate::klog!("bb_error_to_svc: PoolFull");
            SvcError::OperationFailed.to_u32()
        }
    }
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock};
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
        assert_eq!(handle_bb_display(&mut b, &mut p, 0, &[0x11, 0x22, 0x33]), Ok(()));
        assert_eq!(handle_bb_display(&mut b, &mut p, 99, &[1]), Err(BlackboardError::InvalidBoard));
        let mut buf = [0u8; 4];
        assert_eq!(handle_bb_read(&mut b, &mut p, 0, 0, &mut buf, 0, 0), Ok(BbReadOutcome::Read { msg_len: 3 }));
        assert_eq!(&buf[..3], [0x11, 0x22, 0x33]);
        assert_eq!(handle_bb_read(&mut b, &mut p, 99, 0, &mut buf, 0, 0), Err(BlackboardError::InvalidBoard));
        assert_eq!(handle_bb_clear(&mut b, 0), Ok(()));
        assert_eq!(handle_bb_clear(&mut b, 99), Err(BlackboardError::InvalidBoard));
        assert_eq!(handle_bb_display(&mut b, &mut p, 0, &[1]), Ok(()));
        assert_eq!(handle_bb_clear(&mut b, 0), Ok(()));
        assert_eq!(handle_bb_read(&mut b, &mut p, 0, 0, &mut buf, 50, 0), Ok(BbReadOutcome::Blocked));
        assert_eq!(p.get(0).unwrap().state(), PartitionState::Waiting);
    }
    #[test]
    fn error_mapping() {
        use crate::svc::SvcError;
        assert_eq!(bb_error_to_svc(BlackboardError::InvalidBoard), SvcError::InvalidResource.to_u32());
        assert_eq!(bb_error_to_svc(BlackboardError::MessageTooLarge), SvcError::OperationFailed.to_u32());
        assert_eq!(bb_error_to_svc(BlackboardError::WaitQueueFull), SvcError::WaitQueueFull.to_u32());
    }
}
