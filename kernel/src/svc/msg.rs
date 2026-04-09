use crate::message::{MessagePool, MsgError, RecvOutcome, SendOutcome};
use crate::partition::{PartitionState, PartitionTable};
use crate::svc::{try_transition, SvcError};

/// Outcome of a message send/recv at the SVC dispatch level.
#[derive(Debug, PartialEq, Eq)]
pub enum MsgOutcome {
    /// Syscall completed; contains the SVC return value.
    Done(u32),
    /// Caller has been blocked — dispatcher must deschedule.
    Deschedule(u32),
}

impl MsgOutcome {
    pub fn into_svc_return(self) -> (u32, bool) {
        match self {
            Self::Done(r) => (r, false),
            Self::Deschedule(r) => (r, true),
        }
    }
}

/// Handle a `MsgSend` syscall.
///
/// # Safety
///
/// `ptr` must point to a readable region of at least `len` bytes owned by the
/// calling partition (validated by `check_user_ptr` in the caller).
pub unsafe fn handle_msg_send<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut MessagePool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    queue_id: usize,
    caller: usize,
    ptr: *const u8,
    len: usize,
) -> MsgOutcome {
    // SAFETY: Caller guarantees [ptr, ptr+len) is valid, partition-owned memory
    // verified by check_user_ptr against MPU bounds.
    let data = unsafe { core::slice::from_raw_parts(ptr, len) };
    match pool.send(queue_id, caller, data) {
        Ok(SendOutcome::Delivered {
            wake_receiver: Some(p),
        }) => {
            if !try_transition(pt, p, PartitionState::Ready) {
                return MsgOutcome::Done(SvcError::TransitionFailed.to_u32());
            }
            MsgOutcome::Done(0)
        }
        Ok(SendOutcome::Delivered {
            wake_receiver: None,
        }) => MsgOutcome::Done(0),
        Ok(SendOutcome::SenderBlocked { blocked: p }) => {
            if !try_transition(pt, p, PartitionState::Waiting) {
                return MsgOutcome::Done(SvcError::TransitionFailed.to_u32());
            }
            MsgOutcome::Deschedule(0)
        }
        Err(e) => MsgOutcome::Done(msg_error_to_svc(e)),
    }
}

/// Handle a `MsgRecv` syscall.
///
/// # Safety
///
/// `ptr` must point to a writable region of at least `len` bytes owned by the
/// calling partition (validated by `check_user_ptr` in the caller).
pub unsafe fn handle_msg_recv<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut MessagePool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    queue_id: usize,
    caller: usize,
    ptr: *mut u8,
    len: usize,
) -> MsgOutcome {
    // SAFETY: Caller guarantees [ptr, ptr+len) is valid, partition-owned memory
    // verified by check_user_ptr against MPU bounds.
    let buf = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
    match pool.recv(queue_id, caller, buf) {
        Ok(RecvOutcome::Received {
            wake_sender: Some(p),
        }) => {
            if !try_transition(pt, p, PartitionState::Ready) {
                return MsgOutcome::Done(SvcError::TransitionFailed.to_u32());
            }
            MsgOutcome::Done(0)
        }
        Ok(RecvOutcome::Received { wake_sender: None }) => MsgOutcome::Done(0),
        Ok(RecvOutcome::ReceiverBlocked { blocked: p }) => {
            if !try_transition(pt, p, PartitionState::Waiting) {
                return MsgOutcome::Done(SvcError::TransitionFailed.to_u32());
            }
            MsgOutcome::Deschedule(0)
        }
        Err(e) => MsgOutcome::Done(msg_error_to_svc(e)),
    }
}

/// Map a [`MsgError`] to its SVC return code, preserving diagnostic
/// granularity across the syscall boundary.
pub fn msg_error_to_svc(e: MsgError) -> u32 {
    match e {
        MsgError::InvalidQueue => SvcError::InvalidResource.to_u32(),
        MsgError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        MsgError::SizeMismatch => SvcError::OperationFailed.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::MessageQueue;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};

    fn s() -> (PartitionTable<4>, MessagePool<4, 2, 4, 4>) {
        let mut t = PartitionTable::new();
        for i in 0..2u8 {
            let b = 0x2000_0000 + (i as u32) * 0x1000;
            t.add(PartitionControlBlock::new(
                i,
                0x800_0000,
                b,
                b + 0x400,
                MpuRegion::new(b, 4096, 0),
            ))
            .unwrap();
            t.get_mut(i as usize)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
        }
        let mut p = MessagePool::new();
        p.add(MessageQueue::new()).unwrap();
        (t, p)
    }

    macro_rules! tx {
        ($q:expr,$t:expr,$id:expr,$c:expr,$d:expr) => {
            // SAFETY: test-local stack slices are valid for the duration of the call.
            unsafe { handle_msg_send($q, $t, $id, $c, $d.as_ptr(), $d.len()) }.into_svc_return()
        };
    }
    macro_rules! rx {
        ($q:expr,$t:expr,$id:expr,$c:expr,$b:expr) => {
            // SAFETY: test-local stack slices are valid for the duration of the call.
            unsafe { handle_msg_recv($q, $t, $id, $c, $b.as_mut_ptr(), $b.len()) }.into_svc_return()
        };
    }

    #[test]
    fn send_recv_block_invalid_and_cross_wake() {
        let (mut t, mut q) = s();
        let d = [0xAA, 0xBB, 0xCC, 0xDD];
        assert_eq!(tx!(&mut q, &mut t, 0, 0, &d), (0, false));
        assert_eq!(tx!(&mut q, &mut t, 0, 0, &d), (0, false));
        assert_eq!(tx!(&mut q, &mut t, 0, 0, &d), (0, true));
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        let inv = SvcError::InvalidResource.to_u32();
        assert_eq!(tx!(&mut q, &mut t, 99, 1, &d).0, inv);
        let (mut t, mut q) = s();
        tx!(&mut q, &mut t, 0, 0, &[0xDE, 0xAD, 0xBE, 0xEF]);
        let mut buf = [0u8; 4];
        assert_eq!(rx!(&mut q, &mut t, 0, 1, &mut buf), (0, false));
        assert_eq!(buf, [0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(rx!(&mut q, &mut t, 0, 0, &mut buf), (0, true));
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        assert_eq!(rx!(&mut q, &mut t, 99, 1, &mut buf).0, inv);
        let (mut t, mut q) = s();
        rx!(&mut q, &mut t, 0, 1, &mut [0u8; 4]);
        tx!(&mut q, &mut t, 0, 0, &[1, 2, 3, 4]);
        assert_eq!(t.get(1).unwrap().state(), PartitionState::Ready);
        let (mut t, mut q) = s();
        tx!(&mut q, &mut t, 0, 0, &d);
        tx!(&mut q, &mut t, 0, 0, &d);
        tx!(&mut q, &mut t, 0, 1, &d);
        assert_eq!(t.get(1).unwrap().state(), PartitionState::Waiting);
        let mut b = [0u8; 4];
        assert_eq!(rx!(&mut q, &mut t, 0, 0, &mut b), (0, false));
        assert_eq!(b, d);
        assert_eq!(t.get(1).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn error_mapping() {
        assert_eq!(
            msg_error_to_svc(MsgError::InvalidQueue),
            SvcError::InvalidResource.to_u32()
        );
        assert_eq!(
            msg_error_to_svc(MsgError::WaitQueueFull),
            SvcError::WaitQueueFull.to_u32()
        );
        assert_eq!(
            msg_error_to_svc(MsgError::SizeMismatch),
            SvcError::OperationFailed.to_u32()
        );
    }
}
