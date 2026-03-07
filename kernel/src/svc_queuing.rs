use crate::partition::{PartitionState, PartitionTable};
use crate::queuing::{
    QueuingError, QueuingPortPool, QueuingPortStatus, RecvQueuingOutcome, SendQueuingOutcome,
};
use crate::svc::{try_transition, unpack_packed_r2, SvcError};

#[derive(Debug, PartialEq, Eq)]
pub enum QueuingOutcome {
    Done(u32),
    Deschedule,
}

/// Map a [`QueuingError`] to its SVC return code, preserving diagnostic
/// granularity across the syscall boundary.
pub fn queuing_error_to_svc(e: QueuingError) -> u32 {
    match e {
        QueuingError::InvalidPort => SvcError::InvalidResource.to_u32(),
        QueuingError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        QueuingError::QueueFull
        | QueuingError::QueueEmpty
        | QueuingError::DirectionViolation
        | QueuingError::MessageTooLarge
        | QueuingError::PoolFull => SvcError::OperationFailed.to_u32(),
    }
}

/// # Safety
/// `ptr` must be valid for `len` bytes (validated by caller).
#[allow(clippy::too_many_arguments)]
pub unsafe fn handle_queuing_send<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut QueuingPortPool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    port_id: usize,
    ptr: *const u8,
    len: usize,
    pid: u8,
    timeout: u64,
    tick: u64,
) -> QueuingOutcome {
    // SAFETY: Caller guarantees [ptr, ptr+len) is valid, partition-owned memory.
    let data = unsafe { core::slice::from_raw_parts(ptr, len) };
    match pool.send_routed(port_id, pid, data, timeout, tick) {
        Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
            if let Some(wpid) = w {
                try_transition(pt, wpid, PartitionState::Ready);
            }
            QueuingOutcome::Done(0)
        }
        Ok(SendQueuingOutcome::SenderBlocked { .. }) => {
            try_transition(pt, pid, PartitionState::Waiting);
            QueuingOutcome::Deschedule
        }
        Err(e) => QueuingOutcome::Done(queuing_error_to_svc(e)),
    }
}

/// # Safety
/// `ptr` must be valid for `buf_len` bytes (validated by caller).
#[allow(clippy::too_many_arguments)]
pub unsafe fn handle_queuing_recv<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut QueuingPortPool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    port_id: usize,
    ptr: *mut u8,
    buf_len: usize,
    pid: u8,
    timeout: u64,
    tick: u64,
) -> QueuingOutcome {
    // SAFETY: Caller guarantees [ptr, ptr+buf_len) is valid, partition-owned memory.
    let buf = unsafe { core::slice::from_raw_parts_mut(ptr, buf_len) };
    match pool.receive_queuing_message(port_id, pid, buf, timeout, tick) {
        Ok(RecvQueuingOutcome::Received {
            msg_len,
            wake_sender: w,
        }) => {
            if let Some(wpid) = w {
                try_transition(pt, wpid, PartitionState::Ready);
            }
            QueuingOutcome::Done(msg_len as u32)
        }
        Ok(RecvQueuingOutcome::ReceiverBlocked { .. }) => {
            try_transition(pt, pid, PartitionState::Waiting);
            QueuingOutcome::Deschedule
        }
        Err(e) => QueuingOutcome::Done(queuing_error_to_svc(e)),
    }
}

pub fn handle_queuing_status<const S: usize, const D: usize, const M: usize, const W: usize>(
    pool: &QueuingPortPool<S, D, M, W>,
    port_id: usize,
) -> Result<QueuingPortStatus, u32> {
    pool.get_queuing_port_status(port_id)
        .map_err(|_| SvcError::InvalidResource.to_u32())
}

/// # Safety
/// `ptr` must be valid for the low-16 length encoded in `packed_r2`.
#[allow(clippy::too_many_arguments)]
pub unsafe fn handle_queuing_send_timed<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut QueuingPortPool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    port_id: usize,
    ptr: *const u8,
    packed_r2: u32,
    pid: u8,
    tick: u64,
) -> QueuingOutcome {
    let (to, len) = unpack_packed_r2(packed_r2);
    // SAFETY: Caller guarantees [ptr, ptr+len) is valid, partition-owned memory.
    unsafe { handle_queuing_send(pool, pt, port_id, ptr, len as usize, pid, to as u64, tick) }
}

/// # Safety
/// `ptr` must be valid for `min(low16(packed_r2), max_msg)` bytes.
#[allow(clippy::too_many_arguments)]
pub unsafe fn handle_queuing_recv_timed<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut QueuingPortPool<S, D, M, W>,
    pt: &mut PartitionTable<N>,
    port_id: usize,
    ptr: *mut u8,
    packed_r2: u32,
    max_msg: usize,
    pid: u8,
    tick: u64,
) -> QueuingOutcome {
    let (to, bl) = unpack_packed_r2(packed_r2);
    let rlen = core::cmp::min(bl as usize, max_msg);
    // SAFETY: Caller guarantees [ptr, ptr+rlen) is valid, partition-owned memory.
    unsafe { handle_queuing_recv(pool, pt, port_id, ptr, rlen, pid, to as u64, tick) }
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use super::*;
    use crate::queuing::QueuingPortPool;
    use crate::sampling::PortDirection;
    use crate::partition::{MpuRegion, PartitionControlBlock};

    type Pool = QueuingPortPool<4, 4, 16, 4>;
    type Pt = PartitionTable<4>;
    const OK: QueuingOutcome = QueuingOutcome::Done(0);

    fn setup() -> (Pool, Pt) {
        let mut pool = Pool::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        let d = pool.create_port(PortDirection::Destination).unwrap();
        pool.connect_ports(s, d).unwrap();
        let mut pt = Pt::new();
        pt.add(PartitionControlBlock::new(0, 0x0800_0000, 0x2000_0000, 0x2000_0400, MpuRegion::new(0x2000_0000, 4096, 0))).unwrap();
        pt.get_mut(0).unwrap().transition(PartitionState::Running).unwrap();
        (pool, pt)
    }

    macro_rules! snd {($p:expr,$t:expr,$id:expr,$d:expr,$to:expr) => {
        // SAFETY: pointers are stack-local slices valid for call duration.
        unsafe { handle_queuing_send($p,$t,$id,($d).as_ptr(),($d).len(),0,$to,1) }
    }}
    macro_rules! rcv {($p:expr,$t:expr,$id:expr,$b:expr,$to:expr) => {
        // SAFETY: pointers are stack-local slices valid for call duration.
        unsafe { handle_queuing_recv($p,$t,$id,($b).as_mut_ptr(),($b).len(),0,$to,1) }
    }}

    #[test]
    fn send_recv_roundtrip() {
        let (mut p, mut t) = setup();
        assert_eq!(snd!(&mut p, &mut t, 0, [0xDE, 0xAD, 0xBE], 0), OK);
        assert_eq!(p.get(1).unwrap().nb_messages(), 1);
        let mut buf = [0u8; 16];
        assert_eq!(rcv!(&mut p, &mut t, 1, buf, 0), QueuingOutcome::Done(3));
        assert_eq!(&buf[..3], &[0xDE, 0xAD, 0xBE]);
    }

    #[test]
    fn nonblocking_empty_returns_error() {
        let (mut p, mut t) = setup();
        let mut buf = [0u8; 16];
        let eop = QueuingOutcome::Done(SvcError::OperationFailed.to_u32());
        assert_eq!(rcv!(&mut p, &mut t, 1, buf, 0), eop);
    }

    #[test]
    fn nonblocking_full_returns_error() {
        let (mut p, mut t) = setup();
        let eop = QueuingOutcome::Done(SvcError::OperationFailed.to_u32());
        for i in 0..4u8 { assert_eq!(snd!(&mut p, &mut t, 0, [i], 0), OK); }
        assert_eq!(p.get(1).unwrap().nb_messages(), 4);
        assert_eq!(snd!(&mut p, &mut t, 0, [0xFF], 0), eop);
    }

    #[test]
    fn invalid_port_and_direction_errors() {
        let (mut p, mut t) = setup();
        let inv = QueuingOutcome::Done(SvcError::InvalidResource.to_u32());
        let eop = QueuingOutcome::Done(SvcError::OperationFailed.to_u32());
        let mut buf = [0u8; 16];
        assert_eq!(snd!(&mut p, &mut t, 99, [1u8], 0), inv);
        assert_eq!(rcv!(&mut p, &mut t, 99, buf, 0), inv);
        assert_eq!(snd!(&mut p, &mut t, 1, [1u8], 0), eop); // direction violation
    }

    #[test]
    fn blocking_send_returns_deschedule() {
        let (mut p, mut t) = setup();
        for i in 0..4u8 { assert_eq!(snd!(&mut p, &mut t, 0, [i], 0), OK); }
        assert_eq!(snd!(&mut p, &mut t, 0, [0xFF], 50), QueuingOutcome::Deschedule);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
    }

    #[test]
    fn blocking_recv_returns_deschedule() {
        let (mut p, mut t) = setup();
        let mut buf = [0u8; 16];
        assert_eq!(rcv!(&mut p, &mut t, 1, buf, 50), QueuingOutcome::Deschedule);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
    }

    #[test]
    fn error_mapping() {
        assert_eq!(queuing_error_to_svc(QueuingError::InvalidPort), SvcError::InvalidResource.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::DirectionViolation), SvcError::OperationFailed.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::MessageTooLarge), SvcError::OperationFailed.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::QueueFull), SvcError::OperationFailed.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::QueueEmpty), SvcError::OperationFailed.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::WaitQueueFull), SvcError::WaitQueueFull.to_u32());
        assert_eq!(queuing_error_to_svc(QueuingError::PoolFull), SvcError::OperationFailed.to_u32());
    }

    fn pack_r2(timeout: u16, len: u16) -> u32 { ((timeout as u32) << 16) | (len as u32) }
    macro_rules! snd_t {($p:expr,$t:expr,$id:expr,$d:expr,$pr2:expr) => {
        // SAFETY: pointers are stack-local slices valid for call duration.
        unsafe { handle_queuing_send_timed($p,$t,$id,($d).as_ptr(),$pr2,0,1) }
    }}
    macro_rules! rcv_t {($p:expr,$t:expr,$id:expr,$b:expr,$pr2:expr,$mx:expr) => {
        // SAFETY: pointers are stack-local slices valid for call duration.
        unsafe { handle_queuing_recv_timed($p,$t,$id,($b).as_mut_ptr(),$pr2,$mx,0,1) }
    }}

    #[test]
    fn status_valid_port() {
        let (p, _t) = setup();
        let s = handle_queuing_status(&p, 0).unwrap();
        assert_eq!(s.nb_messages, 0);
    }

    #[test]
    fn status_invalid_port() {
        let (p, _t) = setup();
        assert_eq!(handle_queuing_status(&p, 99), Err(SvcError::InvalidResource.to_u32()));
    }
    #[test]
    fn send_timed_roundtrip() {
        let (mut p, mut t) = setup();
        assert_eq!(snd_t!(&mut p, &mut t, 0, [0xAB, 0xCD, 0xEF], pack_r2(10, 3)), OK);
        let mut buf = [0u8; 16];
        assert_eq!(rcv!(&mut p, &mut t, 1, buf, 0), QueuingOutcome::Done(3));
        assert_eq!(&buf[..3], [0xAB, 0xCD, 0xEF]);
    }
    #[test]
    fn send_timed_timeout_zero_full() {
        let (mut p, mut t) = setup();
        let eop = QueuingOutcome::Done(SvcError::OperationFailed.to_u32());
        for i in 0..4u8 { assert_eq!(snd!(&mut p, &mut t, 0, [i], 0), OK); }
        assert_eq!(snd_t!(&mut p, &mut t, 0, [0xFF], pack_r2(0, 1)), eop);
    }
    #[test]
    fn send_timed_error() {
        let (mut p, mut t) = setup();
        let inv = QueuingOutcome::Done(SvcError::InvalidResource.to_u32());
        assert_eq!(snd_t!(&mut p, &mut t, 99, [1u8], pack_r2(0, 1)), inv);
    }
    #[test]
    fn recv_timed_roundtrip() {
        let (mut p, mut t) = setup();
        assert_eq!(snd!(&mut p, &mut t, 0, [0xFE, 0xED], 0), OK);
        let mut buf = [0u8; 16];
        assert_eq!(rcv_t!(&mut p, &mut t, 1, buf, pack_r2(10, 16), 16), QueuingOutcome::Done(2));
        assert_eq!(&buf[..2], [0xFE, 0xED]);
    }
    #[test]
    fn recv_timed_timeout_zero_empty() {
        let (mut p, mut t) = setup();
        let eop = QueuingOutcome::Done(SvcError::OperationFailed.to_u32());
        let mut buf = [0u8; 16];
        assert_eq!(rcv_t!(&mut p, &mut t, 1, buf, pack_r2(0, 16), 16), eop);
    }
    #[test]
    fn recv_timed_error() {
        let (mut p, mut t) = setup();
        let mut buf = [0u8; 16];
        let inv = QueuingOutcome::Done(SvcError::InvalidResource.to_u32());
        assert_eq!(rcv_t!(&mut p, &mut t, 99, buf, pack_r2(0, 16), 16), inv);
    }
}
