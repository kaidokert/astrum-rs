use crate::partition::{PartitionState, PartitionTable};
use crate::queuing::{QueuingError, QueuingPortPool, SendQueuingOutcome};
use crate::svc::{try_transition, SvcError};

// TODO: handle_queuing_send carries 5 generic parameters from QueuingPortPool.
// Consider a trait or simplified pool view if this pattern repeats for other handlers.
pub fn handle_queuing_send<
    const N: usize,
    const S: usize,
    const D: usize,
    const M: usize,
    const W: usize,
>(
    pool: &mut QueuingPortPool<S, D, M, W>,
    partitions: &mut PartitionTable<N>,
    current_partition: u8,
    tick: u64,
    port_id: usize,
    data: &[u8],
) -> u32 {
    match pool.send_routed(port_id, current_partition, data, 0, tick) {
        Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
            if let Some(wpid) = w {
                try_transition(partitions, wpid, PartitionState::Ready);
            }
            0
        }
        Ok(SendQueuingOutcome::SenderBlocked { .. }) => 0,
        Err(QueuingError::QueueFull) => 0,
        Err(_) => SvcError::InvalidResource.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock};
    use crate::queuing::RecvQueuingOutcome;
    use crate::sampling::PortDirection;

    #[rustfmt::skip]
    fn make_pool_and_pt() -> (QueuingPortPool<4, 2, 64, 2>, PartitionTable<4>) {
        let mut pt = PartitionTable::new();
        for i in 0..2u8 {
            let b = 0x2000_0000 + (i as u32) * 0x1000;
            pt.add(PartitionControlBlock::new(i, 0x800_0000, b, b + 0x400, MpuRegion::new(b, 4096, 0))).unwrap();
            pt.get_mut(i as usize).unwrap().transition(PartitionState::Running).unwrap();
        }
        (QueuingPortPool::new(), pt)
    }

    #[rustfmt::skip]
    fn add_route(p: &mut QueuingPortPool<4, 2, 64, 2>) -> (usize, usize) {
        let (s, d) = (p.create_port(PortDirection::Source).unwrap(), p.create_port(PortDirection::Destination).unwrap());
        p.connect_ports(s, d).unwrap(); (s, d)
    }

    #[test]
    fn invalid_port_returns_error() {
        let (mut pool, mut pt) = make_pool_and_pt();
        let r = handle_queuing_send(&mut pool, &mut pt, 0, 100, 99, &[1, 2, 3]);
        assert_eq!(r, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn delivered_without_wake() {
        let (mut pool, mut pt) = make_pool_and_pt();
        let (src, dst) = add_route(&mut pool);
        let r = handle_queuing_send(&mut pool, &mut pt, 0, 100, src, &[0xAA; 8]);
        assert_eq!(r, 0);
        assert_eq!(pt.get(1).unwrap().state(), PartitionState::Running);
        let mut buf = [0u8; 64];
        let recv = pool
            .receive_queuing_message(dst, 1, &mut buf, 0, 0)
            .unwrap();
        assert!(matches!(
            recv,
            RecvQueuingOutcome::Received { msg_len: 8, .. }
        ));
        assert_eq!(&buf[..8], &[0xAA; 8]);
    }

    #[test]
    #[rustfmt::skip]
    fn delivered_with_wake() {
        let (mut pool, mut pt) = make_pool_and_pt();
        let (src, dst) = add_route(&mut pool);
        pt.get_mut(1).unwrap().transition(PartitionState::Waiting).unwrap();
        // TODO: uses internal pool API to set up blocked-receiver state; consider
        // a higher-level test helper if this coupling becomes fragile.
        pool.get_mut(dst).unwrap().enqueue_blocked_receiver(1, 200);
        let r = handle_queuing_send(&mut pool, &mut pt, 0, 100, src, &[0xBB; 4]);
        assert_eq!(r, 0);
        assert_eq!(pt.get(1).unwrap().state(), PartitionState::Ready);
        let mut buf = [0u8; 64];
        let recv = pool.receive_queuing_message(dst, 1, &mut buf, 0, 0).unwrap();
        assert!(matches!(recv, RecvQueuingOutcome::Received { msg_len: 4, .. }));
        assert_eq!(&buf[..4], &[0xBB; 4]);
    }

    #[test]
    #[rustfmt::skip]
    fn queue_full_returns_zero() {
        let (mut pool, mut pt) = make_pool_and_pt();
        let (src, _) = add_route(&mut pool);
        assert_eq!(handle_queuing_send(&mut pool, &mut pt, 0, 100, src, &[1]), 0);
        assert_eq!(handle_queuing_send(&mut pool, &mut pt, 0, 101, src, &[2]), 0);
        assert_eq!(handle_queuing_send(&mut pool, &mut pt, 0, 102, src, &[3]), 0);
    }
}
