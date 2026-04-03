use crate::partition::{PartitionState, PartitionTable};
use crate::svc::{try_transition, SvcError};
use crate::waitqueue::SleepQueue;
use rtos_traits::ids::PartitionId;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepOutcome {
    Done(u32),
    Deschedule(u32),
}

impl SleepOutcome {
    pub fn into_svc_return(self) -> (u32, bool) {
        match self {
            Self::Done(r) => (r, false),
            Self::Deschedule(r) => (r, true),
        }
    }
}

pub fn handle_sleep_ticks<const N: usize, const W: usize>(
    sleep_queue: &mut SleepQueue<W>,
    pt: &mut PartitionTable<N>,
    caller: PartitionId,
    ticks: u32,
    current_tick: u64,
) -> SleepOutcome {
    if ticks == 0 {
        return SleepOutcome::Done(0);
    }
    let expiry = current_tick + ticks as u64;
    if sleep_queue.push(caller, expiry).is_err() {
        return SleepOutcome::Done(SvcError::WaitQueueFull.to_u32());
    }
    if !try_transition(pt, caller, PartitionState::Waiting) {
        return SleepOutcome::Done(SvcError::TransitionFailed.to_u32());
    }
    if let Some(pcb) = pt.get_mut(caller.as_raw() as usize) {
        pcb.set_sleep_until(expiry);
    }
    SleepOutcome::Deschedule(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock};

    #[rustfmt::skip]
    fn make_pt_sq() -> (PartitionTable<4>, SleepQueue<4>) {
        let mut t = PartitionTable::new();
        for i in 0..2u8 {
            let b = 0x2000_0000 + (i as u32) * 0x1000;
            t.add(PartitionControlBlock::new(i, 0x800_0000, b, b + 0x400, MpuRegion::new(b, 4096, 0))).unwrap();
            t.get_mut(i as usize).unwrap().transition(PartitionState::Running).unwrap();
        }
        (t, SleepQueue::new())
    }

    fn pid(v: u32) -> PartitionId {
        PartitionId::new(v)
    }

    #[test]
    fn zero_ticks_is_noop() {
        let (mut pt, mut sq) = make_pt_sq();
        let out = handle_sleep_ticks(&mut sq, &mut pt, pid(0), 0, 100);
        assert_eq!(out, SleepOutcome::Done(0));
        assert_eq!(out.into_svc_return(), (0, false));
        assert!(sq.is_empty());
        assert_eq!(pt.get(0).unwrap().state(), PartitionState::Running);
    }

    #[test]
    fn nonzero_ticks_deschedules() {
        let (mut pt, mut sq) = make_pt_sq();
        let out = handle_sleep_ticks(&mut sq, &mut pt, pid(0), 50, 100);
        assert_eq!(out, SleepOutcome::Deschedule(0));
        assert_eq!(out.into_svc_return(), (0, true));
        assert!(!sq.is_empty() && pt.get(0).unwrap().state() == PartitionState::Waiting);
        assert_eq!(pt.get(0).unwrap().sleep_until(), 150);
    }

    #[test]
    fn full_queue_returns_error() {
        let (mut pt, mut sq) = make_pt_sq();
        for i in 0..4u8 {
            let _ = sq.push(PartitionId::new(i as u32), 1000 + i as u64);
        }
        let out = handle_sleep_ticks(&mut sq, &mut pt, pid(0), 10, 0);
        assert_eq!(out, SleepOutcome::Done(SvcError::WaitQueueFull.to_u32()));
        assert!(!out.into_svc_return().1);
        assert_eq!(pt.get(0).unwrap().state(), PartitionState::Running);
        assert_eq!(pt.get(0).unwrap().sleep_until(), 0);
    }
}
