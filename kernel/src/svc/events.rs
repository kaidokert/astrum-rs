use crate::events;
use crate::partition::PartitionTable;
use rtos_traits::ids::PartitionId;

/// Handle `EventWait`. Returns `(return_value, should_block)`.
///
/// Blocking invariant: `event_wait` returns 0 when the caller has been
/// transitioned to `Waiting` (no matching bits were pending). Any non-zero
/// return indicates matched bits were consumed immediately.
// TODO: the `r == 0` blocking convention is implicit; consider having
// the `events` module return a structured result type instead.
pub fn handle_event_wait<const N: usize>(
    pt: &mut PartitionTable<N>,
    caller: PartitionId,
    mask: u32,
) -> (u32, bool) {
    let r = events::event_wait(pt, caller, mask);
    (r, r == 0)
}

/// Handle `EventSet`: post event bits to a target partition, potentially
/// waking it if it was waiting on any of the posted bits.
pub fn handle_event_set<const N: usize>(
    pt: &mut PartitionTable<N>,
    target: PartitionId,
    mask: u32,
) -> u32 {
    events::event_set(pt, target, mask)
}

/// Handle `EventClear`: atomically clear the specified event bits for the
/// caller. Returns the previous event flags value.
pub fn handle_event_clear<const N: usize>(
    pt: &mut PartitionTable<N>,
    caller: PartitionId,
    mask: u32,
) -> u32 {
    events::event_clear(pt, caller, mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};
    use crate::svc::SvcError;
    fn pt() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        for i in 0..2u8 {
            let base = 0x2000_0000 + (i as u32) * 0x1000;
            let stack_top = base + 0x1000;
            t.add(PartitionControlBlock::new(
                i,
                0x0800_0000,
                base,
                stack_top,
                MpuRegion::new(base, 4096, 0),
            ))
            .unwrap();
            t.get_mut(i as usize)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
        }
        t
    }

    #[test]
    fn wait_acquired_and_blocked_paths() {
        let mut t = pt();
        handle_event_set(&mut t, 0u32.into(), 0b1010);
        let (val, block) = handle_event_wait(&mut t, 0u32.into(), 0b1110);
        assert!(!block, "must not block when matching bits pending");
        assert_eq!(val, 0b1010, "must return all pending bits");
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Running);
        let (val, block) = handle_event_wait(&mut t, 0u32.into(), 0b0011);
        assert!(block, "must block when no matching bits pending");
        assert_eq!(val, 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
    }

    #[test]
    fn set_wakes_and_clear_returns_previous() {
        let mut t = pt();
        let (_, block) = handle_event_wait(&mut t, 0u32.into(), 0b0001);
        assert!(block);
        assert_eq!(handle_event_set(&mut t, 0u32.into(), 0b0001), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0001);
        handle_event_set(&mut t, 1u32.into(), 0b1111);
        assert_eq!(handle_event_clear(&mut t, 1u32.into(), 0b0101), 0b1111);
        assert_eq!(t.get(1).unwrap().event_flags(), 0b1010);
    }

    #[test]
    fn invalid_partition_targets() {
        let mut t = pt();
        let inv = SvcError::InvalidPartition.to_u32();
        let (val, block) = handle_event_wait(&mut t, 99u32.into(), 0b1);
        assert_eq!(val, inv);
        assert!(!block, "must not block on invalid partition");
        assert_eq!(handle_event_set(&mut t, 99u32.into(), 0b1), inv);
        assert_eq!(handle_event_clear(&mut t, 99u32.into(), 0b1), inv);
    }
}
