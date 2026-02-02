use crate::partition::{PartitionState, PartitionTable};
use crate::svc::SvcError;

/// If any `mask` bits are set in caller's flags, clear and return them; else set Waiting.
/// Returns `u32::MAX` on invalid partition ID, `0` when entering Waiting state.
pub fn event_wait<const N: usize>(t: &mut PartitionTable<N>, caller: usize, mask: u32) -> u32 {
    let pcb = match t.get_mut(caller) {
        Some(p) => p,
        None => return SvcError::InvalidPartition.to_u32(),
    };
    let matched = pcb.event_flags() & mask;
    if matched != 0 {
        pcb.clear_event_flags(matched);
        pcb.set_event_wait_mask(0);
        matched
    } else {
        pcb.set_event_wait_mask(mask);
        let _ = pcb.transition(PartitionState::Waiting);
        0
    }
}

/// Set `mask` bits in target's flags; wake if Waiting. Returns `u32::MAX` on bad id.
pub fn event_set<const N: usize>(t: &mut PartitionTable<N>, target: usize, mask: u32) -> u32 {
    let pcb = match t.get_mut(target) {
        Some(p) => p,
        None => return SvcError::InvalidPartition.to_u32(),
    };
    pcb.set_event_flags(mask);
    if pcb.state() == PartitionState::Waiting && (pcb.event_flags() & pcb.event_wait_mask()) != 0 {
        let _ = pcb.transition(PartitionState::Ready);
    }
    0
}

/// Clear `mask` bits in caller's flags. Returns `u32::MAX` on bad id.
pub fn event_clear<const N: usize>(t: &mut PartitionTable<N>, caller: usize, mask: u32) -> u32 {
    match t.get_mut(caller) {
        Some(p) => {
            p.clear_event_flags(mask);
            0
        }
        None => SvcError::InvalidPartition.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};
    fn pcb(id: u8) -> PartitionControlBlock {
        let o = (id as u32) * 0x1000;
        PartitionControlBlock::new(
            id,
            0x0800_0000 + o,
            0x2000_0000 + o,
            0x2000_0400 + o,
            MpuRegion::new(0x2000_0000 + o, 4096, 0),
        )
    }
    fn tbl() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        t.add(pcb(0)).unwrap();
        t.add(pcb(1)).unwrap();
        t.get_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t.get_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t
    }
    #[test]
    fn set_then_wait_returns_immediately() {
        let mut t = tbl();
        event_set(&mut t, 0, 0b1010);
        assert_eq!(event_wait(&mut t, 0, 0b1110), 0b1010);
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Running);
    }
    #[test]
    fn wait_blocks_then_set_wakes() {
        let mut t = tbl();
        assert_eq!(event_wait(&mut t, 0, 0b0001), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        event_set(&mut t, 0, 0b0001);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
        // Verify that the flags were actually set
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0001);
        // Transition back to Running so we can call event_wait again
        t.get_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        // Consume the flags — should return immediately with the matched bits
        assert_eq!(event_wait(&mut t, 0, 0b0001), 0b0001);
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Running);
    }
    #[test]
    fn set_nonmatching_mask_keeps_waiting() {
        let mut t = tbl();
        // Partition 0 waits on bit 0
        assert_eq!(event_wait(&mut t, 0, 0b0001), 0);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        // Set bits that do NOT overlap with the wait mask
        event_set(&mut t, 0, 0b1100);
        // Partition should remain Waiting because no waited-on bits were set
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1100);
    }
    #[test]
    fn clear_cross_partition_bitmask_and_invalid() {
        let mut t = tbl();
        event_set(&mut t, 0, 0b1111);
        event_clear(&mut t, 0, 0b0101);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1010);
        event_set(&mut t, 1, 0b1000);
        assert_eq!(t.get(1).unwrap().event_flags(), 0b1000);
        assert_eq!(event_wait(&mut t, 0, 0b1010), 0b1010);
        let inv = SvcError::InvalidPartition.to_u32();
        assert_eq!(event_set(&mut t, 99, 1), inv);
        assert_eq!(event_clear(&mut t, 99, 1), inv);
        assert_eq!(event_wait(&mut t, 99, 1), inv);
    }
}
