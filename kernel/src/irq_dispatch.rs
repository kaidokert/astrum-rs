//! IRQ-to-partition dispatch mapping.
//!
//! Each `IrqBinding` records which partition should be notified (and with
//! which event bits) when a particular hardware IRQ fires.

use crate::events;
use crate::partition::{PartitionState, PartitionTable};

/// Signal a partition with event bits, returning `true` if it was woken
/// (transitioned from `Waiting` to `Ready`).
///
/// This is the testable inner logic used by ISR dispatch handlers.
/// Returns `false` for invalid partition indices or when the target
/// does not transition to `Ready`.
pub fn signal_partition_inner<const N: usize>(
    t: &mut PartitionTable<N>,
    target: usize,
    event_bits: u32,
) -> bool {
    let was_waiting = t
        .get(target)
        .is_some_and(|p| p.state() == PartitionState::Waiting);
    events::event_set(t, target, event_bits);
    was_waiting
        && t.get(target)
            .is_some_and(|p| p.state() == PartitionState::Ready)
}

/// A const-friendly mapping from an IRQ number to a (partition, event_bits) pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrqBinding {
    pub irq_num: u8,
    pub partition_id: u8,
    pub event_bits: u32,
}

impl IrqBinding {
    /// Create a new IRQ binding.
    pub const fn new(irq_num: u8, partition_id: u8, event_bits: u32) -> Self {
        Self {
            irq_num,
            partition_id,
            event_bits,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events;
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
        // Both start Ready; transition to Running for testing.
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
    fn signal_running_partition_sets_flags_returns_false() {
        let mut t = tbl();
        let woke = signal_partition_inner(&mut t, 0, 0b0101);
        assert!(!woke);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0101);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Running);
    }

    #[test]
    fn signal_waiting_with_matching_mask_wakes() {
        let mut t = tbl();
        // Put partition 0 into Waiting on bit 0.
        events::event_wait(&mut t, 0, 0b0001);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        let woke = signal_partition_inner(&mut t, 0, 0b0001);
        assert!(woke);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn signal_waiting_with_nonmatching_mask_stays_waiting() {
        let mut t = tbl();
        events::event_wait(&mut t, 0, 0b0001);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        let woke = signal_partition_inner(&mut t, 0, 0b1100);
        assert!(!woke);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1100);
    }

    #[test]
    fn signal_invalid_partition_returns_false() {
        let mut t = tbl();
        let woke = signal_partition_inner(&mut t, 99, 0b0001);
        assert!(!woke);
    }

    #[test]
    fn multiple_signals_accumulate_flags() {
        let mut t = tbl();
        // Put partition 0 into Waiting on bits 0 and 1.
        events::event_wait(&mut t, 0, 0b0011);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        // First signal: bit 2 — no overlap with wait mask.
        let woke1 = signal_partition_inner(&mut t, 0, 0b0100);
        assert!(!woke1);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0100);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        // Second signal: bit 0 — overlaps wait mask, should wake.
        let woke2 = signal_partition_inner(&mut t, 0, 0b0001);
        assert!(woke2);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
        // Both signals accumulated.
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0101);
    }

    #[test]
    fn new_constructor_sets_fields() {
        let b = IrqBinding::new(7, 2, 0x0000_0010);
        assert_eq!(b.irq_num, 7);
        assert_eq!(b.partition_id, 2);
        assert_eq!(b.event_bits, 0x0000_0010);
    }

    #[test]
    fn direct_struct_construction() {
        let b = IrqBinding {
            irq_num: 15,
            partition_id: 0,
            event_bits: 0xDEAD_BEEF,
        };
        assert_eq!(b.irq_num, 15);
        assert_eq!(b.partition_id, 0);
        assert_eq!(b.event_bits, 0xDEAD_BEEF);
    }

    #[test]
    fn const_construction() {
        const BINDING: IrqBinding = IrqBinding::new(3, 1, 0x01);
        assert_eq!(BINDING.irq_num, 3);
        assert_eq!(BINDING.partition_id, 1);
        assert_eq!(BINDING.event_bits, 0x01);
    }

    #[test]
    fn clone_produces_equal_value() {
        let a = IrqBinding::new(10, 3, 0xFF);
        // Call Clone::clone explicitly through a reference to avoid clone_on_copy lint.
        let b = Clone::clone(&a);
        assert_eq!(a, b);
    }

    #[test]
    fn copy_semantics() {
        let a = IrqBinding::new(10, 3, 0xFF);
        let b = a; // Copy
        assert_eq!(a, b); // `a` still usable — Copy
    }

    #[test]
    fn partial_eq_detects_difference() {
        let a = IrqBinding::new(1, 2, 3);
        let b = IrqBinding::new(1, 2, 4);
        assert_ne!(a, b);
    }

    #[test]
    fn debug_format_contains_fields() {
        let b = IrqBinding::new(5, 1, 0x80);
        let dbg = format!("{:?}", b);
        assert!(dbg.contains("IrqBinding"));
        assert!(dbg.contains("irq_num: 5"));
        assert!(dbg.contains("partition_id: 1"));
        assert!(dbg.contains("event_bits: 128"));
    }

    #[test]
    fn boundary_values() {
        let b = IrqBinding::new(u8::MAX, u8::MAX, u32::MAX);
        assert_eq!(b.irq_num, 255);
        assert_eq!(b.partition_id, 255);
        assert_eq!(b.event_bits, u32::MAX);

        let z = IrqBinding::new(0, 0, 0);
        assert_eq!(z.irq_num, 0);
        assert_eq!(z.partition_id, 0);
        assert_eq!(z.event_bits, 0);
    }

    #[test]
    fn const_array_of_bindings() {
        const TABLE: [IrqBinding; 3] = [
            IrqBinding::new(0, 0, 0x01),
            IrqBinding::new(1, 0, 0x02),
            IrqBinding::new(2, 1, 0x04),
        ];
        assert_eq!(TABLE[0].irq_num, 0);
        assert_eq!(TABLE[1].event_bits, 0x02);
        assert_eq!(TABLE[2].partition_id, 1);
    }
}
