//! Partition and schedule management state.

use crate::partition::PartitionTable;
use crate::scheduler::ScheduleTable;

/// Groups partition and schedule management state.
pub struct PartitionCore<const N: usize, const SCHED: usize>
where
    [(); N]:,
    [(); SCHED]:,
{
    partitions: PartitionTable<N>,
    schedule: ScheduleTable<SCHED>,
    current_partition: u8,
    next_partition: u8,
    partition_sp: [u32; N],
}

impl<const N: usize, const SCHED: usize> PartitionCore<N, SCHED>
where
    [(); N]:,
    [(); SCHED]:,
{
    pub const fn new() -> Self {
        Self {
            partitions: PartitionTable::new(),
            schedule: ScheduleTable::new(),
            current_partition: 0,
            next_partition: 0,
            partition_sp: [0u32; N],
        }
    }

    pub fn partitions(&self) -> &PartitionTable<N> {
        &self.partitions
    }
    pub fn partitions_mut(&mut self) -> &mut PartitionTable<N> {
        &mut self.partitions
    }
    pub fn schedule(&self) -> &ScheduleTable<SCHED> {
        &self.schedule
    }
    pub fn schedule_mut(&mut self) -> &mut ScheduleTable<SCHED> {
        &mut self.schedule
    }
    pub fn current_partition(&self) -> u8 {
        self.current_partition
    }
    pub fn set_current_partition(&mut self, id: u8) {
        self.current_partition = id;
    }
    pub fn next_partition(&self) -> u8 {
        self.next_partition
    }
    pub fn set_next_partition(&mut self, id: u8) {
        self.next_partition = id;
    }
    pub fn partition_sp(&self) -> &[u32; N] {
        &self.partition_sp
    }
    pub fn partition_sp_mut(&mut self) -> &mut [u32; N] {
        &mut self.partition_sp
    }
    pub fn get_sp(&self, index: usize) -> Option<u32> {
        self.partition_sp.get(index).copied()
    }

    pub fn set_sp(&mut self, index: usize, sp: u32) -> bool {
        if let Some(slot) = self.partition_sp.get_mut(index) {
            *slot = sp;
            true
        } else {
            false
        }
    }
}

impl<const N: usize, const SCHED: usize> Default for PartitionCore<N, SCHED>
where
    [(); N]:,
    [(); SCHED]:,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::ScheduleEntry;

    #[test]
    fn construction_and_field_access() {
        let mut core: PartitionCore<4, 8> = PartitionCore::new();
        // Verify new() creates empty state
        assert!(core.partitions().is_empty());
        assert!(core.schedule().is_empty());
        assert_eq!(core.current_partition(), 0);
        assert_eq!(core.next_partition(), 0);
        assert_eq!(core.partition_sp(), &[0u32; 4]);
        // Verify default() matches new()
        let dflt: PartitionCore<4, 8> = PartitionCore::default();
        assert_eq!(core.partition_sp(), dflt.partition_sp());
        // Test partition index setters
        core.set_current_partition(2);
        core.set_next_partition(3);
        assert_eq!(core.current_partition(), 2);
        assert_eq!(core.next_partition(), 3);
        // Test SP accessors with bounds checking
        assert!(core.set_sp(0, 0x2000_0400));
        assert_eq!(core.get_sp(0), Some(0x2000_0400));
        assert_eq!(core.get_sp(4), None);
        assert!(!core.set_sp(4, 0x2000_0000));
        core.partition_sp_mut()[1] = 0x2000_1400;
        assert_eq!(core.partition_sp()[1], 0x2000_1400);
        // Test mutable schedule accessor
        assert!(core.schedule_mut().add(ScheduleEntry::new(0, 100)).is_ok());
        assert_eq!(core.schedule().len(), 1);
    }
}
