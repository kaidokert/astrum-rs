//! Partition and schedule management state.

use crate::partition::{PartitionControlBlock, PartitionTable};
use crate::scheduler::{ScheduleTable, ScheduleTableOps, ScheduleTableOpsMut};

/// Narrow interface for partition/schedule state access without const bounds.
pub trait PartitionCoreOps {
    /// Returns a slice of partition control blocks.
    fn partitions(&self) -> &[PartitionControlBlock];
    /// Returns a mutable reference to a partition by index.
    fn partition_mut(&mut self, index: usize) -> Option<&mut PartitionControlBlock>;
    /// Returns the number of partitions.
    fn partition_count(&self) -> usize;
    /// Returns the current partition index.
    fn current_partition(&self) -> u8;
    /// Sets the current partition index.
    fn set_current_partition(&mut self, id: u8);
    /// Returns the next partition index.
    fn next_partition(&self) -> u8;
    /// Sets the next partition index.
    fn set_next_partition(&mut self, id: u8);
    /// Gets the stack pointer for a partition by index.
    fn get_sp(&self, index: usize) -> Option<u32>;
    /// Sets the stack pointer for a partition by index. Returns true if valid.
    fn set_sp(&mut self, index: usize, sp: u32) -> bool;
    /// Returns a reference to the schedule table (via trait object).
    fn schedule(&self) -> &dyn ScheduleTableOps;
    /// Returns a mutable reference to the schedule table (via trait object).
    fn schedule_mut(&mut self) -> &mut dyn ScheduleTableOpsMut;
}

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

    /// Replace the schedule table with the provided one.
    ///
    /// Used by `Kernel::new()` to initialize the core with a pre-built
    /// and validated schedule.
    pub fn set_schedule(&mut self, schedule: ScheduleTable<SCHED>) {
        self.schedule = schedule;
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

impl<const N: usize, const SCHED: usize> crate::config::CoreOps for PartitionCore<N, SCHED>
where
    [(); N]:,
    [(); SCHED]:,
{
    type PartTable = PartitionTable<N>;
    type SchedTable = ScheduleTable<SCHED>;

    fn partitions(&self) -> &Self::PartTable {
        &self.partitions
    }
    fn partitions_mut(&mut self) -> &mut Self::PartTable {
        &mut self.partitions
    }
    fn schedule(&self) -> &Self::SchedTable {
        &self.schedule
    }
    fn schedule_mut(&mut self) -> &mut Self::SchedTable {
        &mut self.schedule
    }
    fn current_partition(&self) -> u8 {
        self.current_partition
    }
    fn set_current_partition(&mut self, id: u8) {
        self.current_partition = id;
    }
    fn next_partition(&self) -> u8 {
        self.next_partition
    }
    fn set_next_partition(&mut self, id: u8) {
        self.next_partition = id;
    }
    fn get_sp(&self, index: usize) -> Option<u32> {
        self.partition_sp.get(index).copied()
    }
    fn set_sp(&mut self, index: usize, sp: u32) -> bool {
        if let Some(slot) = self.partition_sp.get_mut(index) {
            *slot = sp;
            true
        } else {
            false
        }
    }
    fn partition_sp(&self) -> &[u32] {
        &self.partition_sp
    }
    fn partition_sp_mut(&mut self) -> &mut [u32] {
        &mut self.partition_sp
    }
}

impl<const N: usize, const SCHED: usize> PartitionCoreOps for PartitionCore<N, SCHED>
where
    [(); N]:,
    [(); SCHED]:,
{
    fn partitions(&self) -> &[PartitionControlBlock] {
        self.partitions.as_slice()
    }

    fn partition_mut(&mut self, index: usize) -> Option<&mut PartitionControlBlock> {
        self.partitions.get_mut(index)
    }

    fn partition_count(&self) -> usize {
        self.partitions.len()
    }

    fn current_partition(&self) -> u8 {
        self.current_partition
    }

    fn set_current_partition(&mut self, id: u8) {
        self.current_partition = id;
    }

    fn next_partition(&self) -> u8 {
        self.next_partition
    }

    fn set_next_partition(&mut self, id: u8) {
        self.next_partition = id;
    }

    fn get_sp(&self, index: usize) -> Option<u32> {
        self.partition_sp.get(index).copied()
    }

    fn set_sp(&mut self, index: usize, sp: u32) -> bool {
        if let Some(slot) = self.partition_sp.get_mut(index) {
            *slot = sp;
            true
        } else {
            false
        }
    }

    fn schedule(&self) -> &dyn ScheduleTableOps {
        &self.schedule
    }

    fn schedule_mut(&mut self) -> &mut dyn ScheduleTableOpsMut {
        &mut self.schedule
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
