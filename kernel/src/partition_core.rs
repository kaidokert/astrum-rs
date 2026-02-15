//! Partition and schedule management state.

use crate::partition::{PartitionControlBlock, PartitionTable};
use crate::scheduler::{ScheduleTable, ScheduleTableOps, ScheduleTableOpsMut};
use crate::tick::TickCounter;

/// Stack storage wrapper aligned to 1024 bytes for MPU region compatibility.
///
/// The MPU on Cortex-M requires that region base addresses be aligned to the
/// region size. For 1024-byte stacks (256 words), the base must be 1024-byte
/// aligned. This wrapper enforces that alignment at compile time.
///
/// # Type Parameters
///
/// - `SW`: Stack word count. Must be 256 for 1024-byte alignment (256 * 4 = 1024).
#[repr(C, align(1024))]
#[derive(Clone, Copy)]
pub struct AlignedStack<const SW: usize>(pub [u32; SW]);

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
    /// Returns the currently active partition index, if any.
    fn active_partition(&self) -> Option<u8>;
    /// Sets the active partition index.
    fn set_active_partition(&mut self, id: Option<u8>);
    /// Returns a reference to the tick counter.
    fn tick(&self) -> &TickCounter;
    /// Returns a mutable reference to the tick counter.
    fn tick_mut(&mut self) -> &mut TickCounter;
    /// Returns whether a yield has been requested.
    fn yield_requested(&self) -> bool;
    /// Sets the yield_requested flag.
    fn set_yield_requested(&mut self, requested: bool);
}

/// Groups partition and schedule management state.
///
/// # Type Parameters
///
/// - `N`: Maximum number of partitions
/// - `SCHED`: Schedule table capacity (number of entries)
/// - `SW`: Stack word count per partition (256 = 1024 bytes for MPU alignment)
pub struct PartitionCore<const N: usize, const SCHED: usize, const SW: usize>
where
    [(); N]:,
    [(); SCHED]:,
    [(); SW]:,
{
    partitions: PartitionTable<N>,
    schedule: ScheduleTable<SCHED>,
    current_partition: u8,
    next_partition: u8,
    partition_sp: [u32; N],
    /// Per-partition stack storage, aligned for MPU region base requirements.
    stacks: [AlignedStack<SW>; N],
    /// Currently active partition index, if any.
    active_partition: Option<u8>,
    /// Monotonic tick counter.
    tick: TickCounter,
    /// Set to `true` by `SYS_YIELD` dispatch; checked and cleared by the
    /// harness so it can force-advance the schedule and call
    /// `set_next_partition()` before PendSV fires.
    yield_requested: bool,
}

impl<const N: usize, const SCHED: usize, const SW: usize> PartitionCore<N, SCHED, SW>
where
    [(); N]:,
    [(); SCHED]:,
    [(); SW]:,
{
    /// Zero-initialized stack constant for const array initialization.
    const ZERO_STACK: AlignedStack<SW> = AlignedStack([0u32; SW]);

    pub const fn new() -> Self {
        Self {
            partitions: PartitionTable::new(),
            schedule: ScheduleTable::new(),
            current_partition: 0,
            next_partition: 0,
            partition_sp: [0u32; N],
            stacks: [Self::ZERO_STACK; N],
            active_partition: None,
            tick: TickCounter::new(),
            yield_requested: false,
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

    /// Returns a reference to all partition stacks.
    pub fn stacks(&self) -> &[AlignedStack<SW>; N] {
        &self.stacks
    }

    /// Returns a mutable reference to all partition stacks.
    pub fn stacks_mut(&mut self) -> &mut [AlignedStack<SW>; N] {
        &mut self.stacks
    }

    /// Returns a mutable reference to a specific partition's stack array.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn stack_mut(&mut self, index: usize) -> Option<&mut [u32; SW]> {
        self.stacks.get_mut(index).map(|s| &mut s.0)
    }

    /// Returns the base address of a partition's stack as an integer.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn stack_base(&self, index: usize) -> Option<u32> {
        self.stacks.get(index).map(|s| s.0.as_ptr() as u32)
    }

    /// Returns the stack size in bytes for each partition.
    ///
    /// This is constant for all partitions and equals `SW * 4` where SW
    /// is the stack word count.
    pub const fn stack_size(&self) -> usize {
        SW * 4
    }

    /// Returns the initial stack pointer for a partition.
    ///
    /// The initial SP points to the top of the stack (stack base + size)
    /// since Cortex-M stacks grow downward. Returns `None` if the index
    /// is out of bounds.
    pub fn stack_ptr_init(&self, index: usize) -> Option<u32> {
        self.stack_base(index)
            .map(|base| base.wrapping_add(self.stack_size() as u32))
    }

    /// Returns the currently active partition index, if any.
    pub fn active_partition(&self) -> Option<u8> {
        self.active_partition
    }

    /// Sets the active partition index.
    pub fn set_active_partition(&mut self, id: Option<u8>) {
        self.active_partition = id;
    }

    /// Returns a reference to the tick counter.
    pub fn tick(&self) -> &TickCounter {
        &self.tick
    }

    /// Returns a mutable reference to the tick counter.
    pub fn tick_mut(&mut self) -> &mut TickCounter {
        &mut self.tick
    }

    /// Returns whether a yield has been requested.
    pub fn yield_requested(&self) -> bool {
        self.yield_requested
    }

    /// Sets the yield_requested flag.
    pub fn set_yield_requested(&mut self, requested: bool) {
        self.yield_requested = requested;
    }

    /// Replace the schedule table with the provided one.
    ///
    /// Used by `Kernel::new()` to initialize the core with a pre-built
    /// and validated schedule.
    pub fn set_schedule(&mut self, schedule: ScheduleTable<SCHED>) {
        self.schedule = schedule;
    }
}

impl<const N: usize, const SCHED: usize, const SW: usize> Default for PartitionCore<N, SCHED, SW>
where
    [(); N]:,
    [(); SCHED]:,
    [(); SW]:,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const SCHED: usize, const SW: usize> crate::config::CoreOps
    for PartitionCore<N, SCHED, SW>
where
    [(); N]:,
    [(); SCHED]:,
    [(); SW]:,
{
    type PartTable = PartitionTable<N>;
    type SchedTable = ScheduleTable<SCHED>;
    type TickCounter = TickCounter;

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
    fn active_partition(&self) -> Option<u8> {
        self.active_partition
    }
    fn set_active_partition(&mut self, id: Option<u8>) {
        self.active_partition = id;
    }
    fn tick(&self) -> &Self::TickCounter {
        &self.tick
    }
    fn tick_mut(&mut self) -> &mut Self::TickCounter {
        &mut self.tick
    }
    fn yield_requested(&self) -> bool {
        self.yield_requested
    }
    fn set_yield_requested(&mut self, requested: bool) {
        self.yield_requested = requested;
    }
    fn stack_mut(&mut self, index: usize) -> Option<&mut [u32]> {
        self.stacks.get_mut(index).map(|s| s.0.as_mut_slice())
    }
}

impl<const N: usize, const SCHED: usize, const SW: usize> PartitionCoreOps
    for PartitionCore<N, SCHED, SW>
where
    [(); N]:,
    [(); SCHED]:,
    [(); SW]:,
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

    fn active_partition(&self) -> Option<u8> {
        self.active_partition
    }

    fn set_active_partition(&mut self, id: Option<u8>) {
        self.active_partition = id;
    }

    fn tick(&self) -> &TickCounter {
        &self.tick
    }

    fn tick_mut(&mut self) -> &mut TickCounter {
        &mut self.tick
    }

    fn yield_requested(&self) -> bool {
        self.yield_requested
    }

    fn set_yield_requested(&mut self, requested: bool) {
        self.yield_requested = requested;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::ScheduleEntry;

    /// Stack word count for tests (256 words = 1024 bytes).
    const TEST_SW: usize = 256;

    #[test]
    fn construction_and_field_access() {
        let mut core: PartitionCore<4, 8, TEST_SW> = PartitionCore::new();
        // Verify new() creates empty state
        assert!(core.partitions().is_empty());
        assert!(core.schedule().is_empty());
        assert_eq!(core.current_partition(), 0);
        assert_eq!(core.next_partition(), 0);
        assert_eq!(core.partition_sp(), &[0u32; 4]);
        // Verify default() matches new()
        let dflt: PartitionCore<4, 8, TEST_SW> = PartitionCore::default();
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

    #[test]
    fn stacks_initialized_to_zero() {
        let core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        // All stack arrays should be zero-initialized
        for i in 0..2 {
            let stack = &core.stacks()[i].0;
            assert!(stack.iter().all(|&w| w == 0));
        }
    }

    #[test]
    fn stacks_are_1024_byte_aligned() {
        let core: PartitionCore<3, 4, TEST_SW> = PartitionCore::new();
        // Each stack base must be 1024-byte aligned for MPU compatibility
        for i in 0..3 {
            let base = core.stack_base(i).unwrap();
            assert_eq!(
                base % 1024,
                0,
                "Stack {} base 0x{:08x} is not 1024-byte aligned",
                i,
                base
            );
        }
    }

    #[test]
    fn stack_base_out_of_bounds_returns_none() {
        let core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        assert!(core.stack_base(0).is_some());
        assert!(core.stack_base(1).is_some());
        assert!(core.stack_base(2).is_none());
        assert!(core.stack_base(100).is_none());
    }

    #[test]
    fn stack_mut_accessor() {
        let mut core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        // Verify we can write to stack and read it back
        {
            let stack = core.stack_mut(0).unwrap();
            stack[0] = 0xDEAD_BEEF;
            stack[TEST_SW - 1] = 0xCAFE_BABE;
        }
        assert_eq!(core.stacks()[0].0[0], 0xDEAD_BEEF);
        assert_eq!(core.stacks()[0].0[TEST_SW - 1], 0xCAFE_BABE);
        // Out of bounds returns None
        assert!(core.stack_mut(2).is_none());
    }

    #[test]
    fn stacks_mut_accessor() {
        let mut core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        // Write different values to each stack
        core.stacks_mut()[0].0[0] = 0x1111_1111;
        core.stacks_mut()[1].0[0] = 0x2222_2222;
        assert_eq!(core.stacks()[0].0[0], 0x1111_1111);
        assert_eq!(core.stacks()[1].0[0], 0x2222_2222);
    }

    #[test]
    fn aligned_stack_size_matches_word_count() {
        // Verify AlignedStack<256> has the expected size (256 * 4 = 1024 bytes)
        assert_eq!(
            core::mem::size_of::<AlignedStack<TEST_SW>>(),
            TEST_SW * 4,
            "AlignedStack<{}> size should be {} bytes",
            TEST_SW,
            TEST_SW * 4
        );
    }

    #[test]
    fn aligned_stack_alignment_is_1024() {
        // Verify AlignedStack has the required 1024-byte alignment
        assert_eq!(
            core::mem::align_of::<AlignedStack<TEST_SW>>(),
            1024,
            "AlignedStack<{}> alignment should be 1024 bytes",
            TEST_SW
        );
    }

    #[test]
    fn stack_size_returns_bytes() {
        let core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        // TEST_SW = 256 words * 4 bytes/word = 1024 bytes
        assert_eq!(core.stack_size(), 1024);
    }

    #[test]
    fn stack_size_different_word_counts() {
        // Test with 128 words (512 bytes)
        let core_small: PartitionCore<1, 1, 128> = PartitionCore::new();
        assert_eq!(core_small.stack_size(), 512);

        // Test with 512 words (2048 bytes)
        let core_large: PartitionCore<1, 1, 512> = PartitionCore::new();
        assert_eq!(core_large.stack_size(), 2048);
    }

    #[test]
    fn stack_ptr_init_returns_top_of_stack() {
        let core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        for i in 0..2 {
            let init_sp = core.stack_ptr_init(i);
            assert!(init_sp.is_some(), "stack_ptr_init({}) should be Some", i);
            let init_sp = init_sp.unwrap();
            let base = core.stack_base(i).unwrap();
            // Initial SP should be at stack_base + stack_size
            assert_eq!(
                init_sp,
                base + core.stack_size() as u32,
                "Initial SP should point to top of stack"
            );
        }
    }

    #[test]
    fn stack_ptr_init_out_of_bounds_returns_none() {
        let core: PartitionCore<2, 4, TEST_SW> = PartitionCore::new();
        assert!(core.stack_ptr_init(0).is_some());
        assert!(core.stack_ptr_init(1).is_some());
        assert!(core.stack_ptr_init(2).is_none());
        assert!(core.stack_ptr_init(100).is_none());
    }
}
