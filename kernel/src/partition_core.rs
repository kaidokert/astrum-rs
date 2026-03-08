//! Partition and schedule management state.

use crate::partition::{PartitionControlBlock, PartitionTable};
use crate::scheduler::{ScheduleTable, ScheduleTableOps, ScheduleTableOpsMut};
use crate::tick::TickCounter;

/// Trait abstracting over stack storage for MPU-aligned partition stacks.
///
/// # Supported stack sizes
///
/// The following power-of-two sizes are supported, each with alignment
/// equal to the stack size (required by the Cortex-M MPU):
///
/// | Type | Size | Words | Alignment |
/// |------|------|-------|-----------|
/// | [`AlignedStack256B`] | 256 B | 64 | 256 |
/// | [`AlignedStack512B`] | 512 B | 128 | 512 |
/// | [`AlignedStack1K`] | 1 KiB | 256 | 1024 |
/// | [`AlignedStack2K`] | 2 KiB | 512 | 2048 |
/// | [`AlignedStack4K`] | 4 KiB | 1024 | 4096 |
pub trait StackStorage: Copy + Default {
    const ZERO: Self;
    const WORDS: usize;
    const SIZE_BYTES: usize;
    const ALIGNMENT: usize;
    fn as_u32_slice(&self) -> &[u32];
    fn as_u32_slice_mut(&mut self) -> &mut [u32];
}

macro_rules! define_tiered_stack {
    ($name:ident, $words:expr, $align:expr) => {
        #[repr(C, align($align))]
        #[derive(Clone, Copy)]
        pub struct $name(pub [u32; $words]);

        // Compile-time assertion: alignment must equal size for MPU compatibility.
        // Evaluated at type definition time, before any PartitionCore is constructed.
        const _: () = assert!(
            $align == $words * 4,
            concat!(stringify!($name), ": alignment must equal size (words * 4)")
        );

        impl Default for $name {
            fn default() -> Self {
                Self([0u32; $words])
            }
        }

        impl StackStorage for $name {
            const ZERO: Self = Self([0u32; $words]);
            const WORDS: usize = $words;
            const SIZE_BYTES: usize = $words * 4;
            const ALIGNMENT: usize = $align;
            fn as_u32_slice(&self) -> &[u32] {
                &self.0
            }
            fn as_u32_slice_mut(&mut self) -> &mut [u32] {
                &mut self.0
            }
        }
    };
}

define_tiered_stack!(AlignedStack256B, 64, 256);
define_tiered_stack!(AlignedStack512B, 128, 512);
define_tiered_stack!(AlignedStack1K, 256, 1024);
define_tiered_stack!(AlignedStack2K, 512, 2048);
define_tiered_stack!(AlignedStack4K, 1024, 4096);

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
///
/// # Representation
///
/// This struct uses `#[repr(C)]` to ensure deterministic field layout for
/// direct memory access from PendSV assembly. The `current_partition`,
/// `next_partition`, and `partition_sp` fields are accessed at known offsets.
#[repr(C)]
pub struct PartitionCore<const N: usize, const SCHED: usize, S: StackStorage>
where
    [(); N]:,
    [(); SCHED]:,
{
    partitions: PartitionTable<N>,
    schedule: ScheduleTable<SCHED>,
    current_partition: u8,
    /// Next partition index to switch to. Public for `offset_of!` in PendSV assembly.
    pub next_partition: u8,
    /// Per-partition saved stack pointers. Public for `offset_of!` in PendSV assembly.
    pub partition_sp: [u32; N],
    /// Per-partition stack storage, aligned for MPU region base requirements.
    stacks: [S; N],
    /// Currently active partition index, if any.
    active_partition: Option<u8>,
    /// Monotonic tick counter.
    tick: TickCounter,
    /// Set to `true` by `SYS_YIELD` dispatch; checked and cleared by the
    /// harness so it can force-advance the schedule and call
    /// `set_next_partition()` before PendSV fires.
    yield_requested: bool,
}

impl<const N: usize, const SCHED: usize, S: StackStorage> PartitionCore<N, SCHED, S>
where
    [(); N]:,
    [(); SCHED]:,
{
    const _ASSERT_ALIGNMENT_EQ_SIZE: () = assert!(
        S::ALIGNMENT == S::SIZE_BYTES,
        "StackStorage: ALIGNMENT must equal SIZE_BYTES"
    );

    pub fn new() -> Self {
        #[allow(clippy::let_unit_value)]
        let _ = Self::_ASSERT_ALIGNMENT_EQ_SIZE;
        Self {
            partitions: PartitionTable::new(),
            schedule: ScheduleTable::new(),
            current_partition: 0,
            next_partition: 0,
            partition_sp: [0u32; N],
            stacks: [S::ZERO; N],
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
    /// Returns a reference to the partition control block at `index`, if valid.
    pub fn pcb(&self, index: usize) -> Option<&PartitionControlBlock> {
        self.partitions.get(index)
    }
    /// Returns a mutable reference to the partition control block at `index`, if valid.
    pub fn pcb_mut(&mut self, index: usize) -> Option<&mut PartitionControlBlock> {
        self.partitions.get_mut(index)
    }
    /// Returns the number of partitions currently registered.
    pub fn partition_count(&self) -> usize {
        self.partitions.len()
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
    pub fn stacks(&self) -> &[S; N] {
        &self.stacks
    }

    /// Returns a mutable reference to all partition stacks.
    pub fn stacks_mut(&mut self) -> &mut [S; N] {
        &mut self.stacks
    }

    /// Returns a mutable reference to a specific partition's stack array.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn stack_mut(&mut self, index: usize) -> Option<&mut [u32]> {
        self.stacks.get_mut(index).map(|s| s.as_u32_slice_mut())
    }

    /// Returns the base address of a partition's stack as an integer.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn stack_base(&self, index: usize) -> Option<u32> {
        self.stacks
            .get(index)
            .map(|s| s.as_u32_slice().as_ptr() as u32)
    }

    /// Returns the stack size in bytes for each partition.
    pub fn stack_size(&self) -> usize {
        S::SIZE_BYTES
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

impl<const N: usize, const SCHED: usize, S: StackStorage> Default for PartitionCore<N, SCHED, S>
where
    [(); N]:,
    [(); SCHED]:,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const SCHED: usize, S: StackStorage> crate::config::CoreOps
    for PartitionCore<N, SCHED, S>
where
    [(); N]:,
    [(); SCHED]:,
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
        self.stacks.get_mut(index).map(|s| s.as_u32_slice_mut())
    }
    fn stack_base(&self, index: usize) -> Option<u32> {
        self.stacks
            .get(index)
            .map(|s| s.as_u32_slice().as_ptr() as u32)
    }
}

impl<const N: usize, const SCHED: usize, S: StackStorage> PartitionCoreOps
    for PartitionCore<N, SCHED, S>
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
    use crate::partition::MpuRegion;
    use crate::scheduler::ScheduleEntry;

    type TestCore<const N: usize, const SCHED: usize> =
        super::PartitionCore<N, SCHED, AlignedStack1K>;

    fn test_pcb(id: u8) -> PartitionControlBlock {
        let o = (id as u32) * 0x1000;
        PartitionControlBlock::new(
            id,
            0x0800_0000 + o,
            0x2000_0000 + o,
            0x2000_0400 + o,
            MpuRegion::new(0x2000_0000 + o, 4096, 0),
        )
    }

    #[test]
    fn pcb_returns_correct_partition_by_index() {
        let mut core: TestCore<4, 4> = TestCore::new();
        core.partitions_mut().add(test_pcb(0)).unwrap();
        core.partitions_mut().add(test_pcb(1)).unwrap();
        assert_eq!(core.partition_count(), 2);
        assert_eq!(core.pcb(0).unwrap().id(), 0);
        assert_eq!(core.pcb(1).unwrap().id(), 1);
    }

    #[test]
    fn pcb_out_of_bounds_returns_none() {
        let mut core: TestCore<4, 4> = TestCore::new();
        core.partitions_mut().add(test_pcb(0)).unwrap();
        assert!(core.pcb(0).is_some());
        assert!(core.pcb(1).is_none());
        assert!(core.pcb(100).is_none());
    }

    #[test]
    fn pcb_mut_allows_mutation() {
        let mut core: TestCore<4, 4> = TestCore::new();
        core.partitions_mut().add(test_pcb(0)).unwrap();
        assert_eq!(core.pcb(0).unwrap().event_flags(), 0);
        core.pcb_mut(0).unwrap().set_event_flags(0xCD);
        assert_eq!(core.pcb(0).unwrap().event_flags(), 0xCD);
    }

    #[test]
    fn construction_and_field_access() {
        let mut core: TestCore<4, 8> = TestCore::new();
        assert!(core.partitions().is_empty());
        assert!(core.schedule().is_empty());
        assert_eq!(core.current_partition(), 0);
        assert_eq!(core.next_partition(), 0);
        assert_eq!(core.partition_sp(), &[0u32; 4]);
        let dflt: TestCore<4, 8> = TestCore::default();
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
        let core: TestCore<2, 4> = TestCore::new();
        // All stack arrays should be zero-initialized
        for i in 0..2 {
            let stack = &core.stacks()[i].0;
            assert!(stack.iter().all(|&w| w == 0));
        }
    }

    #[test]
    fn stacks_are_1024_byte_aligned() {
        let core: TestCore<3, 4> = TestCore::new();
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
        let core: TestCore<2, 4> = TestCore::new();
        assert!(core.stack_base(0).is_some());
        assert!(core.stack_base(1).is_some());
        assert!(core.stack_base(2).is_none());
        assert!(core.stack_base(100).is_none());
    }

    #[test]
    fn stack_mut_accessor() {
        let mut core: TestCore<2, 4> = TestCore::new();
        {
            let stack = core.stack_mut(0).unwrap();
            stack[0] = 0xDEAD_BEEF;
            stack[255] = 0xCAFE_BABE;
        }
        assert_eq!(core.stacks()[0].0[0], 0xDEAD_BEEF);
        assert_eq!(core.stacks()[0].0[255], 0xCAFE_BABE);
        // Out of bounds returns None
        assert!(core.stack_mut(2).is_none());
    }

    #[test]
    fn stacks_mut_accessor() {
        let mut core: TestCore<2, 4> = TestCore::new();
        // Write different values to each stack
        core.stacks_mut()[0].0[0] = 0x1111_1111;
        core.stacks_mut()[1].0[0] = 0x2222_2222;
        assert_eq!(core.stacks()[0].0[0], 0x1111_1111);
        assert_eq!(core.stacks()[1].0[0], 0x2222_2222);
    }

    #[test]
    fn aligned_stack_1k_is_1024_bytes() {
        assert_eq!(core::mem::size_of::<AlignedStack1K>(), 1024);
        assert_eq!(core::mem::align_of::<AlignedStack1K>(), 1024);
    }

    #[test]
    fn stack_size_returns_bytes() {
        let core: TestCore<2, 4> = TestCore::new();
        assert_eq!(core.stack_size(), 1024);
    }

    #[test]
    fn stack_size_different_tiers() {
        let c: super::PartitionCore<1, 1, AlignedStack512B> = super::PartitionCore::new();
        assert_eq!(c.stack_size(), 512);
        let c: super::PartitionCore<1, 1, AlignedStack256B> = super::PartitionCore::new();
        assert_eq!(c.stack_size(), 256);
    }

    #[test]
    fn four_partition_1k_stacks_use_4kb() {
        let stacks_size = 4 * core::mem::size_of::<AlignedStack1K>();
        assert_eq!(stacks_size, 4096);
    }

    #[test]
    fn stack_ptr_init_returns_top_of_stack() {
        let core: TestCore<2, 4> = TestCore::new();
        for i in 0..2 {
            let init_sp = core.stack_ptr_init(i).unwrap();
            let base = core.stack_base(i).unwrap();
            assert_eq!(init_sp, base + core.stack_size() as u32);
        }
    }

    #[test]
    fn stack_ptr_init_out_of_bounds_returns_none() {
        let core: TestCore<2, 4> = TestCore::new();
        assert!(core.stack_ptr_init(0).is_some());
        assert!(core.stack_ptr_init(1).is_some());
        assert!(core.stack_ptr_init(2).is_none());
    }

    macro_rules! tiered_stack_tests {
        ($name:ident, $ty:ty, $words:expr, $align:expr) => {
            mod $name {
                use super::*;
                #[test]
                fn layout_and_constants() {
                    assert_eq!(
                        core::mem::size_of::<$ty>(),
                        <$ty as StackStorage>::SIZE_BYTES
                    );
                    assert_eq!(
                        core::mem::align_of::<$ty>(),
                        <$ty as StackStorage>::ALIGNMENT
                    );
                    assert_eq!(
                        <$ty as StackStorage>::ALIGNMENT,
                        <$ty as StackStorage>::SIZE_BYTES
                    );
                    assert_eq!(<$ty as StackStorage>::WORDS, $words);
                }
                #[test]
                fn zero_init_and_slice_roundtrip() {
                    let z = <$ty as StackStorage>::ZERO;
                    assert!(z.as_u32_slice().iter().all(|&w| w == 0));
                    assert_eq!(z.as_u32_slice().len(), $words);
                    let d = <$ty>::default();
                    assert!(d.as_u32_slice().iter().all(|&w| w == 0));
                    let mut s = <$ty>::default();
                    s.as_u32_slice_mut()[0] = 0xDEAD_BEEF;
                    assert_eq!(s.as_u32_slice()[0], 0xDEAD_BEEF);
                }
            }
        };
    }
    tiered_stack_tests!(stack_256b, AlignedStack256B, 64, 256);
    tiered_stack_tests!(stack_512b, AlignedStack512B, 128, 512);
    tiered_stack_tests!(stack_1k, AlignedStack1K, 256, 1024);
    tiered_stack_tests!(stack_2k, AlignedStack2K, 512, 2048);
    tiered_stack_tests!(stack_4k, AlignedStack4K, 1024, 4096);

    /// Verify all stack tier natural-alignment invariants and kernel coverage.
    /// Checks per tier: ALIGNMENT==SIZE_BYTES, mem::align_of==ALIGNMENT,
    /// mem::size_of==SIZE_BYTES, ALIGNMENT is power-of-two (MPU requirement),
    /// SIZE_BYTES==WORDS*4, and KERNEL_ALIGNMENT >= ALIGNMENT.
    #[test]
    fn stack_tier_natural_alignment_and_kernel_coverage() {
        use crate::state::KERNEL_ALIGNMENT;

        macro_rules! check_tier {
            ($ty:ty, $label:expr) => {{
                let align = <$ty as StackStorage>::ALIGNMENT;
                let size = <$ty as StackStorage>::SIZE_BYTES;
                let words = <$ty as StackStorage>::WORDS;
                assert_eq!(align, size, "{}: ALIGNMENT != SIZE_BYTES", $label);
                assert!(
                    align.is_power_of_two(),
                    "{}: ALIGNMENT ({}) not power of two (MPU requirement)",
                    $label,
                    align
                );
                assert_eq!(
                    size,
                    words * 4,
                    "{}: SIZE_BYTES ({}) != WORDS ({}) * 4",
                    $label,
                    size,
                    words
                );
                assert_eq!(
                    core::mem::align_of::<$ty>(),
                    align,
                    "{}: mem::align_of != ALIGNMENT",
                    $label
                );
                assert_eq!(
                    core::mem::size_of::<$ty>(),
                    size,
                    "{}: mem::size_of != SIZE_BYTES",
                    $label
                );
                assert!(
                    KERNEL_ALIGNMENT >= align,
                    "KERNEL_ALIGNMENT ({}) < {} ALIGNMENT ({})",
                    KERNEL_ALIGNMENT,
                    $label,
                    align
                );
            }};
        }

        check_tier!(AlignedStack256B, "AlignedStack256B");
        check_tier!(AlignedStack512B, "AlignedStack512B");
        check_tier!(AlignedStack1K, "AlignedStack1K");
        check_tier!(AlignedStack2K, "AlignedStack2K");
        check_tier!(AlignedStack4K, "AlignedStack4K");
    }
}
