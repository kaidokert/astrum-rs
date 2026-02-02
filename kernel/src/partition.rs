use heapless::Vec;

use crate::mpu::MpuError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionState {
    Ready,
    Running,
    Waiting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuRegion {
    base: u32,
    size: u32,
    permissions: u32,
}

impl MpuRegion {
    pub const fn new(base: u32, size: u32, permissions: u32) -> Self {
        Self {
            base,
            size,
            permissions,
        }
    }

    pub fn base(&self) -> u32 {
        self.base
    }

    pub fn size(&self) -> u32 {
        self.size
    }

    pub fn permissions(&self) -> u32 {
        self.permissions
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartitionControlBlock {
    id: u8,
    state: PartitionState,
    stack_pointer: u32,
    entry_point: u32,
    stack_base: u32,
    stack_size: u32,
    mpu_region: MpuRegion,
    event_flags: u32,
    event_wait_mask: u32,
}

impl PartitionControlBlock {
    pub fn new(
        id: u8,
        entry_point: u32,
        stack_base: u32,
        stack_pointer: u32,
        mpu_region: MpuRegion,
    ) -> Self {
        Self {
            id,
            state: PartitionState::Ready,
            stack_pointer,
            entry_point,
            stack_base,
            stack_size: stack_pointer.wrapping_sub(stack_base),
            mpu_region,
            event_flags: 0,
            event_wait_mask: 0,
        }
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn state(&self) -> PartitionState {
        self.state
    }

    pub fn stack_pointer(&self) -> u32 {
        self.stack_pointer
    }

    pub fn entry_point(&self) -> u32 {
        self.entry_point
    }

    pub fn stack_base(&self) -> u32 {
        self.stack_base
    }

    pub fn stack_size(&self) -> u32 {
        self.stack_size
    }

    pub fn mpu_region(&self) -> &MpuRegion {
        &self.mpu_region
    }

    pub fn event_flags(&self) -> u32 {
        self.event_flags
    }

    pub fn set_event_flags(&mut self, bits: u32) {
        self.event_flags |= bits;
    }

    pub fn clear_event_flags(&mut self, bits: u32) {
        self.event_flags &= !bits;
    }

    pub fn event_wait_mask(&self) -> u32 {
        self.event_wait_mask
    }

    pub fn set_event_wait_mask(&mut self, mask: u32) {
        self.event_wait_mask = mask;
    }

    pub fn transition(&mut self, to: PartitionState) -> Result<(), TransitionError> {
        let ok = matches!(
            (self.state, to),
            (PartitionState::Ready, PartitionState::Running)
                | (PartitionState::Running, PartitionState::Ready)
                | (PartitionState::Running, PartitionState::Waiting)
                | (PartitionState::Waiting, PartitionState::Ready)
        );
        if ok {
            self.state = to;
            Ok(())
        } else {
            Err(TransitionError)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PartitionConfig {
    pub id: u8,
    pub entry_point: u32,
    pub stack_base: u32,
    pub stack_size: u32,
    pub mpu_region: MpuRegion,
}

/// Errors detected during static configuration validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// The schedule table has no entries.
    ScheduleEmpty,
    /// A schedule entry references a partition index that does not exist.
    ScheduleIndexOutOfBounds {
        entry_index: usize,
        partition_index: u8,
        num_partitions: usize,
    },
    /// A partition's MPU region failed validation.
    MpuRegionInvalid { partition_id: u8, detail: MpuError },
    /// A partition's stack size is not a power of two or is less than 32.
    StackSizeInvalid { partition_id: u8 },
    /// A partition's stack base is not aligned to its stack size.
    StackBaseNotAligned { partition_id: u8 },
    /// A partition's stack base + stack size overflows u32.
    StackOverflow { partition_id: u8 },
    /// The partition table is full; no room for another partition.
    PartitionTableFull,
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ScheduleEmpty => write!(f, "schedule table is empty"),
            Self::ScheduleIndexOutOfBounds {
                entry_index,
                partition_index,
                num_partitions,
            } => write!(
                f,
                "schedule entry {entry_index}: partition index {partition_index} \
                 out of bounds (num_partitions={num_partitions})"
            ),
            Self::MpuRegionInvalid {
                partition_id,
                detail,
            } => write!(f, "partition {partition_id}: MPU region invalid: {detail}"),
            Self::StackSizeInvalid { partition_id } => write!(
                f,
                "partition {partition_id}: stack size must be a power of two and >= 32"
            ),
            Self::StackBaseNotAligned { partition_id } => write!(
                f,
                "partition {partition_id}: stack base not aligned to stack size"
            ),
            Self::StackOverflow { partition_id } => write!(
                f,
                "partition {partition_id}: stack base + stack size overflows u32"
            ),
            Self::PartitionTableFull => write!(f, "partition table is full"),
        }
    }
}

/// Fixed-capacity table of partition control blocks.
pub struct PartitionTable<const N: usize> {
    partitions: Vec<PartitionControlBlock, N>,
}

impl<const N: usize> Default for PartitionTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> PartitionTable<N> {
    pub const fn new() -> Self {
        Self {
            partitions: Vec::new(),
        }
    }

    pub fn add(&mut self, pcb: PartitionControlBlock) -> Result<(), PartitionControlBlock> {
        self.partitions.push(pcb)
    }

    pub fn get(&self, index: usize) -> Option<&PartitionControlBlock> {
        self.partitions.get(index)
    }

    pub fn get_mut(&mut self, index: usize) -> Option<&mut PartitionControlBlock> {
        self.partitions.get_mut(index)
    }

    pub fn len(&self) -> usize {
        self.partitions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.partitions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &PartitionControlBlock> {
        self.partitions.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pcb() -> PartitionControlBlock {
        PartitionControlBlock::new(
            1,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
    }

    #[test]
    fn new_partition_is_ready_with_correct_fields() {
        let pcb = make_pcb();
        assert_eq!(pcb.state(), PartitionState::Ready);
        assert_eq!(pcb.id(), 1);
        assert_eq!(pcb.stack_pointer(), 0x2000_0000 + 1024);
        assert_eq!(pcb.entry_point(), 0x0800_0000);
        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 4096);
        assert_eq!(pcb.mpu_region().permissions(), 0x0306_0000);
        assert_eq!(pcb.event_flags(), 0);
    }

    #[test]
    fn valid_transitions() {
        let mut pcb = make_pcb();
        pcb.transition(PartitionState::Running).unwrap();
        assert_eq!(pcb.state(), PartitionState::Running);
        pcb.transition(PartitionState::Waiting).unwrap();
        assert_eq!(pcb.state(), PartitionState::Waiting);
        pcb.transition(PartitionState::Ready).unwrap();
        assert_eq!(pcb.state(), PartitionState::Ready);
        pcb.transition(PartitionState::Running).unwrap();
        pcb.transition(PartitionState::Ready).unwrap();
    }

    #[test]
    fn invalid_transitions_rejected() {
        let mut pcb = make_pcb();
        assert!(pcb.transition(PartitionState::Waiting).is_err());
        assert_eq!(pcb.state(), PartitionState::Ready);
        assert!(pcb.transition(PartitionState::Ready).is_err());
        pcb.transition(PartitionState::Running).unwrap();
        pcb.transition(PartitionState::Waiting).unwrap();
        assert!(pcb.transition(PartitionState::Running).is_err());
        assert!(pcb.transition(PartitionState::Waiting).is_err());
        assert_eq!(pcb.state(), PartitionState::Waiting);
    }

    #[test]
    fn partition_table_add_and_get() {
        let mut table: PartitionTable<4> = PartitionTable::new();
        assert!(table.is_empty());
        assert_eq!(table.len(), 0);

        let pcb = make_pcb();
        assert!(table.add(pcb).is_ok());
        assert_eq!(table.len(), 1);
        assert!(!table.is_empty());

        let retrieved = table.get(0).unwrap();
        assert_eq!(retrieved.id(), 1);
    }

    #[test]
    fn partition_table_full_rejects() {
        let mut table: PartitionTable<2> = PartitionTable::new();
        let pcb1 = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        );
        let pcb2 = PartitionControlBlock::new(
            1,
            0x0800_1000,
            0x2000_1000,
            0x2000_1400,
            MpuRegion::new(0x2000_1000, 4096, 0x0306_0000),
        );
        let pcb3 = PartitionControlBlock::new(
            2,
            0x0800_2000,
            0x2000_2000,
            0x2000_2400,
            MpuRegion::new(0x2000_2000, 4096, 0x0306_0000),
        );
        assert!(table.add(pcb1).is_ok());
        assert!(table.add(pcb2).is_ok());
        assert!(table.add(pcb3).is_err());
        assert_eq!(table.len(), 2);
    }

    #[test]
    fn partition_table_get_mut_allows_transition() {
        let mut table: PartitionTable<4> = PartitionTable::new();
        table.add(make_pcb()).unwrap();

        let pcb = table.get_mut(0).unwrap();
        pcb.transition(PartitionState::Running).unwrap();
        assert_eq!(pcb.state(), PartitionState::Running);

        assert_eq!(table.get(0).unwrap().state(), PartitionState::Running);
    }

    #[test]
    fn pcb_is_copy() {
        let pcb = make_pcb();
        let copy = pcb;
        // both are usable — proves Copy
        assert_eq!(pcb.id(), copy.id());
    }

    // ------------------------------------------------------------------
    // ConfigError
    // ------------------------------------------------------------------

    #[test]
    fn config_error_is_copy() {
        let e = ConfigError::ScheduleEmpty;
        let e2 = e;
        assert_eq!(e, e2);
    }

    #[test]
    fn config_error_variants_are_distinct() {
        let variants: &[ConfigError] = &[
            ConfigError::ScheduleEmpty,
            ConfigError::ScheduleIndexOutOfBounds {
                entry_index: 0,
                partition_index: 5,
                num_partitions: 4,
            },
            ConfigError::MpuRegionInvalid {
                partition_id: 1,
                detail: MpuError::SizeTooSmall,
            },
            ConfigError::StackSizeInvalid { partition_id: 2 },
            ConfigError::StackBaseNotAligned { partition_id: 3 },
            ConfigError::StackOverflow { partition_id: 4 },
            ConfigError::PartitionTableFull,
        ];
        for (i, a) in variants.iter().enumerate() {
            for (j, b) in variants.iter().enumerate() {
                if i == j {
                    assert_eq!(a, b);
                } else {
                    assert_ne!(a, b, "variants {i} and {j} should differ");
                }
            }
        }
    }

    #[test]
    fn config_error_display_schedule_empty() {
        let msg = format!("{}", ConfigError::ScheduleEmpty);
        assert_eq!(msg, "schedule table is empty");
    }

    #[test]
    fn config_error_display_schedule_index_out_of_bounds() {
        let msg = format!(
            "{}",
            ConfigError::ScheduleIndexOutOfBounds {
                entry_index: 2,
                partition_index: 7,
                num_partitions: 4,
            }
        );
        assert!(msg.contains("entry 2"));
        assert!(msg.contains("partition index 7"));
        assert!(msg.contains("num_partitions=4"));
    }

    #[test]
    fn config_error_display_mpu_region_invalid() {
        let msg = format!(
            "{}",
            ConfigError::MpuRegionInvalid {
                partition_id: 3,
                detail: MpuError::BaseNotAligned,
            }
        );
        assert!(msg.contains("partition 3"));
        assert!(msg.contains("base address not aligned to size"));
    }

    #[test]
    fn config_error_display_stack_size_invalid() {
        let msg = format!("{}", ConfigError::StackSizeInvalid { partition_id: 1 });
        assert!(msg.contains("partition 1"));
        assert!(msg.contains("power of two"));
    }

    #[test]
    fn config_error_display_stack_base_not_aligned() {
        let msg = format!("{}", ConfigError::StackBaseNotAligned { partition_id: 2 });
        assert!(msg.contains("partition 2"));
        assert!(msg.contains("not aligned"));
    }

    #[test]
    fn config_error_display_stack_overflow() {
        let msg = format!("{}", ConfigError::StackOverflow { partition_id: 5 });
        assert!(msg.contains("partition 5"));
        assert!(msg.contains("overflows"));
    }

    #[test]
    fn config_error_display_partition_table_full() {
        let msg = format!("{}", ConfigError::PartitionTableFull);
        assert_eq!(msg, "partition table is full");
    }

    #[test]
    fn config_error_debug_contains_variant_names() {
        assert!(format!("{:?}", ConfigError::ScheduleEmpty).contains("ScheduleEmpty"));
        assert!(format!(
            "{:?}",
            ConfigError::ScheduleIndexOutOfBounds {
                entry_index: 0,
                partition_index: 1,
                num_partitions: 2,
            }
        )
        .contains("ScheduleIndexOutOfBounds"));
        assert!(format!(
            "{:?}",
            ConfigError::MpuRegionInvalid {
                partition_id: 0,
                detail: MpuError::SizeTooSmall,
            }
        )
        .contains("MpuRegionInvalid"));
        assert!(
            format!("{:?}", ConfigError::StackSizeInvalid { partition_id: 0 })
                .contains("StackSizeInvalid")
        );
        assert!(
            format!("{:?}", ConfigError::StackBaseNotAligned { partition_id: 0 })
                .contains("StackBaseNotAligned")
        );
        assert!(
            format!("{:?}", ConfigError::StackOverflow { partition_id: 0 })
                .contains("StackOverflow")
        );
        assert!(format!("{:?}", ConfigError::PartitionTableFull).contains("PartitionTableFull"));
    }

    #[test]
    fn config_error_mpu_wraps_all_mpu_error_variants() {
        let mpu_variants = [
            MpuError::RegionCountMismatch,
            MpuError::SizeTooSmall,
            MpuError::SizeNotPowerOfTwo,
            MpuError::BaseNotAligned,
            MpuError::AddressOverflow,
            MpuError::SlotExhausted,
        ];
        for detail in mpu_variants {
            let e = ConfigError::MpuRegionInvalid {
                partition_id: 0,
                detail,
            };
            let msg = format!("{e}");
            let detail_msg = format!("{detail}");
            assert!(
                msg.contains(&detail_msg),
                "ConfigError display should contain MpuError display: {msg}"
            );
        }
    }
}
