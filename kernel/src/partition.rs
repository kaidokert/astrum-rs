use heapless::Vec;

#[cfg(feature = "partition-debug")]
use crate::debug::DebugBuffer;
use crate::mpu::{validate_mpu_region, MpuError, AP_FULL_ACCESS, RASR_AP_SHIFT};

/// Default data-region RASR attributes: full read-write access with
/// Normal memory (TEX=0, S=1, C=1, B=0).  The RASR enable bit and size
/// field are composed by `build_rasr` when programming the hardware.
pub const SENTINEL_DATA_PERMISSIONS: u32 =
    (AP_FULL_ACCESS << RASR_AP_SHIFT) | (1 << 18) | (1 << 17);

/// Wrapper for a reference to a debug buffer trait object.
///
/// This wrapper provides Clone, Debug, PartialEq, and Eq implementations
/// required by `PartitionControlBlock`'s derived traits.
#[cfg(feature = "partition-debug")]
#[derive(Clone, Copy)]
struct DebugBufferRef(Option<&'static dyn DebugBuffer>);

#[cfg(feature = "partition-debug")]
impl DebugBufferRef {
    const fn none() -> Self {
        Self(None)
    }
}

#[cfg(feature = "partition-debug")]
impl core::fmt::Debug for DebugBufferRef {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Some(buf) => write!(f, "DebugBufferRef(Some({:p}))", buf),
            None => write!(f, "DebugBufferRef(None)"),
        }
    }
}

#[cfg(feature = "partition-debug")]
impl PartialEq for DebugBufferRef {
    fn eq(&self, other: &Self) -> bool {
        match (self.0, other.0) {
            (Some(a), Some(b)) => core::ptr::eq(a, b),
            (None, None) => true,
            _ => false,
        }
    }
}

#[cfg(feature = "partition-debug")]
impl Eq for DebugBufferRef {}

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

    /// Returns `true` when the region's base and size satisfy all ARMv7-M
    /// MPU constraints (minimum 32 bytes, power-of-two size, aligned base).
    pub fn is_mappable(&self) -> bool {
        validate_mpu_region(self.base, self.size).is_ok()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
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
    /// Optional peripheral register block regions for user-space drivers.
    /// Supports up to 3 peripheral regions (mapped to MPU R4–R6).
    peripheral_regions: Vec<MpuRegion, 3>,
    /// Flag set by SYS_DEBUG_NOTIFY syscall, cleared after kernel drains debug ring buffer.
    debug_pending: bool,
    /// Reference to the partition's debug ring buffer (type-erased via trait object).
    #[cfg(feature = "partition-debug")]
    debug_buffer: DebugBufferRef,
    /// Pre-computed (RBAR, RASR) pairs for base MPU regions R0–R3.
    cached_base_regions: [(u32, u32); 4],
    /// Pre-computed (RBAR, RASR) pairs for peripheral MPU regions R4–R6.
    cached_periph_regions: [(u32, u32); 3],
    /// Guard: set true after [`precompute_mpu_cache`] to catch post-boot
    /// mutations that would silently invalidate the cached MPU registers.
    cache_sealed: bool,
    /// Tick at which the partition should wake from `SYS_SLEEP`.
    /// Zero means "not sleeping".
    sleep_until: u64,
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
            peripheral_regions: Vec::new(),
            debug_pending: false,
            #[cfg(feature = "partition-debug")]
            debug_buffer: DebugBufferRef::none(),
            cached_base_regions: [(0, 0); 4],
            cached_periph_regions: [(0, 0); 3],
            cache_sealed: false,
            sleep_until: 0,
        }
    }

    /// Add up to 3 peripheral regions. Only non-zero-size regions are stored.
    pub fn with_peripheral_regions(mut self, regions: &[MpuRegion]) -> Self {
        self.peripheral_regions.clear();
        for region in regions.iter().take(3) {
            if region.size() > 0 {
                let _ = self.peripheral_regions.push(*region);
            }
        }
        self
    }

    /// Returns the peripheral regions configured for this partition.
    pub fn peripheral_regions(&self) -> &[MpuRegion] {
        &self.peripheral_regions
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

    /// Returns the stack region bounds as (stack_base, stack_size).
    ///
    /// This is useful for pointer validation to check if an address
    /// falls within the partition's stack region.
    pub fn stack_region(&self) -> (u32, u32) {
        (self.stack_base, self.stack_size)
    }

    /// Returns (base, size) pairs: data region, stack, then peripheral regions.
    /// Only non-zero-size regions are included. Capacity is 5 (1 data + 1 stack + 3 peripherals).
    pub fn accessible_static_regions(&self) -> Vec<(u32, u32), 5> {
        let mut regions = Vec::new();
        let data_size = self.mpu_region.size();
        if data_size > 0 {
            let _ = regions.push((self.mpu_region.base(), data_size));
        }
        if self.stack_size > 0 {
            let _ = regions.push((self.stack_base, self.stack_size));
        }
        for pr in self.peripheral_regions.iter() {
            let _ = regions.push((pr.base(), pr.size()));
        }
        regions
    }

    /// Returns only stack and peripheral regions (excluding the data/mpu_region).
    ///
    /// Used by the overlap invariant check: data regions may legitimately be
    /// shared across partitions, while stacks and peripherals must be exclusive.
    ///
    /// Capacity 4 assumes 1 stack + at most 3 peripheral regions.
    pub fn exclusive_static_regions(&self) -> Vec<(u32, u32), 4> {
        debug_assert!(
            self.peripheral_regions.len() <= 3,
            "peripheral_regions exceeds 3"
        );
        let mut regions = Vec::new();
        if self.stack_size > 0 {
            let ok = regions.push((self.stack_base, self.stack_size)).is_ok();
            debug_assert!(ok, "stack push exceeded capacity 4");
        }
        for pr in self.peripheral_regions.iter() {
            if pr.size() > 0 {
                let ok = regions.push((pr.base(), pr.size())).is_ok();
                debug_assert!(ok, "peripheral push exceeded capacity 4");
            }
        }
        regions
    }

    pub fn mpu_region(&self) -> &MpuRegion {
        &self.mpu_region
    }

    /// Updates the stack region bounds.
    ///
    /// This method validates the new region parameters against MPU hardware
    /// constraints, then updates `stack_base` and `stack_size`. The data region
    /// (`mpu_region`) is preserved unchanged.
    ///
    /// # Arguments
    /// * `base` - The new stack base address (must be aligned to `size`)
    /// * `size` - The new stack size (must be a power of two and >= 32)
    ///
    /// # Errors
    /// Returns `MpuError` if the base/size violate MPU constraints:
    /// - `SizeTooSmall`: size < 32 bytes
    /// - `SizeNotPowerOfTwo`: size is not a power of two
    /// - `BaseNotAligned`: base is not aligned to size
    /// - `AddressOverflow`: base + size overflows u32
    pub fn fix_stack_region(&mut self, base: u32, size: u32) -> Result<(), MpuError> {
        assert!(
            !self.cache_sealed,
            "fix_stack_region called after MPU cache sealed"
        );
        // Validate MPU constraints before modifying any state
        validate_mpu_region(base, size)?;

        // Only update stack fields; preserve mpu_region (data region) unchanged
        self.stack_base = base;
        self.stack_size = size;

        Ok(())
    }

    /// Updates the MPU data region base address.
    ///
    /// After boot-time stack relocation, the PCB's `mpu_region.base` (set from
    /// `PartitionConfig`) may not match the actual stack buffer address inside
    /// `PartitionCore`. This method patches the base to the real runtime
    /// address, preserving the original size and permissions.
    pub fn fix_mpu_data_region(&mut self, base: u32) {
        assert!(
            !self.cache_sealed,
            "fix_mpu_data_region called after MPU cache sealed"
        );
        self.mpu_region =
            MpuRegion::new(base, self.mpu_region.size(), self.mpu_region.permissions());
    }

    /// Replaces a sentinel MPU region (size==0) with a fully specified region
    /// containing the actual stack buffer base, size, and data-region permissions.
    ///
    /// # Errors
    /// Returns `MpuError::AlreadyInitialized` if this partition's MPU region
    /// is not a sentinel (i.e., `mpu_region.size() != 0`).
    /// Returns alignment/size errors from [`validate_mpu_region`] if the
    /// supplied `base`/`size` violate ARMv7-M MPU constraints.
    pub fn promote_sentinel_mpu(
        &mut self,
        base: u32,
        size: u32,
        permissions: u32,
    ) -> Result<(), MpuError> {
        assert!(
            !self.cache_sealed,
            "promote_sentinel_mpu called after MPU cache sealed"
        );
        if self.mpu_region.size() != 0 {
            return Err(MpuError::AlreadyInitialized);
        }
        validate_mpu_region(base, size)?;
        self.mpu_region = MpuRegion::new(base, size, permissions);
        Ok(())
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

    /// Returns whether this partition has pending debug output.
    pub fn debug_pending(&self) -> bool {
        self.debug_pending
    }

    /// Signals that this partition has pending debug output.
    /// Called by SYS_DEBUG_NOTIFY syscall handler.
    pub fn signal_debug_pending(&mut self) {
        self.debug_pending = true;
    }

    /// Clears the debug pending flag after kernel drains the debug ring buffer.
    pub fn clear_debug_pending(&mut self) {
        self.debug_pending = false;
    }

    /// Returns the tick at which this partition should wake from sleep.
    /// Zero means "not sleeping".
    pub fn sleep_until(&self) -> u64 {
        self.sleep_until
    }

    /// Sets the tick at which this partition should wake from sleep.
    pub fn set_sleep_until(&mut self, tick: u64) {
        self.sleep_until = tick;
    }

    /// Returns the pre-computed (RBAR, RASR) pairs for base MPU regions R0–R3.
    pub fn cached_base_regions(&self) -> &[(u32, u32); 4] {
        &self.cached_base_regions
    }

    /// Sets the pre-computed (RBAR, RASR) pairs for base MPU regions R0–R3.
    pub fn set_cached_base_regions(
        &mut self,
        regions: [(u32, u32); 4],
    ) -> Result<(), &'static str> {
        if self.cache_sealed {
            return Err("set_cached_base_regions called after MPU cache sealed");
        }
        self.cached_base_regions = regions;
        Ok(())
    }

    /// Returns the pre-computed (RBAR, RASR) pairs for peripheral MPU regions R4–R6.
    pub fn cached_periph_regions(&self) -> &[(u32, u32); 3] {
        &self.cached_periph_regions
    }

    /// Sets the pre-computed (RBAR, RASR) pairs for peripheral MPU regions R4–R6.
    pub fn set_cached_periph_regions(
        &mut self,
        regions: [(u32, u32); 3],
    ) -> Result<(), &'static str> {
        if self.cache_sealed {
            return Err("set_cached_periph_regions called after MPU cache sealed");
        }
        self.cached_periph_regions = regions;
        Ok(())
    }

    /// Mark the MPU cache as sealed. Called by [`precompute_mpu_cache`] after
    /// boot-time fixups are complete. Any subsequent mutation of MPU-affecting
    /// fields will trip a debug assertion.
    pub fn seal_cache(&mut self) {
        self.cache_sealed = true;
    }

    /// Returns whether the MPU cache has been sealed (debug-only).
    pub fn cache_sealed(&self) -> bool {
        self.cache_sealed
    }

    /// Returns a reference to this partition's debug ring buffer, if set.
    #[cfg(feature = "partition-debug")]
    pub fn debug_buffer(&self) -> Option<&'static dyn DebugBuffer> {
        self.debug_buffer.0
    }

    /// Sets this partition's debug ring buffer.
    #[cfg(feature = "partition-debug")]
    pub fn set_debug_buffer<const N: usize>(
        &mut self,
        buffer: &'static crate::debug::DebugRingBuffer<N>,
    ) {
        self.debug_buffer = DebugBufferRef(Some(buffer as &'static dyn DebugBuffer));
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

#[derive(Debug, Clone)]
pub struct PartitionConfig {
    pub id: u8,
    pub entry_point: u32,
    pub stack_base: u32,
    pub stack_size: u32,
    pub mpu_region: MpuRegion,
    /// Optional peripheral register block regions for user-space drivers.
    /// Supports up to 2 peripheral regions (Approach D).
    pub peripheral_regions: Vec<MpuRegion, 2>,
}

impl PartitionConfig {
    /// Create a fully specified partition config with no peripheral regions.
    ///
    /// Use this when the partition has real stack addresses and MPU regions
    /// known at build time.  `peripheral_regions` defaults to empty; chain
    /// field assignment if peripherals are needed.
    pub fn new(
        id: u8,
        entry_point: u32,
        stack_base: u32,
        stack_size: u32,
        mpu_region: MpuRegion,
    ) -> Self {
        Self {
            id,
            entry_point,
            stack_base,
            stack_size,
            mpu_region,
            peripheral_regions: Vec::new(),
        }
    }

    /// Create a sentinel (default) partition config.
    ///
    /// Sets `entry_point`, `stack_base`, and `mpu_region` to zero sentinels,
    /// with an empty `peripheral_regions` vector.  Only `id` and `stack_size`
    /// (in bytes) are caller-supplied.
    pub fn sentinel(id: u8, stack_size_bytes: u32) -> Self {
        Self {
            id,
            entry_point: 0,
            stack_base: 0,
            stack_size: stack_size_bytes,
            mpu_region: MpuRegion::new(0, 0, 0),
            peripheral_regions: Vec::new(),
        }
    }

    /// Create an array of `N` sentinel partition configs.
    ///
    /// Each element gets `id = index` and
    /// `stack_size = stack_words * size_of::<u32>()` bytes.
    ///
    /// # Panics
    ///
    /// Panics if `N > 256` (partition ID is `u8`) or if the byte-level
    /// stack size overflows `u32`.
    pub fn sentinel_array<const N: usize>(stack_words: usize) -> [PartitionConfig; N] {
        assert!(
            N <= 256,
            "sentinel_array: N must be <= 256 (partition ID is u8)"
        );
        let stack_bytes: u32 = stack_words
            .checked_mul(core::mem::size_of::<u32>())
            .and_then(|b| u32::try_from(b).ok())
            .expect("sentinel_array: stack size in bytes overflows u32");
        core::array::from_fn(|i| Self::sentinel(i as u8, stack_bytes))
    }

    /// Validate all fields of this partition configuration.
    ///
    /// Checks performed (in order):
    /// 1. `stack_size` must be a power of two and >= 32.
    /// 2. `stack_base` must be aligned to `stack_size`.
    /// 3. `stack_base + stack_size` must not overflow `u32`.
    /// 4. The MPU region `(base, size)` must pass [`validate_mpu_region`].
    /// 5. Each peripheral region `(base, size)` must pass [`validate_mpu_region`].
    pub fn validate(&self) -> Result<(), ConfigError> {
        // Stack size: power of two and >= 32
        if self.stack_size < 32 || !self.stack_size.is_power_of_two() {
            return Err(ConfigError::StackSizeInvalid {
                partition_id: self.id,
            });
        }

        // Stack base alignment
        if self.stack_base & (self.stack_size - 1) != 0 {
            return Err(ConfigError::StackBaseNotAligned {
                partition_id: self.id,
            });
        }

        // Stack overflow check
        if self.stack_base.checked_add(self.stack_size).is_none() {
            return Err(ConfigError::StackOverflow {
                partition_id: self.id,
            });
        }

        // MPU region validation
        validate_mpu_region(self.mpu_region.base(), self.mpu_region.size()).map_err(|detail| {
            ConfigError::MpuRegionInvalid {
                partition_id: self.id,
                detail,
            }
        })?;

        // Peripheral region validation
        for (i, region) in self.peripheral_regions.iter().enumerate() {
            validate_mpu_region(region.base(), region.size()).map_err(|detail| {
                ConfigError::PeripheralRegionInvalid {
                    partition_id: self.id,
                    region_index: i,
                    detail,
                }
            })?;
        }

        Ok(())
    }
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
    /// A partition's id does not match its array index.
    PartitionIdMismatch {
        index: usize,
        expected_id: u8,
        actual_id: u8,
    },
    /// Failed to access internal stack for a partition during initialization.
    StackInitFailed { partition_id: u8 },
    /// A partition's peripheral region failed MPU validation.
    PeripheralRegionInvalid {
        partition_id: u8,
        region_index: usize,
        detail: MpuError,
    },
    /// The schedule contains no system window entries.
    ///
    /// When `dynamic-mpu` is enabled, system windows are required for
    /// kernel bottom-half processing (buffer pool transfers, virtual
    /// device I/O, etc.). Without at least one system window in the
    /// schedule, bottom-half work will never run and IPC will stall.
    #[cfg(feature = "dynamic-mpu")]
    NoSystemWindow,
    /// The gap between system windows exceeds the acceptable threshold.
    ///
    /// When `dynamic-mpu` is enabled, system windows must occur frequently
    /// enough to service bottom-half work in a timely manner. If the maximum
    /// gap between consecutive system windows (including wrap-around) exceeds
    /// the configured threshold, real-time deadlines may be missed.
    #[cfg(feature = "dynamic-mpu")]
    SystemWindowTooInfrequent {
        /// The actual maximum gap in ticks between system windows.
        max_gap_ticks: u32,
        /// The threshold that was exceeded.
        threshold_ticks: u32,
    },
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
            Self::PartitionIdMismatch {
                index,
                expected_id,
                actual_id,
            } => write!(
                f,
                "partition config[{index}] has id {actual_id}, expected {expected_id} \
                 — ids must match array index"
            ),
            Self::StackInitFailed { partition_id } => {
                write!(
                    f,
                    "partition {partition_id}: failed to access internal stack"
                )
            }
            Self::PeripheralRegionInvalid {
                partition_id,
                region_index,
                detail,
            } => write!(
                f,
                "partition {partition_id}: peripheral region {region_index} invalid: {detail}"
            ),
            #[cfg(feature = "dynamic-mpu")]
            Self::NoSystemWindow => write!(
                f,
                "schedule contains no system window: bottom-half processing \
                 (buffer pool transfers, virtual device I/O) requires at least \
                 one system window entry"
            ),
            #[cfg(feature = "dynamic-mpu")]
            Self::SystemWindowTooInfrequent {
                max_gap_ticks,
                threshold_ticks,
            } => write!(
                f,
                "system window gap too large: max gap {max_gap_ticks} ticks \
                 exceeds threshold {threshold_ticks} ticks"
            ),
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

    #[allow(clippy::result_large_err)] // heapless::Vec::push returns the item on failure; boxing is not an option in no-alloc
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

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut PartitionControlBlock> {
        self.partitions.iter_mut()
    }

    /// Returns a slice of all partition control blocks.
    pub fn as_slice(&self) -> &[PartitionControlBlock] {
        &self.partitions
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
        assert!(!pcb.debug_pending());
    }

    #[test]
    fn debug_pending_initially_false() {
        let pcb = make_pcb();
        assert!(!pcb.debug_pending());
    }

    #[test]
    fn test_debug_pending_cycle() {
        let mut pcb = make_pcb();
        // Initially false
        assert!(!pcb.debug_pending());
        // Signal sets it
        pcb.signal_debug_pending();
        assert!(pcb.debug_pending());
        // Clear resets it
        pcb.clear_debug_pending();
        assert!(!pcb.debug_pending());
    }

    #[test]
    fn sleep_until_defaults_to_zero() {
        let pcb = make_pcb();
        assert_eq!(pcb.sleep_until(), 0);
    }

    #[test]
    fn sleep_until_set_and_clear() {
        let mut pcb = make_pcb();
        pcb.set_sleep_until(500);
        assert_eq!(pcb.sleep_until(), 500);
        pcb.set_sleep_until(0);
        assert_eq!(pcb.sleep_until(), 0);
    }

    #[test]
    fn stack_region_returns_base_and_size() {
        let pcb = make_pcb();
        let (base, size) = pcb.stack_region();
        assert_eq!(base, 0x2000_0000);
        // stack_size is computed as stack_pointer - stack_base = 0x2000_0400 - 0x2000_0000 = 1024
        assert_eq!(size, 1024);
        // Verify consistency with individual accessors
        assert_eq!(base, pcb.stack_base());
        assert_eq!(size, pcb.stack_size());
    }

    // ------------------------------------------------------------------
    // fix_stack_region
    // ------------------------------------------------------------------

    #[test]
    fn fix_stack_region_updates_stack_base() {
        let mut pcb = make_pcb();
        let original_base = pcb.stack_base();
        assert_eq!(original_base, 0x2000_0000);

        // 0x2001_0000 is aligned to 2048
        pcb.fix_stack_region(0x2001_0000, 2048).unwrap();
        assert_eq!(pcb.stack_base(), 0x2001_0000);
    }

    #[test]
    fn fix_stack_region_updates_stack_size() {
        let mut pcb = make_pcb();
        let original_size = pcb.stack_size();
        assert_eq!(original_size, 1024);

        // 0x2001_0000 is aligned to 2048
        pcb.fix_stack_region(0x2001_0000, 2048).unwrap();
        assert_eq!(pcb.stack_size(), 2048);
    }

    #[test]
    fn fix_stack_region_preserves_mpu_region() {
        let mut pcb = make_pcb();
        let original_mpu_base = pcb.mpu_region().base();
        let original_mpu_size = pcb.mpu_region().size();
        let original_mpu_permissions = pcb.mpu_region().permissions();

        // 0x2002_0000 is aligned to 8192
        pcb.fix_stack_region(0x2002_0000, 8192).unwrap();

        // Verify mpu_region (data region) is unchanged
        assert_eq!(pcb.mpu_region().base(), original_mpu_base);
        assert_eq!(pcb.mpu_region().size(), original_mpu_size);
        assert_eq!(pcb.mpu_region().permissions(), original_mpu_permissions);

        // Verify stack fields are updated
        assert_eq!(pcb.stack_base(), 0x2002_0000);
        assert_eq!(pcb.stack_size(), 8192);
    }

    #[test]
    fn fix_stack_region_stack_region_accessor_reflects_changes() {
        let mut pcb = make_pcb();
        // 0x2003_0000 is aligned to 4096
        pcb.fix_stack_region(0x2003_0000, 4096).unwrap();

        let (base, size) = pcb.stack_region();
        assert_eq!(base, 0x2003_0000);
        assert_eq!(size, 4096);
    }

    #[test]
    fn fix_stack_region_rejects_size_too_small() {
        let mut pcb = make_pcb();
        let original_base = pcb.stack_base();
        let original_size = pcb.stack_size();

        // Size < 32 is invalid
        let result = pcb.fix_stack_region(0x2000_0000, 16);
        assert_eq!(result, Err(MpuError::SizeTooSmall));

        // Verify state is unchanged on error
        assert_eq!(pcb.stack_base(), original_base);
        assert_eq!(pcb.stack_size(), original_size);
    }

    #[test]
    fn fix_stack_region_rejects_size_not_power_of_two() {
        let mut pcb = make_pcb();
        let original_base = pcb.stack_base();
        let original_size = pcb.stack_size();

        // Size 100 is not a power of two
        let result = pcb.fix_stack_region(0x2000_0000, 100);
        assert_eq!(result, Err(MpuError::SizeNotPowerOfTwo));

        // Verify state is unchanged on error
        assert_eq!(pcb.stack_base(), original_base);
        assert_eq!(pcb.stack_size(), original_size);
    }

    #[test]
    fn fix_stack_region_rejects_misaligned_base() {
        let mut pcb = make_pcb();
        let original_base = pcb.stack_base();
        let original_size = pcb.stack_size();

        // Base 0x2000_0100 is not aligned to size 1024
        let result = pcb.fix_stack_region(0x2000_0100, 1024);
        assert_eq!(result, Err(MpuError::BaseNotAligned));

        // Verify state is unchanged on error
        assert_eq!(pcb.stack_base(), original_base);
        assert_eq!(pcb.stack_size(), original_size);
    }

    #[test]
    fn fix_stack_region_rejects_address_overflow() {
        let mut pcb = make_pcb();
        let original_base = pcb.stack_base();
        let original_size = pcb.stack_size();

        // base + size would overflow u32
        let result = pcb.fix_stack_region(0x8000_0000, 0x8000_0000);
        assert_eq!(result, Err(MpuError::AddressOverflow));

        // Verify state is unchanged on error
        assert_eq!(pcb.stack_base(), original_base);
        assert_eq!(pcb.stack_size(), original_size);
    }

    #[test]
    fn fix_stack_region_accepts_minimum_valid_size() {
        let mut pcb = make_pcb();

        // 32 is the minimum valid size, base must be 32-byte aligned
        pcb.fix_stack_region(0x2000_0020, 32).unwrap();

        assert_eq!(pcb.stack_base(), 0x2000_0020);
        assert_eq!(pcb.stack_size(), 32);
    }

    #[test]
    fn fix_stack_region_updates_stack_fields_independently() {
        let mut pcb = make_pcb();
        let original_mpu_base = pcb.mpu_region().base();
        let original_mpu_size = pcb.mpu_region().size();

        // Use valid aligned values that differ from mpu_region
        pcb.fix_stack_region(0x2004_0000, 4096).unwrap();

        // Verify stack fields are updated to new values
        assert_eq!(pcb.stack_base(), 0x2004_0000);
        assert_eq!(pcb.stack_size(), 4096);

        // Verify mpu_region is unchanged (stack and data are independent)
        assert_eq!(pcb.mpu_region().base(), original_mpu_base);
        assert_eq!(pcb.mpu_region().size(), original_mpu_size);
    }

    // ------------------------------------------------------------------
    // fix_mpu_data_region
    // ------------------------------------------------------------------

    #[test]
    fn fix_mpu_data_region_updates_base() {
        let mut pcb = make_pcb();
        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);

        pcb.fix_mpu_data_region(0x2001_0000);
        assert_eq!(pcb.mpu_region().base(), 0x2001_0000);
    }

    #[test]
    fn fix_mpu_data_region_preserves_size() {
        let mut pcb = make_pcb();
        let original_size = pcb.mpu_region().size();
        assert_eq!(original_size, 4096);

        pcb.fix_mpu_data_region(0x2002_0000);
        assert_eq!(pcb.mpu_region().size(), original_size);
    }

    #[test]
    fn fix_mpu_data_region_preserves_permissions() {
        let mut pcb = make_pcb();
        let original_perms = pcb.mpu_region().permissions();
        assert_eq!(original_perms, 0x0306_0000);

        pcb.fix_mpu_data_region(0x2003_0000);
        assert_eq!(pcb.mpu_region().permissions(), original_perms);
    }

    #[test]
    fn fix_mpu_data_region_does_not_affect_stack_fields() {
        let mut pcb = make_pcb();
        let original_stack_base = pcb.stack_base();
        let original_stack_size = pcb.stack_size();

        pcb.fix_mpu_data_region(0x2004_0000);

        assert_eq!(pcb.stack_base(), original_stack_base);
        assert_eq!(pcb.stack_size(), original_stack_size);
    }

    #[test]
    fn fix_mpu_data_region_post_condition_base_matches_stack_base() {
        let mut pcb = make_pcb();
        let new_base = 0x2005_0000;

        // Simulate boot-time relocation: fix stack, then fix MPU data region.
        pcb.fix_stack_region(new_base, pcb.stack_size()).unwrap();
        pcb.fix_mpu_data_region(new_base);

        assert_eq!(pcb.mpu_region().base(), pcb.stack_base());
    }

    #[test]
    fn fix_mpu_data_region_sentinel_with_nonzero_base_overwritten() {
        // Sentinel PCB: size==0 marks it as sentinel regardless of base.
        // An accidental non-zero base (0xDEAD_BEEF) must still be overwritten.
        let mut pcb = PartitionControlBlock::new(
            1,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0xDEAD_BEEF, 0, 0),
        );

        // Pre-condition: sentinel with non-zero base.
        assert_eq!(pcb.mpu_region().base(), 0xDEAD_BEEF);
        assert_eq!(pcb.mpu_region().size(), 0);
        assert_eq!(pcb.mpu_region().permissions(), 0);

        pcb.fix_mpu_data_region(0x2007_0000);

        // Base overwritten to the new value.
        assert_eq!(pcb.mpu_region().base(), 0x2007_0000);
        // Size and permissions remain zero (still sentinel-sized).
        assert_eq!(pcb.mpu_region().size(), 0);
        assert_eq!(pcb.mpu_region().permissions(), 0);
    }

    #[test]
    fn fix_mpu_data_region_zero_base() {
        // Sentinel PCB with all-zero mpu_region: base=0, size=0, permissions=0.
        // Calling fix_mpu_data_region(0) should keep base at 0 and preserve
        // zero size/permissions — no special-casing for the zero→zero case.
        let mut pcb = PartitionControlBlock::new(
            1,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0, 0, 0),
        );

        assert_eq!(pcb.mpu_region().base(), 0);
        assert_eq!(pcb.mpu_region().size(), 0);
        assert_eq!(pcb.mpu_region().permissions(), 0);

        pcb.fix_mpu_data_region(0);

        assert_eq!(pcb.mpu_region().base(), 0);
        assert_eq!(pcb.mpu_region().size(), 0);
        assert_eq!(pcb.mpu_region().permissions(), 0);
    }

    #[test]
    fn fix_mpu_data_region_high_sram_base() {
        // PCB with a typical SRAM data region; fix base to a high address
        // (0xE000_0000, system control space boundary) and verify only
        // the base changes while size and permissions are preserved.
        let mut pcb = PartitionControlBlock::new(
            1,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        );

        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 1024);
        assert_eq!(pcb.mpu_region().permissions(), 0x0306_0000);

        pcb.fix_mpu_data_region(0xE000_0000);

        assert_eq!(pcb.mpu_region().base(), 0xE000_0000);
        assert_eq!(pcb.mpu_region().size(), 1024);
        assert_eq!(pcb.mpu_region().permissions(), 0x0306_0000);
    }

    #[test]
    fn fix_mpu_data_region_preserves_peripheral_regions() {
        // PCB with a sentinel mpu_region (base=0, size=0) and two
        // peripheral regions.  fix_mpu_data_region must update only
        // the data region base without disturbing peripheral_regions.
        let mut pcb = PartitionControlBlock::new(
            1,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0, 0, 0), // sentinel
        )
        .with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 0x1000, 0x03),
            MpuRegion::new(0x4000_1000, 0x1000, 0x03),
        ]);

        // Pre-conditions: sentinel base and two peripheral regions.
        assert_eq!(pcb.mpu_region().base(), 0);
        assert_eq!(pcb.peripheral_regions().len(), 2);

        pcb.fix_mpu_data_region(0x2007_0000);

        // mpu_region base updated.
        assert_eq!(pcb.mpu_region().base(), 0x2007_0000);

        // peripheral_regions unchanged.
        assert_eq!(pcb.peripheral_regions().len(), 2);
        assert_eq!(pcb.peripheral_regions()[0].base(), 0x4000_0000);
        assert_eq!(pcb.peripheral_regions()[0].size(), 0x1000);
        assert_eq!(pcb.peripheral_regions()[0].permissions(), 0x03);
        assert_eq!(pcb.peripheral_regions()[1].base(), 0x4000_1000);
        assert_eq!(pcb.peripheral_regions()[1].size(), 0x1000);
        assert_eq!(pcb.peripheral_regions()[1].permissions(), 0x03);
    }

    #[test]
    fn fix_mpu_data_region_user_configured_preserves_all_fields() {
        // A user-configured PCB (size > 0) that is never passed through
        // fix_mpu_data_region — matching the boot.rs guard that skips
        // non-sentinel partitions — retains its original MPU region.
        let pcb = PartitionControlBlock::new(
            3,
            0x0800_0000,
            0x2000_4000,
            0x2000_4800, // 2 KiB stack
            MpuRegion::new(0x2000_4000, 8192, 0x0306_0000),
        );

        // Never call fix_mpu_data_region — verify all fields unchanged.
        assert_eq!(pcb.mpu_region().base(), 0x2000_4000);
        assert_eq!(pcb.mpu_region().size(), 8192);
        assert_eq!(pcb.mpu_region().permissions(), 0x0306_0000);
    }

    #[test]
    fn fix_mpu_data_region_on_non_sentinel_only_changes_base() {
        // If fix_mpu_data_region IS called on a non-sentinel PCB (size > 0),
        // only the base address changes; size and permissions are preserved.
        let original_size = 8192u32;
        let original_perms = 0x0306_0000u32;
        let mut pcb = PartitionControlBlock::new(
            3,
            0x0800_0000,
            0x2000_4000,
            0x2000_4800,
            MpuRegion::new(0x2000_4000, original_size, original_perms),
        );

        let new_base = 0x2001_0000;
        pcb.fix_mpu_data_region(new_base);

        assert_eq!(pcb.mpu_region().base(), new_base);
        assert_eq!(pcb.mpu_region().size(), original_size);
        assert_eq!(pcb.mpu_region().permissions(), original_perms);
    }

    // ------------------------------------------------------------------
    // promote_sentinel_mpu
    // ------------------------------------------------------------------

    fn make_sentinel_pcb() -> PartitionControlBlock {
        PartitionControlBlock::new(
            2,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0, 0, 0), // sentinel: size == 0
        )
    }

    #[test]
    fn promote_sentinel_mpu_succeeds_on_sentinel() {
        let mut pcb = make_sentinel_pcb();
        assert_eq!(pcb.mpu_region().size(), 0);

        let result = pcb.promote_sentinel_mpu(0x2000_0000, 4096, SENTINEL_DATA_PERMISSIONS);
        assert_eq!(result, Ok(()));

        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 4096);
        assert_eq!(pcb.mpu_region().permissions(), SENTINEL_DATA_PERMISSIONS);
    }

    #[test]
    fn promote_sentinel_mpu_fails_on_non_sentinel() {
        let mut pcb = make_pcb(); // mpu_region.size() == 4096
        assert_ne!(pcb.mpu_region().size(), 0);

        let result = pcb.promote_sentinel_mpu(0x2000_0000, 2048, SENTINEL_DATA_PERMISSIONS);
        assert_eq!(result, Err(MpuError::AlreadyInitialized));

        // Verify mpu_region is unchanged after error
        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 4096);
        assert_eq!(pcb.mpu_region().permissions(), 0x0306_0000);
    }

    #[test]
    fn promote_sentinel_mpu_does_not_modify_stack_fields() {
        let mut pcb = make_sentinel_pcb();
        let original_stack_base = pcb.stack_base();
        let original_stack_size = pcb.stack_size();

        pcb.promote_sentinel_mpu(0x2001_0000, 8192, 0xABCD_0000)
            .unwrap();

        assert_eq!(pcb.stack_base(), original_stack_base);
        assert_eq!(pcb.stack_size(), original_stack_size);
    }

    #[test]
    fn promote_sentinel_mpu_rejects_invalid_region() {
        let mut pcb = make_sentinel_pcb();
        // size 100 is not a power of two
        let result = pcb.promote_sentinel_mpu(0x2000_0000, 100, SENTINEL_DATA_PERMISSIONS);
        assert_eq!(result, Err(MpuError::SizeNotPowerOfTwo));
        // region should remain a sentinel after validation failure
        assert_eq!(pcb.mpu_region().size(), 0);
    }

    #[test]
    fn sentinel_data_permissions_uses_ap_full_access() {
        let ap_field = (SENTINEL_DATA_PERMISSIONS >> RASR_AP_SHIFT) & 0x7;
        assert_eq!(ap_field, AP_FULL_ACCESS);
    }

    #[test]
    fn sentinel_data_permissions_matches_test_convention() {
        assert_eq!(SENTINEL_DATA_PERMISSIONS, 0x0306_0000);
    }

    // ------------------------------------------------------------------
    // accessible_static_regions
    // ------------------------------------------------------------------

    #[test]
    fn accessible_static_regions_returns_data_then_stack() {
        let pcb = make_pcb();
        let regions = pcb.accessible_static_regions();
        assert_eq!(regions.len(), 2);
        // Data region first (from mpu_region)
        assert_eq!(regions[0], (0x2000_0000, 4096));
        // Stack region second
        assert_eq!(regions[1], (0x2000_0000, 1024));
    }

    #[test]
    fn accessible_static_regions_different_data_and_stack() {
        // Create PCB with distinct data and stack regions
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_1000,                                    // stack_base
            0x2000_1200, // stack_pointer (stack_size = 0x200 = 512)
            MpuRegion::new(0x2000_0000, 2048, 0x0306_0000), // data region
        );
        let regions = pcb.accessible_static_regions();
        assert_eq!(regions.len(), 2);
        // Data region first
        assert_eq!(regions[0], (0x2000_0000, 2048));
        // Stack region second
        assert_eq!(regions[1], (0x2000_1000, 512));
    }

    #[test]
    fn accessible_static_regions_zero_data_size() {
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0x2000_0000, 0, 0), // zero-size data region
        );
        let regions = pcb.accessible_static_regions();
        // Only stack region returned when data size is zero
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0], (0x2000_0000, 1024));
    }

    #[test]
    fn accessible_static_regions_zero_stack_size() {
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000, // stack_pointer == stack_base => stack_size = 0
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        );
        let regions = pcb.accessible_static_regions();
        // Only data region returned when stack size is zero
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0], (0x2000_0000, 4096));
    }

    #[test]
    fn accessible_static_regions_both_zero_size() {
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000,                       // stack_size = 0
            MpuRegion::new(0x2000_0000, 0, 0), // data_size = 0
        );
        let regions = pcb.accessible_static_regions();
        // Empty vector when both sizes are zero
        assert!(regions.is_empty());
    }

    #[test]
    fn accessible_static_regions_reflects_fix_stack_region() {
        // Create PCB with distinct data region (0x2000_0000, 8192) and stack region
        // Note: make_pcb creates a PCB where mpu_region.base == stack_base, so
        // we create a custom PCB with separate regions to properly test.
        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_2000,                                    // stack_base
            0x2000_2400, // stack_pointer (stack_size = 0x400 = 1024)
            MpuRegion::new(0x2000_0000, 8192, 0x0306_0000), // data region at different location
        );

        // Verify initial regions: data region and stack region are distinct
        let regions_before = pcb.accessible_static_regions();
        assert_eq!(regions_before.len(), 2);
        assert_eq!(regions_before[0], (0x2000_0000, 8192)); // data region
        assert_eq!(regions_before[1], (0x2000_2000, 1024)); // initial stack region

        // Call fix_stack_region with new values
        // New stack: base=0x2001_0000, size=2048
        pcb.fix_stack_region(0x2001_0000, 2048).unwrap();

        // Verify accessible_static_regions returns updated stack region
        // while preserving the original data region
        let regions_after = pcb.accessible_static_regions();
        assert_eq!(regions_after.len(), 2);
        assert_eq!(regions_after[0], (0x2000_0000, 8192)); // data region preserved
        assert_eq!(regions_after[1], (0x2001_0000, 2048)); // stack region updated
    }

    #[test]
    fn accessible_static_regions_stack_matches_stack_region_after_fix() {
        let mut pcb = make_pcb();

        // Call fix_stack_region with new values
        pcb.fix_stack_region(0x2004_0000, 4096).unwrap();

        // Verify accessible_static_regions stack entry matches stack_region()
        let (stack_base, stack_size) = pcb.stack_region();
        assert_eq!(stack_base, 0x2004_0000);
        assert_eq!(stack_size, 4096);

        let regions = pcb.accessible_static_regions();
        // Stack region is at index 1 (after data region)
        assert_eq!(regions[1], (stack_base, stack_size));
    }

    #[test]
    fn accessible_static_regions_format_matches_dynamic_strategy() {
        // Verify the (base, size) format matches what DynamicStrategy uses
        let pcb = make_pcb();
        let regions = pcb.accessible_static_regions();
        for (base, size) in regions.iter() {
            // Each region should have non-zero size (function guarantees this)
            assert!(*size > 0);
            // Verify we can compute end address without overflow
            assert!(
                base.checked_add(*size).is_some(),
                "Region end address must not overflow u32"
            );
        }
    }

    #[test]
    fn peripheral_regions_and_accessible_static_regions() {
        assert!(make_pcb().peripheral_regions().is_empty());
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = make_pcb().with_peripheral_regions(&[r(0x4000_0000, 4096)]);
        assert_eq!(pcb.peripheral_regions().len(), 1);
        let pcb = make_pcb().with_peripheral_regions(&[r(0x4000_0000, 0), r(0x4000_1000, 256)]);
        assert_eq!(pcb.peripheral_regions().len(), 1); // zero-size ignored
        let pcb = make_pcb().with_peripheral_regions(&[
            r(0x4000_0000, 4096),
            r(0x4000_1000, 256),
            r(0x4000_2000, 512),
        ]);
        assert_eq!(pcb.peripheral_regions().len(), 3); // limits to three
        let regions = pcb.accessible_static_regions();
        assert_eq!(regions.len(), 5); // data + stack + 3 peripherals
        assert_eq!(regions[2], (0x4000_0000, 4096));
        assert_eq!(regions[3], (0x4000_1000, 256));
        assert_eq!(regions[4], (0x4000_2000, 512));
    }

    // ------------------------------------------------------------------
    // exclusive_static_regions
    // ------------------------------------------------------------------

    #[test]
    fn exclusive_static_regions_returns_stack_and_peripheral_not_data() {
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_2000,                                    // stack_base
            0x2000_2400,                                    // stack_pointer => stack_size = 1024
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000), // data region
        )
        .with_peripheral_regions(&[r(0x4000_0000, 4096)]);
        let regions = pcb.exclusive_static_regions();
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0], (0x2000_2000, 1024)); // stack
        assert_eq!(regions[1], (0x4000_0000, 4096)); // peripheral
    }

    #[test]
    fn exclusive_static_regions_zero_stack_returns_peripherals_only() {
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000, // stack_size = 0
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
        .with_peripheral_regions(&[r(0x4000_0000, 256)]);
        let regions = pcb.exclusive_static_regions();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0], (0x4000_0000, 256));
    }

    #[test]
    fn exclusive_static_regions_no_peripherals_returns_stack_only() {
        let pcb = make_pcb(); // has stack, no peripherals
        let regions = pcb.exclusive_static_regions();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0], (0x2000_0000, 1024));
    }

    #[test]
    fn exclusive_static_regions_filters_zero_size_peripherals() {
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_2000,
            0x2000_2400, // stack_size = 1024
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
        .with_peripheral_regions(&[r(0x4000_0000, 0), r(0x4000_1000, 256)]);
        let regions = pcb.exclusive_static_regions();
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0], (0x2000_2000, 1024)); // stack
        assert_eq!(regions[1], (0x4000_1000, 256)); // non-zero peripheral only
    }

    #[test]
    fn exclusive_static_regions_zero_stack_zero_peripherals_returns_empty() {
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000, // stack_size = 0
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
        .with_peripheral_regions(&[r(0x4000_0000, 0)]);
        let regions = pcb.exclusive_static_regions();
        assert!(regions.is_empty());
    }

    /// Capacity of `exclusive_static_regions` is 4: 1 stack + up to 3 peripherals.
    /// Verify all 4 entries are returned with exactly 3 peripheral regions.
    #[test]
    fn exclusive_static_regions_max_peripherals_fits_capacity() {
        let r = |b, s| MpuRegion::new(b, s, 0x03);
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_2000,
            0x2000_2400, // stack_size = 1024
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
        .with_peripheral_regions(&[
            r(0x4000_0000, 4096),
            r(0x4000_1000, 256),
            r(0x4000_2000, 512),
        ]);
        let regions = pcb.exclusive_static_regions();
        // 1 stack + 3 peripherals = 4 entries, exactly filling capacity 4
        assert_eq!(regions.len(), 4);
        assert_eq!(regions.capacity(), 4);
        assert_eq!(regions[0], (0x2000_2000, 1024)); // stack
        assert_eq!(regions[1], (0x4000_0000, 4096)); // peripheral 1
        assert_eq!(regions[2], (0x4000_1000, 256)); // peripheral 2
        assert_eq!(regions[3], (0x4000_2000, 512)); // peripheral 3
    }

    /// Documents that `peripheral_regions` capacity (3) bounds the maximum
    /// entries in `exclusive_static_regions` to 4 (1 stack + 3 peripherals),
    /// exactly filling the output Vec capacity of 4.
    #[test]
    fn exclusive_static_regions_capacity_bounded_by_peripheral_vec() {
        let pcb = make_pcb();
        // peripheral_regions Vec<MpuRegion, 3>: at most 3 peripherals
        assert!(pcb.peripheral_regions().len() <= 3);
        // exclusive_static_regions Vec<(u32, u32), 4>: at most 1 + 3 = 4 entries
        let regions = pcb.exclusive_static_regions();
        assert!(regions.len() <= 4);
    }

    #[test]
    fn exclusive_static_regions_reflects_fix_stack_region() {
        let mut pcb = make_pcb();
        let before = pcb.exclusive_static_regions();
        assert_eq!(before[0], (0x2000_0000, 1024));

        pcb.fix_stack_region(0x2001_0000, 2048).unwrap();
        let after = pcb.exclusive_static_regions();
        assert_eq!(after.len(), 1);
        assert_eq!(after[0], (0x2001_0000, 2048));
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
    fn pcb_is_clone() {
        let pcb = make_pcb();
        let cloned = pcb.clone();
        // both are usable — proves Clone
        assert_eq!(pcb.id(), cloned.id());
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
            ConfigError::PartitionIdMismatch {
                index: 0,
                expected_id: 0,
                actual_id: 5,
            },
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
    fn config_error_display_partition_id_mismatch() {
        let msg = format!(
            "{}",
            ConfigError::PartitionIdMismatch {
                index: 2,
                expected_id: 2,
                actual_id: 7,
            }
        );
        assert!(msg.contains("config[2]"), "should contain index");
        assert!(msg.contains("id 7"), "should contain actual_id");
        assert!(msg.contains("expected 2"), "should contain expected_id");
        assert!(msg.contains("ids must match array index"));
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
        assert!(format!(
            "{:?}",
            ConfigError::PartitionIdMismatch {
                index: 0,
                expected_id: 0,
                actual_id: 1,
            }
        )
        .contains("PartitionIdMismatch"));
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
            MpuError::AlreadyInitialized,
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

    // ------------------------------------------------------------------
    // PartitionConfig::validate
    // ------------------------------------------------------------------

    fn valid_config() -> PartitionConfig {
        PartitionConfig {
            id: 0,
            entry_point: 0x0800_0000,
            stack_base: 0x2000_0000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
            peripheral_regions: Vec::new(),
        }
    }

    #[test]
    fn validate_accepts_valid_config() {
        assert_eq!(valid_config().validate(), Ok(()));
    }

    #[test]
    fn validate_accepts_minimum_stack_size() {
        let mut cfg = valid_config();
        cfg.stack_size = 32;
        cfg.stack_base = 0x2000_0000; // aligned to 32
        assert_eq!(cfg.validate(), Ok(()));
    }

    #[test]
    fn validate_rejects_stack_size_not_power_of_two() {
        let mut cfg = valid_config();
        cfg.stack_size = 100;
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackSizeInvalid { partition_id: 0 })
        );
    }

    #[test]
    fn validate_rejects_stack_size_too_small() {
        let mut cfg = valid_config();
        cfg.stack_size = 16;
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackSizeInvalid { partition_id: 0 })
        );
    }

    #[test]
    fn validate_rejects_stack_size_zero() {
        let mut cfg = valid_config();
        cfg.stack_size = 0;
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackSizeInvalid { partition_id: 0 })
        );
    }

    #[test]
    fn validate_rejects_misaligned_stack_base() {
        let mut cfg = valid_config();
        cfg.stack_base = 0x2000_0100; // not aligned to 1024
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackBaseNotAligned { partition_id: 0 })
        );
    }

    #[test]
    fn validate_rejects_stack_overflow() {
        let mut cfg = valid_config();
        cfg.stack_size = 0x8000_0000;
        cfg.stack_base = 0x8000_0000; // aligned, but base + size = 2^32 overflow
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackOverflow { partition_id: 0 })
        );
    }

    #[test]
    fn validate_rejects_invalid_mpu_region_size() {
        let mut cfg = valid_config();
        cfg.mpu_region = MpuRegion::new(0x2000_0000, 100, 0); // not power of 2
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::MpuRegionInvalid {
                partition_id: 0,
                detail: MpuError::SizeNotPowerOfTwo,
            })
        );
    }

    #[test]
    fn validate_rejects_invalid_mpu_region_alignment() {
        let mut cfg = valid_config();
        cfg.mpu_region = MpuRegion::new(0x2000_0100, 4096, 0); // misaligned base
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::MpuRegionInvalid {
                partition_id: 0,
                detail: MpuError::BaseNotAligned,
            })
        );
    }

    #[test]
    fn validate_preserves_partition_id_in_error() {
        let mut cfg = valid_config();
        cfg.id = 7;
        cfg.stack_size = 16;
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::StackSizeInvalid { partition_id: 7 })
        );
    }

    // ------------------------------------------------------------------
    // PartitionConfig peripheral_regions validation
    // ------------------------------------------------------------------

    #[test]
    fn validate_peripheral_regions_valid_and_invalid() {
        // Valid: empty peripheral_regions
        assert_eq!(valid_config().validate(), Ok(()));
        // Valid: one peripheral region
        let mut cfg = valid_config();
        let _ = cfg
            .peripheral_regions
            .push(MpuRegion::new(0x4000_0000, 4096, 0x03));
        assert_eq!(cfg.validate(), Ok(()));
        // Invalid: size too small
        let mut cfg = valid_config();
        let _ = cfg
            .peripheral_regions
            .push(MpuRegion::new(0x4000_0000, 16, 0x03));
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::PeripheralRegionInvalid {
                partition_id: 0,
                region_index: 0,
                detail: MpuError::SizeTooSmall,
            })
        );
        // Invalid: second region has non-power-of-two size
        let mut cfg = valid_config();
        let _ = cfg
            .peripheral_regions
            .push(MpuRegion::new(0x4000_0000, 4096, 0x03));
        let _ = cfg
            .peripheral_regions
            .push(MpuRegion::new(0x4000_1000, 100, 0x03));
        assert_eq!(
            cfg.validate(),
            Err(ConfigError::PeripheralRegionInvalid {
                partition_id: 0,
                region_index: 1,
                detail: MpuError::SizeNotPowerOfTwo,
            })
        );
        // Display format check
        let e = ConfigError::PeripheralRegionInvalid {
            partition_id: 2,
            region_index: 1,
            detail: MpuError::BaseNotAligned,
        };
        let msg = format!("{e}");
        assert!(msg.contains("partition 2") && msg.contains("peripheral region 1"));
    }

    // ------------------------------------------------------------------
    // PartitionConfig::new
    // ------------------------------------------------------------------

    #[test]
    fn config_new_field_values_match() {
        let mpu = MpuRegion::new(0x2000_0000, 4096, 0x0306_0000);
        let cfg = PartitionConfig::new(2, 0x0800_1000, 0x2000_0000, 1024, mpu);
        assert_eq!(cfg.id, 2);
        assert_eq!(cfg.entry_point, 0x0800_1000);
        assert_eq!(cfg.stack_base, 0x2000_0000);
        assert_eq!(cfg.stack_size, 1024);
        assert_eq!(cfg.mpu_region.base(), 0x2000_0000);
        assert_eq!(cfg.mpu_region.size(), 4096);
        assert_eq!(cfg.mpu_region.permissions(), 0x0306_0000);
    }

    #[test]
    fn config_new_peripheral_regions_empty() {
        let mpu = MpuRegion::new(0x2000_0000, 4096, 0x0306_0000);
        let cfg = PartitionConfig::new(0, 0x0800_0000, 0x2000_0000, 1024, mpu);
        assert!(cfg.peripheral_regions.is_empty());
    }

    #[test]
    fn config_new_valid_inputs_pass_validate() {
        let mpu = MpuRegion::new(0x2000_0000, 4096, 0x0306_0000);
        let cfg = PartitionConfig::new(0, 0x0800_0000, 0x2000_0000, 1024, mpu);
        assert_eq!(cfg.validate(), Ok(()));
    }

    // ------------------------------------------------------------------
    // PartitionConfig::sentinel
    // ------------------------------------------------------------------

    #[test]
    fn sentinel_field_values_are_correct() {
        let cfg = PartitionConfig::sentinel(3, 1024);
        assert_eq!(cfg.id, 3);
        assert_eq!(cfg.entry_point, 0);
        assert_eq!(cfg.stack_base, 0);
        assert_eq!(cfg.stack_size, 1024);
        assert_eq!(cfg.mpu_region, MpuRegion::new(0, 0, 0));
        assert!(cfg.peripheral_regions.is_empty());
    }

    #[test]
    fn sentinel_mpu_region_is_zero() {
        let cfg = PartitionConfig::sentinel(0, 512);
        assert_eq!(cfg.mpu_region.base(), 0);
        assert_eq!(cfg.mpu_region.size(), 0);
        assert_eq!(cfg.mpu_region.permissions(), 0);
    }

    #[test]
    fn sentinel_different_ids_and_sizes() {
        for id in [0u8, 1, 7, 255] {
            for size in [32u32, 256, 4096] {
                let cfg = PartitionConfig::sentinel(id, size);
                assert_eq!(cfg.id, id);
                assert_eq!(cfg.stack_size, size);
                assert_eq!(cfg.entry_point, 0);
                assert_eq!(cfg.stack_base, 0);
                assert!(cfg.peripheral_regions.is_empty());
            }
        }
    }

    // ------------------------------------------------------------------
    // PartitionConfig::sentinel_array
    // ------------------------------------------------------------------

    #[test]
    fn sentinel_array_ids_and_stack_sizes() {
        let arr = PartitionConfig::sentinel_array::<4>(256);
        let word = core::mem::size_of::<u32>() as u32;
        for (i, cfg) in arr.iter().enumerate() {
            assert_eq!(cfg.id, i as u8);
            assert_eq!(cfg.stack_size, 256 * word);
            assert_eq!(cfg.entry_point, 0);
            assert_eq!(cfg.stack_base, 0);
            assert!(cfg.peripheral_regions.is_empty());
        }
    }

    #[test]
    fn sentinel_array_single_element() {
        let arr = PartitionConfig::sentinel_array::<1>(128);
        let word = core::mem::size_of::<u32>() as u32;
        assert_eq!(arr.len(), 1);
        assert_eq!(arr[0].id, 0);
        assert_eq!(arr[0].stack_size, 128 * word);
    }

    #[test]
    #[should_panic(expected = "N must be <= 256")]
    fn sentinel_array_rejects_n_over_256() {
        let _ = PartitionConfig::sentinel_array::<257>(64);
    }

    #[test]
    #[should_panic(expected = "stack size in bytes overflows u32")]
    fn sentinel_array_rejects_overflow() {
        let _ = PartitionConfig::sentinel_array::<1>(usize::MAX);
    }

    // ------------------------------------------------------------------
    // debug_buffer (partition-debug feature)
    // ------------------------------------------------------------------

    #[cfg(feature = "partition-debug")]
    mod debug_buffer_tests {
        use super::*;
        use crate::debug::DebugRingBuffer;

        static TEST_DEBUG_BUFFER: DebugRingBuffer<64> = DebugRingBuffer::new();

        #[test]
        fn debug_buffer_initially_none() {
            let pcb = make_pcb();
            assert!(pcb.debug_buffer().is_none());
        }

        #[test]
        fn set_and_get_debug_buffer() {
            let mut pcb = make_pcb();
            pcb.set_debug_buffer(&TEST_DEBUG_BUFFER);
            let buf = pcb.debug_buffer();
            assert!(buf.is_some());
            // Verify it's the same buffer by checking it's usable via trait methods
            let buf = buf.unwrap();
            assert!(buf.is_empty());
        }

        #[test]
        fn debug_buffer_returns_correct_reference() {
            static BUF: DebugRingBuffer<128> = DebugRingBuffer::new();
            let mut pcb = make_pcb();
            pcb.set_debug_buffer(&BUF);
            let retrieved = pcb.debug_buffer().unwrap();
            // Write directly to BUF and verify via trait object
            assert_eq!(BUF.write(b"test"), 4);
            assert_eq!(retrieved.available(), 4);
        }
    }

    // ------------------------------------------------------------------
    // ConfigError::NoSystemWindow (dynamic-mpu feature)
    // ------------------------------------------------------------------

    #[cfg(feature = "dynamic-mpu")]
    mod no_system_window_tests {
        use super::*;

        #[test]
        fn no_system_window_is_copy() {
            let e = ConfigError::NoSystemWindow;
            let e2 = e;
            assert_eq!(e, e2);
        }

        #[test]
        fn no_system_window_display_mentions_bottom_half() {
            let msg = format!("{}", ConfigError::NoSystemWindow);
            assert!(
                msg.contains("bottom-half"),
                "should mention bottom-half processing: {msg}"
            );
        }

        #[test]
        fn no_system_window_display_mentions_system_window() {
            let msg = format!("{}", ConfigError::NoSystemWindow);
            assert!(
                msg.contains("system window"),
                "should mention system window: {msg}"
            );
        }

        #[test]
        fn no_system_window_debug_contains_variant_name() {
            let debug_str = format!("{:?}", ConfigError::NoSystemWindow);
            assert!(
                debug_str.contains("NoSystemWindow"),
                "debug should contain variant name: {debug_str}"
            );
        }

        #[test]
        fn no_system_window_distinct_from_other_variants() {
            let no_sys = ConfigError::NoSystemWindow;
            assert_ne!(no_sys, ConfigError::ScheduleEmpty);
            assert_ne!(no_sys, ConfigError::PartitionTableFull);
            assert_ne!(no_sys, ConfigError::StackSizeInvalid { partition_id: 0 });
        }
    }

    // ------------------------------------------------------------------
    // ConfigError::SystemWindowTooInfrequent (dynamic-mpu feature)
    // ------------------------------------------------------------------

    #[cfg(feature = "dynamic-mpu")]
    mod system_window_too_infrequent_tests {
        use super::*;

        #[test]
        fn system_window_too_infrequent_is_copy() {
            let e = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 100,
                threshold_ticks: 50,
            };
            let e2 = e;
            assert_eq!(e, e2);
        }

        #[test]
        fn system_window_too_infrequent_display_shows_gap() {
            let msg = format!(
                "{}",
                ConfigError::SystemWindowTooInfrequent {
                    max_gap_ticks: 200,
                    threshold_ticks: 100,
                }
            );
            assert!(msg.contains("200"), "should contain max_gap_ticks: {msg}");
            assert!(msg.contains("100"), "should contain threshold_ticks: {msg}");
        }

        #[test]
        fn system_window_too_infrequent_display_mentions_threshold() {
            let msg = format!(
                "{}",
                ConfigError::SystemWindowTooInfrequent {
                    max_gap_ticks: 500,
                    threshold_ticks: 250,
                }
            );
            assert!(msg.contains("threshold"), "should mention threshold: {msg}");
        }

        #[test]
        fn system_window_too_infrequent_debug_contains_variant_name() {
            let debug_str = format!(
                "{:?}",
                ConfigError::SystemWindowTooInfrequent {
                    max_gap_ticks: 10,
                    threshold_ticks: 5,
                }
            );
            assert!(
                debug_str.contains("SystemWindowTooInfrequent"),
                "debug should contain variant name: {debug_str}"
            );
        }

        #[test]
        fn system_window_too_infrequent_distinct_from_other_variants() {
            let too_infrequent = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 100,
                threshold_ticks: 50,
            };
            assert_ne!(too_infrequent, ConfigError::ScheduleEmpty);
            assert_ne!(too_infrequent, ConfigError::NoSystemWindow);
            assert_ne!(too_infrequent, ConfigError::PartitionTableFull);
        }

        #[test]
        fn system_window_too_infrequent_equality_checks_both_fields() {
            let e1 = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 100,
                threshold_ticks: 50,
            };
            let e2 = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 100,
                threshold_ticks: 50,
            };
            let e3 = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 200,
                threshold_ticks: 50,
            };
            let e4 = ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 100,
                threshold_ticks: 75,
            };
            assert_eq!(e1, e2);
            assert_ne!(e1, e3);
            assert_ne!(e1, e4);
        }
    }

    // ------------------------------------------------------------------
    // cached MPU region fields
    // ------------------------------------------------------------------

    #[test]
    fn new_pcb_has_zero_initialized_cached_regions() {
        let pcb = make_pcb();
        assert_eq!(*pcb.cached_base_regions(), [(0, 0); 4]);
        assert_eq!(*pcb.cached_periph_regions(), [(0, 0); 3]);
    }

    #[test]
    fn set_get_cached_base_regions_roundtrip() {
        let mut pcb = make_pcb();
        let regions = [
            (0x2000_0010, 0x0000_1305),
            (0x2000_1010, 0x0000_1307),
            (0x2000_2010, 0x0000_1309),
            (0x2000_3010, 0x0000_130B),
        ];
        pcb.set_cached_base_regions(regions).unwrap();
        assert_eq!(*pcb.cached_base_regions(), regions);
    }

    #[test]
    fn set_get_cached_periph_regions_roundtrip() {
        let mut pcb = make_pcb();
        let regions = [
            (0x4000_0014, 0x0000_1315),
            (0x4000_1014, 0x0000_1317),
            (0x4000_2014, 0x0000_1319),
        ];
        pcb.set_cached_periph_regions(regions).unwrap();
        assert_eq!(*pcb.cached_periph_regions(), regions);
    }

    // ── cache_sealed guard tests ────────────────────────────────────

    #[test]
    fn cache_sealed_default_false_then_seal() {
        let mut pcb = make_pcb();
        assert!(!pcb.cache_sealed());
        pcb.seal_cache();
        assert!(pcb.cache_sealed());
    }

    #[test]
    fn seal_cache_is_idempotent() {
        let mut pcb = make_pcb();
        pcb.seal_cache();
        assert!(pcb.cache_sealed());
        // Second call must not panic.
        pcb.seal_cache();
        assert!(pcb.cache_sealed());
    }

    #[test]
    fn mutators_succeed_before_seal() {
        let mut pcb = make_pcb();
        // All mutators should succeed before the cache is sealed.
        assert!(pcb.fix_stack_region(0x2000_0000, 2048).is_ok());
        pcb.fix_mpu_data_region(0x2000_0000);
        pcb.set_cached_base_regions([(1, 2); 4]).unwrap();
        pcb.set_cached_periph_regions([(3, 4); 3]).unwrap();
        // promote_sentinel_mpu requires size==0; use a sentinel PCB.
        let mut sentinel = make_sentinel_pcb();
        assert!(sentinel
            .promote_sentinel_mpu(0x2000_0000, 1024, SENTINEL_DATA_PERMISSIONS)
            .is_ok());
    }

    #[test]
    #[should_panic(expected = "fix_stack_region called after MPU cache sealed")]
    fn fix_stack_region_panics_after_seal() {
        let mut pcb = make_pcb();
        pcb.seal_cache();
        let _ = pcb.fix_stack_region(0x2000_0000, 2048);
    }

    #[test]
    #[should_panic(expected = "fix_mpu_data_region called after MPU cache sealed")]
    fn fix_mpu_data_region_panics_after_seal() {
        let mut pcb = make_pcb();
        pcb.seal_cache();
        pcb.fix_mpu_data_region(0x2000_0000);
    }

    #[test]
    fn set_cached_base_regions_err_after_seal() {
        let mut pcb = make_pcb();
        pcb.seal_cache();
        assert_eq!(
            pcb.set_cached_base_regions([(1, 2); 4]),
            Err("set_cached_base_regions called after MPU cache sealed")
        );
    }

    #[test]
    fn set_cached_periph_regions_err_after_seal() {
        let mut pcb = make_pcb();
        pcb.seal_cache();
        assert_eq!(
            pcb.set_cached_periph_regions([(3, 4); 3]),
            Err("set_cached_periph_regions called after MPU cache sealed")
        );
    }

    #[test]
    #[should_panic(expected = "promote_sentinel_mpu called after MPU cache sealed")]
    fn promote_sentinel_mpu_panics_after_seal() {
        let mut sentinel = make_sentinel_pcb();
        sentinel.seal_cache();
        let _ = sentinel.promote_sentinel_mpu(0x2000_0000, 1024, SENTINEL_DATA_PERMISSIONS);
    }

    // ── is_mappable tests ──

    #[test]
    fn is_mappable_valid_region() {
        let r = MpuRegion::new(0x2000_0000, 4096, 0);
        assert!(r.is_mappable());
    }

    #[test]
    fn is_mappable_size_too_small() {
        let r = MpuRegion::new(0, 16, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn is_mappable_non_power_of_two() {
        let r = MpuRegion::new(0, 48, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn is_mappable_misaligned_base() {
        // base 0x100 is not aligned to size 4096 (0x1000)
        let r = MpuRegion::new(0x100, 4096, 0);
        assert!(!r.is_mappable());
    }
}
