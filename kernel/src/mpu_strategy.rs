//! MPU strategy abstraction for dynamic partition windowing.
//!
//! This module defines the [`MpuStrategy`] trait, allowing the kernel to
//! swap between static (compile-time) and dynamic (runtime) MPU region
//! management.  [`StaticStrategy`] delegates directly to the existing
//! [`crate::mpu`] helpers with no behaviour change.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

pub use crate::mpu::MpuError;
use crate::partition::MAX_PERIPHERAL_REGIONS;
use crate::PartitionId;

#[cfg(not(test))]
use crate::mpu;

/// Execute `f` inside a critical section.
///
/// On the target this disables interrupts via `cortex_m::interrupt::free`.
/// In host-mode unit tests the closure runs directly with a synthetic
/// `CriticalSection` token (single-threaded test runner, no real interrupts).
#[cfg(not(test))]
fn with_cs<F, R>(f: F) -> R
where
    F: FnOnce(&cortex_m::interrupt::CriticalSection) -> R,
{
    cortex_m::interrupt::free(f)
}

#[cfg(test)]
fn with_cs<F, R>(f: F) -> R
where
    F: FnOnce(&cortex_m::interrupt::CriticalSection) -> R,
{
    // SAFETY: Tests run single-threaded on the host — there are no real
    // interrupts to mask, so a synthetic CriticalSection token is sound.
    f(unsafe { &cortex_m::interrupt::CriticalSection::new() })
}

/// Trait abstracting how MPU regions are managed for a partition.
///
/// Implementers decide how `configure_partition`, `add_window`, and
/// `remove_window` map onto hardware MPU region registers.
pub trait MpuStrategy {
    /// Configure MPU regions for a partition from pre-computed (RBAR, RASR)
    /// pairs.
    ///
    /// `partition_id` identifies the partition; `regions` supplies
    /// (RBAR, RASR) pairs to programme into the MPU.  The static strategy
    /// expects exactly 4 pairs; implementations may return `Err` if the
    /// slice length is incorrect.
    ///
    /// `peripheral_reserved` specifies how many leading dynamic slots
    /// (0 or 2) are reserved for peripheral MMIO regions.  When 2, the
    /// partition's private-RAM region is placed after the reserved slots,
    /// and `partition_region_values` emits disabled entries for the reserved
    /// slots so PendSV can overwrite them with per-partition peripherals.
    fn configure_partition(
        &self,
        partition_id: PartitionId,
        regions: &[(u32, u32)],
        peripheral_reserved: usize,
    ) -> Result<(), MpuError>;

    /// Dynamically add a temporary memory window.
    ///
    /// `owner` identifies the partition that owns the window.
    /// Returns the MPU region ID on success, or an [`MpuError`] describing
    /// the failure (e.g. `SlotExhausted` when no free region is available).
    fn add_window(
        &self,
        base: u32,
        size: u32,
        permissions: u32,
        owner: PartitionId,
    ) -> Result<u8, MpuError>;

    /// Remove a previously added window by its region ID.
    fn remove_window(&self, region_id: u8);
}

/// Descriptor for a dynamically-assigned MPU window: base, size,
/// permissions, and owning partition ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WindowDescriptor {
    pub base: u32,
    pub size: u32,
    pub permissions: u32,
    pub owner: PartitionId,
    pub rbar: u32,
}
fn slot_rbar(base: u32, slot_index: usize) -> u32 {
    let r = DYNAMIC_REGION_BASE as u32 + slot_index as u32;
    crate::mpu::build_rbar(base, r).unwrap_or(r | (1 << 4))
}

/// Number of dynamic region slots (R4 through R7).
/// See `notes/architecture/mpu-slot-allocation.md` for the full slot allocation rationale.
const DYNAMIC_SLOT_COUNT: usize = 4;

/// Number of dynamic slots permanently occupied by partition RAM.
const RAM_SLOT_COUNT: usize = 1;

/// Number of leading dynamic slots reserved for peripheral MMIO regions
/// when a partition has peripherals.  Partitions without peripherals
/// reserve 0 slots.
#[deprecated(note = "use per-partition peripheral counts or MAX_PERIPHERAL_REGIONS instead")]
pub const PERIPHERAL_RESERVED_SLOTS: usize = 2;

// Compile-time check: the canonical MAX_PERIPHERAL_REGIONS (in partition.rs)
// must equal the slot-derived value so partition limits stay in sync with MPU capacity.
const _: () = assert!(
    crate::partition::MAX_PERIPHERAL_REGIONS == DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT,
    "MAX_PERIPHERAL_REGIONS out of sync with dynamic MPU slot layout"
);

/// Upper bound (exclusive) on peripherals `wire_boot_peripherals` may wire.
/// Accounts for the RAM slot and preserves at least one free dynamic slot
/// for runtime `add_window`.
pub const BOOT_WIRE_LIMIT: usize = DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT;

/// First hardware MPU region number used by the dynamic strategy.
const DYNAMIC_REGION_BASE: u8 = 4;

/// Construct a disabled (RBAR, RASR) pair for the given MPU region number.
///
/// RBAR selects the region with the VALID bit set; RASR = 0 disables it.
const fn disabled_pair(region: u8) -> (u32, u32) {
    (region as u32 | (1 << 4), 0)
}

/// Compute the RASR value for a peripheral MMIO region of the given `size`.
///
/// The result uses Device-attribute settings (S=1, C=0, B=1), full
/// read/write access, and execute-never.  `size` must be a power-of-two
/// >= 32 bytes.
fn compute_peripheral_rasr(size: u32) -> u32 {
    let tz = size.trailing_zeros();
    debug_assert!(tz >= 5, "MPU region size must be >= 32 bytes (size={size})");
    crate::mpu::build_rasr(
        tz.saturating_sub(1),
        crate::mpu::AP_FULL_ACCESS,
        true,
        (true, false, true),
    )
}

/// Verify that `partition_region_values` output contains no stale peripheral
/// RASR bits after a context switch.
///
/// Rules checked for each slot index `0..DYNAMIC_SLOT_COUNT`:
/// - **Within peripheral-reserved range** (`idx < reserved`) with a cached
///   peripheral (non-zero cache RASR): output RASR must be non-zero.
/// - **Beyond peripheral-reserved range** (`idx > reserved`, i.e. not the
///   RAM slot at `idx == reserved`): output RASR must be zero **or** the
///   slot must hold a shared window owned by this partition.
#[cfg(debug_assertions)]
fn debug_assert_no_stale_regions(
    out: &[(u32, u32); DYNAMIC_SLOT_COUNT],
    partition_id: PartitionId,
    reserved: usize,
    cache: &[(u32, u32); MAX_PERIPHERAL_REGIONS],
    slots: &[Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT],
) {
    for idx in 0..DYNAMIC_SLOT_COUNT {
        let rasr = out[idx].1;
        if idx < reserved {
            // Peripheral-reserved slot with a cached peripheral: RASR must be active.
            if idx < cache.len() && cache[idx].1 != 0 {
                assert!(
                    rasr != 0,
                    "stale MPU: slot {idx} has cached peripheral but RASR=0 \
                     for partition {}",
                    partition_id.as_raw(),
                );
            }
        } else if idx != reserved {
            // Beyond peripheral-reserved range and not the RAM slot.
            // RASR must be zero OR slot must hold a window visible to
            // this partition (owned by it, or shared from another
            // partition outside that owner's peripheral-reserved range).
            if rasr != 0 {
                let is_visible_window = slots.get(idx).and_then(|s| s.as_ref()).is_some();
                assert!(
                    is_visible_window,
                    "stale MPU: slot {idx} has non-zero RASR={rasr:#x} \
                     but no window descriptor for partition {}",
                    partition_id.as_raw(),
                );
            }
        }
        // idx == reserved is the RAM slot — not checked here.
    }
}

/// Per-partition pre-computed (RBAR, RASR) pairs for peripheral regions
/// starting at R4, with count determined by `MAX_PERIPHERAL_REGIONS`.
type PeripheralCache<const N: usize> = [[(u32, u32); MAX_PERIPHERAL_REGIONS]; N];

/// Per-partition private-RAM (RBAR, RASR) descriptor, indexed by partition_id.
type PartitionRamCache<const N: usize> = [Option<(u32, u32)>; N];

/// Dynamic MPU strategy — manages regions R4-R7 at runtime.
///
/// ## Two-tier MPU region scheme
///
/// The layout of dynamic slots R4–R7 depends on
/// [`configure_partition`](MpuStrategy::configure_partition)'s
/// `peripheral_reserved` parameter:
///
/// **`peripheral_reserved = 0` (no peripherals):**
/// Slot 0 (R4) holds the partition's private RAM.  Slots 1–3 (R5–R7)
/// are ad-hoc windows available to
/// [`add_window`](MpuStrategy::add_window) (used by `sys_buf_lend`).
///
/// **`peripheral_reserved = 2` (peripheral-capable partitions):**
/// - **Reserved per-partition slots (R4–R5):** populated at boot by
///   [`wire_boot_peripherals`](Self::wire_boot_peripherals) into `slots[]`,
///   then overridden per-partition at every context switch by
///   [`cached_peripheral_regions`](Self::cached_peripheral_regions)
///   (called from `__pendsv_program_mpu` in `harness.rs`).
/// - **Partition RAM** moves to slot 2 (R6).
/// - **Dynamic window (R7):** allocated at runtime by
///   [`add_window`](MpuStrategy::add_window).
///
/// The `desc_idx < reserved.min(MAX_PERIPHERAL_REGIONS)` guard in
/// [`try_wire_region`](Self::try_wire_region) ensures peripherals handled
/// by the per-partition cache do NOT consume dynamic slots needed by
/// `sys_buf_lend`.
///
/// Pure data-structure tracker — no hardware writes.
///
/// Interior mutability is provided by `Mutex<RefCell<…>>`, which uses a
/// critical section (interrupt-disable) on single-core Cortex-M to ensure
/// exclusive access.
///
/// `N` is the maximum number of partitions whose peripheral regions can be
/// cached. Defaults to `DYNAMIC_SLOT_COUNT` (4) for backwards compatibility.
pub struct DynamicStrategy<const N: usize = DYNAMIC_SLOT_COUNT> {
    slots: Mutex<RefCell<[Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT]>>,
    /// Per-partition count of leading slots (0 or 2) reserved for peripheral
    /// MMIO regions, indexed by `partition_id`. Set by `configure_partition`;
    /// `wire_boot_peripherals` populates these slots with peripheral
    /// descriptors so `partition_region_values` emits them during PendSV.
    peripheral_reserved: Mutex<RefCell<[usize; N]>>,
    /// Pre-computed (RBAR, RASR) pairs for R4-R5, indexed by partition_id.
    peripheral_cache: Mutex<RefCell<PeripheralCache<N>>>,
    /// Per-partition private-RAM (RBAR, RASR) pair, indexed by `partition_id`.
    /// Set by `configure_partition`; `None` until configured.
    partition_ram: Mutex<RefCell<PartitionRamCache<N>>>,
}

impl<const N: usize> Default for DynamicStrategy<N> {
    fn default() -> Self {
        Self::with_partition_count()
    }
}

/// Backwards-compatible constructor.
impl DynamicStrategy {
    /// Create a new strategy with all slots empty and the default partition
    /// cache size (`DYNAMIC_SLOT_COUNT`).
    pub const fn new() -> Self {
        Self::with_partition_count()
    }
}

impl<const N: usize> DynamicStrategy<N> {
    /// Convert a hardware MPU region ID (4-7) to a `partition_region_values`
    /// array index (0-3), or `None` if the ID is out of range.
    pub fn region_to_slot_index(region_id: u8) -> Option<usize> {
        let idx = region_id.checked_sub(DYNAMIC_REGION_BASE)? as usize;
        if idx < DYNAMIC_SLOT_COUNT {
            Some(idx)
        } else {
            None
        }
    }
    /// Create a new strategy with all slots empty and a peripheral cache
    /// sized for `N` partitions.
    pub const fn with_partition_count() -> Self {
        const DISABLED: [(u32, u32); MAX_PERIPHERAL_REGIONS] = {
            let mut arr = [(0u32, 0u32); MAX_PERIPHERAL_REGIONS];
            let mut i = 0usize;
            while i < MAX_PERIPHERAL_REGIONS {
                arr[i] = disabled_pair(DYNAMIC_REGION_BASE + i as u8);
                i += 1;
            }
            arr
        };
        Self {
            slots: Mutex::new(RefCell::new([None; DYNAMIC_SLOT_COUNT])),
            peripheral_reserved: Mutex::new(RefCell::new([0; N])),
            peripheral_cache: Mutex::new(RefCell::new([DISABLED; N])),
            partition_ram: Mutex::new(RefCell::new([None; N])),
        }
    }

    /// Return the peripheral-reserved slot count for `partition_id` (0 if out of range).
    pub fn peripheral_reserved_for(&self, partition_id: PartitionId) -> usize {
        let idx = partition_id.as_raw() as usize;
        with_cs(|cs| {
            let pr = self.peripheral_reserved.borrow(cs);
            pr.borrow().get(idx).copied().unwrap_or(0)
        })
    }

    /// Return the stored private-RAM (RBAR, RASR) pair for `partition_id`,
    /// or `None` if the partition has not been configured or is out of range.
    pub fn partition_ram_for(&self, partition_id: PartitionId) -> Option<(u32, u32)> {
        let idx = partition_id.as_raw() as usize;
        with_cs(|cs| {
            let pr = self.partition_ram.borrow(cs);
            pr.borrow().get(idx).copied().flatten()
        })
    }

    /// Return a copy of the descriptor for a given hardware region ID (4-7),
    /// or `None` if the slot is empty or the ID is out of range.
    pub fn slot(&self, region_id: u8) -> Option<WindowDescriptor> {
        let idx = region_id.checked_sub(DYNAMIC_REGION_BASE)? as usize;
        with_cs(|cs| {
            let slots = self.slots.borrow(cs);
            let slots = slots.borrow();
            slots.get(idx).copied().flatten()
        })
    }

    /// Per-partition (RBAR, RASR) for R4-R7: peripherals from cache,
    /// RAM from `partition_ram_for`, cross-partition RAM disabled.
    pub fn partition_region_values(
        &self,
        partition_id: PartitionId,
    ) -> [(u32, u32); DYNAMIC_SLOT_COUNT] {
        fn rpair(base: u32, perm: u32, region: u32) -> (u32, u32) {
            crate::mpu::build_rbar(base, region)
                .map(|r| (r, perm))
                .unwrap_or(disabled_pair(region as u8))
        }
        with_cs(|cs| {
            let slots = self.slots.borrow(cs);
            let slots = slots.borrow();
            let pr = self.peripheral_reserved.borrow(cs);
            let pr = pr.borrow();
            let reserved = pr.get(partition_id.as_raw() as usize).copied().unwrap_or(0);
            let mut out = [(0u32, 0u32); DYNAMIC_SLOT_COUNT];
            // Fill from shared slots, disabling peripheral-reserved and RAM
            // slots (0..=owner_res) for non-owner partitions to prevent leakage.
            for (idx, (slot, dst)) in slots.iter().zip(out.iter_mut()).enumerate() {
                let region = DYNAMIC_REGION_BASE as u32 + idx as u32;
                *dst = match slot {
                    Some(desc) if desc.owner != partition_id => {
                        let owner_res = pr.get(desc.owner.as_raw() as usize).copied().unwrap_or(0);
                        if idx <= owner_res {
                            disabled_pair(region as u8)
                        } else {
                            rpair(desc.base, desc.permissions, region)
                        }
                    }
                    Some(desc) => rpair(desc.base, desc.permissions, region),
                    None => disabled_pair(region as u8),
                };
            }
            // Override peripheral-reserved slots 0..reserved from cache.
            if reserved >= 1 {
                let pcache = self.peripheral_cache.borrow(cs);
                let pcache = pcache.borrow();
                let cache = pcache
                    .get(partition_id.as_raw() as usize)
                    .copied()
                    .unwrap_or([
                        disabled_pair(DYNAMIC_REGION_BASE),
                        disabled_pair(DYNAMIC_REGION_BASE + 1),
                        disabled_pair(DYNAMIC_REGION_BASE + 2),
                    ]);
                for (dst, val) in out
                    .iter_mut()
                    .zip(cache.iter())
                    .take(reserved.min(cache.len()))
                {
                    *dst = *val;
                }
            }
            // Override the RAM slot with this partition's own RAM.
            let ram_reg = DYNAMIC_REGION_BASE as u32 + reserved as u32;
            let ram_pair = self
                .partition_ram
                .borrow(cs)
                .borrow()
                .get(partition_id.as_raw() as usize)
                .copied()
                .flatten()
                .map(|(rb, rs)| rpair(rb & crate::mpu::RBAR_ADDR_MASK, rs, ram_reg))
                .unwrap_or(disabled_pair(ram_reg as u8));
            // Disable stale peripheral slots between this partition's
            // reserved count and the maximum reserved across all partitions.
            // We disable starting from `reserved` (inclusive) so the slot is
            // explicitly cleared before being overwritten with RAM below.
            let max_reserved = pr.iter().copied().max().unwrap_or(0);
            for idx in reserved..=max_reserved {
                if let Some(dst) = out.get_mut(idx) {
                    let region = DYNAMIC_REGION_BASE as u32 + idx as u32;
                    *dst = disabled_pair(region as u8);
                }
            }
            // Override the disabled slot at `reserved` with this partition's RAM.
            if let Some(dst) = out.get_mut(reserved) {
                *dst = ram_pair;
            }
            #[cfg(debug_assertions)]
            {
                let pcache = self.peripheral_cache.borrow(cs);
                let pcache = pcache.borrow();
                let cache = pcache
                    .get(partition_id.as_raw() as usize)
                    .copied()
                    .unwrap_or([
                        disabled_pair(DYNAMIC_REGION_BASE),
                        disabled_pair(DYNAMIC_REGION_BASE + 1),
                        disabled_pair(DYNAMIC_REGION_BASE + 2),
                    ]);
                debug_assert_no_stale_regions(&out, partition_id, reserved, &cache, &slots);
            }
            out
        })
    }

    /// Directly overwrite a slot with an arbitrary descriptor.
    ///
    /// This bypasses `add_window` validation and is intended **only** for
    /// testing the `partition_region_values` fallback path (defense-in-depth).
    #[cfg(test)]
    pub fn inject_slot(&self, region_id: u8, desc: Option<WindowDescriptor>) {
        if let Some(idx) = Self::region_to_slot_index(region_id) {
            with_cs(|cs| {
                self.slots.borrow(cs).borrow_mut()[idx] = desc;
            });
        }
    }

    /// Return (base, size) pairs for all MPU windows currently assigned to
    /// the given partition.
    ///
    /// Iterates over all four dynamic slots (R4–R7) and collects descriptors
    /// where `owner == partition_id`. Returns an empty vector for partitions
    /// with no assigned windows.
    pub fn accessible_regions(&self, partition_id: PartitionId) -> heapless::Vec<(u32, u32), 4> {
        with_cs(|cs| {
            let slots = self.slots.borrow(cs);
            let slots = slots.borrow();
            slots
                .iter()
                .flatten()
                .filter(|desc| desc.owner == partition_id)
                .map(|desc| (desc.base, desc.size))
                .collect()
        })
    }

    /// Store pre-computed (RBAR, RASR) pairs for a partition's peripherals.
    ///
    /// `partition_id` indexes into the cache (max `N - 1`).
    /// Out-of-range IDs are silently ignored.
    pub fn cache_peripherals(
        &self,
        partition_id: PartitionId,
        descriptors: [Option<WindowDescriptor>; MAX_PERIPHERAL_REGIONS],
    ) {
        let idx = partition_id.as_raw() as usize;
        if idx >= N {
            return;
        }
        let pairs = core::array::from_fn(|i| {
            let rid = DYNAMIC_REGION_BASE + i as u8;
            match &descriptors[i] {
                Some(d) => crate::mpu::build_rbar(d.base, rid as u32)
                    .map(|rbar| (rbar, d.permissions))
                    .unwrap_or(disabled_pair(rid)),
                None => disabled_pair(rid),
            }
        });
        with_cs(|cs| {
            if let Some(entry) = self.peripheral_cache.borrow(cs).borrow_mut().get_mut(idx) {
                *entry = pairs;
            }
        });
    }

    /// Return pre-computed (RBAR, RASR) pairs for `partition_id`'s peripheral
    /// regions (R4-R6).
    ///
    /// Called from `__pendsv_program_mpu` (in `harness.rs`) at every context
    /// switch to overwrite the reserved peripheral slots (R4-R6) with the
    /// incoming partition's cached values.  Partitions without peripherals
    /// receive disabled pairs (RASR = 0).
    pub fn cached_peripheral_regions(
        &self,
        partition_id: PartitionId,
    ) -> [(u32, u32); MAX_PERIPHERAL_REGIONS] {
        let idx = partition_id.as_raw() as usize;
        if idx >= N {
            return core::array::from_fn(|i| disabled_pair(DYNAMIC_REGION_BASE + i as u8));
        }
        with_cs(|cs| self.peripheral_cache.borrow(cs).borrow()[idx])
    }

    /// Verify that cached (RBAR, RASR) pairs match the PCB's peripheral regions.
    ///
    /// Panics (via `debug_assert!`) if the cached RBAR does not match the
    /// expected value derived from the region's base address, or if the
    /// cached RASR is zero (disabled) for a mappable peripheral.
    #[cfg(debug_assertions)]
    pub fn debug_verify_cache_consistency(&self, part: &crate::partition::PartitionControlBlock) {
        let reserved = self.peripheral_reserved_for(part.id());
        let check_count = reserved.min(MAX_PERIPHERAL_REGIONS);
        let cached = self.cached_peripheral_regions(part.id());
        for (ci, region) in part
            .peripheral_regions()
            .iter()
            .filter(|r| r.is_mappable())
            .take(check_count)
            .enumerate()
        {
            let entry = cached.get(ci);
            debug_assert!(
                entry.is_some(),
                "cache index {} out of bounds for partition {}",
                ci,
                part.id().as_raw()
            );
            let (rbar, rasr) = match entry {
                Some(&pair) => pair,
                None => continue,
            };
            let expected_rbar =
                crate::mpu::build_rbar(region.base(), DYNAMIC_REGION_BASE as u32 + ci as u32);
            if let Some(expected) = expected_rbar {
                debug_assert_eq!(
                    rbar,
                    expected,
                    "cache-PCB RBAR mismatch: partition {} descriptor {}",
                    part.id().as_raw(),
                    ci
                );
            }
            debug_assert_ne!(
                rasr,
                0,
                "cache-PCB RASR must be non-zero (enabled): partition {} descriptor {}",
                part.id().as_raw(),
                ci
            );
        }
    }

    /// Wire a single unseen peripheral region via a two-way branch:
    ///
    /// 1. **Reserved slot** (`desc_idx < reserved`): store the descriptor
    ///    in the partition's reserved cache slot (R4 or R5).  Multiple
    ///    partitions may time-share the same physical slot; the
    ///    context-switch cache restores each partition's values.
    /// 2. **`add_window` fallback**: allocate a dynamic slot via the
    ///    normal [`add_window`](MpuStrategy::add_window) path.
    ///
    /// Returns `Ok(wired-count delta)`.
    fn try_wire_region(
        &self,
        seen: &mut heapless::Vec<(u32, u32, usize, u32), 8>,
        _wired: usize,
        desc_idx: usize,
        reserved: usize,
        region: (u32, u32, u32),
        part_id: PartitionId,
    ) -> Result<usize, MpuError> {
        let (base, size, rasr) = region;
        let (slot_idx, delta) = if desc_idx < reserved.min(MAX_PERIPHERAL_REGIONS) {
            with_cs(|cs| {
                let slots = &mut self.slots.borrow(cs).borrow_mut();
                // TODO(panic-free): InternalError is generic; consider a more specific variant
                let slot = slots.get_mut(desc_idx).ok_or(MpuError::InternalError)?;
                *slot = Some(WindowDescriptor {
                    base,
                    size,
                    permissions: rasr,
                    owner: part_id,
                    rbar: slot_rbar(base, desc_idx),
                });
                Ok::<(), MpuError>(())
            })?;
            (desc_idx, 1)
        } else {
            let rn = self.add_window(base, size, rasr, part_id)?;
            ((rn - DYNAMIC_REGION_BASE) as usize, 1)
        };
        if seen.push((base, size, slot_idx, rasr)).is_err() {
            debug_assert!(false, "seen overflow: dedup capacity exceeded");
        }
        Ok(delta)
    }

    /// Populate dynamic slots with deduplicated peripheral regions from
    /// the given partitions (device memory: S=1,C=0,B=1 (Shareable Device
    /// memory), XN, AP_FULL_ACCESS).
    ///
    /// Reserved peripheral slots (R4/R5) are populated at boot into
    /// `slots[]` and their per-partition (RBAR, RASR) pairs are cached so
    /// that [`cached_peripheral_regions`](Self::cached_peripheral_regions)
    /// can override them during each context switch (called from
    /// `__pendsv_program_mpu` in `harness.rs`).  Remaining regions use
    /// [`add_window`](MpuStrategy::add_window).
    ///
    /// Returns the number of dynamic slots wired.
    pub fn wire_boot_peripherals(
        &self,
        partitions: &[crate::partition::PartitionControlBlock],
    ) -> usize {
        // Deduplicate by (base, size).  Stores (base, size, slot_idx, rasr).
        // TODO: capacity should be derived from MAX_PARTITIONS *
        // MAX_PERIPHERAL_REGIONS instead of hardcoded 8.  Deferred:
        // requires adding a MAX_PARTITIONS constant and propagating it
        // through the type system.
        let mut seen: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        let mut wired = 0usize;

        for part in partitions.iter() {
            let reserved = self.peripheral_reserved_for(part.id());
            let mut part_descs: [Option<WindowDescriptor>; MAX_PERIPHERAL_REGIONS] =
                [None; MAX_PERIPHERAL_REGIONS];
            let mut desc_idx = 0usize;

            for region in part.peripheral_regions().iter() {
                if !region.is_mappable() {
                    continue;
                }
                let base = region.base();
                let size = region.size();
                let prior = seen
                    .iter()
                    .find(|(b, s, _, _)| *b == base && *s == size)
                    .copied();
                if prior.is_some() && desc_idx >= MAX_PERIPHERAL_REGIONS {
                    continue;
                }
                let rasr = if let Some((_, _, _, stored)) = prior {
                    stored
                } else {
                    compute_peripheral_rasr(size)
                };
                if prior.is_none() {
                    match self.try_wire_region(
                        &mut seen,
                        wired,
                        desc_idx,
                        reserved,
                        (base, size, rasr),
                        part.id(),
                    ) {
                        Ok(delta) => wired += delta,
                        Err(MpuError::SlotExhausted) => {
                            self.cache_peripherals(part.id(), part_descs);
                            return wired;
                        }
                        Err(_) => continue,
                    }
                }
                // Cache for this partition regardless of prior wiring;
                // shared peripherals must appear in every partition's
                // cache for correct context-switch restoration.
                if let Some(wd) = Self::build_partition_cache_entry(
                    &seen,
                    base,
                    size,
                    rasr,
                    desc_idx,
                    reserved,
                    part.id(),
                ) {
                    part_descs[desc_idx] = Some(wd);
                    desc_idx += 1;
                }
            }
            self.cache_peripherals(part.id(), part_descs);

            #[cfg(debug_assertions)]
            self.debug_verify_cache_consistency(part);
        }

        #[cfg(debug_assertions)]
        {
            let has_free = with_cs(|cs| self.slots.borrow(cs).borrow().iter().any(|s| s.is_none()));
            debug_assert!(
                has_free,
                "boot wiring consumed all dynamic window slots, none left for runtime add_window",
            );
        }

        wired
    }

    /// Build a cache entry for a peripheral region if `desc_idx` is
    /// within the reserved window and `seen` contains a matching slot.
    ///
    /// Returns `Some(WindowDescriptor)` when both conditions hold,
    /// `None` otherwise.
    fn build_partition_cache_entry(
        seen: &heapless::Vec<(u32, u32, usize, u32), 8>,
        base: u32,
        size: u32,
        rasr: u32,
        desc_idx: usize,
        reserved: usize,
        part_id: PartitionId,
    ) -> Option<WindowDescriptor> {
        if desc_idx >= reserved.min(MAX_PERIPHERAL_REGIONS) {
            return None;
        }
        let &(_, _, slot_idx, _) = seen.iter().find(|(b, s, _, _)| *b == base && *s == size)?;
        Some(WindowDescriptor {
            base,
            size,
            permissions: rasr,
            owner: part_id,
            rbar: slot_rbar(base, slot_idx),
        })
    }
}

impl<const N: usize> MpuStrategy for DynamicStrategy<N> {
    fn configure_partition(
        &self,
        partition_id: PartitionId,
        regions: &[(u32, u32)],
        peripheral_reserved: usize,
    ) -> Result<(), MpuError> {
        // The static regions (R0-R3) are handled elsewhere; we only
        // care about the single private-RAM region.
        if regions.len() != 1 {
            return Err(MpuError::RegionCountMismatch);
        }

        // peripheral_reserved must be in 0..=MAX_PERIPHERAL_REGIONS (i.e. 0-3).
        if peripheral_reserved > crate::partition::MAX_PERIPHERAL_REGIONS {
            return Err(MpuError::RegionCountMismatch);
        }
        // At least one dynamic slot is needed for the partition's RAM region.
        if peripheral_reserved >= DYNAMIC_SLOT_COUNT {
            return Err(MpuError::RegionCountMismatch);
        }

        let (rbar, rasr) = regions[0];
        let base = rbar & crate::mpu::RBAR_ADDR_MASK;
        let size_field = (rasr >> crate::mpu::RASR_SIZE_SHIFT) & crate::mpu::RASR_SIZE_MASK;
        let size = 1u32 << (size_field + 1);

        // Place private-RAM after the reserved peripheral slots.
        // peripheral_reserved=0 → slot 0 (R4), peripheral_reserved=2 → slot 2 (R6).
        let ram_slot = peripheral_reserved;

        with_cs(|cs| {
            let mut slots = self.slots.borrow(cs).borrow_mut();
            let mut pr = self.peripheral_reserved.borrow(cs).borrow_mut();
            // Validate partition_id is within the strategy's capacity.
            let pr_entry = match pr.get_mut(partition_id.as_raw() as usize) {
                Some(entry) => entry,
                None => return Err(MpuError::RegionCountMismatch),
            };
            // Clear any previously-occupied RAM slot so the strategy
            // state doesn't carry stale descriptors when the reservation
            // count changes between configure_partition calls.
            // Only clear if the slot is owned by this partition.
            let prev_reserved = *pr_entry;
            if prev_reserved != peripheral_reserved {
                if let Some(slot) = slots.get_mut(prev_reserved) {
                    if slot.is_some_and(|d| d.owner == partition_id) {
                        *slot = None;
                    }
                }
            }
            if let Some(slot) = slots.get_mut(ram_slot) {
                *slot = Some(WindowDescriptor {
                    base,
                    size,
                    permissions: rasr,
                    owner: partition_id,
                    rbar: slot_rbar(base, ram_slot),
                });
            }
            *pr_entry = peripheral_reserved;
            // Store the raw (RBAR, RASR) pair in partition_ram.
            let mut ram = self.partition_ram.borrow(cs).borrow_mut();
            if let Some(entry) = ram.get_mut(partition_id.as_raw() as usize) {
                *entry = Some((rbar, rasr));
            }
            Ok(())
        })
    }

    fn add_window(
        &self,
        base: u32,
        size: u32,
        permissions: u32,
        owner: PartitionId,
    ) -> Result<u8, MpuError> {
        crate::mpu::validate_mpu_region(base, size).inspect_err(|&_e| {
            crate::klog!(
                "add_window: validate_mpu_region failed base=0x{:08x} size={} err={:?}",
                base,
                size,
                _e
            );
        })?;

        with_cs(|cs| {
            let mut slots = self.slots.borrow(cs).borrow_mut();
            let pr = self.peripheral_reserved.borrow(cs);
            // Use the owner partition's reserved count: each partition
            // has its own peripheral cache region(s) at the start of
            // its slot view, so only that partition's count matters.
            let reserved = pr
                .borrow()
                .get(owner.as_raw() as usize)
                .copied()
                .unwrap_or(0);
            // Skip peripheral-reserved slots (0..reserved) and the
            // partition-RAM slot (reserved), scanning from
            // reserved+1 onwards for a free entry.
            let first_window = reserved + 1;
            for (idx, slot) in slots.iter_mut().enumerate().skip(first_window) {
                if slot.is_none() {
                    *slot = Some(WindowDescriptor {
                        base,
                        size,
                        permissions,
                        owner,
                        rbar: slot_rbar(base, idx),
                    });
                    return Ok(DYNAMIC_REGION_BASE + idx as u8);
                }
            }
            Err(MpuError::SlotExhausted)
        })
    }

    fn remove_window(&self, region_id: u8) {
        let idx = match region_id.checked_sub(DYNAMIC_REGION_BASE) {
            Some(i) if (i as usize) < DYNAMIC_SLOT_COUNT => i as usize,
            _ => return,
        };

        with_cs(|cs| {
            self.slots.borrow(cs).borrow_mut()[idx] = None;
        });
    }
}

/// Static MPU strategy — delegates to existing [`mpu`] helpers.
///
/// This strategy applies exactly 4 pre-computed (RBAR, RASR) region pairs
/// (background, code, data, stack guard) using the same disable-program-enable
/// sequence as [`mpu::apply_partition_mpu_cached`].
pub struct StaticStrategy;

/// The number of MPU regions the static layout requires.
const STATIC_REGION_COUNT: usize = 4;

impl MpuStrategy for StaticStrategy {
    fn configure_partition(
        &self,
        _partition_id: PartitionId,
        regions: &[(u32, u32)],
        _peripheral_reserved: usize,
    ) -> Result<(), MpuError> {
        let region_array: [(u32, u32); STATIC_REGION_COUNT] = regions
            .try_into()
            .map_err(|_| MpuError::RegionCountMismatch)?;

        apply_regions(&region_array);
        Ok(())
    }

    fn add_window(
        &self,
        _base: u32,
        _size: u32,
        _permissions: u32,
        _owner: PartitionId,
    ) -> Result<u8, MpuError> {
        // Static strategy does not support dynamic windows.
        Err(MpuError::SlotExhausted)
    }

    fn remove_window(&self, _region_id: u8) {
        // Nothing to do — no dynamic windows exist.
    }
}

/// Program the given (RBAR, RASR) region triples into the MPU hardware.
///
/// Disables the MPU, writes all regions, re-enables with PRIVDEFENA,
/// and issues DSB/ISB barriers — mirroring [`mpu::apply_partition_mpu_cached`].
#[cfg(not(test))]
fn apply_regions(regions: &[(u32, u32); STATIC_REGION_COUNT]) {
    // SAFETY: All unsafe operations in this block are MMIO writes to the
    // ARMv7-M MPU register block at its architecturally-defined address
    // (0xE000_ED90).  The pointer obtained from `cortex_m::peripheral::MPU::PTR`
    // is always valid and properly aligned for the MPU register block.
    //
    // The register programming sequence is:
    //   1. Disable the MPU (CTRL = 0) so region updates are atomic from
    //      the hardware's perspective.
    //   2. Write each (RBAR, RASR) pair — RBAR selects the region number
    //      (encoded in bits [3:0] with VALID bit set) and base address;
    //      RASR configures size, permissions, and memory attributes.
    //   3. Re-enable the MPU with PRIVDEFENA so privileged code retains
    //      a default memory map.
    //
    // DSB + ISB barriers between disable/enable ensure all preceding
    // memory accesses complete and the pipeline refetches with the new
    // MPU configuration.
    //
    // This function is only called during partition context switches,
    // which occur in PendSV (an exception handler running at the lowest
    // priority), so no pre-emption can observe a partially-configured MPU.
    unsafe {
        let rb = &*cortex_m::peripheral::MPU::PTR;

        rb.ctrl.write(0);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        for &(rbar, rasr) in regions {
            rb.rbar.write(rbar);
            rb.rasr.write(rasr);
        }

        // Verify PRIVDEFENA (bit 2) is set — the kernel relies on the default
        // memory map for privileged access.  Evaluated at compile time.
        const { assert!(mpu::MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 2) != 0) }

        rb.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

/// Test stub: validates the region array without touching hardware.
#[cfg(test)]
fn apply_regions(_regions: &[(u32, u32); STATIC_REGION_COUNT]) {
    // No hardware access in unit tests; the slice-to-array conversion
    // and error handling are exercised by the test-mode caller.
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mpu::{
        build_rasr, build_rbar, encode_size, partition_mpu_regions, AP_FULL_ACCESS, AP_NO_ACCESS,
        AP_RO_RO,
    };
    use crate::partition::{MpuRegion, PartitionControlBlock};

    /// Shorthand for `PartitionId::new(v)` in tests.
    fn pid(v: u32) -> PartitionId {
        PartitionId::new(v)
    }

    /// Helper: build a PCB with the given entry, data base, and data size.
    fn make_pcb(entry: u32, data_base: u32, data_size: u32) -> PartitionControlBlock {
        PartitionControlBlock::new(
            0,
            entry,
            data_base,
            data_base.wrapping_add(data_size),
            MpuRegion::new(data_base, data_size, 0),
        )
    }

    /// Helper: build a PCB with an explicit partition id.
    fn make_pcb_id(id: u8, entry: u32, data_base: u32, data_size: u32) -> PartitionControlBlock {
        PartitionControlBlock::new(
            id,
            entry,
            data_base,
            data_base.wrapping_add(data_size),
            MpuRegion::new(data_base, data_size, 0),
        )
    }

    // ------------------------------------------------------------------
    // configure_partition: slice-to-array conversion and dispatch
    // ------------------------------------------------------------------

    #[test]
    fn configure_partition_accepts_4_regions() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let regions = partition_mpu_regions(&pcb).unwrap();
        let strategy = StaticStrategy;

        // Passing the 4 computed regions should succeed.
        assert_eq!(strategy.configure_partition(pid(0), &regions, 0), Ok(()));
    }

    #[test]
    fn configure_partition_rejects_wrong_count() {
        let strategy = StaticStrategy;

        // Too few regions.
        assert_eq!(
            strategy.configure_partition(pid(0), &[(0x0, 0x0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // 3 regions is now too few (was the old count).
        assert_eq!(
            strategy.configure_partition(pid(0), &[(0, 0), (0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // Too many regions.
        assert_eq!(
            strategy.configure_partition(pid(0), &[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // Empty.
        assert_eq!(
            strategy.configure_partition(pid(0), &[], 0),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn configure_partition_with_real_region_values() {
        let pcb = make_pcb(0x0800_0000, 0x2000_0000, 4096);
        let regions = partition_mpu_regions(&pcb).unwrap();
        let strategy = StaticStrategy;

        // Verify the regions we pass through are correctly formed.
        let (bg_rbar, bg_rasr) = regions[0];
        assert_eq!(bg_rbar, build_rbar(0x0000_0000, 0).unwrap());
        assert_eq!(
            bg_rasr,
            build_rasr(31, AP_NO_ACCESS, true, (false, false, false))
        );

        let (code_rbar, code_rasr) = regions[1];
        assert_eq!(code_rbar, build_rbar(0x0800_0000, 1).unwrap());
        let size_field = encode_size(4096).unwrap();
        assert_eq!(
            code_rasr,
            build_rasr(size_field, AP_RO_RO, false, (false, false, false))
        );

        let (data_rbar, data_rasr) = regions[2];
        assert_eq!(data_rbar, build_rbar(0x2000_0000, 2).unwrap());
        assert_eq!(
            data_rasr,
            build_rasr(size_field, AP_FULL_ACCESS, true, (true, true, false))
        );

        let (guard_rbar, guard_rasr) = regions[3];
        assert_eq!(guard_rbar, build_rbar(0x2000_0000, 3).unwrap());
        let guard_sf = encode_size(32).unwrap();
        assert_eq!(
            guard_rasr,
            build_rasr(guard_sf, AP_NO_ACCESS, true, (false, false, false))
        );

        // And configure_partition accepts them.
        assert_eq!(strategy.configure_partition(pid(0), &regions, 0), Ok(()));
    }

    #[test]
    fn configure_partition_different_partitions() {
        let strategy = StaticStrategy;

        let pcb0 = make_pcb(0x0000_0000, 0x2000_0000, 1024);
        let r0 = partition_mpu_regions(&pcb0).unwrap();
        assert_eq!(strategy.configure_partition(pid(0), &r0, 0), Ok(()));

        let pcb1 = make_pcb(0x0000_0000, 0x2000_8000, 1024);
        let r1 = partition_mpu_regions(&pcb1).unwrap();
        assert_eq!(strategy.configure_partition(pid(1), &r1, 0), Ok(()));
    }

    // ------------------------------------------------------------------
    // add_window / remove_window on StaticStrategy
    // ------------------------------------------------------------------

    #[test]
    fn static_strategy_add_window_returns_slot_exhausted() {
        let strategy = StaticStrategy;
        assert_eq!(
            strategy.add_window(0x2000_0000, 256, 0, pid(0)),
            Err(MpuError::SlotExhausted),
        );
    }

    #[test]
    fn static_strategy_remove_window_is_noop() {
        let strategy = StaticStrategy;
        // Should not panic.
        strategy.remove_window(0);
        strategy.remove_window(7);
    }

    // ------------------------------------------------------------------
    // Trait object usage
    // ------------------------------------------------------------------

    #[test]
    fn static_strategy_satisfies_trait_object() {
        let strategy: &dyn MpuStrategy = &StaticStrategy;
        assert_eq!(
            strategy.add_window(0, 0, 0, pid(0)),
            Err(MpuError::SlotExhausted)
        );
        assert_eq!(
            strategy.configure_partition(pid(0), &[(0, 0), (0, 0), (0, 0), (0, 0)], 0),
            Ok(()),
        );
    }

    // ------------------------------------------------------------------
    // DynamicStrategy
    // ------------------------------------------------------------------

    /// Helper: build an (RBAR, RASR) pair for a RW/XN data region.
    fn data_region(base: u32, size_bytes: u32, region: u32) -> (u32, u32) {
        let sf = encode_size(size_bytes).unwrap();
        let rasr = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        (build_rbar(base, region).unwrap(), rasr)
    }

    #[test]
    fn dynamic_new_has_empty_slots() {
        let ds = DynamicStrategy::new();
        for rid in 4..=7 {
            assert!(ds.slot(rid).is_none());
        }
    }

    #[test]
    fn dynamic_configure_partition_stores_r4() {
        let ds = DynamicStrategy::new();
        let size_field = encode_size(4096).unwrap(); // 11
        let rasr = build_rasr(size_field, AP_FULL_ACCESS, true, (true, true, false));
        let rbar = build_rbar(0x2000_0000, 4).unwrap();
        assert_eq!(ds.configure_partition(pid(2), &[(rbar, rasr)], 0), Ok(()));

        let desc = ds.slot(4).expect("R4 should be occupied");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.size, 4096);
        assert_eq!(desc.permissions, rasr);
        assert_eq!(desc.owner, pid(2));
    }

    #[test]
    fn dynamic_configure_partition_rejects_wrong_count() {
        let ds = DynamicStrategy::new();
        // Empty.
        assert_eq!(
            ds.configure_partition(pid(0), &[], 0),
            Err(MpuError::RegionCountMismatch),
        );
        // Multiple regions — only exactly one is accepted.
        assert_eq!(
            ds.configure_partition(pid(0), &[(0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn dynamic_add_window_allocates_r5_r6_r7() {
        let ds = DynamicStrategy::new();
        let r5 = ds.add_window(0x2001_0000, 256, 0xAA, pid(1));
        assert_eq!(r5, Ok(5));

        let r6 = ds.add_window(0x2002_0000, 512, 0xBB, pid(2));
        assert_eq!(r6, Ok(6));

        let r7 = ds.add_window(0x2003_0000, 1024, 0xCC, pid(3));
        assert_eq!(r7, Ok(7));

        // Verify stored descriptors.
        let d5 = ds.slot(5).unwrap();
        assert_eq!(d5.base, 0x2001_0000);
        assert_eq!(d5.size, 256);
        assert_eq!(d5.permissions, 0xAA);
        assert_eq!(d5.owner, pid(1));

        let d6 = ds.slot(6).unwrap();
        assert_eq!(d6.base, 0x2002_0000);
        assert_eq!(d6.size, 512);
        assert_eq!(d6.owner, pid(2));

        let d7 = ds.slot(7).unwrap();
        assert_eq!(d7.base, 0x2003_0000);
        assert_eq!(d7.size, 1024);
        assert_eq!(d7.owner, pid(3));
    }

    #[test]
    fn dynamic_add_window_exhaustion() {
        let ds = DynamicStrategy::new();
        assert!(ds.add_window(0x2001_0000, 256, 0, pid(1)).is_ok()); // R5
        assert!(ds.add_window(0x2002_0000, 256, 0, pid(1)).is_ok()); // R6
        assert!(ds.add_window(0x2003_0000, 256, 0, pid(1)).is_ok()); // R7

        // Fourth dynamic window should fail — only 3 dynamic slots.
        assert_eq!(
            ds.add_window(0x2004_0000, 256, 0, pid(1)),
            Err(MpuError::SlotExhausted)
        );
    }

    #[test]
    fn dynamic_remove_window_clears_slot() {
        let ds = DynamicStrategy::new();
        let rid = ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        assert_eq!(rid, 5);
        assert!(ds.slot(5).is_some());

        ds.remove_window(5);
        assert!(ds.slot(5).is_none());
    }

    #[test]
    fn dynamic_remove_window_out_of_range_is_noop() {
        let ds = DynamicStrategy::new();
        // Should not panic for out-of-range region IDs.
        ds.remove_window(0);
        ds.remove_window(3);
        ds.remove_window(8);
        ds.remove_window(255);
    }

    #[test]
    fn dynamic_reallocation_after_removal() {
        let ds = DynamicStrategy::new();
        let r5 = ds.add_window(0x2001_0000, 256, 0x11, pid(1)).unwrap();
        let r6 = ds.add_window(0x2002_0000, 512, 0x22, pid(2)).unwrap();
        let r7 = ds.add_window(0x2003_0000, 1024, 0x33, pid(3)).unwrap();
        assert_eq!((r5, r6, r7), (5, 6, 7));

        // Remove R6, then a new add should reuse slot index 2 → R6.
        ds.remove_window(6);
        assert!(ds.slot(6).is_none());

        let new = ds.add_window(0x2004_0000, 2048, 0x44, pid(4)).unwrap();
        assert_eq!(new, 6);
        let d = ds.slot(6).unwrap();
        assert_eq!(d.base, 0x2004_0000);
        assert_eq!(d.size, 2048);
        assert_eq!(d.permissions, 0x44);
        assert_eq!(d.owner, pid(4));
    }

    #[test]
    fn dynamic_slot_out_of_range_returns_none() {
        let ds = DynamicStrategy::new();
        assert!(ds.slot(0).is_none());
        assert!(ds.slot(3).is_none());
        assert!(ds.slot(8).is_none());
    }

    #[test]
    fn dynamic_configure_partition_overwrites_r4() {
        let ds = DynamicStrategy::new();
        let r1 = data_region(0x2000_0000, 256, 4);
        ds.configure_partition(pid(1), &[r1], 0).unwrap();
        assert_eq!(ds.slot(4).unwrap().owner, pid(1));

        // Reconfigure with different partition.
        let r2 = data_region(0x2000_8000, 1024, 4);
        ds.configure_partition(pid(3), &[r2], 0).unwrap();
        let d = ds.slot(4).unwrap();
        assert_eq!(d.owner, pid(3));
        assert_eq!(d.base, 0x2000_8000);
        assert_eq!(d.size, 1024);
    }

    #[test]
    fn dynamic_add_window_does_not_touch_r4() {
        let ds = DynamicStrategy::new();
        let r = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[r], 0).unwrap();

        // Fill all dynamic slots.
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2002_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2003_0000, 256, 0, pid(1)).unwrap();

        // R4 should still hold the partition descriptor.
        let d = ds.slot(4).unwrap();
        assert_eq!(d.base, 0x2000_0000);
        assert_eq!(d.owner, pid(0));
    }

    #[test]
    fn dynamic_satisfies_trait_object() {
        let ds = DynamicStrategy::new();
        let strategy: &dyn MpuStrategy = &ds;
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, pid(1)), Ok(5));
        strategy.remove_window(5);
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, pid(1)), Ok(5));
    }

    #[test]
    fn configure_partition_accepts_peripheral_reserved_three() {
        // peripheral_reserved=3 places RAM at slot index 3.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        assert_eq!(ds.configure_partition(pid(0), &[(rbar, rasr)], 3), Ok(()));
        let desc = ds.slot(7).expect("RAM should be in slot 7 (R4+3)");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.owner, pid(0));
    }

    #[test]
    fn configure_partition_rejects_peripheral_reserved_four() {
        // 4 exceeds MAX_PERIPHERAL_REGIONS (3), rejected.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        assert_eq!(
            ds.configure_partition(pid(0), &[(rbar, rasr)], 4),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn configure_partition_peripheral_reserved_one_ram_at_index_one() {
        // Verify RAM slot placement for peripheral_reserved=1.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_4000, 4096, 4);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 1).unwrap();
        // Slot 0 (R4) should be empty (reserved for peripheral).
        assert!(ds.slot(4).is_none(), "R4 reserved for peripheral");
        // Slot 1 (R5) should contain RAM.
        let desc = ds.slot(5).expect("RAM placed at index 1 (R5)");
        assert_eq!(desc.base, 0x2000_4000);
        assert_eq!(desc.owner, pid(0));
    }

    #[test]
    fn configure_partition_switches_reservation() {
        // Switching from peripheral_reserved=0 to 2 moves RAM from slot 0 to slot 2.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 0).unwrap();
        assert!(ds.slot(4).is_some(), "RAM in R4 with reservation=0");

        // Reconfigure partition 1 with reservation=2.
        let (rbar2, rasr2) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar2, rasr2)], 2)
            .unwrap();
        // Raw slot 0 still holds partition 0's RAM; per-partition
        // filtering hides it from partition 1's view.
        let s4 = ds.slot(4).expect("partition 0 RAM persists in slot 4");
        assert_eq!(s4.owner, pid(0), "slot 4 belongs to partition 0");
        let desc = ds.slot(6).expect("RAM should be in R6");
        assert_eq!(desc.base, 0x2000_8000);
        assert_eq!(desc.owner, pid(1));

        // Partition 1's view: R4-R5 disabled (reserved, no peripherals
        // cached), R6 holds its RAM.
        let v = ds.partition_region_values(pid(1));
        assert_eq!(v[0].1, 0, "R4 disabled");
        assert_eq!(v[1].1, 0, "R5 disabled");
        assert_eq!(v[2].1, rasr2, "R6 holds partition RAM");
    }

    // ------------------------------------------------------------------
    // peripheral_reserved_for + per-partition isolation
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_reserved_for_defaults_and_out_of_range() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        assert_eq!(ds.peripheral_reserved_for(pid(0)), 0);
        assert_eq!(ds.peripheral_reserved_for(pid(1)), 0);
        assert_eq!(ds.peripheral_reserved_for(pid(5)), 0, "out-of-range → 0");
        assert_eq!(ds.peripheral_reserved_for(pid(255)), 0, "out-of-range → 0");
    }

    #[test]
    fn peripheral_reserved_for_tracks_per_partition() {
        let ds = DynamicStrategy::<4>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        assert_eq!(ds.peripheral_reserved_for(pid(0)), 2);
        assert_eq!(
            ds.peripheral_reserved_for(pid(1)),
            0,
            "partition 1 untouched"
        );
    }

    #[test]
    fn configure_partition_owner_check_prevents_cross_partition_clear() {
        let ds = DynamicStrategy::<4>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 0).unwrap();
        // Partition 1 with reserved=2 must NOT clear partition 0's slot 0.
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 2).unwrap();
        assert_eq!(ds.slot(4).expect("p0 RAM survives").owner, pid(0));
        assert_eq!(ds.slot(6).expect("p1 RAM in R6").owner, pid(1));
    }

    #[test]
    fn three_partition_heterogeneous_peripheral_reserved() {
        // 3 partitions: p0 reserved=2, p1 reserved=0, p2 reserved=2.
        let ds = DynamicStrategy::<3>::with_partition_count();

        // --- Configure each partition ---
        // p0: reserved=2 → RAM lands in slot 2 (region 6).
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();

        // p1: reserved=0 → RAM lands in slot 0 (region 4).
        let (rb1, rs1) = data_region(0x2000_4000, 4096, 4);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();

        // p2: reserved=2 → RAM lands in slot 2 (region 6).
        let (rb2, rs2) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(2), &[(rb2, rs2)], 2).unwrap();

        // --- Verify peripheral_reserved_for returns correct per-partition values ---
        assert_eq!(ds.peripheral_reserved_for(pid(0)), 2, "p0 reserved=2");
        assert_eq!(ds.peripheral_reserved_for(pid(1)), 0, "p1 reserved=0");
        assert_eq!(ds.peripheral_reserved_for(pid(2)), 2, "p2 reserved=2");

        // --- Verify RAM slot placement per partition ---
        // p0: reserved=2, RAM must be in slot 2 (region 6), owned by partition 0.
        let s0 = ds.slot(6).expect("p0 or p2 RAM in R6");
        // After p2 configures, it overwrites region 6; verify p2 owns it now.
        assert_eq!(s0.owner, pid(2), "p2 last wrote R6");
        assert_eq!(s0.base, 0x2000_8000, "p2 RAM base");

        // p1: reserved=0, RAM must be in slot 0 (region 4), owned by partition 1.
        let s1 = ds.slot(4).expect("p1 RAM in R4");
        assert_eq!(s1.owner, pid(1), "p1 owns R4");
        assert_eq!(s1.base, 0x2000_4000, "p1 RAM base");

        // --- Verify cross-partition owner isolation ---
        // Reconfigure p0 with reserved=2 again; must NOT clear p1's slot 0.
        let (rb0b, rs0b) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0b, rs0b)], 2).unwrap();

        // p1's RAM in R4 must survive p0's reconfiguration.
        let s1_after = ds.slot(4).expect("p1 RAM survives p0 reconfig");
        assert_eq!(s1_after.owner, pid(1), "p1 still owns R4");
        assert_eq!(s1_after.base, 0x2000_4000, "p1 RAM base unchanged");

        // p0 now owns R6 again.
        let s0_after = ds.slot(6).expect("p0 RAM in R6 after reconfig");
        assert_eq!(s0_after.owner, pid(0), "p0 owns R6 after reconfig");
        assert_eq!(s0_after.base, 0x2000_0000, "p0 RAM base");

        // Reconfigure p2 with reserved=0; must NOT clear p0's slot 2 (R6)
        // because owner differs. p2 should land in slot 0 (R4) now.
        let (rb2b, rs2b) = data_region(0x2000_C000, 4096, 4);
        ds.configure_partition(pid(2), &[(rb2b, rs2b)], 0).unwrap();
        assert_eq!(
            ds.peripheral_reserved_for(pid(2)),
            0,
            "p2 switched to reserved=0"
        );

        // p0's R6 must survive p2's switch.
        let s0_final = ds.slot(6).expect("p0 RAM survives p2 switch");
        assert_eq!(s0_final.owner, pid(0), "p0 still owns R6");

        // p2 now in R4; p1 was overwritten since both target slot 0.
        // (Both p1 and p2 with reserved=0 share slot 0 — last writer wins.)
        let s2_final = ds.slot(4).expect("p2 RAM in R4");
        assert_eq!(s2_final.owner, pid(2), "p2 now owns R4");
        assert_eq!(s2_final.base, 0x2000_C000, "p2 RAM base in R4");
    }

    // ------------------------------------------------------------------
    // Single-peripheral and multi-peripheral slot allocation tests
    // ------------------------------------------------------------------

    #[test]
    fn one_peripheral_reserved_for_returns_one() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 5);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 1).unwrap();
        assert_eq!(
            ds.peripheral_reserved_for(pid(0)),
            1,
            "1 peripheral → peripheral_reserved_for must return 1"
        );
    }

    #[test]
    fn one_peripheral_ram_at_slot_index_one() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 5);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 1).unwrap();
        // Slot 0 (R4) reserved for peripheral, should be None.
        assert!(
            ds.slot(4).is_none(),
            "slot 0 (R4) must be None — reserved for peripheral"
        );
        // Slot 1 (R5) should hold the RAM region.
        let desc = ds.slot(5).expect("RAM must be at slot index 1 (R5)");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.owner, pid(0));
    }

    #[test]
    fn one_peripheral_slots_two_and_three_available() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 5);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 1).unwrap();
        // Slots 2 and 3 (R6, R7) must remain None — available for
        // buffer lend windows.
        assert!(
            ds.slot(6).is_none(),
            "slot 2 (R6) must be None — available for lend windows"
        );
        assert!(
            ds.slot(7).is_none(),
            "slot 3 (R7) must be None — available for lend windows"
        );
    }

    #[test]
    fn three_peripherals_ram_at_slot_three_peripherals_at_zero_to_two() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 7);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 3).unwrap();
        assert_eq!(
            ds.peripheral_reserved_for(pid(0)),
            3,
            "3 peripherals → peripheral_reserved_for must return 3"
        );
        // RAM must be at slot 3 (R7).
        let desc = ds.slot(7).expect("RAM must be at slot index 3 (R7)");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.owner, pid(0));
        // Slots 0-2 (R4-R6) reserved for peripherals, should be None.
        assert!(
            ds.slot(4).is_none(),
            "slot 0 (R4) must be None — reserved for peripheral"
        );
        assert!(
            ds.slot(5).is_none(),
            "slot 1 (R5) must be None — reserved for peripheral"
        );
        assert!(
            ds.slot(6).is_none(),
            "slot 2 (R6) must be None — reserved for peripheral"
        );
    }

    /// Simulates the boot-time loop that configures ALL partitions
    /// (boot + non-boot) before `wire_boot_peripherals`, verifying
    /// that every partition's `peripheral_reserved` is set correctly.
    #[test]
    fn boot_loop_configures_all_partitions_peripheral_reserved() {
        let ds = DynamicStrategy::<3>::with_partition_count();
        let boot_pid = pid(0);

        // Per-partition peripheral counts: p0 has 2, p1 has 0, p2 has 1.
        let periph_counts: [usize; 3] = [2, 0, 1];
        let bases: [u32; 3] = [0x2000_0000, 0x2000_4000, 0x2000_8000];

        // Step 1: configure boot partition (as the macro does first).
        let region_num_boot = DYNAMIC_REGION_BASE as u32 + periph_counts[0] as u32;
        let (rb, rs) = data_region(bases[0], 4096, region_num_boot);
        ds.configure_partition(boot_pid, &[(rb, rs)], periph_counts[0])
            .unwrap();

        // Step 2: configure all remaining partitions (the new loop).
        for i in 0u32..3 {
            let p = PartitionId::new(i);
            if p == boot_pid {
                continue;
            }
            let pr = periph_counts[i as usize];
            let rn = DYNAMIC_REGION_BASE as u32 + pr as u32;
            let (rb_i, rs_i) = data_region(bases[i as usize], 4096, rn);
            ds.configure_partition(p, &[(rb_i, rs_i)], pr).unwrap();
        }

        // Verify: all partitions have correct peripheral_reserved.
        assert_eq!(
            ds.peripheral_reserved_for(pid(0)),
            2,
            "p0 has 2 peripherals"
        );
        assert_eq!(
            ds.peripheral_reserved_for(pid(1)),
            0,
            "p1 has no peripherals"
        );
        assert_eq!(ds.peripheral_reserved_for(pid(2)), 1, "p2 has 1 peripheral");
    }

    // ------------------------------------------------------------------
    // DynamicStrategy::partition_ram_for
    // ------------------------------------------------------------------

    #[test]
    fn partition_ram_for_returns_none_unconfigured() {
        let ds = DynamicStrategy::<3>::with_partition_count();
        assert_eq!(ds.partition_ram_for(pid(0)), None);
        assert_eq!(ds.partition_ram_for(pid(1)), None);
        assert_eq!(ds.partition_ram_for(pid(2)), None);
        // Out-of-range returns None.
        assert_eq!(ds.partition_ram_for(pid(3)), None);
        assert_eq!(ds.partition_ram_for(pid(255)), None);
    }

    #[test]
    fn partition_ram_for_returns_values_after_configure() {
        let ds = DynamicStrategy::<3>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 4);
        let (rb1, rs1) = data_region(0x2000_4000, 4096, 6);

        ds.configure_partition(pid(0), &[(rb0, rs0)], 0).unwrap();
        ds.configure_partition(pid(1), &[(rb1, rs1)], 2).unwrap();

        let ram0 = ds.partition_ram_for(pid(0)).expect("p0 configured");
        assert_eq!(ram0, (rb0, rs0));

        let ram1 = ds.partition_ram_for(pid(1)).expect("p1 configured");
        assert_eq!(ram1, (rb1, rs1));

        // p2 still unconfigured.
        assert_eq!(ds.partition_ram_for(pid(2)), None);
    }

    // ------------------------------------------------------------------
    // DynamicStrategy::add_window validation integration
    // ------------------------------------------------------------------

    #[test]
    fn dynamic_add_window_rejects_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 16, 0, pid(1)), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn dynamic_add_window_rejects_non_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0, 48, 0, pid(1)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
    }

    #[test]
    fn dynamic_add_window_rejects_misaligned_base() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(64, 256, 0, pid(1)),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn dynamic_add_window_rejects_overflow() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, pid(1)),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn dynamic_add_window_validation_before_slot_check() {
        let ds = DynamicStrategy::new();
        // Fill all dynamic slots with valid windows.
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, 0, pid(1)).unwrap(); // R6
        ds.add_window(0x2003_0000, 256, 0, pid(1)).unwrap(); // R7

        // Even with all slots full, invalid params yield validation
        // errors, not SlotExhausted.
        assert_eq!(ds.add_window(0, 16, 0, pid(1)), Err(MpuError::SizeTooSmall));
        assert_eq!(
            ds.add_window(0, 48, 0, pid(1)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
        assert_eq!(
            ds.add_window(64, 256, 0, pid(1)),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn dynamic_add_window_valid_params_still_succeed() {
        let ds = DynamicStrategy::new();
        // Valid aligned pair should succeed and return region ID.
        assert_eq!(ds.add_window(0x2001_0000, 256, 0xAA, pid(1)), Ok(5));
        let d = ds.slot(5).unwrap();
        assert_eq!(d.base, 0x2001_0000);
        assert_eq!(d.size, 256);
        assert_eq!(d.permissions, 0xAA);
        assert_eq!(d.owner, pid(1));
    }

    // ------------------------------------------------------------------
    // DynamicStrategy::add_window — exhaustive validation coverage
    // ------------------------------------------------------------------

    #[test]
    fn add_window_size_zero_returns_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 0, 0, pid(0)), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn add_window_size_16_returns_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 16, 0, pid(0)), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn add_window_size_48_returns_not_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0, 48, 0, pid(0)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
    }

    #[test]
    fn add_window_size_100_returns_not_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0, 100, 0, pid(0)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
    }

    #[test]
    fn add_window_base_off_by_one_returns_not_aligned() {
        let ds = DynamicStrategy::new();
        // 0x2000_0001 is 1 byte past a 256-byte boundary.
        assert_eq!(
            ds.add_window(0x2000_0001, 256, 0, pid(1)),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn add_window_base_half_aligned_returns_not_aligned() {
        let ds = DynamicStrategy::new();
        // 0x2000_0080 is 128-aligned but not 256-aligned.
        assert_eq!(
            ds.add_window(0x2000_0080, 256, 0, pid(1)),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn add_window_max_address_overflow() {
        let ds = DynamicStrategy::new();
        // base near u32::MAX with small valid size overflows.
        assert_eq!(
            ds.add_window(0xFFFF_FFE0, 32, 0, pid(0)),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn add_window_overflow_base_0xffff_ff00_size_256() {
        let ds = DynamicStrategy::new();
        // 0xFFFF_FF00 + 256 = 0x1_0000_0000 which overflows u32.
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, pid(0)),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn add_window_size_32_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2000_0000, 32, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_size_64_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2000_0040, 64, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_size_4096_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2001_0000, 4096, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_32_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 32, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_64_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 64, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_4096_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 4096, 0, pid(0)), Ok(5));
    }

    #[test]
    fn add_window_validation_precedes_slot_exhaustion_all_variants() {
        let ds = DynamicStrategy::new();
        // Fill all 3 dynamic slots.
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2002_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2003_0000, 256, 0, pid(1)).unwrap();

        // Every validation error is returned instead of SlotExhausted.
        assert_eq!(ds.add_window(0, 0, 0, pid(0)), Err(MpuError::SizeTooSmall));
        assert_eq!(ds.add_window(0, 16, 0, pid(0)), Err(MpuError::SizeTooSmall));
        assert_eq!(
            ds.add_window(0, 48, 0, pid(0)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
        assert_eq!(
            ds.add_window(0, 100, 0, pid(0)),
            Err(MpuError::SizeNotPowerOfTwo)
        );
        assert_eq!(
            ds.add_window(0x2000_0001, 256, 0, pid(0)),
            Err(MpuError::BaseNotAligned)
        );
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, pid(0)),
            Err(MpuError::AddressOverflow)
        );

        // But a valid request on full slots gives SlotExhausted.
        assert_eq!(
            ds.add_window(0x2004_0000, 256, 0, pid(1)),
            Err(MpuError::SlotExhausted)
        );
    }

    #[test]
    fn add_window_skips_reserved_slots_heterogeneous_counts() {
        // Per-partition peripheral_reserved: p0 reserved=2, p1 reserved=0.
        // owner=1 (reserved=0): skip 0 peripheral + 1 RAM → first_window=1 → slot 1 → region 5.
        // owner=0 (reserved=2): skip 2 peripheral + 1 RAM → first_window=3 → slot 3 → region 7.
        let ds = DynamicStrategy::<3>::with_partition_count();

        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();

        let (rb1, rs1) = data_region(0x2001_0000, 4096, 4);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();

        // owner=1 has reserved=0, so first_window=1 → slot 1 → region 5.
        assert_eq!(ds.add_window(0x2002_0000, 256, 0, pid(1)), Ok(5));

        // owner=0 has reserved=2, so first_window=3 → slot 3 → region 7.
        assert_eq!(ds.add_window(0x2003_0000, 256, 0, pid(0)), Ok(7));
    }

    // ------------------------------------------------------------------
    // accessible_regions
    // ------------------------------------------------------------------

    #[test]
    fn accessible_regions_multiple_slots_same_owner() {
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(1), &[(rbar, rasr)], 0).unwrap();
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2002_0000, 512, 0, pid(1)).unwrap();

        let result = ds.accessible_regions(pid(1));
        assert_eq!(result.len(), 3);
        assert!(result.contains(&(0x2000_0000, 4096)));
        assert!(result.contains(&(0x2001_0000, 256)));
        assert!(result.contains(&(0x2002_0000, 512)));
    }

    #[test]
    fn accessible_regions_mixed_owners() {
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rbar, rasr)], 0).unwrap();
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        ds.add_window(0x2002_0000, 512, 0, pid(0)).unwrap();
        ds.add_window(0x2003_0000, 1024, 0, pid(2)).unwrap();

        let p0 = ds.accessible_regions(pid(0));
        assert_eq!(p0.len(), 2);
        assert!(p0.contains(&(0x2000_0000, 4096)));
        assert!(p0.contains(&(0x2002_0000, 512)));

        assert_eq!(ds.accessible_regions(pid(1)).len(), 1);
        assert_eq!(ds.accessible_regions(pid(2)).len(), 1);
    }

    #[test]
    fn accessible_regions_no_owners_returns_empty() {
        let ds = DynamicStrategy::new();
        // Empty strategy returns empty for any partition.
        assert!(ds.accessible_regions(pid(0)).is_empty());
        assert!(ds.accessible_regions(pid(255)).is_empty());

        // With windows present but different owner.
        ds.add_window(0x2001_0000, 256, 0, pid(1)).unwrap();
        assert!(ds.accessible_regions(pid(0)).is_empty());
        assert!(ds.accessible_regions(pid(2)).is_empty());
    }

    fn periph(base: u32, size: u32) -> MpuRegion {
        MpuRegion::new(base, size, 0)
    }

    // ------------------------------------------------------------------
    // Peripheral partition-switch region emission
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_partition_switch_emits_correct_rasr() {
        let ds = DynamicStrategy::new();

        // Expected device-memory RASR for a 4 KiB peripheral region:
        // AP=FULL_ACCESS, XN=1, S/C/B=1/0/1 (Device), SIZE=11, ENABLE=1.
        let periph_rasr = build_rasr(
            4096u32.trailing_zeros() - 1, // size_field = 11
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );

        // ---- Phase 1: peripheral partition (reserved=2) active ----
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let pcb =
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        assert_eq!(ds.wire_boot_peripherals(&[pcb]), 1);

        let v1 = ds.partition_region_values(pid(0));
        assert_eq!(
            v1[0].0,
            build_rbar(0x4000_0000, 4).unwrap(),
            "R4 RBAR must select peripheral base"
        );
        assert_eq!(
            v1[0].1, periph_rasr,
            "R4 RASR must match device-memory peripheral encoding"
        );
        assert_eq!(v1[1].1, 0, "R5 disabled (no second peripheral wired)");

        // ---- Phase 2: non-peripheral partition (reserved=0) active ----
        // Switch to a partition with 0 peripheral reservations.
        // configure_partition moves peripheral_reserved from 2→0:
        //   - clears slot[2] (old RAM at R6)
        //   - places new RAM in slot[0] (R4), displacing the peripheral
        // No inject_slot calls — we verify configure_partition's actual
        // transition logic displaces the old peripheral descriptor.
        let (rbar_r4, rasr_r4) = data_region(0x2000_8000, 4096, 4);
        ds.configure_partition(pid(1), &[(rbar_r4, rasr_r4)], 0)
            .unwrap();

        let v2 = ds.partition_region_values(pid(1));
        assert_eq!(
            v2[0].0,
            build_rbar(0x2000_8000, 4).unwrap(),
            "R4 must hold new partition's RAM, displacing old peripheral"
        );
        assert_eq!(
            v2[0].1, rasr_r4,
            "R4 RASR must match partition 1 data region"
        );
        assert_eq!(v2[1].1, 0, "R5 disabled (no peripheral reservation)");
        assert_eq!(v2[2].1, 0, "R6 disabled (old RAM cleared by transition)");
        assert_eq!(v2[3].1, 0, "R7 disabled (no windows)");

        // ---- Phase 3: switch back to peripheral partition (reserved=2) ----
        // configure_partition restores peripheral_reserved=2 for partition 0,
        // placing RAM back in slot[2] (R6).  The peripheral cache from
        // phase 1 persists, so partition 0's view still shows R4 as the
        // cached peripheral (not empty).
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();

        let v3_pre = ds.partition_region_values(pid(0));
        assert_ne!(
            v3_pre[0].1, 0,
            "R4 shows cached peripheral from phase 1 wiring"
        );
        assert_eq!(
            v3_pre[2].1, rasr_r6,
            "R6 holds partition RAM after switch-back"
        );

        // NOTE: wire_boot_peripherals is called once at boot; the PendSV
        // handler restores per-partition peripheral regions via
        // cached_peripheral_regions (see harness.rs).  This re-call
        // below simulates the boot-time wiring for the new PCB.
        let pcb2 =
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        assert_eq!(ds.wire_boot_peripherals(&[pcb2]), 1);

        let v3 = ds.partition_region_values(pid(0));
        assert_eq!(
            v3[0].0,
            build_rbar(0x4000_0000, 4).unwrap(),
            "R4 RBAR must re-select peripheral base after round-trip"
        );
        assert_eq!(
            v3[0].1, periph_rasr,
            "R4 RASR must re-emit peripheral encoding after round-trip"
        );
        assert_eq!(v3[2].1, rasr_r6, "R6 still holds partition RAM");
    }

    #[test]
    fn wire_boot_peripherals_correctness() {
        let ds = DynamicStrategy::new();
        let (rb, rs) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rb, rs)], 0).unwrap();
        let pcbs = [
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]),
            make_pcb(0x0, 0x2000_8000, 4096).with_peripheral_regions(&[periph(0x4001_0000, 256)]),
        ];
        assert_eq!(ds.wire_boot_peripherals(&pcbs), 2);
        assert_eq!(ds.slot(4).unwrap().base, 0x2000_0000); // R4 unchanged
        let d5 = ds.slot(5).unwrap();
        assert_eq!((d5.base, d5.size), (0x4000_0000, 4096));
        // Verify device-memory RASR: AP=full, XN=1, S/C/B=1/0/1.
        let r = d5.permissions;
        assert_eq!((r >> 24) & 0x7, AP_FULL_ACCESS);
        assert_eq!((r >> 28) & 1, 1);
        assert_eq!((r >> 16) & 0x7, 0b101);
        assert_eq!(ds.slot(6).unwrap().base, 0x4001_0000);
        assert!(ds.slot(7).is_none());
        // Invalid size (100) skipped; valid 256 wired.
        let ds2 = DynamicStrategy::new();
        let p = make_pcb(0x0, 0x2000_0000, 4096)
            .with_peripheral_regions(&[periph(0x4000_0000, 100), periph(0x4001_0000, 256)]);
        assert_eq!(ds2.wire_boot_peripherals(&[p]), 1);
        assert_eq!(ds2.slot(5).unwrap().base, 0x4001_0000);
        // Dedup: same (base,size) across partitions → one slot.
        let ds3 = DynamicStrategy::new();
        let dup = [
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]),
            make_pcb(0x0, 0x2000_8000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]),
        ];
        assert_eq!(ds3.wire_boot_peripherals(&dup), 1);
        assert!(ds3.slot(6).is_none());
        // Exhaustion: 4 unique regions, 3 dynamic slots.
        let ds4 = DynamicStrategy::new();
        let many = [
            make_pcb(0x0, 0x2000_0000, 4096)
                .with_peripheral_regions(&[periph(0x4000_0000, 4096), periph(0x4001_0000, 4096)]),
            make_pcb(0x0, 0x2000_8000, 4096)
                .with_peripheral_regions(&[periph(0x4002_0000, 4096), periph(0x4003_0000, 4096)]),
        ];
        assert_eq!(ds4.wire_boot_peripherals(&many), 3);
        assert!(ds4.slot(5).is_some() && ds4.slot(6).is_some() && ds4.slot(7).is_some());
    }

    // ------------------------------------------------------------------
    // Multi-partition peripheral context-switch differentiation
    // ------------------------------------------------------------------

    #[test]
    fn multi_partition_peripheral_context_switch_differentiation() {
        use crate::mpu::peripheral_mpu_regions_or_disabled;

        // Partition 0: UART0 peripheral at 0x4000_C000, 4 KiB.
        // Partition 1: GPIO  peripheral at 0x4002_5000, 4 KiB.
        let pcb0 = make_pcb_id(0, 0x0, 0x2000_0000, 4096)
            .with_peripheral_regions(&[periph(0x4000_C000, 4096)]);
        let pcb1 = make_pcb_id(1, 0x0, 0x2000_8000, 4096)
            .with_peripheral_regions(&[periph(0x4002_5000, 4096)]);
        // Partition 2: no peripherals.
        let pcb_none = make_pcb_id(2, 0x0, 0x2001_0000, 4096);

        // --- (1) wire_boot_peripherals populates reserved slots and
        //         partition_region_values returns non-zero RASR for them ---
        let ds = DynamicStrategy::new();
        let (rbar_r6_0, rasr_r6_0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6_0, rasr_r6_0)], 2)
            .unwrap();
        let (rbar_r6_1, rasr_r6_1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_r6_1, rasr_r6_1)], 2)
            .unwrap();
        let wired = ds.wire_boot_peripherals(&[pcb0.clone(), pcb1.clone()]);
        assert_eq!(wired, 2, "both peripherals must be wired");
        // Partition 0's view: R4 = UART0, R5 disabled.
        let vals_p0 = ds.partition_region_values(pid(0));
        assert_ne!(vals_p0[0].1, 0, "R4 RASR must be non-zero (UART0)");
        assert_eq!(vals_p0[1].1, 0, "R5 RASR disabled (no second peripheral)");
        // Partition 1's view: R4 = GPIO, R5 disabled.
        let vals_p1 = ds.partition_region_values(pid(1));
        assert_ne!(vals_p1[0].1, 0, "R4 RASR must be non-zero (GPIO)");
        assert_eq!(vals_p1[1].1, 0, "R5 RASR disabled (no second peripheral)");

        // --- (2) peripheral_mpu_regions_or_disabled returns per-partition
        //         R4/R5 values for context-switch reprogramming ---
        //
        // NOTE: peripheral_mpu_regions_or_disabled is a standalone pure
        // function (static-mode path).  In dynamic mode the PendSV handler
        // uses DynamicStrategy::cached_peripheral_regions instead.  The
        // equivalence between the two paths is asserted by the
        // cache_vs_pcb_peripheral_rbar_rasr_equivalence test.
        let regions_p0 = peripheral_mpu_regions_or_disabled(&pcb0);
        let regions_p1 = peripheral_mpu_regions_or_disabled(&pcb1);

        // peripheral_mpu_regions assigns each partition's first peripheral
        // to R4 and second (if any) to R5, regardless of global boot-time
        // slot order.  Both partitions here have exactly one peripheral,
        // so R4 is enabled and R5 is disabled for each.

        // Partition 0: R4 = UART0 (enabled), R5 = disabled.
        assert_ne!(regions_p0[0].1, 0, "partition 0 R4 RASR enabled (UART0)");
        assert_eq!(
            regions_p0[0].0 & !0x1F,
            0x4000_C000,
            "partition 0 R4 RBAR must encode UART0 base 0x4000_C000"
        );
        assert_eq!(regions_p0[1].1, 0, "partition 0 R5 RASR disabled");

        // Partition 1: R4 = GPIO (enabled), R5 = disabled.
        assert_ne!(regions_p1[0].1, 0, "partition 1 R4 RASR enabled (GPIO)");
        assert_eq!(
            regions_p1[0].0 & !0x1F,
            0x4002_5000,
            "partition 1 R4 RBAR must encode GPIO base 0x4002_5000"
        );
        assert_eq!(regions_p1[1].1, 0, "partition 1 R5 RASR disabled");

        // R4 RBAR must differ between partitions (different peripherals).
        assert_ne!(
            regions_p0[0].0, regions_p1[0].0,
            "RBAR for UART0 vs GPIO must differ"
        );

        // --- (3) partition with no peripherals gets disabled entries ---
        let regions_none = peripheral_mpu_regions_or_disabled(&pcb_none);
        assert_eq!(
            regions_none[0].1, 0,
            "no-peripheral partition R4 RASR must be 0 (disabled)"
        );
        assert_eq!(
            regions_none[1].1, 0,
            "no-peripheral partition R5 RASR must be 0 (disabled)"
        );
    }

    // ------------------------------------------------------------------
    // peripheral_cache: cache_peripherals / cached_peripherals
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_cache_store_and_retrieve() {
        let ds = DynamicStrategy::new();
        let descs = [
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: 0xAB,
                owner: pid(0),
                rbar: 0,
            }),
            Some(WindowDescriptor {
                base: 0x4001_0000,
                size: 256,
                permissions: 0xCD,
                owner: pid(0),
                rbar: 0,
            }),
            None,
        ];
        ds.cache_peripherals(pid(0), descs);
        let got = ds.cached_peripheral_regions(pid(0));
        assert_eq!(got[0], (build_rbar(0x4000_0000, 4).unwrap(), 0xAB));
        assert_eq!(got[1], (build_rbar(0x4001_0000, 5).unwrap(), 0xCD));
    }

    #[test]
    fn peripheral_cache_uncached_returns_disabled() {
        let ds = DynamicStrategy::new();
        let disabled_r4 = disabled_pair(DYNAMIC_REGION_BASE);
        let disabled_r5 = disabled_pair(DYNAMIC_REGION_BASE + 1);
        let disabled_r6 = disabled_pair(DYNAMIC_REGION_BASE + 2);
        let disabled = [disabled_r4, disabled_r5, disabled_r6];
        assert_eq!(ds.cached_peripheral_regions(pid(0)), disabled);
        assert_eq!(ds.cached_peripheral_regions(pid(3)), disabled);
        // Out-of-range partition IDs also return disabled pairs.
        assert_eq!(ds.cached_peripheral_regions(pid(255)), disabled);
    }

    #[test]
    fn peripheral_cache_overwrite_updates_entry() {
        let ds = DynamicStrategy::new();
        let first = [
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: 0x11,
                owner: pid(1),
                rbar: 0,
            }),
            None,
            None,
        ];
        ds.cache_peripherals(pid(1), first);
        let got1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(got1[0], (build_rbar(0x4000_0000, 4).unwrap(), 0x11));
        assert_eq!(got1[1], disabled_pair(DYNAMIC_REGION_BASE + 1));

        let second = [
            Some(WindowDescriptor {
                base: 0x4002_0000,
                size: 512,
                permissions: 0x22,
                owner: pid(1),
                rbar: 0,
            }),
            Some(WindowDescriptor {
                base: 0x4003_0000,
                size: 1024,
                permissions: 0x33,
                owner: pid(1),
                rbar: 0,
            }),
            None,
        ];
        ds.cache_peripherals(pid(1), second);
        let got2 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(got2[0], (build_rbar(0x4002_0000, 4).unwrap(), 0x22));
        assert_eq!(got2[1], (build_rbar(0x4003_0000, 5).unwrap(), 0x33));
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals populates peripheral_cache
    // ------------------------------------------------------------------

    #[test]
    fn wire_boot_peripherals_populates_cache() {
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        // Two partitions with distinct IDs and one peripheral each.
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        assert_eq!(ds.wire_boot_peripherals(&[pcb0, pcb1]), 2);

        let rasr_4k = build_rasr(
            4096u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );
        let rasr_256 = build_rasr(
            256u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );

        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0],
            (build_rbar(0x4000_0000, 4).unwrap(), rasr_4k),
            "partition 0 R4 must hold 4KiB peripheral"
        );
        assert_eq!(
            cached0[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 0 R5 must be disabled"
        );

        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0],
            (build_rbar(0x4001_0000, 4).unwrap(), rasr_256),
            "partition 1 R4 must hold 256B peripheral"
        );
        assert_eq!(
            cached1[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 1 R5 must be disabled"
        );
    }

    #[test]
    fn wire_boot_peripherals_no_peripherals_cached_as_disabled() {
        let ds = DynamicStrategy::new();
        let pcb = make_pcb(0x0, 0x2000_0000, 4096); // no peripheral regions
        ds.wire_boot_peripherals(&[pcb]);
        let cached = ds.cached_peripheral_regions(pid(0));
        assert_eq!(cached[0], disabled_pair(DYNAMIC_REGION_BASE));
        assert_eq!(cached[1], disabled_pair(DYNAMIC_REGION_BASE + 1));
    }

    #[test]
    fn wire_boot_peripherals_shared_peripheral_cached_for_both() {
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        // Both partitions share the same peripheral at 0x4000_0000.
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        // Only 1 wired (deduped), but both partitions must have it cached.
        assert_eq!(ds.wire_boot_peripherals(&[pcb0, pcb1]), 1);

        let rasr = build_rasr(
            4096u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );
        let expected_r4 = (build_rbar(0x4000_0000, 4).unwrap(), rasr);
        let disabled_r5 = disabled_pair(DYNAMIC_REGION_BASE + 1);

        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0], expected_r4,
            "partition 0 R4 must hold shared peripheral"
        );
        assert_eq!(cached0[1], disabled_r5, "partition 0 R5 must be disabled");

        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0], expected_r4,
            "partition 1 R4 must hold shared peripheral"
        );
        assert_eq!(cached1[1], disabled_r5, "partition 1 R5 must be disabled");
    }

    #[test]
    fn wire_boot_peripherals_cache_pcb_consistency() {
        // Verifies that the debug_assertions consistency check inside
        // wire_boot_peripherals does not fire for well-formed inputs:
        // two partitions each with one valid peripheral region.
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        // Must not panic — cached descriptors match PCB data.
        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1]);
        assert_eq!(wired, 2);

        // Verify cached (RBAR, RASR) pairs encode correct base addresses.
        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0].0,
            build_rbar(0x4000_0000, 4).unwrap(),
            "partition 0 R4 RBAR must encode 0x4000_0000"
        );
        assert_ne!(cached0[0].1, 0, "partition 0 R4 must be enabled");

        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0].0,
            build_rbar(0x4001_0000, 4).unwrap(),
            "partition 1 R4 RBAR must encode 0x4001_0000"
        );
        assert_ne!(cached1[0].1, 0, "partition 1 R4 must be enabled");
    }

    #[test]
    #[should_panic(expected = "cache-PCB RBAR mismatch")]
    fn debug_verify_cache_consistency_detects_rbar_mismatch() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let pcb = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        // Wire normally so the cache is populated with correct values.
        ds.wire_boot_peripherals(std::slice::from_ref(&pcb));

        // Corrupt the cached RBAR for partition 0, descriptor 0:
        // overwrite with an RBAR encoding a different base address.
        // TODO(panic-free): unwrap acceptable in test-only code
        let bad_rbar = build_rbar(0x4001_0000, DYNAMIC_REGION_BASE as u32).unwrap();
        let good_rasr = ds.cached_peripheral_regions(pid(0))[0].1;
        ds.cache_peripherals(
            pid(0),
            [
                Some(WindowDescriptor {
                    base: 0x4001_0000,
                    size: 4096,
                    permissions: good_rasr,
                    owner: pid(0),
                    rbar: bad_rbar,
                }),
                None,
                None,
            ],
        );

        // Should panic: cached RBAR (0x4001_0000) != expected (0x4000_0000).
        ds.debug_verify_cache_consistency(&pcb);
    }

    #[test]
    #[should_panic(expected = "cache-PCB RASR must be non-zero")]
    fn debug_verify_cache_consistency_detects_rasr_zero() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let pcb = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        // Reserve 2 peripheral slots so wire_boot_peripherals populates cache.
        let sf = encode_size(4096).unwrap();
        let rasr = build_rasr(sf, AP_FULL_ACCESS, true, (true, true, false));
        let rbar = build_rbar(0x2000_0000, 6).unwrap();
        ds.configure_partition(pid(0), &[(rbar, rasr)], 2).unwrap();

        // Wire normally so the cache is populated with correct values.
        ds.wire_boot_peripherals(std::slice::from_ref(&pcb));

        // Corrupt the cached RASR to zero for partition 0, descriptor 0.
        // Keep the correct base so the RBAR check passes first.
        let good_rbar = ds.cached_peripheral_regions(pid(0))[0].0;
        ds.cache_peripherals(
            pid(0),
            [
                Some(WindowDescriptor {
                    base: 0x4000_0000,
                    size: 4096,
                    permissions: 0, // RASR = 0: should trigger the assertion
                    owner: pid(0),
                    rbar: good_rbar,
                }),
                None,
                None,
            ],
        );

        // Should panic: cached RASR is zero for a mappable peripheral.
        ds.debug_verify_cache_consistency(&pcb);
    }

    // ------------------------------------------------------------------
    // cached_peripheral_regions: (RBAR, RASR) from cached descriptors
    // ------------------------------------------------------------------

    /// Helper: build a device-memory RASR for the given size.
    fn periph_rasr(size: u32) -> u32 {
        build_rasr(
            encode_size(size).unwrap(),
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        )
    }

    /// Helper: build a peripheral WindowDescriptor.
    fn periph_desc(base: u32, size: u32, owner: PartitionId) -> WindowDescriptor {
        WindowDescriptor {
            base,
            size,
            permissions: periph_rasr(size),
            owner,
            rbar: 0,
        }
    }

    #[test]
    fn cached_peripheral_regions_one_cached() {
        let ds = DynamicStrategy::new();
        ds.cache_peripherals(
            pid(0),
            [Some(periph_desc(0x4000_0000, 4096, pid(0))), None, None],
        );
        let r = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            r[0],
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096))
        );
        assert_eq!(r[1], disabled_pair(DYNAMIC_REGION_BASE + 1));
    }

    #[test]
    fn cached_peripheral_regions_two_peripherals_both_enabled() {
        let ds = DynamicStrategy::new();
        ds.cache_peripherals(
            pid(2),
            [
                Some(periph_desc(0x4000_0000, 4096, pid(2))),
                Some(periph_desc(0x4001_0000, 256, pid(2))),
                None,
            ],
        );
        let r = ds.cached_peripheral_regions(pid(2));
        assert_eq!(
            r[0],
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096))
        );
        assert_eq!(
            r[1],
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256))
        );
        assert_ne!(r[0].1, 0);
        assert_ne!(r[1].1, 0);
    }

    #[test]
    fn cached_peripheral_regions_uncached_returns_disabled() {
        let ds = DynamicStrategy::new();
        let disabled = [
            disabled_pair(DYNAMIC_REGION_BASE),
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
        ];
        assert_eq!(ds.cached_peripheral_regions(pid(0)), disabled);
        assert_eq!(ds.cached_peripheral_regions(pid(255)), disabled);
    }

    /// Verify that the dynamic-mode cache path (`cached_peripheral_regions`)
    /// produces identical (RBAR, RASR) pairs to the static-mode reference
    /// path (`peripheral_mpu_regions_or_disabled`) for every partition.
    /// If these two ever diverge, dynamic-mode partitions get wrong MPU
    /// grants at context-switch time.
    #[test]
    fn cache_vs_pcb_peripheral_rbar_rasr_equivalence() {
        use crate::mpu::peripheral_mpu_regions_or_disabled;

        // Partition 0: UART0 at 0x4000_C000, 4 KiB.
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_C000, 4096)]);

        // Partition 1: GPIO at 0x4002_5000, 4 KiB.
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4002_5000, 4096)]);

        // Partition 2: no peripherals.
        let pcb_none = PartitionControlBlock::new(
            2,
            0x0,
            0x2001_0000,
            0x2001_1000,
            MpuRegion::new(0x2001_0000, 4096, 0),
        );

        let ds = DynamicStrategy::new();
        // Fully configure all partitions before wiring peripherals.
        // Reserve 2 peripheral slots (R4-R5) so configure_partition
        // places RAM in R6.
        let (rbar_p0, rasr_p0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_p0, rasr_p0)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        let (rbar_p2, rasr_p2) = data_region(0x2001_0000, 4096, 6);
        ds.configure_partition(pid(2), &[(rbar_p2, rasr_p2)], 2)
            .unwrap();

        let wired = ds.wire_boot_peripherals(&[pcb0.clone(), pcb1.clone(), pcb_none.clone()]);
        assert_eq!(wired, 2, "both peripherals must be wired");

        // --- Partition 0: one peripheral ---
        let cached_p0 = ds.cached_peripheral_regions(pid(0));
        let static_p0 = peripheral_mpu_regions_or_disabled(&pcb0);
        for i in 0..2 {
            assert_eq!(
                cached_p0[i],
                static_p0[i],
                "partition 0 R{} mismatch",
                4 + i
            );
        }

        // --- Partition 1: one peripheral ---
        let cached_p1 = ds.cached_peripheral_regions(pid(1));
        let static_p1 = peripheral_mpu_regions_or_disabled(&pcb1);
        for i in 0..2 {
            assert_eq!(
                cached_p1[i],
                static_p1[i],
                "partition 1 R{} mismatch",
                4 + i
            );
        }

        // --- Partition 2: no peripherals — both paths must yield disabled ---
        let cached_none = ds.cached_peripheral_regions(pid(2));
        let static_none = peripheral_mpu_regions_or_disabled(&pcb_none);
        for i in 0..2 {
            assert_eq!(
                cached_none[i],
                static_none[i],
                "no-periph R{} mismatch",
                4 + i
            );
        }
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals: MpuRegion.permissions is dead code
    // ------------------------------------------------------------------

    /// Verify RASR is computed once per region: slot RASR equals cached
    /// RASR, and deduped shared peripherals produce identical cache entries.
    #[test]
    fn wire_boot_peripherals_single_rasr_and_correct_slot() {
        let ds = DynamicStrategy::new();
        let (base, sz) = (0x4000_0000u32, 4096u32);
        let p = |id, db| {
            PartitionControlBlock::new(id, 0x0, db, db + 4096, MpuRegion::new(db, 4096, 0))
                .with_peripheral_regions(&[periph(base, sz)])
        };
        // peripheral_reserved=2 so wiring goes through reserved slots.
        ds.configure_partition(pid(0), &[data_region(0x2000_0000, 4096, 6)], 2)
            .unwrap();
        ds.configure_partition(pid(1), &[data_region(0x2000_8000, 4096, 6)], 2)
            .unwrap();
        assert_eq!(
            ds.wire_boot_peripherals(&[p(0, 0x2000_0000), p(1, 0x2000_8000)]),
            1
        );
        let expected = periph_rasr(sz);
        // Slot RASR must equal cached RASR (single computation path).
        let slot_rasr = with_cs(|cs| {
            ds.slots.borrow(cs).borrow()[0]
                .as_ref()
                .unwrap()
                .permissions
        });
        assert_eq!(slot_rasr, expected, "slot RASR mismatch");
        let c0 = ds.cached_peripheral_regions(pid(0));
        let c1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(c0[0].1, expected, "partition 0 cached RASR mismatch");
        assert_eq!(c1[0].1, expected, "partition 1 cached RASR mismatch");
        assert_eq!(c0[0], c1[0], "shared peripheral cache must match");
    }

    /// Prove `wire_boot_peripherals` produces identical cached RASR for
    /// partitions with different `MpuRegion.permissions` on the same
    /// peripheral base/size (Approach A intentional override).
    #[test]
    fn wire_boot_peripherals_permissions_independence() {
        let ds = DynamicStrategy::new();
        let (base, size) = (0x4000_0000u32, 4096u32);
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[MpuRegion::new(base, size, 0)]);
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[MpuRegion::new(base, size, 0xDEAD_BEEF)]);

        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        assert_eq!(ds.wire_boot_peripherals(&[pcb0, pcb1]), 1);

        let cached0 = ds.cached_peripheral_regions(pid(0));
        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached0[0].1, cached1[0].1,
            "RASR must match despite different MpuRegion.permissions"
        );

        // Verify fixed Shareable Device attributes.
        let rasr = cached0[0].1;
        assert_eq!(rasr & 1, 1, "region must be enabled");
        assert_eq!((rasr >> 24) & 0x7, AP_FULL_ACCESS, "AP must be full-access");
        assert_eq!((rasr >> 28) & 1, 1, "XN must be set");
        assert_eq!((rasr >> 19) & 0x7, 0, "TEX must be 0");
        assert_eq!((rasr >> 16) & 0x7, 0b101, "S/C/B must be 101");
    }

    #[test]
    fn wire_boot_peripherals_skips_add_window_for_cached_peripherals() {
        // 3 partitions, each with 1 unique peripheral, reserved=2.
        // Partitions 0 and 1 fill the two reserved slots (R4, R5) via
        // the `wired < reserved` branch.  Partition 2's peripheral hits
        // the NEW `desc_idx < reserved.min(MAX_PERIPHERAL_REGIONS)` branch: it is cached but
        // NOT wired, so `wired` stays at 2 and no dynamic window slot
        // is consumed.
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        let (rbar_p2, rasr_p2) = data_region(0x2001_0000, 4096, 6);
        ds.configure_partition(pid(2), &[(rbar_p2, rasr_p2)], 2)
            .unwrap();

        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        let pcb2 = PartitionControlBlock::new(
            2,
            0x0,
            0x2001_0000,
            0x2001_1000,
            MpuRegion::new(0x2001_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4002_0000, 4096)]);

        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1, pcb2]);
        assert_eq!(wired, 3, "all 3 unique peripherals wired to reserved slots");

        // R7 (slot 3) must be None — no dynamic window slot consumed.
        assert!(ds.slot(7).is_none(), "R7 must be empty after wiring");

        let rasr_4k = periph_rasr(4096);
        let rasr_256 = periph_rasr(256);

        // Partition 0: peripheral cached at R4 (slot 0, discovery order).
        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0],
            (build_rbar(0x4000_0000, 4).unwrap(), rasr_4k),
            "partition 0 cache[0] at R4"
        );
        assert_eq!(cached0[1], disabled_pair(DYNAMIC_REGION_BASE + 1));

        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0],
            (build_rbar(0x4001_0000, 4).unwrap(), rasr_256),
            "partition 1 cache[0] at R4"
        );
        assert_eq!(cached1[1], disabled_pair(DYNAMIC_REGION_BASE + 1));

        let cached2 = ds.cached_peripheral_regions(pid(2));
        assert_eq!(
            cached2[0],
            (build_rbar(0x4002_0000, 4).unwrap(), rasr_4k),
            "partition 2 cache[0] at R4 (via new cache-only branch)"
        );
        assert_eq!(cached2[1], disabled_pair(DYNAMIC_REGION_BASE + 1));

        // Dynamic window slots are still free.
        assert!(
            ds.add_window(0x2002_0000, 4096, rasr_4k, pid(0)).is_ok(),
            "add_window must succeed — no dynamic slots consumed"
        );
    }

    #[test]
    fn wire_boot_peripherals_reserved_zero_all_via_add_window() {
        // With peripheral_reserved=0 every peripheral is wired via
        // add_window into global dynamic slots.  No caching must occur
        // because cache[0] targets R4, which holds partition RAM.
        let ds = DynamicStrategy::new();
        let (rb, rs) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rb, rs)], 0).unwrap();

        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1]);
        assert_eq!(wired, 2, "both peripherals must be wired via add_window");

        // RAM occupies slot 0 (R4); add_window skips reserved+1=1 slot,
        // so peripherals land in slots 1 (R5) and 2 (R6).
        let s5 = ds.slot(5).expect("R5 must hold first peripheral");
        assert_eq!((s5.base, s5.size), (0x4000_0000, 4096));
        let s6 = ds.slot(6).expect("R6 must hold second peripheral");
        assert_eq!((s6.base, s6.size), (0x4001_0000, 256));
        assert!(ds.slot(7).is_none(), "R7 must be empty");

        // Verify device-memory RASR on both.
        assert_eq!(s5.permissions, periph_rasr(4096));
        assert_eq!(s6.permissions, periph_rasr(256));

        // When reserved=0, R4 holds partition RAM.  Caching peripherals
        // into cache[0] (which targets R4) would clobber RAM on context
        // switch, so the cache must remain disabled for both partitions.
        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0],
            disabled_pair(DYNAMIC_REGION_BASE),
            "partition 0 cache[0] must be disabled — R4 holds RAM"
        );
        assert_eq!(cached0[1], disabled_pair(DYNAMIC_REGION_BASE + 1));

        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0],
            disabled_pair(DYNAMIC_REGION_BASE),
            "partition 1 cache[0] must be disabled — R4 holds RAM"
        );
        assert_eq!(cached1[1], disabled_pair(DYNAMIC_REGION_BASE + 1));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "boot wiring consumed all dynamic window slots")]
    fn wire_boot_peripherals_three_peripherals_overflow_exhausts_slots() {
        // Single partition with 3 peripherals, reserved=2.
        // First 2 go to reserved slots R4/R5; 3rd overflows to add_window
        // which fills the last slot (R7), leaving none for runtime
        // add_window.  The debug_assert must fire.
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();

        let pcb = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[
            periph(0x4000_0000, 4096),
            periph(0x4001_0000, 4096),
            periph(0x4002_0000, 4096),
        ]);

        // Panics: all dynamic slots consumed, none left for buf_lend.
        let _wired = ds.wire_boot_peripherals(&[pcb]);
    }

    #[test]
    fn wire_boot_peripherals_reserved3_all_three_cached() {
        // Key acceptance criterion: a partition with reserved=3 and 3
        // peripheral regions must have all 3 programmed into the MPU cache.
        // R4-R6 are reserved for peripherals; data region goes to R7.
        //
        // With reserved=3 + 1 data region = 4 slots, all DYNAMIC_SLOT_COUNT
        // slots are occupied.  In debug mode, wire_boot_peripherals fires a
        // debug_assert warning that no slots remain for runtime add_window.
        // The cache is populated *before* that assert, so we catch the panic
        // and verify caching succeeded regardless.
        let ds = DynamicStrategy::new();
        let (rbar_r7, rasr_r7) = data_region(0x2000_0000, 4096, 7);
        ds.configure_partition(pid(0), &[(rbar_r7, rasr_r7)], 3)
            .unwrap();

        let pcb = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[
            periph(0x4000_C000, 4096), // e.g. UART0
            periph(0x4002_0000, 4096), // e.g. I2C0
            periph(0x4003_8000, 4096), // e.g. ADC0
        ]);

        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            ds.wire_boot_peripherals(&[pcb])
        }));

        // In release mode (no debug_assertions), wired count is returned directly.
        // In debug mode, the debug_assert may panic after caching succeeds.
        if let Ok(wired) = result {
            assert_eq!(wired, 3, "all 3 peripheral regions must be wired");
        }

        // Verify all 3 cached peripheral regions are present and enabled.
        let cached = ds.cached_peripheral_regions(pid(0));

        // Slot 0 (R4): first peripheral
        let expected_rbar0 = build_rbar(0x4000_C000, DYNAMIC_REGION_BASE as u32).unwrap();
        assert_eq!(
            cached[0].0, expected_rbar0,
            "cache[0] RBAR must point to UART0"
        );
        assert_ne!(cached[0].1, 0, "cache[0] RASR must be non-zero (enabled)");
        assert_eq!(cached[0].1, periph_rasr(4096));

        // Slot 1 (R5): second peripheral
        let expected_rbar1 = build_rbar(0x4002_0000, DYNAMIC_REGION_BASE as u32 + 1).unwrap();
        assert_eq!(
            cached[1].0, expected_rbar1,
            "cache[1] RBAR must point to I2C0"
        );
        assert_ne!(cached[1].1, 0, "cache[1] RASR must be non-zero (enabled)");
        assert_eq!(cached[1].1, periph_rasr(4096));

        // Slot 2 (R6): third peripheral
        let expected_rbar2 = build_rbar(0x4003_8000, DYNAMIC_REGION_BASE as u32 + 2).unwrap();
        assert_eq!(
            cached[2].0, expected_rbar2,
            "cache[2] RBAR must point to ADC0"
        );
        assert_ne!(cached[2].1, 0, "cache[2] RASR must be non-zero (enabled)");
        assert_eq!(cached[2].1, periph_rasr(4096));

        // Verify wired count: count occupied peripheral slots (indices 0-2).
        let peripheral_slots_occupied = (0..3)
            .filter(|&i| ds.slot(DYNAMIC_REGION_BASE + i as u8).is_some())
            .count();
        assert_eq!(
            peripheral_slots_occupied, 3,
            "all 3 peripheral slots must be occupied"
        );
    }

    // ------------------------------------------------------------------
    // try_wire_region tests
    // ------------------------------------------------------------------

    #[test]
    fn try_wire_region_two_way_branch() {
        let rasr = periph_rasr(4096);
        let r = |b| (b, 4096u32, rasr);
        let ds = DynamicStrategy::<4>::new();
        with_cs(|cs| ds.peripheral_reserved.borrow(cs).borrow_mut()[0] = 2);
        let mut seen: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        // Branch 1: reserved slot → delta=1, slot populated.
        assert_eq!(
            ds.try_wire_region(&mut seen, 0, 0, 2, r(0x4000_0000), pid(0)),
            Ok(1)
        );
        let s = ds.slot(4).expect("R4 occupied");
        assert_eq!((s.base, s.size, s.permissions), (0x4000_0000, 4096, rasr));
        // Reserved slot from a different partition (desc_idx=0 < reserved=2)
        // still wires with delta=1 (shared slot, context-switch restores).
        assert_eq!(
            ds.try_wire_region(&mut seen, 2, 0, 2, r(0x4001_0000), pid(1)),
            Ok(1)
        );
        // Branch 3: add_window fallback → delta=1.
        let ds2 = DynamicStrategy::<4>::new();
        let mut seen2: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        assert_eq!(
            ds2.try_wire_region(&mut seen2, 0, 0, 0, r(0x4000_0000), pid(0)),
            Ok(1)
        );
        // Exhaust remaining slots → SlotExhausted.
        for i in 1u32..3 {
            let b = 0x4000_0000 + i * 0x1_0000;
            assert!(ds2
                .try_wire_region(&mut seen2, i as usize, 0, 0, r(b), pid(0))
                .is_ok());
        }
        assert_eq!(
            ds2.try_wire_region(&mut seen2, 3, 0, 0, r(0x4003_0000), pid(0)),
            Err(MpuError::SlotExhausted)
        );
    }

    // ------------------------------------------------------------------
    // compute_peripheral_rasr tests
    // ------------------------------------------------------------------

    #[test]
    fn compute_peripheral_rasr_sizes() {
        // size=32 → trailing_zeros=5, size_field=4
        let rasr_32 = compute_peripheral_rasr(32);
        assert_eq!(rasr_32, periph_rasr(32));

        // size=256 → trailing_zeros=8, size_field=7
        let rasr_256 = compute_peripheral_rasr(256);
        assert_eq!(rasr_256, periph_rasr(256));

        // size=4096 → trailing_zeros=12, size_field=11
        let rasr_4096 = compute_peripheral_rasr(4096);
        assert_eq!(rasr_4096, periph_rasr(4096));
    }

    #[test]
    #[should_panic(expected = "MPU region size must be >= 32 bytes")]
    fn compute_peripheral_rasr_too_small() {
        // size=16 → trailing_zeros=4, which is < 5 → debug_assert fires.
        let _ = compute_peripheral_rasr(16);
    }

    #[test]
    fn build_partition_cache_entry_returns_some_when_cached() {
        let mut seen: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        let rasr = periph_rasr(4096);
        seen.push((0x4000_0000, 4096, 1, rasr)).unwrap();

        let result = DynamicStrategy::<4>::build_partition_cache_entry(
            &seen,
            0x4000_0000,
            4096,
            rasr,
            0,
            2,
            pid(5),
        );
        let wd = result.expect("expected Some for desc_idx < reserved with matching seen entry");
        assert_eq!(wd.base, 0x4000_0000);
        assert_eq!(wd.size, 4096);
        assert_eq!(wd.permissions, rasr);
        assert_eq!(wd.owner, pid(5));
        assert_eq!(wd.rbar, slot_rbar(0x4000_0000, 1));
    }

    #[test]
    fn build_partition_cache_entry_none_when_desc_idx_ge_reserved() {
        let mut seen: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        let rasr = periph_rasr(4096);
        seen.push((0x4000_0000, 4096, 0, rasr)).unwrap();

        // desc_idx == reserved → None
        let result = DynamicStrategy::<4>::build_partition_cache_entry(
            &seen,
            0x4000_0000,
            4096,
            rasr,
            2,
            2,
            pid(0),
        );
        assert!(result.is_none(), "expected None when desc_idx >= reserved");

        // reserved == 0 → None even at desc_idx 0
        let result = DynamicStrategy::<4>::build_partition_cache_entry(
            &seen,
            0x4000_0000,
            4096,
            rasr,
            0,
            0,
            pid(0),
        );
        assert!(result.is_none(), "expected None when reserved is 0");
    }

    #[test]
    fn wire_boot_peripherals_shared_peripheral_dedup_stress() {
        // 4 partitions each mapping the same 2 peripherals (same base+size).
        // With reserved=2, only 2 slots should be wired (dedup), all 4
        // partition caches must hold the correct (RBAR, RASR) pairs, and
        // no dynamic slots (R6/R7) should be consumed.
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rbar_r6, rasr_r6)], 2)
            .unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rbar_p1, rasr_p1)], 2)
            .unwrap();
        let (rbar_p2, rasr_p2) = data_region(0x2001_0000, 4096, 6);
        ds.configure_partition(pid(2), &[(rbar_p2, rasr_p2)], 2)
            .unwrap();
        let (rbar_p3, rasr_p3) = data_region(0x2001_8000, 4096, 6);
        ds.configure_partition(pid(3), &[(rbar_p3, rasr_p3)], 2)
            .unwrap();

        let shared_periphs = [periph(0x4000_0000, 4096), periph(0x4001_0000, 256)];

        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&shared_periphs);

        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&shared_periphs);

        let pcb2 = PartitionControlBlock::new(
            2,
            0x0,
            0x2001_0000,
            0x2001_1000,
            MpuRegion::new(0x2001_0000, 4096, 0),
        )
        .with_peripheral_regions(&shared_periphs);

        let pcb3 = PartitionControlBlock::new(
            3,
            0x0,
            0x2001_8000,
            0x2001_9000,
            MpuRegion::new(0x2001_8000, 4096, 0),
        )
        .with_peripheral_regions(&shared_periphs);

        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1, pcb2, pcb3]);
        assert_eq!(
            wired, 2,
            "dedup must collapse 8 mappings into 2 wired slots"
        );

        // Expected (RBAR, RASR) for each peripheral.
        let rasr_4k = periph_rasr(4096);
        let rasr_256 = periph_rasr(256);
        let expected_r4 = (build_rbar(0x4000_0000, 4).unwrap(), rasr_4k);
        let expected_r5 = (build_rbar(0x4001_0000, 5).unwrap(), rasr_256);

        // All 4 partition caches must hold the same pairs.
        for i in 0u32..4 {
            let cached = ds.cached_peripheral_regions(PartitionId::new(i));
            assert_eq!(
                cached[0], expected_r4,
                "partition {i} cache[0] must hold peripheral at 0x4000_0000"
            );
            assert_eq!(
                cached[1], expected_r5,
                "partition {i} cache[1] must hold peripheral at 0x4001_0000"
            );
        }

        // No dynamic window slots consumed by peripherals.
        // R6 (slot 2) holds the last-configured partition's RAM;
        // R7 (slot 3) must be empty.
        let r6 = ds.slot(6).expect("R6 must hold partition RAM");
        assert_eq!(
            r6.base, 0x2001_8000,
            "R6 must be partition RAM, not a peripheral"
        );
        assert!(
            ds.slot(7).is_none(),
            "R7 must be empty — no dynamic slot consumed"
        );
    }

    #[test]
    fn build_partition_cache_entry_none_when_unseen() {
        let seen: heapless::Vec<(u32, u32, usize, u32), 8> = heapless::Vec::new();
        let rasr = periph_rasr(4096);

        let result = DynamicStrategy::<4>::build_partition_cache_entry(
            &seen,
            0x4000_0000,
            4096,
            rasr,
            0,
            2,
            pid(0),
        );
        assert!(result.is_none(), "expected None when seen is empty");
    }

    // ------------------------------------------------------------------
    // DynamicStrategy<6>: N > DYNAMIC_SLOT_COUNT
    // ------------------------------------------------------------------

    #[test]
    fn dynamic_strategy_6_cache_all_partitions() {
        let ds = DynamicStrategy::<6>::with_partition_count();
        let disabled_r4 = disabled_pair(DYNAMIC_REGION_BASE);
        let disabled_r5 = disabled_pair(DYNAMIC_REGION_BASE + 1);
        let disabled_r6 = disabled_pair(DYNAMIC_REGION_BASE + 2);

        // Cache a peripheral for each of the 6 partitions.
        for pid in 0u32..6 {
            let base = 0x4000_0000 + pid * 0x1_0000;
            let descs = [
                Some(WindowDescriptor {
                    base,
                    size: 4096,
                    permissions: periph_rasr(4096),
                    owner: PartitionId::new(pid),
                    rbar: 0,
                }),
                None,
                None,
            ];
            ds.cache_peripherals(PartitionId::new(pid), descs);
        }

        // Verify each partition's cached entry.
        for pid in 0u32..6 {
            let base = 0x4000_0000 + pid * 0x1_0000;
            let got = ds.cached_peripheral_regions(PartitionId::new(pid));
            assert_eq!(
                got[0],
                (build_rbar(base, 4).unwrap(), periph_rasr(4096)),
                "partition {pid} R4 must hold its peripheral"
            );
            assert_eq!(got[1], disabled_r5, "partition {pid} R5 must be disabled");
            assert_eq!(got[2], disabled_r6, "partition {pid} R6 must be disabled");
        }

        // Specifically check partition_id=5 (highest valid index).
        let got5 = ds.cached_peripheral_regions(pid(5));
        assert_eq!(
            got5[0],
            (build_rbar(0x4005_0000, 4).unwrap(), periph_rasr(4096)),
            "partition 5 R4 must hold peripheral at 0x4005_0000"
        );
        assert_eq!(got5[1], disabled_r5);

        // Out-of-range: partition_id=6 must return disabled pairs.
        let got6 = ds.cached_peripheral_regions(pid(6));
        assert_eq!(
            got6[0], disabled_r4,
            "partition 6 (OOB) R4 must be disabled"
        );
        assert_eq!(
            got6[1], disabled_r5,
            "partition 6 (OOB) R5 must be disabled"
        );
    }

    #[test]
    fn dynamic_strategy_5_cache_peripherals_and_oob() {
        let ds = DynamicStrategy::<5>::with_partition_count();
        let rasr_4k = periph_rasr(4096);

        // Cache one peripheral per partition for 5 partitions.
        for pid in 0u32..5 {
            let base = 0x4000_0000 + pid * 0x1_0000;
            let descs = [
                Some(WindowDescriptor {
                    base,
                    size: 4096,
                    permissions: rasr_4k,
                    owner: PartitionId::new(pid),
                    rbar: 0,
                }),
                None,
                None,
            ];
            ds.cache_peripherals(PartitionId::new(pid), descs);
        }

        // Verify each partition's cached entry is correct.
        for pid in 0u32..5 {
            let expected_base = 0x4000_0000 + pid * 0x1_0000;
            let cached = ds.cached_peripheral_regions(PartitionId::new(pid));
            assert_eq!(
                cached[0],
                (build_rbar(expected_base, 4).unwrap(), rasr_4k),
                "partition {pid} cache[0] must hold its peripheral"
            );
            assert_eq!(
                cached[1],
                disabled_pair(DYNAMIC_REGION_BASE + 1),
                "partition {pid} cache[1] must be disabled"
            );
        }

        // Out-of-range: partition_id=5 with N=5 must return disabled.
        let oob = ds.cached_peripheral_regions(pid(5));
        assert_eq!(oob[0], disabled_pair(DYNAMIC_REGION_BASE));
        assert_eq!(oob[1], disabled_pair(DYNAMIC_REGION_BASE + 1));
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals: SlotExhausted early-return
    // ------------------------------------------------------------------

    #[test]
    fn wire_boot_peripherals_early_return_on_slot_exhaustion() {
        // reserved=0 (default): all peripherals go through add_window.
        // add_window skips slot 0 (partition-RAM) and scans slots 1-3,
        // giving 3 usable window slots.  With 5 partitions each owning
        // one unique peripheral, the 4th partition must trigger
        // SlotExhausted and the 5th must never be processed.
        let ds = DynamicStrategy::<5>::with_partition_count();

        let pcbs: heapless::Vec<PartitionControlBlock, 5> = (0u8..5)
            .map(|pid| {
                let ram_base = 0x2000_0000 + u32::from(pid) * 0x1_0000;
                let periph_base = 0x4000_0000 + u32::from(pid) * 0x1_0000;
                PartitionControlBlock::new(
                    pid,
                    0x0,
                    ram_base,
                    ram_base + 0x1000,
                    MpuRegion::new(ram_base, 4096, 0),
                )
                .with_peripheral_regions(&[periph(periph_base, 4096)])
            })
            .collect();

        let wired = ds.wire_boot_peripherals(&pcbs);

        // (1) Wired count equals the 3 available dynamic window slots.
        assert_eq!(wired, 3, "must wire exactly 3 peripherals (slots 1-3)");

        // (2) Partition 3 triggered exhaustion; verify its cache was
        //     flushed (disabled pairs, since reserved=0 → no cache
        //     entries are built, but cache_peripherals must still run).
        let disabled_r4 = disabled_pair(DYNAMIC_REGION_BASE);
        let disabled_r5 = disabled_pair(DYNAMIC_REGION_BASE + 1);
        let cache3 = ds.cached_peripheral_regions(pid(3));
        assert_eq!(
            cache3[0], disabled_r4,
            "partition 3 cache[0] must be disabled (flushed on early return)"
        );
        assert_eq!(
            cache3[1], disabled_r5,
            "partition 3 cache[1] must be disabled (flushed on early return)"
        );

        // (3) Remaining partition (4) was never processed — cache is
        //     initial disabled state (cache_peripherals was never called).
        let cache4 = ds.cached_peripheral_regions(pid(4));
        assert_eq!(
            cache4[0], disabled_r4,
            "partition 4 cache[0] must be initial"
        );
        assert_eq!(
            cache4[1], disabled_r5,
            "partition 4 cache[1] must be initial"
        );

        // Verify the 3 wired peripherals occupy slots 1-3 (R5-R7).
        for i in 0u8..3 {
            let region_id = DYNAMIC_REGION_BASE + 1 + i; // R5, R6, R7
            let desc = ds
                .slot(region_id)
                .unwrap_or_else(|| panic!("R{} must be occupied", region_id));
            let expected_base = 0x4000_0000 + u32::from(i) * 0x1_0000;
            assert_eq!(
                desc.base, expected_base,
                "R{region_id} must hold peripheral at {expected_base:#x}"
            );
        }

        // Partitions 3 & 4 peripherals must not appear in any slot.
        for rid in 4..8u8 {
            if let Some(d) = ds.slot(rid) {
                assert!(
                    d.base != 0x4003_0000 && d.base != 0x4004_0000,
                    "unwired peripheral found in R{rid}"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals: per-partition peripheral_reserved isolation
    // ------------------------------------------------------------------

    #[test]
    fn wire_boot_peripherals_respects_per_partition_reserved() {
        // Partition 0: reserved=2, 1 peripheral  → cached in slot 0, slot 1 disabled
        // Partition 1: reserved=0, 1 peripheral  → wired via add_window, cache all-disabled
        //
        // Using different reserved counts (2 vs 0) proves the strategy reads
        // the per-partition value from peripheral_reserved_for(), not a global.
        let ds = DynamicStrategy::<2>::with_partition_count();

        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();

        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1]);
        // Partition 0: wired=0 < reserved=2 → slot 0, delta=1, wired→1.
        // Partition 1: wired=1, reserved=0 → not reserved, desc_idx=0 >= 0
        //   → falls through to add_window → slot 1, delta=1, wired→2.
        assert_eq!(wired, 2, "1 reserved-slot wire + 1 add_window wire");

        // Expected RASR values for each peripheral size.
        let rasr_4k = build_rasr(
            4096u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );

        // Partition 0 (reserved=2): peripheral cached in slot 0, slot 1 disabled.
        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0],
            (
                build_rbar(0x4000_0000, DYNAMIC_REGION_BASE as u32).unwrap(),
                rasr_4k
            ),
            "partition 0 cache[0] must hold the 4KiB peripheral"
        );
        assert_eq!(
            cached0[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 0 cache[1] must be disabled (only 1 peripheral)"
        );

        // Partition 1 (reserved=0): no cache slots available, both disabled.
        // The peripheral was wired via add_window but is not cached because
        // reserved=0 means build_partition_cache_entry returns None.
        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0],
            disabled_pair(DYNAMIC_REGION_BASE),
            "partition 1 cache[0] must be disabled (reserved=0)"
        );
        assert_eq!(
            cached1[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 1 cache[1] must be disabled (reserved=0)"
        );

        // Cross-check: the two partitions' caches must differ — partition 0
        // has an active peripheral in slot 0, partition 1 does not.
        assert_ne!(
            cached0[0], cached1[0],
            "partition 0 cache[0] (active) must differ from partition 1 cache[0] (disabled)"
        );
    }

    #[test]
    fn wire_boot_peripherals_three_partitions_mixed_reserved() {
        // Partition 0: reserved=2, 1 peripheral (4KiB)  → cached in slot 0
        // Partition 1: reserved=0, 1 peripheral (256B)  → wired via add_window
        // Partition 2: reserved=2, 1 peripheral (1KiB)  → cached in own cache
        //
        // Extends the 2-partition test to 3 partitions, proving that
        // per-partition reserved counts are independent across partitions.
        let ds = DynamicStrategy::<3>::with_partition_count();

        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();
        let (rb2, rs2) = data_region(0x2001_0000, 4096, 6);
        ds.configure_partition(pid(2), &[(rb2, rs2)], 2).unwrap();

        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096)]);

        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4001_0000, 256)]);

        let pcb2 = PartitionControlBlock::new(
            2,
            0x0,
            0x2001_0000,
            0x2001_1000,
            MpuRegion::new(0x2001_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4002_0000, 1024)]);

        let wired = ds.wire_boot_peripherals(&[pcb0, pcb1, pcb2]);
        // P0: desc_idx=0 < reserved=2 → reserved slot 0, delta=1, wired→1
        // P1: reserved=0 → add_window (slot 3), delta=1, wired→2
        // P2: desc_idx=0 < reserved=2 → reserved slot 0 (shared), delta=1, wired→3
        assert_eq!(
            wired, 3,
            "each partition's peripheral is wired independently"
        );

        // Expected RASR for each peripheral size.
        let rasr_4k = build_rasr(
            4096u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );
        let rasr_1k = build_rasr(
            1024u32.trailing_zeros() - 1,
            AP_FULL_ACCESS,
            true,
            (true, false, true),
        );

        // Partition 0 (reserved=2): peripheral cached in slot 0, slot 1 disabled.
        let cached0 = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached0[0],
            (
                build_rbar(0x4000_0000, DYNAMIC_REGION_BASE as u32).unwrap(),
                rasr_4k
            ),
            "partition 0 cache[0] must hold the 4KiB peripheral"
        );
        assert_eq!(
            cached0[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 0 cache[1] must be disabled"
        );

        // Partition 1 (reserved=0): all cache slots disabled.
        let cached1 = ds.cached_peripheral_regions(pid(1));
        assert_eq!(
            cached1[0],
            disabled_pair(DYNAMIC_REGION_BASE),
            "partition 1 cache[0] must be disabled (reserved=0)"
        );
        assert_eq!(
            cached1[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 1 cache[1] must be disabled (reserved=0)"
        );

        // Partition 2 (reserved=2): 1KiB peripheral cached in slot 0, slot 1 disabled.
        let cached2 = ds.cached_peripheral_regions(pid(2));
        assert_eq!(
            cached2[0],
            (
                build_rbar(0x4002_0000, DYNAMIC_REGION_BASE as u32).unwrap(),
                rasr_1k
            ),
            "partition 2 cache[0] must hold the 1KiB peripheral"
        );
        assert_eq!(
            cached2[1],
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            "partition 2 cache[1] must be disabled"
        );

        // Cross-check: partitions 0 and 2 have different cached peripherals.
        assert_ne!(
            cached0[0], cached2[0],
            "partition 0 and 2 must have different cached peripherals"
        );
    }

    // -- partition_region_values ----------------------------------------

    #[test]
    fn partition_region_values_homogeneous_reserved0() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 4);
        let (rb1, rs1) = data_region(0x2000_1000, 4096, 4);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 0).unwrap();
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();
        let v0 = ds.partition_region_values(pid(0));
        assert_eq!(v0[0], (build_rbar(0x2000_0000, 4).unwrap(), rs0));
        assert_eq!(v0[1].1, 0);
        assert_eq!(v0[2].1, 0);
        assert_eq!(v0[3].1, 0);
        let v1 = ds.partition_region_values(pid(1));
        assert_eq!(v1[0], (build_rbar(0x2000_1000, 4).unwrap(), rs1));
    }

    #[test]
    fn partition_region_values_homogeneous_reserved2() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        let (rb1, rs1) = data_region(0x2000_1000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        ds.configure_partition(pid(1), &[(rb1, rs1)], 2).unwrap();
        let v0 = ds.partition_region_values(pid(0));
        assert_eq!(v0[0], disabled_pair(4));
        assert_eq!(v0[1], disabled_pair(5));
        assert_eq!(v0[2], (build_rbar(0x2000_0000, 6).unwrap(), rs0));
        assert_eq!(v0[3].1, 0);
        let v1 = ds.partition_region_values(pid(1));
        assert_eq!(v1[2], (build_rbar(0x2000_1000, 6).unwrap(), rs1));
    }

    #[test]
    fn partition_region_values_heterogeneous() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        let (rb1, rs1) = data_region(0x2000_1000, 4096, 4);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();
        let v1 = ds.partition_region_values(pid(1));
        assert_eq!(v1[0], (build_rbar(0x2000_1000, 4).unwrap(), rs1));
        assert_eq!(v1[2], disabled_pair(6), "P0 RAM disabled for P1");
        let v0 = ds.partition_region_values(pid(0));
        assert_eq!(v0[2], (build_rbar(0x2000_0000, 6).unwrap(), rs0));
        assert_eq!(v0[0], disabled_pair(4), "R4 from peripheral cache");
    }

    #[test]
    fn partition_region_values_preserves_dynamic_window() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        let (rb1, rs1) = data_region(0x2000_1000, 4096, 4);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 0).unwrap();
        let sf = encode_size(256).unwrap();
        let win_rasr = build_rasr(sf, AP_RO_RO, true, (false, false, false));
        ds.add_window(0x2000_2000, 256, win_rasr, pid(0)).unwrap();
        let v1 = ds.partition_region_values(pid(1));
        assert_eq!(
            v1[3],
            (build_rbar(0x2000_2000, 7).unwrap(), win_rasr),
            "dynamic window from P0 visible to P1"
        );
    }

    #[test]
    fn partition_region_values_no_peripheral_slot_leakage() {
        // P0 has reserved=2 with two peripherals wired into slots 0-1.
        // P1 has reserved=0 (RAM at slot 0). P1 must NOT see P0's
        // peripheral descriptor at slot 1 (R5) — the cross-partition
        // filter must disable peripheral-reserved slots, not just RAM.
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        let (rb1, _rs1) = data_region(0x2000_8000, 4096, 4);
        ds.configure_partition(pid(1), &[(rb1, _rs1)], 0).unwrap();

        // Wire two peripherals for P0 into reserved slots 0 and 1.
        let pcb0 = PartitionControlBlock::new(
            0,
            0x0,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 4096, 0),
        )
        .with_peripheral_regions(&[periph(0x4000_0000, 4096), periph(0x4001_0000, 256)]);
        let pcb1 = PartitionControlBlock::new(
            1,
            0x0,
            0x2000_8000,
            0x2000_9000,
            MpuRegion::new(0x2000_8000, 4096, 0),
        );
        ds.wire_boot_peripherals(&[pcb0, pcb1]);

        let v1 = ds.partition_region_values(pid(1));
        // Slot 1 (R5): P0's peripheral — must be disabled for P1.
        // Before the fix, idx(1) == owner_res(2) was false, leaking
        // P0's peripheral RASR to P1. With idx <= owner_res, it is
        // correctly disabled.
        assert_eq!(
            v1[1].1, 0,
            "P0 peripheral slot 1 must be disabled for P1 (RASR=0)"
        );
        // Slot 2 (R6): P0's RAM — must be disabled for P1.
        assert_eq!(v1[2], disabled_pair(6), "P0 RAM disabled for P1");
    }

    #[test]
    fn partition_region_values_unconfigured_returns_disabled() {
        // Partition 0 is never configured: peripheral_reserved defaults to 0,
        // partition_ram is None. All RASR values must be 0 (disabled).
        let ds = DynamicStrategy::<2>::with_partition_count();
        let vals = ds.partition_region_values(pid(0));
        for (i, &(_rbar, rasr)) in vals.iter().enumerate() {
            assert_eq!(
                rasr, 0,
                "slot {} RASR must be 0 for unconfigured partition",
                i
            );
        }
    }

    /// Exercises the full round-trip of reconfiguring a single partition's
    /// `peripheral_reserved` from 2→0→2, verifying that stale RAM
    /// descriptors are cleared by the owner check at each transition and
    /// that `partition_region_values` returns the correct layout.
    #[test]
    fn reconfigure_partition_peripheral_reserved_round_trip() {
        let ds = DynamicStrategy::<1>::with_partition_count();

        // --- Phase 1: reserved=2, RAM lands in slot 2 (R6) ---
        let (rb_r6, rs_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb_r6, rs_r6)], 2)
            .unwrap();

        assert_eq!(ds.peripheral_reserved_for(pid(0)), 2);
        let s2 = ds.slot(6).expect("RAM in R6 after reserved=2");
        assert_eq!(s2.owner, pid(0));
        assert_eq!(s2.base, 0x2000_0000);

        let v = ds.partition_region_values(pid(0));
        // Slots 0-1 (R4-R5): peripheral cache (unconfigured → disabled).
        assert_eq!(v[0], disabled_pair(4), "R4 disabled (no peripheral)");
        assert_eq!(v[1], disabled_pair(5), "R5 disabled (no peripheral)");
        // Slot 2 (R6): partition RAM.
        assert_ne!(v[2].1, 0, "R6 RASR enabled for RAM");
        // Slot 3 (R7): unused window.
        assert_eq!(v[3], disabled_pair(7), "R7 disabled (unused)");

        // --- Phase 2: reconfigure to reserved=0, RAM moves to slot 0 (R4) ---
        let (rb_r4, rs_r4) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(pid(0), &[(rb_r4, rs_r4)], 0)
            .unwrap();

        assert_eq!(ds.peripheral_reserved_for(pid(0)), 0);
        // Old slot 2 (R6) must have been cleared by the owner check.
        assert!(ds.slot(6).is_none(), "stale R6 cleared after 2→0");
        // New slot 0 (R4) holds the RAM.
        let s0 = ds.slot(4).expect("RAM in R4 after reserved=0");
        assert_eq!(s0.owner, pid(0));
        assert_eq!(s0.base, 0x2000_0000);

        let v = ds.partition_region_values(pid(0));
        // Slot 0 (R4): partition RAM.
        assert_ne!(v[0].1, 0, "R4 RASR enabled for RAM");
        // Slots 1-3 (R5-R7): all disabled.
        assert_eq!(v[1], disabled_pair(5), "R5 disabled");
        assert_eq!(v[2], disabled_pair(6), "R6 disabled (stale cleared)");
        assert_eq!(v[3], disabled_pair(7), "R7 disabled");

        // --- Phase 3: reconfigure back to reserved=2, RAM returns to slot 2 (R6) ---
        let (rb_r6b, rs_r6b) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb_r6b, rs_r6b)], 2)
            .unwrap();

        assert_eq!(ds.peripheral_reserved_for(pid(0)), 2);
        // Old slot 0 (R4) must have been cleared.
        assert!(ds.slot(4).is_none(), "stale R4 cleared after 0→2");
        // Slot 2 (R6) holds the RAM again.
        let s2b = ds.slot(6).expect("RAM in R6 after reserved=2 again");
        assert_eq!(s2b.owner, pid(0));
        assert_eq!(s2b.base, 0x2000_0000);

        let v = ds.partition_region_values(pid(0));
        // Slots 0-1: peripheral cache (still unconfigured → disabled).
        assert_eq!(v[0], disabled_pair(4), "R4 disabled (peripheral slot)");
        assert_eq!(v[1], disabled_pair(5), "R5 disabled (peripheral slot)");
        // Slot 2 (R6): partition RAM.
        assert_ne!(v[2].1, 0, "R6 RASR enabled for RAM after round-trip");
        // Slot 3 (R7): unused.
        assert_eq!(v[3], disabled_pair(7), "R7 disabled");
    }

    #[test]
    fn partition_region_values_out_of_range_returns_disabled() {
        // partition_id beyond the N=2 partition array: every slot must be disabled.
        let ds = DynamicStrategy::<2>::with_partition_count();
        let vals = ds.partition_region_values(pid(255));
        for (i, &(_rbar, rasr)) in vals.iter().enumerate() {
            assert_eq!(
                rasr, 0,
                "slot {} RASR must be 0 for out-of-range partition_id",
                i
            );
        }
    }

    // ------------------------------------------------------------------
    // 3-entry peripheral cache tests
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_cache_stores_and_retrieves_three_pairs() {
        let ds = DynamicStrategy::<2>::with_partition_count();
        let descs = [
            Some(periph_desc(0x4000_0000, 4096, pid(0))),
            Some(periph_desc(0x4001_0000, 256, pid(0))),
            Some(periph_desc(0x4002_0000, 1024, pid(0))),
        ];
        ds.cache_peripherals(pid(0), descs);
        let got = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            got[0],
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096)),
            "cache[0] must hold 4KiB peripheral at 0x4000_0000"
        );
        assert_eq!(
            got[1],
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256)),
            "cache[1] must hold 256B peripheral at 0x4001_0000"
        );
        assert_eq!(
            got[2],
            (build_rbar(0x4002_0000, 6).unwrap(), periph_rasr(1024)),
            "cache[2] must hold 1KiB peripheral at 0x4002_0000"
        );
    }

    #[test]
    fn cached_peripheral_regions_returns_three_entries_for_reserved_3() {
        // Partition with reserved=3: directly populate cache via
        // cache_peripherals and verify all 3 entries are retrievable.
        let ds = DynamicStrategy::<2>::with_partition_count();
        let (rb, rs) = data_region(0x2000_0000, 4096, 7);
        ds.configure_partition(pid(0), &[(rb, rs)], 3).unwrap();
        assert_eq!(ds.peripheral_reserved_for(pid(0)), 3);

        let descs = [
            Some(periph_desc(0x4000_0000, 4096, pid(0))),
            Some(periph_desc(0x4001_0000, 256, pid(0))),
            Some(periph_desc(0x4002_0000, 1024, pid(0))),
        ];
        ds.cache_peripherals(pid(0), descs);

        let cached = ds.cached_peripheral_regions(pid(0));
        assert_eq!(
            cached[0],
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096)),
            "cache[0] must hold 4KiB peripheral"
        );
        assert_eq!(
            cached[1],
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256)),
            "cache[1] must hold 256B peripheral"
        );
        assert_eq!(
            cached[2],
            (build_rbar(0x4002_0000, 6).unwrap(), periph_rasr(1024)),
            "cache[2] must hold 1KiB peripheral"
        );
        for (i, &(_rbar, rasr)) in cached.iter().enumerate() {
            assert_ne!(rasr, 0, "cache[{i}] RASR must be non-zero");
        }
    }

    // -- asymmetric peripheral counts (2 vs 3) ----------------------------

    /// Helper: set up a 2-partition DynamicStrategy with P0(reserved=2)
    /// and P1(reserved=3), populate peripheral caches directly, and
    /// return the strategy.
    fn setup_asymmetric_2v3() -> DynamicStrategy<2> {
        let ds = DynamicStrategy::<2>::with_partition_count();
        // P0: reserved=2, RAM at slot 2 (R6)
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(pid(0), &[(rb0, rs0)], 2).unwrap();
        // P1: reserved=3, RAM at slot 3 (R7)
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 7);
        ds.configure_partition(pid(1), &[(rb1, rs1)], 3).unwrap();

        // Populate P0 cache: 2 peripherals + 1 disabled slot.
        ds.cache_peripherals(
            pid(0),
            [
                Some(periph_desc(0x4000_0000, 4096, pid(0))),
                Some(periph_desc(0x4001_0000, 256, pid(0))),
                None,
            ],
        );
        // Populate P1 cache: 3 peripherals.
        ds.cache_peripherals(
            pid(1),
            [
                Some(periph_desc(0x4002_0000, 4096, pid(1))),
                Some(periph_desc(0x4003_0000, 256, pid(1))),
                Some(periph_desc(0x4004_0000, 1024, pid(1))),
            ],
        );
        ds
    }

    #[test]
    fn asymmetric_p0_reserved2_has_ram_at_r6_and_r7_disabled() {
        let ds = setup_asymmetric_2v3();
        let v0 = ds.partition_region_values(pid(0));

        // R4 (slot 0): P0 peripheral from cache (4KiB @ 0x4000_0000)
        assert_ne!(v0[0].1, 0, "R4 must be active peripheral for P0");
        assert_eq!(
            v0[0].0,
            build_rbar(0x4000_0000, 4).unwrap(),
            "R4 RBAR must point to P0 peripheral 0"
        );

        // R5 (slot 1): P0 peripheral from cache (256B @ 0x4001_0000)
        assert_ne!(v0[1].1, 0, "R5 must be active peripheral for P0");
        assert_eq!(
            v0[1].0,
            build_rbar(0x4001_0000, 5).unwrap(),
            "R5 RBAR must point to P0 peripheral 1"
        );

        // R6 (slot 2): P0 RAM — not a stale peripheral
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        assert_eq!(v0[2], (rb0, rs0), "R6 must be P0 RAM, not stale peripheral");

        // R7 (slot 3): disabled (RASR=0)
        assert_eq!(
            v0[3].1, 0,
            "R7 must be disabled for P0 (unused peripheral slot)"
        );
    }

    #[test]
    fn asymmetric_p1_reserved3_has_peripherals_r4_r6_and_ram_r7() {
        let ds = setup_asymmetric_2v3();
        let v1 = ds.partition_region_values(pid(1));

        // R4 (slot 0): P1 peripheral from cache (4KiB @ 0x4002_0000)
        assert_eq!(
            v1[0].0,
            build_rbar(0x4002_0000, 4).unwrap(),
            "R4 RBAR must point to P1 peripheral 0"
        );
        assert_eq!(
            v1[0].1,
            periph_rasr(4096),
            "R4 RASR must be 4KiB peripheral"
        );

        // R5 (slot 1): P1 peripheral from cache (256B @ 0x4003_0000)
        assert_eq!(
            v1[1].0,
            build_rbar(0x4003_0000, 5).unwrap(),
            "R5 RBAR must point to P1 peripheral 1"
        );
        assert_eq!(v1[1].1, periph_rasr(256), "R5 RASR must be 256B peripheral");

        // R6 (slot 2): P1 peripheral from cache (1KiB @ 0x4004_0000)
        assert_eq!(
            v1[2].0,
            build_rbar(0x4004_0000, 6).unwrap(),
            "R6 RBAR must point to P1 peripheral 2"
        );
        assert_eq!(
            v1[2].1,
            periph_rasr(1024),
            "R6 RASR must be 1KiB peripheral"
        );

        // R7 (slot 3): P1 RAM
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 7);
        assert_eq!(v1[3], (rb1, rs1), "R7 must be P1 RAM");
    }

    #[test]
    fn asymmetric_no_stale_rasr_between_partitions() {
        // Verify that alternating partition_region_values calls produce
        // no stale (non-zero RASR) peripheral entries in unused slots.
        let ds = setup_asymmetric_2v3();

        // Simulate switching: compute P1 values first, then P0.
        let _v1 = ds.partition_region_values(pid(1));
        let v0 = ds.partition_region_values(pid(0));

        // P0 has reserved=2: slots 0,1 are peripherals, slot 2 is RAM.
        // Slot 3 (R7) must be disabled — no stale P1 peripheral bits.
        assert_eq!(
            v0[3].1, 0,
            "R7 RASR must be 0 for P0 after P1 was active (no stale leak)"
        );
        // Also verify slot 2 is P0's RAM, not P1's third peripheral.
        let (rb0, rs0) = data_region(0x2000_0000, 4096, 6);
        assert_eq!(
            v0[2],
            (rb0, rs0),
            "R6 must be P0 RAM, not P1 stale peripheral"
        );

        // Switch back to P1 and verify P1 is clean too.
        let v1 = ds.partition_region_values(pid(1));
        for (i, &(_rbar, rasr)) in v1[0..3].iter().enumerate() {
            assert_ne!(
                rasr, 0,
                "P1 slot {i} RASR must be non-zero (active peripheral)"
            );
        }
        let (rb1, rs1) = data_region(0x2000_8000, 4096, 7);
        assert_eq!(v1[3], (rb1, rs1), "R7 must be P1 RAM after switch-back");
    }

    // ------------------------------------------------------------------
    // debug_assert_no_stale_regions
    // ------------------------------------------------------------------

    #[test]
    fn debug_assert_correct_config_passes_silently() {
        // P0 reserved=2: slots 0,1 are peripherals, slot 2 is RAM, slot 3 disabled.
        let cache = [
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096)),
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256)),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
        ];
        let (rb, rs) = data_region(0x2000_0000, 4096, 6);
        let out = [
            cache[0],
            cache[1],
            (rb, rs), // RAM at slot 2
            disabled_pair(DYNAMIC_REGION_BASE + 3),
        ];
        let slots: [Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT] = [None, None, None, None];
        debug_assert_no_stale_regions(&out, pid(0), 2, &cache, &slots);
    }

    #[test]
    #[should_panic(expected = "stale MPU: slot 3")]
    fn debug_assert_catches_stale_peripheral_beyond_reserved() {
        // P0 reserved=2 but slot 3 has a non-zero RASR with no window
        // descriptor — this is a stale peripheral leak.
        let cache = [
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096)),
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256)),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
        ];
        let (rb, rs) = data_region(0x2000_0000, 4096, 6);
        let stale_rasr = periph_rasr(1024);
        let out = [
            cache[0],
            cache[1],
            (rb, rs),                             // RAM at slot 2
            (0x4002_0000 | (1 << 4), stale_rasr), // stale at slot 3
        ];
        let slots: [Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT] = [None, None, None, None];
        debug_assert_no_stale_regions(&out, pid(0), 2, &cache, &slots);
    }

    #[test]
    #[should_panic(expected = "stale MPU: slot 0")]
    fn debug_assert_catches_zeroed_cached_peripheral() {
        // P0 reserved=2 but slot 0 has RASR=0 despite a cached peripheral.
        let cache = [
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096)),
            (build_rbar(0x4001_0000, 5).unwrap(), periph_rasr(256)),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
        ];
        let (rb, rs) = data_region(0x2000_0000, 4096, 6);
        let out = [
            disabled_pair(DYNAMIC_REGION_BASE), // stale: should have periph
            cache[1],
            (rb, rs),
            disabled_pair(DYNAMIC_REGION_BASE + 3),
        ];
        let slots: [Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT] = [None, None, None, None];
        debug_assert_no_stale_regions(&out, pid(0), 2, &cache, &slots);
    }

    #[test]
    fn debug_assert_shared_window_beyond_reserved_is_ok() {
        // P1 reserved=0, RAM at slot 0, slot 3 has a shared window from P0.
        let cache = [
            disabled_pair(DYNAMIC_REGION_BASE),
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
        ];
        let (rb, rs) = data_region(0x2000_0000, 4096, 4);
        let win_rasr = build_rasr(
            encode_size(256).unwrap(),
            AP_RO_RO,
            true,
            (false, false, false),
        );
        let out = [
            (rb, rs), // RAM at slot 0
            disabled_pair(DYNAMIC_REGION_BASE + 1),
            disabled_pair(DYNAMIC_REGION_BASE + 2),
            (build_rbar(0x2000_2000, 7).unwrap(), win_rasr),
        ];
        let mut slots: [Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT] = [None, None, None, None];
        slots[3] = Some(WindowDescriptor {
            base: 0x2000_2000,
            size: 256,
            permissions: win_rasr,
            owner: pid(0),
            rbar: build_rbar(0x2000_2000, 7).unwrap(),
        });
        // Must not panic — shared window is legitimate.
        debug_assert_no_stale_regions(&out, pid(1), 0, &cache, &slots);
    }
}
