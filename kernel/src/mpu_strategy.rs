//! MPU strategy abstraction for dynamic partition windowing.
//!
//! This module defines the [`MpuStrategy`] trait, allowing the kernel to
//! swap between static (compile-time) and dynamic (runtime) MPU region
//! management.  [`StaticStrategy`] delegates directly to the existing
//! [`crate::mpu`] helpers with no behaviour change.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

pub use crate::mpu::MpuError;

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
    /// and `compute_region_values` emits disabled entries for the reserved
    /// slots so PendSV can overwrite them with per-partition peripherals.
    fn configure_partition(
        &self,
        partition_id: u8,
        regions: &[(u32, u32)],
        peripheral_reserved: usize,
    ) -> Result<(), MpuError>;

    /// Dynamically add a temporary memory window.
    ///
    /// `owner` identifies the partition that owns the window.
    /// Returns the MPU region ID on success, or an [`MpuError`] describing
    /// the failure (e.g. `SlotExhausted` when no free region is available).
    fn add_window(&self, base: u32, size: u32, permissions: u32, owner: u8)
        -> Result<u8, MpuError>;

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
    pub owner: u8,
}

/// Number of dynamic region slots (R4 through R7).
const DYNAMIC_SLOT_COUNT: usize = 4;

/// First hardware MPU region number used by the dynamic strategy.
const DYNAMIC_REGION_BASE: u8 = 4;

/// Per-partition peripheral descriptor cache (up to 2 descriptors per partition).
type PeripheralCache = [Option<[Option<WindowDescriptor>; 2]>; DYNAMIC_SLOT_COUNT];

/// Dynamic MPU strategy — manages regions R4-R7 at runtime.
///
/// Slot 0 (R4) holds the partition's private RAM. Slots 1-3 (R5-R7)
/// are ad-hoc windows. Pure data-structure tracker — no hardware writes.
///
/// Interior mutability is provided by `Mutex<RefCell<…>>`, which uses a
/// critical section (interrupt-disable) on single-core Cortex-M to ensure
/// exclusive access.
pub struct DynamicStrategy {
    slots: Mutex<RefCell<[Option<WindowDescriptor>; DYNAMIC_SLOT_COUNT]>>,
    /// Number of leading slots (0 or 2) reserved for peripheral MMIO regions.
    /// Set by `configure_partition`; `wire_boot_peripherals` populates these
    /// slots with peripheral descriptors so `compute_region_values` emits
    /// them during PendSV context switches.
    peripheral_reserved: Mutex<RefCell<usize>>,
    /// Per-partition cache of peripheral descriptors (up to 2 per partition).
    /// Indexed by `partition_id`; avoids re-wiring peripherals on every
    /// context switch.
    peripheral_cache: Mutex<RefCell<PeripheralCache>>,
}

impl Default for DynamicStrategy {
    fn default() -> Self {
        Self::new()
    }
}

impl DynamicStrategy {
    /// Create a new strategy with all slots empty.
    pub const fn new() -> Self {
        Self {
            slots: Mutex::new(RefCell::new([None; DYNAMIC_SLOT_COUNT])),
            peripheral_reserved: Mutex::new(RefCell::new(0)),
            peripheral_cache: Mutex::new(RefCell::new([None; DYNAMIC_SLOT_COUNT])),
        }
    }

    /// Convert a hardware MPU region ID (4-7) to a `compute_region_values`
    /// array index (0-3), or `None` if the ID is out of range.
    pub fn region_to_slot_index(region_id: u8) -> Option<usize> {
        let idx = region_id.checked_sub(DYNAMIC_REGION_BASE)? as usize;
        if idx < DYNAMIC_SLOT_COUNT {
            Some(idx)
        } else {
            None
        }
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

    /// Compute the (RBAR, RASR) register values for regions R4-R7.
    ///
    /// Occupied slots (including peripheral-reserved R4/R5 populated by
    /// [`wire_boot_peripherals`]) emit their stored descriptor; empty
    /// slots emit a disabled region (RASR = 0).
    ///
    /// This is the testable, hardware-free counterpart of [`Self::program_regions`].
    pub fn compute_region_values(&self) -> [(u32, u32); DYNAMIC_SLOT_COUNT] {
        with_cs(|cs| {
            let slots = self.slots.borrow(cs);
            let slots = slots.borrow();
            let mut out = [(0u32, 0u32); DYNAMIC_SLOT_COUNT];
            for (idx, slot) in slots.iter().enumerate() {
                let region = DYNAMIC_REGION_BASE as u32 + idx as u32;
                // Defense-in-depth: add_window validates alignment before
                // storing descriptors, so build_rbar should always succeed
                // for occupied slots.  If a descriptor is somehow invalid
                // (e.g. corrupted base address), fall back to a disabled
                // region (RBAR = region select only, RASR = 0) rather than
                // panicking — this code runs in PendSV and must never fault.
                out[idx] = match slot {
                    Some(desc) => crate::mpu::build_rbar(desc.base, region)
                        .map(|rbar| (rbar, desc.permissions))
                        .unwrap_or((region, 0)),
                    None => (crate::mpu::build_rbar(0, region).unwrap_or(region), 0),
                };
            }
            out
        })
    }

    /// Directly overwrite a slot with an arbitrary descriptor.
    ///
    /// This bypasses `add_window` validation and is intended **only** for
    /// testing the `compute_region_values` fallback path (defense-in-depth).
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
    pub fn accessible_regions(&self, partition_id: u8) -> heapless::Vec<(u32, u32), 4> {
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

    /// Store peripheral descriptors for a partition in the cache.
    ///
    /// `partition_id` indexes into the cache (max `DYNAMIC_SLOT_COUNT - 1`).
    /// Out-of-range IDs are silently ignored.
    pub fn cache_peripherals(&self, partition_id: u8, descriptors: [Option<WindowDescriptor>; 2]) {
        let idx = partition_id as usize;
        if idx >= DYNAMIC_SLOT_COUNT {
            return;
        }
        with_cs(|cs| {
            self.peripheral_cache.borrow(cs).borrow_mut()[idx] = Some(descriptors);
        });
    }

    /// Retrieve cached peripheral descriptors for a partition.
    ///
    /// Returns `[None, None]` if no entry has been cached for `partition_id`
    /// or if the ID is out of range.
    pub fn cached_peripherals(&self, partition_id: u8) -> [Option<WindowDescriptor>; 2] {
        let idx = partition_id as usize;
        if idx >= DYNAMIC_SLOT_COUNT {
            return [None; 2];
        }
        with_cs(|cs| {
            self.peripheral_cache
                .borrow(cs)
                .borrow()
                .get(idx)
                .copied()
                .flatten()
                .unwrap_or([None; 2])
        })
    }

    /// Return (RBAR, RASR) register pairs for the cached peripheral
    /// descriptors of `partition_id`, targeting MPU regions R4 and R5.
    ///
    /// Cached descriptors (populated by [`wire_boot_peripherals`]) are
    /// converted via [`crate::mpu::build_rbar`]; uncached or absent
    /// entries yield a proper disabled pair (RBAR with VALID bit +
    /// region number, RASR = 0) so the MPU explicitly targets R4/R5
    /// instead of overwriting whatever is in MPU_RNR.
    pub fn cached_peripheral_regions(&self, partition_id: u8) -> [(u32, u32); 2] {
        let descs = self.cached_peripherals(partition_id);
        let mut out = [(0u32, 0u32); 2];
        for (slot_out, (i, slot)) in out.iter_mut().zip(descs.iter().enumerate()) {
            let region_id = DYNAMIC_REGION_BASE as u32 + i as u32;
            // Disabled pair: RBAR with VALID bit (bit 4) + region number,
            // RASR = 0.  Constructed directly so this path is infallible.
            let disabled = (region_id | (1 << 4), 0u32);
            *slot_out = match slot {
                Some(desc) => crate::mpu::build_rbar(desc.base, region_id)
                    .map(|rbar| (rbar, desc.permissions))
                    .unwrap_or(disabled),
                None => disabled,
            };
        }
        out
    }

    /// Populate dynamic slots with deduplicated peripheral regions from
    /// the given partitions (device memory: S=1,C=0,B=1 (Shareable Device memory), XN, AP_FULL_ACCESS).
    /// Reserved peripheral slots (R4/R5) are written directly; remaining
    /// regions use [`add_window`].  Returns the number wired.
    pub fn wire_boot_peripherals(
        &self,
        partitions: &[crate::partition::PartitionControlBlock],
    ) -> usize {
        // Deduplicate by (base, size). At most 3 slots available,
        // so a small inline set suffices.
        let mut seen: heapless::Vec<(u32, u32), 3> = heapless::Vec::new();
        let mut wired = 0usize;
        let reserved = with_cs(|cs| *self.peripheral_reserved.borrow(cs).borrow());

        for part in partitions.iter() {
            let mut part_descs: [Option<WindowDescriptor>; 2] = [None; 2];
            let mut desc_idx = 0usize;

            for region in part.peripheral_regions().iter() {
                let base = region.base();
                let size = region.size();
                if size < 32 || !size.is_power_of_two() {
                    continue;
                }
                let key = (base, size);
                let already_wired = seen.contains(&key);
                // Only wire if not already wired by a preceding partition.
                if !already_wired {
                    // MpuRegion.permissions is intentionally ignored: all peripheral
                    // MMIO uses fixed Shareable Device attributes (TEX=0 S=1 C=0 B=1,
                    // AP=full-access, XN=true).
                    let rasr = crate::mpu::build_rasr(
                        size.trailing_zeros() - 1,
                        crate::mpu::AP_FULL_ACCESS,
                        true,
                        (true, false, true),
                    );
                    // Populate reserved peripheral slots directly; add_window
                    // skips them so they must be written here.
                    if wired < reserved {
                        if crate::mpu::validate_mpu_region(base, size).is_err() {
                            continue;
                        }
                        with_cs(|cs| {
                            self.slots.borrow(cs).borrow_mut()[wired] = Some(WindowDescriptor {
                                base,
                                size,
                                permissions: rasr,
                                owner: part.id(),
                            });
                        });
                        let _ = seen.push(key);
                        wired += 1;
                    } else {
                        match self.add_window(base, size, rasr, part.id()) {
                            Ok(_) => {
                                let _ = seen.push(key);
                                wired += 1;
                            }
                            Err(MpuError::SlotExhausted) => {
                                self.cache_peripherals(part.id(), part_descs);
                                return wired;
                            }
                            Err(_) => continue,
                        }
                    }
                }
                // Cache for this partition regardless of prior wiring;
                // shared peripherals must appear in every partition's
                // cache for correct context-switch restoration.
                if desc_idx < 2 {
                    let rasr = crate::mpu::build_rasr(
                        size.trailing_zeros() - 1,
                        crate::mpu::AP_FULL_ACCESS,
                        true,
                        (true, false, true),
                    );
                    part_descs[desc_idx] = Some(WindowDescriptor {
                        base,
                        size,
                        permissions: rasr,
                        owner: part.id(),
                    });
                    desc_idx += 1;
                }
            }
            self.cache_peripherals(part.id(), part_descs);

            // Verify cached descriptors match the source PCB data.
            #[cfg(debug_assertions)]
            {
                let cached = self.cached_peripherals(part.id());
                // TODO: extract a shared `MpuRegion::is_mappable()` predicate so the
                // main selection loop and this debug check use the same filter;
                // for now we delegate to `validate_mpu_region` to avoid inlining
                // the size/power-of-two/alignment rules here.
                for (ci, region) in part
                    .peripheral_regions()
                    .iter()
                    .filter(|r| crate::mpu::validate_mpu_region(r.base(), r.size()).is_ok())
                    .take(cached.len())
                    .enumerate()
                {
                    if let Some(Some(desc)) = cached.get(ci) {
                        debug_assert_eq!(
                            desc.base,
                            region.base(),
                            "cache-PCB base mismatch: partition {} descriptor {}",
                            part.id(),
                            ci
                        );
                        debug_assert_eq!(
                            desc.size,
                            region.size(),
                            "cache-PCB size mismatch: partition {} descriptor {}",
                            part.id(),
                            ci
                        );
                    }
                }
            }
        }
        wired
    }

    /// Program regions R4-R7 into the MPU hardware.
    ///
    /// Uses [`mpu::configure_region`] to write each of the four dynamic
    /// slots' (RBAR, RASR) pairs, then issues DSB + ISB barriers to
    /// ensure the new configuration takes effect before any subsequent
    /// memory access.
    ///
    /// If the MPU is currently enabled (CTRL bit 0 set), it is disabled
    /// before writing regions and re-enabled with PRIVDEFENA afterwards.
    /// This prevents partially-configured region access during updates
    /// (ARM architecture requirement).  If the MPU is not enabled
    /// (pre-boot or non-MPU configuration), regions are written without
    /// touching CTRL, preserving existing behaviour.
    ///
    /// Regions R0-R3 (static background/code/data/guard) are left unchanged.
    ///
    /// The caller must pass an `&MPU` reference obtained from the
    /// peripheral singleton to make the hardware dependency explicit.
    #[cfg(not(test))]
    pub fn program_regions(&self, mpu_periph: &cortex_m::peripheral::MPU) {
        let values = self.compute_region_values();

        let ctrl = mpu_periph.ctrl.read();
        let mpu_was_enabled = ctrl & 1 != 0;

        // TODO: DRY — the disable-modify-enable MPU pattern is duplicated
        // here, in __boot_mpu_init, and in mpu::init_mpu_regions.  Extract
        // a safe helper (e.g. `mpu::with_disabled(|mpu| { ... })`) to
        // centralise the disable/barrier/enable sequence.
        if mpu_was_enabled {
            // SAFETY: Disabling the MPU before reprogramming regions is
            // required by the ARMv7-M architecture to avoid unpredictable
            // behaviour from partially-configured regions.  DSB+ISB ensure
            // the disable takes effect before any region writes.
            unsafe { mpu_periph.ctrl.write(0) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }

        for &(rbar, rasr) in &values {
            mpu::configure_region(mpu_periph, rbar, rasr);
        }

        if mpu_was_enabled {
            // Verify PRIVDEFENA (bit 2) is set — the kernel relies on the
            // default memory map for privileged access.  Evaluated at
            // compile time.
            const { assert!(mpu::MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 2) != 0) };

            // SAFETY: Re-enabling the MPU with PRIVDEFENA after all regions
            // have been programmed.  The exclusive &MPU reference guarantees
            // no concurrent access.
            unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }
}

impl MpuStrategy for DynamicStrategy {
    fn configure_partition(
        &self,
        partition_id: u8,
        regions: &[(u32, u32)],
        peripheral_reserved: usize,
    ) -> Result<(), MpuError> {
        // The static regions (R0-R3) are handled elsewhere; we only
        // care about the single private-RAM region.
        if regions.len() != 1 {
            return Err(MpuError::RegionCountMismatch);
        }

        // Peripheral MMIO slots must be reserved as a pair (R4-R5) or
        // not at all.  Any other value is a configuration error.
        if peripheral_reserved != 0 && peripheral_reserved != 2 {
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
            // Clear any previously-occupied RAM slot so the strategy
            // state doesn't carry stale descriptors when the reservation
            // count changes between configure_partition calls.
            let prev_reserved = *self.peripheral_reserved.borrow(cs).borrow();
            if prev_reserved != peripheral_reserved {
                slots[prev_reserved] = None;
            }
            slots[ram_slot] = Some(WindowDescriptor {
                base,
                size,
                permissions: rasr,
                owner: partition_id,
            });
            *self.peripheral_reserved.borrow(cs).borrow_mut() = peripheral_reserved;
        });
        Ok(())
    }

    fn add_window(
        &self,
        base: u32,
        size: u32,
        permissions: u32,
        owner: u8,
    ) -> Result<u8, MpuError> {
        crate::mpu::validate_mpu_region(base, size)?;

        with_cs(|cs| {
            let mut slots = self.slots.borrow(cs).borrow_mut();
            let reserved = *self.peripheral_reserved.borrow(cs).borrow();
            // Skip peripheral-reserved slots (0..reserved) and the
            // partition-RAM slot (reserved), scanning from reserved+1
            // onwards for a free entry.
            let first_window = reserved + 1;
            for (idx, slot) in slots.iter_mut().enumerate().skip(first_window) {
                if slot.is_none() {
                    *slot = Some(WindowDescriptor {
                        base,
                        size,
                        permissions,
                        owner,
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
/// sequence as [`mpu::apply_partition_mpu`].
pub struct StaticStrategy;

/// The number of MPU regions the static layout requires.
const STATIC_REGION_COUNT: usize = 4;

impl MpuStrategy for StaticStrategy {
    fn configure_partition(
        &self,
        _partition_id: u8,
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
        _owner: u8,
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
/// and issues DSB/ISB barriers — mirroring [`mpu::apply_partition_mpu`].
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

    // ------------------------------------------------------------------
    // configure_partition: slice-to-array conversion and dispatch
    // ------------------------------------------------------------------

    #[test]
    fn configure_partition_accepts_4_regions() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let regions = partition_mpu_regions(&pcb).unwrap();
        let strategy = StaticStrategy;

        // Passing the 4 computed regions should succeed.
        assert_eq!(strategy.configure_partition(0, &regions, 0), Ok(()));
    }

    #[test]
    fn configure_partition_rejects_wrong_count() {
        let strategy = StaticStrategy;

        // Too few regions.
        assert_eq!(
            strategy.configure_partition(0, &[(0x0, 0x0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // 3 regions is now too few (was the old count).
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // Too many regions.
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );

        // Empty.
        assert_eq!(
            strategy.configure_partition(0, &[], 0),
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
        assert_eq!(strategy.configure_partition(0, &regions, 0), Ok(()));
    }

    #[test]
    fn configure_partition_different_partitions() {
        let strategy = StaticStrategy;

        let pcb0 = make_pcb(0x0000_0000, 0x2000_0000, 1024);
        let r0 = partition_mpu_regions(&pcb0).unwrap();
        assert_eq!(strategy.configure_partition(0, &r0, 0), Ok(()));

        let pcb1 = make_pcb(0x0000_0000, 0x2000_8000, 1024);
        let r1 = partition_mpu_regions(&pcb1).unwrap();
        assert_eq!(strategy.configure_partition(1, &r1, 0), Ok(()));
    }

    // ------------------------------------------------------------------
    // add_window / remove_window on StaticStrategy
    // ------------------------------------------------------------------

    #[test]
    fn static_strategy_add_window_returns_slot_exhausted() {
        let strategy = StaticStrategy;
        assert_eq!(
            strategy.add_window(0x2000_0000, 256, 0, 0),
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
            strategy.add_window(0, 0, 0, 0),
            Err(MpuError::SlotExhausted)
        );
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0), (0, 0)], 0),
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
        assert_eq!(ds.configure_partition(2, &[(rbar, rasr)], 0), Ok(()));

        let desc = ds.slot(4).expect("R4 should be occupied");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.size, 4096);
        assert_eq!(desc.permissions, rasr);
        assert_eq!(desc.owner, 2);
    }

    #[test]
    fn dynamic_configure_partition_rejects_wrong_count() {
        let ds = DynamicStrategy::new();
        // Empty.
        assert_eq!(
            ds.configure_partition(0, &[], 0),
            Err(MpuError::RegionCountMismatch),
        );
        // Multiple regions — only exactly one is accepted.
        assert_eq!(
            ds.configure_partition(0, &[(0, 0), (0, 0)], 0),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn dynamic_add_window_allocates_r5_r6_r7() {
        let ds = DynamicStrategy::new();
        let r5 = ds.add_window(0x2001_0000, 256, 0xAA, 1);
        assert_eq!(r5, Ok(5));

        let r6 = ds.add_window(0x2002_0000, 512, 0xBB, 2);
        assert_eq!(r6, Ok(6));

        let r7 = ds.add_window(0x2003_0000, 1024, 0xCC, 3);
        assert_eq!(r7, Ok(7));

        // Verify stored descriptors.
        let d5 = ds.slot(5).unwrap();
        assert_eq!(d5.base, 0x2001_0000);
        assert_eq!(d5.size, 256);
        assert_eq!(d5.permissions, 0xAA);
        assert_eq!(d5.owner, 1);

        let d6 = ds.slot(6).unwrap();
        assert_eq!(d6.base, 0x2002_0000);
        assert_eq!(d6.size, 512);
        assert_eq!(d6.owner, 2);

        let d7 = ds.slot(7).unwrap();
        assert_eq!(d7.base, 0x2003_0000);
        assert_eq!(d7.size, 1024);
        assert_eq!(d7.owner, 3);
    }

    #[test]
    fn dynamic_add_window_exhaustion() {
        let ds = DynamicStrategy::new();
        assert!(ds.add_window(0x2001_0000, 256, 0, 1).is_ok()); // R5
        assert!(ds.add_window(0x2002_0000, 256, 0, 1).is_ok()); // R6
        assert!(ds.add_window(0x2003_0000, 256, 0, 1).is_ok()); // R7

        // Fourth dynamic window should fail — only 3 dynamic slots.
        assert_eq!(
            ds.add_window(0x2004_0000, 256, 0, 1),
            Err(MpuError::SlotExhausted)
        );
    }

    #[test]
    fn dynamic_remove_window_clears_slot() {
        let ds = DynamicStrategy::new();
        let rid = ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
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
        let r5 = ds.add_window(0x2001_0000, 256, 0x11, 1).unwrap();
        let r6 = ds.add_window(0x2002_0000, 512, 0x22, 2).unwrap();
        let r7 = ds.add_window(0x2003_0000, 1024, 0x33, 3).unwrap();
        assert_eq!((r5, r6, r7), (5, 6, 7));

        // Remove R6, then a new add should reuse slot index 2 → R6.
        ds.remove_window(6);
        assert!(ds.slot(6).is_none());

        let new = ds.add_window(0x2004_0000, 2048, 0x44, 4).unwrap();
        assert_eq!(new, 6);
        let d = ds.slot(6).unwrap();
        assert_eq!(d.base, 0x2004_0000);
        assert_eq!(d.size, 2048);
        assert_eq!(d.permissions, 0x44);
        assert_eq!(d.owner, 4);
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
        ds.configure_partition(1, &[r1], 0).unwrap();
        assert_eq!(ds.slot(4).unwrap().owner, 1);

        // Reconfigure with different partition.
        let r2 = data_region(0x2000_8000, 1024, 4);
        ds.configure_partition(3, &[r2], 0).unwrap();
        let d = ds.slot(4).unwrap();
        assert_eq!(d.owner, 3);
        assert_eq!(d.base, 0x2000_8000);
        assert_eq!(d.size, 1024);
    }

    #[test]
    fn dynamic_add_window_does_not_touch_r4() {
        let ds = DynamicStrategy::new();
        let r = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[r], 0).unwrap();

        // Fill all dynamic slots.
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2002_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2003_0000, 256, 0, 1).unwrap();

        // R4 should still hold the partition descriptor.
        let d = ds.slot(4).unwrap();
        assert_eq!(d.base, 0x2000_0000);
        assert_eq!(d.owner, 0);
    }

    #[test]
    fn dynamic_satisfies_trait_object() {
        let ds = DynamicStrategy::new();
        let strategy: &dyn MpuStrategy = &ds;
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, 1), Ok(5));
        strategy.remove_window(5);
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, 1), Ok(5));
    }

    // ------------------------------------------------------------------
    // compute_region_values
    // ------------------------------------------------------------------

    #[test]
    fn compute_region_values_all_empty() {
        let ds = DynamicStrategy::new();
        let vals = ds.compute_region_values();
        // All four slots empty → RBAR selects region, RASR = 0 (disabled).
        for (idx, &(rbar, rasr)) in vals.iter().enumerate() {
            let region = 4 + idx as u32;
            assert_eq!(rbar, build_rbar(0, region).unwrap());
            assert_eq!(rasr, 0);
        }
    }

    #[test]
    fn compute_region_values_single_window() {
        let ds = DynamicStrategy::new();
        let (rbar_in, rasr_in) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar_in, rasr_in)], 0).unwrap();

        let vals = ds.compute_region_values();

        // Slot 0 (R4): partition RAM — RBAR has base 0x2000_0000, region 4.
        assert_eq!(vals[0].0, build_rbar(0x2000_0000, 4).unwrap());
        assert_eq!(vals[0].1, rasr_in);

        // Slots 1-3 (R5-R7) still disabled.
        for &(rbar, rasr) in &vals[1..] {
            assert_eq!(rasr, 0);
            // RBAR should still select the correct region.
            assert_ne!(rbar, 0);
        }
    }

    #[test]
    fn compute_region_values_multiple_windows() {
        let ds = DynamicStrategy::new();
        let (rbar_r4, rasr_r4) = data_region(0x2000_0000, 1024, 4);
        ds.configure_partition(1, &[(rbar_r4, rasr_r4)], 0).unwrap();

        let sf_256 = encode_size(256).unwrap();
        let rasr_r5 = build_rasr(sf_256, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2001_0000, 256, rasr_r5, 1).unwrap(); // R5

        let sf_512 = encode_size(512).unwrap();
        let rasr_r6 = build_rasr(sf_512, AP_RO_RO, false, (false, false, false));
        ds.add_window(0x2002_0000, 512, rasr_r6, 1).unwrap(); // R6

        let vals = ds.compute_region_values();

        // R4
        assert_eq!(vals[0].0, build_rbar(0x2000_0000, 4).unwrap());
        assert_eq!(vals[0].1, rasr_r4);
        // R5
        assert_eq!(vals[1].0, build_rbar(0x2001_0000, 5).unwrap());
        assert_eq!(vals[1].1, rasr_r5);
        // R6
        assert_eq!(vals[2].0, build_rbar(0x2002_0000, 6).unwrap());
        assert_eq!(vals[2].1, rasr_r6);
        // R7 still disabled
        assert_eq!(vals[3].0, build_rbar(0, 7).unwrap());
        assert_eq!(vals[3].1, 0);
    }

    #[test]
    fn compute_region_values_after_removal() {
        let ds = DynamicStrategy::new();
        let (rbar_r4, rasr_r4) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)], 0).unwrap();

        let sf = encode_size(256).unwrap();
        let rasr_win = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2001_0000, 256, rasr_win, 0).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, rasr_win, 0).unwrap(); // R6

        // Remove R5, keep R6.
        ds.remove_window(5);

        let vals = ds.compute_region_values();
        // R4 still active
        assert_eq!(vals[0].1, rasr_r4);
        // R5 now disabled
        assert_eq!(vals[1].1, 0);
        // R6 still active
        assert_eq!(vals[2].0, build_rbar(0x2002_0000, 6).unwrap());
        assert_eq!(vals[2].1, rasr_win);
        // R7 disabled
        assert_eq!(vals[3].1, 0);
    }

    #[test]
    fn compute_region_values_all_slots_occupied() {
        let ds = DynamicStrategy::new();
        let (rbar_r4, rasr_r4) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)], 0).unwrap();

        let sf = encode_size(256).unwrap();
        let rasr_win = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2001_0000, 256, rasr_win, 0).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, rasr_win, 0).unwrap(); // R6
        ds.add_window(0x2003_0000, 256, rasr_win, 0).unwrap(); // R7

        let vals = ds.compute_region_values();
        // All four regions should have non-zero RASR.
        for &(_rbar, rasr) in &vals {
            assert_ne!(rasr, 0);
        }
        // Verify specific RBAR base addresses.
        assert_eq!(vals[0].0, build_rbar(0x2000_0000, 4).unwrap());
        assert_eq!(vals[1].0, build_rbar(0x2001_0000, 5).unwrap());
        assert_eq!(vals[2].0, build_rbar(0x2002_0000, 6).unwrap());
        assert_eq!(vals[3].0, build_rbar(0x2003_0000, 7).unwrap());
    }

    // ------------------------------------------------------------------
    // compute_region_values: defense-in-depth fallback
    // ------------------------------------------------------------------

    #[test]
    fn compute_region_values_fallback_on_bad_descriptor() {
        let ds = DynamicStrategy::new();

        // Inject a descriptor with a misaligned base address into R5.
        // build_rbar will return None for base 0x03 (not 32-byte aligned),
        // so compute_region_values must fall back to a disabled region.
        ds.inject_slot(
            5,
            Some(WindowDescriptor {
                base: 0x03, // misaligned — build_rbar returns None
                size: 256,
                permissions: 0xDEAD,
                owner: 1,
            }),
        );

        let vals = ds.compute_region_values();

        // R4 (slot 0) — empty, normal disabled region.
        assert_eq!(vals[0].0, build_rbar(0, 4).unwrap());
        assert_eq!(vals[0].1, 0);

        // R5 (slot 1) — bad descriptor, fallback: disabled region.
        // RBAR is the bare region number (no VALID bit, no base),
        // RASR is 0 (region fully disabled).
        assert_eq!(vals[1].0, 5); // region number only, no VALID bit
        assert_eq!(vals[1].1, 0); // disabled — RASR must be 0

        // R6, R7 — empty, normal disabled regions.
        assert_eq!(vals[2].0, build_rbar(0, 6).unwrap());
        assert_eq!(vals[2].1, 0);
        assert_eq!(vals[3].0, build_rbar(0, 7).unwrap());
        assert_eq!(vals[3].1, 0);
    }

    #[test]
    fn compute_region_values_fallback_preserves_other_valid_slots() {
        let ds = DynamicStrategy::new();

        // Configure R4 with a valid partition region.
        let (rbar_r4, rasr_r4) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)], 0).unwrap();

        // Add valid windows in R5 and R6.
        let sf = encode_size(256).unwrap();
        let rasr_win = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2001_0000, 256, rasr_win, 0).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, rasr_win, 0).unwrap(); // R6

        // Inject a bad descriptor into R7 (misaligned base).
        ds.inject_slot(
            7,
            Some(WindowDescriptor {
                base: 0x07,
                size: 64,
                permissions: 0xBEEF,
                owner: 2,
            }),
        );

        let vals = ds.compute_region_values();

        // R4 — valid, should be unaffected.
        assert_eq!(vals[0].0, build_rbar(0x2000_0000, 4).unwrap());
        assert_eq!(vals[0].1, rasr_r4);

        // R5 — valid window, unaffected.
        assert_eq!(vals[1].0, build_rbar(0x2001_0000, 5).unwrap());
        assert_eq!(vals[1].1, rasr_win);

        // R6 — valid window, unaffected.
        assert_eq!(vals[2].0, build_rbar(0x2002_0000, 6).unwrap());
        assert_eq!(vals[2].1, rasr_win);

        // R7 — bad descriptor, fallback: disabled region.
        assert_eq!(vals[3].0, 7);
        assert_eq!(vals[3].1, 0); // disabled — RASR must be 0
    }

    // ------------------------------------------------------------------
    // compute_region_values: peripheral slot reservation
    // ------------------------------------------------------------------

    #[test]
    fn compute_region_values_peripheral_reservation_zero() {
        // peripheral_reserved=0: RAM in slot 0 (R4), windows in slots 1-3.
        let ds = DynamicStrategy::new();
        let (rbar_r4, rasr_r4) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)], 0).unwrap();
        let sf = encode_size(256).unwrap();
        let rasr_win = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2001_0000, 256, rasr_win, 0).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, rasr_win, 0).unwrap(); // R6
        ds.add_window(0x2003_0000, 256, rasr_win, 0).unwrap(); // R7

        let v = ds.compute_region_values();
        assert_eq!(v[0].0, build_rbar(0x2000_0000, 4).unwrap());
        assert_eq!(v[0].1, rasr_r4, "R4 holds partition RAM");
        assert_eq!(v[1].1, rasr_win, "R5 holds window");
        assert_eq!(v[2].1, rasr_win, "R6 holds window");
        assert_eq!(v[3].1, rasr_win, "R7 holds window");
    }

    #[test]
    fn compute_region_values_peripheral_reservation_two() {
        // peripheral_reserved=2: R4-R5 reserved (disabled), RAM in slot 2 (R6),
        // window in slot 3 (R7).
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();
        let sf = encode_size(256).unwrap();
        let rasr_win = build_rasr(sf, AP_FULL_ACCESS, true, (false, false, false));
        ds.add_window(0x2003_0000, 256, rasr_win, 0).unwrap(); // R7

        let v = ds.compute_region_values();
        // R4-R5 disabled (reserved for peripheral MMIO).
        assert_eq!(v[0].0, build_rbar(0, 4).unwrap());
        assert_eq!(v[0].1, 0, "R4 must be disabled (peripheral-reserved)");
        assert_eq!(v[1].0, build_rbar(0, 5).unwrap());
        assert_eq!(v[1].1, 0, "R5 must be disabled (peripheral-reserved)");
        // R6 holds partition RAM.
        assert_eq!(v[2].0, build_rbar(0x2000_0000, 6).unwrap());
        assert_eq!(v[2].1, rasr_r6, "R6 holds partition RAM");
        // R7 holds dynamic window.
        assert_eq!(v[3].0, build_rbar(0x2003_0000, 7).unwrap());
        assert_eq!(v[3].1, rasr_win, "R7 holds window");
    }

    #[test]
    fn compute_region_values_includes_wired_peripheral() {
        // peripheral_reserved=2, wire one peripheral → R4 must carry it.
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();
        let pcb =
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        assert_eq!(ds.wire_boot_peripherals(&[pcb]), 1);
        let v = ds.compute_region_values();
        // R4: wired peripheral with device-memory attributes.
        assert_eq!(v[0].0, build_rbar(0x4000_0000, 4).unwrap());
        let rasr = v[0].1;
        assert_ne!(rasr, 0, "R4 RASR must be non-zero (peripheral enabled)");
        assert_eq!((rasr >> 24) & 0x7, AP_FULL_ACCESS);
        assert_eq!((rasr >> 28) & 1, 1, "XN must be set");
        assert_eq!((rasr >> 16) & 0x7, 0b101, "S/C/B=101 (device memory)");
        assert_eq!(v[1].1, 0, "R5 disabled (no second peripheral)");
        assert_eq!(v[2].0, build_rbar(0x2000_0000, 6).unwrap());
        assert_eq!(v[2].1, rasr_r6, "R6 holds partition RAM");
        assert_eq!(v[3].1, 0, "R7 disabled (no window)");
    }

    #[test]
    fn configure_partition_rejects_peripheral_reserved_one() {
        // Only 0 or 2 are valid; 1 is rejected.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        assert_eq!(
            ds.configure_partition(0, &[(rbar, rasr)], 1),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn configure_partition_rejects_peripheral_reserved_three() {
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        assert_eq!(
            ds.configure_partition(0, &[(rbar, rasr)], 3),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn configure_partition_switches_reservation() {
        // Switching from peripheral_reserved=0 to 2 moves RAM from slot 0 to slot 2.
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar, rasr)], 0).unwrap();
        assert!(ds.slot(4).is_some(), "RAM in R4 with reservation=0");

        // Reconfigure with reservation=2.
        let (rbar2, rasr2) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(1, &[(rbar2, rasr2)], 2).unwrap();
        assert!(
            ds.slot(4).is_none(),
            "R4 cleared after switch to reservation=2"
        );
        let desc = ds.slot(6).expect("RAM should be in R6");
        assert_eq!(desc.base, 0x2000_8000);
        assert_eq!(desc.owner, 1);

        // R4-R5 disabled in compute_region_values.
        let v = ds.compute_region_values();
        assert_eq!(v[0].1, 0, "R4 disabled");
        assert_eq!(v[1].1, 0, "R5 disabled");
        assert_eq!(v[2].1, rasr2, "R6 holds partition RAM");
    }

    // ------------------------------------------------------------------
    // DynamicStrategy::add_window validation integration
    // ------------------------------------------------------------------

    #[test]
    fn dynamic_add_window_rejects_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 16, 0, 1), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn dynamic_add_window_rejects_non_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 48, 0, 1), Err(MpuError::SizeNotPowerOfTwo));
    }

    #[test]
    fn dynamic_add_window_rejects_misaligned_base() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(64, 256, 0, 1), Err(MpuError::BaseNotAligned));
    }

    #[test]
    fn dynamic_add_window_rejects_overflow() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, 1),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn dynamic_add_window_validation_before_slot_check() {
        let ds = DynamicStrategy::new();
        // Fill all dynamic slots with valid windows.
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap(); // R5
        ds.add_window(0x2002_0000, 256, 0, 1).unwrap(); // R6
        ds.add_window(0x2003_0000, 256, 0, 1).unwrap(); // R7

        // Even with all slots full, invalid params yield validation
        // errors, not SlotExhausted.
        assert_eq!(ds.add_window(0, 16, 0, 1), Err(MpuError::SizeTooSmall));
        assert_eq!(ds.add_window(0, 48, 0, 1), Err(MpuError::SizeNotPowerOfTwo));
        assert_eq!(ds.add_window(64, 256, 0, 1), Err(MpuError::BaseNotAligned));
    }

    #[test]
    fn dynamic_add_window_valid_params_still_succeed() {
        let ds = DynamicStrategy::new();
        // Valid aligned pair should succeed and return region ID.
        assert_eq!(ds.add_window(0x2001_0000, 256, 0xAA, 1), Ok(5));
        let d = ds.slot(5).unwrap();
        assert_eq!(d.base, 0x2001_0000);
        assert_eq!(d.size, 256);
        assert_eq!(d.permissions, 0xAA);
        assert_eq!(d.owner, 1);
    }

    // ------------------------------------------------------------------
    // DynamicStrategy::add_window — exhaustive validation coverage
    // ------------------------------------------------------------------

    #[test]
    fn add_window_size_zero_returns_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 0, 0, 0), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn add_window_size_16_returns_size_too_small() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 16, 0, 0), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn add_window_size_48_returns_not_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 48, 0, 0), Err(MpuError::SizeNotPowerOfTwo));
    }

    #[test]
    fn add_window_size_100_returns_not_power_of_two() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.add_window(0, 100, 0, 0),
            Err(MpuError::SizeNotPowerOfTwo)
        );
    }

    #[test]
    fn add_window_base_off_by_one_returns_not_aligned() {
        let ds = DynamicStrategy::new();
        // 0x2000_0001 is 1 byte past a 256-byte boundary.
        assert_eq!(
            ds.add_window(0x2000_0001, 256, 0, 1),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn add_window_base_half_aligned_returns_not_aligned() {
        let ds = DynamicStrategy::new();
        // 0x2000_0080 is 128-aligned but not 256-aligned.
        assert_eq!(
            ds.add_window(0x2000_0080, 256, 0, 1),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn add_window_max_address_overflow() {
        let ds = DynamicStrategy::new();
        // base near u32::MAX with small valid size overflows.
        assert_eq!(
            ds.add_window(0xFFFF_FFE0, 32, 0, 0),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn add_window_overflow_base_0xffff_ff00_size_256() {
        let ds = DynamicStrategy::new();
        // 0xFFFF_FF00 + 256 = 0x1_0000_0000 which overflows u32.
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, 0),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn add_window_size_32_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2000_0000, 32, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_size_64_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2000_0040, 64, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_size_4096_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0x2001_0000, 4096, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_32_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 32, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_64_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 64, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_base_zero_size_4096_succeeds() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.add_window(0, 4096, 0, 0), Ok(5));
    }

    #[test]
    fn add_window_validation_precedes_slot_exhaustion_all_variants() {
        let ds = DynamicStrategy::new();
        // Fill all 3 dynamic slots.
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2002_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2003_0000, 256, 0, 1).unwrap();

        // Every validation error is returned instead of SlotExhausted.
        assert_eq!(ds.add_window(0, 0, 0, 0), Err(MpuError::SizeTooSmall));
        assert_eq!(ds.add_window(0, 16, 0, 0), Err(MpuError::SizeTooSmall));
        assert_eq!(ds.add_window(0, 48, 0, 0), Err(MpuError::SizeNotPowerOfTwo));
        assert_eq!(
            ds.add_window(0, 100, 0, 0),
            Err(MpuError::SizeNotPowerOfTwo)
        );
        assert_eq!(
            ds.add_window(0x2000_0001, 256, 0, 0),
            Err(MpuError::BaseNotAligned)
        );
        assert_eq!(
            ds.add_window(0xFFFF_FF00, 256, 0, 0),
            Err(MpuError::AddressOverflow)
        );

        // But a valid request on full slots gives SlotExhausted.
        assert_eq!(
            ds.add_window(0x2004_0000, 256, 0, 1),
            Err(MpuError::SlotExhausted)
        );
    }

    // ------------------------------------------------------------------
    // accessible_regions
    // ------------------------------------------------------------------

    #[test]
    fn accessible_regions_multiple_slots_same_owner() {
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(1, &[(rbar, rasr)], 0).unwrap();
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2002_0000, 512, 0, 1).unwrap();

        let result = ds.accessible_regions(1);
        assert_eq!(result.len(), 3);
        assert!(result.contains(&(0x2000_0000, 4096)));
        assert!(result.contains(&(0x2001_0000, 256)));
        assert!(result.contains(&(0x2002_0000, 512)));
    }

    #[test]
    fn accessible_regions_mixed_owners() {
        let ds = DynamicStrategy::new();
        let (rbar, rasr) = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[(rbar, rasr)], 0).unwrap();
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
        ds.add_window(0x2002_0000, 512, 0, 0).unwrap();
        ds.add_window(0x2003_0000, 1024, 0, 2).unwrap();

        let p0 = ds.accessible_regions(0);
        assert_eq!(p0.len(), 2);
        assert!(p0.contains(&(0x2000_0000, 4096)));
        assert!(p0.contains(&(0x2002_0000, 512)));

        assert_eq!(ds.accessible_regions(1).len(), 1);
        assert_eq!(ds.accessible_regions(2).len(), 1);
    }

    #[test]
    fn accessible_regions_no_owners_returns_empty() {
        let ds = DynamicStrategy::new();
        // Empty strategy returns empty for any partition.
        assert!(ds.accessible_regions(0).is_empty());
        assert!(ds.accessible_regions(255).is_empty());

        // With windows present but different owner.
        ds.add_window(0x2001_0000, 256, 0, 1).unwrap();
        assert!(ds.accessible_regions(0).is_empty());
        assert!(ds.accessible_regions(2).is_empty());
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
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();
        let pcb =
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_0000, 4096)]);
        assert_eq!(ds.wire_boot_peripherals(&[pcb]), 1);

        let v1 = ds.compute_region_values();
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
        ds.configure_partition(1, &[(rbar_r4, rasr_r4)], 0).unwrap();

        let v2 = ds.compute_region_values();
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
        // configure_partition moves peripheral_reserved from 0→2:
        //   - clears slot[0] (old RAM at R4)
        //   - places RAM in slot[2] (R6)
        //   - R4-R5 are now empty (reserved for peripherals)
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();

        // Verify R4 is properly cleared (peripheral slot empty before wiring).
        let v3_pre = ds.compute_region_values();
        assert_eq!(v3_pre[0].1, 0, "R4 empty before peripheral re-wiring");
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

        let v3 = ds.compute_region_values();
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
        ds.configure_partition(0, &[(rb, rs)], 0).unwrap();
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
        let pcb0 =
            make_pcb(0x0, 0x2000_0000, 4096).with_peripheral_regions(&[periph(0x4000_C000, 4096)]);
        let pcb1 =
            make_pcb(0x0, 0x2000_8000, 4096).with_peripheral_regions(&[periph(0x4002_5000, 4096)]);
        // Partition 2: no peripherals.
        let pcb_none = make_pcb(0x0, 0x2001_0000, 4096);

        // --- (1) wire_boot_peripherals populates reserved slots and
        //         compute_region_values returns non-zero RASR for them ---
        let ds = DynamicStrategy::new();
        let (rbar_r6, rasr_r6) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();
        let wired = ds.wire_boot_peripherals(&[pcb0.clone(), pcb1.clone()]);
        assert_eq!(wired, 2, "both peripherals must be wired");
        let vals = ds.compute_region_values();
        assert_ne!(vals[0].1, 0, "R4 RASR must be non-zero (UART0)");
        assert_ne!(vals[1].1, 0, "R5 RASR must be non-zero (GPIO)");

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
                owner: 0,
            }),
            Some(WindowDescriptor {
                base: 0x4001_0000,
                size: 256,
                permissions: 0xCD,
                owner: 0,
            }),
        ];
        ds.cache_peripherals(0, descs);
        let got = ds.cached_peripherals(0);
        assert_eq!(got, descs);
    }

    #[test]
    fn peripheral_cache_uncached_returns_none_none() {
        let ds = DynamicStrategy::new();
        assert_eq!(ds.cached_peripherals(0), [None, None]);
        assert_eq!(ds.cached_peripherals(3), [None, None]);
        // Out-of-range partition IDs also return [None, None].
        assert_eq!(ds.cached_peripherals(255), [None, None]);
    }

    #[test]
    fn peripheral_cache_overwrite_updates_entry() {
        let ds = DynamicStrategy::new();
        let first = [
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: 0x11,
                owner: 1,
            }),
            None,
        ];
        ds.cache_peripherals(1, first);
        assert_eq!(ds.cached_peripherals(1), first);

        let second = [
            Some(WindowDescriptor {
                base: 0x4002_0000,
                size: 512,
                permissions: 0x22,
                owner: 1,
            }),
            Some(WindowDescriptor {
                base: 0x4003_0000,
                size: 1024,
                permissions: 0x33,
                owner: 1,
            }),
        ];
        ds.cache_peripherals(1, second);
        assert_eq!(ds.cached_peripherals(1), second);
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals populates peripheral_cache
    // ------------------------------------------------------------------

    #[test]
    fn wire_boot_peripherals_populates_cache() {
        let ds = DynamicStrategy::new();
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

        let cached0 = ds.cached_peripherals(0);
        assert_eq!(
            cached0[0],
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: rasr_4k,
                owner: 0
            })
        );
        assert_eq!(cached0[1], None);

        let cached1 = ds.cached_peripherals(1);
        assert_eq!(
            cached1[0],
            Some(WindowDescriptor {
                base: 0x4001_0000,
                size: 256,
                permissions: rasr_256,
                owner: 1
            })
        );
        assert_eq!(cached1[1], None);
    }

    #[test]
    fn wire_boot_peripherals_no_peripherals_cached_as_none() {
        let ds = DynamicStrategy::new();
        let pcb = make_pcb(0x0, 0x2000_0000, 4096); // no peripheral regions
        ds.wire_boot_peripherals(&[pcb]);
        assert_eq!(ds.cached_peripherals(0), [None, None]);
    }

    #[test]
    fn wire_boot_peripherals_shared_peripheral_cached_for_both() {
        let ds = DynamicStrategy::new();
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

        let cached0 = ds.cached_peripherals(0);
        assert_eq!(
            cached0[0],
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: rasr,
                owner: 0
            })
        );
        assert_eq!(cached0[1], None);

        let cached1 = ds.cached_peripherals(1);
        assert_eq!(
            cached1[0],
            Some(WindowDescriptor {
                base: 0x4000_0000,
                size: 4096,
                permissions: rasr,
                owner: 1
            })
        );
        assert_eq!(cached1[1], None);
    }

    #[test]
    fn wire_boot_peripherals_cache_pcb_consistency() {
        // Verifies that the debug_assertions consistency check inside
        // wire_boot_peripherals does not fire for well-formed inputs:
        // two partitions each with one valid peripheral region.
        let ds = DynamicStrategy::new();
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

        // Explicitly verify base/size of cached descriptors match PCB regions.
        let cached0 = ds.cached_peripherals(0);
        assert_eq!(cached0[0].unwrap().base, 0x4000_0000);
        assert_eq!(cached0[0].unwrap().size, 4096);

        let cached1 = ds.cached_peripherals(1);
        assert_eq!(cached1[0].unwrap().base, 0x4001_0000);
        assert_eq!(cached1[0].unwrap().size, 256);
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
    fn periph_desc(base: u32, size: u32, owner: u8) -> WindowDescriptor {
        WindowDescriptor {
            base,
            size,
            permissions: periph_rasr(size),
            owner,
        }
    }

    #[test]
    fn cached_peripheral_regions_one_cached() {
        let ds = DynamicStrategy::new();
        ds.cache_peripherals(0, [Some(periph_desc(0x4000_0000, 4096, 0)), None]);
        let r = ds.cached_peripheral_regions(0);
        assert_eq!(
            r[0],
            (build_rbar(0x4000_0000, 4).unwrap(), periph_rasr(4096))
        );
        assert_eq!(r[1], (build_rbar(0, 5).unwrap(), 0));
    }

    #[test]
    fn cached_peripheral_regions_two_peripherals_both_enabled() {
        let ds = DynamicStrategy::new();
        ds.cache_peripherals(
            2,
            [
                Some(periph_desc(0x4000_0000, 4096, 2)),
                Some(periph_desc(0x4001_0000, 256, 2)),
            ],
        );
        let r = ds.cached_peripheral_regions(2);
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
        let disabled_r4 = (build_rbar(0, 4).unwrap(), 0);
        let disabled_r5 = (build_rbar(0, 5).unwrap(), 0);
        assert_eq!(ds.cached_peripheral_regions(0), [disabled_r4, disabled_r5],);
        assert_eq!(
            ds.cached_peripheral_regions(255),
            [disabled_r4, disabled_r5],
        );
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
        ds.configure_partition(0, &[(rbar_p0, rasr_p0)], 2).unwrap();
        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(1, &[(rbar_p1, rasr_p1)], 2).unwrap();
        let (rbar_p2, rasr_p2) = data_region(0x2001_0000, 4096, 6);
        ds.configure_partition(2, &[(rbar_p2, rasr_p2)], 2).unwrap();

        let wired = ds.wire_boot_peripherals(&[pcb0.clone(), pcb1.clone(), pcb_none.clone()]);
        assert_eq!(wired, 2, "both peripherals must be wired");

        // --- Partition 0: one peripheral ---
        let cached_p0 = ds.cached_peripheral_regions(0);
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
        let cached_p1 = ds.cached_peripheral_regions(1);
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
        let cached_none = ds.cached_peripheral_regions(2);
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

    /// Simulate the exact PendSV R4-R7 write sequence for two partitions
    /// (one with a peripheral, one without) and verify that
    /// `cached_peripheral_regions` correctly overrides the R4-R5 values
    /// produced by `compute_region_values`.
    #[test]
    fn peripheral_cache_override_correctness_in_pendsv_flow() {
        let ds = DynamicStrategy::new();

        // -- Setup: partition 0 has a UART peripheral; partition 1 has none --
        // Both use peripheral_reserved=2 so RAM lands in R6.
        let (rbar_p0, rasr_p0) = data_region(0x2000_0000, 4096, 6);
        ds.configure_partition(0, &[(rbar_p0, rasr_p0)], 2).unwrap();

        let (rbar_p1, rasr_p1) = data_region(0x2000_8000, 4096, 6);
        ds.configure_partition(1, &[(rbar_p1, rasr_p1)], 2).unwrap();

        // Wire partition 0's peripheral at boot time.
        // Use explicit partition IDs so the cache is keyed correctly.
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
        );
        assert_eq!(ds.wire_boot_peripherals(&[pcb0, pcb1]), 1);

        // ---- Simulate PendSV switching TO partition 0 (peripheral) ----
        ds.configure_partition(0, &[(rbar_p0, rasr_p0)], 2).unwrap();
        let compute_p0 = ds.compute_region_values();
        let cached_p0 = ds.cached_peripheral_regions(0);

        // R4 from compute should hold the wired peripheral (slot[0]).
        // The cache must agree — both should emit the peripheral RBAR/RASR.
        let expected_periph_rbar = build_rbar(0x4000_0000, 4).unwrap();
        let expected_periph_rasr = periph_rasr(4096);
        assert_eq!(
            cached_p0[0],
            (expected_periph_rbar, expected_periph_rasr),
            "cache R4 must emit peripheral for partition 0"
        );
        assert_ne!(cached_p0[0].1, 0, "peripheral R4 RASR must be enabled");

        // For the peripheral partition, compute and cache agree on R4.
        assert_eq!(
            compute_p0[0], cached_p0[0],
            "compute and cache R4 must agree for the peripheral partition"
        );

        // R5 disabled in both paths (only one peripheral wired).
        assert_eq!(cached_p0[1].1, 0, "cache R5 disabled (no 2nd peripheral)");

        // ---- Simulate PendSV switching TO partition 1 (no peripheral) ----
        ds.configure_partition(1, &[(rbar_p1, rasr_p1)], 2).unwrap();
        let compute_p1 = ds.compute_region_values();
        let cached_p1 = ds.cached_peripheral_regions(1);

        // CRITICAL: compute_region_values R4-R5 may still reflect stale
        // slot contents from the previous partition's configure call.
        // The cache must DIFFER from compute for R4 when partition 1
        // has no peripheral but partition 0's peripheral descriptor is
        // still in slot[0].
        //
        // After configure_partition(1, ..., 2), slot[0] is left as-is
        // (it belongs to partition 0's peripheral reservation), so
        // compute_region_values still emits partition 0's peripheral in R4.
        // The cache, keyed by pid=1 (no peripherals), must return disabled.
        let disabled_r4 = (4u32 | (1 << 4), 0u32);
        let disabled_r5 = (5u32 | (1 << 4), 0u32);
        assert_eq!(
            cached_p1[0], disabled_r4,
            "cache R4 must be disabled for non-peripheral partition 1"
        );
        assert_eq!(
            cached_p1[1], disabled_r5,
            "cache R5 must be disabled for non-peripheral partition 1"
        );

        // Prove the override is necessary: compute_region_values R4
        // differs from cached_peripheral_regions R4 for partition 1.
        assert_ne!(
            compute_p1[0], cached_p1[0],
            "compute R4 must differ from cache R4 when switching \
             from peripheral to non-peripheral partition — \
             the PendSV override is necessary"
        );
        assert_eq!(
            compute_p1[0].1, expected_periph_rasr,
            "compute R4 still holds stale peripheral RASR from partition 0"
        );

        // ---- Verify round-trip: switch back to partition 0 ----
        ds.configure_partition(0, &[(rbar_p0, rasr_p0)], 2).unwrap();
        let cached_p0_again = ds.cached_peripheral_regions(0);
        assert_eq!(
            cached_p0_again[0],
            (expected_periph_rbar, expected_periph_rasr),
            "cache R4 must restore peripheral on switch-back to partition 0"
        );
    }

    // ------------------------------------------------------------------
    // wire_boot_peripherals: MpuRegion.permissions is dead code
    // ------------------------------------------------------------------

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
        ds.configure_partition(0, &[(rbar_r6, rasr_r6)], 2).unwrap();
        assert_eq!(ds.wire_boot_peripherals(&[pcb0, pcb1]), 1);

        let desc0 = ds.cached_peripherals(0)[0].expect("p0 must have cached peripheral");
        let desc1 = ds.cached_peripherals(1)[0].expect("p1 must have cached peripheral");
        assert_eq!(
            desc0.permissions, desc1.permissions,
            "RASR must match despite different perms"
        );

        // Verify fixed Shareable Device attributes.
        let rasr = desc0.permissions;
        assert_eq!(rasr & 1, 1, "region must be enabled");
        assert_eq!((rasr >> 24) & 0x7, AP_FULL_ACCESS, "AP must be full-access");
        assert_eq!((rasr >> 28) & 1, 1, "XN must be set");
        assert_eq!((rasr >> 19) & 0x7, 0, "TEX must be 0");
        assert_eq!((rasr >> 16) & 0x7, 0b101, "S/C/B must be 101");
    }
}
