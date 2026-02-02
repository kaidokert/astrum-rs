//! MPU strategy abstraction for dynamic partition windowing.
//!
//! This module defines the [`MpuStrategy`] trait, allowing the kernel to
//! swap between static (compile-time) and dynamic (runtime) MPU region
//! management.  [`StaticStrategy`] delegates directly to the existing
//! [`crate::mpu`] helpers with no behaviour change.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

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
    fn configure_partition(&self, partition_id: u8, regions: &[(u32, u32)])
        -> Result<(), MpuError>;

    /// Dynamically add a temporary memory window.
    ///
    /// `owner` identifies the partition that owns the window.
    /// Returns the MPU region ID on success, or `None` if no free
    /// region slot is available.
    fn add_window(&self, base: u32, size: u32, permissions: u32, owner: u8) -> Option<u8>;

    /// Remove a previously added window by its region ID.
    fn remove_window(&self, region_id: u8);
}

/// Errors from MPU strategy operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuError {
    /// The caller supplied a region count that does not match the expected
    /// fixed layout (e.g. 3 regions for the static strategy).
    RegionCountMismatch,
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
    /// For occupied slots the RBAR encodes the descriptor's base address and
    /// region number, while RASR is the stored permissions word.  For empty
    /// slots RBAR selects the region number and RASR is 0 (disabled).
    ///
    /// This is the testable, hardware-free counterpart of [`Self::program_regions`].
    pub fn compute_region_values(&self) -> [(u32, u32); DYNAMIC_SLOT_COUNT] {
        with_cs(|cs| {
            let slots = self.slots.borrow(cs);
            let slots = slots.borrow();
            let mut out = [(0u32, 0u32); DYNAMIC_SLOT_COUNT];
            for (idx, slot) in slots.iter().enumerate() {
                let region = DYNAMIC_REGION_BASE as u32 + idx as u32;
                match slot {
                    Some(desc) => {
                        // build_rbar cannot fail: region 4-7 ≤ 7 and
                        // bases stored in WindowDescriptor are always
                        // 32-byte aligned (sourced from RBAR_ADDR_MASK).
                        let rbar = crate::mpu::build_rbar(desc.base, region)
                            .expect("invalid RBAR for dynamic slot");
                        out[idx] = (rbar, desc.permissions);
                    }
                    None => {
                        // Select the region via RBAR and disable via RASR=0.
                        let rbar =
                            crate::mpu::build_rbar(0, region).expect("invalid region number");
                        out[idx] = (rbar, 0);
                    }
                }
            }
            out
        })
    }

    /// Program regions R4-R7 into the MPU hardware.
    ///
    /// Uses [`mpu::configure_region`] to write each of the four dynamic
    /// slots' (RBAR, RASR) pairs, then issues DSB + ISB barriers to
    /// ensure the new configuration takes effect before any subsequent
    /// memory access.
    ///
    /// Regions R0-R3 (static background/code/data) are left unchanged.
    ///
    /// The caller must pass an `&MPU` reference obtained from the
    /// peripheral singleton to make the hardware dependency explicit.
    #[cfg(not(test))]
    pub fn program_regions(&self, mpu_periph: &cortex_m::peripheral::MPU) {
        let values = self.compute_region_values();

        for &(rbar, rasr) in &values {
            mpu::configure_region(mpu_periph, rbar, rasr);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

impl MpuStrategy for DynamicStrategy {
    fn configure_partition(
        &self,
        partition_id: u8,
        regions: &[(u32, u32)],
    ) -> Result<(), MpuError> {
        // The static regions (R0-R2) are handled elsewhere; we only
        // care about the single private-RAM region destined for R4.
        if regions.is_empty() {
            return Err(MpuError::RegionCountMismatch);
        }

        let (rbar, rasr) = regions[0];
        let base = rbar & crate::mpu::RBAR_ADDR_MASK;
        let size_field = (rasr >> crate::mpu::RASR_SIZE_SHIFT) & crate::mpu::RASR_SIZE_MASK;
        let size = 1u32 << (size_field + 1);

        with_cs(|cs| {
            self.slots.borrow(cs).borrow_mut()[0] = Some(WindowDescriptor {
                base,
                size,
                permissions: rasr,
                owner: partition_id,
            });
        });
        Ok(())
    }

    fn add_window(&self, base: u32, size: u32, permissions: u32, owner: u8) -> Option<u8> {
        with_cs(|cs| {
            let mut slots = self.slots.borrow(cs).borrow_mut();
            // Scan slots 1..3 (R5-R7) for a free entry.
            for (idx, slot) in slots.iter_mut().enumerate().skip(1) {
                if slot.is_none() {
                    *slot = Some(WindowDescriptor {
                        base,
                        size,
                        permissions,
                        owner,
                    });
                    return Some(DYNAMIC_REGION_BASE + idx as u8);
                }
            }
            None // All dynamic slots occupied.
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
    ) -> Result<(), MpuError> {
        let region_array: [(u32, u32); STATIC_REGION_COUNT] = regions
            .try_into()
            .map_err(|_| MpuError::RegionCountMismatch)?;

        apply_regions(&region_array);
        Ok(())
    }

    fn add_window(&self, _base: u32, _size: u32, _permissions: u32, _owner: u8) -> Option<u8> {
        // Static strategy does not support dynamic windows.
        None
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
        assert_eq!(strategy.configure_partition(0, &regions), Ok(()));
    }

    #[test]
    fn configure_partition_rejects_wrong_count() {
        let strategy = StaticStrategy;

        // Too few regions.
        assert_eq!(
            strategy.configure_partition(0, &[(0x0, 0x0)]),
            Err(MpuError::RegionCountMismatch),
        );

        // 3 regions is now too few (was the old count).
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0)]),
            Err(MpuError::RegionCountMismatch),
        );

        // Too many regions.
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]),
            Err(MpuError::RegionCountMismatch),
        );

        // Empty.
        assert_eq!(
            strategy.configure_partition(0, &[]),
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
        assert_eq!(strategy.configure_partition(0, &regions), Ok(()));
    }

    #[test]
    fn configure_partition_different_partitions() {
        let strategy = StaticStrategy;

        let pcb0 = make_pcb(0x0000_0000, 0x2000_0000, 1024);
        let r0 = partition_mpu_regions(&pcb0).unwrap();
        assert_eq!(strategy.configure_partition(0, &r0), Ok(()));

        let pcb1 = make_pcb(0x0000_0000, 0x2000_8000, 1024);
        let r1 = partition_mpu_regions(&pcb1).unwrap();
        assert_eq!(strategy.configure_partition(1, &r1), Ok(()));
    }

    // ------------------------------------------------------------------
    // add_window / remove_window on StaticStrategy
    // ------------------------------------------------------------------

    #[test]
    fn static_strategy_add_window_returns_none() {
        let strategy = StaticStrategy;
        assert_eq!(strategy.add_window(0x2000_0000, 256, 0, 0), None);
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
        assert_eq!(strategy.add_window(0, 0, 0, 0), None);
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0), (0, 0)]),
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
        assert_eq!(ds.configure_partition(2, &[(rbar, rasr)]), Ok(()));

        let desc = ds.slot(4).expect("R4 should be occupied");
        assert_eq!(desc.base, 0x2000_0000);
        assert_eq!(desc.size, 4096);
        assert_eq!(desc.permissions, rasr);
        assert_eq!(desc.owner, 2);
    }

    #[test]
    fn dynamic_configure_partition_rejects_empty() {
        let ds = DynamicStrategy::new();
        assert_eq!(
            ds.configure_partition(0, &[]),
            Err(MpuError::RegionCountMismatch),
        );
    }

    #[test]
    fn dynamic_add_window_allocates_r5_r6_r7() {
        let ds = DynamicStrategy::new();
        let r5 = ds.add_window(0x2001_0000, 256, 0xAA, 1);
        assert_eq!(r5, Some(5));

        let r6 = ds.add_window(0x2002_0000, 512, 0xBB, 2);
        assert_eq!(r6, Some(6));

        let r7 = ds.add_window(0x2003_0000, 1024, 0xCC, 3);
        assert_eq!(r7, Some(7));

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
        assert!(ds.add_window(0x2001_0000, 256, 0, 1).is_some()); // R5
        assert!(ds.add_window(0x2002_0000, 256, 0, 1).is_some()); // R6
        assert!(ds.add_window(0x2003_0000, 256, 0, 1).is_some()); // R7

        // Fourth dynamic window should fail — only 3 dynamic slots.
        assert_eq!(ds.add_window(0x2004_0000, 256, 0, 1), None);
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
        ds.configure_partition(1, &[r1]).unwrap();
        assert_eq!(ds.slot(4).unwrap().owner, 1);

        // Reconfigure with different partition.
        let r2 = data_region(0x2000_8000, 1024, 4);
        ds.configure_partition(3, &[r2]).unwrap();
        let d = ds.slot(4).unwrap();
        assert_eq!(d.owner, 3);
        assert_eq!(d.base, 0x2000_8000);
        assert_eq!(d.size, 1024);
    }

    #[test]
    fn dynamic_add_window_does_not_touch_r4() {
        let ds = DynamicStrategy::new();
        let r = data_region(0x2000_0000, 4096, 4);
        ds.configure_partition(0, &[r]).unwrap();

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
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, 1), Some(5));
        strategy.remove_window(5);
        assert_eq!(strategy.add_window(0x2001_0000, 256, 0, 1), Some(5));
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
        ds.configure_partition(0, &[(rbar_in, rasr_in)]).unwrap();

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
        ds.configure_partition(1, &[(rbar_r4, rasr_r4)]).unwrap();

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
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)]).unwrap();

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
        ds.configure_partition(0, &[(rbar_r4, rasr_r4)]).unwrap();

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
}
