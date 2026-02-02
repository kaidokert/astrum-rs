//! MPU strategy abstraction for dynamic partition windowing.
//!
//! This module defines the [`MpuStrategy`] trait, allowing the kernel to
//! swap between static (compile-time) and dynamic (runtime) MPU region
//! management.  [`StaticStrategy`] delegates directly to the existing
//! [`crate::mpu`] helpers with no behaviour change.

#[cfg(not(test))]
use crate::mpu;

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
    /// expects exactly 3 pairs; implementations may return `Err` if the
    /// slice length is incorrect.
    fn configure_partition(&self, partition_id: u8, regions: &[(u32, u32)])
        -> Result<(), MpuError>;

    /// Dynamically add a temporary memory window.
    ///
    /// Returns the MPU region ID on success, or `None` if no free
    /// region slot is available.
    fn add_window(&self, base: u32, size: u32, permissions: u32) -> Option<u8>;

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

/// Static MPU strategy — delegates to existing [`mpu`] helpers.
///
/// This strategy applies exactly 3 pre-computed (RBAR, RASR) region pairs
/// using the same disable-program-enable sequence as
/// [`mpu::apply_partition_mpu`].
pub struct StaticStrategy;

/// The number of MPU regions the static layout requires.
const STATIC_REGION_COUNT: usize = 3;

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

    fn add_window(&self, _base: u32, _size: u32, _permissions: u32) -> Option<u8> {
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
    fn configure_partition_accepts_3_regions() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let regions = partition_mpu_regions(&pcb).unwrap();
        let strategy = StaticStrategy;

        // Passing the 3 computed regions should succeed.
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

        // Too many regions.
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0), (0, 0)]),
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
        assert_eq!(strategy.add_window(0x2000_0000, 256, 0), None);
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
        assert_eq!(strategy.add_window(0, 0, 0), None);
        assert_eq!(
            strategy.configure_partition(0, &[(0, 0), (0, 0), (0, 0)]),
            Ok(()),
        );
    }
}
