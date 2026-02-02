// -- RBAR field masks --
// See ARMv7-M Architecture Reference Manual, section B3.5.6 (MPU Region Base Address Register).

/// RBAR address mask: bits [31:5] hold the base address.
/// ARMv7-M ARM B3.5.6, Table B3-38: ADDR field = bits [31:5].
pub const RBAR_ADDR_MASK: u32 = !0x1F;

// -- RASR field positions and masks --
// See ARMv7-M Architecture Reference Manual, section B3.5.7 (MPU Region Attribute and Size Register).

/// Bit position of the SIZE field within RASR (bits [5:1]).
/// ARMv7-M ARM B3.5.7, Table B3-39: SIZE field = bits [5:1].
pub const RASR_SIZE_SHIFT: u32 = 1;

/// Mask for the 5-bit SIZE field (after shifting).
/// ARMv7-M ARM B3.5.7, Table B3-39: SIZE is a 5-bit field.
pub const RASR_SIZE_MASK: u32 = 0x1F;

/// Bit position of the AP field within RASR (bits [26:24]).
/// ARMv7-M ARM B3.5.7, Table B3-39: AP field = bits [26:24].
pub const RASR_AP_SHIFT: u32 = 24;

/// Mask for the 3-bit AP field (after shifting).
/// ARMv7-M ARM B3.5.7, Table B3-39: AP is a 3-bit field.
pub const RASR_AP_MASK: u32 = 0x7;

/// AP field: no access for any privilege level.
pub const AP_NO_ACCESS: u32 = 0b000;

/// AP field: privileged read-write, unprivileged no access.
pub const AP_PRIV_RW: u32 = 0b001;

/// AP field: privileged read-only, unprivileged no access.
pub const AP_PRIV_RO: u32 = 0b101;

/// AP field: privileged read-only, unprivileged read-only.
pub const AP_RO_RO: u32 = 0b110;

/// AP field: full read-write access (privileged + unprivileged).
pub const AP_FULL_ACCESS: u32 = 0b011;

/// Encode region size in bytes to the 5-bit RASR SIZE field (`log2(size) - 1`).
/// Returns `None` if `size_bytes` is not a power of 2 or is less than 32.
pub fn encode_size(size_bytes: u32) -> Option<u32> {
    if size_bytes < 32 || !size_bytes.is_power_of_two() {
        return None;
    }
    Some(size_bytes.trailing_zeros() - 1)
}

/// Build RBAR value: base address with VALID bit and region number (0..=7).
/// Returns `None` if `region > 7` or `base` has any of bits [4:0] set.
pub const fn build_rbar(base: u32, region: u32) -> Option<u32> {
    if region > 7 || base & 0x1F != 0 {
        return None;
    }
    Some(base | (1 << 4) | region)
}

/// Build RASR value from size field, access permissions, XN, and S/C/B bits.
pub fn build_rasr(size_field: u32, ap: u32, xn: bool, scb: (bool, bool, bool)) -> u32 {
    let (s, c, b) = scb;
    (u32::from(xn) << 28)
        | ((ap & 0x7) << 24)
        | (u32::from(s) << 18)
        | (u32::from(c) << 17)
        | (u32::from(b) << 16)
        | ((size_field & 0x1F) << 1)
        | 1
}

/// Write RBAR and RASR to configure a single MPU region.
pub fn configure_region(mpu: &cortex_m::peripheral::MPU, rbar: u32, rasr: u32) {
    unsafe {
        mpu.rbar.write(rbar);
        mpu.rasr.write(rasr);
    }
}

use crate::partition::PartitionControlBlock;

/// MPU CTRL value: enable MPU (bit 0) + PRIVDEFENA (bit 2).
///
/// PRIVDEFENA (bit 2) grants privileged code access to the default memory map
/// when no MPU region matches.  This is a deliberate design choice: the kernel
/// runs in privileged (handler) mode and relies on the default map for its own
/// code, data, and peripheral access instead of dedicating scarce MPU regions
/// to kernel memory.  Only unprivileged partition code is restricted to the
/// explicitly configured regions (code RX, data RW, stack guard).
pub const MPU_CTRL_ENABLE_PRIVDEFENA: u32 = (1 << 2) | 1;

/// Compute four (RBAR, RASR) pairs for a partition's MPU layout.
/// Returns `None` if region size/base is invalid for the MPU.
///
/// Region 0 = background no-access (XN, 4 GiB),
/// Region 1 = code RX (priv+unpriv RO),
/// Region 2 = data RW/XN,
/// Region 3 = 32-byte no-access stack guard at stack_base.
/// Higher region number wins on overlap.
pub fn partition_mpu_regions(pcb: &PartitionControlBlock) -> Option<[(u32, u32); 4]> {
    let region_size = pcb.mpu_region().size();

    // Validate code region (entry_point must be aligned to region_size).
    validate_mpu_region(pcb.entry_point(), region_size).ok()?;
    // Validate data region (base must be aligned to size).
    validate_mpu_region(pcb.mpu_region().base(), region_size).ok()?;
    // Validate stack guard region (stack_base must be 32-byte aligned).
    validate_mpu_region(pcb.stack_base(), 32).ok()?;

    let size_field = encode_size(region_size)?;
    let bg_size_field = 31u32; // 4 GiB = 2^32 → SIZE field = 31

    let bg_rbar = build_rbar(0x0000_0000, 0)?;
    let bg_rasr = build_rasr(bg_size_field, AP_NO_ACCESS, true, (false, false, false));

    let code_rbar = build_rbar(pcb.entry_point(), 1)?;
    let code_rasr = build_rasr(size_field, AP_RO_RO, false, (false, false, false));

    let data_rbar = build_rbar(pcb.mpu_region().base(), 2)?;
    let data_rasr = build_rasr(size_field, AP_FULL_ACCESS, true, (true, true, false));

    let guard_size_field = encode_size(32)?; // 32 bytes → SIZE field = 4
    let guard_rbar = build_rbar(pcb.stack_base(), 3)?;
    let guard_rasr = build_rasr(guard_size_field, AP_NO_ACCESS, true, (false, false, false));

    Some([
        (bg_rbar, bg_rasr),
        (code_rbar, code_rasr),
        (data_rbar, data_rasr),
        (guard_rbar, guard_rasr),
    ])
}

/// Build a deny-all MPU region set: region 0 is background no-access
/// covering the full 4 GiB address space (XN), regions 1-3 are disabled.
/// Used as a safe fallback when `partition_mpu_regions` returns `None`,
/// ensuring the faulting partition gets no memory access instead of panicking.
pub fn deny_all_regions() -> [(u32, u32); 4] {
    let bg_size_field = 31u32; // 4 GiB = 2^32 → SIZE field = 31

    // Precomputed at compile time — base=0x0 is aligned, region=0 is in range.
    const BG_RBAR: u32 = match build_rbar(0x0000_0000, 0) {
        Some(v) => v,
        None => panic!("invariant: base=0x0 aligned, region=0 in range"),
    };
    let bg_rasr = build_rasr(bg_size_field, AP_NO_ACCESS, true, (false, false, false));

    [
        (BG_RBAR, bg_rasr),
        (0, 0), // region 1 disabled (RASR enable bit = 0)
        (0, 0), // region 2 disabled
        (0, 0), // region 3 disabled
    ]
}

/// Return partition MPU regions, falling back to a deny-all configuration
/// if the partition's MPU parameters are invalid.
///
/// This is the testable counterpart of `apply_partition_mpu`.  When
/// `partition_mpu_regions` returns `None` (e.g. non-power-of-2 region size),
/// the deny-all fallback ensures the partition gets zero memory access
/// rather than causing a panic — critical for handler-mode safety where
/// panics are unrecoverable.
pub fn partition_mpu_regions_or_deny_all(pcb: &PartitionControlBlock) -> [(u32, u32); 4] {
    partition_mpu_regions(pcb).unwrap_or_else(deny_all_regions)
}

/// Index of the first dynamic region within the array returned by
/// [`partition_mpu_regions`].  Regions before this index (background,
/// code, stack guard) are static; the region at this index (data RW)
/// is the one forwarded to [`DynamicStrategy::configure_partition`].
const DYNAMIC_REGION_START: usize = 2;

/// Number of dynamic regions extracted from [`partition_mpu_regions`].
const DYNAMIC_REGION_COUNT: usize = 1;

/// Return the slice of dynamic (RBAR, RASR) pairs for a partition.
///
/// This encapsulates the knowledge of which entries in the 4-region
/// static layout are forwarded to the dynamic MPU strategy (currently
/// only the data region at index 2, destined for hardware region R4).
pub fn partition_dynamic_regions(
    pcb: &PartitionControlBlock,
) -> Option<[(u32, u32); DYNAMIC_REGION_COUNT]> {
    let regions = partition_mpu_regions(pcb)?;
    let mut out = [(0u32, 0u32); DYNAMIC_REGION_COUNT];
    out.copy_from_slice(
        &regions[DYNAMIC_REGION_START..DYNAMIC_REGION_START + DYNAMIC_REGION_COUNT],
    );
    Some(out)
}

/// Configure MPU regions for a partition: disable MPU, program four
/// regions (background no-access, code RX, data RW/XN, stack guard),
/// re-enable with PRIVDEFENA, and execute DSB/ISB barriers.
///
/// If the partition's MPU configuration is invalid, a deny-all fallback
/// is applied instead of panicking.  This is necessary because this
/// function is called from the SysTick handler (handler mode), where a
/// panic is unrecoverable — there is no unwinding, and a panic in an
/// ISR typically results in a HardFault double-fault lockup.
#[cfg(not(test))]
pub fn apply_partition_mpu(mpu: &cortex_m::peripheral::MPU, pcb: &PartitionControlBlock) {
    let regions = partition_mpu_regions_or_deny_all(pcb);

    // SAFETY: Disabling the MPU before reprogramming regions is required by
    // the ARMv7-M architecture to avoid unpredictable behaviour from
    // partially-configured regions.  We hold an exclusive `&MPU` reference,
    // and the subsequent DSB/ISB ensures the write is visible before any
    // region registers are modified.
    unsafe { mpu.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    for &(rbar, rasr) in &regions {
        configure_region(mpu, rbar, rasr);
    }

    // Verify PRIVDEFENA (bit 2) is set — the kernel relies on the default
    // memory map for privileged access.  Evaluated at compile time.
    const { assert!(MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 2) != 0) }

    // SAFETY: Re-enabling the MPU with PRIVDEFENA after all regions have been
    // programmed.  The exclusive `&MPU` reference guarantees no concurrent
    // access, and the preceding region writes are complete.  PRIVDEFENA allows
    // privileged code to use the default memory map, while unprivileged access
    // is restricted to the explicitly configured regions (or denied entirely
    // by the deny-all fallback).
    unsafe { mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Errors from MPU region validation and strategy operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuError {
    /// The caller supplied a region count that does not match the expected
    /// fixed layout (e.g. 4 regions for the static strategy).
    RegionCountMismatch,
    /// Requested region size is smaller than the 32-byte MPU minimum.
    SizeTooSmall,
    /// Requested region size is not a power of two.
    SizeNotPowerOfTwo,
    /// Base address is not aligned to the region size.
    BaseNotAligned,
    /// `base + size` overflows the 32-bit address space.
    AddressOverflow,
    /// All available MPU region slots are in use.
    SlotExhausted,
}

impl core::fmt::Display for MpuError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::RegionCountMismatch => write!(f, "region count mismatch"),
            Self::SizeTooSmall => write!(f, "size too small (minimum 32 bytes)"),
            Self::SizeNotPowerOfTwo => write!(f, "size is not a power of two"),
            Self::BaseNotAligned => write!(f, "base address not aligned to size"),
            Self::AddressOverflow => write!(f, "base + size overflows u32"),
            Self::SlotExhausted => write!(f, "no free MPU region slots"),
        }
    }
}

/// Validate MPU region parameters for alignment and size constraints.
///
/// Returns `Ok(())` if `base` and `size` satisfy all ARMv7-M MPU region
/// requirements, or the first applicable [`MpuError`] variant:
///
/// 1. `size >= 32` (minimum MPU region size)
/// 2. `size` is a power of two
/// 3. `base` is aligned to `size` (`base & (size - 1) == 0`)
/// 4. `base + size` does not overflow `u32`
pub fn validate_mpu_region(base: u32, size: u32) -> Result<(), MpuError> {
    if size < 32 {
        return Err(MpuError::SizeTooSmall);
    }
    if !size.is_power_of_two() {
        return Err(MpuError::SizeNotPowerOfTwo);
    }
    if base & (size - 1) != 0 {
        return Err(MpuError::BaseNotAligned);
    }
    if base.checked_add(size).is_none() {
        return Err(MpuError::AddressOverflow);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_size_valid() {
        assert_eq!(encode_size(32), Some(4));
        assert_eq!(encode_size(64), Some(5));
        assert_eq!(encode_size(256), Some(7));
        assert_eq!(encode_size(1 << 31), Some(30));
    }

    #[test]
    fn encode_size_invalid() {
        assert_eq!(encode_size(0), None);
        assert_eq!(encode_size(16), None);
        assert_eq!(encode_size(48), None);
    }

    #[test]
    fn rbar_valid_and_invalid() {
        assert_eq!(build_rbar(0x2000_0000, 0), Some(0x2000_0010));
        assert_eq!(build_rbar(0x0800_0000, 7), Some(0x0800_0000 | (1 << 4) | 7));
        assert_eq!(build_rbar(0x2000_0000, 8), None); // region out of range
        assert_eq!(build_rbar(0x2000_0001, 0), None); // misaligned
    }

    #[test]
    fn ap_constants_in_rasr() {
        // No-access region: AP_NO_ACCESS
        let rasr = build_rasr(7, AP_NO_ACCESS, true, (false, false, false));
        assert_eq!((rasr >> 24) & 0x7, AP_NO_ACCESS);

        // RX region: AP_PRIV_RO, XN=false
        let rasr = build_rasr(7, AP_PRIV_RO, false, (false, false, false));
        assert_eq!((rasr >> 24) & 0x7, AP_PRIV_RO);
        assert_eq!((rasr >> 28) & 1, 0); // XN=0

        // RW/XN region: AP_FULL_ACCESS, XN=true
        let rasr = build_rasr(7, AP_FULL_ACCESS, true, (true, true, false));
        assert_eq!((rasr >> 24) & 0x7, AP_FULL_ACCESS);
        assert_eq!((rasr >> 28) & 1, 1); // XN=1
    }

    #[test]
    fn rasr_bit_layout() {
        assert_eq!(build_rasr(7, 0b011, true, (true, true, false)), 0x1306_000F);
        assert_eq!(
            build_rasr(4, 0b110, false, (false, false, false)),
            0x0600_0009
        );
        assert_eq!(build_rasr(5, 0b001, false, (true, true, true)), 0x0107_000B);
    }

    use crate::partition::MpuRegion;

    fn make_pcb(entry: u32, data_base: u32, data_size: u32) -> PartitionControlBlock {
        PartitionControlBlock::new(
            0,
            entry,
            data_base,
            data_base.wrapping_add(data_size),
            MpuRegion::new(data_base, data_size, 0),
        )
    }

    /// Shared setup: default PCB (entry=0x0, data=0x2000_0000, 4 KiB) and its MPU regions.
    fn default_regions() -> [(u32, u32); 4] {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        partition_mpu_regions(&pcb).unwrap()
    }

    #[test]
    fn partition_regions_background_kernel_only() {
        let regions = default_regions();
        let (rbar, rasr) = regions[0]; // background = region 0 (lowest prio)
        assert_eq!(rbar, build_rbar(0x0000_0000, 0).unwrap());
        assert_eq!((rasr >> 24) & 0x7, AP_NO_ACCESS); // no access (kernel uses PRIVDEFENA)
        assert_eq!((rasr >> 28) & 1, 1); // XN=1
        assert_eq!((rasr >> 1) & 0x1F, 31); // 4 GiB size field
    }

    #[test]
    fn partition_regions_code_rx() {
        let regions = default_regions();
        let (rbar, rasr) = regions[1]; // code = region 1
        assert_eq!(rbar, build_rbar(0x0000_0000, 1).unwrap());
        assert_eq!((rasr >> 24) & 0x7, AP_RO_RO); // priv+unpriv read-only
        assert_eq!((rasr >> 28) & 1, 0); // XN=0 (executable)
    }

    #[test]
    fn partition_regions_data_rwxn() {
        let regions = default_regions();
        let (rbar, rasr) = regions[2]; // data = region 2 (highest prio)
        assert_eq!(rbar, build_rbar(0x2000_0000, 2).unwrap());
        assert_eq!((rasr >> 24) & 0x7, AP_FULL_ACCESS); // RW
        assert_eq!((rasr >> 28) & 1, 1); // XN=1
    }

    #[test]
    fn partition_regions_stack_guard() {
        let regions = default_regions();
        // Stack guard = region 3: RBAR = stack_base | VALID | region 3
        let (rbar, rasr) = regions[3];
        assert_eq!(rbar, build_rbar(0x2000_0000, 3).unwrap());
        // AP = no access
        assert_eq!((rasr >> RASR_AP_SHIFT) & RASR_AP_MASK, AP_NO_ACCESS);
        // XN = 1
        assert_eq!((rasr >> 28) & 1, 1);
        // SIZE field = 4 (32 bytes)
        assert_eq!((rasr >> RASR_SIZE_SHIFT) & RASR_SIZE_MASK, 4);
        // Enable bit set
        assert_eq!(rasr & 1, 1);
    }

    #[test]
    fn partition_regions_stack_guard_custom_base() {
        let pcb = make_pcb(0x0800_0000, 0x2000_4000, 1024);
        let regions = partition_mpu_regions(&pcb).unwrap();
        let (rbar, rasr) = regions[3];
        // Guard should be at stack_base = 0x2000_4000
        assert_eq!(rbar, build_rbar(0x2000_4000, 3).unwrap());
        // Same no-access, XN, 32-byte encoding
        let expected_rasr = build_rasr(
            encode_size(32).unwrap(),
            AP_NO_ACCESS,
            true,
            (false, false, false),
        );
        assert_eq!(rasr, expected_rasr);
    }

    #[test]
    fn partition_regions_different_partitions() {
        let p0 = make_pcb(0x0000_0000, 0x2000_0000, 1024);
        let p1 = make_pcb(0x0000_0000, 0x2000_8000, 1024);
        let r0 = partition_mpu_regions(&p0).unwrap();
        let r1 = partition_mpu_regions(&p1).unwrap();
        // Background regions identical
        assert_eq!(r0[0], r1[0]);
        // Code regions identical
        assert_eq!(r0[1], r1[1]);
        // Data regions differ in RBAR (different base)
        assert_ne!(r0[2].0, r1[2].0);
        // Data RASR identical (same size/permissions)
        assert_eq!(r0[2].1, r1[2].1);
        // Stack guard regions differ in RBAR (different stack_base)
        assert_ne!(r0[3].0, r1[3].0);
        // Stack guard RASR identical (same 32-byte no-access)
        assert_eq!(r0[3].1, r1[3].1);
    }

    #[test]
    fn partition_dynamic_regions_returns_data_region() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let full = partition_mpu_regions(&pcb).unwrap();
        let dynamic = partition_dynamic_regions(&pcb).unwrap();
        // Should return exactly the data region (index 2).
        assert_eq!(dynamic.len(), 1);
        assert_eq!(dynamic[0], full[2]);
        // Verify it is the RW data region, not the guard.
        let ap = (dynamic[0].1 >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_FULL_ACCESS);
    }

    #[test]
    fn partition_dynamic_regions_invalid_pcb() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 100); // not power of 2
        assert!(partition_dynamic_regions(&pcb).is_none());
    }

    #[test]
    fn partition_regions_invalid_size() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 100); // not power of 2
        assert!(partition_mpu_regions(&pcb).is_none());
    }

    #[test]
    fn mpu_ctrl_constant() {
        assert_eq!(MPU_CTRL_ENABLE_PRIVDEFENA, 0b101);

        // ENABLE bit (bit 0): turns the MPU on.
        assert_ne!(
            MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 0),
            0,
            "ENABLE (bit 0) must be set"
        );

        // PRIVDEFENA bit (bit 2): privileged code uses default memory map
        // when no MPU region matches, avoiding dedicated kernel regions.
        assert_ne!(
            MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 2),
            0,
            "PRIVDEFENA (bit 2) must be set"
        );
    }

    #[test]
    fn deny_all_regions_region0_is_background_no_access() {
        let regions = deny_all_regions();
        let (rbar, rasr) = regions[0];
        // Region 0: base=0x0, VALID, region number 0
        assert_eq!(rbar, build_rbar(0x0000_0000, 0).unwrap());
        // AP = no access
        assert_eq!((rasr >> RASR_AP_SHIFT) & RASR_AP_MASK, AP_NO_ACCESS);
        // XN = 1
        assert_eq!((rasr >> 28) & 1, 1);
        // SIZE field = 31 (4 GiB)
        assert_eq!((rasr >> RASR_SIZE_SHIFT) & RASR_SIZE_MASK, 31);
        // Enable bit set
        assert_eq!(rasr & 1, 1);
    }

    #[test]
    fn deny_all_regions_regions_1_to_3_disabled() {
        let regions = deny_all_regions();
        for (i, &(rbar, rasr)) in regions.iter().enumerate().skip(1) {
            assert_eq!(rbar, 0, "region {} RBAR should be 0", i);
            assert_eq!(rasr, 0, "region {} RASR should be 0 (disabled)", i);
        }
    }

    #[test]
    fn partition_mpu_regions_or_deny_all_valid_pcb() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let regions = partition_mpu_regions_or_deny_all(&pcb);
        let expected = partition_mpu_regions(&pcb).unwrap();
        assert_eq!(regions, expected);
    }

    #[test]
    fn partition_mpu_regions_or_deny_all_invalid_pcb_returns_deny_all() {
        // Non-power-of-2 size triggers None from partition_mpu_regions
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 100);
        let regions = partition_mpu_regions_or_deny_all(&pcb);
        let expected = deny_all_regions();
        assert_eq!(regions, expected);
    }

    #[test]
    fn partition_mpu_regions_or_deny_all_invalid_size_16() {
        // Size < 32 also triggers None
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 16);
        let regions = partition_mpu_regions_or_deny_all(&pcb);
        assert_eq!(regions, deny_all_regions());
    }

    // ------------------------------------------------------------------
    // validate_mpu_region
    // ------------------------------------------------------------------

    #[test]
    fn validate_mpu_region_rejects_size_too_small() {
        assert_eq!(validate_mpu_region(0, 0), Err(MpuError::SizeTooSmall));
        assert_eq!(validate_mpu_region(0, 1), Err(MpuError::SizeTooSmall));
        assert_eq!(validate_mpu_region(0, 16), Err(MpuError::SizeTooSmall));
        assert_eq!(validate_mpu_region(0, 31), Err(MpuError::SizeTooSmall));
    }

    #[test]
    fn validate_mpu_region_rejects_non_power_of_two() {
        assert_eq!(validate_mpu_region(0, 48), Err(MpuError::SizeNotPowerOfTwo));
        assert_eq!(
            validate_mpu_region(0, 100),
            Err(MpuError::SizeNotPowerOfTwo)
        );
        assert_eq!(
            validate_mpu_region(0, 255),
            Err(MpuError::SizeNotPowerOfTwo)
        );
    }

    #[test]
    fn validate_mpu_region_rejects_misaligned_base() {
        assert_eq!(validate_mpu_region(64, 256), Err(MpuError::BaseNotAligned));
        assert_eq!(
            validate_mpu_region(0x2000_0100, 4096),
            Err(MpuError::BaseNotAligned)
        );
    }

    #[test]
    fn validate_mpu_region_rejects_address_overflow() {
        assert_eq!(
            validate_mpu_region(0xFFFF_FF00, 256),
            Err(MpuError::AddressOverflow)
        );
        assert_eq!(
            validate_mpu_region(0x8000_0000, 0x8000_0000),
            Err(MpuError::AddressOverflow)
        );
    }

    #[test]
    fn validate_mpu_region_accepts_valid_params() {
        assert_eq!(validate_mpu_region(0, 32), Ok(()));
        assert_eq!(validate_mpu_region(32, 32), Ok(()));
        assert_eq!(validate_mpu_region(0x2000_0000, 256), Ok(()));
        assert_eq!(validate_mpu_region(0x2000_0000, 4096), Ok(()));
        assert_eq!(validate_mpu_region(0x4000_0000, 0x4000_0000), Ok(()));
    }

    #[test]
    fn validate_mpu_region_table_driven() {
        // Comprehensive table-driven test covering all MpuError variants,
        // valid cases, boundary conditions, and error-priority ordering.
        let cases: &[(u32, u32, Result<(), MpuError>)] = &[
            // --- SizeTooSmall: size < 32 ---
            (0, 0, Err(MpuError::SizeTooSmall)),
            (0, 1, Err(MpuError::SizeTooSmall)),
            (0, 16, Err(MpuError::SizeTooSmall)),
            (0, 31, Err(MpuError::SizeTooSmall)),
            // SizeTooSmall takes priority over SizeNotPowerOfTwo
            (0, 3, Err(MpuError::SizeTooSmall)),
            (0, 15, Err(MpuError::SizeTooSmall)),
            // --- SizeNotPowerOfTwo: size >= 32 but not power of two ---
            (0, 33, Err(MpuError::SizeNotPowerOfTwo)),
            (0, 48, Err(MpuError::SizeNotPowerOfTwo)),
            (0, 100, Err(MpuError::SizeNotPowerOfTwo)),
            (0, 255, Err(MpuError::SizeNotPowerOfTwo)),
            (0, 4000, Err(MpuError::SizeNotPowerOfTwo)),
            // --- BaseNotAligned: base & (size - 1) != 0 ---
            (1, 32, Err(MpuError::BaseNotAligned)),
            (64, 256, Err(MpuError::BaseNotAligned)),
            (0x2000_0100, 4096, Err(MpuError::BaseNotAligned)),
            (0x0000_1000, 0x0001_0000, Err(MpuError::BaseNotAligned)),
            // --- AddressOverflow: base + size > u32::MAX ---
            (0x8000_0000, 0x8000_0000, Err(MpuError::AddressOverflow)),
            (0xFFFF_FF00, 256, Err(MpuError::AddressOverflow)),
            (0xFFFF_0000, 0x0001_0000, Err(MpuError::AddressOverflow)),
            // --- Valid cases ---
            // Minimum valid region
            (0, 32, Ok(())),
            (32, 32, Ok(())),
            (0x2000_0000, 32, Ok(())),
            // Various valid power-of-two sizes with base=0
            (0, 64, Ok(())),
            (0, 1024, Ok(())),
            (0, 0x8000_0000, Ok(())),
            // Aligned bases with larger sizes
            (0x2000_0000, 256, Ok(())),
            (0x2000_0000, 4096, Ok(())),
            (0x2000_0000, 0x2000_0000, Ok(())),
            (0x4000_0000, 0x4000_0000, Ok(())),
            // Just under overflow boundary
            (0x7FFF_F000, 4096, Ok(())),
        ];

        for (i, &(base, size, ref expected)) in cases.iter().enumerate() {
            let result = validate_mpu_region(base, size);
            assert_eq!(
                result, *expected,
                "case {i}: validate_mpu_region({base:#010X}, {size:#010X}) = {result:?}, expected {expected:?}"
            );
        }
    }

    #[test]
    fn mpu_error_display_messages() {
        assert_eq!(
            format!("{}", MpuError::RegionCountMismatch),
            "region count mismatch"
        );
        assert_eq!(
            format!("{}", MpuError::SizeTooSmall),
            "size too small (minimum 32 bytes)"
        );
        assert_eq!(
            format!("{}", MpuError::SizeNotPowerOfTwo),
            "size is not a power of two"
        );
        assert_eq!(
            format!("{}", MpuError::BaseNotAligned),
            "base address not aligned to size"
        );
        assert_eq!(
            format!("{}", MpuError::AddressOverflow),
            "base + size overflows u32"
        );
        assert_eq!(
            format!("{}", MpuError::SlotExhausted),
            "no free MPU region slots"
        );
    }

    #[test]
    fn mpu_error_debug_contains_variant_name() {
        assert!(format!("{:?}", MpuError::SizeTooSmall).contains("SizeTooSmall"));
        assert!(format!("{:?}", MpuError::SlotExhausted).contains("SlotExhausted"));
    }

    // ------------------------------------------------------------------
    // partition_mpu_regions: validate_mpu_region integration
    // ------------------------------------------------------------------

    #[test]
    fn partition_regions_misaligned_entry_point_returns_none() {
        // entry_point=0x100 is not aligned to 4096-byte region size
        let pcb = make_pcb(0x0000_0100, 0x2000_0000, 4096);
        assert!(partition_mpu_regions(&pcb).is_none());
    }

    #[test]
    fn partition_regions_misaligned_data_base_returns_none() {
        // data base=0x2000_0100 is not aligned to 4096-byte region size
        let pcb = make_pcb(0x0000_0000, 0x2000_0100, 4096);
        assert!(partition_mpu_regions(&pcb).is_none());
    }

    #[test]
    fn partition_regions_misaligned_stack_base_returns_none() {
        // stack_base must be 32-byte aligned; 0x2000_0010 + 4 = 0x2000_0014 is not
        let entry = 0x0000_0000;
        let data_base = 0x2000_0000;
        let data_size = 4096u32;
        // Use a stack_base that is NOT 32-byte aligned
        let misaligned_stack_base = data_base + 4; // 0x2000_0004, not 32-byte aligned
        let pcb = PartitionControlBlock::new(
            0,
            entry,
            misaligned_stack_base,
            misaligned_stack_base.wrapping_add(data_size),
            MpuRegion::new(data_base, data_size, 0),
        );
        assert!(partition_mpu_regions(&pcb).is_none());
    }

    #[test]
    fn partition_regions_entry_point_overflow_returns_none() {
        // entry_point near end of address space + large region causes overflow
        let pcb = make_pcb(0xFFFF_0000, 0x2000_0000, 0x0001_0000);
        assert!(partition_mpu_regions(&pcb).is_none());
    }
}
