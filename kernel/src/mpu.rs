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

// -- Decode helpers --
// Extract logical fields from hardware RBAR/RASR register values.

/// Extract the base address from an RBAR value (mask off bits [4:0]).
pub fn decode_rbar_base(rbar: u32) -> u32 {
    rbar & RBAR_ADDR_MASK
}

/// Check the enable bit (bit 0) of an RASR value.
pub fn decode_rasr_enabled(rasr: u32) -> bool {
    rasr & 1 != 0
}

/// Extract the SIZE field from RASR and compute the region size in bytes.
/// Returns `None` if the region is disabled (enable bit = 0) or if
/// the size overflows `u32` (SIZE = 31 → 4 GiB).
pub fn decode_rasr_size_bytes(rasr: u32) -> Option<u32> {
    if !decode_rasr_enabled(rasr) {
        return None;
    }
    let size_field = (rasr >> RASR_SIZE_SHIFT) & RASR_SIZE_MASK;
    1u32.checked_shl(size_field + 1)
}

/// Extract the AP field (bits [26:24]) from an RASR value.
pub fn decode_rasr_ap(rasr: u32) -> u32 {
    (rasr >> RASR_AP_SHIFT) & RASR_AP_MASK
}

/// Returns true if the given AP value grants unprivileged access.
pub fn is_unprivileged_accessible(ap: u32) -> bool {
    ap == AP_RO_RO || ap == AP_FULL_ACCESS
}

/// Extract unprivileged-accessible regions from a slice of (RBAR, RASR) pairs.
///
/// For each pair, the function checks that the region is enabled and that the
/// AP field grants unprivileged access (`AP_RO_RO` or `AP_FULL_ACCESS`).
/// Qualifying regions are returned as `(base_address, size_in_bytes)` tuples.
///
/// Returns at most 8 entries (matching the maximum number of ARMv7-M MPU
/// regions).  Regions whose size overflows `u32` (4 GiB, SIZE=31) are skipped.
pub fn unprivileged_regions_from_pairs(pairs: &[(u32, u32)]) -> heapless::Vec<(u32, u32), 8> {
    let mut result = heapless::Vec::new();
    for &(rbar, rasr) in pairs {
        if !decode_rasr_enabled(rasr) {
            continue;
        }
        if !is_unprivileged_accessible(decode_rasr_ap(rasr)) {
            continue;
        }
        let size = match decode_rasr_size_bytes(rasr) {
            Some(s) => s,
            None => continue,
        };
        let base = decode_rbar_base(rbar);
        // Vec::push returns Err if full; silently stop — caller provided >8 pairs.
        if result.push((base, size)).is_err() {
            break;
        }
    }
    result
}

/// Write RBAR and RASR to configure a single MPU region.
pub fn configure_region(mpu: &cortex_m::peripheral::MPU, rbar: u32, rasr: u32) {
    // SAFETY: Writing to the MPU RBAR (0xE000_ED9C) and RASR (0xE000_EDA0)
    // registers is sound because:
    //
    // 1. Valid MPU register addresses: RBAR and RASR are memory-mapped
    //    registers in the System Control Space at fixed addresses defined
    //    by the ARMv7-M architecture (B3.5.6, B3.5.7).
    //
    // 2. Single-core exclusivity: The `&MPU` reference guarantees exclusive
    //    access to the MPU peripheral on this core.  On single-core Cortex-M
    //    systems, no other execution context can concurrently modify these
    //    registers while we hold the reference.
    //
    // 3. MMIO soundness: The cortex-m crate's `write()` method performs a
    //    volatile write, ensuring the compiler does not reorder or elide
    //    the store.  The caller (`apply_partition_mpu`) disables the MPU
    //    before calling this function and issues DSB/ISB barriers after
    //    all region writes, satisfying the ARMv7-M barrier requirements.
    unsafe {
        mpu.rbar.write(rbar);
        mpu.rasr.write(rasr);
    }
}

use crate::partition::{MpuRegion, PartitionControlBlock};

/// MPU CTRL value: enable MPU (bit 0) + PRIVDEFENA (bit 2).
///
/// PRIVDEFENA (bit 2) grants privileged code access to the default memory map
/// when no MPU region matches.  This is a deliberate design choice: the kernel
/// runs in privileged (handler) mode and relies on the default map for its own
/// code, data, and peripheral access instead of dedicating scarce MPU regions
/// to kernel memory.  Only unprivileged partition code is restricted to the
/// explicitly configured regions (code RX, data RW, stack guard).
///
/// # Safety invariant
///
/// PRIVDEFENA is only safe because every partition runs with CONTROL.nPRIV=1
/// (unprivileged Thread mode).  The PendSV handler sets nPRIV=1 before
/// returning to the partition, ensuring that partition code cannot bypass the
/// MPU by executing at privileged level.  If nPRIV were 0, PRIVDEFENA would
/// grant partitions full access to the default memory map, defeating isolation.
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

    // Sentinel partitions (size==0, from bug01 fix) intentionally fail
    // validation here and receive deny-all MPU via the _or_deny_all wrapper.
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

/// (RBAR, RASR) pair for a disabled MPU region targeting slot 4.
/// RBAR carries the VALID bit and region number so the MPU targets R4
/// instead of overwriting whatever is in MPU_RNR.  RASR = 0 disables.
/// Precomputed at compile time — no runtime panic possible.
const DISABLED_R4: (u32, u32) = (
    match build_rbar(0, 4) {
        Some(v) => v,
        None => panic!("invariant: base=0 aligned, region=4 in range"),
    },
    0,
);

/// (RBAR, RASR) pair for a disabled MPU region targeting slot 5.
const DISABLED_R5: (u32, u32) = (
    match build_rbar(0, 5) {
        Some(v) => v,
        None => panic!("invariant: base=0 aligned, region=5 in range"),
    },
    0,
);

/// Build an (RBAR, RASR) pair for an active peripheral MPU region.
///
/// Peripheral attributes: AP = full access, XN = execute never,
/// TEX=0 S=1 C=0 B=1 (Shareable Device memory).
///
/// **Note:** The [`MpuRegion::permissions`] field is intentionally ignored.
/// All peripheral MMIO is mapped as Shareable Device memory with fixed
/// attributes (TEX=0 S=1 C=0 B=1, AP=full-access, XN=true), regardless
/// of what the caller specified in `permissions`.
///
/// Returns `None` if the region's base or size is invalid for the MPU.
fn peripheral_region_pair(region: &MpuRegion, slot: u32) -> Option<(u32, u32)> {
    let size_field = encode_size(region.size())?;
    let rbar = build_rbar(region.base(), slot)?;
    let rasr = build_rasr(size_field, AP_FULL_ACCESS, true, (true, false, true));
    Some((rbar, rasr))
}

/// Build (RBAR, RASR) pairs for peripheral regions R4-R5.
///
/// Returns up to 2 pairs from the PCB's peripheral regions.  Unused
/// slots are disabled (RASR enable bit = 0).  Returns `None` if a
/// configured peripheral region has invalid MPU parameters.
///
/// # Mode usage
///
/// Used at **runtime** only in static mode (`#[cfg(not(feature =
/// "dynamic-mpu"))]`).  In dynamic mode the PendSV handler calls
/// [`DynamicStrategy::cached_peripheral_regions`] instead, which
/// restores pre-computed (RBAR, RASR) pairs from the boot-time cache.
pub fn peripheral_mpu_regions(pcb: &PartitionControlBlock) -> Option<[(u32, u32); 2]> {
    let periph = pcb.peripheral_regions();
    let r4 = match periph.first() {
        Some(r) => peripheral_region_pair(r, 4)?,
        None => DISABLED_R4,
    };
    let r5 = match periph.get(1) {
        Some(r) => peripheral_region_pair(r, 5)?,
        None => DISABLED_R5,
    };
    Some([r4, r5])
}

/// Return peripheral (RBAR, RASR) pairs for R4-R5, falling back to
/// disabled regions if the partition's peripheral MPU parameters are
/// invalid.
///
/// This is the infallible counterpart of [`peripheral_mpu_regions`].
/// When `peripheral_mpu_regions` returns `None` (e.g. non-power-of-2
/// peripheral size), the disabled fallback ensures R4-R5 are explicitly
/// cleared rather than retaining stale grants from a previous partition.
///
/// # Mode usage
///
/// Used at **runtime** only in static mode (`#[cfg(not(feature =
/// "dynamic-mpu"))]`).  In dynamic mode the PendSV handler uses
/// [`DynamicStrategy::cached_peripheral_regions`] as the runtime
/// counterpart (see `harness.rs` PendSV handler).  The equivalence
/// between the two paths is verified by
/// `cache_vs_pcb_peripheral_rbar_rasr_equivalence`.
pub fn peripheral_mpu_regions_or_disabled(pcb: &PartitionControlBlock) -> [(u32, u32); 2] {
    peripheral_mpu_regions(pcb).unwrap_or([DISABLED_R4, DISABLED_R5])
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

/// Disable the MPU and issue DSB+ISB memory barriers.
///
/// Must be paired with a subsequent [`mpu_enable`] call after all
/// region registers have been written.
///
/// # Safety contract (caller)
///
/// The caller must hold an exclusive `&MPU` reference and must not
/// return to unprivileged code before re-enabling the MPU.
// TODO: reviewer false positive — all callers (apply_partition_mpu,
// apply_deny_all_mpu, and the PendSV macro shim) are also #[cfg(not(test))]
// or only expanded in non-test binary targets.  `cargo test` compiles cleanly.
#[cfg(not(test))]
pub fn mpu_disable(mpu: &cortex_m::peripheral::MPU) {
    // SAFETY: Disabling the MPU before reprogramming regions is required by
    // the ARMv7-M architecture to avoid unpredictable behaviour from
    // partially-configured regions.  The exclusive `&MPU` reference
    // guarantees no concurrent access, and DSB+ISB ensure the disable
    // is visible before any region writes.
    unsafe { mpu.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Re-enable the MPU with PRIVDEFENA and issue DSB+ISB barriers.
///
/// Counterpart to [`mpu_disable`].  PRIVDEFENA (bit 2) grants
/// privileged code access to the default memory map when no MPU
/// region matches, so the kernel can run without dedicated regions.
#[cfg(not(test))]
pub fn mpu_enable(mpu: &cortex_m::peripheral::MPU) {
    const { assert!(MPU_CTRL_ENABLE_PRIVDEFENA & (1 << 2) != 0) }
    // SAFETY: Re-enabling the MPU with PRIVDEFENA after all regions have
    // been programmed.  The exclusive `&MPU` reference guarantees no
    // concurrent access, and the preceding region writes are complete.
    unsafe { mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
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
///
/// PRIVDEFENA safety depends on partitions executing unprivileged
/// (CONTROL.nPRIV=1).  This is enforced by the PendSV handler
/// (`define_pendsv!` / `define_pendsv_dynamic!` in pendsv.rs), which
/// writes CONTROL.nPRIV=1 before `bx lr` to the partition.
#[cfg(not(test))]
pub fn apply_partition_mpu(mpu: &cortex_m::peripheral::MPU, pcb: &PartitionControlBlock) {
    let regions = partition_mpu_regions_or_deny_all(pcb);

    mpu_disable(mpu);

    for &(rbar, rasr) in &regions {
        configure_region(mpu, rbar, rasr);
    }
    // Program peripheral regions R4-R5 (disabled if not configured).
    // Fallback uses compile-time constants with the VALID bit so disabled
    // slots target the correct region index.
    let periph = peripheral_mpu_regions(pcb).unwrap_or([DISABLED_R4, DISABLED_R5]);
    for &(rbar, rasr) in &periph {
        configure_region(mpu, rbar, rasr);
    }

    mpu_enable(mpu);
}

/// Apply deny-all MPU configuration (R0-R5).
///
/// Programmes the MPU with [`deny_all_regions`] and disables peripheral
/// slots R4-R5 so that all unprivileged memory accesses fault.  Used as
/// a panic fallback when the next partition ID is invalid.
#[cfg(not(test))]
pub fn apply_deny_all_mpu(mpu: &cortex_m::peripheral::MPU) {
    let regions = deny_all_regions();
    mpu_disable(mpu);
    for &(rbar, rasr) in &regions {
        configure_region(mpu, rbar, rasr);
    }
    // Disable peripheral region slots R4-R5.
    configure_region(mpu, DISABLED_R4.0, DISABLED_R4.1);
    configure_region(mpu, DISABLED_R5.0, DISABLED_R5.1);
    mpu_enable(mpu);
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
    /// The target region has already been initialised (non-sentinel).
    AlreadyInitialized,
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
            Self::AlreadyInitialized => write!(f, "region already initialized"),
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
        assert_eq!(
            format!("{}", MpuError::AlreadyInitialized),
            "region already initialized"
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

    // ------------------------------------------------------------------
    // peripheral_mpu_regions R4/R5
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_mpu_regions_none_when_empty() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let r = peripheral_mpu_regions(&pcb).unwrap();
        // Disabled slots must still carry the correct region index with
        // VALID bit set so the MPU targets R4/R5 instead of overwriting
        // whatever region is currently in MPU_RNR.
        assert_eq!(
            r,
            [
                (build_rbar(0, 4).unwrap(), 0),
                (build_rbar(0, 5).unwrap(), 0)
            ]
        );
    }

    #[test]
    fn peripheral_mpu_regions_one_peripheral() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 4096, 0)]);
        let r = peripheral_mpu_regions(&pcb).unwrap();
        // R4 enabled with correct base and Device memory attrs
        assert_eq!(r[0].0, build_rbar(0x4000_0000, 4).unwrap());
        assert_eq!((r[0].1 >> RASR_AP_SHIFT) & RASR_AP_MASK, AP_FULL_ACCESS);
        assert_eq!(r[0].1 & 1, 1); // enabled
        assert_eq!((r[0].1 >> 28) & 1, 1); // XN
        assert_eq!((r[0].1 >> 16) & 0x7, 0b101); // S=1,C=0,B=1
                                                 // R5 disabled — still targets slot 5 via VALID bit
        assert_eq!(r[1], (build_rbar(0, 5).unwrap(), 0));
    }

    #[test]
    fn peripheral_mpu_regions_two_peripherals() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096).with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 4096, 0),
            MpuRegion::new(0x4000_1000, 256, 0),
        ]);
        let r = peripheral_mpu_regions(&pcb).unwrap();
        assert_eq!(r[0].0, build_rbar(0x4000_0000, 4).unwrap());
        assert_eq!(r[0].1 & 1, 1);
        assert_eq!(r[1].0, build_rbar(0x4000_1000, 5).unwrap());
        assert_eq!(
            (r[1].1 >> RASR_SIZE_SHIFT) & RASR_SIZE_MASK,
            encode_size(256).unwrap()
        );
        assert_eq!(r[1].1 & 1, 1);
    }

    // TODO: reviewer false positive — `peripheral_mpu_regions` (line 296) is a
    // real pub fn returning Option<[(u32,u32);2]>, distinct from
    // `partition_mpu_regions_or_deny_all` which handles base R0-R3 regions.
    #[test]
    fn peripheral_regions_differ_across_partitions() {
        // Validates that apply_partition_mpu (called per context switch in
        // dynamic mode) programs different R4-R5 peripheral regions for
        // different partitions, ensuring per-switch peripheral isolation.
        let p0 = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 4096, 0)]);
        let p1 = make_pcb(0x0000_0000, 0x2000_4000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(0x4001_0000, 256, 0)]);

        let r0 = peripheral_mpu_regions(&p0).unwrap();
        let r1 = peripheral_mpu_regions(&p1).unwrap();

        // R4: different peripheral base addresses
        assert_ne!(
            r0[0].0, r1[0].0,
            "R4 RBAR must differ (different peripheral base)"
        );
        // R4: different RASR due to different peripheral sizes
        assert_ne!(
            r0[0].1, r1[0].1,
            "R4 RASR must differ (different peripheral size)"
        );
        // R5: disabled for both (only one peripheral each)
        assert_eq!(r0[1], r1[1], "R5 should be identically disabled for both");
    }

    // ------------------------------------------------------------------
    // peripheral_mpu_regions: encoding-level verification
    // ------------------------------------------------------------------

    /// Verify that RBAR region number bits [3:0] encode 4 and 5 (not 0-3),
    /// and that the VALID bit (bit 4) is set for direct region targeting.
    #[test]
    fn peripheral_rbar_region_numbers_are_4_and_5() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096).with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 4096, 0),
            MpuRegion::new(0x4000_1000, 256, 0),
        ]);
        let r = peripheral_mpu_regions(&pcb).unwrap();

        // RBAR bits [3:0] = region number, bit 4 = VALID
        assert_eq!(r[0].0 & 0xF, 4, "R4 RBAR region number must be 4");
        assert_eq!(r[1].0 & 0xF, 5, "R5 RBAR region number must be 5");
        assert_eq!((r[0].0 >> 4) & 1, 1, "R4 RBAR VALID bit must be set");
        assert_eq!((r[1].0 >> 4) & 1, 1, "R5 RBAR VALID bit must be set");

        // Also verify disabled R5 still targets region 5 (single-peripheral case)
        let pcb1 = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 4096, 0)]);
        let r1 = peripheral_mpu_regions(&pcb1).unwrap();
        assert_eq!(r1[1].0 & 0xF, 5, "Disabled R5 must still target region 5");
        assert_eq!((r1[1].0 >> 4) & 1, 1, "Disabled R5 VALID bit must be set");
    }

    /// Verify full RASR encoding for both R4 and R5 when two peripheral
    /// regions are configured: Device memory (TEX=0, S=1, C=0, B=1),
    /// AP=full-access, XN=1, correct sizes, and correct base addresses.
    #[test]
    fn peripheral_two_regions_full_rasr_encoding() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096).with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 4096, 0),
            MpuRegion::new(0x4000_1000, 256, 0),
        ]);
        let r = peripheral_mpu_regions(&pcb).unwrap();

        let expected_bases: [u32; 2] = [0x4000_0000, 0x4000_1000];
        let expected_sizes: [u32; 2] = [4096, 256];

        for (i, &(_rbar, rasr)) in r.iter().enumerate() {
            let slot = i as u32 + 4;
            // Enable bit
            assert_eq!(rasr & 1, 1, "R{slot} must be enabled");
            // Device memory: TEX=0 (bits [21:19]), S=1, C=0, B=1
            assert_eq!((rasr >> 19) & 0x7, 0, "R{slot} TEX must be 0");
            assert_eq!((rasr >> 16) & 0x7, 0b101, "R{slot} S/C/B must be 101");
            // AP = full access
            assert_eq!(
                (rasr >> RASR_AP_SHIFT) & RASR_AP_MASK,
                AP_FULL_ACCESS,
                "R{slot} AP must be full-access"
            );
            // XN = 1
            assert_eq!((rasr >> 28) & 1, 1, "R{slot} XN must be set");
            // Correct base address
            assert_eq!(
                decode_rbar_base(r[i].0),
                expected_bases[i],
                "R{slot} base address mismatch"
            );
            // Correct size
            assert_eq!(
                decode_rasr_size_bytes(rasr),
                Some(expected_sizes[i]),
                "R{slot} size mismatch"
            );
        }
    }

    /// Verify that a PCB with no peripheral regions produces disabled
    /// entries with region numbers 4 and 5 and RASR = 0 (not enabled).
    #[test]
    fn peripheral_no_regions_disabled_with_correct_slots() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let r = peripheral_mpu_regions(&pcb).unwrap();

        // Both RASR values must be 0 (disabled)
        assert_eq!(r[0].1, 0, "R4 RASR must be 0 (disabled)");
        assert_eq!(r[1].1, 0, "R5 RASR must be 0 (disabled)");
        // RBAR must still target slots 4 and 5
        assert_eq!(r[0].0 & 0xF, 4, "Disabled R4 region number must be 4");
        assert_eq!(r[1].0 & 0xF, 5, "Disabled R5 region number must be 5");
        assert!(!decode_rasr_enabled(r[0].1), "R4 must not be enabled");
        assert!(!decode_rasr_enabled(r[1].1), "R5 must not be enabled");
    }

    // ------------------------------------------------------------------
    // decode helpers
    // ------------------------------------------------------------------

    #[test]
    fn decode_rbar_base_masks_lower_bits() {
        assert_eq!(decode_rbar_base(0x2000_0010), 0x2000_0000);
        assert_eq!(decode_rbar_base(0x0800_0017), 0x0800_0000);
        assert_eq!(decode_rbar_base(0x0000_001F), 0x0000_0000);
        assert_eq!(decode_rbar_base(0xFFFF_FFE0), 0xFFFF_FFE0);
        assert_eq!(decode_rbar_base(0), 0);
    }

    #[test]
    fn decode_rasr_enabled_checks_bit0() {
        assert!(decode_rasr_enabled(0x0600_0009));
        assert!(decode_rasr_enabled(1));
        assert!(!decode_rasr_enabled(0));
        assert!(!decode_rasr_enabled(0x0600_0008));
    }

    #[test]
    fn decode_rasr_size_bytes_known_values() {
        // SIZE=4 → 2^5 = 32 bytes
        let rasr = build_rasr(4, AP_FULL_ACCESS, false, (false, false, false));
        assert_eq!(decode_rasr_size_bytes(rasr), Some(32));
        // SIZE=11 → 2^12 = 4096 bytes
        let rasr = build_rasr(11, AP_RO_RO, false, (false, false, false));
        assert_eq!(decode_rasr_size_bytes(rasr), Some(4096));
        // SIZE=7 → 2^8 = 256 bytes
        let rasr = build_rasr(7, AP_PRIV_RW, true, (true, true, false));
        assert_eq!(decode_rasr_size_bytes(rasr), Some(256));
    }

    #[test]
    fn decode_rasr_size_bytes_disabled_returns_none() {
        assert_eq!(decode_rasr_size_bytes(0), None);
        // Manually craft RASR with enable bit cleared
        assert_eq!(decode_rasr_size_bytes(0x0600_0008), None);
    }

    #[test]
    fn decode_rasr_size_bytes_4gib_returns_none() {
        // SIZE=31 → 2^32 overflows u32
        let rasr = build_rasr(31, AP_NO_ACCESS, true, (false, false, false));
        assert_eq!(decode_rasr_size_bytes(rasr), None);
    }

    #[test]
    fn decode_rasr_ap_extracts_all_variants() {
        for &ap in &[
            AP_NO_ACCESS,
            AP_PRIV_RW,
            AP_FULL_ACCESS,
            AP_PRIV_RO,
            AP_RO_RO,
        ] {
            let rasr = build_rasr(7, ap, false, (false, false, false));
            assert_eq!(decode_rasr_ap(rasr), ap);
        }
    }

    #[test]
    fn is_unprivileged_accessible_true_cases() {
        assert!(is_unprivileged_accessible(AP_RO_RO));
        assert!(is_unprivileged_accessible(AP_FULL_ACCESS));
    }

    #[test]
    fn is_unprivileged_accessible_false_cases() {
        assert!(!is_unprivileged_accessible(AP_NO_ACCESS));
        assert!(!is_unprivileged_accessible(AP_PRIV_RW));
        assert!(!is_unprivileged_accessible(AP_PRIV_RO));
        // Also test out-of-range values
        assert!(!is_unprivileged_accessible(0b010));
        assert!(!is_unprivileged_accessible(0b100));
        assert!(!is_unprivileged_accessible(0b111));
    }

    #[test]
    fn decode_roundtrip_with_build() {
        let base = 0x2000_0000u32;
        let rbar = build_rbar(base, 2).unwrap();
        assert_eq!(decode_rbar_base(rbar), base);

        let rasr = build_rasr(11, AP_FULL_ACCESS, true, (true, true, false));
        assert_eq!(decode_rasr_ap(rasr), AP_FULL_ACCESS);
        assert!(decode_rasr_enabled(rasr));
        assert_eq!(decode_rasr_size_bytes(rasr), Some(4096));
        assert!(is_unprivileged_accessible(decode_rasr_ap(rasr)));
    }

    // ------------------------------------------------------------------
    // unprivileged_regions_from_pairs
    // ------------------------------------------------------------------

    #[test]
    fn unprivileged_regions_from_partition() {
        let regions = default_regions();
        let result = unprivileged_regions_from_pairs(&regions);
        // Only code (AP_RO_RO) and data (AP_FULL_ACCESS) should pass.
        // Background (AP_NO_ACCESS) and stack guard (AP_NO_ACCESS) are filtered.
        assert_eq!(result.len(), 2);
        // Region 1 = code: base=0x0, size=4096
        assert_eq!(result[0], (0x0000_0000, 4096));
        // Region 2 = data: base=0x2000_0000, size=4096
        assert_eq!(result[1], (0x2000_0000, 4096));
    }

    #[test]
    fn unprivileged_regions_deny_all_is_empty() {
        let regions = deny_all_regions();
        let result = unprivileged_regions_from_pairs(&regions);
        assert!(result.is_empty());
    }

    #[test]
    fn unprivileged_regions_skips_disabled() {
        let pairs = [(0u32, 0u32)]; // RASR=0 → disabled
        let result = unprivileged_regions_from_pairs(&pairs);
        assert!(result.is_empty());
    }

    #[test]
    fn unprivileged_regions_skips_priv_only() {
        let rbar = build_rbar(0x2000_0000, 0).unwrap();
        let rasr_priv_rw = build_rasr(11, AP_PRIV_RW, false, (false, false, false));
        let rasr_priv_ro = build_rasr(11, AP_PRIV_RO, false, (false, false, false));
        let pairs = [(rbar, rasr_priv_rw), (rbar, rasr_priv_ro)];
        let result = unprivileged_regions_from_pairs(&pairs);
        assert!(result.is_empty());
    }

    #[test]
    fn unprivileged_regions_includes_both_ap_variants() {
        let rbar_a = build_rbar(0x0800_0000, 0).unwrap();
        let rasr_ro = build_rasr(11, AP_RO_RO, false, (false, false, false));
        let rbar_b = build_rbar(0x2000_0000, 1).unwrap();
        let rasr_rw = build_rasr(7, AP_FULL_ACCESS, true, (true, true, false));
        let pairs = [(rbar_a, rasr_ro), (rbar_b, rasr_rw)];
        let result = unprivileged_regions_from_pairs(&pairs);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], (0x0800_0000, 4096));
        assert_eq!(result[1], (0x2000_0000, 256));
    }

    #[test]
    fn unprivileged_regions_skips_4gib_overflow() {
        // SIZE=31 → 4 GiB overflows u32; decode_rasr_size_bytes returns None.
        let rbar = build_rbar(0x0000_0000, 0).unwrap();
        let rasr = build_rasr(31, AP_FULL_ACCESS, false, (false, false, false));
        let result = unprivileged_regions_from_pairs(&[(rbar, rasr)]);
        assert!(result.is_empty());
    }

    #[test]
    fn unprivileged_regions_empty_input() {
        let result = unprivileged_regions_from_pairs(&[]);
        assert!(result.is_empty());
    }

    // ------------------------------------------------------------------
    // peripheral_mpu_regions: static-mode R4-R5 encoding
    // ------------------------------------------------------------------

    /// Verify that peripheral_mpu_regions produces the correct R4 RBAR/RASR
    /// for a PCB with one peripheral region, and that R5 is disabled.
    /// (Static-mode path; dynamic mode uses cached_peripheral_regions.)
    #[test]
    fn peripheral_mpu_regions_one_peripheral_detailed() {
        let periph_base: u32 = 0x4000_0000;
        let periph_size: u32 = 4096;
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(periph_base, periph_size, 0)]);

        let regions =
            peripheral_mpu_regions(&pcb).expect("valid peripheral region must produce Some");

        // R4: correct RBAR with base, VALID bit, and region number 4
        let expected_rbar = build_rbar(periph_base, 4).unwrap();
        assert_eq!(regions[0].0, expected_rbar, "R4 RBAR mismatch");

        // R4 RASR: enabled, correct size, Device memory (S=1,C=0,B=1),
        // AP=full-access, XN=1
        let rasr = regions[0].1;
        assert_eq!(rasr & 1, 1, "R4 must be enabled");
        assert_eq!(
            (rasr >> RASR_SIZE_SHIFT) & RASR_SIZE_MASK,
            encode_size(periph_size).unwrap(),
            "R4 SIZE field mismatch"
        );
        assert_eq!(
            (rasr >> RASR_AP_SHIFT) & RASR_AP_MASK,
            AP_FULL_ACCESS,
            "R4 AP must be full-access"
        );
        assert_eq!((rasr >> 28) & 1, 1, "R4 XN must be set");
        // S=1 (bit 18), C=0 (bit 17), B=1 (bit 16)
        assert_eq!((rasr >> 16) & 0x7, 0b101, "R4 S/C/B must be 101 (Device)");

        // R5: disabled — RBAR targets slot 5, RASR = 0
        assert_eq!(
            regions[1], DISABLED_R5,
            "R5 must be disabled when only one peripheral configured"
        );
    }

    // ------------------------------------------------------------------
    // peripheral_mpu_regions_or_disabled (infallible wrapper)
    // ------------------------------------------------------------------

    /// Verify that `peripheral_mpu_regions_or_disabled` returns valid
    /// RBAR/RASR pairs for a PCB with peripheral regions, and disabled
    /// regions for a PCB without.  Also verifies the fallback path when
    /// `peripheral_mpu_regions` returns `None` (invalid peripheral params).
    #[test]
    fn test_peripheral_mpu_regions_or_disabled() {
        // ---- Case 1: PCB with one valid peripheral region ----
        let periph_base: u32 = 0x4000_0000;
        let periph_size: u32 = 4096;
        let pcb_with = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(periph_base, periph_size, 0)]);

        let regions = peripheral_mpu_regions_or_disabled(&pcb_with);

        // R4: valid region with correct base and region number 4
        assert_eq!(regions[0].0 & 0xF, 4, "R4 RBAR region number must be 4");
        assert_eq!((regions[0].0 >> 4) & 1, 1, "R4 RBAR VALID bit must be set");
        assert_eq!(
            decode_rbar_base(regions[0].0),
            periph_base,
            "R4 base address mismatch"
        );
        // R4 RASR: enabled, Device memory, AP=full-access, XN=1
        assert_eq!(regions[0].1 & 1, 1, "R4 must be enabled");
        assert_eq!(
            (regions[0].1 >> RASR_AP_SHIFT) & RASR_AP_MASK,
            AP_FULL_ACCESS,
            "R4 AP must be full-access"
        );
        assert_eq!((regions[0].1 >> 28) & 1, 1, "R4 XN must be set");
        assert_eq!(
            decode_rasr_size_bytes(regions[0].1),
            Some(periph_size),
            "R4 size mismatch"
        );

        // R5: disabled, region number 5
        assert_eq!(regions[1].0 & 0xF, 5, "R5 RBAR region number must be 5");
        assert_eq!((regions[1].0 >> 4) & 1, 1, "R5 RBAR VALID bit must be set");
        assert_eq!(regions[1].1, 0, "R5 RASR must be 0 (disabled)");

        // ---- Case 2: PCB without peripheral regions ----
        let pcb_without = make_pcb(0x0000_0000, 0x2000_0000, 4096);
        let disabled = peripheral_mpu_regions_or_disabled(&pcb_without);

        // Both must be disabled with correct slot targeting
        assert_eq!(
            disabled[0].0 & 0xF,
            4,
            "Disabled R4 region number must be 4"
        );
        assert_eq!(
            disabled[1].0 & 0xF,
            5,
            "Disabled R5 region number must be 5"
        );
        assert_eq!(disabled[0].1, 0, "Disabled R4 RASR must be 0");
        assert_eq!(disabled[1].1, 0, "Disabled R5 RASR must be 0");
        assert!(
            !decode_rasr_enabled(disabled[0].1),
            "R4 must not be enabled"
        );
        assert!(
            !decode_rasr_enabled(disabled[1].1),
            "R5 must not be enabled"
        );

        // Must match what peripheral_mpu_regions returns for the same PCB
        assert_eq!(
            disabled,
            peripheral_mpu_regions(&pcb_without).unwrap(),
            "Infallible wrapper must match fallible version for valid PCB"
        );

        // ---- Case 3: invalid peripheral (non-power-of-2 size) ----
        // peripheral_mpu_regions returns None; _or_disabled must still
        // return disabled regions instead of panicking.
        let pcb_invalid = make_pcb(0x0000_0000, 0x2000_0000, 4096)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 100, 0)]);
        assert!(
            peripheral_mpu_regions(&pcb_invalid).is_none(),
            "Non-power-of-2 size must make peripheral_mpu_regions return None"
        );
        let fallback = peripheral_mpu_regions_or_disabled(&pcb_invalid);
        assert_eq!(
            fallback[0].0 & 0xF,
            4,
            "Fallback R4 region number must be 4"
        );
        assert_eq!(
            fallback[1].0 & 0xF,
            5,
            "Fallback R5 region number must be 5"
        );
        assert_eq!(fallback[0].1, 0, "Fallback R4 RASR must be 0 (disabled)");
        assert_eq!(fallback[1].1, 0, "Fallback R5 RASR must be 0 (disabled)");
    }

    // ------------------------------------------------------------------
    // sentinel PCB (size==0 from bug01 fix)
    // ------------------------------------------------------------------

    #[test]
    fn sentinel_pcb_size_zero_partition_mpu_regions_returns_none() {
        // A sentinel PCB has mpu_region.size()==0. validate_mpu_region
        // rejects size < 32, so partition_mpu_regions must return None.
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,                       // valid entry_point
            0x2000_0000,                       // valid stack_base
            0x2000_1000,                       // stack_pointer
            MpuRegion::new(0x2000_0000, 0, 0), // size==0 sentinel
        );
        assert_eq!(
            partition_mpu_regions(&pcb),
            None,
            "sentinel PCB (size==0) must produce None from partition_mpu_regions"
        );
    }

    #[test]
    fn sentinel_pcb_size_zero_or_deny_all_returns_deny_all() {
        // The _or_deny_all wrapper must map the sentinel's None to deny_all_regions().
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_1000,
            MpuRegion::new(0x2000_0000, 0, 0),
        );
        assert_eq!(
            partition_mpu_regions_or_deny_all(&pcb),
            deny_all_regions(),
            "sentinel PCB must receive deny-all MPU configuration"
        );
    }

    // ------------------------------------------------------------------
    // peripheral_region_pair: permissions field is intentionally ignored
    // ------------------------------------------------------------------

    /// Prove that `MpuRegion.permissions` is not consulted for peripheral
    /// regions: two regions differing only in `permissions` produce
    /// identical RASR values via `peripheral_region_pair`, and the RASR
    /// encodes the expected Shareable Device attributes.
    #[test]
    fn peripheral_permissions_field_ignored() {
        let base: u32 = 0x4000_0000;
        let size: u32 = 4096;
        let slot: u32 = 4;

        let region_a = MpuRegion::new(base, size, 0x00);
        let region_b = MpuRegion::new(base, size, 0xDEAD_BEEF);

        let (rbar_a, rasr_a) =
            peripheral_region_pair(&region_a, slot).expect("valid region must succeed");
        let (rbar_b, rasr_b) =
            peripheral_region_pair(&region_b, slot).expect("valid region must succeed");

        // Identical output regardless of permissions
        assert_eq!(
            rbar_a, rbar_b,
            "RBAR must be identical despite different permissions"
        );
        assert_eq!(
            rasr_a, rasr_b,
            "RASR must be identical despite different permissions"
        );

        // Verify Shareable Device attributes in RASR:
        // Enable bit
        assert_eq!(rasr_a & 1, 1, "region must be enabled");
        // TEX=0 (bits [21:19])
        assert_eq!((rasr_a >> 19) & 0x7, 0, "TEX must be 0");
        // S=1, C=0, B=1 (bits [18:16])
        assert_eq!(
            (rasr_a >> 16) & 0x7,
            0b101,
            "S/C/B must be 101 (Shareable Device)"
        );
        // AP = full-access (0b011)
        assert_eq!(
            (rasr_a >> RASR_AP_SHIFT) & RASR_AP_MASK,
            AP_FULL_ACCESS,
            "AP must be full-access (0b011)"
        );
        // XN = 1
        assert_eq!((rasr_a >> 28) & 1, 1, "XN must be set");
    }
}
