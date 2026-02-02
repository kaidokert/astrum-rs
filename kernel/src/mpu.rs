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
pub fn build_rbar(base: u32, region: u32) -> Option<u32> {
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
pub const MPU_CTRL_ENABLE_PRIVDEFENA: u32 = (1 << 2) | 1;

/// Compute three (RBAR, RASR) pairs for a partition's MPU layout.
/// Returns `None` if region size/base is invalid for the MPU.
///
/// Region 0 = background no-access (XN, 4 GiB),
/// Region 1 = code RX (priv+unpriv RO), Region 2 = data RW/XN. Higher number wins on overlap.
pub fn partition_mpu_regions(pcb: &PartitionControlBlock) -> Option<[(u32, u32); 3]> {
    let region_size = pcb.mpu_region().size();
    let size_field = encode_size(region_size)?;
    let bg_size_field = 31u32; // 4 GiB = 2^32 → SIZE field = 31

    let bg_rbar = build_rbar(0x0000_0000, 0)?;
    let bg_rasr = build_rasr(bg_size_field, AP_NO_ACCESS, true, (false, false, false));

    let code_rbar = build_rbar(pcb.entry_point(), 1)?;
    let code_rasr = build_rasr(size_field, AP_RO_RO, false, (false, false, false));

    let data_rbar = build_rbar(pcb.mpu_region().base(), 2)?;
    let data_rasr = build_rasr(size_field, AP_FULL_ACCESS, true, (true, true, false));

    Some([
        (bg_rbar, bg_rasr),
        (code_rbar, code_rasr),
        (data_rbar, data_rasr),
    ])
}

/// Configure MPU regions for a partition: disable MPU, program three
/// regions (code RX, data RW/XN, background no-access), re-enable with
/// PRIVDEFENA, and execute DSB/ISB barriers.
#[cfg(not(test))]
pub fn apply_partition_mpu(mpu: &cortex_m::peripheral::MPU, pcb: &PartitionControlBlock) {
    let regions = partition_mpu_regions(pcb).expect("invalid MPU config");

    unsafe { mpu.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    for &(rbar, rasr) in &regions {
        configure_region(mpu, rbar, rasr);
    }

    unsafe { mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
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
    fn default_regions() -> [(u32, u32); 3] {
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
    }

    #[test]
    fn partition_regions_invalid_size() {
        let pcb = make_pcb(0x0000_0000, 0x2000_0000, 100); // not power of 2
        assert!(partition_mpu_regions(&pcb).is_none());
    }

    #[test]
    fn mpu_ctrl_constant() {
        assert_eq!(MPU_CTRL_ENABLE_PRIVDEFENA, 0b101);
    }
}
