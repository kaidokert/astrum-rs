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
    fn rasr_bit_layout() {
        assert_eq!(build_rasr(7, 0b011, true, (true, true, false)), 0x1306_000F);
        assert_eq!(
            build_rasr(4, 0b110, false, (false, false, false)),
            0x0600_0009
        );
        assert_eq!(build_rasr(5, 0b001, false, (true, true, true)), 0x0107_000B);
    }
}
