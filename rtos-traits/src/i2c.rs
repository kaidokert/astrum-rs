//! I2C peripheral configuration types.
//!
//! These types are `#[repr(C)]` for stable ABI layout across partitions
//! and kernel boundaries.

/// I2C device address: 7-bit or 10-bit.
#[repr(C, u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum I2cAddress {
    SevenBit(u8),
    TenBit(u16),
}

/// I2C bus speed.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum I2cSpeed {
    /// Standard mode: 100 kHz.
    Standard100k,
    /// Fast mode: 400 kHz.
    Fast400k,
    /// Fast-mode Plus: 1 MHz.
    FastPlus1M,
}

impl I2cSpeed {
    /// Return the clock frequency in Hz.
    pub const fn as_hz(self) -> u32 {
        match self {
            Self::Standard100k => 100_000,
            Self::Fast400k => 400_000,
            Self::FastPlus1M => 1_000_000,
        }
    }
}

/// Complete I2C configuration bundle.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct I2cConfig {
    pub address: I2cAddress,
    pub speed: I2cSpeed,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cConfig {
    /// Create a default configuration: 7-bit address 0x00, standard 100 kHz.
    pub const fn new() -> Self {
        Self {
            address: I2cAddress::SevenBit(0),
            speed: I2cSpeed::Standard100k,
        }
    }

    pub const fn with_address(mut self, address: I2cAddress) -> Self {
        self.address = address;
        self
    }

    pub const fn with_speed(mut self, speed: I2cSpeed) -> Self {
        self.speed = speed;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem;

    #[test]
    fn default_and_const_construction() {
        let cfg = I2cConfig::new();
        assert_eq!(cfg.address, I2cAddress::SevenBit(0));
        assert_eq!(cfg.speed, I2cSpeed::Standard100k);
        assert_eq!(I2cConfig::default(), cfg);
        const CFG: I2cConfig = I2cConfig::new();
        assert_eq!(CFG.speed, I2cSpeed::Standard100k);
    }

    #[test]
    fn builder_overrides() {
        let cfg = I2cConfig::new()
            .with_address(I2cAddress::TenBit(0x3FF))
            .with_speed(I2cSpeed::FastPlus1M);
        assert_eq!(cfg.address, I2cAddress::TenBit(0x3FF));
        assert_eq!(cfg.speed, I2cSpeed::FastPlus1M);
    }

    #[test]
    fn speed_as_hz() {
        assert_eq!(I2cSpeed::Standard100k.as_hz(), 100_000);
        assert_eq!(I2cSpeed::Fast400k.as_hz(), 400_000);
        assert_eq!(I2cSpeed::FastPlus1M.as_hz(), 1_000_000);
    }

    #[test]
    #[allow(clippy::clone_on_copy)]
    fn copy_clone_partial_eq() {
        let a = I2cConfig::new();
        let b = a;
        assert_eq!(a, b);
        assert_eq!(a, a.clone());
        assert_ne!(a, I2cConfig::new().with_speed(I2cSpeed::Fast400k));
        assert_ne!(a, I2cConfig::new().with_address(I2cAddress::SevenBit(0x42)));
    }

    #[test]
    fn enum_variants_distinct() {
        assert_ne!(I2cAddress::SevenBit(0), I2cAddress::TenBit(0));
        assert_ne!(I2cAddress::SevenBit(1), I2cAddress::SevenBit(2));
        let speeds = [
            I2cSpeed::Standard100k,
            I2cSpeed::Fast400k,
            I2cSpeed::FastPlus1M,
        ];
        for (i, a) in speeds.iter().enumerate() {
            for (j, b) in speeds.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
    }

    #[test]
    fn repr_c_layout() {
        assert_eq!(mem::size_of::<I2cAddress>(), 4);
        assert_eq!(mem::align_of::<I2cAddress>(), 2);
        assert_eq!(mem::size_of::<I2cSpeed>(), 1);
        assert_eq!(mem::size_of::<I2cConfig>(), 6);
        assert_eq!(mem::align_of::<I2cConfig>(), 2);
    }
}
