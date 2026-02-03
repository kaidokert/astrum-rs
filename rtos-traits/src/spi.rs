//! SPI peripheral configuration types.
//!
//! These types are `#[repr(C)]` for stable ABI layout across partitions
//! and kernel boundaries.

/// SPI clock polarity and phase mode (CPOL/CPHA combinations).
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

/// Bit transmission order.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BitOrder {
    MsbFirst,
    LsbFirst,
}

/// Complete SPI configuration bundle.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpiConfig {
    pub mode: SpiMode,
    pub bit_order: BitOrder,
    /// Clock frequency in Hz.
    pub frequency_hz: u32,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl SpiConfig {
    /// Create a default configuration: Mode0, MSB-first, 1 MHz.
    pub const fn new() -> Self {
        Self {
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
            frequency_hz: 1_000_000,
        }
    }

    pub const fn with_mode(mut self, mode: SpiMode) -> Self {
        self.mode = mode;
        self
    }

    pub const fn with_bit_order(mut self, bit_order: BitOrder) -> Self {
        self.bit_order = bit_order;
        self
    }

    pub const fn with_frequency_hz(mut self, frequency_hz: u32) -> Self {
        self.frequency_hz = frequency_hz;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem;

    #[test]
    fn default_and_const_construction() {
        let cfg = SpiConfig::new();
        assert_eq!(cfg.mode, SpiMode::Mode0);
        assert_eq!(cfg.bit_order, BitOrder::MsbFirst);
        assert_eq!(cfg.frequency_hz, 1_000_000);
        assert_eq!(SpiConfig::default(), cfg);
        const CFG: SpiConfig = SpiConfig::new();
        assert_eq!(CFG.mode, SpiMode::Mode0);
    }

    #[test]
    fn builder_overrides_all_fields() {
        let cfg = SpiConfig::new()
            .with_mode(SpiMode::Mode3)
            .with_bit_order(BitOrder::LsbFirst)
            .with_frequency_hz(4_000_000);
        assert_eq!(cfg.mode, SpiMode::Mode3);
        assert_eq!(cfg.bit_order, BitOrder::LsbFirst);
        assert_eq!(cfg.frequency_hz, 4_000_000);
    }

    #[test]
    #[allow(clippy::clone_on_copy)]
    fn copy_clone_partial_eq() {
        let a = SpiConfig::new();
        let b = a;
        assert_eq!(a, b);
        assert_eq!(a, a.clone());
        assert_ne!(a, SpiConfig::new().with_mode(SpiMode::Mode1));
        assert_ne!(a, SpiConfig::new().with_bit_order(BitOrder::LsbFirst));
        assert_ne!(a, SpiConfig::new().with_frequency_hz(8_000_000));
    }

    #[test]
    fn enum_variants_distinct() {
        let modes = [
            SpiMode::Mode0,
            SpiMode::Mode1,
            SpiMode::Mode2,
            SpiMode::Mode3,
        ];
        for (i, a) in modes.iter().enumerate() {
            for (j, b) in modes.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
        assert_ne!(BitOrder::MsbFirst, BitOrder::LsbFirst);
    }

    #[test]
    fn repr_c_layout() {
        assert_eq!(mem::size_of::<SpiMode>(), 1);
        assert_eq!(mem::size_of::<BitOrder>(), 1);
        assert_eq!(mem::size_of::<SpiConfig>(), 8);
        assert_eq!(mem::align_of::<SpiConfig>(), 4);
    }
}
