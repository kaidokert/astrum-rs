//! UART peripheral configuration types.
//!
//! These types are `#[repr(C)]` for stable ABI layout across partitions
//! and kernel boundaries.

/// Common baud rates plus an escape hatch for non-standard values.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BaudRate {
    B9600,
    B19200,
    B38400,
    B57600,
    B115200,
    Custom(u32),
}

impl BaudRate {
    /// Return the numeric baud rate in bits per second.
    pub const fn as_bps(self) -> u32 {
        match self {
            Self::B9600 => 9600,
            Self::B19200 => 19200,
            Self::B38400 => 38400,
            Self::B57600 => 57600,
            Self::B115200 => 115200,
            Self::Custom(bps) => bps,
        }
    }
}

/// Parity bit configuration.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

/// Data word size.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WordSize {
    Eight,
    Nine,
}

/// Stop bit configuration.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum StopBits {
    One,
    Two,
}

/// Complete UART configuration bundle.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UartConfig {
    pub baud_rate: BaudRate,
    pub parity: Parity,
    pub word_size: WordSize,
    pub stop_bits: StopBits,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl UartConfig {
    /// Create a default configuration: 115200 8N1.
    pub const fn new() -> Self {
        Self {
            baud_rate: BaudRate::B115200,
            parity: Parity::None,
            word_size: WordSize::Eight,
            stop_bits: StopBits::One,
        }
    }

    /// Builder-style setter for baud rate.
    pub const fn with_baud_rate(mut self, baud_rate: BaudRate) -> Self {
        self.baud_rate = baud_rate;
        self
    }

    /// Builder-style setter for parity.
    pub const fn with_parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    /// Builder-style setter for word size.
    pub const fn with_word_size(mut self, word_size: WordSize) -> Self {
        self.word_size = word_size;
        self
    }

    /// Builder-style setter for stop bits.
    pub const fn with_stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem;

    // --- Construction and field access ---

    #[test]
    fn default_config_is_115200_8n1() {
        let cfg = UartConfig::new();
        assert_eq!(cfg.baud_rate, BaudRate::B115200);
        assert_eq!(cfg.parity, Parity::None);
        assert_eq!(cfg.word_size, WordSize::Eight);
        assert_eq!(cfg.stop_bits, StopBits::One);
    }

    #[test]
    fn const_construction() {
        const CFG: UartConfig = UartConfig::new();
        assert_eq!(CFG.baud_rate, BaudRate::B115200);
    }

    #[test]
    fn builder_overrides() {
        let cfg = UartConfig::new()
            .with_baud_rate(BaudRate::B9600)
            .with_parity(Parity::Even)
            .with_word_size(WordSize::Nine)
            .with_stop_bits(StopBits::Two);
        assert_eq!(cfg.baud_rate, BaudRate::B9600);
        assert_eq!(cfg.parity, Parity::Even);
        assert_eq!(cfg.word_size, WordSize::Nine);
        assert_eq!(cfg.stop_bits, StopBits::Two);
    }

    #[test]
    fn custom_baud_rate() {
        let cfg = UartConfig::new().with_baud_rate(BaudRate::Custom(1_000_000));
        assert_eq!(cfg.baud_rate, BaudRate::Custom(1_000_000));
        assert_eq!(cfg.baud_rate.as_bps(), 1_000_000);
    }

    // --- BaudRate::as_bps ---

    #[test]
    fn baud_rate_as_bps_standard() {
        assert_eq!(BaudRate::B9600.as_bps(), 9600);
        assert_eq!(BaudRate::B19200.as_bps(), 19200);
        assert_eq!(BaudRate::B38400.as_bps(), 38400);
        assert_eq!(BaudRate::B57600.as_bps(), 57600);
        assert_eq!(BaudRate::B115200.as_bps(), 115200);
    }

    #[test]
    fn baud_rate_as_bps_custom() {
        assert_eq!(BaudRate::Custom(250_000).as_bps(), 250_000);
        assert_eq!(BaudRate::Custom(0).as_bps(), 0);
    }

    // --- Copy / Clone ---

    #[test]
    fn config_is_copy() {
        let a = UartConfig::new();
        let b = a; // Copy
        assert_eq!(a, b); // `a` still usable
    }

    #[test]
    #[allow(clippy::clone_on_copy)]
    fn config_is_clone() {
        let a = UartConfig::new().with_baud_rate(BaudRate::B9600);
        let b = a.clone();
        assert_eq!(a, b);
    }

    // --- PartialEq ---

    #[test]
    fn configs_with_different_fields_are_not_equal() {
        let a = UartConfig::new();
        let b = UartConfig::new().with_parity(Parity::Odd);
        assert_ne!(a, b);
    }

    // --- Enum variant distinctness ---

    #[test]
    fn all_enum_variants_distinct() {
        // BaudRate
        let baud: &[BaudRate] = &[
            BaudRate::B9600,
            BaudRate::B19200,
            BaudRate::B38400,
            BaudRate::B57600,
            BaudRate::B115200,
            BaudRate::Custom(0),
        ];
        for (i, a) in baud.iter().enumerate() {
            for (j, b) in baud.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
        // Parity
        assert_ne!(Parity::None, Parity::Even);
        assert_ne!(Parity::None, Parity::Odd);
        assert_ne!(Parity::Even, Parity::Odd);
        // WordSize & StopBits
        assert_ne!(WordSize::Eight, WordSize::Nine);
        assert_ne!(StopBits::One, StopBits::Two);
    }

    // --- repr(C) layout stability ---

    #[test]
    fn repr_c_sizes_and_alignment() {
        // BaudRate: tag (u32) + u32 payload
        assert_eq!(mem::size_of::<BaudRate>(), 8);
        assert_eq!(mem::align_of::<BaudRate>(), 4);
        // Fieldless repr(C) enums
        assert!(mem::size_of::<Parity>() <= 4);
        assert!(mem::size_of::<WordSize>() <= 4);
        assert!(mem::size_of::<StopBits>() <= 4);
        // UartConfig
        assert!(mem::size_of::<UartConfig>() >= 8);
        assert_eq!(mem::align_of::<UartConfig>(), 4);
    }

    #[test]
    fn uart_config_field_offsets_are_stable() {
        let cfg = UartConfig::new();
        let base = &cfg as *const UartConfig as usize;
        let baud_off = &cfg.baud_rate as *const BaudRate as usize - base;
        let parity_off = &cfg.parity as *const Parity as usize - base;
        let word_off = &cfg.word_size as *const WordSize as usize - base;
        let stop_off = &cfg.stop_bits as *const StopBits as usize - base;
        // repr(C): fields laid out in declaration order
        assert_eq!(baud_off, 0, "baud_rate must be at offset 0");
        assert!(parity_off > baud_off, "parity must follow baud_rate");
        assert!(word_off > parity_off, "word_size must follow parity");
        assert!(stop_off > word_off, "stop_bits must follow word_size");
    }

    // --- Debug formatting ---

    #[test]
    fn debug_format_contains_field_names() {
        extern crate alloc;
        let cfg = UartConfig::new();
        let dbg = alloc::format!("{cfg:?}");
        assert!(dbg.contains("baud_rate"), "missing baud_rate in debug");
        assert!(dbg.contains("parity"), "missing parity in debug");
        assert!(dbg.contains("word_size"), "missing word_size in debug");
        assert!(dbg.contains("stop_bits"), "missing stop_bits in debug");
    }
}
