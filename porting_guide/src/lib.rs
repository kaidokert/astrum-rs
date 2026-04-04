//! Porting guide crate: Board trait and board-specific implementations.
//!
//! This crate provides a minimal `Board` trait that each BSP implements,
//! plus re-exports of kernel logging infrastructure.

#![no_std]

pub mod boards;

/// Re-export the kernel logging macro so board crates can use `klog!`
/// without depending on `kernel` directly.
pub use kernel::klog;

/// A no-op LED type for boards without user LEDs.
pub struct NoLed;

/// A no-op delay type for boards without a hardware delay source.
pub struct NoDelay;

/// Minimal board abstraction for porting the RTOS to new hardware.
///
/// Implementers provide optional LED and delay peripherals.
/// Boards without these peripherals use `NoLed` / `NoDelay`.
pub trait Board {
    /// LED peripheral type. Use `NoLed` if the board has no user LED.
    type Led;
    /// Delay peripheral type. Use `NoDelay` if the board has no delay source.
    type Delay;

    /// Initialize the board and return LED and delay peripherals.
    fn init() -> (Self::Led, Self::Delay);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify NoLed and NoDelay are constructible (zero-sized).
    #[test]
    fn no_led_no_delay_are_zst() {
        assert_eq!(core::mem::size_of::<NoLed>(), 0);
        assert_eq!(core::mem::size_of::<NoDelay>(), 0);
    }

    /// Verify the Board trait is object-safe enough to be named as a bound.
    /// (It is not object-safe due to associated types, but we can use it
    /// as a generic bound.)
    #[test]
    fn board_trait_usable_as_generic_bound() {
        fn _assert_board<B: Board>() {}
        _assert_board::<boards::qemu_lm3s6965::QemuBoard>();
    }

    /// Verify QemuBoard::init returns the expected no-op types.
    #[test]
    fn qemu_board_init_returns_no_ops() {
        let (led, delay) = boards::qemu_lm3s6965::QemuBoard::init();
        // Both are ZSTs — just verify they exist.
        let _ = led;
        let _ = delay;
    }
}
