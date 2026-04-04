//! Board-specific implementations.
//!
//! Each board module implements the `Board` trait from the parent crate.
//! Enable the corresponding feature flag to include a board.

pub mod qemu_lm3s6965;

#[cfg(feature = "board-qemu")]
pub use qemu_lm3s6965::QemuBoard;
