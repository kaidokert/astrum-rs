//! Board-specific implementations.
//!
//! Each board module implements the `Board` trait from the parent crate.
//! Enable the corresponding feature flag to include a board.

#[cfg(feature = "board-nrf52840-dk")]
pub mod nrf52840_dk;
pub mod qemu_lm3s6965;
#[cfg(feature = "board-stm32f4-discovery")]
pub mod stm32f4_discovery;

#[cfg(feature = "board-nrf52840-dk")]
pub use nrf52840_dk::Nrf52840Dk;
#[cfg(feature = "board-qemu")]
pub use qemu_lm3s6965::QemuBoard;
#[cfg(feature = "board-stm32f4-discovery")]
pub use stm32f4_discovery::Stm32f4Discovery;
