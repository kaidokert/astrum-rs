//! Re-exports of virtual device types from `rtos_traits::device`.
//!
//! The canonical definitions live in the `rtos-traits` crate.
//! This module re-exports them so that `use crate::virtual_device::*`
//! continues to work throughout the kernel.

pub use rtos_traits::device::{DeviceError, DeviceRegistry, VirtualDevice};
