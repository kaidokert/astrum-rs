//! Safe typed wrapper layer for partition syscalls.
//!
//! This module re-exports [`SvcError`] and [`decode_rc`] from `rtos-traits`,
//! which provides ergonomic Rust types on top of the raw `u32` return codes
//! produced by SVC calls, converting the ABI-level error bit convention into
//! `Result<u32, SvcError>`.

pub use rtos_traits::api::{decode_rc, SvcError};
