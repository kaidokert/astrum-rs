//! Board support for the LM3S6965 (Stellaris) — primarily targeting the
//! QEMU `lm3s6965evb` machine used for CI emulation.
//!
//! Provides peripheral wrappers for use from partition code via Approach D
//! (direct MPU-mapped register access).

#![cfg_attr(not(test), no_std)]

pub mod pwm;
