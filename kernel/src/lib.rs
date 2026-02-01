#![cfg_attr(not(test), no_std)]

pub mod context;
pub mod kernel;
pub mod mpu;
pub mod partition;
pub mod scheduler;
pub mod syscall;
pub mod tick;
