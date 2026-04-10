#![cfg_attr(not(any(test, feature = "std")), no_std)]

pub mod macros;

pub mod api;
pub mod buf_syscall;
pub mod debug;
pub mod device;
pub mod fmt;
pub mod hal;
pub mod i2c;
pub mod ids;
pub mod isr_ring;
pub mod partition;
pub mod register;
pub mod spi;
#[cfg(any(test, feature = "mock-hal"))]
pub mod test_register;
pub mod syscall;
pub mod thread;
pub mod uart;
