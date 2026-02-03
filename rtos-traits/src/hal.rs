//! Re-exports of key async hardware-abstraction traits.
//!
//! BSP crates should import traits from here rather than depending on
//! `embedded-hal-async` / `embedded-io-async` directly. This ensures every
//! crate in the workspace uses the same trait versions.

pub use embedded_hal_async::digital::Wait;
pub use embedded_hal_async::i2c::I2c;
pub use embedded_hal_async::spi::SpiBus;
pub use embedded_io_async::{Read, Write};
