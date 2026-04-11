#![no_std]

/// STM32F207ZG core clock: 16 MHz HSI (default, no PLL).
pub const CORE_CLOCK_HZ: u32 = 16_000_000;

/// Internal flash base address.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// Flash size (1 MB).
pub const FLASH_SIZE: u32 = 0x0010_0000;

/// SRAM base: 128 KB contiguous (no CCM on F2).
pub const SRAM_BASE: u32 = 0x2000_0000;
/// SRAM size: 128 KB.
pub const SRAM_SIZE: u32 = 128 * 1024;

#[cfg(all(feature = "uses-kernel", not(test)))]
use kernel::kpanic as _;

#[cfg(all(not(feature = "uses-kernel"), not(test)))]
use panic_rtt_target as _;
