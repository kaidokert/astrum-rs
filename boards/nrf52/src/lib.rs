#![no_std]

// Panic handler selection:
//
// Kernel builds (uses-kernel active): delegate to kernel::kpanic, which
// provides an inline halt-loop fallback (panic_backend = "halt", no panic-halt
// feature).  Registering panic_halt here too would cause a duplicate lang item
// error (upstream added an inline fallback to kpanic that also provides
// #[panic_handler]).
//
// Standalone examples (no kernel dep): register panic_halt directly.
#[cfg(all(not(test), feature = "uses-kernel"))]
use kernel::kpanic as _;

#[cfg(all(not(test), not(feature = "uses-kernel")))]
use panic_halt as _;

/// nRF52833 core clock frequency.
/// Default: 64 MHz internal oscillator (HFRC), no clock configuration needed.
pub const CORE_CLOCK_HZ: u32 = 64_000_000;

/// Flash base address (nRF52833 internal flash is mapped at address 0).
pub const FLASH_BASE: u32 = 0x0000_0000;

/// SRAM base address.
pub const SRAM_BASE: u32 = 0x2000_0000;

/// MPU data window size.  Actual SRAM is 128 KB; 256 KB is used so the
/// window also covers the upper half of flash where debug strings land.
pub const SRAM_SIZE: u32 = 256 * 1024;
