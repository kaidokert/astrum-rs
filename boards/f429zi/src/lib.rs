#![no_std]

/// STM32F429ZI core clock frequency without HAL clock configuration.
/// Boots from the 16 MHz HSI oscillator by default (no PLL, no HSE).
/// Kernel examples run at this frequency. HAL-based examples configure
/// their own clocks and should not rely on this constant.
pub const CORE_CLOCK_HZ: u32 = 16_000_000;

/// SYSCLK for HAL examples that configure the PLL for USB operation.
/// HSE 8 MHz (NUCLEO-144 crystal) + PLL with require_pll48clk() gives
/// SYSCLK = 168 MHz (not the 180 MHz hardware max, which cannot satisfy
/// the 48 MHz USB PLL constraint simultaneously).
pub const USB_SYSCLK_HZ: u32 = 168_000_000;

/// Internal flash base address (AXI bus mapping, post-boot alias).
pub const FLASH_BASE: u32 = 0x0800_0000;
/// Flash size for MPU code region (2 MB, power-of-two for ARMv7-M MPU).
pub const FLASH_SIZE: u32 = 0x0020_0000;

/// Cortex-M EXC_RETURN value: return to Thread mode using PSP.
/// Writing this to LR and executing BX LR performs an exception return
/// that restores context from the process stack pointer.
pub const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;

/// Initial XPSR value for a new thread: Thumb bit set, all others zero.
/// Cortex-M always executes in Thumb state; clearing this bit faults.
pub const XPSR_THUMB: u32 = 0x0100_0000;

/// SRAM base: SRAM1 + SRAM2 + SRAM3 form a contiguous 256 KB window.
pub const SRAM_BASE: u32 = 0x2000_0000;

/// Contiguous SRAM window: SRAM1 (112 KB) + SRAM2 (16 KB) + SRAM3 (64 KB).
pub const SRAM_SIZE: u32 = 256 * 1024;

// Panic handler selection:
//
// Kernel builds (uses-kernel active): delegate to kernel::kpanic, which uses
// panic_rtt_target internally via kernel/log-rtt.  The kernel's kpanic module
// registers the #[panic_handler]; registering one here too would cause a
// duplicate lang item error (upstream added an inline fallback to kpanic).
//
// Standalone examples (no kernel dep): register panic_rtt_target directly so
// panics are visible over RTT (context_switch_*, mpu_basic, etc.).
//
// HAL builds (default feature): example file provides `use panic_halt as _`.
#[cfg(all(not(feature = "hal"), feature = "uses-kernel", not(test)))]
use kernel::kpanic as _;

#[cfg(all(not(feature = "hal"), not(feature = "uses-kernel"), not(test)))]
use panic_rtt_target as _;
