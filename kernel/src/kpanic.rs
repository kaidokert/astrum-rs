//! Kernel panic handler with compile-time backend selection.
//!
//! Re-exports the appropriate panic handler based on `panic_backend` cfg.
//! Examples can use `use kernel::kpanic as _;` for uniform panic handling.
//!
//! Backend selection is handled by build.rs which emits `panic_backend`:
//! - `semihosting`: Uses `panic_semihosting` (exits QEMU on panic)
//! - `rtt`: Uses `panic_rtt_target` (outputs via RTT)
//! - `halt`: Uses `panic_halt` (infinite loop, no output)

// Import the panic handler for the selected backend.
// The `use ... as _` pattern pulls in the #[panic_handler] without naming it.
// When consumers write `use kernel::kpanic as _;`, this module is processed
// and the panic handler is linked.
// Only active in no_std builds; tests use std's panic handler.

#[cfg(all(not(test), panic_backend = "semihosting", feature = "log-semihosting"))]
use panic_semihosting as _;

#[cfg(all(not(test), panic_backend = "rtt", feature = "log-rtt"))]
use panic_rtt_target as _;

#[cfg(all(not(test), panic_backend = "halt", feature = "panic-halt"))]
use panic_halt as _;

#[cfg(test)]
mod tests {
    //! Verify module structure compiles for all backend configurations.
    //! The actual panic handler is pulled in by `use kernel::kpanic as _;`
    //! in examples, so we just verify the module compiles.
    //!
    //! Note: We cannot test the actual re-exports here because:
    //! 1. Tests run with std which provides its own panic handler
    //! 2. The handler re-exports are gated on `not(test)`
    //!
    //! Real verification is done by building examples with each feature set.

    #[test]
    fn module_compiles() {
        // This test verifies that the kpanic module compiles.
        // The actual panic handler selection is tested by building
        // examples with different feature combinations.
    }
}
