//! Kernel panic handler with compile-time backend selection.
//!
//! Provides a single-import panic handler for all feature combinations.
//! Examples can use `use kernel::kpanic as _;` for uniform panic handling.
//!
//! Backend selection is handled by build.rs which emits `panic_backend`:
//! - `semihosting`: Re-exports `panic_semihosting` (exits QEMU on panic)
//! - `rtt`: Re-exports `panic_rtt_target` (outputs via RTT)
//! - `halt` + `panic-halt` feature: Re-exports `panic_halt`
//! - `halt` without `panic-halt` feature: Inline halt-loop fallback

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

// Inline fallback: halt-loop when panic_backend is "halt" but the
// `panic-halt` crate feature is not enabled.  This makes kpanic a
// complete single-import solution for every feature combination.
#[cfg(all(
    not(test),
    target_arch = "arm",
    panic_backend = "halt",
    not(feature = "panic-halt")
))]
#[panic_handler]
#[inline(never)]
fn _fallback_panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    //! Verify module structure compiles for all backend configurations.
    //!
    //! Note: The actual panic handlers (re-exports and inline fallback) are
    //! gated on `not(test)` and `target_arch = "arm"`, so they cannot be
    //! exercised in host unit tests.  Real verification is done by:
    //! - `cargo check --lib --no-default-features` (no duplicate handler)
    //! - Building hw_integration with each feature set on ARM targets

    #[test]
    fn module_compiles() {
        // Verifies kpanic module compiles on host.  All four cfg paths
        // (semihosting, rtt, panic-halt, inline fallback) are gated on
        // not(test), so this confirms no syntax or type errors in the
        // module-level items that are visible during test builds.
    }
}
