//! Integration tests for the adversarial test infrastructure module.
//!
//! This file includes the adversarial module from examples/ and runs
//! its unit tests via `cargo test`.

#[path = "../examples/adversarial/mod.rs"]
mod adversarial;

// Re-run the tests from the adversarial module.
// The #[cfg(test)] tests in mod.rs are automatically included.
