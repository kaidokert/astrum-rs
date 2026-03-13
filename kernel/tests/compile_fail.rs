//! Compile-fail test harness.
//!
//! Uses `trybuild` to verify that certain misuses of the kernel API
//! produce compile-time errors.  Run with:
//!
//!     cargo test --test compile_fail
//!
//! Tests live in `tests/ui/`.  Files named `*.rs` are compiled; the
//! expected compiler output is in the corresponding `*.stderr` file.
//! Files without a `.stderr` companion are expected to compile
//! successfully.

// Gate the entire test binary to match the cfg in Cargo.toml so that
// building for the ARM target does not fail due to missing `trybuild`.
#![cfg(not(target_arch = "arm"))]

#[test]
fn compile_fail_tests() {
    let t = trybuild::TestCases::new();
    t.pass("tests/ui/pass_*.rs");
    t.compile_fail("tests/ui/fail_*.rs");
    t.compile_fail("tests/ui/oversized_partition_config.rs");
}
