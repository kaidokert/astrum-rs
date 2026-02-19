//! Kernel invariant assertions.
//!
//! This module provides runtime checks for kernel invariants. These checks
//! are only enabled in debug builds and tests to avoid overhead in release
//! builds.
//!
//! # Usage
//!
//! Call `assert_kernel_invariants()` at strategic points (e.g., after context
//! switches, syscall entry/exit) to verify the kernel's internal state is
//! consistent.

/// Assert all kernel invariants hold.
///
/// In debug builds and tests, this function performs runtime validation of
/// kernel state. In release builds, this is a no-op that compiles away.
///
/// # Panics
///
/// Panics if any kernel invariant is violated (debug/test builds only).
#[cfg(any(debug_assertions, test))]
pub fn assert_kernel_invariants() {
    // Invariant checks will be added in subsequent commits.
    // This is the entry point that will call individual check functions.
}

/// No-op version for release builds.
///
/// This function compiles to nothing, ensuring zero runtime overhead.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_kernel_invariants() {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_assert_kernel_invariants_does_not_panic() {
        // The stub should complete without panicking.
        // Once invariants are added, this test verifies the happy path.
        assert_kernel_invariants();
    }

    #[test]
    fn test_assert_kernel_invariants_is_callable_multiple_times() {
        // Invariant checks should be idempotent and safe to call repeatedly.
        for _ in 0..10 {
            assert_kernel_invariants();
        }
    }
}
