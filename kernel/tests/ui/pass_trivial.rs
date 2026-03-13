// Minimal no_std compile-pass smoke test for the trybuild harness.
//
// Uses #![no_std] to opt out of the std prelude, verifying that
// kernel-like code relying only on `core` types will compile.
//
// TODO: provide a #[panic_handler] instead of linking std for the
// host runtime.  Currently impossible because:
//   1. The precompiled host `core` emits compile_error! when
//      panic != abort and std is absent (no_std + unwind).
//   2. Trybuild strips RUSTFLAGS (cargo.rs:47), so we cannot
//      inject `-C panic=abort`.
//   3. `-Zbuild-std=core` requires network access that trybuild's
//      `--offline` mode blocks.
//   4. #[panic_handler] and `extern crate std` are mutually
//      exclusive (duplicate lang item `panic_impl`).
// Once trybuild gains rustflags passthrough or build-std support,
// replace `extern crate std` with a bare #[panic_handler].

#![no_std]
extern crate std;

fn main() {}
