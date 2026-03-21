// Compile-fail test: PartitionSpec::new() rejects a PartitionBody-typed
// function (extern "C" fn(u32) -> !) where it expects PartitionEntry
// (extern "C" fn() -> !).
//
// This ensures the type aliases provide compile-time safety at the API
// boundary.
//
// TODO: replace `extern crate std` + `fn main()` with a proper no_std
// harness (#![no_main] + #[panic_handler]) once trybuild supports
// RUSTFLAGS passthrough (see pass_trivial.rs for full rationale).

#![no_std]
extern crate std;

extern "C" fn body_fn(_arg: u32) -> ! {
    loop {}
}

fn main() {
    let _spec = kernel::partition::PartitionSpec::new(body_fn, 0);
}
