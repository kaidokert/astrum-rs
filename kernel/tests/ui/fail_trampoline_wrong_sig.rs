// Compile-fail test: partition_trampoline! rejects a function whose
// signature does not match PartitionBody (extern "C" fn(u32) -> !).
//
// TODO: replace `extern crate std` + `fn main()` with a proper no_std
// harness (#![no_main] + #[panic_handler]) once trybuild supports
// RUSTFLAGS passthrough (see pass_trivial.rs for full rationale).

#![no_std]
extern crate std;

extern "C" fn wrong_sig() -> ! {
    loop {}
}

kernel::partition_trampoline!(wrong_entry => wrong_sig);

fn main() {}
