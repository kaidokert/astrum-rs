// Compile-fail test: PartitionSpec::new() rejects a bare fn item whose
// type has not been explicitly cast to PartitionEntry or PartitionBody.
// The `impl EntryPointFn` bound requires a concrete fn-pointer type, so
// passing the un-coerced fn item `body_fn` is a type error.
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

    // entry() must reject a PartitionBody-shaped fn (wrong arity)
    let _bad = kernel::PartitionSpec::entry(body_fn);
}
