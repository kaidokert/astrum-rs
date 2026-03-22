// Compile-pass test: PartitionSpec::entry() accepts a bare fn item
// without requiring an explicit `as PartitionEntry` cast.
//
// Verifies that both the kernel and plib crate re-exports expose
// entry() and body() constructors.

#![no_std]
extern crate std;

extern "C" fn my_entry() -> ! {
    loop {}
}

extern "C" fn my_body(_arg: u32) -> ! {
    loop {}
}

/// Wraps constructor calls so they are type-checked but never executed
/// (on a 64-bit host the address-to-u32 conversion would panic).
#[allow(dead_code, unused_variables)]
fn compile_only() {
    // kernel crate re-export
    let _a = kernel::PartitionSpec::entry(my_entry);
    let _b = kernel::PartitionSpec::body(my_body, 42);

    // plib crate re-export
    let _c = plib::PartitionSpec::entry(my_entry);
    let _d = plib::PartitionSpec::body(my_body, 7);
}

fn main() {}
