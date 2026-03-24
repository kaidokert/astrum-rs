include!("../build-support/fpu_check.rs");

use std::env;
use std::fs;
use std::path::PathBuf;

/// Linker fragment for .kernel_state section (matches kernel/build.rs).
/// Explicit `> RAM` is required for hardware targets; NOLOAD sections
/// with INSERT AFTER cannot inherit the output region from .bss.
const KERNEL_STATE_X: &str = "\
SECTIONS
{
  .kernel_state (NOLOAD) : ALIGN(4096)
  {
    __kernel_state_start = .;
    KEEP(*(.kernel_state .kernel_state.*))
    . = ALIGN(4);
    __kernel_state_end = .;
  } > RAM
} INSERT AFTER .bss;
";

/// Linker fragment for .noinit section (matches kernel/build.rs).
const TOMBSTONE_X: &str = "\
SECTIONS
{
  .noinit (NOLOAD) : ALIGN(4)
  {
    __noinit_start = .;
    KEEP(*(.noinit .noinit.*))
    . = ALIGN(4);
    __noinit_end = .;
  } > RAM
} INSERT AFTER .bss;
";

fn main() {
    check_fpu_context();

    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("thumb") {
        return;
    }

    let out = PathBuf::from(env::var("OUT_DIR").unwrap()); // TODO(panic-free): build script env

    // Copy board memory.x into OUT_DIR for cortex-m-rt to find.
    fs::copy("memory.x", out.join("memory.x")).unwrap(); // TODO(panic-free): fs in build script

    // Emit kernel linker fragments (kernel's build.rs link-args don't
    // propagate to dependent crates, so the BSP must emit its own).
    fs::write(out.join("kernel_state.x"), KERNEL_STATE_X).unwrap(); // TODO(panic-free): fs in build script
    fs::write(out.join("tombstone.x"), TOMBSTONE_X).unwrap(); // TODO(panic-free): fs in build script

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tkernel_state.x");
    println!("cargo:rustc-link-arg=-Ttombstone.x");

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
