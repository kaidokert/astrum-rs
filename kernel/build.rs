//! Kernel build script — conditional linker fragment output.
//!
//! Emitted files (all written to `OUT_DIR`):
//!
//! | File             | When                        | Purpose                                 |
//! |------------------|-----------------------------|-----------------------------------------|
//! | `kernel_state.x` | Always (thumb targets)      | `.kernel_state` SECTIONS / INSERT AFTER  |
//! | `tombstone.x`    | Always (thumb targets)      | `.noinit` NOLOAD section for tombstone   |
//! | `device.x`       | Always (thumb targets)      | Minimal device.x for cortex-m-rt        |
//! | `memory.x`       | `qemu` feature only         | MEMORY{} block for QEMU LM3S6965EVB     |
//! | `link.x`         | `qemu` feature only         | cortex-m-rt full link (needs MEMORY{})   |
//!
//! When the `qemu` feature is **not** active, `memory.x` and `link.x` are
//! omitted so a BSP crate can supply its own memory map without conflict.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// Linker fragment for .kernel_state SECTIONS block.
///
/// Kernel state section — reserves a fixed address for assembly access.
/// NOLOAD: not initialized by the C runtime (kernel initializes it).
/// ALIGN(4096): must match repr(C, align(4096)) on KernelStorageBuffer so
///   the linker-placed section satisfies the Rust type's alignment requirement.
///
/// Symbols defined:
///   __kernel_state_start — start address of kernel state region
///   __kernel_state_end   — end address of kernel state region
///
/// Usage: Assembly can load the kernel pointer directly via:
///   ldr r0, =__kernel_state_start
///
/// The `{output_region}` placeholder is replaced at emit time:
///   - QEMU targets: `> RAM` (matching the MEMORY block we provide)
///   - Other targets: empty (inherits the region from INSERT AFTER .bss)
const KERNEL_STATE_X_TEMPLATE: &str = "\
/* Kernel state section — ALIGN(4096) must match KernelStorageBuffer alignment.
 * Symbols __kernel_state_start / __kernel_state_end are used by assembly
 * (ldr r0, =__kernel_state_start) for direct pointer access. */
SECTIONS
{
  .kernel_state (NOLOAD) : ALIGN(4096)
  {
    __kernel_state_start = .;
    KEEP(*(.kernel_state .kernel_state.*))
    . = ALIGN(4);
    __kernel_state_end = .;
  }{output_region}
} INSERT AFTER .bss;
";

/// Linker fragment for .noinit section (PanicTombstone).
///
/// NOLOAD: not loaded from flash — survives soft resets so the panic
/// handler can stash diagnostic data that the next boot reads back.
///
/// Symbols defined:
///   __noinit_start — start address of .noinit region
///   __noinit_end   — end address of .noinit region
///
/// The `{output_region}` placeholder is replaced at emit time (same
/// logic as `kernel_state.x`).
const TOMBSTONE_X_TEMPLATE: &str = "\
/* .noinit section — survives soft reset for post-mortem diagnostics. */
SECTIONS
{
  .noinit (NOLOAD) : ALIGN(4)
  {
    __noinit_start = .;
    KEEP(*(.noinit .noinit.*))
    . = ALIGN(4);
    __noinit_end = .;
  }{output_region}
} INSERT AFTER .bss;
";

/// MEMORY definition for QEMU LM3S6965EVB (256K FLASH / 64K RAM).
const MEMORY_X: &str = "\
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
";

/// Backend names in priority order (highest first).
const BACKEND_PRIORITY: &[&str] = &["semihosting", "rtt", "swo", "defmt"];

/// Select the highest-priority backend from enabled backend names.
///
/// Priority: semihosting > rtt > swo > defmt > none.
/// Iterates the priority list and returns the first backend present in `enabled`.
fn select_backend(enabled: &[&str]) -> &'static str {
    BACKEND_PRIORITY
        .iter()
        .copied()
        .find(|b| enabled.contains(b))
        .unwrap_or("none")
}

/// Map a log backend to its panic backend (semihosting/rtt pass through, others → halt).
fn select_panic_backend(log_backend: &str) -> &'static str {
    match log_backend {
        "semihosting" => "semihosting",
        "rtt" => "rtt",
        _ => "halt",
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Emit klog backend cfg based on enabled features.
    // Priority: semihosting > rtt > swo > defmt > none
    // If multiple are enabled, we pick the first in priority order and emit a warning.
    let enabled: Vec<&str> = BACKEND_PRIORITY
        .iter()
        .copied()
        .filter(|name| {
            let env_var = format!("CARGO_FEATURE_LOG_{}", name.to_uppercase());
            env::var(env_var).is_ok()
        })
        .collect();

    let backend = select_backend(&enabled);
    if enabled.len() > 1 {
        println!(
            "cargo:warning=Multiple klog backends enabled ({:?}), using '{}'",
            enabled, backend
        );
    }

    println!("cargo:rustc-cfg=klog_backend=\"{}\"", backend);
    // Declare valid klog_backend values for check-cfg lint.
    println!("cargo::rustc-check-cfg=cfg(klog_backend, values(\"semihosting\", \"rtt\", \"swo\", \"defmt\", \"none\"))");

    let panic_backend = select_panic_backend(backend);
    println!("cargo:rustc-cfg=panic_backend=\"{}\"", panic_backend);
    println!(
        "cargo::rustc-check-cfg=cfg(panic_backend, values(\"semihosting\", \"rtt\", \"halt\"))"
    );

    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("thumb") {
        return Ok(());
    }

    let out = PathBuf::from(env::var("OUT_DIR")?);
    let is_qemu = env::var("CARGO_FEATURE_QEMU").is_ok();

    // When targeting QEMU we explicitly place sections in `> RAM` (matching our MEMORY
    // block); for other targets we omit the output region so it inherits from
    // INSERT AFTER .bss, avoiding a hard dependency on a region named "RAM".
    let output_region = if is_qemu { " > RAM" } else { "" };

    // Emit kernel_state.x for the .kernel_state SECTIONS block.
    let kernel_state_x = KERNEL_STATE_X_TEMPLATE.replace("{output_region}", output_region);
    File::create(out.join("kernel_state.x"))?.write_all(kernel_state_x.as_bytes())?;

    // Emit tombstone.x for the .noinit SECTIONS block (PanicTombstone).
    let tombstone_x = TOMBSTONE_X_TEMPLATE.replace("{output_region}", output_region);
    File::create(out.join("tombstone.x"))?.write_all(tombstone_x.as_bytes())?;

    // cortex-m-rt 0.7 requires __INTERRUPTS to be defined. Without a device PAC,
    // provide a minimal device.x that defines the symbol.
    File::create(out.join("device.x"))?
        .write_all(b"/* Minimal device.x for generic Cortex-M without a PAC */\n")?;

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tdevice.x");
    println!("cargo:rustc-link-arg=-Tkernel_state.x");
    println!("cargo:rustc-link-arg=-Ttombstone.x");

    // Only emit memory.x (QEMU memory map) and -Tlink.x when the qemu feature is active.
    // Without MEMORY{}, link.x from cortex-m-rt would fail, so both are gated together.
    if is_qemu {
        File::create(out.join("memory.x"))?.write_all(MEMORY_X.as_bytes())?;
        println!("cargo:rustc-link-arg=-Tlink.x");
    }

    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backend_no_features() {
        assert_eq!(select_backend(&[]), "none");
    }

    #[test]
    fn backend_single_feature() {
        assert_eq!(select_backend(&["semihosting"]), "semihosting");
        assert_eq!(select_backend(&["rtt"]), "rtt");
        assert_eq!(select_backend(&["swo"]), "swo");
        assert_eq!(select_backend(&["defmt"]), "defmt");
    }

    #[test]
    fn backend_multi_highest_priority_wins() {
        // In priority order — sanity check.
        assert_eq!(select_backend(&["semihosting", "rtt"]), "semihosting");
        assert_eq!(select_backend(&["rtt", "swo"]), "rtt");
        assert_eq!(select_backend(&["swo", "defmt"]), "swo");

        // Out of priority order — verifies the function enforces priority,
        // not just picking the first element.
        assert_eq!(select_backend(&["rtt", "semihosting"]), "semihosting");
        assert_eq!(select_backend(&["defmt", "swo"]), "swo");
        assert_eq!(select_backend(&["defmt", "rtt", "swo"]), "rtt");
        assert_eq!(
            select_backend(&["defmt", "swo", "rtt", "semihosting"]),
            "semihosting"
        );
    }

    #[test]
    fn tombstone_x_fragment_contains_noinit_section() {
        let fragment = TOMBSTONE_X_TEMPLATE.replace("{output_region}", "");
        assert!(
            fragment.contains(".noinit (NOLOAD)"),
            "must define a .noinit NOLOAD section"
        );
        assert!(
            fragment.contains("KEEP(*(.noinit .noinit.*))"),
            "must KEEP .noinit input sections"
        );
        assert!(
            fragment.contains("INSERT AFTER .bss"),
            "must INSERT AFTER .bss"
        );
        assert!(
            fragment.contains("__noinit_start"),
            "must define __noinit_start symbol"
        );
        assert!(
            fragment.contains("__noinit_end"),
            "must define __noinit_end symbol"
        );
    }

    #[test]
    fn tombstone_x_qemu_gets_ram_region() {
        let fragment = TOMBSTONE_X_TEMPLATE.replace("{output_region}", " > RAM");
        assert!(
            fragment.contains("> RAM"),
            "QEMU variant must place section in RAM"
        );
    }

    #[test]
    fn panic_backend_mappings() {
        assert_eq!(select_panic_backend("semihosting"), "semihosting");
        assert_eq!(select_panic_backend("rtt"), "rtt");
        assert_eq!(select_panic_backend("swo"), "halt");
        assert_eq!(select_panic_backend("defmt"), "halt");
        assert_eq!(select_panic_backend("none"), "halt");
    }
}
