use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // For kernel examples, provide a complete device.x with all STM32F429 interrupt handlers
    // and memory.x with kernel_state section (this overrides kernel's memory.x)
    if env::var("CARGO_FEATURE_KERNEL_EXAMPLE").is_ok() {
        File::create(out.join("device.x"))
            .unwrap()
            .write_all(include_bytes!("device.x"))
            .unwrap();
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("memory_master.x"))
            .unwrap();
        // Add our out directory first so our memory.x takes precedence
        println!("cargo:rustc-link-search={}", out.display());
        // Force the linker to use our memory.x
        println!("cargo:rustc-link-arg=-L{}", out.display());
        println!("cargo:rerun-if-changed=device.x");
        println!("cargo:rerun-if-changed=memory_master.x");
    }
}
