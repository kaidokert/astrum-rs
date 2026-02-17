use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("thumb") {
        return;
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // cortex-m-rt 0.7 requires __INTERRUPTS to be defined. Without a device PAC,
    // provide a minimal device.x that defines the symbol. The actual interrupt
    // vector array must be provided in Rust code with #[link_section = ".vector_table.interrupts"].
    File::create(out.join("device.x"))
        .unwrap()
        .write_all(b"/* Minimal device.x for generic Cortex-M without a PAC */\n")
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tdevice.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
