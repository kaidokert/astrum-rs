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

    // cortex-m-rt's link.x uses EXTERN(__INTERRUPTS) which requires
    // the symbol to be defined (not just PROVIDEd). PAC crates normally
    // define this via svd2rust. Without a PAC, we define it as an
    // empty section at address 0.
    File::create(out.join("device.x"))
        .unwrap()
        .write_all(b"__INTERRUPTS = 0;\n")
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
