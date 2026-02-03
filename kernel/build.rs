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

    // TODO: reviewer noted this device.x fix is an unrelated build-system
    // change that should be in a separate commit.  It is kept here because
    // without it all examples (including get_time_test) fail to link.
    //
    // cortex-m-rt's link.x INCLUDEs device.x for interrupt vector
    // definitions.  Without a PAC crate providing one, supply an empty
    // file so the linker resolves the include.
    File::create(out.join("device.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
