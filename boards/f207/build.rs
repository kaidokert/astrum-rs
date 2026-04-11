use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    if env::var("CARGO_FEATURE_KERNEL_EXAMPLE").is_ok() {
        File::create(out.join("device.x"))
            .unwrap()
            .write_all(include_bytes!("device.x"))
            .unwrap();
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("memory_master.x"))
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
        println!("cargo:rustc-link-arg=-L{}", out.display());
        println!("cargo:rerun-if-changed=device.x");
        println!("cargo:rerun-if-changed=memory_master.x");
    }
}
