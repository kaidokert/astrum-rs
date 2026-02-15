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

    // cortex-m-rt 0.7 asserts SIZEOF(.vector_table) > 0x40, requiring at least
    // one interrupt vector. Without a device PAC, we provide a single dummy entry.
    File::create(out.join("device.x"))
        .unwrap()
        .write_all(
            b"PROVIDE(__INTERRUPTS = __interrupts);\n\
              SECTIONS {\n\
                .vector_table.interrupts : {\n\
                  __interrupts = .;\n\
                  LONG(0);\n\
                } > FLASH\n\
              }\n",
        )
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tdevice.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
