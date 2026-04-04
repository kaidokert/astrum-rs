use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const MEMORY_X: &str = "\
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("thumb") {
        return Ok(());
    }

    let out = PathBuf::from(env::var("OUT_DIR")?);
    let is_qemu = env::var("CARGO_FEATURE_BOARD_QEMU").is_ok();

    // Minimal device.x for cortex-m-rt (provides __INTERRUPTS symbol).
    File::create(out.join("device.x"))?
        .write_all(b"/* Minimal device.x for porting_guide examples */\n")?;

    println!("cargo:rustc-link-search={}", out.display());

    if is_qemu {
        File::create(out.join("memory.x"))?.write_all(MEMORY_X.as_bytes())?;
        println!("cargo:rustc-link-arg=-Tlink.x");
    }

    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}
