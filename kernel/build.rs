use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Emit klog backend cfg based on enabled features.
    // Priority: semihosting > rtt > swo > defmt > none
    // If multiple are enabled, we pick the first in priority order and emit a warning.
    let backends = [
        ("CARGO_FEATURE_LOG_SEMIHOSTING", "semihosting"),
        ("CARGO_FEATURE_LOG_RTT", "rtt"),
        ("CARGO_FEATURE_LOG_SWO", "swo"),
        ("CARGO_FEATURE_LOG_DEFMT", "defmt"),
    ];

    let enabled: Vec<&str> = backends
        .iter()
        .filter(|(env_var, _)| env::var(env_var).is_ok())
        .map(|(_, name)| *name)
        .collect();

    let backend = match enabled.len() {
        0 => "none",
        1 => enabled[0],
        _ => {
            println!(
                "cargo:warning=Multiple klog backends enabled ({:?}), using '{}'",
                enabled, enabled[0]
            );
            enabled[0]
        }
    };

    println!("cargo:rustc-cfg=klog_backend=\"{}\"", backend);
    // Declare valid klog_backend values for check-cfg lint.
    println!("cargo::rustc-check-cfg=cfg(klog_backend, values(\"semihosting\", \"rtt\", \"swo\", \"defmt\", \"none\"))");

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
