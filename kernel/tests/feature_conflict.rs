//! Tests that conflicting kernel-size features produce compile errors.
//!
//! Run with: `cargo test --test feature_conflict`

#![cfg(not(target_arch = "arm"))]

use std::process::Command;

/// Helper: run `cargo check` with the given features and return (success, stderr).
fn cargo_check_features(features: &[&str]) -> (bool, String) {
    let mut cmd = Command::new("cargo");
    cmd.arg("check")
        .arg("--no-default-features")
        .arg("--lib")
        .current_dir(env!("CARGO_MANIFEST_DIR"));

    if !features.is_empty() {
        cmd.arg("--features").arg(features.join(","));
    }

    let output = cmd.output().expect("failed to run cargo check");
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    (output.status.success(), stderr)
}

#[test]
fn conflict_4k_and_8k() {
    let (ok, stderr) = cargo_check_features(&["kernel-4k", "kernel-8k"]);
    assert!(!ok, "expected compile error for kernel-4k + kernel-8k");
    assert!(
        stderr.contains("`kernel-4k` and `kernel-8k` are mutually exclusive"),
        "error message should mention kernel-4k and kernel-8k conflict, got:\n{stderr}"
    );
}

#[test]
fn conflict_4k_and_16k() {
    let (ok, stderr) = cargo_check_features(&["kernel-4k", "kernel-16k"]);
    assert!(!ok, "expected compile error for kernel-4k + kernel-16k");
    assert!(
        stderr.contains("`kernel-4k` and `kernel-16k` are mutually exclusive"),
        "error message should mention kernel-4k and kernel-16k conflict, got:\n{stderr}"
    );
}

#[test]
fn conflict_8k_and_16k() {
    let (ok, stderr) = cargo_check_features(&["kernel-8k", "kernel-16k"]);
    assert!(!ok, "expected compile error for kernel-8k + kernel-16k");
    assert!(
        stderr.contains("`kernel-8k` and `kernel-16k` are mutually exclusive"),
        "error message should mention kernel-8k and kernel-16k conflict, got:\n{stderr}"
    );
}

#[test]
fn single_feature_compiles() {
    for feature in &["kernel-4k", "kernel-8k", "kernel-16k"] {
        let (ok, stderr) = cargo_check_features(&[feature]);
        assert!(ok, "expected {feature} alone to compile, stderr:\n{stderr}");
    }
}

#[test]
fn no_feature_compiles() {
    let (ok, stderr) = cargo_check_features(&[]);
    assert!(ok, "expected no features to compile, stderr:\n{stderr}");
}

#[test]
fn escape_hatch_suppresses_conflict() {
    let (ok, stderr) =
        cargo_check_features(&["kernel-4k", "kernel-8k", "kernel-16k", "_kernel-size-any"]);
    assert!(
        ok,
        "expected _kernel-size-any to suppress conflict, stderr:\n{stderr}"
    );
}
