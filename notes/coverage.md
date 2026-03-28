# Code Coverage

The project uses `cargo-llvm-cov` for line-coverage tracking on host-side
unit tests. The CI gate enforces a minimum of 75 % line coverage.

## Quick reference

```bash
cd kernel

# Summary table (terminal)
cargo llvm-cov

# Detailed per-line report (terminal)
cargo llvm-cov --text

# HTML report (open target/llvm-cov/html/index.html)
cargo llvm-cov --html

# Fail if coverage drops below threshold (all features)
cargo llvm-cov --all-features --fail-under-lines 75
```

## Full CI pipeline

```bash
./scripts/ci.sh
```

This runs, in order: `cargo fmt --check`, `cargo clippy` (host and
target), `cargo test` with coverage enforcement, and all QEMU examples.
