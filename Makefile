# Convenience targets for building and running QEMU examples.
#
# Examples:  kernel/examples/smoke_test.rs, kernel/examples/qemu_smoke.rs
# Runner:   .cargo/config.toml (qemu-system-arm with semihosting)
# Script:   scripts/run-qemu-examples.sh
TARGET   ?= thumbv7m-none-eabi
FEATURES ?= qemu,log-semihosting

.PHONY: smoke-test qemu-smoke build-smoke test-qemu

# Minimal single-partition smoke test (SYS_YIELD)
smoke-test:
	cargo run -p kernel --target $(TARGET) --features $(FEATURES) --example smoke_test

# Two-partition smoke test (SYS_YIELD + semaphore IPC)
qemu-smoke:
	cargo run -p kernel --target $(TARGET) --features $(FEATURES) --example qemu_smoke

# Build-only check (no QEMU)
build-smoke:
	cargo build -p kernel --target $(TARGET) --features $(FEATURES) --example smoke_test
	cargo build -p kernel --target $(TARGET) --features $(FEATURES) --example qemu_smoke

# Run all QEMU integration examples
test-qemu:
	./scripts/run-qemu-examples.sh
