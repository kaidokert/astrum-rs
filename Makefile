# Convenience targets for building and running QEMU examples.
#
# Examples:  kernel/examples/smoke_test.rs, kernel/examples/qemu_smoke.rs
# Runner:   .cargo/config.toml (qemu-system-arm with semihosting)
# Script:   scripts/run-qemu-examples.sh
TARGET   ?= thumbv7m-none-eabi
FEATURES ?= qemu,log-semihosting

.PHONY: smoke-test qemu-smoke build-smoke test-qemu custom-ivt-test irq-dispatch-test

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

# Run custom-IVT example (qemu_custom_ivt)
custom-ivt-test:
	cargo run -p kernel --target $(TARGET) --features $(FEATURES),custom-ivt --example qemu_custom_ivt

# Run IRQ dispatch test (irq_dispatch_test)
irq-dispatch-test:
	cargo run -p kernel --target $(TARGET) --features $(FEATURES),custom-ivt --example irq_dispatch_test

# Run all QEMU integration examples
test-qemu:
	./scripts/run-qemu-examples.sh
