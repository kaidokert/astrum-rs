# Convenience targets for building and running QEMU examples.
#
# Examples:  kernel/examples/smoke_test.rs, kernel/examples/qemu_smoke.rs
# Runner:   .cargo/config.toml (qemu-system-arm with semihosting)
# Script:   scripts/run-qemu-examples.sh
TARGET   ?= thumbv7m-none-eabi
FEATURES ?= qemu,log-semihosting

.PHONY: smoke-test qemu-smoke build-smoke test-qemu custom-ivt-test irq-dispatch-test check-rtt check-rtt-combos check-rtt-lint check-pac-singleton

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

# Build all examples with log-rtt to catch duplicate _SEGGER_RTT symbols at link time
check-rtt:
	cargo build -p kernel --examples --features log-rtt --target $(TARGET)

# Build with conflicting log-backend pairs to verify build.rs priority selection
check-rtt-combos:
	cargo build -p kernel --lib --features log-rtt,log-swo --target $(TARGET)
	cargo build -p kernel --lib --features log-rtt,log-defmt --target $(TARGET)
	cargo build -p kernel --lib --features log-semihosting,log-rtt --target $(TARGET)
	cargo build -p kernel --lib --features log-semihosting,log-defmt --target $(TARGET)

# Lint: rtt_init_print! must only appear in boot.rs
check-rtt-lint:
	@offenders=$$(grep -rn 'rtt_init_print!' kernel/src/ kernel/examples/ --include='*.rs' | grep -v 'boot\.rs:' || true); \
	if [ -n "$$offenders" ]; then \
		echo "FAIL: rtt_init_print! must only be used in kernel/src/boot.rs."; \
		echo "      Use kernel::init_rtt() instead."; \
		echo "$$offenders"; \
		exit 1; \
	fi
	@echo "  ok — rtt_init_print! confined to boot.rs"

# Lint: Peripherals::take() must not appear in files containing partition _entry functions
check-pac-singleton:
	@offenders=$$(grep -rlF 'Peripherals::take()' kernel/examples/ kernel/src/ --include='*.rs' \
		| xargs -r grep -lE 'extern\s+"C"\s+fn\s+\w+_entry' \
		| xargs -r grep -nF 'Peripherals::take()' || true); \
	if [ -n "$$offenders" ]; then \
		echo "FAIL: Peripherals::take() must not be used in partition code."; \
		echo "      Partitions run after take() has already been called by boot()."; \
		echo "      See docs/porting-guide.md § PAC Singleton in Partitioned Code."; \
		echo "$$offenders"; \
		exit 1; \
	fi
	@echo "  ok — no Peripherals::take() in partition entry files"

# Run all QEMU integration examples
test-qemu:
	./scripts/run-qemu-examples.sh
