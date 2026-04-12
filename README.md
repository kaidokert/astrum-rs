# Astrum RTOS

A statically-partitioned, ARINC 653-inspired real-time kernel for ARM
Cortex-M, written in Rust. Each partition is given a fixed time slice
enforced by SysTick, and the MPU provides spatial isolation between
partitions and the kernel. All memory is allocated at compile time;
there is no heap.

## Status

- The QEMU port (LM3S / Cortex-M3) is the default target and
  is what the in-tree examples and tests are wired up against.
- Several hardware ports (STM32F429, STM32F207, nRF52, SAME51) live
  under `boards/`. See [`boards/README.md`](boards/README.md) for the
  full feature matrix and per-board status.

## Quick start

Prerequisites:

- Rust nightly toolchain (the kernel relies on `generic_const_exprs`)
- `qemu-system-arm` for running examples

```bash
rustup default nightly
rustup target add thumbv7m-none-eabi

# Build and run a demo on QEMU
cargo run -p kernel --target thumbv7m-none-eabi \
    --example sampling_demo --features qemu,log-semihosting
```

`.cargo/config.toml` configures `qemu-system-arm` as the runner for
`thumbv7m-none-eabi`, so `cargo run` launches QEMU automatically. For
the list of demos and what each one exercises, see
[`notes/demos.md`](notes/demos.md).

## Layout

```
kernel/          #![no_std] kernel, examples, adversarial tests
plib/            Partition-side library (syscall wrappers, helpers)
rtos-traits/     Traits shared between kernel and partitions
boards/          Per-board crates
docs/            Finalized docs
notes/           Working design notes
porting_guide/   Reference partition layouts for new ports
```

## Documentation

- **Architecture overview** — memory model, scheduling, SVC dispatch,
  MPU isolation, key design decisions:
  [`docs/architecture.md`](docs/architecture.md)
- **IPC reference** — full syscall table and per-primitive semantics:
  [`docs/reference/ipc-reference.md`](docs/reference/ipc-reference.md)
- **Driver architecture** — kernel-as-resource-mediator model and how
  partitions own peripherals:
  [`docs/driver-architecture.md`](docs/driver-architecture.md)
- **Demo walkthroughs and expected output** —
  [`notes/demos.md`](notes/demos.md)
- **Porting to new hardware** —
  [`docs/porting-guide.md`](docs/porting-guide.md)

## License

Apache-2.0. See [`LICENSE`](LICENSE).
