# f429zi Examples

24 examples organised by layer, from bare-metal bring-up through to the full
kernel + debug stack.  Build commands assume `cd f429zi`.

---

## Layer 1 — Bare-metal / HAL bring-up

No kernel, no RTOS.  Validates the toolchain, linker script, HAL, and
on-board peripherals.  Requires `--features hal` (the default).

| Example | What it tests | HW verified |
|---|---|---|
| `blinky` | GPIO output via stm32f4xx-hal; LED blink | ✅ |
| `systick` | SysTick exception, RTT tick counter (10 ticks then stops) | ✅ |
| `mpu_info` | Reads MPU TYPE register, prints region count via RTT | ✅ |
| `usb_simple` | USB OTG-FS CDC device, interrupt-driven; counts IN/OUT packets | ✅ |
| `usb_serial` | USB CDC with RTT echo, line-buffered | ✅ |
| `usb_commands` | USB CDC command parser (`led on/off`, `status`, `help`) | ✅ |

Build: `cargo build --example <name>` (default `hal` feature)

---

## Layer 2 — Context-switch scaffolding

Manual bring-up of the context-switch machinery before wiring in the full
kernel.  No `define_kernel!`, no SysTick-driven scheduling.

| Example | What it tests | HW verified |
|---|---|---|
| `context_switch` | PendSV two-partition switch, hand-rolled stacks (Step 1) | ✅ |
| `context_switch_svc` | Adds voluntary yield via `svc #0` (Step 2) | ✅ |
| `context_switch_schedule` | Adds `ScheduleTable` time-partitioned scheduling (Step 3) | ✅ |
| `context_switch_ipc` | Adds sampling-port data exchange between partitions (Step 4) | ✅ |

Build: `cargo build --example <name> --features kernel-example --no-default-features`

---

## Layer 3 — MPU enforcement

Tests the MPU wiring independently of the full kernel IPC stack.

| Example | What it tests | HW verified |
|---|---|---|
| `mpu_basic` | 8 static MPU regions, CTRL=0x5 (ENABLE\|PRIVDEFENA), R/W access PASS | ✅ |
| `context_switch_mpu` | Static MPU regions survive 10 context switches; PASS logged | ✅ |
| `context_switch_mpu_dynamic` | Dynamic MPU: R2 reprogrammed per-switch, 10-switch PASS | ✅ |
| `mpu_kernel_demo` | MPU wired into `define_kernel!`; both partitions run, MemManage=0 | ✅ |

Build: `cargo build --example <name> --features kernel-mpu --no-default-features`
(except `mpu_basic` / `context_switch_mpu` which use `kernel-example`)

---

## Layer 4 — Kernel IPC primitives

Full kernel (`define_kernel!`, SysTick scheduler, PendSV switch).
Each example exercises one IPC primitive end-to-end on real hardware.

| Example | What it tests | Success criterion | HW verified |
|---|---|---|---|
| `semaphore_demo` | Binary semaphore producer/consumer; blocking acquire | PROD > 10, CONS > 10, no missed | ✅ |
| `mutex_demo` | Mutex mutual exclusion; shared counter integrity | `SHARED == A_INC + B_INC` | ✅ |
| `events_demo` | Event flags; P1 blocks in Waiting, woken by P0 | SET ≈ RECV ≈ BLOCK > 1000 | ✅ |
| `msg_demo` | Message queues; P0 sends counter, P1 receives | SENT == RECV > 10 | ✅ |
| `queuing_demo` | Queuing ports; command/response round-trip | CMD=255/255, RSP=255/255 | ✅ |
| `blackboard_demo` | Blackboard; one publisher, two subscribers | PUB, SUB_A, SUB_B all advancing | ✅ |
| `sampling_demo` | Sampling ports; sustained write/read at 1ms tick | write_count == read_count | ✅ |
| `gpio_blinky` | Event-driven GPIO; P0 waits on SysTick event, toggles LEDs | LED blinks ~1 Hz, toggle_count++ | ✅ |

Build: `cargo build --example <name> --features kernel-example --no-default-features`

---

## Layer 5 — Partition debug facility

Exercises `plib` / `dprint!` — the partition-side structured debug logging
path — and its integration with the kernel auto-drain → RTT pipeline.

| Example | What it tests | Success criterion | HW verified |
|---|---|---|---|
| `events_dprint_demo` | Event flags narrated with `dprint!`; drain path exercised alongside IPC | SET≈RECV≈BLOCK > 4800; `[P1:DBG]` lines in RTT | ✅ |
| `plib_demo` | `dprint!` (INFO) and `debug_warn!` (WARN) from two partitions; auto-drain | `[P0:INF]` and `[P1:WRN]` lines in RTT, dprint calls > 5 each | ✅ |

Build: `cargo build --example <name> --features partition-debug --no-default-features`

---

## Feature flag quick reference

| Flag | Enables |
|---|---|
| `hal` (default) | stm32f4xx-hal, usb-device, usbd-serial, panic-halt |
| `kernel-example` | kernel crate, heapless |
| `kernel-mpu` | kernel-example + dynamic-mpu MPU enforcement |
| `partition-debug` | kernel-example + plib + `partition-debug` syscalls |
