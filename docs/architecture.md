# Architecture

A statically-partitioned hard-realtime microkernel for ARM Cortex-M3/M4
that enforces **temporal isolation** (a fixed schedule table driven by
SysTick) and **spatial isolation** (per-partition MPU regions). The
design follows ARINC 653 principles adapted for the constraints of
small Cortex-M hardware.

This document is an overview of the kernel's structure. For the
full IPC API surface, see [`reference/ipc-reference.md`](reference/ipc-reference.md).
For the kernel's driver philosophy and how partitions own peripherals,
see [`driver-architecture.md`](driver-architecture.md).
For demo walkthroughs, see [`../notes/demos.md`](../notes/demos.md). For
porting to a new board, see [`porting-guide.md`](porting-guide.md).

## Design philosophy

- **Static allocation only.** Every kernel structure has a compile-time
  fixed size. No heap, no allocation failures, no fragmentation, fully
  deterministic memory usage. Pool capacities are const-generic
  parameters on a `KernelConfig` trait.
- **Two-level scheduling.** A static schedule table drives a repeating
  major frame; partitions are non-preemptive within their slot but
  cooperatively yield via `SYS_YIELD` or block on a syscall.
- **MPU as the only isolation primitive.** No software-emulated memory
  protection, no page tables, no virtual memory.
- **Cortex-M-native exception model.** SysTick for the scheduler tick,
  PendSV for context switch, SVC for syscall entry, MemManage for fault
  containment.
- **Single canonical kernel object.** All kernel state lives in one
  `Kernel<C>` struct stored in a single static, accessed through critical
  sections. Sub-structs group related state (partitions, IPC pools,
  runtime context) so that bounds on `C` only flow where they are needed.
- **Unprivileged partitions, privileged kernel.** Partitions run in
  Thread mode with PSP and `CONTROL.nPRIV = 1`. The kernel runs in
  Handler mode (PendSV / SysTick / SVC) and uses `MPU_CTRL.PRIVDEFENA`
  for default-map access without consuming MPU regions.

## Memory model

### Memory map (LM3S6965EVB / Cortex-M3 reference)

```
0x0000_0000  +---------------------------+
             | FLASH (256 KB)            |
             |  Vector table             |
             |  Kernel code              |
             |  Partition code           |
0x0004_0000  +---------------------------+

0x2000_0000  +---------------------------+
             | RAM (64 KB)               |
             |  Kernel BSS / data        |
             |  Partition 0 stack + data |
             |  Partition 1 stack + data |
             |  ...                      |
             |  Kernel stack (top of RAM)|
0x2001_0000  +---------------------------+
```

Partition memory regions are power-of-two sized and power-of-two
aligned for MPU compatibility. Sizes are configurable at compile time
through the `KernelConfig` trait and the schedule table.

### Partition control block

The PCB is the kernel's per-partition bookkeeping structure, statically
allocated as one entry per partition in an array indexed by partition
ID. Conceptually it carries:

- `id`, `state` (`Ready`, `Running`, `Waiting`)
- `stack_pointer` (current PSP value, updated on context switch)
- `entry_point` (initial PC)
- `stack_base`, `stack_size`
- `mpu_region` (base + size — used to derive MPU registers on the fly)
- IPC primitive state (event flag bitmasks, wait masks)

State transitions are enforced by `PartitionControlBlock::transition()`:

| From | To | Trigger |
|---|---|---|
| Ready | Running | Scheduler dispatches the partition at slot start |
| Running | Ready | Time slot expires or partition yields |
| Running | Waiting | Partition executes a blocking syscall |
| Waiting | Ready | The waited-for condition is satisfied |

The PCB does **not** store precomputed MPU register values. The
`partition_mpu_regions()` helper in `kernel/src/mpu.rs` derives the
four `(RBAR, RASR)` pairs at context-switch time from the PCB's
`MpuRegion`. This keeps the PCB small and eliminates stale-config bugs.

## Scheduling

### Schedule table

The schedule table is an immutable array of slots, each naming a
partition ID and a duration in SysTick ticks. The major frame is the
sum of all slots; when the last slot ends the table wraps to slot 0.

```rust
static SCHEDULE: ScheduleTable<3> = ScheduleTable {
    slots: [
        Slot { partition_id: 0, duration_ticks: 10 }, // 100 ms
        Slot { partition_id: 1, duration_ticks: 5 },  //  50 ms
        Slot { partition_id: 0, duration_ticks: 5 },  //  50 ms
    ],
};
// Major frame = 20 ticks = 200 ms at 10 ms/tick
// Partition 0 gets 150 ms (75%), partition 1 gets 50 ms (25%)
```

Properties:

- **Deterministic.** Every partition gets its exact allocated time.
- **Non-preemptive within slot.** A partition runs for its full slot
  unless it yields or blocks.
- **Auditable.** The schedule is statically known and can be reviewed
  at compile time.

### SysTick → PendSV coordination

The SysTick handler decrements the current slot's tick counter inside
a critical section. When a slot boundary is reached it writes the next
partition ID to a `NEXT_PARTITION` static and pends PendSV via
`SCB::set_pendsv()`. PendSV runs at the lowest exception priority and
performs the actual context switch after SysTick returns. This is the
standard Cortex-M RTOS pattern (FreeRTOS, Zephyr, ThreadX use the same
shape) and avoids context-switching inside a higher-priority handler.

### Exception priority tiers

A three-tier model with a deliberate gap between kernel and application
interrupts:

| Tier | Handlers | Priority |
|---|---|---|
| 1 (kernel) | SVCall, SysTick | `0x00`, `0x10` |
| 2 (application) | Device IRQs | ≥ `0x20` (default `0xC0`) |
| 3 (deferred switch) | PendSV | `0xFF` |

Within Tier 1, SVCall outranks SysTick so an in-progress syscall
completes without tick interruption. SysTick outranks all application
IRQs so the scheduler tick is never deferred by device handlers. PendSV
at the lowest priority ensures context switches run only after all
other work completes.

### PendSV mechanism

PendSV is a small `global_asm!` handler in `kernel/src/pendsv.rs`. On
entry:

1. The hardware has already pushed `{r0-r3, r12, lr, pc, xpsr}` onto
   the partition's PSP.
2. The handler reads `CURRENT_PCB`. If non-null, it pushes the
   callee-saved registers `{r4-r11}` onto the partition's stack and
   stores the updated PSP into the PCB.
3. It reads `NEXT_PARTITION`, updates `CURRENT_PCB`, applies the
   incoming partition's MPU regions, restores `{r4-r11}` from the new
   stack, sets `CONTROL.nPRIV = 1` (followed by an `ISB`), and returns
   via `EXC_RETURN = 0xFFFFFFFD` (Thread mode, PSP).

The `cbz` check on `CURRENT_PCB` lets the very first PendSV skip the
save phase — there is no current partition to save on cold boot. The
boot sequence pends PendSV and then `wfi`s; the first PendSV bootstraps
into partition 0.

The handler is ~20 instructions and is written in assembly so the
compiler does not insert a prologue that would clobber registers
before they can be saved.

## SVC syscall dispatch

### Calling convention

Partitions invoke kernel services via the `svc #0` instruction. The
immediate byte is ignored — the syscall number is passed in `r0`.
Arguments go in `r1`-`r3`. The return value comes back in `r0`.

| Register | Purpose |
|---|---|
| `r0` | Syscall number (in) / return value (out) |
| `r1` | Argument 1 (also: error detail on `Err`) |
| `r2` | Argument 2 |
| `r3` | Argument 3 |

The decision to use `r0` for the syscall number rather than the SVC
immediate avoids the need for the handler to back up the PC and decode
the Thumb instruction at `[stacked_pc - 2]`. It also makes the entry
sequence trivially portable across Cortex-M variants.

The SVC handler is an assembly trampoline that captures the exception
frame from PSP and calls `dispatch_svc(frame: &mut ExceptionFrame)`.
The dispatch hook installed by `define_kernel!` borrows the unified
`Kernel` static inside a critical section and calls `Kernel::dispatch()`.

### Syscall categories

The kernel exposes ~30 syscalls grouped by primitive:

- **Core** — `SYS_YIELD`, `SYS_GET_ID`, `SYS_GET_TIME`
- **Events** — wait/set/clear bitmask flags across partitions
- **Semaphores** — counting semaphores with bounded wait queues
- **Mutexes** — ownership-tracked mutual exclusion
- **Message queues** — fixed-size intra-partition FIFOs
- **Sampling ports** — latest-value, non-blocking inter-partition
  communication with validity/freshness tracking
- **Queuing ports** — FIFO message passing with blocking send/receive
  and deadline-based timeouts
- **Blackboards** — single-message broadcast buffers with blocking
  reads and wake-all semantics
- **Buffer pool** — partition-owned slots with MPU-window lending for
  zero-copy transfer
- **Virtual devices** — open / close / read / write / ioctl on
  registered device backends

For the full syscall number table, the per-call argument layout, and
the error-detail (`r1`) convention, see
[`reference/ipc-reference.md`](reference/ipc-reference.md).

### Blocking and deschedule

When a syscall determines that the caller must block, it transitions
the partition to `Waiting`, enqueues the partition ID into the
primitive's wait queue (or `TimedWaitQueue` for timeout-aware variants),
and calls `Kernel::trigger_deschedule()`, which sets a flag and pends
PendSV. Because PendSV runs after the SVC handler returns, the
remaining time slot is yielded to the next runnable partition
immediately rather than spinning to slot expiry.

Wake-up is symmetric: when the awaited condition is satisfied (event
set, semaphore signaled, message arrives, deadline expires), the
partition's PID is popped from the wait queue and its state moves
back to `Ready`. It will be dispatched at its next scheduled slot.

The full blocking and wake-up protocol, including timeout enforcement,
is documented in [`reference/ipc-reference.md`](reference/ipc-reference.md) §8 and §9.

## MPU isolation

### Static region layout

`partition_mpu_regions()` returns exactly four `(RBAR, RASR)` pairs per
partition, programmed into hardware regions R0–R3:

| Region | Purpose | AP | XN | Size |
|---|---|---|---|---|
| 0 | Background deny-all | no access | yes | 4 GiB |
| 1 | Partition code (FLASH) | priv+unpriv RO | no | varies |
| 2 | Partition data (RAM) | full RW | yes | varies |
| 3 | Stack guard | no access | yes | 32 B |

Higher region numbers take precedence on overlap, so the deny-all
background in R0 is carved out by the more specific regions in R1–R3.
Any unprivileged access outside R1–R3 falls through to the deny-all
background and triggers a MemManage fault.

The 32-byte stack guard at the bottom of each partition's stack
catches stack overflows that would otherwise silently corrupt adjacent
memory. It costs one MPU region per partition.

For the full slot allocation rationale, the peripheral ceiling rule,
and context-switch reprogramming details, see
[`mpu-slot-allocation.md`](mpu-slot-allocation.md).

### PRIVDEFENA — kernel access without dedicated regions

The MPU is enabled with `MPU_CTRL = ENABLE | PRIVDEFENA`. PRIVDEFENA
grants the default memory map **only** to privileged code, which lets
the kernel run without spending any of the eight MPU regions on its
own code or data:

1. **Handler mode is privileged.** SysTick, PendSV, SVC, and MemManage
   always execute privileged by hardware definition. PRIVDEFENA gives
   the kernel full access to its code, data, and peripherals from
   these handlers without dedicated regions.
2. **Thread mode is unprivileged.** PendSV sets `CONTROL.nPRIV = 1`
   followed by an `ISB` before returning to the partition. PRIVDEFENA
   does not apply to unprivileged code, so any access outside R1–R3
   hits the R0 deny-all background and faults.

This is the same isolation pattern used by FreeRTOS, Zephyr, and other
production Cortex-M RTOSes. An inline `const { assert!(...) }` block
in `apply_partition_mpu` verifies that PRIVDEFENA is set, catching
accidental removal at compile time.

### Apply sequence

`apply_partition_mpu()` is called from PendSV during context switch:

1. Disable the MPU (`MPU_CTRL = 0`) followed by `DSB + ISB` to avoid
   faults from a partially-configured region set.
2. Program R0–R3 with the new partition's `(RBAR, RASR)` pairs.
3. Re-enable the MPU with `ENABLE | PRIVDEFENA`, again followed by
   `DSB + ISB`.

If `partition_mpu_regions()` returns `None` (invalid size or
alignment), the helper falls back to a deny-all layout — R0 covers
4 GiB with no access, R1–R3 disabled — rather than panicking inside
PendSV.

## Dynamic MPU and virtual devices

In addition to the four static regions per partition, the kernel
supports runtime-managed dynamic regions in MPU slots R4–R7 via the
`MpuStrategy` trait:

- **`StaticStrategy`** — only manages R0–R3, treats R4–R7 as unused.
- **`DynamicStrategy`** — manages R4–R7 with a slot table protected
  by `Mutex<RefCell<…>>`. R4 holds the active partition's private
  RAM region; R5–R7 are ad-hoc windows opened by syscalls.

`DynamicStrategy` enables three additional kernel facilities:

- **Buffer pool.** `BufferPool<SLOTS, SIZE>` provides fixed-size
  partition-owned slots with MPU-window lending. A partition can
  "lend" a slot's RAM to another partition via syscall — the kernel
  programs an R5–R7 window with the appropriate permissions. This
  provides zero-copy cross-partition data transfer with full MPU
  enforcement.
- **Virtual devices.** `VirtualDevice` is a trait with
  `open / close / read / write / ioctl` operations. Devices register
  with the kernel and partitions access them through syscalls.
  `VirtualUartPair` is a paired TX/RX ring-buffer UART backend used
  by the IPC demos.
- **System windows.** Dedicated schedule slots for kernel bottom-half
  processing — UART transfers, ISR drain, etc. — that need to run
  with MPU access to buffers owned by partitions.

The dynamic MPU subsystem is part of the unified kernel build; it has
no feature flag. Buffer pool, virtual device, and system window
syscall semantics are documented in
[`reference/ipc-reference.md`](reference/ipc-reference.md) §10–§11.

## Adversarial test suite

`kernel/examples/adversarial/` contains regression tests that
deliberately attempt to violate the kernel's isolation guarantees and
verify that the expected fault or error code occurs. The suite covers
three categories:

| Category | What it checks |
|---|---|
| **Memory isolation** | Partition reads/writes kernel memory or another partition's stack; deep recursion into the stack guard region; partition writes to MPU registers. Expected: MemManage fault with `DACCVIOL`. |
| **Privilege enforcement** | Partition executes `CPSID i` / writes `BASEPRI`; partition tries to clear `CONTROL.nPRIV`. Expected: instructions silently ignored; `CONTROL.nPRIV` remains sticky. |
| **Syscall boundaries** | Syscalls with pointers in kernel memory or another partition's region; null and wrapping pointers; invalid resource IDs. Expected: `SvcError::InvalidPointer` / `InvalidResource`. |

Each test boots a single partition that performs the violating
operation, captures `MMFSR` / `MMFAR` from a custom MemManage handler,
and reports PASS/FAIL via semihosting.

These tests probe the failure path of the security model — the
functional demos and integration examples cover the success path.

## Key design decisions

- **D1: Static allocation only.** No heap. Trades runtime flexibility
  for guaranteed determinism.
- **D2: PendSV for context switch.** Slightly more latency than
  switching in SysTick, but avoids switching inside a higher-priority
  handler. Standard Cortex-M RTOS pattern.
- **D3: Syscall number in `r0`, not the SVC immediate.** Simpler entry
  path, no need to back up the PC and decode the Thumb instruction.
- **D4: No dynamic priorities.** Fixed time slots defined at compile
  time. No priority inversion, no inheritance, no rate-monotonic
  analysis at runtime — but the schedule must be designed offline to
  meet all deadlines.
- **D5: On-the-fly MPU region computation with PRIVDEFENA.** Partition
  regions are derived from PCB fields at context-switch time rather
  than stored as precomputed register values, eliminating stale-config
  bugs and keeping the PCB small. PRIVDEFENA covers the kernel without
  consuming any MPU regions.
- **D6: `global_asm!` for the PendSV handler.** Stable Rust, no need
  for `#[naked]` (nightly), no external `.s` files. The handler is
  small enough for manual review.
- **D7: Critical sections via `cortex_m::interrupt::free`.** Shared
  mutable state lives in `Mutex<RefCell<T>>`. A few cycles of overhead
  per access in exchange for soundness and idiomatic Rust.
