# Demo Examples

This document describes the QEMU-runnable demo examples included in
`kernel/examples/`. Each demo builds a complete partition layout with
SysTick-driven scheduling and exercises one or more IPC primitives.

All demos target the **LM3S6965EVB** machine under `qemu-system-arm` and
use semihosting for console output. Run any demo with:

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example <name> --features qemu
```

---

## sampling\_demo -- Sensor Telemetry Pipeline

**Source:** `kernel/examples/sampling_demo.rs`

### What It Demonstrates

- **Sampling ports** (latest-value, non-blocking IPC)
- Three-partition pipeline: sensor -> controller -> display
- `SYS_SAMPLING_WRITE` / `SYS_SAMPLING_READ` / `SYS_YIELD` syscalls
- Overwrite semantics (the controller always reads the *latest* value)

### Kernel Features Exercised

Sampling port creation and connection, SVC dispatch, PendSV context
switching, SysTick round-robin scheduling, partition stack initialization
via `init_stack_frame`.

### Architecture

```
Partitions: 3          Schedule: round-robin, 2 ticks each
Stacks:     1 KB each  SysTick reload: 120,000 cycles

Sampling ports (source -> destination):
  s0 -> d0   (sensor -> controller)
  s1 -> d1   (controller -> display)

+----------+   sampling    +------------+   sampling    +-----------+
| P0       |   port s0/d0  | P1         |   port s1/d1  | P2        |
| sensor   | ------------> | controller | ------------> | display   |
|          |   val (u8)    |            |   alert flag  |           |
+----------+               +------------+               +-----------+
  writes                     reads val,                   reads flag,
  incrementing               classifies                   prints
  sensor value               (>2 = ALERT),                status,
  each slot                  writes alert                 exits after
                             flag                         4 cycles
```

### How to Run

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example sampling_demo --features qemu
```

### Expected Output

The full output is ~1197 lines (excluding the cargo/QEMU header). The
sensor writes continuously during its time slot (~802 lines of
`[sensor]` output) before the SysTick scheduler switches to the
controller. The controller then reads repeatedly (~390 lines), followed
by 4 display lines and the final pass message.

Below is a representative slice captured from an actual run, showing the
start, the sensor-to-controller context switch boundary, the
controller-to-display boundary, and the end:

```
sampling_demo: start
[sensor]  write val=1
[sensor]  write val=2
[sensor]  write val=3
[sensor]  write val=4
[sensor]  write val=5
  (...sensor writes val=6 through val=30...)
[sensor]  write val=31
[sensor]  write val=32
[sensor]  write val=33                          <-- last sensor write before context switch
[control] val=33 valid=true -> ALERT            <-- controller scheduled; reads latest value
[control] val=33 valid=true -> ALERT
[control] val=33 valid=true -> ALERT
[control] val=33 valid=true -> ALERT
[control] val=33 valid=true -> ALERT
  (...~385 identical [control] lines...)
[control] val=33 valid=true -> ALERT            <-- last controller read before context switch
[display] status=ALERT valid=true               <-- display partition scheduled
[display] status=ALERT valid=true
[display] status=ALERT valid=true
[display] status=ALERT valid=true
sampling_demo: all checks passed
```

Key observation: because sampling ports use **overwrite semantics**, the
controller only ever sees the latest value written by the sensor
(val=33). The sensor wrote val=1 through val=33 during its time slot,
but intermediate values were silently overwritten. The threshold is
`> 2`, so every controller read is classified as ALERT. This
demonstrates that sampling ports are "last writer wins" -- readers
always get the most recent value, never a queue of historical values.

---

## queuing\_demo -- Command/Response Pipeline

**Source:** `kernel/examples/queuing_demo.rs`

### What It Demonstrates

- **Queuing ports** (FIFO message passing)
- Two-partition request/reply pattern
- Queue-full detection (5 sends into a depth-4 queue)
- `SYS_QUEUING_SEND` / `SYS_QUEUING_RECV` / `SYS_YIELD` syscalls

### Kernel Features Exercised

Queuing port creation and connection, FIFO ordering, queue-full error
reporting (`SvcError` with high bit set), SVC dispatch, PendSV context
switching, SysTick scheduling.

### Architecture

```
Partitions: 2             Schedule: round-robin, 2 ticks each
Stacks:     1 KB each     Queue depth: 4, message size: 4 bytes
SysTick reload: 120,000 cycles

Queuing ports (bidirectional pair):
  cs -> cd   (commander -> worker, commands)
  rs -> rd   (worker -> commander, responses)

+------------+   queuing port   +----------+
| P0         |   cs -> cd       | P1       |
| commander  | ---------------> | worker   |
|            |                  |          |
|            | <--------------- |          |
|            |   rs -> rd       |          |
+------------+   (responses)    +----------+

Protocol:
  Commander sends 5 commands (CMD_START=1, CMD_MEASURE=2,
  CMD_STOP=3, CMD_EXTRA_1=4, CMD_EXTRA_2=5).
  Queue depth is 4 so the 5th send fails (queue full).
  Worker maps each command to an ACK response byte.
  Commander receives and validates all 4 responses.
```

### How to Run

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example queuing_demo --features qemu
```

### Expected Output

```
queuing_demo: start
[commander] sent cmd=1 depth=1
[commander] sent cmd=2 depth=2
[commander] sent cmd=3 depth=3
[commander] sent cmd=4 depth=4
[commander] send cmd=5 -> QUEUE FULL (expected)
[commander] queue-full correctly detected, 4 delivered
[worker]    cmd=1 -> rsp=0x10
[worker]    cmd=2 -> rsp=0x20
[worker]    cmd=3 -> rsp=0x30
[worker]    cmd=4 -> rsp=0x40
[commander] recv rsp=0x10 (expected 0x10)
[commander] recv rsp=0x20 (expected 0x20)
[commander] recv rsp=0x30 (expected 0x30)
[commander] recv rsp=0x40 (expected 0x40)
queuing_demo: all checks passed
```

---

## blackboard\_demo -- Shared Configuration

**Source:** `kernel/examples/blackboard_demo.rs`

### What It Demonstrates

- **Blackboard** IPC (single-message broadcast buffer with wake-all)
- **Semaphore**-guarded critical section
- **Event flags** for worker-to-config completion signalling
- Three primitives composed in one demo
- `SYS_BB_DISPLAY` / `SYS_BB_READ` / `SYS_SEM_WAIT` / `SYS_SEM_SIGNAL` /
  `SYS_EVT_SET` / `SYS_EVT_WAIT` / `SYS_YIELD` syscalls

### Kernel Features Exercised

Blackboard creation, display (write-with-wake-all), and read. Counting
semaphore acquire/release for mutual exclusion. Event flag set/wait for
synchronization. SVC dispatch, PendSV context switching, SysTick
scheduling.

### Architecture

```
Partitions: 3             Schedule: round-robin, 2 ticks each
Stacks:     1 KB each     Blackboard: 4-byte messages, wait-queue depth 3
SysTick reload: 120,000   Semaphore: binary (count=1, max=1)
  cycles

                     blackboard (bb)
                   one shared buffer
                          |
+----------+    display   |   read     +----------+
| P0       | ---------->[ bb ]-------> | P1       |
| config   |              |            | worker_a |
|          |              |            +----------+
|          |              |
|          |              +----------> +----------+
|          |                   read    | P2       |
|          |                           | worker_b |
+----------+                           +----------+
      |         sem (binary)                |
      |   <---- evt 0x01 (worker_a) -------+
      |   <---- evt 0x02 (worker_b) -------+

  Config (P0)                Workers (P1, P2)
  ~~~~~~~~~~                 ~~~~~~~~~~~~~~~~
  2 rounds:                  each round:
    display [v, thresh]        read blackboard
    on blackboard              acquire semaphore
    wait for events            release semaphore
    0x01 AND 0x02              signal event (0x01 or 0x02)
```

### How to Run

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example blackboard_demo --features qemu
```

### Expected Output

```
blackboard_demo: start
[config] display v=1 thresh=10
[config] round 0 done
[config] display v=2 thresh=11
[config] round 1 done
blackboard_demo: all checks passed
```

Note: the worker partitions (`worker_a`, `worker_b`) contain `hprintln!`
calls that would print `[wrkA] cfg v=... thresh=...` and
`[wrkB] cfg v=... thresh=...` lines along with semaphore
acquire/release messages. However, these lines do not appear in the
captured output. The config partition calls `debug::exit(EXIT_SUCCESS)`
as soon as both event flags are received, which terminates the QEMU
process before semihosting flushes the worker output. The workers *do*
execute (they must, in order to set the event flags that unblock config),
but their print output is lost to the early exit. The demo's correctness
is validated by the event flag round-trip, not by worker console output.

---

## integration -- Full Kernel Integration Test

**Source:** `kernel/examples/integration.rs`

### What It Demonstrates

- SysTick-driven partition scheduling with MPU validation
- PendSV context switching between two busy-loop partitions
- Kernel-level message queue send/receive
- Event flag set and verification
- End-to-end kernel smoke test (scheduling + MPU + IPC)

### Kernel Features Exercised

`KernelState` and `ScheduleTable` initialization, `advance_schedule_tick`,
`partition_mpu_regions` validation, `MessageQueue` send/receive,
`events::event_set`, PendSV global assembly, `init_stack_frame`, partition
state transitions. Unlike the IPC demos, all IPC testing happens inside
the SysTick handler (kernel-level API calls, not SVC).

### Architecture

```
Partitions: 2             Schedule: round-robin, 3 ticks each
Stacks:     1 KB each     SysTick reload: 120,000 cycles

+----------+                         +----------+
| P0       |   <--- PendSV switch    | P1       |
| p0_main  |         back and        | p1_main  |
| (busy    |         forth --->      | (busy    |
|  loop)   |                         |  loop)   |
+----------+                         +----------+
     |                                     |
     +------ P_RAN atomic (tracks who ran last)
                        |
              SysTick handler tests:
                1. MPU region validity on each switch
                2. MessageQueue send (P0) + recv (P1)
                3. event_set on P0, flag verification
                4. Context switch confirmation via P_RAN
```

### How to Run

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example integration --features qemu
```

### Expected Output

```
integration: start
[PASS] MPU + switch 1 -> P1
[PASS] msg_send + msg_recv
[PASS] event_flag ack
[PASS] MPU + switch 2 -> P0
[PASS] MPU + switch 3 -> P1
[PASS] MPU + switch 4 -> P0
[PASS] ctx switch (last P1)
integration: all checks passed
```

---

## Lower-Level Examples

These smaller examples each exercise a single kernel subsystem. They
are useful for understanding individual building blocks before looking
at the full IPC demos.

| Example | Description |
|---------|-------------|
| `hello` | Minimal semihosting "hello from kernel" -- verifies toolchain and QEMU setup. |
| `panic_test` | Intentional `panic!()` to verify panic-semihosting handler. |
| `systick` | SysTick exception handler counting ticks, exits after 5. |
| `mpu_info` | Queries the MPU type register and prints the number of available regions. |
| `mpu_region` | Configures a single MPU region (256 B, RW/XN at 0x2000\_0000) and verifies RBAR/RASR. |
| `mpu_wx` | Demonstrates write-execute restriction with multiple MPU regions. |
| `mpu_fault` | Triggers a MemManage fault by accessing a no-access MPU region and handles it. |
| `mpu_partition` | Verifies `partition_mpu_regions` computes correct RBAR/RASR for two partitions. |
| `context_switch` | Two-partition PendSV context switch with separate stacks, monitored by SysTick. |
| `partition_switch` | SysTick-driven partition toggling with MPU region updates on each switch (6 switches). |
| `scheduler_tick` | Full `KernelState` + `ScheduleTable` integration with SysTick-driven scheduling. |
| `svc_yield` | SVC handler integration -- links the kernel SVC trampoline and dispatches a yield. |

Run any of these with:

```bash
cargo run -p kernel --target thumbv7m-none-eabi --example <name> --features qemu
```
