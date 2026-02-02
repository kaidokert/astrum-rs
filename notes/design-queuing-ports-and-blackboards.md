> **Deprecated**: This design document is superseded by
> [notes/ipc-reference.md](ipc-reference.md), which contains the
> consolidated, implementation-accurate IPC reference.

# Design: Queuing Ports and Blackboards

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [Queuing Port Data Structure Layout](#2-queuing-port-data-structure-layout)
3. [Queuing Port Ring Buffer Design](#3-queuing-port-ring-buffer-design)
4. [Blackboard Data Structure Layout](#4-blackboard-data-structure-layout)
5. [Memory Budget](#5-memory-budget)
6. [SVC Call Interface Specification](#6-svc-call-interface-specification)
7. [Blocking/Wakeup Protocol](#7-blockingwakeup-protocol) — includes priority inversion analysis (7.8)
8. [Timeout Expiry Integration with Scheduler Tick](#8-timeout-expiry-integration-with-scheduler-tick)
9. [Port Connection Routing for Inter-Partition Communication](#9-port-connection-routing-for-inter-partition-communication)
10. [SVC Dispatch Integration](#10-svc-dispatch-integration)
11. [Extended Kernel Struct](#11-extended-kernel-struct)
12. [Unit Testing Strategy](#12-unit-testing-strategy)
13. [Risks and Open Questions](#13-risks-and-open-questions)
14. [Future Work](#14-future-work)

---

## 1. Problem Statement

The kernel currently supports sampling ports (latest-value, non-blocking
IPC) and intra-partition synchronization (event flags, semaphores, mutexes,
message queues). It lacks two ARINC 653-style inter-partition communication
primitives needed for reliable ordered delivery and shared configuration:

**Queuing ports** provide bounded FIFO message passing between partitions
with blocking send/receive and timeout support. Use cases: commands,
requests, log entries, any scenario requiring ordered reliable delivery.

**Blackboards** provide a single shared message buffer with
display/read/clear semantics and blocking reads. Use cases: shared
configuration, status announcements, one-shot broadcasts.

Both primitives build on patterns already established in the codebase:
const-generic static allocation via `heapless`, pool-based resource
management, outcome-enum-driven state transitions, and SVC dispatch
through the `Kernel::dispatch` method.

### Relationship to Existing Primitives

| Primitive | Buffering | Blocking | Direction | Routing |
|-----------|-----------|----------|-----------|---------|
| Message Queue (existing) | FIFO, fixed-size | Yes (send/recv) | None | Intra-partition |
| Sampling Port (exists) | Latest-value | No | Source/Dest | Inter-partition |
| **Queuing Port** (new) | FIFO, variable-size | Yes (send/recv + timeout) | Source/Dest | Inter-partition |
| **Blackboard** (new) | Single message | Yes (read + timeout) | None | Any partition |

Queuing ports differ from existing message queues in three ways: (1)
directional Source/Dest ports with inter-partition routing, (2) variable
message sizes up to `M` bytes per message (message queues require exact
`M`-byte messages), and (3) deadline-based timeout on blocking operations.

---

## 2. Queuing Port Data Structure Layout

### 2.1 Control Block

```rust
/// Control block for a single queuing port.
pub struct QueuingPort<const D: usize, const M: usize, const W: usize> {
    direction: PortDirection,                      // Source or Destination
    buf: heapless::Deque<(usize, [u8; M]), D>,     // Ring buffer: (actual_size, data) pairs
    wait_queue: heapless::Deque<WaitEntry, W>,     // Blocked partition entries
    connected_port: Option<u8>,                     // Routing target (set by ConnectionTable)
    partition_id: u8,                               // Owning partition
}
```

**Field details**:

- `direction`: `PortDirection::Source` or `PortDirection::Destination`.
  Source ports can only send; destination ports can only receive.
  Reuses the `PortDirection` enum from `sampling.rs`.
- `buf`: The ring buffer holding enqueued messages. Each entry is a
  `(usize, [u8; M])` tuple where `usize` records the actual message
  size (which may be less than `M`). `D` is the maximum number of
  messages. See Section 3 for ring buffer design details.
- `wait_queue`: FIFO queue of `WaitEntry` structs for partitions
  blocked on this port. For Source ports, this holds senders blocked
  when the connected destination is full. For Destination ports, this
  holds receivers blocked when the buffer is empty.
- `connected_port`: Index into the same `QueuingPortPool` identifying
  the peer port. Set during init by `ConnectionTable::resolve()`.
  `None` if not connected (send will return `NoConnection`).
- `partition_id`: The partition that owns this port. **Enforced by the
  kernel** on every SVC call: the dispatcher compares
  `port.partition_id` against `self.active_partition()` (the caller
  identity determined internally by the scheduler, not passed in a
  register) and returns `ERR_NOT_OWNER` if they differ. A partition
  cannot send through another partition's source port, nor receive
  from another partition's destination port. This is a hard kernel
  enforcement, not merely a convention.

### 2.2 Wait Entry

```rust
/// Entry in a wait queue, tracking the blocked partition and its timeout deadline.
#[derive(Debug, Clone, Copy)]
pub struct WaitEntry {
    pub partition_id: u8,
    pub deadline: u64,    // Tick at which timeout expires; u64::MAX = infinite wait
}
```

This struct is shared between queuing ports and blackboards. It should
live in a foundational module (e.g., `ipc_common.rs`) that has **no
dependencies** on higher-level primitive modules (`queuing.rs`,
`blackboard.rs`, `routing.rs`). Placing it in `routing.rs` was
considered but rejected because `routing.rs` depends on port pool types
(`SamplingPortPool`, `QueuingPortPool`) for its `resolve()` method,
which would create a circular dependency if those pool types also
imported `WaitEntry` from `routing.rs`. The `ipc_common.rs` module
contains only leaf types (`WaitEntry`, `PortDirection`) that are
imported by all IPC modules but import nothing from them.

**Size and alignment**: The raw field sizes are 9 bytes (1 byte
`partition_id` + 8 bytes `deadline`). On 32-bit ARM (Cortex-M3/M4),
`u64` has 8-byte alignment. With `#[derive(Clone, Copy)]` and default
Rust layout, the compiler inserts 7 bytes of padding after the `u8`
field to satisfy the `u64` alignment requirement, giving a total struct
size of **16 bytes** (1 byte `partition_id` + 7 bytes padding + 8 bytes
`deadline`). This can be verified at compile time with:

```rust
const_assert!(core::mem::size_of::<WaitEntry>() == 16);
const_assert!(core::mem::align_of::<WaitEntry>() == 8);
```

**Note**: Using `#[repr(C)]` would yield the same 16-byte layout on
ARM targets. A `#[repr(packed)]` annotation could reduce the size to
9 bytes by eliminating padding, but packed structs on ARM require
unaligned memory access (which Cortex-M3 supports for normal loads/
stores but at a performance cost) and create references that are
unsound to use with `&` in safe Rust. The 7-byte waste per entry is
acceptable given the small wait-queue depths (W=4, so 28 bytes wasted
per port).

### 2.3 Pool

```rust
/// Pool of queuing ports, indexed by integer ID.
pub struct QueuingPortPool<const S: usize, const D: usize, const M: usize, const W: usize> {
    ports: heapless::Vec<QueuingPort<D, M, W>, S>,
}
```

### 2.4 Const-Generic Parameters

| Parameter | Meaning | Suggested Default | Rationale |
|-----------|---------|-------------------|-----------|
| `S` | Max number of queuing ports | 8 | 4 bidirectional pairs = 8 ports |
| `D` | Queue depth (messages per port) | 8 | Enough to buffer a burst; bounded to limit RAM |
| `M` | Max message size (bytes) | 64 | Sufficient for command structs; matches sampling port `M` |
| `W` | Wait-queue depth per port | 4 | At most 4 partitions can block on one port |

### 2.5 Outcome Enums

```rust
#[derive(Debug, PartialEq, Eq)]
pub enum QPortSendOutcome {
    /// Message was enqueued in the destination port's ring buffer.
    /// If `wake_receiver` is `Some(pid)`, transition that partition Ready.
    Delivered { wake_receiver: Option<u8> },
    /// Destination buffer is full and caller specified timeout > 0.
    /// The caller has been placed on the source port's wait queue.
    SenderBlocked { blocked: u8 },
    /// Destination buffer is full and timeout was 0 (non-blocking).
    Full,
}

#[derive(Debug, PartialEq, Eq)]
pub enum QPortRecvOutcome {
    /// A message was dequeued. If `wake_sender` is `Some(pid)`,
    /// transition that partition Ready.
    Received { size: usize, wake_sender: Option<u8> },
    /// Buffer is empty and caller specified timeout > 0.
    /// The caller has been placed on the destination port's wait queue.
    ReceiverBlocked { blocked: u8 },
    /// Buffer is empty and timeout was 0 (non-blocking).
    Empty,
}

#[derive(Debug, PartialEq, Eq)]
pub enum QPortError {
    InvalidPort,
    WrongDirection,
    SizeTooLarge,
    WaitQueueFull,
    NoConnection,
    NotOwner,
    BufTooSmall,
}
```

These follow the pattern from `message.rs` (`SendOutcome`/`RecvOutcome`)
with three additions: `Full`/`Empty` for non-blocking failures,
`WaitEntry` deadlines for timeout, and ownership validation.

**`WaitQueueFull` semantics**: When a partition attempts a blocking
operation (timeout > 0) and the wait queue is already at capacity `W`,
the kernel returns `ERR_WAIT_FULL` to the caller. The caller is NOT
blocked — it remains `Running` and receives the error code immediately.
This is a non-fatal, recoverable error: the partition can retry later,
fall back to non-blocking mode, or treat it as a transient overload.

`WaitQueueFull` indicates a **static configuration mismatch**: the
wait-queue depth `W` is too small for the number of partitions that can
concurrently block on this port. Since `W` is a compile-time constant,
the system integrator should set `W >= N` (number of partitions) if all
partitions may block on the same resource simultaneously. In practice,
the suggested default `W=4` equals the maximum partition count in the
demo configurations, so this error should not occur in correctly
configured systems.

### 2.6 Status Query

```rust
pub struct QPortStatus {
    pub current_depth: usize,    // Messages currently in buffer
    pub max_depth: usize,        // D (compile-time max)
    pub max_msg_size: usize,     // M (compile-time max)
    pub direction: PortDirection,
    pub waiting_count: usize,    // Partitions blocked on this port
}
```

Returned by `QueuingPortPool::status(port_id)`. The SVC path returns
only `current_depth` in `r0` to keep the calling convention simple.

---

## 3. Queuing Port Ring Buffer Design

### 3.1 Choice: `heapless::Deque`

The ring buffer uses `heapless::Deque<(usize, [u8; M]), D>`, the same
`heapless::Deque` type already used by existing message queues in
`message.rs`. This provides:

- `push_back()`: O(1) enqueue
- `pop_front()`: O(1) dequeue (FIFO ordering)
- `len()` / `is_full()`: O(1) capacity queries
- No heap allocation; fully static, const-constructible
- Already a dependency (no new crates needed)

### 3.2 Element Format: `(usize, [u8; M])`

Each ring buffer entry stores a `(usize, [u8; M])` tuple:

```
+-------+------------------+
| size  | data: [u8; M]    |
| usize | (padded to M)    |
+-------+------------------+
```

The `size` field records the actual number of valid bytes in `data`.
Messages smaller than `M` bytes waste the trailing space. This is the
simplest approach that avoids variable-length allocation in `no_alloc`
environments.

**Why not just `[u8; M]` like existing message queues?** Existing
message queues (`MessageQueue` in `message.rs`) require messages to be
exactly `M` bytes — `send()` returns `SizeMismatch` if `data.len() != M`.
Queuing ports accept variable-size messages up to `M`, which is more
practical for real payloads (command structs of different sizes). The
`usize` prefix tracks the actual size.

**Space overhead**: `size_of::<usize>()` = 4 bytes per message on
Cortex-M (32-bit). For `M=64`, each slot is 68 bytes (64 data + 4 size).
With `D=8` slots per port: 544 bytes for the ring buffer alone.

### 3.3 Alternative Considered: Byte-Level Ring Buffer

A byte-level ring buffer (single `[u8; D*M]` array with head/tail
pointers and per-message length headers) would eliminate internal
fragmentation for small messages. Rejected because:

- Significantly more complex (variable-length entries, wraparound
  splitting, compaction)
- `heapless::Deque` is well-tested and already in use
- The wasted space for small messages is bounded by `D * M` total,
  which is a compile-time budget the integrator explicitly accepts
- Message size variability in RTOS command protocols is typically small
  (most messages are near `M`)

### 3.4 Alternative Considered: Separate Size Array

Store sizes in a `heapless::Deque<usize, D>` parallel to
`heapless::Deque<[u8; M], D>`. Rejected because:

- Two deques must stay synchronized (push/pop both atomically)
- No real advantage over the tuple approach
- Tuple approach is used successfully in similar embedded RTOS designs

### 3.5 Ring Buffer Invariants

1. `buf.len()` always equals the number of enqueued messages.
2. Messages are dequeued in FIFO order (`pop_front`).
3. `buf.is_full()` (i.e., `buf.len() == D`) means the queue is at
   capacity; new sends must block or return `Full`.
4. `buf.is_empty()` means no messages available; receives must block
   or return `Empty`.

---

## 4. Blackboard Data Structure Layout

### 4.1 Control Block

```rust
/// Control block for a single blackboard.
pub struct Blackboard<const M: usize, const W: usize> {
    data: [u8; M],                                 // Message buffer
    current_size: usize,                            // Actual size (0 when cleared/never written)
    is_empty: bool,                                 // true initially and after clear
    wait_queue: heapless::Deque<WaitEntry, W>,     // Readers blocked on empty blackboard
}
```

**Field details**:

- `data`: Single message buffer. Overwritten on each `display()`.
- `current_size`: Actual message size. Set to 0 on clear.
- `is_empty`: Distinguishes "never written / cleared" from "written
  with 0-length message". When `is_empty == true`, reads block (or
  return `Empty` with timeout=0). When `is_empty == false`, reads
  succeed immediately.
- `wait_queue`: FIFO of readers blocked while the blackboard is empty.
  When `display()` writes new content, ALL waiting readers are woken
  (broadcast wakeup). **Thundering herd note**: Broadcast wakeup can
  in general cause scheduling jitter by transitioning multiple partitions
  to `Ready` simultaneously. However, this risk is mitigated for
  blackboards because (a) read does not consume the data, so all woken
  readers will successfully read on retry, and (b) the static ARINC 653
  schedule table spreads retry SVCs across separate time slots, preventing
  contention. See Section 7.3 for the full analysis.

**`clear()` semantics**: `clear()` sets `is_empty = true` and
`current_size = 0`. Partitions currently blocked in the wait queue
**remain blocked** — they are NOT woken. Rationale: blocked readers
are waiting for data to become available. `clear()` removes data,
making the blackboard "more empty," not "newly available." Waking
readers on clear would cause them to immediately re-block (the board
is still empty) or receive an `Empty` error, wasting a syscall
round-trip per woken partition with no benefit. Blocked readers will
be woken when a subsequent `display()` writes new data, or when their
timeout expires. This matches the ARINC 653 `CLEAR_BLACKBOARD`
semantics where clearing does not unblock waiting readers.

The `clear()` implementation:

```rust
pub fn clear(&mut self, bb_id: usize) -> Result<(), BbError> {
    let board = self.boards.get_mut(bb_id)
        .ok_or(BbError::InvalidBlackboard)?;
    board.is_empty = true;
    board.current_size = 0;
    // Wait queue is NOT drained — blocked readers remain Waiting.
    Ok(())
}
```

**Atomicity**: All operations on blackboard fields (`is_empty`,
`current_size`, `data`, `wait_queue`) execute inside the SVC handler,
which runs in Handler mode at SVCall priority 0x00 — the highest
configurable interrupt priority on Cortex-M. On single-core Cortex-M,
this provides three guarantees:

1. **SVC is non-preemptible by partition code.** No partition can
   execute while an SVC handler is running, so a `read()` checking
   `is_empty` and then copying `data` cannot be interleaved with
   a concurrent `display()` modifying those same fields.

2. **SVC is non-preemptible by SysTick.** SysTick priority (0xFE)
   is lower than SVCall (0x00). The SysTick timeout sweep (Section 8)
   cannot modify wait queues while an SVC operation is also accessing
   them. If SysTick fires during SVC, it pends until SVC completes.

3. **SysTick is non-preemptible by SVC.** Since no partition code
   runs during SysTick, no SVC instruction can fire during the
   timeout sweep. This prevents the reverse interleaving.

**Implication**: No additional synchronization primitives (critical
sections, atomics, disable-interrupt guards) are required for any
blackboard (or queuing port) operation. The interrupt priority
configuration is the sole concurrency control mechanism. This is a
critical architectural invariant — if the priorities were misconfigured
(e.g., SysTick at higher priority than SVCall), wait queue corruption
could occur. Section 7.6.5 specifies a boot-time priority verification
assertion to prevent this.

See Section 7.6 for the complete atomicity analysis covering all
interaction scenarios, including the double-wakeup prevention proof.

**No direction field**: Unlike ports, blackboards have no directional
constraint. Any partition can display, read, or clear any blackboard.

**Access model — open by design**: Blackboards intentionally do not
enforce kernel-level access control. Any partition can display, read,
or clear any blackboard by ID. This is a deliberate design decision,
not an oversight, based on the following reasoning:

1. **ARINC 653 precedent**: In ARINC 653, blackboards are shared
   communication buffers accessible by multiple processes within a
   partition. This design extends the model to inter-partition use
   (shared across partitions), consistent with the blackboard's role
   as a broadcast mechanism for shared state (configuration, mode
   announcements, status).

2. **Use case alignment**: The primary use cases — shared configuration
   (`mode=ACTIVE`), status broadcasts, and one-shot announcements —
   inherently require multi-writer or at least multi-reader access.
   Adding per-blackboard ACLs would add complexity (additional
   const-generic parameters, per-blackboard writer/reader lists, new
   error codes) without clear benefit for the current system scope.

3. **Risks of the open model**: A buggy or misbehaving partition could:
   - **Corrupt shared state**: Write incorrect data to a blackboard
     that other partitions rely on for configuration.
   - **Clear prematurely**: Clear a blackboard before all readers have
     consumed the data, causing them to block unexpectedly.
   - **Interfere with blocking**: Display empty/garbage data that wakes
     blocked readers with meaningless content.

4. **Mandatory application-level conventions**: To prevent misuse, the
   system integrator MUST follow these conventions:
   - **Single-writer convention**: Each blackboard should have exactly
     one designated writer partition. Document this in the system
     configuration. Only the designated writer should call
     `SYS_BB_DISPLAY` and `SYS_BB_CLEAR` on that blackboard.
   - **Well-known IDs**: Assign blackboard IDs by convention (e.g.,
     `BB_CONFIG=0`, `BB_STATUS=1`) and document which partition owns
     each.
   - **Defense via MPU isolation**: Spatial isolation (MPU) prevents
     partitions from directly accessing each other's memory. Blackboard
     corruption is limited to the kernel-mediated `display()` path,
     which at least guarantees size validation (`SizeTooLarge`).

5. **Future hardening**: Kernel-enforced blackboard access control
   (per-blackboard writer partition ID, reader partition bitmask) is
   deferred to future work. Adding it is straightforward: store
   `writer_pid: Option<u8>` and `reader_mask: u8` in the `Blackboard`
   struct, check on each `display`/`read`/`clear` call, return a new
   `ERR_NOT_AUTHORIZED` error code.

**No `partition_id` field**: Since blackboards are multi-writer,
multi-reader under the current access model, there is no single
owner to validate against. This field will be added if kernel-
enforced access control is implemented (see point 5 above).

### 4.2 Pool

```rust
/// Pool of blackboards, indexed by integer ID.
///
/// The pool manages up to `S` blackboards, each with a maximum message
/// size of `M` bytes and a wait-queue depth of `W`. Blackboards are
/// added during kernel initialization via `add()` and accessed by
/// integer index at runtime. All blackboards in a pool share the same
/// `M` and `W` parameters (see R4 in Section 13 for discussion of
/// this limitation).
pub struct BlackboardPool<const S: usize, const M: usize, const W: usize> {
    boards: heapless::Vec<Blackboard<M, W>, S>,
}
```

The pool follows the same pattern as `SemaphorePool`, `MutexPool`,
`MessagePool`, and `SamplingPortPool`: a `heapless::Vec` of control
blocks indexed by integer ID. The `add()` method appends a new
blackboard and returns its index. Index-based lookup is O(1).

### 4.3 Const-Generic Parameters

| Parameter | Meaning | Suggested Default | Rationale |
|-----------|---------|-------------------|-----------|
| `S` | Max number of blackboards | 4 | Typical: config, status, mode, alerts |
| `M` | Max message size (bytes) | 64 | Matches queuing/sampling port `M` |
| `W` | Wait-queue depth per board | 4 | At most 4 partitions can block on one board |

### 4.4 Outcome Enums

```rust
#[derive(Debug, PartialEq, Eq)]
pub enum BbReadOutcome {
    /// Message was read successfully.
    Read { size: usize },
    /// Blackboard is empty and caller specified timeout > 0.
    /// Caller has been placed on the wait queue.
    ReaderBlocked { blocked: u8 },
    /// Blackboard is empty and timeout was 0 (non-blocking).
    Empty,
}

#[derive(Debug, PartialEq, Eq)]
pub enum BbError {
    InvalidBlackboard,
    SizeTooLarge,
    WaitQueueFull,
    BufTooSmall,
}
```

`display()` does not need an outcome enum because its wakeup behavior
is handled inline (see Section 7.3).

---

## 5. Memory Budget

### 5.1 Per-Instance Costs

#### Queuing Port

| Component | Size | Notes |
|-----------|------|-------|
| `direction` | 1 byte | Enum, single byte |
| `connected_port` | 2 bytes | `Option<u8>` |
| `partition_id` | 1 byte | |
| Ring buffer: `D` entries of `(usize, [u8; M])` | `D * (4 + M)` | 4 bytes for `usize` on ARM |
| Wait queue: `W` entries of `WaitEntry` | `W * 16` | 16 bytes per entry (see Section 2.2 for alignment analysis) |
| `heapless::Deque` overhead (2 deques) | ~24 bytes | Head/tail indices, len |

**Per-port total** with `D=8, M=64, W=4`:
`4 + 8*(4+64) + 4*16 + 24 = 4 + 544 + 64 + 24 = 636 bytes`

#### Blackboard

| Component | Size | Notes |
|-----------|------|-------|
| `data: [u8; M]` | `M` bytes | |
| `current_size` | 4 bytes | `usize` on ARM |
| `is_empty` | 1 byte | `bool` |
| Wait queue: `W` entries of `WaitEntry` | `W * 16` | 16 bytes per entry (see Section 2.2) |
| `heapless::Deque` overhead | ~12 bytes | |

**Per-board total** with `M=64, W=4`:
`64 + 4 + 1 + 64 + 12 = 145 bytes` (~146 bytes rounded)

### 5.2 Total Budget (Suggested Defaults)

| Component | Per-Instance | Count | Total |
|-----------|-------------|-------|-------|
| QueuingPort (D=8, M=64, W=4) | ~636 bytes | 8 | ~5,088 bytes |
| Blackboard (M=64, W=4) | ~146 bytes | 4 | ~584 bytes |
| **Total new** | | | **~5,672 bytes** |

Combined with sampling ports (~704 bytes for 8 ports) and the
connection table (~64 bytes), total new IPC memory is ~6,440 bytes.
This is well within the 64 KB RAM of the lm3s6965evb target. With
4 partitions using 1 KB stacks each (~4 KB), existing IPC pools
(~1 KB), and kernel data structures (~1 KB), total usage is ~12.4 KB,
leaving ~51.6 KB headroom.

### 5.3 Tuning

For memory-constrained deployments, reduce `D` (queue depth) and `M`
(message size) first — these dominate the budget. `W` (wait-queue
depth) has minimal impact at 16 bytes per entry. Reducing `D` from 8
to 4 saves ~2.2 KB across 8 ports. Reducing `M` from 64 to 32 saves
~2.0 KB across 8 ports.

---

## 6. SVC Call Interface Specification

### 6.1 Numbering

Existing syscalls occupy 0-13 (with gap at 1). New syscalls continue
from 14:

| Number | Constant | Purpose |
|--------|----------|---------|
| 14 | `SYS_SAMP_STATUS` | Get sampling port status |
| 15 | `SYS_QPORT_SEND` | Send queuing port message |
| 16 | `SYS_QPORT_RECV` | Receive queuing port message |
| 17 | `SYS_QPORT_STATUS` | Get queuing port status |
| 18 | `SYS_BB_DISPLAY` | Display blackboard message |
| 19 | `SYS_BB_READ` | Read blackboard message |
| 20 | `SYS_BB_CLEAR` | Clear blackboard |

**Rationale for starting at 14**: SYS_SAMPLING_WRITE=12 and
SYS_SAMPLING_READ=13 are already assigned. SYS_SAMP_STATUS=14 fills
the gap for sampling port status introspection. Queuing ports (15-17)
and blackboards (18-20) follow in logical groups.

### 6.2 Register Convention

All syscalls follow the existing convention:

| Register | Purpose |
|----------|---------|
| `r0` (entry) | Syscall number |
| `r0` (exit) | Return value: 0 = success, 1 = non-blocking failure (Full/Empty), 0xFFFF_FFxx = error code |
| `r1` | Resource ID (port_id or bb_id) |
| `r2` | Data size (send) or buffer capacity (receive); unused for status/clear |
| `r3` | Pointer to data or buffer struct; unused for status queries returning scalar |

The caller's partition ID is **never** passed in a register — the kernel
reads it from `self.active_partition()`, which is set by the scheduler.

**Uniform `r2` convention**: `r2` describes only the message data size
or data buffer capacity. It never includes the size of struct headers
(timeout fields, size fields, etc.) that surround the data in the
pointed-to struct. The kernel adds the header size internally when
computing pointer validation bounds.

### 6.3 Per-Syscall Function Signatures and Parameters

#### `SYS_QPORT_SEND` (15) — Send Queuing Port Message

**Pool method signature**:

```rust
pub fn send(
    &mut self,
    port_id: usize,
    caller: usize,
    data: &[u8],
    msg_size: usize,
    timeout_ticks: u64,
    current_tick: u64,
) -> Result<QPortSendOutcome, QPortError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 15 (`SYS_QPORT_SEND`) |
| `r1` | `port_id` — index into `QueuingPortPool` |
| `r2` | `msg_size` — number of message data bytes |
| `r3` | `data_ptr` — pointer to `QueuingSendArgs` struct |
| `r0` (out) | `0` = delivered or blocked, `1` = full (non-blocking), `0xFFFF_FFxx` = error |

**Pointer layout** (`data_ptr` points to):

```rust
#[repr(C)]
struct QueuingSendArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    data: [u8; /* msg_size */],   // Message payload (msg_size from r2)
}
```

**Pointer validation size**: `4 + msg_size` bytes (4-byte timeout prefix + data).

**Operation steps**:

1. Validate `port_id` is in range → `QPortError::InvalidPort`.
2. Validate `port.partition_id == caller` → `QPortError::NotOwner`.
3. Validate `port.direction == Source` → `QPortError::WrongDirection`.
4. Validate `msg_size <= M` → `QPortError::SizeTooLarge`.
5. Resolve `connected_port` → `QPortError::NoConnection` if `None`.
6. Attempt `push_back((msg_size, data))` on destination port's `buf`.
7. If destination has space:
   - Enqueue the message.
   - If destination's wait queue has a blocked receiver, `pop_front()` it.
   - Return `Delivered { wake_receiver }`.
8. If destination is full:
   - If `timeout_ticks == 0`: return `Full`.
   - If wait queue is at capacity `W`: return `Err(QPortError::WaitQueueFull)`.
   - Compute `deadline`: if `timeout_ticks == u32::MAX` then `u64::MAX`, else `current_tick + timeout_ticks as u64`.
   - Push `WaitEntry { partition_id: caller, deadline }` onto **source port's** wait queue.
   - Return `SenderBlocked { blocked: caller }`.

**Return value mapping**:

| Outcome | `r0` | Partition transition |
|---------|------|---------------------|
| `Delivered { wake_receiver: None }` | `0` | None |
| `Delivered { wake_receiver: Some(pid) }` | `0` | `pid`: Waiting → Ready |
| `SenderBlocked { blocked }` | `0` | `blocked`: Running → Waiting |
| `Full` | `1` | None |
| `Err(QPortError::InvalidPort)` | `ERR_INVALID_ID` (0xFFFF_FFFE) | None |
| `Err(QPortError::NotOwner)` | `ERR_NOT_OWNER` (0xFFFF_FFF8) | None |
| `Err(QPortError::WrongDirection)` | `ERR_WRONG_DIR` (0xFFFF_FFFD) | None |
| `Err(QPortError::SizeTooLarge)` | `ERR_SIZE` (0xFFFF_FFFC) | None |
| `Err(QPortError::NoConnection)` | `ERR_NO_CONNECTION` (0xFFFF_FFFA) | None |
| `Err(QPortError::WaitQueueFull)` | `ERR_WAIT_FULL` (0xFFFF_FFF9) | None |
| Pointer validation failure | `ERR_BAD_PTR` (0xFFFF_FFF6) | None |

#### `SYS_QPORT_RECV` (16) — Receive Queuing Port Message

**Pool method signature**:

```rust
pub fn receive(
    &mut self,
    port_id: usize,
    caller: usize,
    buf: &mut [u8],
    buf_capacity: usize,
    timeout_ticks: u64,
    current_tick: u64,
) -> Result<QPortRecvOutcome, QPortError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 16 (`SYS_QPORT_RECV`) |
| `r1` | `port_id` — index into `QueuingPortPool` |
| `r2` | `buf_capacity` — capacity of the data portion of the receive buffer |
| `r3` | `buf_ptr` — pointer to `QueuingRecvArgs` struct |
| `r0` (out) | `0` = received or blocked, `1` = empty (non-blocking), `0xFFFF_FFxx` = error |

**Pointer layout** (`buf_ptr` points to):

```rust
#[repr(C)]
struct QueuingRecvArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    size: u32,                    // Filled by kernel: actual message size
    data: [u8; /* buf_capacity */],   // Filled by kernel (buf_capacity from r2)
}
```

**Pointer validation size**: `8 + buf_capacity` bytes (4-byte timeout + 4-byte size + data).

**Operation steps**:

1. Validate `port_id` → `QPortError::InvalidPort`.
2. Validate `port.partition_id == caller` → `QPortError::NotOwner`.
3. Validate `port.direction == Destination` → `QPortError::WrongDirection`.
4. If the port's ring buffer has messages:
   - Peek at the oldest `(size, data)`.
   - If `size > buf_capacity` → `Err(QPortError::BufTooSmall)`.
   - `pop_front()` the message. Copy `data[..size]` into caller's buffer.
   - Write `size` into the `QueuingRecvArgs.size` field.
   - If the connected source port's wait queue has a blocked sender, `pop_front()` it.
   - Return `Received { size, wake_sender }`.
5. If empty:
   - If `timeout_ticks == 0`: return `Empty`.
   - If wait queue is at capacity `W`: return `Err(QPortError::WaitQueueFull)`.
   - Compute `deadline`.
   - Push `WaitEntry` onto **destination port's** wait queue.
   - Return `ReceiverBlocked { blocked: caller }`.

**Return value mapping**:

| Outcome | `r0` | Partition transition |
|---------|------|---------------------|
| `Received { size, wake_sender: None }` | `0` | None |
| `Received { size, wake_sender: Some(pid) }` | `0` | `pid`: Waiting → Ready |
| `ReceiverBlocked { blocked }` | `0` | `blocked`: Running → Waiting |
| `Empty` | `1` | None |
| `Err(QPortError::InvalidPort)` | `ERR_INVALID_ID` | None |
| `Err(QPortError::NotOwner)` | `ERR_NOT_OWNER` | None |
| `Err(QPortError::WrongDirection)` | `ERR_WRONG_DIR` | None |
| `Err(QPortError::BufTooSmall)` | `ERR_BUF_TOO_SMALL` (0xFFFF_FFF7) | None |
| `Err(QPortError::WaitQueueFull)` | `ERR_WAIT_FULL` | None |
| Pointer validation failure | `ERR_BAD_PTR` | None |

#### `SYS_QPORT_STATUS` (17) — Get Queuing Port Status

**Pool method signature**:

```rust
pub fn status(&self, port_id: usize) -> Result<QPortStatus, QPortError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 17 (`SYS_QPORT_STATUS`) |
| `r1` | `port_id` |
| `r2` | unused |
| `r3` | unused |
| `r0` (out) | `current_depth` on success, `0xFFFF_FFxx` = error |

No pointer argument. Returns `current_depth` directly in `r0`.

**Return value mapping**:

| Outcome | `r0` |
|---------|------|
| `Ok(status)` | `status.current_depth` (0..D) |
| `Err(QPortError::InvalidPort)` | `ERR_INVALID_ID` |

**Note**: Because `current_depth` values (0..8 with default `D=8`) and
error codes (0xFFFF_FFxx) occupy non-overlapping ranges, there is no
ambiguity.

#### `SYS_BB_DISPLAY` (18) — Display Blackboard Message

**Pool method signature**:

```rust
pub fn display<const N: usize>(
    &mut self,
    parts: &mut PartitionTable<N>,
    bb_id: usize,
    data: &[u8],
) -> Result<(), BbError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 18 (`SYS_BB_DISPLAY`) |
| `r1` | `bb_id` — index into `BlackboardPool` |
| `r2` | `msg_size` — number of message data bytes |
| `r3` | `data_ptr` — pointer directly to message bytes (no header struct) |
| `r0` (out) | `0` = displayed, `0xFFFF_FFxx` = error |

**Pointer validation size**: `msg_size` bytes.

**Operation steps**:

1. Validate `bb_id` → `BbError::InvalidBlackboard`.
2. Validate `msg_size <= M` → `BbError::SizeTooLarge`.
3. Copy `data[..msg_size]` into `board.data[..msg_size]`.
4. Set `board.current_size = msg_size`, `board.is_empty = false`.
5. Drain entire wait queue: for each `WaitEntry`, transition partition `Waiting → Ready`.
6. Return `Ok(())`.

**Return value mapping**:

| Outcome | `r0` | Partition transitions |
|---------|------|---------------------|
| `Ok(())` | `0` | All waiters: Waiting → Ready |
| `Err(BbError::InvalidBlackboard)` | `ERR_INVALID_ID` | None |
| `Err(BbError::SizeTooLarge)` | `ERR_SIZE` | None |
| Pointer validation failure | `ERR_BAD_PTR` | None |

#### `SYS_BB_READ` (19) — Read Blackboard Message

**Pool method signature**:

```rust
pub fn read(
    &mut self,
    bb_id: usize,
    caller: u8,
    buf: &mut [u8],
    buf_capacity: usize,
    timeout_ticks: u64,
    current_tick: u64,
) -> Result<BbReadOutcome, BbError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 19 (`SYS_BB_READ`) |
| `r1` | `bb_id` — index into `BlackboardPool` |
| `r2` | `buf_capacity` — capacity of the data portion of the receive buffer |
| `r3` | `buf_ptr` — pointer to `BlackboardReadArgs` struct |
| `r0` (out) | `0` = read or blocked, `1` = empty (non-blocking), `0xFFFF_FFxx` = error |

**Pointer layout** (`buf_ptr` points to):

```rust
#[repr(C)]
struct BlackboardReadArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    size: u32,                    // Filled by kernel: actual message size
    data: [u8; /* buf_capacity */],   // Filled by kernel (buf_capacity from r2)
}
```

**Pointer validation size**: `8 + buf_capacity` bytes.

**Operation steps**:

1. Validate `bb_id` → `BbError::InvalidBlackboard`.
2. If `!board.is_empty`:
   - If `board.current_size > buf_capacity` → `Err(BbError::BufTooSmall)`.
   - Copy `board.data[..current_size]` into caller's buffer.
   - Write `current_size` into `BlackboardReadArgs.size`.
   - Return `BbReadOutcome::Read { size: current_size }`.
3. If `board.is_empty`:
   - If `timeout_ticks == 0`: return `BbReadOutcome::Empty`.
   - If wait queue is at capacity `W`: return `Err(BbError::WaitQueueFull)`.
   - Compute `deadline`.
   - Push `WaitEntry` onto the board's wait queue.
   - Return `BbReadOutcome::ReaderBlocked { blocked: caller }`.

**Return value mapping**:

| Outcome | `r0` | Partition transition |
|---------|------|---------------------|
| `Read { size }` | `0` | None |
| `ReaderBlocked { blocked }` | `0` | `blocked`: Running → Waiting |
| `Empty` | `1` | None |
| `Err(BbError::InvalidBlackboard)` | `ERR_INVALID_ID` | None |
| `Err(BbError::BufTooSmall)` | `ERR_BUF_TOO_SMALL` | None |
| `Err(BbError::WaitQueueFull)` | `ERR_WAIT_FULL` | None |
| Pointer validation failure | `ERR_BAD_PTR` | None |

#### `SYS_BB_CLEAR` (20) — Clear Blackboard

**Pool method signature**:

```rust
pub fn clear(&mut self, bb_id: usize) -> Result<(), BbError>
```

**Register layout**:

| Register | Value |
|----------|-------|
| `r0` (in) | 20 (`SYS_BB_CLEAR`) |
| `r1` | `bb_id` |
| `r2` | unused |
| `r3` | unused |
| `r0` (out) | `0` = cleared, `0xFFFF_FFxx` = error |

No pointer argument.

**Operation steps**:

1. Validate `bb_id` → `BbError::InvalidBlackboard`.
2. Set `board.is_empty = true`, `board.current_size = 0`.
3. Do NOT drain the wait queue. Blocked readers remain Waiting.
4. Return `Ok(())`.

**Return value mapping**:

| Outcome | `r0` |
|---------|------|
| `Ok(())` | `0` |
| `Err(BbError::InvalidBlackboard)` | `ERR_INVALID_ID` |

### 6.4 Error Codes

All new syscalls use the unified error code scheme (high `u32` values):

| Code | Constant | Meaning |
|------|----------|---------|
| `0xFFFF_FFFF` | `ERR_UNKNOWN` | Unrecognized syscall |
| `0xFFFF_FFFE` | `ERR_INVALID_ID` | Invalid port/blackboard ID |
| `0xFFFF_FFFD` | `ERR_WRONG_DIR` | Wrong port direction |
| `0xFFFF_FFFC` | `ERR_SIZE` | Message exceeds `max_msg_size` |
| `0xFFFF_FFF9` | `ERR_WAIT_FULL` | Wait queue full; cannot block |
| `0xFFFF_FFFA` | `ERR_NO_CONNECTION` | Source port has no connected destination |
| `0xFFFF_FFF8` | `ERR_NOT_OWNER` | Caller does not own this port |
| `0xFFFF_FFF7` | `ERR_BUF_TOO_SMALL` | Caller buffer too small for message |
| `0xFFFF_FFF6` | `ERR_BAD_PTR` | Pointer outside partition memory |

Error enums map to these codes via `From` impls:

```rust
impl From<QPortError> for u32 {
    fn from(e: QPortError) -> u32 {
        match e {
            QPortError::InvalidPort    => ERR_INVALID_ID,
            QPortError::WrongDirection => ERR_WRONG_DIR,
            QPortError::SizeTooLarge   => ERR_SIZE,
            QPortError::WaitQueueFull  => ERR_WAIT_FULL,
            QPortError::NoConnection   => ERR_NO_CONNECTION,
            QPortError::NotOwner       => ERR_NOT_OWNER,
            QPortError::BufTooSmall    => ERR_BUF_TOO_SMALL,
        }
    }
}

impl From<BbError> for u32 {
    fn from(e: BbError) -> u32 {
        match e {
            BbError::InvalidBlackboard => ERR_INVALID_ID,
            BbError::SizeTooLarge      => ERR_SIZE,
            BbError::WaitQueueFull     => ERR_WAIT_FULL,
            BbError::BufTooSmall       => ERR_BUF_TOO_SMALL,
        }
    }
}
```

### 6.5 `SyscallId` Enum Extension

Add to `syscall.rs`:

```rust
pub const SYS_SAMP_STATUS: u32 = 14;
pub const SYS_QPORT_SEND: u32 = 15;
pub const SYS_QPORT_RECV: u32 = 16;
pub const SYS_QPORT_STATUS: u32 = 17;
pub const SYS_BB_DISPLAY: u32 = 18;
pub const SYS_BB_READ: u32 = 19;
pub const SYS_BB_CLEAR: u32 = 20;

pub enum SyscallId {
    // ... existing variants through SamplingRead ...
    SampStatus,
    QPortSend,
    QPortRecv,
    QPortStatus,
    BbDisplay,
    BbRead,
    BbClear,
}
```

Update `from_u32()` and `as_u32()` match arms accordingly.

### 6.6 Pointer Validation

Every pointer received from user space is validated before the kernel
dereferences it. The check verifies that the entire referenced range
`[ptr, ptr + len)` falls within the calling partition's data region as
defined by its MPU configuration.

```rust
fn validate_user_ptr(pcb: &PartitionControlBlock, ptr: u32, len: u32) -> bool {
    if len == 0 {
        return true;
    }
    let region_base = pcb.mpu_region().base();
    let region_end = region_base.wrapping_add(pcb.mpu_region().size());
    let access_end = match ptr.checked_add(len) {
        Some(end) => end,
        None => return false,
    };
    ptr >= region_base && access_end <= region_end
}
```

The kernel calls `validate_user_ptr` early in each syscall handler and
returns `ERR_BAD_PTR` on failure, before any read/write to user memory.

**Validation sizes per syscall**:

| Syscall | Validated Size |
|---------|----------------|
| `SYS_QPORT_SEND` | `r2 + 4` (msg_size + timeout prefix) |
| `SYS_QPORT_RECV` | `r2 + 8` (buf_capacity + timeout + size prefix) |
| `SYS_QPORT_STATUS` | No pointer |
| `SYS_BB_DISPLAY` | `r2` (msg_size) |
| `SYS_BB_READ` | `r2 + 8` (buf_capacity + timeout + size prefix) |
| `SYS_BB_CLEAR` | No pointer |

---

## 7. Blocking/Wakeup Protocol

### 7.1 Queuing Port Send — Blocking Flow

When a partition calls `SEND_QUEUING_MESSAGE` with `timeout > 0` and the
connected destination's ring buffer is full:

```
Partition A              Kernel (SVC handler)              Partition B
    |                          |                               |
    |--SYS_QPORT_SEND(p0)---->|                               |
    |  timeout=100, data=[..]  |                               |
    |                          |  1. Validate port_id, owner,  |
    |                          |     direction, size, pointer  |
    |                          |  2. Resolve connected dest    |
    |                          |     port (p1)                 |
    |                          |  3. Attempt push to p1.buf    |
    |                          |     -> FULL                   |
    |                          |  4. timeout > 0, so:          |
    |                          |     deadline = tick + 100     |
    |                          |     push WaitEntry to p0.wq   |
    |                          |  5. Return SenderBlocked{A}   |
    |                          |  6. Dispatcher transitions    |
    |                          |     A: Running -> Waiting     |
    |                          |  7. Return r0=0 to A          |
    |  (now Waiting)           |                               |
    |                          |                               |
```

**Key**: The `WaitEntry` is pushed onto the **source port's** (p0)
wait queue, because blocked senders logically belong to the source.
The deadline is computed as `current_tick + timeout_ticks` (or
`u64::MAX` if `timeout_ticks == u32::MAX`).

### 7.2 Queuing Port Receive — Wakeup Flow

When Partition B later calls `RECV_QUEUING_MESSAGE` on the connected
destination port and the ring buffer has messages:

```
Partition B              Kernel (SVC handler)              Partition A
    |                          |                               |
    |--SYS_QPORT_RECV(p1)---->|                               |
    |  timeout=0, buf=[..]    |                               |
    |                          |  1. Validate port_id, owner,  |
    |                          |     direction, buf capacity   |
    |                          |  2. Pop front message from    |
    |                          |     p1.buf -> (size, data)    |
    |                          |  3. Copy data to B's buffer   |
    |                          |  4. Check connected source    |
    |                          |     port (p0) wait queue      |
    |                          |  5. Pop WaitEntry for A       |
    |                          |  6. Return Received{size,     |
    |                          |     wake_sender: Some(A)}     |
    |                          |  7. Dispatcher transitions    |
    |                          |     A: Waiting -> Ready       |
    |                          |  8. Write size to B's struct  |
    |                          |  9. Return r0=0 to B          |
    |  (B has data)            |                               |  (A now Ready)
```

**Cross-port wakeup**: The receive operation on the **destination**
port wakes a sender blocked on the **connected source** port. This
is the inverse: send targets the destination's buffer; receive frees
space and wakes the source's blocked senders.

### 7.3 Blackboard Display — Broadcast Wakeup

When a partition calls `DISPLAY_BLACKBOARD` and readers are blocked:

```
Config Partition          Kernel (SVC handler)          Worker A, Worker B
    |                          |                               |
    |--SYS_BB_DISPLAY(bb0)---->|                               |
    |  data=[mode=ACTIVE,..]   |                               |
    |                          |  1. Validate bb_id, size      |
    |                          |  2. Copy data to bb0.data     |
    |                          |  3. Set is_empty = false      |
    |                          |  4. Drain wait_queue:         |
    |                          |     pop A -> Ready            |
    |                          |     pop B -> Ready            |
    |                          |  5. Return r0=0               |
    |  (displayed)             |                          (A, B now Ready)
```

**Broadcast**: `display()` wakes ALL blocked readers, not just one.
Each reader will re-issue its `READ_BLACKBOARD` syscall when it next
runs, and will find the data available.

**Design rationale for broadcast wakeup**: Broadcast (wake-all) is the
intentional semantic for blackboards, not unicast (wake-one). A
blackboard is a shared state buffer — when new data is displayed, ALL
interested readers should see it, not just the first one. This matches
the ARINC 653 blackboard model: a blackboard is analogous to a notice
board where posted information is visible to every reader. Waking only
one reader would leave other readers blocked despite data being
available, which is incorrect for the blackboard use case. The
unicast alternative (wake-one) would only be appropriate for a
producer-consumer queue where each message is consumed by exactly one
receiver — that use case is served by queuing ports, not blackboards.
After `display()`, the data remains in the blackboard for subsequent
reads by other partitions, further confirming that broadcast is the
correct semantic: the data is not consumed by the first reader.

**Thundering herd analysis — broadcast is the correct semantic**:

Broadcasting to all waiters means that if `W` partitions are blocked
on a single blackboard, all `W` are transitioned to `Ready`
simultaneously. The reviewer raised a concern that this creates a
"thundering herd" where only the first scheduled partition
successfully reads the data, while others wake, run, and find the
blackboard empty again, wasting CPU cycles.

**This scenario does not apply to blackboards.** Unlike a queue (where
data is consumed by the first reader), `display()` writes data that
**persists** in the blackboard. The blackboard is NOT cleared by
`read()` — it remains non-empty after any number of reads. Therefore,
ALL woken readers will successfully read the data when they execute
their retry `SYS_BB_READ`. No reader will find the blackboard empty
after being woken by `display()` (unless an intervening `clear()`
occurs, which is an explicit application-level action, not a race).

The cost per woken partition is one additional SVC call (the retry
read, per the re-entry protocol in Section 7.5). With a static ARINC
653 schedule table, these retries are spread across separate time
slots — there is no contention or "herd" behavior because partitions
do not execute simultaneously.

**Comparison with wake-one alternative**: Waking only one reader
would be **incorrect** for blackboards. If three partitions are
waiting for a configuration update, waking only one would leave the
other two indefinitely blocked despite the data being available. They
would only unblock when their timeout expires (returning `Empty` —
misleading, since data IS present) or when a subsequent `display()`
occurs. This violates the fundamental blackboard contract: displayed
data is visible to all readers.

**When thundering herd IS a concern**: If a future `clear()` variant
were added that clears the blackboard and wakes readers (which this
design does NOT do — `clear()` leaves readers blocked, per Section
4.1), then a thundering herd would be a real issue. This design
explicitly avoids that pattern.

**Cost summary**: At most `W` additional SVC calls (default W=4),
each costing ~1 us, spread across separate schedule table time
slots. Negligible overhead.

This is the recommended "semaphore-style direct wakeup" pattern: the
`display()` method accepts `&mut PartitionTable<N>` and performs
transitions inline, avoiding the need to return a variable-length list
of partition IDs:

```rust
pub fn display<const N: usize>(
    &mut self,
    parts: &mut PartitionTable<N>,
    bb_id: usize,
    data: &[u8],
) -> Result<(), BbError> {
    let board = self.boards.get_mut(bb_id)
        .ok_or(BbError::InvalidBlackboard)?;
    if data.len() > M {
        return Err(BbError::SizeTooLarge);
    }
    board.data[..data.len()].copy_from_slice(data);
    board.current_size = data.len();
    board.is_empty = false;
    // Wake all blocked readers
    while let Some(entry) = board.wait_queue.pop_front() {
        if let Some(p) = parts.get_mut(entry.partition_id as usize) {
            let _ = p.transition(PartitionState::Ready);
        }
    }
    Ok(())
}
```

### 7.4 Blackboard Read — Blocking Flow

When a partition calls `READ_BLACKBOARD` with `timeout > 0` on an
empty (cleared) blackboard:

1. Validate `bb_id`, `buf_capacity`.
2. Check `board.is_empty == true`.
3. Since `timeout > 0`, compute deadline and push `WaitEntry` to
   the board's wait queue.
4. Return `BbReadOutcome::ReaderBlocked { blocked: caller }`.
5. Dispatcher transitions caller: `Running -> Waiting`.

When the blackboard is subsequently displayed, or the timeout expires,
the partition is transitioned back to `Ready`.

### 7.5 Partition Re-Entry After Wakeup — Retry Approach

When a blocked partition is woken (either by complementary operation or
timeout expiry), execution resumes at the instruction after the SVC.
The partition must re-issue the syscall to determine the actual outcome:

- **Woken by data**: Re-issue returns `0` (success) with data.
- **Woken by timeout**: Re-issue returns `1` (empty/full, non-blocking)
  because the condition hasn't changed.

**Design decision**: Use the retry approach (no deferred copy). This is
simpler and avoids storing raw user-space pointers in `WaitEntry`. The
`WaitEntry` structure stays simple:

```rust
pub struct WaitEntry {
    pub partition_id: u8,
    pub deadline: u64,
}
```

The partition-side wrapper should loop:

```rust
fn recv_queuing_blocking(port_id: u32, buf: &mut [u8], timeout: u32) -> Result<usize, u32> {
    loop {
        let mut args = QueuingRecvArgs { timeout_ticks: timeout, size: 0, data: ... };
        let ret = syscall(SYS_QPORT_RECV, port_id, buf.len() as u32, &args);
        if ret == 0 && args.size > 0 {
            return Ok(args.size as usize);  // Actually received data
        } else if ret == 1 {
            return Err(1);  // Timed out
        } else if ret >= 0xFFFF_FF00 {
            return Err(ret);  // Error
        }
        // ret == 0 but no data: was blocked and woken, retry
        // (On retry, use timeout=0 to avoid double-blocking)
    }
}
```

### 7.6 Atomicity and Concurrency Analysis

All queuing port and blackboard operations (send, receive, display,
read, clear) execute inside the SVC handler, which runs in Handler mode
at the highest configurable priority (SVCall priority = 0x00). On
single-core Cortex-M, this provides the following atomicity properties:

1. **SVC operations are non-preemptible by application code.** No
   partition code can execute while an SVC handler is running. This
   means a sequence like "check `is_empty` -> read `data` -> copy to
   caller buffer" in `BlackboardPool::read()` executes atomically with
   respect to all partition activity.

2. **SVC operations are non-preemptible by SysTick.** SysTick priority
   (0xFE) is lower than SVCall (0x00). If SysTick fires while an SVC
   is in progress, it pends until the SVC completes. This prevents the
   timeout sweep (Section 8) from modifying wait queues concurrently
   with an SVC operation that is also modifying wait queues.

3. **SysTick operations are non-preemptible by SVC.** SVC is triggered
   only by user-space `svc` instructions. Since no partition code runs
   during the SysTick handler, no SVC can fire during the timeout
   sweep.

#### 7.6.1 Interaction Matrix

| Operation A (context) | Operation B (context) | Shared State | Safe? | Why |
|------------------------|-----------------------|-------------|-------|-----|
| `send()` (SVC) | `receive()` (SVC) | Ring buffer, wait queues | Yes | Only one SVC active at a time. |
| `send()` (SVC) | Timeout sweep (SysTick) | Source port wait queue | Yes | SysTick pends until SVC completes. |
| `receive()` (SVC) | Timeout sweep (SysTick) | Source port wait queue (wakeup) | Yes | Same priority ordering. |
| `display()` (SVC) | `read()` (SVC) | Blackboard data, `is_empty`, wait queue | Yes | Both are SVC; mutual exclusion. |
| `display()` (SVC) | Timeout sweep (SysTick) | Blackboard wait queue | Yes | SVC completes and drains queue before SysTick runs. |
| `clear()` (SVC) | Timeout sweep (SysTick) | Blackboard `is_empty`, wait queue | Yes | SVC completes before SysTick. |
| `clear()` (SVC) | `display()` (SVC) | Blackboard data, `is_empty` | Yes | Both are SVC; cannot overlap. |
| Timeout sweep (SysTick) | PendSV (context switch) | Partition state | Yes | SysTick priority > PendSV priority. |

**Key invariant**: On single-core Cortex-M, the priority ordering
SVCall (0x00) > SysTick (0xFE) > PendSV (0xFF) guarantees that:
- No two SVC calls overlap.
- SysTick never preempts an SVC handler.
- SVC never fires during SysTick.
- PendSV never preempts either SVC or SysTick.

**No additional synchronization primitives** are required.

#### 7.6.2 Double-Wakeup Prevention

A critical correctness concern is the **double-wakeup scenario**: a
blocked partition could be woken by both a complementary operation
and a timeout expiry in the same tick.

**This cannot occur**, for two reasons:

1. **Entry removal**: When a complementary operation wakes a partition,
   it removes the `WaitEntry` from the wait queue. The timeout sweep
   will not find the removed entry.

2. **Priority ordering**: The SVC handler has higher priority than
   SysTick. If both are pending in the same tick:
   - SVC executes first, removes the `WaitEntry`, transitions the
     partition to `Ready`.
   - SysTick executes after SVC completes, finds the entry already
     removed, does nothing.

**Consequence**: No race conditions exist between SVC operations and
the timeout sweep. The interrupt priority configuration is the sole
synchronization mechanism.

#### 7.6.3 SysTick Modifying Wait Queue During SVC — Impossible

A concern might be: "What if SysTick fires mid-SVC and modifies a
wait queue that the SVC handler is iterating?" This is impossible
because SysTick (priority 0xFE) cannot preempt SVCall (priority 0x00).
On Cortex-M, a lower numerical priority value means higher urgency.
If SysTick becomes pending while SVCall is executing, the SysTick
handler is deferred until SVCall returns to Thread mode or to a lower-
priority handler. The SVC handler runs to completion before SysTick
can execute.

#### 7.6.4 SVC Firing During SysTick — Impossible

Another concern: "What if a partition issues an SVC while the SysTick
handler is running the timeout sweep?" SVC instructions execute only
in Thread mode (unprivileged partition code). The SysTick handler
runs in Handler mode; no partition code executes during it. Therefore,
no SVC can be triggered while SysTick is active.

The only way an SVC could theoretically fire during SysTick is if
handler-mode code executed an `svc` instruction, but:
- The kernel never issues `svc` from handler mode.
- Even if it did, SVCall would preempt SysTick (higher priority),
  but this code path does not exist.

#### 7.6.5 Boot-Time Priority Verification

The correctness of the entire concurrency model depends on the
interrupt priority configuration. The implementation MUST verify
priorities at boot:

```rust
fn verify_interrupt_priorities() {
    let svcall_pri = unsafe { core::ptr::read_volatile(0xE000_ED1F as *const u8) };
    let systick_pri = unsafe { core::ptr::read_volatile(0xE000_ED23 as *const u8) };
    let pendsv_pri = unsafe { core::ptr::read_volatile(0xE000_ED22 as *const u8) };
    assert!(svcall_pri < systick_pri,
        "SVCall must have higher priority (lower value) than SysTick");
    assert!(systick_pri < pendsv_pri || systick_pri == pendsv_pri,
        "SysTick must have equal or higher priority than PendSV");
}
```

This assertion runs once at startup and panics if the priorities are
misconfigured, preventing silent corruption of IPC state.

### 7.7 State Transition Summary

```
Running --(SVC, resource unavailable, timeout > 0)--> Waiting
Waiting --(complementary op wakes entry)--> Ready
Waiting --(timeout sweep expires entry)--> Ready
Ready --(scheduler assigns time slot)--> Running
```

Existing `PartitionState` enum (`Ready`, `Running`, `Waiting`) is
sufficient — no new states needed.

### 7.8 Priority Inversion Analysis

**Classical priority inversion** occurs in priority-based preemptive
schedulers when a high-priority task is blocked waiting for a resource
held by a low-priority task, and a medium-priority unrelated task
preempts the low-priority task, indefinitely delaying the high-priority
task. The standard mitigations are priority inheritance (temporarily
raise the holder's priority to the waiter's priority) and priority
ceiling (assign each resource a ceiling priority equal to the highest
priority of any task that may use it).

**This kernel does not use priority-based preemptive scheduling.**
Partitions execute in a deterministic static ARINC 653 schedule table:
each partition is assigned fixed time slots within a repeating major
frame. When a partition's time slot begins, it runs; when the slot
ends, the next partition in the table runs, regardless of any partition
"priority." There is no preemption based on task priority — only the
schedule table drives partition switches.

**Consequence**: Classical priority inversion **cannot occur** in this
scheduler model. Specifically:

1. **No medium-priority preemption**: A "medium-priority" partition
   cannot preempt a "low-priority" partition, because there is no
   priority-based preemption. Each partition runs only in its assigned
   time slot. A low-priority partition holding a resource (e.g., having
   data to send that would unblock a high-priority receiver) will
   execute in its next time slot regardless of what other partitions
   are doing.

2. **Bounded blocking duration**: A partition blocked on a queuing port
   or blackboard waits at most one major frame cycle (plus timeout).
   The complementary partition (sender or displayer) will execute in
   its assigned slot and may complete the operation. The worst case is
   deterministic and bounded by the schedule table period.

3. **No unbounded priority inversion**: Since no partition can be
   indefinitely starved by another (all partitions receive their time
   slots per the static schedule), the unbounded inversion scenario
   that motivates priority inheritance/ceiling protocols does not arise.

**Why priority inheritance is not applicable**: Priority inheritance
requires the kernel to temporarily elevate a partition's "priority" so
it executes sooner. In a static schedule table, there is no "priority"
to elevate — partitions do not have dynamic priorities, and the
schedule table cannot be modified at runtime. Implementing priority
inheritance would require switching to a priority-based scheduler,
which contradicts the ARINC 653 temporal isolation model.

**Wait queue ordering**: Wait queues are strictly FIFO (Section R3 in
Section 13). In a static schedule, FIFO ordering is natural and fair:
the first partition to block is the first to be woken. Priority-ordered
wait queues would only be beneficial in a priority-based scheduler.

**When this analysis changes**: If the kernel is extended with a
priority-based scheduler (either replacing or supplementing the static
schedule table), priority inversion becomes a real concern and the
following mitigations must be added:

- **Priority Inheritance Protocol (PIP)**: When a high-priority
  partition blocks on a queuing port whose complementary partition is
  lower-priority, temporarily elevate the lower-priority partition's
  scheduling priority to the waiter's priority. Restore the original
  priority when the operation completes.
- **Immediate Priority Ceiling Protocol (IPCP)**: Assign each IPC
  resource a ceiling priority (the highest priority of any partition
  that may use it). When a partition accesses the resource, elevate
  its effective priority to the ceiling. This prevents chained blocking.
- **Wait queue re-ordering**: Change wait queues from FIFO to
  priority-ordered so higher-priority partitions are woken first.

These changes are deferred to future work (see Section 13, R3).

---

## 8. Timeout Expiry Integration with Scheduler Tick

### 8.1 Problem

Existing blocking primitives (semaphores, mutexes, message queues) block
indefinitely — there is no timeout mechanism. Queuing ports and
blackboards require deadline-based timeout: a blocked partition must be
woken when its deadline passes, even if no complementary operation occurs.

### 8.2 Design: Per-Tick Timeout Sweep

On every SysTick interrupt, after advancing the schedule table and
incrementing the tick counter, the kernel scans all wait queues in
queuing ports and blackboards for expired entries.

**Execution order in SysTick handler**:

```
1. tick.increment()                    // Advance monotonic counter
2. schedule.advance_tick()             // Advance schedule table, maybe trigger PendSV
3. queuing_ports.expire_timeouts()     // Scan queuing port wait queues
4. blackboards.expire_timeouts()       // Scan blackboard wait queues
5. (If schedule advanced) trigger PendSV for context switch
```

The timeout sweep runs AFTER the tick increment so that a partition
blocked with `timeout_ticks = 1` is woken after exactly 1 tick.

### 8.3 Sweep Implementation: Drain-and-Re-enqueue

Each pool implements `expire_timeouts`:

```rust
impl<const S: usize, const D: usize, const M: usize, const W: usize>
    QueuingPortPool<S, D, M, W>
{
    pub fn expire_timeouts<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        current_tick: u64,
    ) {
        for port in self.ports.iter_mut() {
            let len = port.wait_queue.len();
            for _ in 0..len {
                if let Some(entry) = port.wait_queue.pop_front() {
                    if entry.deadline != u64::MAX && entry.deadline <= current_tick {
                        // Expired: transition partition Waiting -> Ready
                        if let Some(p) = parts.get_mut(entry.partition_id as usize) {
                            let _ = p.transition(PartitionState::Ready);
                        }
                        // Entry is consumed (not re-enqueued)
                    } else {
                        // Not expired: re-enqueue at back
                        let _ = port.wait_queue.push_back(entry);
                    }
                }
            }
        }
    }
}
```

**Algorithm**: For each port, drain all entries from the front of the
wait queue. Expired entries are dropped (partition transitioned to
Ready). Non-expired entries are pushed back to the tail. This preserves
FIFO ordering among non-expired entries.

**Why drain-and-re-enqueue**: `heapless::Deque` does not support
`retain()` or indexed removal. The drain approach is the simplest
correct implementation for a bounded deque without those features.

**`u64::MAX` check**: Entries with `deadline == u64::MAX` represent
infinite waits (the partition specified `timeout_ticks == u32::MAX`).
These are never expired by the sweep — they can only be woken by a
complementary operation.

The blackboard pool's `expire_timeouts` is identical in structure,
iterating `boards` instead of `ports`:

```rust
impl<const S: usize, const M: usize, const W: usize>
    BlackboardPool<S, M, W>
{
    pub fn expire_timeouts<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        current_tick: u64,
    ) {
        for board in self.boards.iter_mut() {
            let len = board.wait_queue.len();
            for _ in 0..len {
                if let Some(entry) = board.wait_queue.pop_front() {
                    if entry.deadline != u64::MAX && entry.deadline <= current_tick {
                        if let Some(p) = parts.get_mut(entry.partition_id as usize) {
                            let _ = p.transition(PartitionState::Ready);
                        }
                    } else {
                        let _ = board.wait_queue.push_back(entry);
                    }
                }
            }
        }
    }
}
```

### 8.4 Kernel-Level Aggregation

The `Kernel` struct provides a single method that sweeps all pools:

```rust
impl Kernel<...> {
    pub fn expire_all_timeouts(&mut self) {
        let tick = self.tick.get();
        self.queuing_ports.expire_timeouts(&mut self.partitions, tick);
        self.blackboards.expire_timeouts(&mut self.partitions, tick);
        // Sampling ports have no wait queues — no sweep needed.
    }
}
```

### 8.5 SysTick Handler Integration

The existing `on_systick` function in `tick.rs` advances the schedule
and triggers PendSV. A new `on_systick_with_timeouts` function (or an
extension of the existing one) adds the timeout sweep:

```rust
pub fn on_systick_with_timeouts<...>(kernel: &mut Kernel<...>) -> Option<u8> {
    // Step 1: Advance tick counter (already done inside advance_schedule_tick)
    let next = kernel.advance_schedule_tick();

    // Step 2: Sweep all wait queues for expired timeouts
    kernel.expire_all_timeouts();

    // Step 3: If a partition switch is pending, trigger PendSV
    if next.is_some() {
        #[cfg(not(test))]
        unsafe {
            core::ptr::write_volatile(SCB_ICSR as *mut u32, ICSR_PENDSVSET);
        }
    }
    next
}
```

**Why a new function rather than modifying `on_systick`**: The existing
`on_systick` operates on `KernelState<P, S>` which does not include
the IPC pools. The new function operates on the full `Kernel` struct
which includes `queuing_ports` and `blackboards`. This avoids modifying
the `KernelState` type, which is used independently in some test
configurations.

**Alternative**: If the `Kernel` struct subsumes `KernelState` (which
is the plan per Section 11), then `on_systick` can be extended directly
as a method on `Kernel`.

### 8.6 Worst-Case Sweep Cost

| Resource | Ports | Wait-Queue Depth | Total Entries |
|----------|-------|-----------------|---------------|
| Queuing ports | 8 | 4 | 32 |
| Blackboards | 4 | 4 | 16 |
| **Total** | | | **48** |

Each entry check: one comparison + conditional pop/push = ~10 cycles.
Total: ~480 cycles per tick.

At 12 MHz with 10ms tick (120,000 cycles per tick), the sweep consumes
**0.4%** of the tick budget. Negligible.

**Scaling**: If future configurations increase pool sizes (e.g., 16
queuing ports with W=8), worst-case entries would be 160, costing
~1,600 cycles (~1.3% of tick budget). Still acceptable. The sweep
becomes a concern only at hundreds of ports, which is unlikely on a
single-core Cortex-M.

### 8.7 Race Prevention: Complementary Wakeup vs. Timeout Sweep

**Invariant**: When a complementary operation wakes a partition, it
removes the `WaitEntry` from the wait queue. The timeout sweep will
then never find that entry.

Specifically:
- `QueuingPortPool::receive()` pops from the connected source port's
  wait queue when it wakes a blocked sender.
- `QueuingPortPool::send()` pops from the destination port's wait
  queue when it wakes a blocked receiver.
- `BlackboardPool::display()` drains the entire wait queue.

There is no concurrent access risk because the SysTick handler and
SVC handler do not execute simultaneously on single-core Cortex-M.
See Section 7.6 for the full proof.

**Scenario: Complementary wakeup and timeout in same tick**:

Consider Partition A blocked on a queuing port receive with
`deadline = tick + 5`. At tick T+5:

Case 1 — Timeout fires first (no complementary op in this tick):
1. SysTick fires, increments tick to T+5, runs the timeout sweep.
2. Sweep finds A's entry with `deadline <= T+5`, removes it,
   transitions A to `Ready`.
3. A re-issues `SYS_QPORT_RECV`, either gets data or gets `Empty`.

Case 2 — Complementary op fires first (B sends before SysTick):
1. B's `SYS_QPORT_SEND` executes in SVC context, delivers message,
   wakes A by removing A's entry and transitioning A to `Ready`.
2. SysTick fires later, sweep finds nothing to expire for A.
3. A re-issues recv and gets the data.

In both orderings, exactly one wakeup occurs.

### 8.8 Deadline Computation

```rust
let deadline = if timeout_ticks == u32::MAX {
    u64::MAX  // Infinite wait
} else {
    current_tick + timeout_ticks as u64
};
```

The `u64` tick counter wraps after ~5.8 billion years at 10ms/tick,
so overflow is not a practical concern.

### 8.9 Interaction with Existing `on_systick`

The existing `on_systick` in `tick.rs` has two variants:

1. `on_systick(state: &mut KernelState<P, S>)` — basic schedule advance.
2. `on_systick_mpu(state: &mut KernelState<P, S>)` — schedule advance
   with MPU reconfiguration.

The new `on_systick_with_timeouts` replaces these when the kernel uses
queuing ports or blackboards. For backward compatibility, the existing
functions remain available for configurations that do not use timeout-
based IPC.

**Migration path**: Once the `Kernel` struct (Section 11) is the
standard entry point, `on_systick_with_timeouts` becomes the default
SysTick handler and the legacy variants are retained only for minimal
test configurations.

---

## 9. Port Connection Routing for Inter-Partition Communication

### 9.1 Routing Model

Queuing ports use the same one-to-one static routing model as sampling
ports:

- Each **Source** port connects to exactly one **Destination** port.
- Connections are configured at kernel init time via the
  `ConnectionTable`.
- At runtime, `SEND_QUEUING_MESSAGE` on a source port pushes messages
  into the connected destination port's ring buffer.
- `RECEIVE_QUEUING_MESSAGE` on a destination port pops messages from
  its own ring buffer.

Blackboards do NOT use port connection routing. They are shared resources
accessible by any partition by ID.

### 9.2 Connection Table Data Structures

The `ConnectionTable` and `PortKind` enum support both sampling and
queuing connections:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortKind {
    Sampling,
    Queuing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PortConnection {
    pub source_kind: PortKind,
    pub source_id: u8,
    pub dest_kind: PortKind,
    pub dest_id: u8,
}

/// Static connection table with capacity for `C` connections.
pub struct ConnectionTable<const C: usize> {
    connections: heapless::Vec<PortConnection, C>,
}

impl<const C: usize> ConnectionTable<C> {
    pub fn new() -> Self {
        Self { connections: heapless::Vec::new() }
    }

    pub fn add(&mut self, conn: PortConnection) -> Result<(), ()> {
        self.connections.push(conn).map_err(|_| ())
    }
}
```

**Memory**: Each `PortConnection` is 4 bytes. With `C=16`: 64 bytes +
Vec overhead. Negligible.

### 9.3 Configuration Process

Connections are configured during kernel initialization, before any
partition runs. The process has four steps:

```rust
// 1. Create pools (empty)
let mut sampling_ports = SamplingPortPool::<8, 64>::new();
let mut queuing_ports = QueuingPortPool::<8, 8, 64, 4>::new();
let mut connections = ConnectionTable::<16>::new();

// 2. Add ports with their direction and owning partition
let qp_src = queuing_ports.add(QueuingPort::new(PortDirection::Source, /*partition_id=*/ 0));
let qp_dst = queuing_ports.add(QueuingPort::new(PortDirection::Destination, /*partition_id=*/ 1));

// 3. Define connections
connections.add(PortConnection {
    source_kind: PortKind::Queuing, source_id: 0,
    dest_kind: PortKind::Queuing, dest_id: 1,
}).unwrap();

// 4. Resolve connections (sets connected_port on source ports)
connections.resolve(&mut sampling_ports, &mut queuing_ports).unwrap();
```

**Step 4 (resolve)** is the critical step: it iterates all declared
connections and writes the `connected_port` field on each source port.
After resolution, the connection table is no longer consulted at
runtime — all routing is via the `connected_port` field on each port.

### 9.4 Connection Resolution Implementation

```rust
impl<const C: usize> ConnectionTable<C> {
    pub fn resolve<
        const SS: usize, const SM: usize,
        const QS: usize, const QD: usize, const QM: usize, const QW: usize,
    >(
        &self,
        sampling: &mut SamplingPortPool<SS, SM>,
        queuing: &mut QueuingPortPool<QS, QD, QM, QW>,
    ) -> Result<(), ConnectionError> {
        for conn in self.connections.iter() {
            if conn.source_kind != conn.dest_kind {
                return Err(ConnectionError::KindMismatch);
            }
            match conn.source_kind {
                PortKind::Sampling => {
                    let src = sampling.get_mut(conn.source_id as usize)
                        .ok_or(ConnectionError::InvalidSourcePort)?;
                    if src.direction != PortDirection::Source {
                        return Err(ConnectionError::WrongSourceDirection);
                    }
                    if src.connected_port.is_some() {
                        return Err(ConnectionError::DuplicateConnection);
                    }
                    let dst = sampling.get(conn.dest_id as usize)
                        .ok_or(ConnectionError::InvalidDestPort)?;
                    if dst.direction != PortDirection::Destination {
                        return Err(ConnectionError::WrongDestDirection);
                    }
                    src.connected_port = Some(conn.dest_id);
                }
                PortKind::Queuing => {
                    let src = queuing.get_mut(conn.source_id as usize)
                        .ok_or(ConnectionError::InvalidSourcePort)?;
                    if src.direction != PortDirection::Source {
                        return Err(ConnectionError::WrongSourceDirection);
                    }
                    if src.connected_port.is_some() {
                        return Err(ConnectionError::DuplicateConnection);
                    }
                    let dst = queuing.get(conn.dest_id as usize)
                        .ok_or(ConnectionError::InvalidDestPort)?;
                    if dst.direction != PortDirection::Destination {
                        return Err(ConnectionError::WrongDestDirection);
                    }
                    src.connected_port = Some(conn.dest_id);
                }
            }
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ConnectionError {
    InvalidSourcePort,
    InvalidDestPort,
    WrongSourceDirection,
    WrongDestDirection,
    DuplicateConnection,
    KindMismatch,
}
```

### 9.5 Validation During `resolve()`

The `resolve()` function validates:

1. Source port exists and has `direction == Source`.
2. Destination port exists and has `direction == Destination`.
3. No source port has more than one connection (`DuplicateConnection`).
4. Source and destination are the same `PortKind` (no sampling-to-queuing
   cross-connections — `KindMismatch`).

If any validation fails, `resolve()` returns an error and no connections
are partially applied (the function returns on first error).

### 9.6 Data Flow: Queuing Port Send with Routing

```
Partition A (owns source p0)     Kernel                Partition B (owns dest p1)
         |                          |                          |
         |--SEND(p0, data)--------->|                          |
         |                          |  resolve connected: p1   |
         |                          |  push data to p1.buf     |
         |                          |  (if p1 has blocked      |
         |                          |   receiver, wake it)     |
         |                          |                          |
         |                          |         RECV(p1, buf)    |
         |                          |<-------------------------|
         |                          |  pop from p1.buf         |
         |                          |  (if p0 has blocked      |
         |                          |   sender, wake it)       |
         |                          |  return data to B        |
```

**Key**: The sender addresses its own source port (`p0`). The kernel
resolves `p0.connected_port` to find destination `p1` and pushes the
message into `p1.buf`. The receiver addresses its own destination port
(`p1`) and pops from `p1.buf`.

### 9.7 Bidirectional Communication

For bidirectional communication (e.g., command/response), create two
source-destination port pairs:

```
P0 (source, owned by A) --connected--> P1 (dest, owned by B)   // A -> B
P2 (source, owned by B) --connected--> P3 (dest, owned by A)   // B -> A
```

The connection table has two entries. Each partition sends on its own
source port and receives on its own destination port.

### 9.8 Runtime Enforcement

The kernel enforces ownership and direction on every syscall. The
caller's identity is determined by `self.active_partition()` (set by
the scheduler), never by a register argument from user space.

On every `SEND_QUEUING_MESSAGE`:

1. **Ownership check**: `port.partition_id == active_partition()`.
2. **Direction check**: `port.direction == Source`.
3. Resolve `connected_port`; return `ERR_NO_CONNECTION` if `None`.
4. Validate message size and pointer.
5. Push message to destination's ring buffer (or block caller).

On every `RECEIVE_QUEUING_MESSAGE`:

1. **Ownership check**: `port.partition_id == active_partition()`.
2. **Direction check**: `port.direction == Destination`.
3. Validate buffer capacity and pointer.
4. Pop from this port's ring buffer (or block caller).
5. If a blocked sender exists on the connected source's wait queue,
   wake it.

### 9.9 Unconnected Port Behavior

If a source port's `connected_port` is `None` (no connection was
configured or `resolve()` was not called), `send()` returns
`QPortError::NoConnection`. The partition receives `ERR_NO_CONNECTION`
in `r0`. This is a configuration error, not a runtime transient.

Destination ports do not use `connected_port` for receiving — they
always read from their own `buf`. However, the `receive()` operation
does use the peer source port's `connected_port` (in reverse) to
locate the source port's wait queue for waking blocked senders. If the
source port is not known (no connection), `wake_sender` is `None`.

---

## 10. SVC Dispatch Integration

### 10.1 Overview

The `Kernel::dispatch()` method in `svc.rs` routes syscalls based on
`SyscallId`. New match arms are added for `SYS_QPORT_SEND` (15),
`SYS_QPORT_RECV` (16), `SYS_QPORT_STATUS` (17), `SYS_BB_DISPLAY` (18),
`SYS_BB_READ` (19), and `SYS_BB_CLEAR` (20).

Each arm follows the pattern established by existing syscalls:

1. Extract caller from `self.active_partition()`.
2. Get the caller's `PartitionControlBlock` for pointer validation.
3. Validate the pointer (if applicable) — return `ERR_BAD_PTR` on failure.
4. Read arguments from the pointed-to struct (timeout, data).
5. Call the pool method.
6. Apply the outcome (state transitions, write-back results).

### 10.2 Queuing Port Dispatch

```rust
Some(SyscallId::QPortSend) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let msg_size = frame.r2 as usize;
    let total_size = msg_size + 4;  // + timeout_ticks prefix

    if !validate_user_ptr(pcb, frame.r3, total_size as u32) {
        return ERR_BAD_PTR;
    }

    let args_ptr = frame.r3 as *const QueuingSendArgs;
    let timeout = unsafe { (*args_ptr).timeout_ticks };
    let data = unsafe {
        core::slice::from_raw_parts((*args_ptr).data.as_ptr(), msg_size)
    };
    let tick = self.tick.get();

    match self.queuing_ports.send(
        frame.r1 as usize, caller, data, msg_size, timeout as u64, tick,
    ) {
        Ok(outcome) => apply_qport_send_outcome(&mut self.partitions, outcome),
        Err(e) => u32::from(e),
    }
}

Some(SyscallId::QPortRecv) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let buf_capacity = frame.r2 as usize;
    let total_size = buf_capacity + 8;  // + timeout_ticks + size prefix

    if !validate_user_ptr(pcb, frame.r3, total_size as u32) {
        return ERR_BAD_PTR;
    }

    let args_ptr = frame.r3 as *mut QueuingRecvArgs;
    let timeout = unsafe { (*args_ptr).timeout_ticks };
    let tick = self.tick.get();

    match self.queuing_ports.receive(
        frame.r1 as usize, caller,
        unsafe { &mut (*args_ptr).data[..buf_capacity] },
        buf_capacity, timeout as u64, tick,
    ) {
        Ok(outcome) => apply_qport_recv_outcome(
            &mut self.partitions, outcome, args_ptr,
        ),
        Err(e) => u32::from(e),
    }
}

Some(SyscallId::QPortStatus) => {
    match self.queuing_ports.status(frame.r1 as usize) {
        Ok(s) => s.current_depth as u32,
        Err(e) => u32::from(e),
    }
}
```

### 10.3 Outcome Applicators

Following the pattern from `apply_send_outcome` in `svc.rs`:

```rust
fn apply_qport_send_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: QPortSendOutcome,
) -> u32 {
    match outcome {
        QPortSendOutcome::Delivered { wake_receiver: Some(pid) } => {
            let _ = try_transition(partitions, pid, PartitionState::Ready);
            0
        }
        QPortSendOutcome::Delivered { wake_receiver: None } => 0,
        QPortSendOutcome::SenderBlocked { blocked } => {
            let _ = try_transition(partitions, blocked, PartitionState::Waiting);
            0  // "In progress" — caller is now Waiting
        }
        QPortSendOutcome::Full => 1,  // Non-blocking failure
    }
}

fn apply_qport_recv_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: QPortRecvOutcome,
    recv_args_ptr: *mut QueuingRecvArgs,
) -> u32 {
    match outcome {
        QPortRecvOutcome::Received { size, wake_sender: Some(pid) } => {
            let _ = try_transition(partitions, pid, PartitionState::Ready);
            unsafe { (*recv_args_ptr).size = size as u32 };
            0
        }
        QPortRecvOutcome::Received { size, wake_sender: None } => {
            unsafe { (*recv_args_ptr).size = size as u32 };
            0
        }
        QPortRecvOutcome::ReceiverBlocked { blocked } => {
            let _ = try_transition(partitions, blocked, PartitionState::Waiting);
            0
        }
        QPortRecvOutcome::Empty => 1,
    }
}
```

### 10.4 Blackboard Dispatch

```rust
Some(SyscallId::BbDisplay) => {
    let pcb = self.partitions.get(self.active_partition()).unwrap();
    let size = frame.r2 as usize;

    if !validate_user_ptr(pcb, frame.r3, frame.r2) {
        return ERR_BAD_PTR;
    }

    let data = unsafe {
        core::slice::from_raw_parts(frame.r3 as *const u8, size)
    };
    match self.blackboards.display(
        &mut self.partitions, frame.r1 as usize, data,
    ) {
        Ok(()) => 0,
        Err(e) => u32::from(e),
    }
}

Some(SyscallId::BbRead) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let buf_capacity = frame.r2 as usize;
    let total_size = buf_capacity + 8;

    if !validate_user_ptr(pcb, frame.r3, total_size as u32) {
        return ERR_BAD_PTR;
    }

    let args_ptr = frame.r3 as *mut BlackboardReadArgs;
    let timeout = unsafe { (*args_ptr).timeout_ticks };
    let tick = self.tick.get();
    match self.blackboards.read(
        frame.r1 as usize, caller as u8,
        unsafe { &mut (*args_ptr).data[..buf_capacity] },
        buf_capacity, timeout as u64, tick,
    ) {
        Ok(BbReadOutcome::Read { size }) => {
            unsafe { (*args_ptr).size = size as u32 };
            0
        }
        Ok(BbReadOutcome::ReaderBlocked { blocked }) => {
            let _ = try_transition(&mut self.partitions, blocked, PartitionState::Waiting);
            0
        }
        Ok(BbReadOutcome::Empty) => 1,
        Err(e) => u32::from(e),
    }
}

Some(SyscallId::BbClear) => {
    match self.blackboards.clear(frame.r1 as usize) {
        Ok(()) => 0,
        Err(e) => u32::from(e),
    }
}
```

### 10.5 Borrow Checker Considerations

The `display()` method takes `&mut PartitionTable<N>` to perform
inline wakeups. This requires that `self.blackboards` and
`self.partitions` are borrowed as disjoint fields. In `dispatch()`,
which holds `&mut self`, Rust's borrow checker allows this because
the two fields are structurally disjoint within the `Kernel` struct.

The pattern is the same as used by `apply_send_outcome()` for message
queues: the outcome applicator takes `&mut self.partitions` separately
from the pool reference. For `BbDisplay`, the `display()` method itself
handles the wakeup, so the dispatch code passes `&mut self.partitions`
directly to `self.blackboards.display()`.

If the borrow checker rejects this (which can happen if both borrows
are taken from `self` in the same expression), the solution is to
destructure `self` into separate field borrows:

```rust
let Kernel { blackboards, partitions, .. } = self;
blackboards.display(partitions, frame.r1 as usize, data)
```

This is the same pattern already used in the codebase for similar
situations.

---

## 11. Extended Kernel Struct

### 11.1 Full Definition

```rust
pub struct Kernel<
    const N: usize,    // Max partitions
    const S: usize,    // Max semaphores
    const SW: usize,   // Semaphore wait-queue depth
    const MS: usize,   // Max mutexes
    const MW: usize,   // Mutex wait-queue depth
    const QS: usize,   // Max message queues
    const QD: usize,   // Queue depth (messages per queue)
    const QM: usize,   // Message queue message size (bytes)
    const QW: usize,   // Message queue wait-queue depth
    const SPS: usize,  // Max sampling ports
    const SPM: usize,  // Sampling port max message size
    // New for queuing ports:
    const QPS: usize,  // Max queuing ports
    const QPD: usize,  // Queuing port depth (messages per port)
    const QPM: usize,  // Queuing port max message size
    const QPW: usize,  // Queuing port wait-queue depth
    // New for blackboards:
    const BBS: usize,  // Max blackboards
    const BBM: usize,  // Blackboard max message size
    const BBW: usize,  // Blackboard wait-queue depth
    // Connection table:
    const CC: usize,   // Max port connections
> {
    pub partitions: PartitionTable<N>,
    pub semaphores: SemaphorePool<S, SW>,
    pub mutexes: MutexPool<MS, MW>,
    pub messages: MessagePool<QS, QD, QM, QW>,
    pub tick: TickCounter,
    pub sampling: SamplingPortPool<SPS, SPM>,
    pub queuing_ports: QueuingPortPool<QPS, QPD, QPM, QPW>,
    pub blackboards: BlackboardPool<BBS, BBM, BBW>,
    pub connections: ConnectionTable<CC>,
}
```

### 11.2 New Methods on `Kernel`

```rust
impl Kernel<...> {
    /// Returns the index of the currently active (Running) partition.
    pub fn active_partition(&self) -> usize {
        // Delegates to the scheduler/partition table.
        self.partitions.active()
    }

    /// Expire all timeout-based wait entries across queuing ports
    /// and blackboards. Called from the SysTick handler.
    pub fn expire_all_timeouts(&mut self) {
        let tick = self.tick.get();
        self.queuing_ports.expire_timeouts(&mut self.partitions, tick);
        self.blackboards.expire_timeouts(&mut self.partitions, tick);
    }

    /// Full dispatch method. See Section 10 for the match arms.
    pub fn dispatch(&mut self, frame: &mut ExceptionFrame) -> u32 {
        match SyscallId::from_u32(frame.r0) {
            // ... existing arms ...
            // ... new arms from Section 10 ...
            None => ERR_UNKNOWN,
        }
    }
}
```

### 11.3 Demo Configuration Type Alias

```rust
/// Standard kernel for Demo 2 (command/response with queuing ports).
type Demo2Kernel = Kernel<
    4,   // N: partitions
    4,   // S: semaphores
    4,   // SW: semaphore wait-queue
    4,   // MS: mutexes
    4,   // MW: mutex wait-queue
    4,   // QS: message queues
    4,   // QD: message queue depth
    4,   // QM: message queue msg size
    4,   // QW: message queue wait-queue
    8,   // SPS: sampling ports
    64,  // SPM: sampling port msg size
    8,   // QPS: queuing ports
    8,   // QPD: queuing port depth
    64,  // QPM: queuing port msg size
    4,   // QPW: queuing port wait-queue
    4,   // BBS: blackboards
    64,  // BBM: blackboard msg size
    4,   // BBW: blackboard wait-queue
    16,  // CC: connections
>;
```

### 11.4 Const-Generic Mitigation

20 const-generic parameters is verbose but unavoidable in
`no_std`/`no_alloc`. Mitigations:

1. **Type aliases**: Define `DemoKernel`, `TestKernel`, etc. for common
   configurations.
2. **Documentation**: Comment each parameter at the alias definition.
3. **Defaults**: The suggested defaults above work for all demos. Only
   override when a specific demo needs different bounds.

### 11.5 New Source File Layout

| File | Purpose |
|------|---------|
| `kernel/src/ipc_common.rs` | `WaitEntry`, `PortDirection` — foundational leaf types shared by all IPC modules, with **no dependencies** on any other IPC module |
| `kernel/src/queuing.rs` | `QueuingPort`, `QueuingPortPool`, `QPortError`, `QPortSendOutcome`, `QPortRecvOutcome` (imports `WaitEntry`, `PortDirection` from `ipc_common`) |
| `kernel/src/blackboard.rs` | `Blackboard`, `BlackboardPool`, `BbError`, `BbReadOutcome` (imports `WaitEntry` from `ipc_common`) |
| `kernel/src/routing.rs` | `PortConnection`, `PortKind`, `ConnectionTable`, `ConnectionError` (imports `PortDirection` from `ipc_common`; depends on pool types for `resolve()`) |
| `kernel/src/validate.rs` | `validate_user_ptr`, error code constants (`ERR_*`), `From` impls |

**Dependency graph** (arrows mean "imports from"):

```
ipc_common.rs          (leaf: no IPC imports)
    ^      ^
    |      |
queuing.rs  blackboard.rs    validate.rs   (mid-level: import ipc_common only)
    ^           ^
    |           |
routing.rs                   (top-level: imports pool types for resolve())
```

This layering avoids circular dependencies. `WaitEntry` was previously
suggested for `routing.rs`, but that module depends on pool types
(`SamplingPortPool`, `QueuingPortPool`) for its `resolve()` method,
which would create a circular dependency if pool modules also imported
`WaitEntry` from it.

Each file follows the same structure as existing modules (`message.rs`,
`semaphore.rs`, `mutex.rs`):
1. Error enum
2. Outcome enums (if applicable)
3. Control block struct
4. Pool struct with `new()`, `add()`, `get()`, operation methods
5. `#[cfg(test)] mod tests` with unit tests

Add to `lib.rs`:
```rust
pub mod blackboard;
pub mod ipc_common;
pub mod queuing;
pub mod routing;
pub mod validate;
```

---

## 12. Unit Testing Strategy

### 12.1 Queuing Port Tests

All tests go in `queuing.rs` under `#[cfg(test)] mod tests`.

**Basic operations**:
- `send_and_recv_basic`: Send one message, receive it, verify data and size.
- `fifo_ordering`: Send multiple messages, receive in order.
- `variable_message_sizes`: Send messages of different sizes (1 byte,
  M/2 bytes, M bytes), verify each is received with correct size.
- `send_to_full_queue_blocks`: Fill the destination buffer to D, send
  again with timeout > 0, verify `SenderBlocked`.
- `send_to_full_queue_nonblocking`: Same but timeout=0, verify `Full`.
- `recv_from_empty_queue_blocks`: Receive with timeout > 0 on empty,
  verify `ReceiverBlocked`.
- `recv_from_empty_queue_nonblocking`: Same but timeout=0, verify `Empty`.

**Wakeup**:
- `recv_wakes_blocked_sender`: Block sender on full queue, then recv
  frees space, verify `wake_sender: Some(pid)`.
- `send_wakes_blocked_receiver`: Block receiver on empty queue, then
  send delivers, verify `wake_receiver: Some(pid)`.

**Direction enforcement**:
- `send_on_destination_port_rejected`: Verify `WrongDirection`.
- `recv_on_source_port_rejected`: Verify `WrongDirection`.

**Ownership enforcement**:
- `send_by_wrong_partition_rejected`: Verify `NotOwner`.
- `recv_by_wrong_partition_rejected`: Verify `NotOwner`.

**Connection routing**:
- `send_without_connection_rejected`: Verify `NoConnection`.
- `send_routed_to_destination`: Connect ports, send on source, recv
  on destination, verify data arrives.

**Buffer validation**:
- `recv_buf_too_small`: Message is 10 bytes, recv buffer is 5 bytes,
  verify `BufTooSmall`.
- `send_message_too_large`: Message is M+1 bytes, verify `SizeTooLarge`.

**Timeout**:
- `wait_entry_deadline_computation`: Verify deadline = current_tick +
  timeout_ticks, and u32::MAX timeout maps to u64::MAX deadline.
- `expire_timeouts_wakes_expired`: Add entries with past deadlines,
  call `expire_timeouts`, verify partitions transitioned to Ready.
- `expire_timeouts_preserves_unexpired`: Mix expired and unexpired
  entries, verify only expired are removed.
- `infinite_timeout_not_expired`: Entry with `u64::MAX` deadline
  survives the sweep.

**Capacity**:
- `wait_queue_full`: Fill wait queue to W, try to block another,
  verify `WaitQueueFull`.
- `pool_full`: Fill pool to S ports, try to add another, verify error.
- `invalid_port_id`: Verify `InvalidPort` for out-of-range IDs.

**Status**:
- `status_returns_correct_depth`: Send N messages, verify
  `current_depth == N`.

### 12.2 Blackboard Tests

All tests in `blackboard.rs` under `#[cfg(test)] mod tests`.

**Basic operations**:
- `display_and_read`: Display data, read it back, verify contents.
- `display_overwrites_previous`: Display twice, read, verify latest.
- `read_empty_blocks`: Read with timeout > 0 on empty board, verify
  `ReaderBlocked`.
- `read_empty_nonblocking`: Read with timeout=0, verify `Empty`.
- `clear_makes_board_empty`: Display, clear, read with timeout=0,
  verify `Empty`.
- `clear_then_display_unblocks`: Display, clear, block reader, display
  again, verify reader woken.

**Broadcast wakeup**:
- `display_wakes_all_readers`: Block 3 readers, display, verify all 3
  transitioned to Ready.
- `display_wakes_no_readers_when_none_blocked`: Display with empty
  wait queue, verify no error.

**Timeout**:
- `expire_timeouts_wakes_expired_readers`: Block reader with short
  deadline, expire, verify Ready.
- `clear_does_not_wake_readers`: Block reader on empty board, display
  to unblock, re-block on empty, clear the board, verify reader
  remains in `Waiting` state and wait queue entry is preserved.
- `clear_with_multiple_blocked_readers`: Block 3 readers, clear,
  verify all 3 remain `Waiting`. Then display, verify all 3 woken.

**Buffer validation**:
- `read_buf_too_small`: Display 10 bytes, read with 5-byte buffer,
  verify `BufTooSmall`.
- `display_too_large`: Display M+1 bytes, verify `SizeTooLarge`.

**Edge cases**:
- `display_zero_length_message`: Valid — sets `is_empty = false` with
  `current_size = 0`. Read returns 0-length data immediately.
- `invalid_blackboard_id`: Verify `InvalidBlackboard`.

### 12.3 Connection Table Tests

All tests in `routing.rs` under `#[cfg(test)] mod tests`.

- `resolve_valid_queuing_connection`: Connect source to dest, verify
  `connected_port` is set.
- `resolve_valid_sampling_connection`: Same for sampling ports.
- `resolve_kind_mismatch`: Sampling source to queuing dest, verify
  `KindMismatch`.
- `resolve_wrong_source_direction`: Destination port as source, verify
  `WrongSourceDirection`.
- `resolve_wrong_dest_direction`: Source port as dest, verify
  `WrongDestDirection`.
- `resolve_duplicate_connection`: Connect same source twice, verify
  `DuplicateConnection`.
- `resolve_invalid_source_id`: Out-of-range source ID, verify
  `InvalidSourcePort`.
- `resolve_invalid_dest_id`: Out-of-range dest ID, verify
  `InvalidDestPort`.

### 12.4 Integration Tests (SVC-Level)

These test the full dispatch path in `svc.rs`, operating on a
`Kernel` instance with mock `ExceptionFrame` values:

- `svc_qport_send_delivers_message`: Set up connected ports, call
  dispatch with `SYS_QPORT_SEND`, verify `r0=0`, then dispatch
  `SYS_QPORT_RECV`, verify data matches.
- `svc_qport_send_blocks_on_full`: Fill the queue, send with
  timeout > 0, verify `r0=0` and caller transitions to `Waiting`.
- `svc_qport_recv_blocks_on_empty`: Receive with timeout > 0 on
  empty queue, verify `r0=0` and caller transitions to `Waiting`.
- `svc_qport_bad_ptr_returns_error`: Pass an out-of-range pointer,
  verify `ERR_BAD_PTR`.
- `svc_bb_display_wakes_readers`: Block readers, dispatch
  `SYS_BB_DISPLAY`, verify all readers transition to `Ready`.
- `svc_bb_read_returns_data`: Display data, dispatch `SYS_BB_READ`,
  verify `r0=0` and data is written to the buffer.
- `svc_bb_clear_does_not_wake`: Block a reader, dispatch
  `SYS_BB_CLEAR`, verify reader remains `Waiting`.

### 12.5 Timeout Sweep Tests

- `expire_all_timeouts_sweeps_both_pools`: Add expired entries to
  both queuing port and blackboard wait queues, call
  `expire_all_timeouts`, verify all expired partitions are Ready.
- `expire_all_timeouts_no_effect_when_empty`: Call on empty pools,
  verify no panics or state changes.
- `systick_with_timeouts_triggers_sweep`: Simulate a SysTick by
  calling `on_systick_with_timeouts`, verify expired entries are
  cleaned up and PendSV is triggered when appropriate.

---

## 13. Risks and Open Questions

### R1: Retry Overhead on Wakeup

The retry approach (Section 7.5) adds one extra syscall round-trip when
a blocked partition is woken. This means the woken partition re-issues
the recv/read syscall in its next time slot, consuming ~1 us of CPU time.
For a 10ms time slot this is negligible (0.01%). If profiling shows this
is a problem, the deferred-copy approach (storing `buf_ptr` in
`WaitEntry`) can be implemented as a targeted optimization.

### R2: Deadlock with Bidirectional Blocking

Two partitions using queuing ports bidirectionally with infinite timeout
can deadlock if both destination queues fill up simultaneously. Each
partition blocks on send, waiting for the other to receive. Neither can
receive because both are `Waiting`.

**Mitigations**:
1. Use non-blocking sends (`timeout=0`) for at least one direction.
2. Use finite timeouts to break the deadlock (with data loss).
3. Size queues large enough that one side can drain before the other
   fills (asymmetric `D` values, but current design uses uniform `D`
   across the pool — see R4).
4. Document this as an integration-time constraint.

### R3: Wait Queue FIFO vs. Priority

Wait queues are strictly FIFO. In a system with partition priorities,
a higher-priority partition may wait behind a lower-priority one. This
is acceptable for ARINC 653 where the schedule is deterministic and
partition time slots are the primary scheduling mechanism, not priority.
See Section 7.8 for the full priority inversion analysis, which
demonstrates that classical priority inversion cannot occur under the
static schedule table model. If a priority-based scheduler is added
in the future, wait queues must be changed to priority-ordered and
a priority inheritance or ceiling protocol must be implemented (see
Section 7.8 for the specific requirements).

### R4: Uniform Const-Generic Parameters Across Pool

All queuing ports in a pool share the same `D`, `M`, and `W`.
A system that needs one 8-deep port and one 32-deep port must set
`D=32` for all ports, wasting memory on the 8-deep one. This is a
fundamental limitation of the const-generic pool pattern used
throughout the codebase (message queues have the same constraint).

**Mitigation**: If heterogeneous ports are needed, create multiple
pools with different parameters. The current design is intentionally
simple.

### R5: `display()` Taking `&mut PartitionTable`

The blackboard `display()` method takes `&mut PartitionTable<N>` to
perform broadcast wakeups inline. This creates a borrow conflict with
`Kernel::dispatch()` which already holds `&mut self` (including
`self.partitions`). The solution is the same pattern used by
semaphore `signal()`: Rust's borrow checker allows this because
`self.blackboards` and `self.partitions` are disjoint fields.
See Section 10.5 for the destructuring workaround if needed.

### R6: Message Size Type: `usize` vs `u16`

The ring buffer stores `(usize, [u8; M])` where `usize` is 4 bytes on
ARM. Using `u16` or `u8` would save 2-3 bytes per entry. With `D=8`
times 8 ports, savings are 128-192 bytes — marginal. Using `usize`
matches Rust's slice API conventions. Accept the minor waste.

### R7: Blackboard Access Control

Any partition can display/read/clear any blackboard. This open access
model risks cross-partition interference if a buggy partition writes
incorrect data or clears a blackboard prematurely. See Section 4.1
for the full justification, risk analysis, mandatory application-level
conventions (single-writer convention, well-known IDs), and the
deferred kernel-enforced access control design.

### R8: Interaction with Existing Message Queues

The kernel now has two FIFO message primitives: `MessageQueue` (intra-
partition, exact-size messages, no direction, no routing, no timeout)
and `QueuingPort` (inter-partition, variable-size, directional, routed,
with timeout). These coexist without conflict — they serve different
use cases and are accessed through different syscall numbers.

### R9: Concurrency Between SVC and SysTick (Wait Queue Access)

**Risk**: The SVC handler and the SysTick handler both access the
same wait queues. If they could execute concurrently, a race condition
could corrupt the `heapless::Deque` internal state.

**Mitigation**: This race cannot occur on single-core Cortex-M due to
the interrupt priority configuration (SVCall = 0x00, SysTick = 0xFE).
See Section 7.6 for the full atomicity analysis.

**Severity**: Critical if the priority configuration were incorrect.
The implementation MUST verify priorities at boot (see Section 7.6.5).

### R10: Multi-Core Portability

The atomicity analysis relies on single-core Cortex-M properties. On a
multi-core target, SVC and SysTick could execute on different cores
simultaneously, breaking all atomicity guarantees.

**Mitigation**: Multi-core support is out of scope. If a multi-core
port is undertaken, all IPC pool operations must be wrapped in spinlocks
or critical sections. This risk is documented here for future reference.

---

## 14. Future Work

The following items are explicitly out of scope for this design but are
documented here for future reference:

1. **Priority inheritance / ceiling protocol for IPC blocking**: The
   current static ARINC 653 schedule table eliminates classical priority
   inversion (see Section 7.8). If a priority-based scheduler is added
   in the future, priority inheritance (PIP) or immediate priority
   ceiling (IPCP) must be implemented for queuing port and blackboard
   wait queues, and wait queue ordering must change from FIFO to
   priority-ordered.

2. **Kernel-enforced blackboard access control**: The current open
   access model (any partition can display/read/clear any blackboard)
   is intentional but carries risks (see Section 4.1). Future hardening
   could add per-blackboard `writer_pid: Option<u8>` and
   `reader_mask: u8` fields with a new `ERR_NOT_AUTHORIZED` error code.

3. **One-to-many (fan-out) routing**: The current connection model is
   one source to one destination. Fan-out for sampling ports and queuing
   ports can be added by extending `connected_port` to a list.

4. **Deferred-copy wakeup**: The current retry approach (Section 7.5)
   adds one extra syscall round-trip on wakeup. A deferred-copy
   approach (storing the caller's buffer pointer in `WaitEntry`) would
   eliminate this but adds complexity and requires storing raw pointers.

5. **Priority-ordered wait queues**: Currently FIFO. Priority ordering
   is only useful with a priority-based scheduler (see item 1).

6. **Heterogeneous const-generic parameters per port**: All ports in a
   pool share the same `D`, `M`, `W`. Supporting per-port parameters
   would require multiple pools or a trait-based approach.

7. **Blackboard `display()` wake-one variant**: An optional wake-one
   semantic (instead of broadcast) could be added as a separate syscall
   for producer-consumer patterns, though queuing ports better serve
   that use case.

8. **Full `SYS_QPORT_STATUS` struct return via pointer**: Currently
   returns only `current_depth` in `r0`. A pointer-based return could
   provide the full `QPortStatus` struct.
