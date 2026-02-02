> **Deprecated**: This design document is superseded by
> [notes/ipc-reference.md](ipc-reference.md), which contains the
> consolidated, implementation-accurate IPC reference.

# IPC Primitives Design: Sampling Ports, Queuing Ports, Blackboards, GET_TIME, and Port Connection Routing

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [Existing Architecture Summary](#2-existing-architecture-summary)
3. [SVC Call Number Allocation](#3-svc-call-number-allocation) — register conventions, data layouts, error codes
4. [GET_TIME Syscall](#4-get_time-syscall)
5. [User-Space Pointer Validation](#5-user-space-pointer-validation) — MPU-based validation mechanism, per-syscall validation sizes
6. [Sampling Ports](#6-sampling-ports) — data structures, operations, dispatch
7. [Queuing Ports](#7-queuing-ports) — data structures, blocking operations, dispatch
8. [Blackboards](#8-blackboards) — data structures, broadcast wakeup, dispatch
9. [Port Connection Routing](#9-port-connection-routing) — connection table, resolution, runtime routing
10. [Blocking/Wakeup Protocol](#10-blockingwakeup-protocol) — blocking flow, wakeup flow, re-entry
11. [Timeout Enforcement Mechanism](#11-timeout-enforcement-mechanism) — SysTick sweep, pool-level expiry, worst-case cost
12. [Extended Kernel Struct](#12-extended-kernel-struct) — full definition with all new pools
13. [New Source File Layout](#13-new-source-file-layout)
14. [Memory Budget Summary](#14-memory-budget-summary)
15. [Risks and Open Questions](#15-risks-and-open-questions)
16. [Implementation Checklist](#16-implementation-checklist)
17. [Future Work](#17-future-work)

## 1. Problem Statement

The kernel currently supports intra-partition synchronization (event flags,
semaphores, mutexes) and a basic message queue, but lacks ARINC 653-style
inter-partition communication (IPC) primitives with directional ports,
static routing, validity tracking, and timeout-based blocking. These
capabilities are required for building realistic multi-partition systems
where partitions communicate through well-defined, kernel-mediated channels.

Specifically, the kernel needs:

- **Sampling ports**: Latest-value, non-blocking IPC with freshness
  tracking (for sensor telemetry, state broadcasts)
- **Queuing ports**: FIFO message passing with blocking send/receive and
  timeout support (for commands, reliable ordered delivery)
- **Blackboards**: Shared message buffers with display/read/clear semantics
  and blocking reads (for configuration, status announcements)
- **GET_TIME syscall**: Monotonic tick counter accessible from partitions
  (already implemented in working tree, documented here for completeness)
- **Port connection routing**: Static source-to-destination mapping that
  the kernel uses to route data between ports in different partitions

## 2. Existing Architecture Summary

### Current SVC Numbering (syscall.rs)

| Number | Constant | Purpose |
|--------|----------|---------|
| 0 | `SYS_YIELD` | Yield time slice |
| 1 | *(reserved)* | `SYS_GET_ID` placeholder |
| 2 | `SYS_EVT_WAIT` | Wait on event flags |
| 3 | `SYS_EVT_SET` | Set event flags |
| 4 | `SYS_EVT_CLEAR` | Clear event flags |
| 5 | `SYS_SEM_WAIT` | Decrement semaphore |
| 6 | `SYS_SEM_SIGNAL` | Increment semaphore |
| 7 | `SYS_MTX_LOCK` | Lock mutex |
| 8 | `SYS_MTX_UNLOCK` | Unlock mutex |
| 9 | `SYS_MSG_SEND` | Send message to queue |
| 10 | `SYS_MSG_RECV` | Receive message from queue |
| 11 | `SYS_GET_TIME` | Get monotonic tick count |
| 12 | `SYS_SAMPLING_WRITE` | Write to sampling port |
| 13 | `SYS_SAMPLING_READ` | Read from sampling port |

### Register Calling Convention (svc.rs)

- `r0`: syscall number on entry; return value on exit
- `r1`: first argument (resource ID, or partition index)
- `r2`: second argument (varies per syscall)
- `r3`: third argument (varies per syscall)
- Return: `0` = success, nonzero = error code (see Section 3.3)

### Caller Identification

The SVC handler identifies the calling partition from
`KernelState.active_partition` (set by the scheduler on each partition
switch). Syscalls do **not** accept a `caller_pid` argument in any
register — the kernel determines the caller internally. This eliminates
an entire class of spoofing bugs where a partition could claim to be
another partition.

**Rationale**: The existing `dispatch()` implementation in `svc.rs`
currently passes `frame.r2` as `caller_pid` for some syscalls
(`SYS_SEM_WAIT`, `SYS_MTX_LOCK`, `SYS_MSG_SEND`, `SYS_MSG_RECV`).
These will be migrated to use `self.active_partition()` internally,
freeing `r2` for actual arguments. The migration is backwards-compatible
because the kernel is the sole consumer of the caller identity.

### Current Data Structure Patterns

All primitives use const-generic static allocation via `heapless`:

- **Pool pattern**: `heapless::Vec<ControlBlock, S>` indexed by integer ID
- **Wait queues**: `heapless::Deque<u8, W>` storing partition IDs (FIFO)
- **Data buffers**: `heapless::Deque<[u8; M], D>` for message queues
- **Outcome pattern** (message.rs): Operations return outcome enums
  (`SendOutcome`, `RecvOutcome`) that the SVC dispatcher translates into
  partition state transitions via `apply_send_outcome`/`apply_recv_outcome`
- **Kernel struct** (svc.rs): Bundles all resource pools with
  const-generic parameters. `dispatch()` routes syscalls to the
  appropriate pool operation.

### Partition State Model (partition.rs)

```
Ready -> Running -> Ready     (normal scheduling)
Running -> Waiting            (blocked on IPC)
Waiting -> Ready              (unblocked by complementary operation or timeout)
```

The scheduler is purely time-driven (ARINC 653 static schedule table).
Blocking marks a partition as `Waiting`; the scheduler still assigns the
time slot but the partition does not execute useful work until unblocked.

### Current Kernel Struct (Before This Work)

```rust
pub struct Kernel<
    const N: usize,   // Max partitions
    const S: usize,   // Max semaphores
    const SW: usize,  // Semaphore wait-queue depth
    const MS: usize,  // Max mutexes
    const MW: usize,  // Mutex wait-queue depth
    const QS: usize,  // Max message queues
    const QD: usize,  // Queue depth
    const QM: usize,  // Message size (bytes)
    const QW: usize,  // Queue wait-queue depth
    const SP: usize,  // Max sampling ports
    const SM: usize,  // Sampling port max message size
> {
    pub partitions: PartitionTable<N>,
    pub semaphores: SemaphorePool<S, SW>,
    pub mutexes: MutexPool<MS, MW>,
    pub messages: MessagePool<QS, QD, QM, QW>,
    pub tick: TickCounter,
    pub sampling: SamplingPortPool<SP, SM>,
}
```

### Extended Kernel Struct (After This Work)

This design adds queuing ports, blackboards, and a connection table to
the `Kernel` struct. See Section 12 for the full definition with all
const-generic parameters. Summary of new fields:

```rust
pub struct Kernel</* existing params..., */
    const QPS: usize, const QPD: usize, const QPM: usize, const QPW: usize,
    const BBS: usize, const BBM: usize, const BBW: usize, const CC: usize,
> {
    // ... existing fields ...
    pub queuing_ports: QueuingPortPool<QPS, QPD, QPM, QPW>,
    pub blackboards: BlackboardPool<BBS, BBM, BBW>,
    pub connections: ConnectionTable<CC>,
}
```

## 3. SVC Call Number Allocation

New syscalls extend the existing numbering starting at 14:

| Number | Constant | Purpose |
|--------|----------|---------|
| 14 | `SYS_SAMP_STATUS` | Get sampling port status |
| 15 | `SYS_QPORT_SEND` | Send queuing port message |
| 16 | `SYS_QPORT_RECV` | Receive queuing port message |
| 17 | `SYS_QPORT_STATUS` | Get queuing port status |
| 18 | `SYS_BB_DISPLAY` | Display blackboard message |
| 19 | `SYS_BB_READ` | Read blackboard message |
| 20 | `SYS_BB_CLEAR` | Clear blackboard |

**Rationale**: Creation syscalls (`CREATE_SAMPLING_PORT`, etc.) are omitted
because all resources are statically allocated at compile time. Ports and
blackboards are configured by the kernel initialization code, not by
runtime syscalls. This matches the existing pattern: semaphores, mutexes,
and message queues are created by `pool.add()` during init, not via SVC.

### 3.1 Register Convention for New Syscalls

All new syscalls follow the existing convention. Because the ARM SVC path
only provides `r0`-`r3` (4 registers), multi-argument operations pack
information as follows. The caller's partition ID is **never** passed
in a register — the kernel reads it from `self.active_partition()`.

| Syscall | r0 (entry) | r1 | r2 | r3 | r0 (return) |
|---------|------------|----|----|----|----|
| `SYS_SAMP_STATUS` | 14 | port_id | — | out_ptr | 0 or error code |
| `SYS_QPORT_SEND` | 15 | port_id | size | data_ptr | 0, 1, or error code |
| `SYS_QPORT_RECV` | 16 | port_id | buf_size | buf_ptr | 0, 1, or error code |
| `SYS_QPORT_STATUS` | 17 | port_id | — | — | current depth (or error code) |
| `SYS_BB_DISPLAY` | 18 | bb_id | size | data_ptr | 0 or error code |
| `SYS_BB_READ` | 19 | bb_id | buf_size | buf_ptr | 0, 1, or error code |
| `SYS_BB_CLEAR` | 20 | bb_id | — | — | 0 or error code |

**Key change from earlier revision**: The `caller_pid` argument has been
removed from all syscalls. The kernel already tracks the active partition
via `KernelState.active_partition`, which is set by the scheduler on each
context switch. Accepting `caller_pid` from user space would be both
redundant and a security risk (a partition could spoof another's identity).

**Key change: Buffer size passed in `r2`**: For read operations
(`SYS_SAMP_READ`, `SYS_QPORT_RECV`, `SYS_BB_READ`), the caller passes
the capacity of the **data portion** of its receive buffer in `r2`.
This allows partitions to allocate only as much stack space as they
need, rather than requiring a fixed `MAX_MSG_SIZE` allocation. The
kernel validates that the message fits before writing.

**Uniform `r2` convention**: Across all syscalls, `r2` describes only
the message data size or data buffer capacity. It never includes the
size of struct headers (timeout fields, size fields, etc.) that
surround the data in the pointed-to struct. The kernel adds the header
size internally when computing pointer validation bounds (see Section
5.4).

**Timeout for blocking operations**: `SYS_QPORT_SEND`, `SYS_QPORT_RECV`,
and `SYS_BB_READ` support blocking with timeout. Because all four
registers are occupied, the timeout value is passed via a struct at the
pointer location (see Section 3.2).

### 3.2 Data Layout for Pointer Arguments

#### `SYS_SAMP_WRITE`: `data_ptr` (r3)

Points directly to the message bytes. The size is in `r2`. No header
struct needed.

```
data_ptr --> [u8; size]    (message data, read by kernel)
```

#### `SYS_SAMP_READ`: `out_ptr` (r3)

Points to a caller-allocated `SamplingReadResult` struct:

```rust
#[repr(C)]
struct SamplingReadResult {
    size: u32,                   // Filled by kernel: actual message size
    data: [u8; /* buf_size */],  // Filled by kernel: message bytes
}
```

The caller allocates `4 + buf_size` bytes on its stack, where `buf_size`
is whatever the caller chooses (up to the port's `max_msg_size`). **`r2`
carries `buf_size` — the capacity of the `data` portion only, NOT
including the 4-byte `size` prefix.** This matches the convention used
by `SYS_QPORT_RECV` and `SYS_BB_READ`, where `r2` always describes the
data capacity, and the kernel accounts for any struct header separately.

The kernel writes `size` first, then `data[..actual_size]`. If the
message is larger than `buf_size`, the kernel returns error code
`ERR_BUF_TOO_SMALL`.

Return value in `r0`:
- `0` = data was read but is INVALID (stale / refresh period expired)
- `1` = data was read and is VALID (within refresh period)
- Error code = error (see Section 3.3)

#### `SYS_SAMP_STATUS`: `out_ptr` (r3)

Points to a `SamplingPortStatus` struct in the caller's stack:

```rust
#[repr(C)]
struct SamplingPortStatus {
    max_msg_size: u32,
    refresh_period: u32,
    direction: u32,       // 0 = Source, 1 = Destination
    last_msg_validity: u32, // 0 = Invalid, 1 = Valid, 2 = NeverWritten
}
```

This struct is exactly 16 bytes. The caller must provide at least 16
bytes at `out_ptr`. The kernel validates the pointer covers 16 bytes
within the partition's data region.

#### `SYS_QPORT_SEND`: `data_ptr` (r3)

Points to a `QueuingSendArgs` struct:

```rust
#[repr(C)]
struct QueuingSendArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    data: [u8; /* size */],       // Message payload (size from r2)
}
```

The caller places the timeout and message data contiguously. `r2`
carries the message size (not counting the 4-byte timeout prefix).
The kernel reads `timeout_ticks`, then reads `size` bytes from `data`.

Return values for `SYS_QPORT_SEND`:
- `0` = message delivered (or caller blocked, will be completed later)
- `1` = queue full and timeout was 0 (non-blocking failure)
- Error code = error (see Section 3.3)

#### `SYS_QPORT_RECV`: `buf_ptr` (r3)

Points to a `QueuingRecvArgs` struct:

```rust
#[repr(C)]
struct QueuingRecvArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    size: u32,                    // Filled by kernel: actual message size
    data: [u8; /* buf_size */],   // Filled by kernel (buf_size from r2)
}
```

The caller allocates `8 + buf_size` bytes. `r2` carries `buf_size`
(the capacity of the `data` portion only). The kernel reads
`timeout_ticks`, then on success writes `size` and `data[..actual_size]`.

Return values for `SYS_QPORT_RECV`:
- `0` = message received (or caller blocked, will be completed later)
- `1` = queue empty and timeout was 0 (non-blocking failure)
- Error code = error (see Section 3.3)

#### `SYS_BB_DISPLAY`: `data_ptr` (r3)

Points directly to the message bytes. The size is in `r2`.

#### `SYS_BB_READ`: `buf_ptr` (r3)

Points to a `BlackboardReadArgs` struct:

```rust
#[repr(C)]
struct BlackboardReadArgs {
    timeout_ticks: u32,           // 0 = non-blocking, u32::MAX = infinite
    size: u32,                    // Filled by kernel: actual message size
    data: [u8; /* buf_size */],   // Filled by kernel (buf_size from r2)
}
```

Same layout as `QueuingRecvArgs`. `r2` carries `buf_size` (capacity
of `data` portion). Return value: `0` = read succeeded, `1` = empty
and non-blocking or timeout expired, error code = error.

### 3.3 Error Code Convention

The previous convention mapped all specific errors to a single generic
value (`u32::MAX`). This prevents user-space from diagnosing failures.
The revised convention assigns distinct error codes to each failure mode.

Error codes occupy the range `0xFFFF_FF00` to `0xFFFF_FFFF`, leaving
the lower range for normal return values:

| Code | Constant | Meaning |
|------|----------|---------|
| `0xFFFF_FFFF` | `ERR_UNKNOWN` | Unrecognized syscall or internal error |
| `0xFFFF_FFFE` | `ERR_INVALID_ID` | Port/blackboard/resource ID out of range |
| `0xFFFF_FFFD` | `ERR_WRONG_DIR` | Wrong port direction (e.g., reading from a Source port) |
| `0xFFFF_FFFC` | `ERR_SIZE` | Message size exceeds port's `max_msg_size` |
| `0xFFFF_FFFB` | `ERR_NEVER_WRITTEN` | Sampling port has never been written to |
| `0xFFFF_FFFA` | `ERR_NO_CONNECTION` | Source port has no connected destination |
| `0xFFFF_FFF9` | `ERR_WAIT_FULL` | Wait queue is full; cannot block |
| `0xFFFF_FFF8` | `ERR_NOT_OWNER` | Caller does not own this port |
| `0xFFFF_FFF7` | `ERR_BUF_TOO_SMALL` | Caller's buffer is too small for the message |
| `0xFFFF_FFF6` | `ERR_BAD_PTR` | Pointer failed validation (outside partition memory) |

```rust
pub const ERR_UNKNOWN: u32       = 0xFFFF_FFFF;
pub const ERR_INVALID_ID: u32    = 0xFFFF_FFFE;
pub const ERR_WRONG_DIR: u32     = 0xFFFF_FFFD;
pub const ERR_SIZE: u32          = 0xFFFF_FFFC;
pub const ERR_NEVER_WRITTEN: u32 = 0xFFFF_FFFB;
pub const ERR_NO_CONNECTION: u32 = 0xFFFF_FFFA;
pub const ERR_WAIT_FULL: u32     = 0xFFFF_FFF9;
pub const ERR_NOT_OWNER: u32     = 0xFFFF_FFF8;
pub const ERR_BUF_TOO_SMALL: u32 = 0xFFFF_FFF7;
pub const ERR_BAD_PTR: u32       = 0xFFFF_FFF6;
```

**Mapping from internal error enums**: Each internal error variant maps
to a specific code:

```rust
impl From<SampError> for u32 {
    fn from(e: SampError) -> u32 {
        match e {
            SampError::InvalidPort    => ERR_INVALID_ID,
            SampError::WrongDirection => ERR_WRONG_DIR,
            SampError::SizeTooLarge   => ERR_SIZE,
            SampError::NeverWritten   => ERR_NEVER_WRITTEN,
            SampError::NoConnection   => ERR_NO_CONNECTION,
            SampError::NotOwner       => ERR_NOT_OWNER,
            SampError::BufTooSmall    => ERR_BUF_TOO_SMALL,
        }
    }
}

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

The SVC dispatch code uses `Err(e) => u32::from(e)` instead of
`Err(_) => u32::MAX`, giving user-space partitions specific failure
information.

### 3.4 SyscallId Enum Extension

```rust
pub enum SyscallId {
    // ... existing variants through SamplingRead (13) ...
    SampStatus,    // 14
    QPortSend,     // 15
    QPortRecv,     // 16
    QPortStatus,   // 17
    BbDisplay,     // 18
    BbRead,        // 19
    BbClear,       // 20
}
```

## 4. GET_TIME Syscall

**Status**: Already implemented in the working tree.

### Implementation

- `TickCounter` in `tick.rs`: `u64` counter, incremented on every SysTick
- `SYS_GET_TIME = 11` in `syscall.rs`, `SyscallId::GetTime` variant
- `Kernel::dispatch` returns `self.tick.get() as u32` (truncated to 32 bits)
- At 10ms tick interval (12 MHz / 120,000), the `u32` wraps after ~497 days

### Usage by New Primitives

- **Sampling ports**: `GET_TIME()` is called internally by the kernel
  during `WRITE_SAMPLING_MESSAGE` to timestamp the message, and during
  `READ_SAMPLING_MESSAGE` to compute validity. Partitions can also call
  `GET_TIME()` directly.
- **Queuing ports**: The kernel records the absolute tick deadline
  (`current_tick + timeout_ticks`) when a partition blocks on send/receive.
  The SysTick timeout sweep (Section 11) checks these deadlines.
- **Blackboards**: Same timeout mechanism as queuing ports for blocking reads.

## 5. User-Space Pointer Validation

### 5.1 The Problem

All data-transfer syscalls accept pointers from user space (`data_ptr`,
`out_ptr`, `buf_ptr` in `r3`). Without validation, a malicious or buggy
partition could pass a pointer into kernel memory, another partition's
memory, or an unmapped region, causing:

- **Kernel memory corruption**: Writing to kernel data structures via a
  crafted `out_ptr`.
- **Information leakage**: Reading kernel or other partitions' data via a
  crafted `data_ptr`.
- **Fault escalation**: Triggering a MemManage or HardFault in handler
  mode (which is unrecoverable on Cortex-M without a fault-recovery
  mechanism).

### 5.2 Validation Strategy

Every pointer received from user space must be validated before the
kernel dereferences it. The check verifies that the entire referenced
range `[ptr, ptr + len)` falls within the calling partition's **data
region** as defined by its MPU configuration.

#### Which region is checked

Each partition's `PartitionControlBlock` stores an `MpuRegion` (via
`pcb.mpu_region()`) that defines the partition's read-write data region.
In the MPU hardware configuration (`mpu.rs::partition_mpu_regions`),
this corresponds to **MPU Region 2**: the data region configured with
`AP_FULL_ACCESS` (read-write for both privileged and unprivileged code)
and execute-never (XN). The `MpuRegion` struct exposes:

- `base() -> u32`: The starting address of the data region.
- `size() -> u32`: The size of the data region in bytes.

The validation function checks that the entire pointer range falls
within `[base, base + size)`. It does **not** check against the code
region (MPU Region 1, which is read-only and executable) — data
pointers must reside in the data region.

#### Why software validation is needed

The SVC handler runs in privileged Handler mode. With the current MPU
configuration (`PRIVDEFENA = 1`), privileged code has default background
access to all memory. This means the MPU hardware does **not** fault
when the kernel dereferences an out-of-bounds pointer — the kernel can
read/write any address. Software validation is therefore the primary
defense mechanism.

#### Implementation

```rust
/// Validate that the range [ptr, ptr+len) falls entirely within the
/// partition's data region as defined by its MpuRegion.
///
/// Returns `true` if the range is valid, `false` otherwise.
fn validate_user_ptr(pcb: &PartitionControlBlock, ptr: u32, len: u32) -> bool {
    if len == 0 {
        return true; // Zero-length access is always valid
    }
    let region_base = pcb.mpu_region().base();
    let region_end = region_base.wrapping_add(pcb.mpu_region().size());

    // Check for overflow in ptr+len (prevents wraparound attacks)
    let access_end = match ptr.checked_add(len) {
        Some(end) => end,
        None => return false,
    };

    ptr >= region_base && access_end <= region_end
}
```

### 5.3 Integration with SVC Dispatch

The validation is performed **before** any kernel operation reads from or
writes to user memory. Each syscall that accepts a pointer calls
`validate_user_ptr` early and returns `ERR_BAD_PTR` on failure.

For example, `SYS_SAMP_WRITE`:

```rust
Some(SyscallId::SampWrite) => {
    let caller = self.active_partition();
    let pcb = match self.partitions.get(caller) {
        Some(p) => p,
        None => return ERR_UNKNOWN,
    };
    let data_ptr = frame.r3;
    let size = frame.r2;

    if !validate_user_ptr(pcb, data_ptr, size) {
        return ERR_BAD_PTR;
    }

    let data = unsafe { core::slice::from_raw_parts(data_ptr as *const u8, size as usize) };
    let tick = self.tick.get();
    match self.sampling_ports.write(frame.r1 as usize, caller, data, size as usize, tick) {
        Ok(()) => 0,
        Err(e) => u32::from(e),
    }
}
```

### 5.4 Validation Sizes for Each Syscall

In all cases, `r2` carries only the data size/capacity. The kernel adds
any struct header overhead to compute the total validated region:

| Syscall | Pointer | r2 meaning | Validated Size |
|---------|---------|------------|----------------|
| `SYS_SAMP_WRITE` | `data_ptr` (r3) | message size | `size` (r2) |
| `SYS_SAMP_READ` | `out_ptr` (r3) | data capacity | `buf_size` (r2) + 4 (`SamplingReadResult.size` prefix) |
| `SYS_SAMP_STATUS` | `out_ptr` (r3) | *(unused)* | 16 (sizeof `SamplingPortStatus`) |
| `SYS_QPORT_SEND` | `data_ptr` (r3) | message size | `size` (r2) + 4 (`QueuingSendArgs.timeout_ticks` prefix) |
| `SYS_QPORT_RECV` | `buf_ptr` (r3) | data capacity | `buf_size` (r2) + 8 (`QueuingRecvArgs.timeout_ticks` + `.size` prefix) |
| `SYS_QPORT_STATUS` | *(none)* | *(unused)* | *(no pointer)* |
| `SYS_BB_DISPLAY` | `data_ptr` (r3) | message size | `size` (r2) |
| `SYS_BB_READ` | `buf_ptr` (r3) | data capacity | `buf_size` (r2) + 8 (`BlackboardReadArgs.timeout_ticks` + `.size` prefix) |
| `SYS_BB_CLEAR` | *(none)* | *(unused)* | *(no pointer)* |

### 5.5 Existing Syscalls

The existing `SYS_MSG_SEND` and `SYS_MSG_RECV` syscalls also accept
pointers (`r3`) but do not currently validate them. As part of this work,
pointer validation should be added to these syscalls as well, using the
same `validate_user_ptr` function.

### 5.6 Defense in Depth: MPU as Backstop

On Cortex-M with MPU enabled (`PRIVDEFENA` set), the MPU is always active
— even during handler mode execution. However, with `PRIVDEFENA = 1`,
privileged code has default background access to all memory. Therefore,
**software validation is required** and the MPU cannot be relied upon as
the sole defense.

A future hardening measure could configure the MPU to restrict privileged
access during pointer dereferences (temporarily removing `PRIVDEFENA`),
but this adds complexity and latency. For now, software validation is
the primary mechanism.

### 5.7 Port Ownership Validation

In addition to pointer validation, each port syscall validates that the
calling partition owns the port it is accessing. Each port's control block
stores its `partition_id`; the kernel checks `port.partition_id ==
active_partition` before proceeding.

```rust
let caller = self.active_partition();
let port = self.sampling_ports.get(port_id)?;
if port.partition_id != caller {
    return Err(SampError::NotOwner);
}
```

This prevents Partition A from writing to Partition B's source port,
even though both run in the same address space at different times.

**Note**: The existing `SamplingPort` struct does not have a
`partition_id` field. This must be added as part of the sampling port
rework (see Section 6).

## 6. Sampling Ports

### 6.1 Data Structures

The existing `sampling.rs` implements `SamplingPort`, `SamplingPortPool`,
and `PortDirection`. This design adds `partition_id`, ownership
validation, a `status()` operation, and richer error handling.

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDirection {
    Source,
    Destination,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Validity {
    Valid,
    Invalid,
}

/// Control block for a single sampling port.
pub struct SamplingPort<const M: usize> {
    direction: PortDirection,
    max_size: usize,           // Compile-time max = M, runtime may be smaller
    refresh_period: u32,       // In ticks; 0 = always valid
    data: [u8; M],             // Message buffer
    current_size: usize,       // Actual size of last written message (0 = never written)
    timestamp: u64,            // Tick at which last message was written
    connected_port: Option<u8>, // Index of connected port (set by routing table)
    partition_id: u8,          // Owning partition (NEW)
}
```

**Memory budget per port**: `M + 24` bytes (M for data buffer, 24 for
control fields). With `M=64` and 8 ports: `88 * 8 = 704 bytes`.

```rust
/// Pool of sampling ports, indexed by port ID.
pub struct SamplingPortPool<const S: usize, const M: usize> {
    ports: heapless::Vec<SamplingPort<M>, S>,
}
```

### 6.2 Const-Generic Parameters

| Parameter | Meaning | Suggested Default |
|-----------|---------|-------------------|
| `S` | Max number of sampling ports | 8 |
| `M` | Max message size (bytes) | 64 |

### 6.3 Operations

#### `write(port_id, caller, data, size, current_tick) -> Result<(), SampError>`

1. Validate `port_id` is in range; return `SampError::InvalidPort` if not.
2. Validate `port.partition_id == caller`; return `SampError::NotOwner` if not.
3. Validate `direction == Source`; return `SampError::WrongDirection` if not.
4. Validate `size <= M`; return `SampError::SizeTooLarge` if not.
5. Copy `data[..size]` into `port.data[..size]`.
6. Set `port.current_size = size`.
7. Set `port.timestamp = current_tick`.
8. If `port.connected_port` is `Some(dest_id)`, copy the data and
   metadata to the destination port.
9. Return `Ok(())`.

#### `read(port_id, caller, out_buf, buf_capacity, current_tick) -> Result<(usize, Validity), SampError>`

1. Validate `port_id`, ownership, direction (`Destination`).
2. If `current_size == 0`, return `Err(SampError::NeverWritten)`.
3. If `current_size > buf_capacity`, return `Err(SampError::BufTooSmall)`.
4. Copy `port.data[..current_size]` into `out_buf[..current_size]`.
5. Compute validity:
   - If `refresh_period == 0`: always `Valid`
   - Else if `current_tick - timestamp < refresh_period as u64`: `Valid`
   - Else: `Invalid`
6. Return `Ok((current_size, validity))`.

Both operations are non-blocking. No wait queues needed.

#### `status(port_id, current_tick) -> Result<SamplingPortStatusInfo, SampError>`

```rust
pub struct SamplingPortStatusInfo {
    pub max_msg_size: usize,
    pub refresh_period: u32,
    pub direction: PortDirection,
    pub last_msg_validity: SampMsgValidity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampMsgValidity {
    Valid,
    Invalid,
    NeverWritten,
}
```

### 6.4 Error Enum

```rust
#[derive(Debug, PartialEq, Eq)]
pub enum SampError {
    InvalidPort,
    WrongDirection,
    SizeTooLarge,
    NeverWritten,
    NoConnection,
    NotOwner,
    BufTooSmall,
}
```

### 6.5 SVC Dispatch Integration

In `Kernel::dispatch`:

```rust
Some(SyscallId::SampWrite) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let data_ptr = frame.r3;
    let size = frame.r2 as usize;

    if !validate_user_ptr(pcb, data_ptr, frame.r2) {
        return ERR_BAD_PTR;
    }

    let data = unsafe { core::slice::from_raw_parts(data_ptr as *const u8, size) };
    let tick = self.tick.get();
    match self.sampling_ports.write(frame.r1 as usize, caller, data, size, tick) {
        Ok(()) => 0,
        Err(e) => u32::from(e),
    }
}
Some(SyscallId::SampRead) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let out_ptr = frame.r3;
    let buf_size = frame.r2 as usize;
    let total_size = buf_size + 4;     // + SamplingReadResult.size prefix

    if !validate_user_ptr(pcb, out_ptr, total_size as u32) {
        return ERR_BAD_PTR;
    }

    let tick = self.tick.get();
    let result_ptr = out_ptr as *mut SamplingReadResult;
    match self.sampling_ports.read(
        frame.r1 as usize, caller,
        unsafe { &mut (*result_ptr).data[..buf_size] },
        buf_size, tick,
    ) {
        Ok((actual_size, Validity::Invalid)) => {
            unsafe { (*result_ptr).size = actual_size as u32 };
            0  // Data read, but stale
        }
        Ok((actual_size, Validity::Valid)) => {
            unsafe { (*result_ptr).size = actual_size as u32 };
            1  // Data read and fresh
        }
        Err(e) => u32::from(e),
    }
}
Some(SyscallId::SampStatus) => {
    let pcb = self.partitions.get(self.active_partition()).unwrap();
    let out_ptr = frame.r3;

    if !validate_user_ptr(pcb, out_ptr, 16) {
        return ERR_BAD_PTR;
    }

    let tick = self.tick.get();
    match self.sampling_ports.status(frame.r1 as usize, tick) {
        Ok(info) => {
            let status = SamplingPortStatus {
                max_msg_size: info.max_msg_size as u32,
                refresh_period: info.refresh_period,
                direction: info.direction as u32,
                last_msg_validity: info.last_msg_validity as u32,
            };
            unsafe { core::ptr::write(out_ptr as *mut SamplingPortStatus, status) };
            0
        }
        Err(e) => u32::from(e),
    }
}
```

## 7. Queuing Ports

### 7.1 Data Structures

```rust
/// Control block for a single queuing port.
pub struct QueuingPort<const D: usize, const M: usize, const W: usize> {
    direction: PortDirection,
    buf: heapless::Deque<(usize, [u8; M]), D>,  // (size, data) pairs
    wait_queue: heapless::Deque<WaitEntry, W>,   // Blocked partitions
    connected_port: Option<u8>,                   // Routing target
    partition_id: u8,                             // Owning partition
}

/// Entry in a port's wait queue, storing the deadline for timeout.
#[derive(Debug, Clone, Copy)]
pub struct WaitEntry {
    pub partition_id: u8,
    pub deadline: u64,    // Tick at which timeout expires; u64::MAX = infinite wait
}
```

**Memory budget per port**: `D * (M + size_of::<usize>()) + W * 12 + 8`
overhead. With `D=8`, `M=64`, `W=4`: `8*68 + 4*12 + 8 = 600 bytes`.
With 8 queuing ports: ~4.8 KB.

```rust
/// Pool of queuing ports.
pub struct QueuingPortPool<const S: usize, const D: usize, const M: usize, const W: usize> {
    ports: heapless::Vec<QueuingPort<D, M, W>, S>,
}
```

### 7.2 Const-Generic Parameters

| Parameter | Meaning | Suggested Default |
|-----------|---------|-------------------|
| `S` | Max number of queuing ports | 8 |
| `D` | Queue depth (messages per port) | 8 |
| `M` | Max message size (bytes) | 64 |
| `W` | Wait-queue depth per port | 4 |

### 7.3 Operations

#### `send(port_id, caller, data, size, timeout_ticks, current_tick) -> Result<QPortSendOutcome, QPortError>`

1. Validate port exists; return `QPortError::InvalidPort` if not.
2. Validate `port.partition_id == caller`; return `QPortError::NotOwner` if not.
3. Validate `direction == Source`; return `QPortError::WrongDirection` if not.
4. Validate `size <= M`; return `QPortError::SizeTooLarge` if not.
5. Resolve the connected destination port (`connected_port`); return
   `QPortError::NoConnection` if `None`.
6. Attempt to push `(size, data)` into the **destination port's** ring buffer.
7. If the destination buffer has space:
   - Enqueue the message.
   - If the destination's wait queue has a blocked receiver, pop it.
   - Return `QPortSendOutcome::Delivered { wake_receiver }`.
8. If the destination buffer is full:
   - If `timeout_ticks == 0`: return `QPortSendOutcome::Full`.
   - Else: compute `deadline`, push `WaitEntry` onto the **source port's**
     wait queue, return `QPortSendOutcome::SenderBlocked { blocked: caller }`.

#### `receive(port_id, caller, buf, buf_capacity, timeout_ticks, current_tick) -> Result<QPortRecvOutcome, QPortError>`

1. Validate port exists, ownership, direction (`Destination`).
2. If the port's ring buffer has messages:
   - Peek at the oldest `(size, data)`.
   - If `size > buf_capacity`, return `Err(QPortError::BufTooSmall)`.
   - Pop the oldest message. Copy into caller's buffer.
   - If the connected source port's wait queue has a blocked sender, pop it.
   - Return `QPortRecvOutcome::Received { size, wake_sender }`.
3. If empty:
   - If `timeout_ticks == 0`: return `QPortRecvOutcome::Empty`.
   - Else: push `WaitEntry` onto destination port's wait queue,
     return `QPortRecvOutcome::ReceiverBlocked { blocked: caller }`.

#### `status(port_id) -> Result<QPortStatus, QPortError>`

```rust
pub struct QPortStatus {
    pub current_depth: usize,
    pub max_depth: usize,
    pub max_msg_size: usize,
    pub direction: PortDirection,
    pub waiting_count: usize,
}
```

For the SVC return path, only `current_depth` is returned in `r0`.

### 7.4 Outcome Enums

```rust
#[derive(Debug, PartialEq, Eq)]
pub enum QPortSendOutcome {
    Delivered { wake_receiver: Option<u8> },
    SenderBlocked { blocked: u8 },
    Full, // Non-blocking, queue was full, timeout=0
}

#[derive(Debug, PartialEq, Eq)]
pub enum QPortRecvOutcome {
    Received { size: usize, wake_sender: Option<u8> },
    ReceiverBlocked { blocked: u8 },
    Empty, // Non-blocking, queue was empty, timeout=0
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

### 7.5 SVC Dispatch Integration

```rust
Some(SyscallId::QPortSend) => {
    let caller = self.active_partition();
    let pcb = self.partitions.get(caller).unwrap();
    let msg_size = frame.r2 as usize;
    let total_size = msg_size + 4; // + timeout_ticks prefix

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
    let total_size = buf_capacity + 8;

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
        Ok(status) => status.current_depth as u32,
        Err(e) => u32::from(e),
    }
}
```

**Outcome applicators**:

```rust
fn apply_qport_send_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: QPortSendOutcome,
) -> u32 {
    match outcome {
        QPortSendOutcome::Delivered { wake_receiver: Some(pid) } => {
            try_transition(partitions, pid, PartitionState::Ready);
            0
        }
        QPortSendOutcome::Delivered { wake_receiver: None } => 0,
        QPortSendOutcome::SenderBlocked { blocked } => {
            try_transition(partitions, blocked, PartitionState::Waiting);
            0
        }
        QPortSendOutcome::Full => 1,
    }
}

fn apply_qport_recv_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: QPortRecvOutcome,
    recv_args_ptr: *mut QueuingRecvArgs,
) -> u32 {
    match outcome {
        QPortRecvOutcome::Received { size, wake_sender: Some(pid) } => {
            try_transition(partitions, pid, PartitionState::Ready);
            unsafe { (*recv_args_ptr).size = size as u32 };
            0
        }
        QPortRecvOutcome::Received { size, wake_sender: None } => {
            unsafe { (*recv_args_ptr).size = size as u32 };
            0
        }
        QPortRecvOutcome::ReceiverBlocked { blocked } => {
            try_transition(partitions, blocked, PartitionState::Waiting);
            0
        }
        QPortRecvOutcome::Empty => 1,
    }
}
```

## 8. Blackboards

### 8.1 Data Structures

```rust
/// Control block for a single blackboard.
pub struct Blackboard<const M: usize, const W: usize> {
    data: [u8; M],
    current_size: usize,          // 0 when cleared/never written
    is_empty: bool,               // true initially and after clear
    wait_queue: heapless::Deque<WaitEntry, W>,  // Readers blocked on empty
}
```

**Memory budget per blackboard**: `M + W * 12 + 8` bytes. With `M=64`,
`W=4`: `64 + 48 + 8 = 120 bytes`. With 4 blackboards: ~480 bytes.

```rust
/// Pool of blackboards.
pub struct BlackboardPool<const S: usize, const M: usize, const W: usize> {
    boards: heapless::Vec<Blackboard<M, W>, S>,
}
```

**No direction field**: Unlike ports, blackboards have no directional
constraint. Any partition can display, read, or clear any blackboard.
Access control is out of scope.

### 8.2 Const-Generic Parameters

| Parameter | Meaning | Suggested Default |
|-----------|---------|-------------------|
| `S` | Max number of blackboards | 4 |
| `M` | Max message size (bytes) | 64 |
| `W` | Wait-queue depth per board | 4 |

### 8.3 Operations

#### `display(parts, bb_id, data, size) -> Result<(), BbError>`

1. Validate `bb_id`, `size <= M`.
2. Copy `data[..size]` into `board.data[..size]`.
3. Set `board.current_size = size`, `board.is_empty = false`.
4. Drain the entire wait queue: wake ALL blocked readers (broadcast).
5. Return `Ok(())`.

**Key**: Display wakes **all** waiting readers, not just one. This
is the correct semantic for a shared state buffer — all interested
readers should see the new data.

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

#### `read(bb_id, caller, buf, buf_capacity, timeout_ticks, current_tick) -> Result<BbReadOutcome, BbError>`

1. Validate `bb_id`, `buf_capacity`.
2. If `!board.is_empty`:
   - If `current_size > buf_capacity`, return `Err(BbError::BufTooSmall)`.
   - Copy data. Return `BbReadOutcome::Read { size }`.
3. If `board.is_empty`:
   - If `timeout_ticks == 0`: return `BbReadOutcome::Empty`.
   - Else: push `WaitEntry`, return `BbReadOutcome::ReaderBlocked { blocked }`.

#### `clear(bb_id) -> Result<(), BbError>`

1. Set `board.is_empty = true`, `board.current_size = 0`.
2. Do NOT wake any readers. Blocked readers remain blocked until a
   subsequent `display` or their timeout expires.

**Rationale**: Clearing makes the blackboard "more empty" — readers are
waiting for data, so waking them would just cause immediate re-blocking.

### 8.4 Outcome Enums

```rust
#[derive(Debug, PartialEq, Eq)]
pub enum BbReadOutcome {
    Read { size: usize },
    ReaderBlocked { blocked: u8 },
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

### 8.5 SVC Dispatch Integration

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
            try_transition(&mut self.partitions, blocked, PartitionState::Waiting);
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

## 9. Port Connection Routing

### 9.1 Design

ARINC 653 ports are directional (SOURCE or DESTINATION) and connected by a
static routing table configured at system integration time. The kernel
maintains a connection table that maps each source port to its destination
port(s).

For this implementation, the routing is **one-to-one**: each source port
connects to exactly one destination port. One-to-many (fan-out) is out of
scope.

### 9.2 Connection Table

```rust
/// A single connection between a source port and a destination port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PortConnection {
    pub source_kind: PortKind,
    pub source_id: u8,
    pub dest_kind: PortKind,
    pub dest_id: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortKind {
    Sampling,
    Queuing,
}

/// Static connection table.
pub struct ConnectionTable<const C: usize> {
    connections: heapless::Vec<PortConnection, C>,
}
```

### 9.3 Configuration Process

```rust
// 1. Create pools (empty)
let mut sampling_ports = SamplingPortPool::<8, 64>::new();
let mut queuing_ports = QueuingPortPool::<8, 8, 64, 4>::new();
let mut connections = ConnectionTable::<16>::new();

// 2. Add ports with their direction and owning partition
sampling_ports.add(SamplingPort::new(PortDirection::Source, 32, 0)).unwrap();
sampling_ports.add(SamplingPort::new(PortDirection::Destination, 32, 1)).unwrap();
queuing_ports.add(QueuingPort::new(PortDirection::Source, 0)).unwrap();
queuing_ports.add(QueuingPort::new(PortDirection::Destination, 1)).unwrap();

// 3. Define connections
connections.add(PortConnection {
    source_kind: PortKind::Sampling, source_id: 0,
    dest_kind: PortKind::Sampling, dest_id: 1,
}).unwrap();
connections.add(PortConnection {
    source_kind: PortKind::Queuing, source_id: 0,
    dest_kind: PortKind::Queuing, dest_id: 1,
}).unwrap();

// 4. Resolve connections
connections.resolve(&mut sampling_ports, &mut queuing_ports).unwrap();
```

### 9.4 Connection Resolution

```rust
impl<const C: usize> ConnectionTable<C> {
    pub fn resolve<...>(
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
                    // Validate source is Source, dest is Destination
                    // Set source.connected_port = Some(dest_id)
                }
                PortKind::Queuing => {
                    // Same validation for queuing ports
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

### 9.5 Runtime Routing

**Sampling ports**: When `WRITE_SAMPLING_MESSAGE` is called on a source
port, the kernel copies the data directly into the connected destination
port's buffer within the same SVC call.

**Queuing ports**: When `SEND_QUEUING_MESSAGE` is called on a source
port, the kernel pushes the message into the connected destination port's
ring buffer. If the destination's ring buffer is full and the caller
specified a nonzero timeout, the caller blocks on the source port's wait
queue.

### 9.6 Data Flow Diagrams

```
Partition A (SOURCE)          Kernel                Partition B (DESTINATION)
     |                          |                          |
     |--WRITE_SAMPLING(port 0)->|                          |
     |                          |--copy data to port 1---->|
     |                          |          READ_SAMPLING(port 1)
     |                          |<-------------------------|
     |                          |--return data + validity->|
```

```
Partition A (SOURCE)          Kernel                Partition B (DESTINATION)
     |                          |                          |
     |--SEND_QUEUING(port 0)--->|                          |
     |                          |--enqueue in port 1------>|
     |                          |          RECV_QUEUING(port 1)
     |                          |<-------------------------|
     |                          |--dequeue + return data-->|
```

### 9.7 Memory Budget

Each `PortConnection` is 4 bytes. With `C=16` connections: 64 bytes + Vec
overhead. Negligible.

## 10. Blocking/Wakeup Protocol

### 10.1 Overview

Queuing ports and blackboard reads support blocking with timeout. The
protocol follows the existing pattern (semaphores, mutexes, message
queues) with the addition of deadline-based timeout expiry.

### 10.2 Blocking Flow

When a partition calls a blocking operation (e.g., `RECV_QUEUING_MESSAGE`
with `timeout > 0` on an empty queue):

1. The pool operation pushes a `WaitEntry { partition_id, deadline }` onto
   the resource's wait queue.
2. The pool operation returns an outcome indicating the caller should block.
3. The SVC dispatcher calls `try_transition(partitions, blocked, Waiting)`.
4. Control returns to the SVC handler. The partition is now `Waiting`.
5. The scheduler continues to assign time slots per the static schedule.

**Key**: In the current ARINC 653-style scheduler, blocking does NOT skip
the partition's time slot. The partition still receives its time window
but cannot do useful work. This is by design: the schedule table is
deterministic.

### 10.3 Wakeup Flow

When a complementary operation occurs (e.g., `SEND_QUEUING_MESSAGE`
delivers a message to a queue that has blocked receivers):

1. The pool operation pops the front `WaitEntry` from the wait queue.
2. Returns an outcome indicating the woken partition.
3. The SVC dispatcher calls `try_transition(partitions, pid, Ready)`.

For blackboards, `display` wakes **all** blocked readers (broadcast).

### 10.4 Partition Re-Entry After Wakeup

When a blocked partition is woken and subsequently scheduled, execution
resumes at the instruction after the SVC call. The partition re-issues
the syscall to determine the actual outcome:

- **Woken by data**: Re-issue returns `0` (success) with data.
- **Woken by timeout**: Re-issue returns `1` (empty/full).

**Decision**: Use the retry approach (no deferred copy). This is simpler
and avoids storing raw user-space pointers in `WaitEntry`. The `WaitEntry`
structure stays simple:

```rust
pub struct WaitEntry {
    pub partition_id: u8,
    pub deadline: u64,
}
```

### 10.5 Atomicity and Concurrency Analysis

All operations execute inside the SVC handler, which runs in Handler mode
at SVCall priority 0x00 (highest configurable). On single-core Cortex-M:

1. **SVC operations are non-preemptible by application code.**
2. **SVC operations are non-preemptible by SysTick** (priority 0xFE).
3. **SysTick operations are non-preemptible by SVC** (no partition code
   runs during SysTick handler).

**No additional synchronization primitives** are required.

**Double-wakeup prevention**: When a complementary operation wakes a
partition, it removes the `WaitEntry` from the wait queue. The timeout
sweep will not find the removed entry. The priority ordering ensures
SVC always completes before SysTick runs.

## 11. Timeout Enforcement Mechanism

### 11.1 Problem

Existing blocking primitives block indefinitely — there is no timeout.
Queuing ports and blackboards require deadline-based timeout.

### 11.2 Design: Per-Tick Timeout Sweep

On every SysTick interrupt, after advancing the schedule table, scan all
wait queues for expired entries.

**Execution order in SysTick handler**:

```
1. tick.increment()
2. schedule.advance_tick()
3. queuing_ports.expire_timeouts()
4. blackboards.expire_timeouts()
5. (If schedule advanced) trigger PendSV
```

### 11.3 Sweep Implementation: Drain-and-Re-enqueue

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
                        if let Some(p) = parts.get_mut(entry.partition_id as usize) {
                            let _ = p.transition(PartitionState::Ready);
                        }
                    } else {
                        let _ = port.wait_queue.push_back(entry);
                    }
                }
            }
        }
    }
}
```

**Algorithm**: For each port, drain all entries from the front. Expired
entries are dropped (partition transitioned to Ready). Non-expired entries
are pushed back. This preserves FIFO ordering.

Blackboard pool's `expire_timeouts` is identical in structure.

### 11.4 Kernel-Level Aggregation

```rust
impl Kernel<...> {
    pub fn expire_all_timeouts(&mut self) {
        let tick = self.tick.get();
        self.queuing_ports.expire_timeouts(&mut self.partitions, tick);
        self.blackboards.expire_timeouts(&mut self.partitions, tick);
    }
}
```

### 11.5 SysTick Integration

```rust
pub fn on_systick_with_timeouts<...>(kernel: &mut Kernel<...>) -> Option<u8> {
    let next = kernel.advance_schedule_tick();
    kernel.expire_all_timeouts();
    if next.is_some() {
        #[cfg(not(test))]
        unsafe {
            core::ptr::write_volatile(SCB_ICSR as *mut u32, ICSR_PENDSVSET);
        }
    }
    next
}
```

### 11.6 Worst-Case Sweep Cost

| Resource | Ports | Wait-Queue Depth | Total Entries |
|----------|-------|-----------------|---------------|
| Queuing ports | 8 | 4 | 32 |
| Blackboards | 4 | 4 | 16 |
| **Total** | | | **48** |

Each entry check: ~10 cycles. Total: ~480 cycles per tick.
At 12 MHz with 10ms tick (120,000 cycles), the sweep consumes **0.4%**
of the tick budget. Negligible.

### 11.7 Deadline Computation

```rust
let deadline = if timeout_ticks == u32::MAX {
    u64::MAX  // Infinite wait
} else {
    current_tick + timeout_ticks as u64
};
```

The `u64` tick counter wraps after ~5.8 billion years at 10ms/tick.

## 12. Extended Kernel Struct

### 12.1 Full Definition

```rust
pub struct Kernel<
    const N: usize,    // Max partitions
    const S: usize,    // Max semaphores
    const SW: usize,   // Semaphore wait-queue depth
    const MS: usize,   // Max mutexes
    const MW: usize,   // Mutex wait-queue depth
    const QS: usize,   // Max message queues
    const QD: usize,   // Queue depth
    const QM: usize,   // Message size (bytes)
    const QW: usize,   // Queue wait-queue depth
    const SP: usize,   // Max sampling ports
    const SM: usize,   // Sampling port max message size
    // New for queuing ports:
    const QPS: usize,  // Max queuing ports
    const QPD: usize,  // Queuing port depth
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
    pub sampling: SamplingPortPool<SP, SM>,
    pub queuing_ports: QueuingPortPool<QPS, QPD, QPM, QPW>,
    pub blackboards: BlackboardPool<BBS, BBM, BBW>,
    pub connections: ConnectionTable<CC>,
}
```

### 12.2 Demo Configuration Type Alias

```rust
/// Standard kernel for demos with all IPC primitives.
pub type DemoKernel = Kernel<
    4,   // N: partitions
    4,   // S: semaphores
    4,   // SW: semaphore wait-queue
    4,   // MS: mutexes
    4,   // MW: mutex wait-queue
    4,   // QS: message queues
    4,   // QD: message queue depth
    64,  // QM: message queue msg size
    4,   // QW: message queue wait-queue
    8,   // SP: sampling ports
    64,  // SM: sampling port msg size
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

## 13. New Source File Layout

| File | Purpose |
|------|---------|
| `kernel/src/sampling.rs` | `SamplingPort`, `SamplingPortPool`, `SampError`, `SamplingPortStatusInfo` (rework existing) |
| `kernel/src/queuing.rs` | `QueuingPort`, `QueuingPortPool`, `QPortError`, `WaitEntry` |
| `kernel/src/blackboard.rs` | `Blackboard`, `BlackboardPool`, `BbError` |
| `kernel/src/routing.rs` | `PortConnection`, `PortKind`, `ConnectionTable`, `PortDirection`, `ConnectionError` |
| `kernel/src/validate.rs` | `validate_user_ptr`, error code constants |

Each file follows the same structure as existing modules:
1. Error enum
2. Control block struct
3. Pool struct with `new()`, `add()`, `get()`, operation methods
4. `#[cfg(test)] mod tests` with thorough unit tests

`PortDirection` and `WaitEntry` are used by both sampling and queuing
modules. Place them in `routing.rs` (shared types).

Add to `lib.rs`:
```rust
pub mod blackboard;
pub mod queuing;
pub mod routing;
pub mod validate;
```

## 14. Memory Budget Summary

Assumptions: 4 partitions, 8 sampling ports, 8 queuing ports, 4
blackboards, 16 connections. `M=64` for all message buffers. `D=8`
queue depth. `W=4` wait-queue depth.

| Component | Per-Instance | Count | Total |
|-----------|-------------|-------|-------|
| SamplingPort (M=64) | 88 bytes | 8 | 704 bytes |
| QueuingPort (D=8, M=64, W=4) | ~600 bytes | 8 | ~4,800 bytes |
| Blackboard (M=64, W=4) | ~120 bytes | 4 | ~480 bytes |
| ConnectionTable (C=16) | 4 bytes/entry | 16 | 64 bytes |
| **Total new IPC** | | | **~6,048 bytes** |

This fits well within the 64 KB RAM budget. Existing kernel usage
(partition stacks, existing IPC pools, etc.) is ~6 KB. The new IPC adds
~6 KB, leaving ample headroom.

## 15. Risks and Open Questions

### R1: Const-Generic Explosion

The `Kernel` struct will have 20 const-generic parameters. This makes
type signatures verbose and error messages hard to read. Mitigations:
type aliases, builder pattern for kernel init.

### R2: Timeout Sweep Cost

Linear scan of all wait queues every tick: O(total_ports * wait_queue_depth).
With small bounds (~48 entries), this is ~500 cycles. Acceptable.

### R3: No Deferred Copy for Queuing Port Wakeup

Woken partitions must re-issue the recv syscall. Adds one extra syscall
round-trip. Simplicity outweighs the performance cost.

### R4: Single-Writer Assumption for Sampling Ports

One SOURCE to one DESTINATION. ARINC 653 multicast can be added later.

### R5: Blackboard Access Control

Any partition can read/write/clear any blackboard. Out of scope.

### R6: Partition State Transition During Timeout Sweep

The `expire_timeouts` function transitions partitions `Waiting -> Ready`.
The `let _ =` pattern provides a safety net for edge cases.

### R7: Message Size Mismatch Between Source and Destination

All ports in a pool share the same `M`. The connection table validation
should verify that connected ports are in the same pool.

### R8: `heapless::Deque` Removal by Index

The drain-and-re-enqueue approach in `expire_timeouts` is O(n) per queue
per tick. With wait queues of depth 4, this is 4 iterations. Acceptable.

### R9: Deadlock Between Partitions Using Queuing Ports

Two partitions can deadlock with bidirectional infinite-timeout blocking.
Mitigations: non-blocking sends, finite timeouts, documentation.

### R10: Timeout Discrimination on Wakeup

The retry approach naturally resolves the ambiguity between "woken by
data" and "woken by timeout" at the cost of an extra syscall.

## 16. Implementation Checklist

1. **GET_TIME**: Already implemented. Commit the working tree changes.
2. **validate.rs**: `validate_user_ptr`, error code constants, `From` impls.
3. **routing.rs**: `PortDirection`, `PortKind`, `PortConnection`,
   `ConnectionTable`, `ConnectionError`, `WaitEntry`.
4. **sampling.rs**: Add `partition_id`, ownership checks, `BufTooSmall`,
   `NeverWritten`, `status()` operation. Update error enum.
5. **syscall.rs**: Add constants 14-20, `SyscallId` variants.
6. **svc.rs**: Extend `Kernel` struct. Add `active_partition()`. Add
   sampling dispatch with pointer validation. Migrate existing syscalls
   to use `active_partition()`.
7. **Demo 1**: `kernel/examples/sampling_telemetry.rs` — 3-partition
   QEMU example.
8. **queuing.rs**: `QueuingPort`, `QueuingPortPool`, send/recv with
   blocking, ownership checks, buffer-size validation.
9. **svc.rs**: Add queuing port dispatch and outcome applicators.
10. **tick.rs**: Add `on_systick_with_timeouts` and `expire_all_timeouts`.
11. **Demo 2**: `kernel/examples/queuing_cmd_response.rs`.
12. **blackboard.rs**: `Blackboard`, `BlackboardPool`, display/read/clear.
13. **svc.rs**: Add blackboard dispatch.
14. **Demo 3**: `kernel/examples/blackboard_config.rs`.
15. **svc.rs**: Add pointer validation to existing `SYS_MSG_SEND`/`SYS_MSG_RECV`.

## 17. Future Work

- **One-to-many (fan-out) routing** for sampling ports
- **Deferred-copy wakeup** to avoid retry round-trip
- **Priority-based wait queues**
- **External configuration file** for connection table
- **Blackboard access control**
- **`SYS_QPORT_STATUS` full struct return** via pointer
- **Privileged-mode MPU restriction** during pointer dereferences
