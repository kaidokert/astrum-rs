# IPC Reference

Consolidated reference for all IPC primitives: event flags, semaphores,
mutexes, message queues, sampling ports, queuing ports, and blackboards.
For context on the SVC dispatch mechanism, PendSV context switching, and
the static scheduler that drives partition execution, see
[architecture.md](architecture.md).

## Table of Contents

- [1. SVC Syscall Table](#1-svc-syscall-table)
- [2. Register Calling Convention](#2-register-calling-convention)
- [3. Error Codes](#3-error-codes)
- [4. Sampling Ports](#4-sampling-ports)
- [5. Queuing Ports](#5-queuing-ports)
- [6. Blackboards](#6-blackboards)
- [7. Port Connection Routing](#7-port-connection-routing)
- [8. Blocking and Wakeup Protocol](#8-blocking-and-wakeup-protocol)
- [9. Timeout Enforcement](#9-timeout-enforcement)

## 1. SVC Syscall Table

Defined in `kernel/src/syscall.rs`. Number 1 is reserved (gap).

| Number | Constant             | SyscallId variant | Purpose                      | Feature       |
|--------|----------------------|-------------------|------------------------------|---------------|
| 0      | `SYS_YIELD`          | `Yield`           | Yield time slice (PendSV)    |               |
| 1      | *(reserved)*         | —                 | —                            |               |
| 2      | `SYS_EVT_WAIT`       | `EventWait`       | Wait on event flags          |               |
| 3      | `SYS_EVT_SET`        | `EventSet`        | Set event flags              |               |
| 4      | `SYS_EVT_CLEAR`      | `EventClear`      | Clear event flags            |               |
| 5      | `SYS_SEM_WAIT`       | `SemWait`         | Decrement semaphore          |               |
| 6      | `SYS_SEM_SIGNAL`     | `SemSignal`       | Increment semaphore          |               |
| 7      | `SYS_MTX_LOCK`       | `MutexLock`       | Lock mutex                   |               |
| 8      | `SYS_MTX_UNLOCK`     | `MutexUnlock`     | Unlock mutex                 |               |
| 9      | `SYS_MSG_SEND`       | `MsgSend`         | Send to message queue        |               |
| 10     | `SYS_MSG_RECV`       | `MsgRecv`         | Receive from message queue   |               |
| 11     | `SYS_GET_TIME`       | `GetTime`         | Get monotonic tick count     |               |
| 12     | `SYS_SAMPLING_WRITE` | `SamplingWrite`   | Write to sampling port       |               |
| 13     | `SYS_SAMPLING_READ`  | `SamplingRead`    | Read from sampling port      |               |
| 14     | `SYS_QUEUING_SEND`   | `QueuingSend`     | Send queuing port message    |               |
| 15     | `SYS_QUEUING_RECV`   | `QueuingRecv`     | Receive queuing port message |               |
| 16     | `SYS_QUEUING_STATUS` | `QueuingStatus`   | Get queuing port status      |               |
| 17     | `SYS_BB_DISPLAY`     | `BbDisplay`       | Display blackboard message   |               |
| 18     | `SYS_BB_READ`        | `BbRead`          | Read blackboard message      |               |
| 19     | `SYS_BB_CLEAR`       | `BbClear`         | Clear blackboard             |               |
| 20     | `SYS_BUF_ALLOC`     | `BufferAlloc`     | Allocate buffer pool slot    | `dynamic-mpu` |
| 21     | `SYS_BUF_RELEASE`   | `BufferRelease`   | Release buffer pool slot     | `dynamic-mpu` |
| 22     | `SYS_DEV_OPEN`      | `DevOpen`         | Open virtual device          | `dynamic-mpu` |
| 23     | `SYS_DEV_READ`      | `DevRead`         | Read from virtual device     | `dynamic-mpu` |
| 24     | `SYS_DEV_WRITE`     | `DevWrite`        | Write to virtual device      | `dynamic-mpu` |
| 25     | `SYS_DEV_IOCTL`     | `DevIoctl`        | Device I/O control           | `dynamic-mpu` |
| 26     | `SYS_BUF_WRITE`     | `BufferWrite`     | Write data to buffer slot    | `dynamic-mpu` |

Total: 20 base syscall numbers (0-19), with a gap at 1 (reserved for
`SYS_GET_ID`). When the `dynamic-mpu` feature is enabled, 7 additional
syscalls (20-26) are available. The first invalid number is 20 (without
`dynamic-mpu`) or 27 (with it); all invalid numbers return
`SvcError::InvalidSyscall`.

## 2. Register Calling Convention

Defined by the SVCall assembly trampoline and `Kernel::dispatch` in
`kernel/src/svc.rs`.

- **r0**: syscall number on entry; return value on exit
- **r1**: first argument (resource ID — port, semaphore, mutex, queue,
  or blackboard index)
- **r2**: second argument (event mask, data length, timeout, or status
  pointer — varies per syscall)
- **r3**: third argument (pointer to user data buffer, where applicable)

The caller's partition identity comes from `Kernel.current_partition`
(set by the scheduler), not from any register.

### Per-syscall register usage

The kernel identifies the calling partition via `Kernel.current_partition`.
User partitions never supply their own PID in a register.

| Syscall          | r1             | r2           | r3          | r0 (return)        |
|------------------|----------------|--------------|-------------|---------------------|
| Yield            | —              | —            | —           | 0                   |
| EventWait        | partition_idx  | mask         | —           | matched flags or 0  |
| EventSet         | target_idx     | mask         | —           | 0 or error          |
| EventClear       | partition_idx  | mask         | —           | 0 or error          |
| SemWait          | sem_id         | —            | —           | 0 or error          |
| SemSignal        | sem_id         | —            | —           | 0 or error          |
| MutexLock        | mutex_id       | —            | —           | 0 or error          |
| MutexUnlock      | mutex_id       | —            | —           | 0 or error          |
| MsgSend          | queue_id       | —            | data_ptr    | 0 or error          |
| MsgRecv          | queue_id       | —            | buf_ptr     | byte count or error |
| GetTime          | —              | —            | —           | tick count (u32)    |
| SamplingWrite    | port_id        | data_len     | data_ptr    | 0 or error          |
| SamplingRead     | port_id        | —            | buf_ptr     | byte count or error |
| QueuingSend      | port_id        | data_len     | data_ptr    | 0 or error          |
| QueuingRecv      | port_id        | —            | buf_ptr     | byte count or error |
| QueuingStatus    | port_id        | status_ptr   | —           | 0 or error          |
| BbDisplay        | bb_id          | data_len     | data_ptr    | 0 or error          |
| BbRead           | bb_id          | timeout      | buf_ptr     | byte count or error |
| BbClear          | bb_id          | —            | —           | 0 or error          |

<!-- Note: Some dispatch paths in svc.rs pass frame.r2 as the caller
     parameter rather than using Kernel.current_partition. The table
     above documents the intended convention. -->

## 3. Error Codes

Defined as `SvcError` enum in `kernel/src/svc.rs`. All error codes have
the high bit set (>= `0x8000_0000`), distinguishing them from success
values.

| Variant            | Value          | Meaning                                    |
|--------------------|----------------|--------------------------------------------|
| `InvalidSyscall`   | `0xFFFF_FFFF`  | Unrecognized syscall number                |
| `InvalidResource`  | `0xFFFF_FFFE`  | Resource ID out of range or not allocated  |
| `WaitQueueFull`    | `0xFFFF_FFFD`  | Wait queue at capacity, cannot block       |
| `TransitionFailed` | `0xFFFF_FFFC`  | Partition state transition not permitted   |
| `InvalidPartition` | `0xFFFF_FFFB`  | Partition index out of range               |
| `OperationFailed`  | `0xFFFF_FFFA`  | Direction violation, message too large, etc |
| `InvalidPointer`   | `0xFFFF_FFF9`  | User pointer outside caller's MPU region   |

Module-level error enums (`SamplingError`, `QueuingError`,
`BlackboardError`) are mapped to `SvcError` variants by the dispatch
layer.

### Pointer validation

Syscalls that dereference a user-space data buffer pointer (r3 for data
buffers, r2 for `QueuingStatus`) are validated before the kernel
dereferences them. The `validate_user_ptr(partitions, pid, ptr, len)`
function in `kernel/src/svc.rs` checks that the byte range
`[ptr, ptr+len)` lies entirely within the calling partition's MPU data
region and that `ptr + len` does not overflow `u32`. If validation
fails, the syscall returns `SvcError::InvalidPointer` (`0xFFFF_FFF9`)
without performing any operation. The validated syscalls are: MsgSend,
MsgRecv, SamplingWrite, SamplingRead, QueuingSend, QueuingRecv,
QueuingStatus, BbDisplay, BbRead, and (with `dynamic-mpu`) BufferWrite,
DevRead, and DevWrite. Syscalls that pass integer arguments rather than
pointers (e.g., DevOpen passes a device ID, DevIoctl passes opaque
command/argument values) do not require pointer validation.

## 4. Sampling Ports

Source: `kernel/src/sampling.rs`.

### Data structures

```
SamplingPort<const M: usize>
    id:              usize           // Port ID (pool index)
    direction:       PortDirection   // Source or Destination
    max_size:        usize           // = M (compile-time max message bytes)
    refresh_period:  u32             // Ticks; validity window
    data:            [u8; M]         // Message buffer
    current_size:    usize           // Actual size of last written message
    timestamp:       u64             // Tick at which last write occurred
    connected_port:  Option<usize>   // Destination port ID (routing)

SamplingPortPool<const S: usize, const M: usize>
    ports: heapless::Vec<SamplingPort<M>, S>
```

`PortDirection` enum: `Source`, `Destination`.

`Validity` enum: `Valid`, `Invalid`.

### Error enum

```
SamplingError { PoolFull, DirectionViolation, MessageTooLarge, InvalidPort }
```

### Operations

**write_sampling_message(port_id, data, timestamp)**
- Validates port exists, direction is `Source`, data fits in `M`.
- Copies data into the source port's buffer, updates `current_size`
  and `timestamp`.
- If a connected destination port exists, copies data there too.
- Non-blocking. No wait queues.

**read_sampling_message(port_id, buf, current_tick) -> (size, Validity)**
- Validates port exists, direction is `Destination`.
- Copies current data into caller's buffer.
- Computes validity: `Invalid` if `current_size == 0` or
  `current_tick - timestamp > refresh_period`.
- Non-blocking.

**validity(current_time) -> Validity** (per-port method)
- `Invalid` if never written (`current_size == 0`).
- `Invalid` if `current_time - timestamp > refresh_period`.
- `Valid` otherwise.

### Usage pattern

```rust
let src = pool.create_port(PortDirection::Source, refresh_period)?;
let dst = pool.create_port(PortDirection::Destination, refresh_period)?;
pool.connect_ports(src, dst)?;
pool.write_sampling_message(src, &sensor_data, tick)?;       // Source partition
let (size, validity) = pool.read_sampling_message(dst, &mut buf, tick)?; // Dest partition
```

## 5. Queuing Ports

Source: `kernel/src/queuing.rs`.

### Data structures

```
QueuingPort<const D: usize, const M: usize, const W: usize>
    direction:      PortDirection              // Source or Destination
    buf:            heapless::Deque<(usize, [u8; M]), D>  // Ring buffer
    sender_wq:      TimedWaitQueue<W>          // Blocked senders
    receiver_wq:    TimedWaitQueue<W>          // Blocked receivers
    connected_port: Option<usize>              // Routing target

QueuingPortPool<const S: usize, const D: usize, const M: usize, const W: usize>
    ports: heapless::Vec<QueuingPort<D, M, W>, S>

QueuingPortStatus (#[repr(C)])
    nb_messages:      u32    // Current queue depth
    max_nb_messages:  u32    // = D
    max_message_size: u32    // = M
    direction:        u32    // 0 = Source, 1 = Destination
```

Const-generic parameters: `S` = max ports, `D` = queue depth (messages),
`M` = max message size (bytes), `W` = wait-queue capacity.

### Error enum

```
QueuingError {
    PoolFull, DirectionViolation, MessageTooLarge,
    QueueFull, QueueEmpty, WaitQueueFull, InvalidPort
}
```

### Ring buffer design

Each enqueued message is stored as a `(usize, [u8; M])` tuple: the
actual message length alongside a fixed-size buffer. This allows
variable-length messages up to `M` bytes while keeping the Deque
element size uniform.

FIFO ordering: `push_back` to enqueue, `pop_front` to dequeue.

### Operations

**send_queuing_message(caller, data, timeout_ticks, current_tick)**
- Direction must be `Source`; data must fit in `M`.
- If buffer has space: enqueue message, wake one blocked receiver
  (if any). Returns `Delivered { wake_receiver }`.
- If buffer full and `timeout_ticks == 0`: returns `Err(QueueFull)`.
- If buffer full and `timeout_ticks > 0`: blocks sender with
  `expiry = current_tick + timeout_ticks`. Returns
  `SenderBlocked { expiry_tick }`.

**receive_queuing_message(caller, buf, timeout_ticks, current_tick)**
- Direction must be `Destination`.
- If buffer has messages: dequeue oldest, copy into `buf`, wake one
  blocked sender (if any). Returns `Received { msg_len, wake_sender }`.
- If empty and `timeout_ticks == 0`: returns `Err(QueueEmpty)`.
- If empty and `timeout_ticks > 0`: blocks receiver with
  `expiry = current_tick + timeout_ticks`. Returns
  `ReceiverBlocked { expiry_tick }`.

**get_queuing_port_status(port_id) -> QueuingPortStatus**
- Returns `#[repr(C)]` struct with current depth, max depth, max message
  size, and direction.

**send_routed(src_id, caller, data, timeout_ticks, current_tick)**
- Validates source port direction is `Source`.
- Resolves `connected_port` to find destination.
- Enqueues into destination's buffer (same blocking semantics as
  `send_queuing_message`).

### Blocking semantics

Senders block on the destination port's `sender_wq`. Receivers block on
their own port's `receiver_wq`. Each wait queue entry is
`(partition_id: u8, expiry_tick: u64)`.

When a message is dequeued, the oldest blocked sender is woken. When a
message is enqueued, the oldest blocked receiver is woken. This is
strictly FIFO.

### Timeout behavior

- `timeout_ticks == 0`: non-blocking, returns error immediately.
- `timeout_ticks > 0`: blocks with `expiry = current_tick + timeout_ticks`.
  The SysTick handler sweeps wait queues each tick (see Section 9).
- `u64::MAX` expiry: used internally for indefinite waits (the simple
  `send`/`recv` methods).

### Outcome enums

```
SendQueuingOutcome { Delivered { wake_receiver: Option<u8> },
                     SenderBlocked { expiry_tick: u64 } }

RecvQueuingOutcome { Received { msg_len: usize, wake_sender: Option<u8> },
                     ReceiverBlocked { expiry_tick: u64 } }
```

## 6. Blackboards

Source: `kernel/src/blackboard.rs`.

### Data structures

```
Blackboard<const M: usize, const W: usize>
    id:           usize             // Board ID (pool index)
    data:         [u8; M]           // Single message buffer
    current_size: usize             // Actual data length
    is_empty:     bool              // True initially and after clear
    wait_queue:   TimedWaitQueue<W> // Readers blocked on empty board

BlackboardPool<const S: usize, const M: usize, const W: usize>
    boards: heapless::Vec<Blackboard<M, W>, S>
```

Const-generic parameters: `S` = max blackboards, `M` = max message size,
`W` = wait-queue capacity.

Unlike ports, blackboards have no direction. Any partition can display,
read, or clear any blackboard.

### Error enum

```
BlackboardError { PoolFull, MessageTooLarge, InvalidBoard, BoardEmpty, WaitQueueFull }
```

### Operations

**display(data) -> Vec<u8, W> (woken PIDs)**
- Validates `data.len() <= M`.
- Overwrites buffer, sets `is_empty = false`.
- Drains entire wait queue: wakes ALL blocked readers (broadcast).
- Returns list of woken partition IDs.

**read_timed(caller, buf, timeout, current_tick)**
- If board is not empty: copies data into `buf`. Returns
  `Read { msg_len }`.
- If empty and `timeout == 0`: returns `Err(BoardEmpty)`.
- If empty and `timeout > 0`: enqueues caller with
  `expiry = current_tick + timeout`. Returns `ReaderBlocked`.

**clear()**
- Sets `is_empty = true`, `current_size = 0`.
- Does NOT wake blocked readers. Readers remain blocked until a
  subsequent `display` or their timeout expires.

### Broadcast wakeup

Display wakes all waiting readers, not just one. This matches the
semantics of a shared state buffer: when new data arrives, every
interested reader should see it. The `drain_all` method on
`TimedWaitQueue` returns all PIDs in FIFO order.

### Outcome enum

```
ReadBlackboardOutcome { Read { msg_len: usize }, ReaderBlocked }
```

## 7. Port Connection Routing

Both sampling ports and queuing ports use a `connected_port: Option<usize>`
field to implement one-to-one source-to-destination routing.

### Configuration

Both `SamplingPortPool` and `QueuingPortPool` expose
`connect_ports(src_id, dst_id)` which validates that `src_id` is a
`Source` port and `dst_id` is a `Destination` port. Returns
`DirectionViolation` or `InvalidPort` on error.

### Runtime routing

**Sampling**: `write_sampling_message` on a source port copies data
directly to the connected destination port within the same syscall.

**Queuing**: `send_routed` on a source port pushes the message into the
connected destination port's ring buffer. If the destination's buffer is
full and the caller specified a nonzero timeout, the caller blocks.

### Data flow

Sampling: `SamplingWrite(src)` -> kernel copies to dst ->
`SamplingRead(dst)` returns data + validity.

Queuing: `QueuingSend(src)` -> kernel enqueues in dst ->
`QueuingRecv(dst)` dequeues and returns data.

## 8. Blocking and Wakeup Protocol

### Wait queue types

Two wait queue types in `kernel/src/waitqueue.rs`:

- **WaitQueue\<W\>**: FIFO queue of `u8` partition IDs. Used by
  semaphores, mutexes, and message queues (no timeout support).
- **TimedWaitQueue\<W\>**: FIFO queue of `(u8, u64)` pairs
  (partition ID, expiry tick). Used by queuing ports and blackboards.

### Blocking flow

1. Pool operation pushes `(pid, expiry)` onto the resource's wait queue.
2. Returns an outcome enum indicating the caller should block.
3. SVC dispatch calls `try_transition(partitions, pid, Waiting)`.
4. Partition state becomes `Waiting`.

In the ARINC 653-style static scheduler (see
[architecture.md](architecture.md#4-static-schedule-table-format)),
blocking does NOT skip the partition's time slot. The partition still
receives its window but cannot do useful work.

### Wakeup flow

When a complementary operation occurs (e.g., send delivers to a queue
with blocked receivers):

1. Pool operation pops the front entry from the wait queue.
2. Returns an outcome indicating the woken partition.
3. SVC dispatch calls `try_transition(partitions, pid, Ready)`.

For blackboard `display`: all blocked readers are woken (broadcast).

### Concurrency model

All SVC operations execute in Handler mode (SVCall priority). On
single-core Cortex-M, SVC operations are non-preemptible by application
code and by SysTick (which runs at lower priority). No additional
synchronization primitives are needed.

## 9. Timeout Enforcement

### Mechanism

On each SysTick interrupt (see
[architecture.md](architecture.md#4-static-schedule-table-format) for
the SysTick handler), after advancing the schedule table, the kernel
sweeps all timed wait queues for expired entries.

Both `QueuingPortPool` and `BlackboardPool` provide a `tick_timeouts`
method that iterates over all ports/boards, calling `drain_expired` on
each `TimedWaitQueue`.

### drain_expired algorithm

For each entry: pop from front. If `current_tick >= expiry`, the entry
is expired (PID added to output). Otherwise, re-enqueue at back. This
preserves FIFO ordering of non-expired entries.

### Expiry computation

```
expiry = current_tick + timeout_ticks
```

- `timeout_ticks == 0` means non-blocking (never enqueued).
- `u64::MAX` means indefinite wait (never expires via sweep).
- The `u64` tick counter wraps after ~5.8 billion years at 10ms/tick.

### Kernel integration

```rust
// In SysTick handler:
let expired_queuing: Vec<u8, E> = kernel.queuing.tick_timeouts(current_tick);
let expired_bb: Vec<u8, E> = kernel.blackboards.tick_timeouts(current_tick);
// Transition each expired PID: Waiting -> Ready
```

Expired partitions are transitioned to `Ready` so they can be scheduled
in their next time slot.
