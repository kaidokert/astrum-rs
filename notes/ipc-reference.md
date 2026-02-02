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
- [10. Buffer Pool](#10-buffer-pool)
- [11. Virtual Devices](#11-virtual-devices)

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

Total: 19 base syscalls spanning numbers 0-19 (number 1 is reserved and
returns `InvalidSyscall`). When the `dynamic-mpu` feature is enabled, 7
additional syscalls (20-26) bring the total to 26. The first invalid
number is 20 (without `dynamic-mpu`) or 27 (with it); all invalid
numbers return `SvcError::InvalidSyscall`.

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
| BufferAlloc      | mode (0=R,1=W) | —            | —           | slot index or error |
| BufferRelease    | slot_index     | —            | —           | 0 or error          |
| BufferWrite      | slot_index     | data_len     | data_ptr    | bytes written or err|
| DevOpen          | device_id      | —            | —           | 0 or error          |
| DevRead          | device_id      | buf_len      | buf_ptr     | byte count or error |
| DevWrite         | device_id      | data_len     | data_ptr    | byte count or error |
| DevIoctl         | device_id      | cmd          | arg         | result or error     |

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

## 10. Buffer Pool

Source: `kernel/src/buffer_pool.rs`. Requires `dynamic-mpu` feature.
For the MPU region strategy (DynamicStrategy, R4-R7 slot tracking) and
context-switch integration, see
[architecture.md §11.1](architecture.md#111-mpustrategy-trait-and-dynamicstrategy)
and [§11.2](architecture.md#112-buffer-pool-memory-model).

### Data structures

```
BufferSlot<const SIZE: usize>    (#[repr(C, align(32))])
    data:       [u8; SIZE]       // Fixed-size byte buffer (32-byte aligned)
    state:      BorrowState      // Free | BorrowedRead{owner} | BorrowedWrite{owner}
    mpu_region: Option<u8>       // MPU region ID (5-7) set by lend_to_partition

BufferPool<const SLOTS: usize, const SIZE: usize>
    slots: [BufferSlot<SIZE>; SLOTS]
```

`KernelConfig` provides two associated constants: `BP` (slot count =
`SLOTS`) and `BZ` (slot size in bytes = `SIZE`). The kernel struct
holds a `BufferPool<{C::BP}, {C::BZ}>`.

### Borrow state machine

```
Free --alloc(pid,Read)--> BorrowedRead{owner}
Free --alloc(pid,Write)--> BorrowedWrite{owner}
BorrowedRead{owner} --release(owner)--> Free
BorrowedWrite{owner} --release(owner)--> Free
```

Only the owning partition can release a slot. Attempting to borrow an
already-borrowed slot returns `AlreadyBorrowed`. Releasing a free slot
returns `NotBorrowed`.

### Error enums

```
BufferPoolError { InvalidSlot, NotBorrowed, AlreadyBorrowed, NotOwner }
BufferError     { InvalidSlot, SlotNotFree, SlotNotBorrowed, MpuWindowExhausted }
```

`BufferPoolError` is used by `alloc`/`borrow`/`release` (pure
ownership operations). `BufferError` is used by `lend_to_partition`
and `revoke_from_partition` (MPU-aware operations).

### Syscall semantics

**SYS_BUF_ALLOC (20)** — `r1` = borrow mode (0 = Read, 1 = Write).
Scans for the first free slot and transitions it to the requested
borrow state with the calling partition as owner. Returns the slot
index on success, or `SvcError::OperationFailed` if no free slots.

**SYS_BUF_RELEASE (21)** — `r1` = slot index. Releases the slot back
to `Free` state. The caller must be the current owner; otherwise
returns `SvcError::InvalidResource`.

**SYS_BUF_WRITE (26)** — `r1` = slot index, `r2` = data length,
`r3` = source data pointer (validated). Copies `r2` bytes from the
user buffer into the slot's data array. The slot must be in
`BorrowedWrite` state owned by the caller. If `r2` exceeds the
slot's `SIZE`, the write is rejected entirely (no partial write) and
returns `SvcError::OperationFailed`. On success, returns the number
of bytes written (always equal to `r2`). Returns
`SvcError::InvalidResource` if the slot index is invalid or the
caller is not the write-owner.

### Zero-copy lending model

For cross-partition buffer sharing, the pool coordinates with
`DynamicStrategy` (from `mpu_strategy.rs`) to install MPU windows:

**lend_to_partition(slot, partition_id, writable, strategy)** — Slot
must be `Free`. Computes the slot's physical base address and builds
an MPU RASR value with `AP_RO_RO` (read-only) or `AP_FULL_ACCESS`
(writable). Calls `strategy.add_window()` to claim a dynamic MPU
region (R5-R7). On success, transitions the slot to `BorrowedRead`
or `BorrowedWrite` and records the MPU region ID. Returns the region
ID (5-7) or `BufferError::MpuWindowExhausted` if all 3 dynamic
window slots are occupied.

**revoke_from_partition(slot, strategy)** — Removes the MPU window
via `strategy.remove_window(region_id)` and returns the slot to
`Free`. Returns `SlotNotBorrowed` if the slot has no MPU region
assigned.

### MPU window integration

The `BufferSlot` struct is `#[repr(C, align(32))]` so that the `data`
field (placed first) is always 32-byte aligned — the minimum MPU
region alignment for sizes >= 32. The `SIZE` const generic must be a
power-of-two >= 32 for MPU compatibility.

Dynamic MPU regions R5-R7 are managed by `DynamicStrategy`. Static
partition regions (R0-R3: code, data, background, stack guard) are
never disturbed by buffer lending. At most 3 buffer slots can be
simultaneously lent across the system.

## 11. Virtual Devices

Source: `kernel/src/virtual_device.rs` and `kernel/src/virtual_uart.rs`.
Requires `dynamic-mpu` feature. For the system-window schedule entries
that drive bottom-half processing (UART transfer, ISR drain), see
[architecture.md §11.3](architecture.md#113-system-window-schedule-entries-and-bottom-half-processing)
and [§11.5](architecture.md#115-virtual-device-abstraction-layer).

### VirtualDevice trait

```rust
pub trait VirtualDevice {
    fn device_id(&self) -> u8;
    fn open(&mut self, partition_id: u8)  -> Result<(), DeviceError>;
    fn close(&mut self, partition_id: u8) -> Result<(), DeviceError>;
    fn read(&mut self, partition_id: u8, buf: &mut [u8])  -> Result<usize, DeviceError>;
    fn write(&mut self, partition_id: u8, data: &[u8])    -> Result<usize, DeviceError>;
    fn ioctl(&mut self, partition_id: u8, cmd: u32, arg: u32) -> Result<u32, DeviceError>;
}
```

Every method takes an explicit `partition_id` for access control. The
kernel supplies `current_partition` — user code never provides its
own PID.

### DeviceError enum

```
DeviceError { NotFound, PermissionDenied, NotOpen, BufferFull, BufferEmpty, InvalidPartition }
```

### DeviceRegistry

`DeviceRegistry<'a, const N: usize>` is a fixed-capacity array of
`&mut dyn VirtualDevice` references with O(n) lookup by device ID.
`add()` panics on duplicate IDs or full registry (these are
configuration errors at init time). `get_mut(id)` returns a mutable
trait object or `None`.

### VirtualUartBackend

Source: `kernel/src/virtual_uart.rs`.

```
VirtualUartBackend
    device_id:       u8
    tx:              Deque<u8, 64>    // Partition writes here
    rx:              Deque<u8, 64>    // Partition reads from here
    open_partitions: u8               // Bitmask (supports partitions 0-7)
    loopback_peer:   Option<u8>       // Peer device ID for loopback wiring
```

Ring buffer capacity is 64 bytes (`VirtualUartBackend::CAPACITY`).

**Open/close**: Sets/clears a bit in `open_partitions`. Multiple
partitions can open the same UART simultaneously (bitmask, not
exclusive). Partition IDs >= 8 return `InvalidPartition`.

**Write** (via trait): Pushes bytes into the TX ring buffer. Returns
the number of bytes enqueued (may be less than requested if the buffer
is nearly full). Requires the partition to have opened the device.

**Read** (via trait): Drains bytes from the RX ring buffer into the
caller's buffer. Returns the number of bytes read (may be less than
buffer size if fewer bytes are available).

**IOCTL commands**:

| Constant          | Value | Behavior                                |
|-------------------|-------|-----------------------------------------|
| `IOCTL_FLUSH`     | 0x01  | Drain all bytes from TX ring buffer     |
| `IOCTL_AVAILABLE` | 0x02  | Return byte count in RX ring buffer     |
| `IOCTL_SET_PEER`  | 0x03  | Set loopback peer device ID (`arg`)     |

Unknown IOCTL commands return `DeviceError::NotFound`.

### VirtualUartPair and loopback model

`VirtualUartPair` holds two `VirtualUartBackend` instances (fields
`a` and `b`) with their loopback peers cross-wired at construction.

**transfer()** performs the bottom-half routing: drains UART-A's TX
into UART-B's RX, then drains UART-B's TX into UART-A's RX. Returns
`(a_to_b, b_to_a)` byte counts. If the destination's RX buffer is
full, untransferred bytes remain in the source's TX buffer for a
later `transfer()` call.

In the kernel's system-window tick handler, `transfer()` is called
during bottom-half processing to move data between the two UART
endpoints.

### Syscall semantics

**SYS_DEV_OPEN (22)** — `r1` = device ID. Looks up the device via
`dev_dispatch`, which calls `self.uart_pair.get_mut(device_id)` on the
kernel's `VirtualUartPair`, and then calls `open(current_partition)`.
Returns 0 on success.

**SYS_DEV_READ (23)** — `r1` = device ID, `r2` = buffer length,
`r3` = buffer pointer (validated). Calls `read(pid, buf)` on the
device. Returns byte count read.

**SYS_DEV_WRITE (24)** — `r1` = device ID, `r2` = data length,
`r3` = data pointer (validated). Calls `write(pid, data)` on the
device. Returns byte count written.

**SYS_DEV_IOCTL (25)** — `r1` = device ID, `r2` = command,
`r3` = argument. Calls `ioctl(pid, cmd, arg)` on the device. Returns
the IOCTL result value. No pointer validation (r2/r3 are opaque
integers).

All device syscalls use `dev_dispatch` in svc.rs, which resolves the
device ID from `r1`, injects `current_partition` as the PID, and maps
`DeviceError` variants to `SvcError` codes.
