# plib Public API Surface

## Motivation

**ABI isolation.** Partitions run unprivileged on PSP. All kernel interaction
goes through `svc #0`. `plib` encapsulates this: argument packing, pointer
casting, and error decoding happen once behind safe Rust wrappers.

**Ergonomics.** Wrappers convert slices to `(ptr, len)`, map `bool` to mode
flags, pack timed-variant fields, and decode `u32` → `Result<u32, SvcError>`.

**Single source of truth.** Syscall numbers live in `rtos-traits`; `plib`
re-exports them. Partitions should never import `rtos-traits` directly.

## API Inventory (40 functions + 4 macros)

### Always available (22 functions)

| Function | Syscall | # | r1 | r2 | r3 | Return |
|---|---|---|---|---|---|---|
| `sys_yield()` | YIELD | 0 | 0 | 0 | 0 | `Ok(0)` |
| `sys_get_partition_id()` | GET_PARTITION_ID | 1 | 0 | 0 | 0 | `Ok(id)` |
| `sys_get_time()` | GET_TIME | 11 | 0 | 0 | 0 | `Ok(ticks)` |
| `sys_irq_ack(irq: u8)` | IRQ_ACK | 38 | irq | 0 | 0 | `Ok(0)` |
| `sys_event_wait(mask: u32)` | EVT_WAIT | 2 | mask | 0 | 0 | `Ok(bits)` |
| `sys_event_set(tgt: u32, mask: u32)` | EVT_SET | 3 | tgt | mask | 0 | `Ok(0)` |
| `sys_event_clear(mask: u32)` | EVT_CLEAR | 4 | mask | 0 | 0 | `Ok(prev)` |
| `sys_sem_wait(id: u32)` | SEM_WAIT | 5 | id | 0 | 0 | `Ok(0)` |
| `sys_sem_signal(id: u32)` | SEM_SIGNAL | 6 | id | 0 | 0 | `Ok(0)` |
| `sys_mtx_lock(id: u32)` | MTX_LOCK | 7 | id | 0 | 0 | `Ok(0)` |
| `sys_mtx_unlock(id: u32)` | MTX_UNLOCK | 8 | id | 0 | 0 | `Ok(0)` |
| `sys_msg_send(tgt: u32, data: &[u8])` | MSG_SEND | 9 | tgt | len | ptr | `Ok(0)` |
| `sys_msg_recv(buf: &mut [u8])` | MSG_RECV | 10 | 0 | len | ptr | `Ok(n)` |
| `sys_sampling_write(port: u32, data: &[u8])` | SAMPLING_WRITE | 12 | port | len | ptr | `Ok(0)` |
| `sys_sampling_read(port: u32, buf: &mut [u8])` | SAMPLING_READ | 13 | port | len | ptr | `Ok(n)` |
| `sys_queuing_send(port: u32, data: &[u8])` | QUEUING_SEND | 14 | port | len | ptr | `Ok(0)` |
| `sys_queuing_recv(port: u32, buf: &mut [u8])` | QUEUING_RECV | 15 | port | len | ptr | `Ok(n)` |
| `sys_queuing_status(port: u32)` | QUEUING_STATUS | 16 | port | 0 | 0 | `Ok(st)` |
| `sys_queuing_send_timed(port: u32, data: &[u8], t: u16)` | QUEUING_SEND_TIMED | 27 | port | packed | ptr | `Ok(0)` |
| `sys_queuing_recv_timed(port: u32, buf: &mut [u8], t: u16)` | QUEUING_RECV_TIMED | 28 | port | packed | ptr | `Ok(n)` |
| `sys_debug_print(msg: &[u8])` | DEBUG_PRINT | 31 | ptr | len | 0 | `Ok(0)` |
| `sys_debug_exit(code: u32)` | DEBUG_EXIT | 32 | code | 0 | 0 | `Ok(0)` |

### `ipc-blackboard` (3 functions)

| Function | Syscall | # | r1 | r2 | r3 | Return |
|---|---|---|---|---|---|---|
| `sys_bb_display(id: u32, data: &[u8])` | BB_DISPLAY | 17 | id | len | ptr | `Ok(0)` |
| `sys_bb_read(id: u32, buf: &mut [u8])` | BB_READ | 18 | id | len | ptr | `Ok(n)` |
| `sys_bb_clear(id: u32)` | BB_CLEAR | 19 | id | 0 | 0 | `Ok(0)` |

### Devices (7 functions)

| Function | Syscall | # | r1 | r2 | r3 | Return |
|---|---|---|---|---|---|---|
| `sys_dev_open(dev: u8)` | DEV_OPEN | 22 | dev | 0 | 0 | `Ok(0)` |
| `sys_dev_close(dev: u8)` | DEV_CLOSE | 29 | dev | 0 | 0 | `Ok(0)` |
| `sys_dev_ioctl(dev: u8, cmd: u32, arg: u32)` | DEV_IOCTL | 25 | dev | cmd | arg | `Ok(val)` |
| `sys_dev_read(dev: u8, buf: &mut [u8])` | DEV_READ | 23 | dev | len | ptr | `Ok(n)` |
| `sys_dev_write(dev: u8, data: &[u8])` | DEV_WRITE | 24 | dev | len | ptr | `Ok(0)` |
| `sys_dev_read_timed(dev: u8, buf: &mut [u8], t: u16)` | DEV_READ_TIMED | 30 | dev | packed | ptr | `Ok(n)` |
| `sys_query_bottom_half(dev: u8)` | QUERY_BOTTOM_HALF | 33 | dev | 0 | 0 | `Ok(st)` |

### Buffer pool (7 functions)

| Function | Syscall | # | r1 | r2 | r3 | Return |
|---|---|---|---|---|---|---|
| `sys_buf_alloc(writable: bool, ticks: u16)` | BUF_ALLOC | 20 | mode | ticks | 0 | `Ok(slot)` |
| `sys_buf_release(slot: u8)` | BUF_RELEASE | 21 | slot | 0 | 0 | `Ok(0)` |
| `sys_buf_read(slot: u8, dst: &mut [u8])` | BUF_READ | 37 | slot | len | ptr | `Ok(n)` |
| `sys_buf_write(slot: u8, data: &[u8])` | BUF_WRITE | 26 | slot | len | ptr | `Ok(n)` |
| `sys_buf_lend(slot: u8, tgt: u8, w: bool)` | BUF_LEND | 34 | slot | packed | 0 | `Ok((base_addr, rgn))` |
| `sys_buf_revoke(slot: u8, tgt: u8)` | BUF_REVOKE | 35 | slot | tgt | 0 | `Ok(0)` |
| `sys_buf_transfer(slot: u8, owner: u8)` | BUF_TRANSFER | 36 | slot | owner | 0 | `Ok(0)` |

### `partition-debug` (1 function + 4 macros)

- `debug_write<N>(buf, level, msg)` → `SYS_DEBUG_NOTIFY` (#64)
- `define_partition_debug!(NAME, SIZE)` — declare static ring buffer
- `dprint!`, `debug_warn!`, `debug_error!` — formatted writes via `debug_write`

## ABI Conventions

**SVC invocation:** `svc #0` with r0=syscall#, r1–r3=args. Kernel reads
from hardware exception frame on PSP, writes result to r0, returns via
`EXC_RETURN_THREAD_PSP` (`0xFFFF_FFFD`). r4–r11 preserved; r12 clobbered.

**Buffer-passing convention:** Syscalls with a resource identifier (port, slot,
dev, tgt) place it in r1, then r2=len, r3=ptr. `sys_debug_print` has no
resource id, so it uses r1=ptr, r2=len, r3=0 — no special-case logic needed,
the kernel SVC handler reads the same frame offsets for both patterns.

**Error bit:** `r0 & 0x8000_0000 != 0` → error. `decode_rc()` splits this.

**Error codes** (all have bit 31 set; values count down from `0xFFFF_FFFF`):

| Code | Value |
|---|---|
| InvalidSyscall | `0xFFFF_FFFF` |
| InvalidResource | `0xFFFF_FFFE` |
| WaitQueueFull | `0xFFFF_FFFD` |
| TransitionFailed | `0xFFFF_FFFC` |
| InvalidPartition | `0xFFFF_FFFB` |
| OperationFailed | `0xFFFF_FFFA` |
| InvalidPointer | `0xFFFF_FFF9` |
| NotImplemented | `0xFFFF_FFF8` |
| BufferFull | `0xFFFF_FFF7` |
| NotSupported | `0xFFFF_FFF6` |
| PermissionDenied | `0xFFFF_FFF5` |

**Packed r2 (timed variants):** `r2 = (timeout_ticks << 16) | length`.
Both u16; max buffer 65 535 bytes. Wrappers pre-reject `len > u16::MAX`.

**Buffer pool length limit:** `buf_read`/`buf_write` also pre-reject
`len > u16::MAX` at the wrapper level for consistency with buffer pool slot
sizes, even though their r2 carries an unpacked length.

**Packed r2 (buf lend):** `r2 = (target as u32) | flags`. Bit 8 =
`lend_flags::WRITABLE` (`0x100`).

## Feature Flag Matrix

| Feature | Cargo flag | Propagates to | Gated count |
|---|---|---|---|
| Partition debug | `partition-debug` | `rtos-traits/partition-debug` | 2 syscalls + 4 macros + types |
| Dynamic MPU | *(always enabled)* | — | 14 syscalls + `lend_flags` |
| Blackboard IPC | `ipc-blackboard` | `kernel/ipc-blackboard` | 3 syscalls |

## Testing Strategy

**Host stubs:** `svc!` compiles to `0u32` on non-ARM. `cargo test` exercises
packing, validation, and error decoding without QEMU.

**Re-export verification:** Tests assert every re-exported `SYS_*` constant
equals its `rtos_traits::syscall` source, catching shadowing/drift.

**Boundary tests:** Timed variants and `buf_read`/`buf_write` reject
`len > u16::MAX` pre-syscall. Tests cover exact boundary (`u16::MAX` accepted,
`u16::MAX + 1` rejected). Lend packing tested for targets 0, 1, 0xFF with
both writable and read-only modes.

**Feature-gated sets:** Run with `--features ipc-blackboard,partition-debug`.

**On-target:** Register-level ABI verification requires QEMU integration tests
(tracked as TODOs in `plib/src/lib.rs`).
