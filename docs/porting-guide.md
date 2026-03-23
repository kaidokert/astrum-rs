# Porting Guide

Reference for hardware integrators bringing up new boards.

## Boot Error Visibility

When bringing up a new board, configuration mistakes (wrong entry point,
misaligned stack, invalid MPU region) must be visible *before* the kernel
takes over exception handling. This section describes the guarantees the
boot path provides and how to observe errors.

### RTT Initializes Before `init_kernel()`

The `define_unified_harness!` macro calls `init_rtt()` as the first
operation inside `init_kernel()`, before any partition or kernel
configuration runs. This ensures that `klog!` output is routed to
an RTT channel before validation occurs — so if `Kernel::new()` or
`ExternalPartitionMemory::new()` returns a `ConfigError`, the
corresponding `klog!("init_kernel failed: {:?}", e)` message is
already being captured.

```text
init_kernel()
  ├── init_rtt()          ← RTT channel available
  ├── ExternalPartitionMemory::new(...)
  │     └── on error → klog! + return BootError::KernelInit(ConfigError)
  └── Kernel::new(...)
        └── on error → klog! + return BootError::KernelInit(ConfigError)
```

### `BootError::KernelInit(ConfigError)` Error Chain

`BootError::KernelInit` wraps `ConfigError` without discarding any detail.
`ConfigError` variants carry enough context to identify the exact problem:

| `ConfigError` variant | What it tells you |
|---|---|
| `EntryPointNotThumb { partition_id, entry_point }` | Entry point address missing the Thumb bit |
| `EntryPointOutsideCodeRegion { partition_id, entry_point, region_base, region_size }` | Entry point falls outside the partition's code region |
| `StackBaseNotAligned { partition_id }` | Stack base violates alignment requirements |
| `StackSizeInvalid { partition_id }` | Stack size is zero or not a power of two |
| `MpuRegionInvalid { partition_id, detail }` | MPU region failed hardware constraints (`detail` is an `MpuError`) |
| `ScheduleIndexOutOfBounds { entry_index, partition_index, num_partitions }` | Schedule references a nonexistent partition |
| `PeripheralRegionInvalid { partition_id, region_index, detail }` | Peripheral MPU region invalid |

All variants implement `Debug`, so the `klog!` output includes the full
variant name and all fields.

### Reading Boot Errors via RTT

Connect a debug probe and open an RTT viewer before powering the board:

```sh
# Terminal 1 — start OpenOCD (adapt interface/target to your probe)
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg

# Terminal 2 — attach RTT viewer
# Option A: probe-rs
probe-rs attach --chip STM32F407VGTx --protocol swd

# Option B: JLinkRTTClient / telnet to OpenOCD RTT port
```

When `init_kernel()` hits a `ConfigError`, you will see a line like:

```
init_kernel failed: EntryPointNotThumb { partition_id: 0, entry_point: 0x08004000 }
```

### Reading Boot Errors via GDB

If RTT is not available or you need to inspect state after a panic:

```sh
# Connect GDB to OpenOCD
arm-none-eabi-gdb -ex "target remote :3333" target/thumbv7em-none-eabihf/debug/my-app
```

Useful commands at the fault point:

```
(gdb) bt                    # backtrace through the panic handler
(gdb) info locals           # inspect ConfigError / BootError on the stack
(gdb) p/x *0xE000ED28      # CFSR — shows MemManage / BusFault / UsageFault bits
(gdb) p/x *0xE000ED34      # MMFAR — faulting address if MemManage
```

If the board hard-faults before RTT output appears, set a breakpoint on
the error path:

```
(gdb) break init_kernel     # step through init_kernel
(gdb) break rust_begin_unwind  # catch panics
```

### `init_rtt()` Double-Call Guard

`init_rtt()` uses a static `AtomicBool` (`RTT_INITIALIZED`) so that the
second and subsequent calls are no-ops. This is safe to rely on in custom
boot sequences where you might call `init_rtt()` explicitly before the
harness macro calls it again:

```rust
// Custom main — calling init_rtt() early is fine
#[entry]
fn main() -> ! {
    init_rtt();          // first call: initializes RTT
    // ... early debug logging ...
    init_kernel(...);    // macro calls init_rtt() again internally — no-op
    boot();
}
```

The guard uses `Ordering::Relaxed` because boot runs single-threaded
before the scheduler starts.

## FPU Context Switching (`fpu-context` Feature)

When the `fpu-context` Cargo feature is enabled, the kernel saves and
restores FPU callee-saved registers (s16–s31) on every context switch.
This adds significant stack overhead to each partition.

### Stack Overhead Breakdown

| Component | Size | Who saves | When |
|---|---|---|---|
| s16–s31 (callee-saved) | 64 bytes (16 × 4) | PendSV software (`vstmdb`/`vldmia`) | Every context switch |
| s0–s15 + FPSCR + reserved (caller-saved) | 72 bytes (18 × 4) | Hardware (lazy stacking) | On exception entry when FPU was used |
| **Total FPU overhead** | **~136 bytes** | | |

The 64-byte callee-saved area is always present on the stack when a
partition is preempted. The 72-byte hardware frame is pushed lazily by
the processor when the FPU was active before the exception — on
Cortex-M4F this is controlled by FPCCR LSPEN/ASPEN (verified at boot by
`init_fpu()`).

Combined with the base 32-byte integer exception frame (r0–r3, r12, LR,
PC, xPSR) and 32-byte software-saved context (r4–r11), the worst-case
context frame is **200 bytes** per preemption level.

### Minimum Stack Sizing

Without `fpu-context`, the default 1 KB partition stack is sufficient for
most workloads. With `fpu-context` enabled:

- **Minimum recommended stack: 2 KB** per partition.
- Partitions performing deep call chains or using large local arrays
  should increase further.
- The kernel verifies stack pointer alignment at context switch time and
  stores a sentinel for overflow detection, but undersized stacks will
  cause silent corruption before the sentinel is reached.

### EXC_RETURN Values

The kernel uses different EXC_RETURN values depending on FPU state:

| Constant | Value | Meaning |
|---|---|---|
| `EXC_RETURN_THREAD_PSP` | `0xFFFF_FFFD` | Return to Thread mode, PSP, no FPU frame |
| `EXC_RETURN_THREAD_PSP_FPU` | `0xFFFF_FFED` | Return to Thread mode, PSP, extended (FPU) frame |

When `fpu-context` is enabled, the PendSV handler preserves the
`EXC_RETURN` value from `LR` at exception entry and restores it on
return. The hardware sets bit 4 of `EXC_RETURN` to indicate whether
the interrupted context used a standard (bit 4 = 1) or extended/FPU
(bit 4 = 0) frame. The handler must **not** force
`EXC_RETURN_THREAD_PSP_FPU` unconditionally — doing so on a task
whose exception frame is standard (no FPU usage) would cause the
hardware to pop 72 bytes of non-existent FPU state, corrupting the
stack.
