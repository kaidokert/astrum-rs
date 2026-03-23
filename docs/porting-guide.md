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
