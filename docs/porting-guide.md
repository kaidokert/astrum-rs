# Porting Guide

Reference for hardware integrators bringing up new boards.

## Boot Error Visibility

When bringing up a new board, configuration mistakes (wrong entry point,
misaligned stack, invalid MPU region) must be visible *before* the kernel
takes over exception handling. This section describes the guarantees the
boot path provides and how to observe errors.

### RTT Initializes Before `init_kernel()`

The `define_harness!` macro calls `init_rtt()` as the first
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

### Pre-Init `rprintln!` Hazard

**Warning:** Calling `rprintln!` before `init_rtt()` corrupts RAM and
causes silent reset loops. The RTT control block is a static structure
in SRAM; writing to it before `rtt_target::rtt_init_print!()` has
initialized it overwrites arbitrary memory, which typically triggers a
HardFault or data corruption that leads to an infinite boot loop with
no diagnostic output.

This bug was first identified during a hardware team BME280 I2C
bring-up where early `rprintln!` debug prints (added before the harness
macro ran) caused the board to silently reset on every boot. The
failure was particularly difficult to diagnose because of stale RTT
control block masking: if a previous run successfully initialized RTT,
the control block magic bytes may persist in SRAM across a soft reset.
The stale control block makes `rprintln!` appear to work on the *first*
boot after a successful run, but fail on cold boot or after a power
cycle — making the bug non-reproducible across examples and debug
sessions.

**`klog!` is safe before `init_rtt()`** — the `klog!` macro checks
`is_rtt_initialized()` and silently no-ops if RTT has not been set up
yet. All kernel-internal logging uses `klog!`, so the kernel itself
will never trigger this bug.

**Direct `rprintln!` in partition code is the caller's responsibility.**
The kernel cannot guard against user code calling `rprintln!` directly.
If your partition code uses `rprintln!` for debug output, you must
ensure that `init_rtt()` has already been called (which `define_harness!`
/ `init_kernel()` guarantees before any partition runs). The hazard
arises only when `rprintln!` is called *outside* the harness — for
example, in a custom `main()` before `init_kernel()`, or in a
`#[pre_init]` function.

**Safe pattern — use `klog!` or guard manually:**

```rust
// SAFE: klog! is always safe, even before init_rtt()
klog!("early debug: x = {}", x);

// SAFE: explicit guard before rprintln!
if kernel::is_rtt_initialized() {
    rprintln!("debug: x = {}", x);
}

// UNSAFE: rprintln! before init_rtt() — will corrupt RAM
rprintln!("this will crash if RTT is not initialized");
```

## MPU Enforcement and Build Profiles

When `mpu_enforce = true`, the PendSV handler reprograms MPU regions on
every context switch. In debug builds (`opt-level = 0`), this region
programming is significantly slower because the compiler does not inline
or optimize the register-write sequences. With the default 1 ms SysTick
period, PendSV can take longer than one tick to complete.

### Failure Mode

When PendSV exceeds the tick period, SysTick fires before the context
switch finishes. The system enters a state where:

1. PendSV never fully completes a context switch.
2. No partition code runs — the processor bounces between PendSV and
   SysTick indefinitely.
3. RTT output stops (no partition or kernel log code executes), so the
   system appears completely locked with no diagnostic output.

This failure is silent and looks identical to a hard hang. There is no
fault, no panic, and no RTT message — just a dead board.

### Runtime Warning

The kernel emits a `klog!` warning at boot when it detects this risky
combination (`debug_assertions` enabled and `mpu_enforce = true`):

```
WARNING: MPU enforcement with debug build — PendSV may be too slow for 1ms tick. Use --release or increase tick period.
```

This warning is printed by `warn_mpu_debug_build()`, which runs early in
the boot sequence before the scheduler starts. If you see this message
and then the system locks up, the build profile is almost certainly the
cause.

### Recommended Build Command

Always compile with `--release` (or at least `opt-level >= 1`) when MPU
enforcement is enabled:

```sh
cargo build --release --example <name>
```

Custom Cargo profiles with `opt-level = 1` also work:

```toml
# Cargo.toml
[profile.release-debug]
inherits = "release"
opt-level = 1
debug = true
```

```sh
cargo build --profile release-debug --example <name>
```

The key requirement is that the compiler optimizes the MPU register
writes enough to keep PendSV under the tick period. `opt-level = 0`
(the default for `cargo build` without `--release`) does not meet this
requirement on any tested target.

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

## Interrupt Latency Constraints

The Split-ISR model routes device interrupts through the cyclic
scheduler: the top-half runs immediately in Handler mode, but the
bottom-half executes only when the owning partition's next slot arrives.
This means **bottom-half latency is bounded by the major frame period
`T = N × S`** (number of slots × slot duration). In the worst case a
partition waits a full major frame before it can service the event.

### Checking Device Frequency Against Your Schedule

Before assigning a device to a Split-ISR partition, verify that the
device's interrupt frequency does not exceed the schedule's fundamental
frequency ceiling of `1/T` Hz. For example, a 5-slot × 2 ms schedule
gives `T = 10 ms` and a ceiling of 100 Hz — any source firing faster
than 100 Hz will lose events (Model B) or accumulate unbounded pending
bits (Model A).

Compare your device's expected interrupt rate against your configured
`T`. If the rate approaches or exceeds `1/T`, the Split-ISR bottom-half
path is not appropriate for that device.

### Devices That May Not Suit Split-ISR

High-frequency or hard-deadline devices — motor control (FOC at
10–50 kHz), power converter switching (> 50 kHz), or continuous ADC
streaming (1–10 kHz) — typically cannot tolerate millisecond-scale
scheduling delay. These devices require one of:

- **Kernel top-half handling**: the entire interrupt is serviced in
  Handler mode with no bottom-half. Suitable for moderate rates
  (up to ~10 kHz) where processing time is short.
- **A dedicated ISR outside the Split-ISR model**: the device's handler
  runs directly via the NVIC with no kernel involvement. Required when
  sub-microsecond response is needed.

For a detailed breakdown of latency bounds (best/worst/average case),
the Model A vs Model B trade-offs, and a use-case selection table
mapping common peripherals to recommended approaches, see
[notes/interrupt-latency-analysis.md](../notes/interrupt-latency-analysis.md).

## PAC Singleton in Partitioned Code

The `cortex_m` crate exposes core peripherals through a singleton
(`cortex_m::Peripherals`) guarded by an internal `AtomicBool`. Two
methods exist to obtain an instance: `take()` and `steal()`. In a
partitioned RTOS context, choosing the wrong one causes subtle failures.

### Why `take()` Must Not Be Used in Partition Code

`Peripherals::take()` sets a global `AtomicBool` to `true` on the first
call and returns `None` on every subsequent call. The boot path
(`main()` / `define_harness!`) calls `take()` once in privileged
mode to configure the MPU, SCB priorities, and SysTick. After that, the
`AtomicBool` is permanently set — any later `take()` from partition
code, PendSV, or SysTick will return `None`.

```rust
// Boot (privileged, runs once) — take() is correct here:
let p = cortex_m::Peripherals::take().unwrap();
boot::boot_preconfigured(p, kernel);

// Partition or exception handler — take() ALWAYS returns None:
let p = cortex_m::Peripherals::take(); // → None, panics on unwrap
```

`main()` and boot code **can and should** use `take()`, since they run
once in privileged mode before the scheduler starts.

### Why `steal()` Is Safe Under MPU Enforcement

`Peripherals::steal()` bypasses the `AtomicBool` and returns a new
instance unconditionally. In a bare-metal single-threaded program this
would be unsound — two call sites could hold aliasing `&mut` references
to the same MMIO registers. In this RTOS, the MPU is the real ownership
mechanism:

- Each partition declares its accessible peripherals via
  `peripheral_regions` (mapped to MPU regions R4–R6).
- The kernel programs these regions into the MPU on every context switch
  (PendSV).
- A partition can only access the MMIO addresses the MPU grants it.
  Accessing anything else triggers a MemManage fault.

Because the MPU enforces non-overlapping peripheral grants at the
hardware level, `steal()` inside a partition does not create true
aliasing — only the partition currently scheduled can reach its granted
registers.

### Canonical SAFETY Comment

Every `steal()` call in partition or exception-handler code must carry
this justification pattern:

```rust
// SAFETY: The MPU restricts this partition to the peripheral regions
// granted via `peripheral_regions`. No other partition or exception
// handler at this priority level can access the same MMIO range
// concurrently. The AtomicBool singleton is consumed by boot; steal()
// is the only way to obtain a handle post-boot.
let p = unsafe { cortex_m::Peripherals::steal() };
```

Adapt the comment when the exclusivity argument differs (e.g., in
PendSV where the guarantee is priority-based rather than MPU-based):

```rust
// SAFETY: PendSV is the lowest-priority exception. No other context
// at this priority can preempt us, so we have exclusive access to
// the MPU registers. steal() is required because take() was consumed
// by boot.
let p = unsafe { cortex_m::Peripherals::steal() };
```

### When MPU Enforcement Is Off

If the MPU is disabled or the build does not configure
`peripheral_regions` for a partition, `steal()` is still required
(since `take()` returns `None` post-boot), but the safety argument is
weaker: **the developer is responsible** for ensuring no two execution
contexts access the same peripheral concurrently. This is the same
contract as any bare-metal `steal()` usage — the RTOS provides no
additional protection without MPU enforcement.

In this configuration, review every `steal()` call site and verify
that only one partition (or exception handler) accesses each peripheral
register block. Treat every such `steal()` as a potential aliasing
hazard until you have confirmed exclusivity by inspection.

## Post-mortem Panic Diagnostics (`panic-tombstone` Feature)

When a partition panics, the kernel writes a 128-byte diagnostic record
(a *tombstone*) into a `.noinit` RAM region that survives soft resets.
After the board resets, a debugger or the application itself can read
the tombstone to determine what panicked and where.

### Enabling the Feature

Add `panic-tombstone` to your kernel dependency in `Cargo.toml`:

```toml
[dependencies.kernel]
features = ["panic-tombstone"]
```

If you are using a BSP crate, forward the feature:

```toml
# In your BSP's Cargo.toml
[features]
panic-tombstone = ["kernel/panic-tombstone"]
```

### `PanicTombstone` Memory Layout

The tombstone is a `#[repr(C)]` struct, exactly 128 bytes:

| Offset | Size (bytes) | Field     | Description                              |
|--------|-------------|-----------|------------------------------------------|
| 0      | 4           | `magic`   | `0xDEAD_C0DE` when valid, else uninitialized |
| 4      | 4           | `line`    | Source line number (0 if unavailable)    |
| 8      | 48          | `file`    | File path, UTF-8, zero-padded            |
| 56     | 72          | `message` | Panic message, UTF-8, zero-padded        |

**Total: 128 bytes.** The `magic` field distinguishes a valid tombstone
from uninitialized SRAM — always check it before interpreting other
fields.

### Reading the Tombstone with GDB

The tombstone lives at the linker symbol `__noinit_start`. After a
panic-triggered reset, connect GDB and inspect the region:

```
(gdb) # Dump the full 128-byte tombstone as 32 words
(gdb) x/32xw &__noinit_start

(gdb) # Quick validity check — first word should be 0xDEADC0DE
(gdb) p/x *(uint32_t*)&__noinit_start

(gdb) # Decode individual fields:
(gdb) # magic (offset 0)
(gdb) p/x *(uint32_t*)&__noinit_start
(gdb) # line  (offset 4)
(gdb) p *(uint32_t*)((char*)&__noinit_start + 4)
(gdb) # file  (offset 8, 48-byte string)
(gdb) x/s (char*)&__noinit_start + 8
(gdb) # message (offset 56, 72-byte string)
(gdb) x/s (char*)&__noinit_start + 56
```

### Reading the Tombstone with probe-rs

Use `target.read_word_32()` with the `.noinit` base address to check
for a valid tombstone, then read the remaining fields:

```rust
use probe_rs::Session;

fn read_tombstone(session: &mut Session) -> Result<(), probe_rs::Error> {
    let mut core = session.core(0)?;
    let noinit_base: u32 = 0x2000_0000; // adjust to your board's .noinit address

    let magic = core.read_word_32(noinit_base.into())?;
    if magic != 0xDEAD_C0DE {
        println!("No valid tombstone found (magic: {:#010X})", magic);
        return Ok(());
    }

    let line = core.read_word_32((noinit_base + 4).into())?;

    let mut file_buf = [0u8; 48];
    core.read_8((noinit_base + 8).into(), &mut file_buf)?;
    let file = core::str::from_utf8(&file_buf)
        .unwrap_or("<invalid utf8>")
        .trim_end_matches('\0');

    let mut msg_buf = [0u8; 72];
    core.read_8((noinit_base + 56).into(), &mut msg_buf)?;
    let message = core::str::from_utf8(&msg_buf)
        .unwrap_or("<invalid utf8>")
        .trim_end_matches('\0');

    println!("PANIC at {}:{}", file, line);
    println!("  {}", message);
    Ok(())
}
```

Replace `noinit_base` with the actual address of `__noinit_start` from
your board's linker map (run `arm-none-eabi-nm` on the ELF to find it).

### `.noinit` Section and Soft-Reset Survival

The kernel's build script (`build.rs`) emits a `tombstone.x` linker
fragment that places the `.noinit` section as a `NOLOAD` region after
`.bss`:

```ld
.noinit (NOLOAD) : ALIGN(4)
{
    __noinit_start = .;
    KEEP(*(.noinit .noinit.*))
    . = ALIGN(4);
    __noinit_end = .;
} INSERT AFTER .bss;
```

The `NOLOAD` attribute ensures the startup code does **not** zero this
region, so data written by the panic handler persists across a soft
reset (NVIC `SYSRESETREQ` or watchdog timeout).

**Most Cortex-M boards preserve SRAM contents across soft resets by
default** — the power domain stays active while only the CPU and
peripherals reset. However, be aware of the following board-specific
considerations:

- **Boards with ECC-protected SRAM** (e.g., some STM32L5/STM32H7
  variants) may zero SRAM on reset to initialize ECC parity bits. Check
  the reference manual's RCC reset behavior and, if needed, disable the
  SRAM-initialization-on-reset option in the option bytes.
- **External SRAM / SDRAM** requires the memory controller to be
  reconfigured after reset before the `.noinit` region is readable.
  Place `.noinit` in internal SRAM to avoid this.
- **Debugger-initiated resets** (e.g., probe-rs `reset-halt`) may
  behave differently from software resets. Some probes assert the
  `SRST` pin which can trigger a full power-on reset. Use
  `SYSRESETREQ`-based resets to guarantee SRAM survival.
- **Custom linker scripts**: if your board supplies its own `memory.x`
  that overrides the default MEMORY regions, ensure the `.noinit`
  section lands in internal SRAM. The `INSERT AFTER .bss` directive
  inherits the output region from `.bss`, which is correct for most
  layouts. For QEMU targets, the build script explicitly routes to
  `> RAM`.
