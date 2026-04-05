# Porting Guide

A 10-step progression for bringing up the RTOS on a new Cortex-M target.
Each step builds on the previous one and has clear pass/fail criteria.
If a step fails, fix it before moving on — later steps depend on earlier
foundations being solid.

## Prerequisites

### Debug Channel Verification

Before running any example, confirm your debug output channel works
independently of the kernel. On QEMU with semihosting this is automatic.
On real hardware, verify one of:

- **Semihosting** — `hprintln!("hello")` prints to your debugger console
- **RTT** — `rprintln!("hello")` appears in your RTT viewer (J-Link, probe-rs)
- **SWO/ITM** — ITM stimulus port 0 output appears in your trace viewer

If you see no output, nothing else in this guide will be diagnosable.
Fix your debug channel first.

## Progression Table

| Step | Example | What It Verifies | Pass Criteria |
|------|---------|-------------------|---------------|
| 00 | `00_context_switch` | Bare-metal PendSV save/restore (r4-r11; r0-r3, r12, LR, PC, xPSR are hardware-stacked), SysTick preemption | 10+ context switches, both partitions alternate |
| 01 | `01_mpu_static` | Static MPU region programming and RBAR/RASR readback | All 4 regions match expected base/size/AP |
| 02 | `02_mpu_dynamic` | Dynamic MPU Region 4 reprogramming across context switches | Each partition verified ≥4 times with correct MPU Region 4 |
| 03 | `03_svc_handler` | SVC voluntary yield coexisting with SysTick preemption | ≥4 SVC yields + ≥4 SysTick preemptions, both partitions advance |
| 04 | `04_schedule_table` | Deterministic time-slice scheduling (ScheduleTable, major frame) | 3 major frames complete; P0≥12 ticks, P1≥6 ticks |
| 05 | `05_sampling_port` | Raw inter-partition communication via atomics | Consumer receives ≥10 distinct in-order values |
| 06 | `06_kernel_minimal` | Full kernel API boot (`Kernel::new`, `define_kernel!`, `SYS_YIELD`) | SYS_YIELD returns 0 (success) |
| 07 | `07_queuing_ports` | Kernel queuing port IPC with sequence validation | 50+ messages exchanged, no corruption or reordering |
| 08 | `08_semaphores` | Semaphore-based synchronization between two partitions | 10+ signal/wait cycles completed |
| 09 | `09_full_demo` | Combined scheduling + sampling + queuing + semaphores (3 partitions) | All IPC channels active, 3+ major frames |

**Steps 00–05** are bare-metal — they exercise hardware mechanisms
without the kernel. **Steps 06–09** use the full kernel API.

## Running Examples

### QEMU (lm3s6965evb)

All examples are pre-configured for QEMU via `.cargo/config.toml`.
The runner invokes `qemu-system-arm` with semihosting automatically:

```bash
# Single example
cargo run -p porting_guide --target thumbv7m-none-eabi \
    --features board-qemu,log-semihosting \
    --example 00_context_switch

# All examples (CI script pattern)
for step in 00_context_switch 01_mpu_static 02_mpu_dynamic \
    03_svc_handler 04_schedule_table 05_sampling_port \
    06_kernel_minimal 07_queuing_ports 08_semaphores 09_full_demo; do
  timeout 30 cargo run -p porting_guide --target thumbv7m-none-eabi \
      --features board-qemu,log-semihosting --example "$step"
done
```

Output appears on stdout via semihosting. Each example prints `PASS` or
`FAIL` on its last line.

### Real Hardware

1. Select your board feature and debug channel:
   ```bash
   cargo build -p porting_guide --target thumbv7em-none-eabihf \
       --features board-stm32f4-discovery,log-rtt \
       --example 00_context_switch
   ```

2. Flash with your tool of choice (`probe-rs`, `openocd`, `J-Link`):
   ```bash
   probe-rs run --chip STM32F407VGTx \
       target/thumbv7em-none-eabihf/debug/examples/00_context_switch
   ```

3. Monitor debug output via RTT viewer or semihosting console.

For a new board, implement the `Board` trait in
`src/boards/your_board.rs` and add a corresponding feature in
`Cargo.toml`.

## Troubleshooting

### 1. PAC Accessor Mismatch (`Peripherals::take()` Returns `None`)

**Symptom:** Partition code calls `Peripherals::take()` and gets `None`,
causing a panic.

**Cause:** The boot code already consumed the PAC singleton. In a
partitioned RTOS, `take()` will always return `None` after boot.

**Fix:** Prefer the kernel's safe peripheral accessor if available.
When direct PAC access is necessary, use `Peripherals::steal()` with a
safety comment documenting the MPU-based isolation invariant:
```rust
// SAFETY: This partition has exclusive MPU-granted access to the GPIOA
// region declared in `with_peripherals(&[MpuRegion::new(...)])`.
// No other partition can access this address range while we are scheduled.
let pac = unsafe { Peripherals::steal() };
```
Declare the peripheral in `with_peripherals(&[MpuRegion::new(...)])` so
the kernel grants exclusive access to that partition.

### 2. Debug Output Before RTT Initialization

**Symptom:** Board silently resets in a loop with no output. May appear
intermittent if a previous run left valid RTT magic bytes in SRAM.

**Cause:** Calling `rprintln!()` before `init_rtt()` writes to
uninitialized memory, corrupting the RTT control block.

**Fix:** Use `klog!()` instead of `rprintln!()` — it checks
`is_rtt_initialized()` internally and is safe to call at any point.
If you must use `rprintln!()` directly, guard it:
```rust
if kernel::is_rtt_initialized() {
    rprintln!("safe to print");
}
```

### 3. SysTick / PendSV Deadlock in Debug Builds

**Symptom:** System appears to hang — no output, no partition progress.
RTT/semihosting stops updating.

**Cause:** In unoptimized (`dev`) builds, PendSV (context switch +
MPU reprogramming) can exceed the 1ms SysTick period. SysTick fires
mid-PendSV, creating an infinite bounce between the two handlers.

**Fix:** Always use `--release` (or `opt-level >= 1`) when MPU
enforcement is active. Alternatively, increase the SysTick period.

For JTAG/SWD debugging where WFI halts the debug interface, enable
`debug-no-wfi` to replace WFI with NOP in idle loops:
```bash
cargo run --features board-qemu,log-semihosting,debug-no-wfi ...
```

### 4. Rust 2024 Edition Changes

**Symptom:** Compiler warnings or errors about `#[no_mangle]` after
upgrading to Rust edition 2024.

**Cause:** Edition 2024 requires `#[unsafe(no_mangle)]` instead of
`#[no_mangle]`. The partition entry-point macros in `kernel/src/macros.rs`
currently use the edition 2021 syntax.

**Fix:** When migrating to edition 2024, update partition entry macros:
```rust
// Edition 2021 (current)
#[no_mangle]
// SAFETY: Partition entry point called by the kernel's context switch
// mechanism. Must be `extern "C"` for ABI compatibility with the
// assembly trampoline. The function is never called from safe Rust.
pub unsafe extern "C" fn partition_entry() { ... }

// Edition 2024 (required after migration)
#[unsafe(no_mangle)]
// SAFETY: Same as above — kernel-invoked partition entry point with
// C ABI required by the PendSV assembly trampoline.
pub unsafe extern "C" fn partition_entry() { ... }
```

## Feature Flag Reference

### Debug Channels

Select exactly one. If none is selected, `klog!()` compiles to nothing.

| Feature | Backend | Use Case |
|---------|---------|----------|
| `log-semihosting` | ARM semihosting | QEMU, debugger-attached targets |
| `log-rtt` | SEGGER RTT | J-Link / probe-rs on real hardware |
| `log-swo` | SWO/ITM trace | SWO pin available, minimal overhead |
| `log-defmt` | defmt framework | Size-optimized structured logging |

### Board Support

| Feature | Target | Chip |
|---------|--------|------|
| `board-qemu` | `thumbv7m-none-eabi` | LM3S6965EVB (QEMU) |
| `board-stm32f4-discovery` | `thumbv7em-none-eabihf` | STM32F407VG |
| `board-nrf52840-dk` | `thumbv7em-none-eabihf` | nRF52840 |

### Other Flags

| Feature | Effect |
|---------|--------|
| `debug-no-wfi` | Replace WFI with NOP in idle loops (keeps JTAG alive) |
| `qemu-peripherals` | Enable typed PAC access for LM3S6965 peripherals |
| `fpu-context` | Save/restore FPU registers (s16-s31) on context switch |
| `custom-ivt` | Omit built-in interrupt vector table (supply your own) |
