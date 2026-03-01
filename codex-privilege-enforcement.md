# Privilege Enforcement Resolution

Status: **Resolved** — all enforcement mechanisms implemented and tested.

## 1. Threat Model

Partitions execute unprivileged on Cortex-M3. Three mechanisms enforce this:

**CONTROL.nPRIV=1 via PendSV** — The PendSV context-switch handler
(`pendsv_asm.rs:185-192`) sets CONTROL.nPRIV=1 before returning to Thread
mode. An ISB barrier ensures the privilege drop takes effect immediately.
EXC_RETURN is set to `0xFFFFFFFD` (Thread mode, PSP, no FPU).

**MPU with PRIVDEFENA** — The MPU is configured with PRIVDEFENA=1 in
MPU_CTRL. Privileged code uses the default memory map; unprivileged code
faults (MemManage/DACCVIOL) on any access outside explicitly granted MPU
regions. Each partition receives two regions: code (RX) and stack/data (RW,
XN). Kernel memory is unmapped for unprivileged access.

**SVCall EXC_RETURN validation** — The SVC trampoline (`svc.rs:404-416`)
checks `lr == 0xFFFFFFFD` before dispatching any syscall. If EXC_RETURN
does not match Thread-mode PSP, execution enters a fault loop. This
prevents Handler-mode code from invoking syscalls.

## 2. Security Fixes Applied

**Confused deputy fix** — `EventWait` and `EventClear`
previously read the target partition index from the user-supplied register
`r1`. A malicious partition could pass another partition's index to wait on
or clear its events. Fix: both syscalls now use the kernel-derived `caller`
parameter (`svc.rs:854-856`, `events.rs:6,37`). `EventSet` intentionally
still uses `r1` since setting events on other partitions is by design.

**KERNEL_DATA_END runtime derivation** — Pointer validation
previously used a compile-time constant for the kernel data boundary. Fix:
on ARM targets, `kernel_data_end()` (`svc.rs:36-46`) reads the linker
symbol `__kernel_state_end` at runtime. `validate_user_ptr()` rejects
pointers overlapping kernel code `[0, 0x10000)` or kernel data
`[0x2000_0000, __kernel_state_end)` before checking MPU region bounds.

## 3. Test Coverage

**Adversarial QEMU suite (12 tests)** — Hardware integration tests in
`kernel/examples/adversarial/` exercise real fault paths on QEMU:

| Test | Verifies |
|------|----------|
| `read_kernel_mem` | MemManage fault on unprivileged kernel read |
| `write_kernel_mem` | MemManage fault on unprivileged kernel write |
| `read_other_stack` | Cross-partition read causes fault |
| `write_other_stack` | Cross-partition write causes fault |
| `syscall_kernel_ptr` | `validate_user_ptr` rejects kernel addresses |
| `syscall_other_ptr` | `validate_user_ptr` rejects cross-partition ptrs |
| `syscall_bad_ptrs` | Null, wrapping, and OOB pointers rejected |
| `stack_guard` | Stack overflow triggers MPU fault |
| `write_mpu_regs` | Unprivileged MPU register write faults |
| `disable_interrupts` | Unprivileged CPSID is no-op / faults |
| `write_control` | Unprivileged CONTROL write is no-op / faults |
| `svc_from_handler` | (covered by EXC_RETURN guard) |

**MPU_ENFORCE integration test** —
`mpu_enforce_test.rs` runs two partitions with `MPU_ENFORCE=true`,
validating that hardware MPU regions are programmed on every context
switch and partitions execute correctly under full enforcement.

**Privilege drop verification** —
- `priv_drop_test.rs` — reads CONTROL register in partition context,
  asserts nPRIV=1 and SPSEL=1 (PSP).
- `priv_fault_test.rs` — configures MPU with PRIVDEFENA, attempts
  unprivileged access to kernel RAM, validates MemManage/DACCVIOL.

**Unit tests** — `svc.rs` contains regression tests for the confused
deputy fix (`confused_deputy_eventWait_uses_caller_not_r1`,
`confused_deputy_eventClear_uses_caller_not_r1`,
`event_set_routes_to_target_in_r1`) and EXC_RETURN constant validation
(`svc_exc_return_guard_matches_context_constant`).

## 4. Residual Items

**MPU_ENFORCE defaults to `false`** — `config.rs:14` sets
`KernelConfig::MPU_ENFORCE = false`. Rationale: enabling MPU enforcement
requires all partition MPU regions to be correctly sized and aligned
(power-of-two, naturally aligned per ARMv7-M). The default-off setting
allows development and testing without MPU constraints. Production
deployments must set `MPU_ENFORCE = true` in their KernelConfig.

**PendSV partition index bounds check** — The PendSV assembly does not
validate that the partition index passed by the scheduler is within bounds.
Rationale: the partition index originates from the kernel scheduler (trusted
privileged code), not from user input. Adding a bounds check in the
hot-path context switch would add latency to every partition transition.
This is documented as a caller (scheduler) responsibility.
