# CRITICAL Port Bug #11: PendSV Clobbers Callee-Saved Registers

Status: **Complete** (2026-03-01)

## Bug Summary

The PendSV handler's pre-save section loaded `__kernel_state_start` into
`r5` and `current_partition` into `r4` **before** calling
`pendsv_context_save`, silently corrupting the outgoing partition's
callee-saved registers. Because `r4` and `r5` were overwritten with
kernel bookkeeping values before the save routine had a chance to push
them to the partition's stack, the partition would resume with wrong
register contents after a context switch.

### Symptoms

- Callee-saved registers (`r4`–`r11`) in the outgoing partition contained
  kernel-internal values (pointer to `__kernel_state_start`, partition
  index) instead of the partition's own values after being switched back in.
- Corruption was **silent** — no fault, no trap. The partition simply
  continued with wrong data in `r4`/`r5`.
- Affected any partition that relied on callee-saved registers across a
  context switch (i.e., any non-trivial code compiled by LLVM).

## Root Cause

**ARM AAPCS violation.** Registers `r4`–`r11` are callee-saved under the
ARM Architecture Procedure Call Standard. On Cortex-M exception entry,
the hardware automatically saves `r0`–`r3`, `r12`, `lr`, `pc`, and
`xPSR` to the process stack — but **not** `r4`–`r11`. Those must be
preserved by software before any use.

The original PendSV code used `r4` and `r5` as scratch registers in the
pre-save section:

```asm
PendSV:
    ldr     r5, =__kernel_state_start       /* CLOBBERS r5 */
    ldr     r0, =KERNEL_CURRENT_PARTITION_OFFSET
    ldr     r0, [r0]
    ldrb    r4, [r5, r0]                    /* CLOBBERS r4 */
    cmp     r4, #0xFF
    beq     .Lpendsv_skip_save
    mov     r0, r4
    bl      pendsv_context_save             /* saves r4-r11, but r4/r5 are already wrong */
```

By the time `pendsv_context_save` pushed `r4`–`r11`, the original values
of `r4` and `r5` were already lost.

## Fix (commit 582c17c)

Rewrote the pre-save section to use only `r0`–`r1`, which are
hardware-saved on exception entry and therefore safe to use as scratch:

```asm
PendSV:
    /* r4-r11 contain the outgoing partition's callee-saved registers
     * and must NOT be modified before pendsv_context_save. */

    /* Load current_partition using only r0-r3 (hardware-saved) */
    ldr     r0, =__kernel_state_start
    ldr     r1, =KERNEL_CURRENT_PARTITION_OFFSET
    ldr     r1, [r1]
    ldrb    r1, [r0, r1]        /* r1 = current_partition */

    cmp     r1, #0xFF
    beq     .Lpendsv_skip_save

    mov     r0, r1
    bl      pendsv_context_save

.Lpendsv_skip_save:
    /* Load kernel base into r5 (safe now: context is saved) */
    ldr     r5, =__kernel_state_start
```

Key changes:
1. **Pre-save uses only `r0`–`r1`** — both are hardware-saved on
   exception entry, so clobbering them does not lose partition state.
2. **`r5` load deferred** to after `.Lpendsv_skip_save`, where the
   outgoing context has already been saved (or skipped on first switch).
3. `r4` is no longer used pre-save at all; `current_partition` stays
   in `r1` until passed to `pendsv_context_save` via `r0`.

## Affected File

`kernel/src/pendsv.rs` lines 122–142 (the `PendSV` entry point in the
`define_pendsv!` macro's `global_asm!` block).

## Regression Test

`kernel/examples/callee_save_check.rs` — QEMU integration test
(commit cc4e343) that:

1. Loads known sentinel values (`0xAAAA0004`–`0xAAAA000B`) into
   `r4`–`r11` in a partition.
2. Spins in a delay loop to force multiple context switches via SysTick.
3. Verifies all eight callee-saved registers still hold the expected
   sentinels after the partition resumes.
4. Repeats 10+ times; fails on any single mismatch.

Run: `./scripts/qemu-test.sh callee_save_check`

## Resolution Checklist

- [x] Root cause identified (AAPCS violation: r4/r5 used before context save)
- [x] Fix implemented (pre-save uses only r0-r1; r5 deferred post-save)
- [x] Commit landed (582c17c)
- [x] Regression test added (callee_save_check QEMU example, cc4e343)
- [x] Safety comments updated in pendsv.rs to document register convention
- [x] No other callee-saved register misuse found in PendSV handler
