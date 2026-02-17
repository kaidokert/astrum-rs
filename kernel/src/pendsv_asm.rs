//! Shared PendSV assembly routines for context switching.
//!
//! Defines reusable assembly functions as linkable symbols for PendSV handlers:
//! - `pendsv_context_save` — saves r4-r11 and PSP for the outgoing partition
//! - `pendsv_context_restore` — restores r4-r11 and PSP for the incoming partition
//! - `pendsv_return_unprivileged` — sets CONTROL.nPRIV=1 and returns to Thread mode
//!
//! # Safety
//!
//! These functions must only be called from PendSV handlers in Handler mode.

// SAFETY: This assembly block defines PendSV context switch routines that are sound
// because:
//
// 1. **Register Operations (r4-r11 save/restore)**:
//    - The Cortex-M hardware automatically saves r0-r3, r12, lr, pc, xpsr on exception
//      entry (the "hardware frame"). We only save/restore r4-r11 (the "software frame")
//      which AAPCS designates as callee-saved registers.
//    - `stmdb r3!, {r4-r11}` decrements PSP then stores, preserving the pre-exception
//      register values before any assembly code modifies them.
//    - `ldmia r3!, {r4-r11}` restores these registers and increments PSP, matching the
//      stmdb operation exactly.
//    - The push/pop of lr around `bl` calls follows AAPCS for nested function calls.
//
// 2. **PSP/MSP Stack Pointer Conventions**:
//    - Partitions always use PSP (Process Stack Pointer) in Thread mode.
//    - Exception handlers (including PendSV) always use MSP (Main Stack Pointer).
//    - `mrs r3, psp` reads the partition's stack pointer without affecting MSP.
//    - `msr psp, r3` updates PSP for the incoming partition before exception return.
//    - The kernel's MSP is never modified by context switch code.
//
// 3. **CONTROL.nPRIV=1 Return Behavior**:
//    - `pendsv_return_unprivileged` sets CONTROL.nPRIV=1 before `bx lr`, ensuring the
//      partition returns to Thread mode as unprivileged code.
//    - The ISB after MSR CONTROL ensures the privilege drop takes effect before any
//      subsequent instructions execute.
//    - EXC_RETURN=0xFFFFFFFD specifies: return to Thread mode, use PSP, no FPU context.
//    - Combined with PRIVDEFENA in MPU_CTRL, unprivileged code cannot access kernel
//      memory or privileged peripherals.
//
// 4. **Calling Context Assumptions**:
//    - These functions MUST be called only from PendSV handler (Handler mode, priority
//      0xFF lowest). They are NOT safe to call from Thread mode or other exceptions.
//    - The caller must ensure `r0` contains a valid partition index before calling
//      `pendsv_context_save` or `pendsv_context_restore`.
//    - `set_partition_sp` and `get_partition_sp` are Rust FFI functions that safely
//      access the `PARTITION_SP` array with bounds checking.
//    - The null-pointer check in `pendsv_context_restore` (cmp r3, #0; beq fault)
//      prevents dereferencing an invalid SP if get_partition_sp returns 0 for an
//      uninitialized or invalid partition.
//
// 5. **Interrupt Safety**:
//    - PendSV runs at lowest priority (0xFF), so it cannot preempt other exceptions.
//    - No other code can preempt PendSV mid-context-switch, ensuring atomicity.
//    - The fault loop in restore path keeps the system debuggable rather than causing
//      an unpredictable HardFault.
#[cfg(target_arch = "arm")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .thumb

    /* ================================================================
     * pendsv_context_save
     * ================================================================
     * Saves the current partition's context (r4-r11 and PSP).
     *
     * Input:  r0 = current partition index (must be valid, not 0xFF)
     * Output: none
     * Clobbers: r0, r1, r2, r3, lr (follows AAPCS)
     *
     * The save path:
     *   1. mrs r3, psp — get current process stack pointer
     *   2. stmdb r3!, {{r4-r11}} — push callee-saved regs onto process stack
     *   3. bl set_partition_sp(idx, sp) — store updated PSP
     */
    .global pendsv_context_save
    .type pendsv_context_save, %function
    .thumb_func
pendsv_context_save:
    /* Save lr since we'll call set_partition_sp */
    push    {{lr}}

    /* Get PSP and push r4-r11 onto process stack BEFORE modifying any of them.
     * r0 (partition index) is not affected by mrs or stmdb. */
    mrs     r3, psp
    stmdb   r3!, {{r4-r11}}     /* r3 now points to saved context */

    /* set_partition_sp(idx, sp) — r0 still holds partition index */
    mov     r1, r3              /* r1 = new stack pointer */
    bl      set_partition_sp

    pop     {{pc}}
    .size pendsv_context_save, . - pendsv_context_save

    /* ================================================================
     * pendsv_context_restore
     * ================================================================
     * Restores an incoming partition's context (r4-r11 and PSP).
     *
     * Input:  r0 = next partition index
     * Output: none (PSP and r4-r11 restored)
     * Clobbers: r0, r1, r2, r3, lr (follows AAPCS)
     *
     * SAFETY: If get_partition_sp returns 0 (invalid index or
     * uninitialized kernel), this function branches to a fault
     * loop rather than dereferencing a null pointer.
     *
     * The restore path:
     *   1. bl get_partition_sp(idx) — retrieve saved SP
     *   2. Validate SP is non-zero (fault if zero)
     *   3. ldmia r3!, {{r4-r11}} — pop callee-saved regs from process stack
     *   4. msr psp, r3 — update PSP
     */
    .global pendsv_context_restore
    .type pendsv_context_restore, %function
    .thumb_func
pendsv_context_restore:
    push    {{lr}}

    /* get_partition_sp(idx) */
    bl      get_partition_sp
    mov     r3, r0              /* r3 = saved stack pointer */

    /* Validate: SP must be non-zero */
    cmp     r3, #0
    beq     .Lrestore_fault

    /* Restore r4-r11 and update PSP */
    ldmia   r3!, {{r4-r11}}
    msr     psp, r3

    pop     {{pc}}

.Lrestore_fault:
    /* Invalid partition SP - loop forever to trigger watchdog or debug.
     * This is a fatal error that should never occur in a correctly
     * configured system. Looping is safer than HardFaulting because
     * it keeps the system in a debuggable state. */
    b       .Lrestore_fault
    .size pendsv_context_restore, . - pendsv_context_restore

    /* ================================================================
     * pendsv_return_unprivileged
     * ================================================================
     * Returns to Thread mode as unprivileged.
     *
     * Input:  none
     * Output: does not return (performs exception return)
     *
     * The return path:
     *   1. Sets CONTROL.nPRIV = 1 to drop to unprivileged mode
     *   2. Issues ISB to ensure CONTROL write takes effect
     *   3. Loads EXC_RETURN = 0xFFFFFFFD (Thread mode, PSP)
     *   4. bx lr to complete exception return
     */
    .global pendsv_return_unprivileged
    .type pendsv_return_unprivileged, %function
    .thumb_func
pendsv_return_unprivileged:
    mrs     r0, CONTROL
    orr     r0, r0, #1
    msr     CONTROL, r0
    isb
    ldr     lr, =0xFFFFFFFD
    bx      lr
    .size pendsv_return_unprivileged, . - pendsv_return_unprivileged
"#
);
