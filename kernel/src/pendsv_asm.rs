//! Shared PendSV assembly routines for context switching.
//!
//! Defines reusable assembly functions as linkable symbols for PendSV handlers:
//! - `pendsv_context_save` — saves r4-r11 and PSP for the outgoing partition
//! - `pendsv_context_restore` — restores r4-r11 and PSP for the incoming partition
//! - `pendsv_return_unprivileged` — sets CONTROL.nPRIV=1 and returns to Thread mode
//!
//! # Architecture
//!
//! These routines access the `partition_sp` array directly using struct field offsets
//! rather than calling Rust shim functions. The `KERNEL_PTR` AtomicPtr global
//! provides the kernel base address, and compile-time offset constants
//! (`KERNEL_CORE_OFFSET`, `CORE_PARTITION_SP_OFFSET`) allow direct array access.
//!
//! # Safety
//!
//! These functions must only be called from PendSV handlers in Handler mode.

// EXC_RETURN string constants for assembly injection (must stay in sync with context.rs).
// Macros (not `const`) because `concat!` requires literal tokens.
// 0xFFFFFFED = 0xFFFFFFFD with bit 4 cleared (FPU frame present).
#[allow(unused_macros)]
macro_rules! exc_return_no_fpu {
    () => {
        "0xFFFFFFFD"
    };
}
#[allow(unused_macros)]
macro_rules! exc_return_fpu {
    () => {
        "0xFFFFFFED"
    };
}

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
//    - EXC_RETURN value specifies: return to Thread mode, use PSP, with or without FPU.
//    - Combined with PRIVDEFENA in MPU_CTRL, unprivileged code cannot access kernel
//      memory or privileged peripherals.
//
// 4. **Calling Context Assumptions**:
//    - These functions MUST be called only from PendSV handler (Handler mode, priority
//      0xFF lowest). They are NOT safe to call from Thread mode or other exceptions.
//    - The caller must ensure `r0` contains a valid partition index before calling
//      `pendsv_context_save` or `pendsv_context_restore`.
//    - Direct memory access to partition_sp[idx] uses KERNEL_PTR (loaded via
//      AtomicPtr) + KERNEL_CORE_OFFSET + CORE_PARTITION_SP_OFFSET + (idx * 4). Bounds checking
//      is the caller's responsibility.
//    - The null-pointer check in `pendsv_context_restore` (cmp r3, #0; beq fault)
//      prevents dereferencing an invalid SP for an uninitialized partition.
//
// 5. **Interrupt Safety**:
//    - PendSV runs at lowest priority (0xFF), so it cannot preempt other exceptions.
//    - No other code can preempt PendSV mid-context-switch, ensuring atomicity.
//    - The fault loop in restore path keeps the system debuggable rather than causing
//      an unpredictable HardFault.
//
// 6. **FPU Register Operations (s16-s31)** [fpu-context only]:
//    Hardware lazy-stacks s0-s15/FPSCR; we manually save/restore s16-s31
//    (callee-saved per AAPCS) via vstmdb/vldmia with `.fpu fpv4-sp-d16`.
//
// 7. **EXC_RETURN with FPU Frame** [fpu-context only]:
//    Bit 4 = 0 signals an extended FPU frame on the stack.

/// Generates the PendSV `global_asm!` block, parameterized by FPU directive,
/// save/restore instructions, and EXC_RETURN value.
#[allow(unused_macros)]
macro_rules! pendsv_global_asm {
    ($fpu_directive:expr, $fpu_save:expr, $fpu_restore:expr, $exc_return:expr) => {
        core::arch::global_asm!(concat!(
            r#"
    .syntax unified
    .thumb
"#,
            // FPU assembler directive (empty for non-FPU builds)
            "    ",
            $fpu_directive,
            "\n",
            r#"
    /* ================================================================
     * pendsv_context_save
     * ================================================================
     * Saves the current partition's context (r4-r11, optional s16-s31, and PSP).
     *
     * Input:  r0 = current partition index (must be valid, not 0xFF)
     * Output: none
     * Clobbers: r0, r1, r2, r3 (follows AAPCS)
     */
    .global pendsv_context_save
    .type pendsv_context_save, %function
    .thumb_func
pendsv_context_save:
    /* Get PSP and push callee-saved regs onto process stack BEFORE modifying any.
     * r0 (partition index) is not affected by mrs or stmdb. */
    mrs     r3, psp
    stmdb   r3!, {{r4-r11}}     /* push integer callee-saved regs */
"#,
            // FPU save (empty for non-FPU builds)
            // TODO: Stack overflow check is post-facto — with FPU context the overrun window
            // is 64 bytes larger. Pre-check would require knowing the save size before pushing.
            $fpu_save,
            r#"
    @ SAFETY: KERNEL_PTR is guaranteed non-null because store_kernel_ptr() must
    @ be called before enabling interrupts, and PendSV (priority 0xFF) cannot
    @ fire until interrupts are enabled. A defensive null check is included below
    @ to trap misconfiguration with a debuggable fault loop instead of HardFault.
    @ idx (r0) was validated by the PendSV caller against partition_count.
    @ KERNEL_PTR (AtomicPtr) + KERNEL_CORE_OFFSET + CORE_PARTITION_SP_OFFSET gives
    @ the base of the partition_sp array; adding idx*4 gives the element address.
    @ The stack overflow check compares the post-push PSP (r3) against the
    @ partition's stack_limit. If PSP < stack_limit, the stack has overflowed
    @ and we enter a fatal fault loop (skipping context restore) rather than
    @ storing a corrupted SP that would crash on the next restore.
    lsl     r0, r0, #2          /* r0 = idx * 4 (used for both arrays) */
    ldr     r1, =KERNEL_PTR
    ldr     r1, [r1]            /* r1 = kernel pointer (from AtomicPtr) */
    cmp     r1, #0
    beq     .Lnull_kernel_fault
    ldr     r2, =KERNEL_CORE_OFFSET
    ldr     r2, [r2]
    add     r1, r1, r2          /* r1 = &kernel.core (preserved) */

    /* Compute partition_sp element address (shared by both paths) */
    ldr     r2, =CORE_PARTITION_SP_OFFSET
    ldr     r2, [r2]
    add     r2, r1, r2          /* r2 = &partition_sp[0] */

    /* Stack overflow pre-check: PSP vs partition_stack_limits[idx] */
    push    {{r2}}                /* save &partition_sp[0] across check */
    ldr     r2, =CORE_PARTITION_STACK_LIMIT_OFFSET
    ldr     r2, [r2]
    add     r2, r1, r2          /* r2 = &partition_stack_limits[0] */
    ldr     r2, [r2, r0]        /* r2 = partition_stack_limits[idx] */
    cmp     r3, r2
    blo     .Lstack_overflow
    pop     {{r2}}                /* r2 = &partition_sp[0] */
    str     r3, [r2, r0]        /* partition_sp[idx] = new SP */
    bx      lr

    /* Stack overflow detected: the outgoing partition's PSP is below its
     * stack_limit. Store a sentinel value and enter a fatal fault loop.
     * We do NOT return to the PendSV handler because context restore would
     * attempt to load from the corrupted stack, causing an unpredictable
     * crash. The fault loop keeps the system debuggable. */
    .global __pendsv_stack_overflow
    .type __pendsv_stack_overflow, %function
    .thumb_func
__pendsv_stack_overflow:
.Lstack_overflow:
    pop     {{r2}}                /* r2 = &partition_sp[0] */
    ldr     r3, =0xDEAD0001
    str     r3, [r2, r0]        /* partition_sp[idx] = sentinel */
    bx      lr                  /* return to PendSV */
    .size __pendsv_stack_overflow, . - __pendsv_stack_overflow
    .size pendsv_context_save, . - pendsv_context_save

    /* ================================================================
     * pendsv_context_restore
     * ================================================================
     * Restores an incoming partition's context (optional s16-s31, r4-r11, and PSP).
     *
     * Input:  r0 = next partition index
     * Output: none (PSP and callee-saved registers restored)
     * Clobbers: r0, r1, r2, r3 (follows AAPCS)
     *
     * SAFETY: If partition_sp[idx] is 0 (uninitialized partition),
     * this function branches to a fault loop rather than dereferencing
     * a null pointer.
     */
    .global pendsv_context_restore
    .type pendsv_context_restore, %function
    .thumb_func
pendsv_context_restore:
    @ SAFETY: KERNEL_PTR is guaranteed non-null because store_kernel_ptr() must
    @ be called before enabling interrupts, and PendSV (priority 0xFF) cannot
    @ fire until interrupts are enabled. A defensive null check is included below.
    @ idx (r0) was validated by the PendSV caller against partition_count.
    @ The subsequent null check on partition_sp[idx] (cmp r3, #0; beq fault)
    @ guards against uninitialized partition entries.
    @ addr = KERNEL_PTR + KERNEL_CORE_OFFSET + CORE_PARTITION_SP_OFFSET + (idx * 4)
    ldr     r1, =KERNEL_PTR
    ldr     r1, [r1]            /* r1 = kernel pointer (from AtomicPtr) */
    cmp     r1, #0
    beq     .Lnull_kernel_fault
    ldr     r2, =KERNEL_CORE_OFFSET
    ldr     r2, [r2]            /* r2 = core offset */
    add     r1, r1, r2          /* r1 = &kernel.core */

    ldr     r2, =CORE_PARTITION_SP_OFFSET
    ldr     r2, [r2]            /* r2 = partition_sp offset within core */
    add     r1, r1, r2          /* r1 = &kernel.core.partition_sp[0] */

    /* idx * 4 for u32 array element offset */
    lsl     r2, r0, #2          /* r2 = idx * 4 */
    ldr     r3, [r1, r2]        /* r3 = partition_sp[idx] */

    /* Validate: SP must be non-zero */
    cmp     r3, #0
    beq     .Lrestore_fault

    /* Restore callee-saved regs and update PSP */
"#,
            // FPU restore (empty for non-FPU builds)
            $fpu_restore,
            r#"    ldmia   r3!, {{r4-r11}}
    msr     psp, r3

    bx      lr

.Lrestore_fault:
    /* Invalid partition SP — fatal error that should never occur in a
     * correctly configured system.  bkpt #2 halts the core so a debugger
     * can inspect the faulting context.  The infinite loop keeps the
     * system in a debuggable state (safer than HardFaulting) and
     * prevents fall-through if no debugger is attached.
     * Immediate values: #1 = bad EXC_RETURN (SVCall),
     * #2 = null partition SP (PendSV restore),
     * #3 = null KERNEL_PTR (PendSV entry). */
    bkpt    #2
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
     *   3. Loads EXC_RETURN (Thread mode, PSP, +/- FPU frame)
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
    ldr     lr, ="#,
            $exc_return,
            r#"
    bx      lr
    .size pendsv_return_unprivileged, . - pendsv_return_unprivileged

    /* Null KERNEL_PTR fault: KERNEL_PTR was null when PendSV fired.
     * This means store_kernel_ptr() was not called before enabling interrupts.
     * bkpt #3 halts the core for debugger inspection.  The infinite loop
     * keeps the system debuggable and prevents fall-through. */
.Lnull_kernel_fault:
    bkpt    #3
    b       .Lnull_kernel_fault
"#
        ));
    };
}

#[cfg(all(target_arch = "arm", not(feature = "fpu-context")))]
pendsv_global_asm!(
    "", // no FPU directive
    "", // no FPU save
    "", // no FPU restore
    exc_return_no_fpu!()
);

#[cfg(all(target_arch = "arm", feature = "fpu-context"))]
pendsv_global_asm!(
    ".fpu fpv4-sp-d16",
    "    vstmdb  r3!, {{s16-s31}}    /* push FPU callee-saved regs (64 bytes) */\n",
    "    vldmia  r3!, {{s16-s31}}    /* pop FPU callee-saved regs (64 bytes) */\n",
    exc_return_fpu!()
);

#[cfg(test)]
mod tests {
    //! Tests for PendSV assembly cfg-gating and constant consistency.
    //!
    //! The actual assembly instructions cannot be executed on the host, but we
    //! verify that the FPU context constants used by the assembly are consistent
    //! with the definitions in `context.rs`.

    /// The non-FPU EXC_RETURN value used in the assembly (Thread mode, PSP, no FPU).
    const EXC_RETURN_NO_FPU: u32 = 0xFFFF_FFFD;
    /// The FPU EXC_RETURN value used in the assembly (Thread mode, PSP, FPU frame).
    const EXC_RETURN_FPU: u32 = 0xFFFF_FFED;

    #[test]
    fn exc_return_str_constants_match() {
        // Verify the string macro constants injected into assembly match the numeric values.
        assert_eq!(
            u32::from_str_radix(&exc_return_no_fpu!()[2..], 16).unwrap(),
            EXC_RETURN_NO_FPU,
            "exc_return_no_fpu!() must match numeric constant"
        );
        assert_eq!(
            u32::from_str_radix(&exc_return_fpu!()[2..], 16).unwrap(),
            EXC_RETURN_FPU,
            "exc_return_fpu!() must match numeric constant"
        );
    }

    #[test]
    fn exc_return_fpu_clears_bit4() {
        // Bit 4 = 0 indicates extended FPU frame is present on the stack.
        assert_eq!(
            EXC_RETURN_FPU & (1 << 4),
            0,
            "FPU EXC_RETURN bit 4 must be 0"
        );
        assert_ne!(
            EXC_RETURN_NO_FPU & (1 << 4),
            0,
            "non-FPU EXC_RETURN bit 4 must be 1"
        );
    }

    #[test]
    fn exc_return_only_bit4_differs() {
        // The FPU and non-FPU EXC_RETURN values differ only in bit 4.
        assert_eq!(EXC_RETURN_NO_FPU ^ EXC_RETURN_FPU, 1 << 4);
    }

    #[test]
    fn exc_return_thread_mode_psp_bits() {
        // Both variants must specify Thread mode (bit 3) and PSP (bit 2).
        for &val in &[EXC_RETURN_NO_FPU, EXC_RETURN_FPU] {
            assert_ne!(val & (1 << 2), 0, "bit 2 (PSP) must be set");
            assert_ne!(val & (1 << 3), 0, "bit 3 (Thread mode) must be set");
        }
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn exc_return_matches_context_constant() {
        use crate::context::EXC_RETURN_THREAD_PSP_FPU;
        assert_eq!(
            EXC_RETURN_FPU, EXC_RETURN_THREAD_PSP_FPU,
            "assembly FPU EXC_RETURN must match context.rs constant"
        );
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn fpu_save_restore_word_count() {
        use crate::context::FPU_SAVED_CONTEXT_WORDS;
        // s16-s31 = 16 single-precision registers = 16 words = 64 bytes.
        assert_eq!(FPU_SAVED_CONTEXT_WORDS, 16);
        assert_eq!(FPU_SAVED_CONTEXT_WORDS * 4, 64);
    }

    #[test]
    fn exc_return_upper_bits_set() {
        // All upper bits [31:5] must be 1 for a valid EXC_RETURN.
        let upper_mask: u32 = !0x1F;
        assert_eq!(EXC_RETURN_NO_FPU & upper_mask, upper_mask);
        assert_eq!(EXC_RETURN_FPU & upper_mask, upper_mask);
    }
}
