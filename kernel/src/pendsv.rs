//! PendSV context-switch handler for Cortex-M partitions.
//!
//! Provides reusable macros that emit the PendSV handler assembly.
//! This avoids each example duplicating the context-switch logic while
//! keeping the assembly out of binaries that do not need it (since
//! `global_asm!` in a library crate is linked into every binary).
//!
//! - [`define_pendsv!`] — standard context switch (register save/restore only).
//! - [`define_pendsv_dynamic!`] — context switch that also programs dynamic
//!   MPU regions R4-R7 via a [`DynamicStrategy`](crate::mpu_strategy::DynamicStrategy).
//!
//! # Requirements
//!
//! The binary must invoke [`define_unified_kernel!`] which provides the
//! Rust shims for accessing kernel state:
//! - `get_current_partition()` — returns current partition index
//! - `get_next_partition()` — returns next partition index
//! - `set_current_partition(pid)` — updates current partition
//! - `get_partition_sp(idx)` — reads saved SP for partition
//! - `set_partition_sp(idx, sp)` — saves SP for partition
//!
//! # Usage
//!
//! ```ignore
//! // Standard (no MPU):
//! kernel::define_pendsv!();
//!
//! // Dynamic MPU — pass the DynamicStrategy static:
//! kernel::define_pendsv_dynamic!(STRATEGY);
//! ```

/// Emit the PendSV context-switch handler via `global_asm!`.
///
/// This macro defines a single canonical implementation of the PendSV
/// handler. Invoke it once at the crate root of any binary that performs
/// partition context switches.
///
/// The handler:
/// 1. Calls `get_current_partition()` — skips save if 0xFF (first switch).
/// 2. Saves r4-r11 via `stmdb`; calls `get_partition_sp(idx)` to validate,
///    then `set_partition_sp(idx, sp)` to store PSP.
/// 3. Calls `get_next_partition()`, then `set_current_partition(next)`.
/// 4. Calls `get_partition_sp(idx)` to read saved SP; restores r4-r11 via `ldmia`.
/// 5. Sets `CONTROL.nPRIV = 1` + ISB; returns with EXC_RETURN=0xFFFFFFFD.
///
/// # Implementation Note
///
/// The context save/restore logic is shared with [`define_pendsv_dynamic!`]
/// via the `pendsv_context_save`, `pendsv_context_restore`, and
/// `pendsv_return_unprivileged` assembly macros defined inline. Any changes
/// to the context-switch logic should be made in both macros to maintain
/// consistency.
#[macro_export]
macro_rules! define_pendsv {
    () => {
        #[cfg(target_arch = "arm")]
        // SAFETY: PendSV exception handler — accesses kernel via Rust shims
        // (interrupt::free). No aliasing: PendSV cannot preempt itself.
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            /* ================================================================
             * Context Save Macro (shared with define_pendsv_dynamic!)
             * ================================================================
             * Expects: r1 = current partition index (not 0xFF)
             * Uses: r0, r1, r2, r3
             * Clobbers lr (caller must push/pop)
             *
             * The save path:
             *   1. mrs r3, psp
             *   2. stmdb r3!, {{r4-r11}}
             *   3. mov r0, <current_idx>; bl get_partition_sp  (validate)
             *   4. mov r0, <idx>; mov r1, <sp>; bl set_partition_sp
             */
            .macro pendsv_context_save
            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}     /* r3 now points to saved context */
            mov     r2, r3              /* r2 = sp to save (preserve across call) */

            /* get_partition_sp(current_idx) — validation call */
            mov     r0, r1              /* r0 = current partition index */
            push    {{r1, r2, lr}}
            bl      get_partition_sp
            pop     {{r1, r2, lr}}
            /* validation result in r0 (unused, but validates index) */

            /* set_partition_sp(current_idx, sp) */
            mov     r0, r1              /* r0 = current partition index */
            mov     r1, r2              /* r1 = stack pointer to save */
            push    {{lr}}
            bl      set_partition_sp
            pop     {{lr}}
            .endm

            /* ================================================================
             * Context Restore Macro (shared with define_pendsv_dynamic!)
             * ================================================================
             * Expects: r1 = next partition index
             * Uses: r0, r1, r3
             * Clobbers lr (caller must push/pop)
             * On exit: psp updated, r4-r11 restored
             */
            .macro pendsv_context_restore
            /* get_partition_sp(next_idx) */
            mov     r0, r1              /* r0 = next partition index */
            push    {{lr}}
            bl      get_partition_sp
            pop     {{lr}}
            mov     r3, r0              /* r3 = saved stack pointer */

            ldmia   r3!, {{r4-r11}}
            msr     psp, r3
            .endm

            /* ================================================================
             * Unprivileged Return Macro (shared with define_pendsv_dynamic!)
             * ================================================================
             * Sets CONTROL.nPRIV = 1, issues ISB, returns via EXC_RETURN.
             */
            .macro pendsv_return_unprivileged
            mrs     r0, CONTROL
            orr     r0, r0, #1
            msr     CONTROL, r0
            isb
            ldr     lr, =0xFFFFFFFD
            bx      lr
            .endm

            .global PendSV
            .type PendSV, %function

        PendSV:
            push    {{lr}}
            bl      get_current_partition
            pop     {{lr}}
            mov     r1, r0              /* r1 = current partition index */

            cmp     r1, #0xFF
            beq     .Lpendsv_skip_save

            pendsv_context_save

        .Lpendsv_skip_save:
            push    {{lr}}
            bl      get_next_partition
            pop     {{lr}}
            mov     r1, r0              /* r1 = next partition index */

            push    {{r1, lr}}
            mov     r0, r1
            bl      set_current_partition
            pop     {{r1, lr}}

            pendsv_context_restore
            pendsv_return_unprivileged

            .size PendSV, . - PendSV
        "#
        );
    };
}

/// Emit a PendSV handler that programs dynamic MPU regions before switching.
///
/// Like [`define_pendsv!`], but the generated handler calls a Rust shim
/// (`__pendsv_program_mpu`) that invokes
/// [`DynamicStrategy::program_regions`](crate::mpu_strategy::DynamicStrategy::program_regions)
/// just before restoring the incoming partition's context.  This ensures
/// the MPU is reconfigured **inside PendSV** — the lowest-priority
/// exception — rather than in SysTick, eliminating the race where code
/// runs with a stale memory map.
///
/// Like `define_pendsv!`, the handler also sets `CONTROL.nPRIV = 1`
/// before returning to Thread mode so the partition runs unprivileged.
///
/// # Arguments
///
/// `$strategy` — the identifier of a `static DynamicStrategy` that the
/// SysTick handler has already called `configure_partition` on.
///
/// # Requirements
///
/// Same symbol requirements as [`define_pendsv!`].
///
/// # Implementation Note
///
/// The context save/restore logic is shared with [`define_pendsv!`]
/// via the `pendsv_context_save`, `pendsv_context_restore`, and
/// `pendsv_return_unprivileged` assembly macros defined inline. Any changes
/// to the context-switch logic should be made in both macros to maintain
/// consistency.
///
/// # Example
///
/// ```ignore
/// static STRATEGY: DynamicStrategy = DynamicStrategy::new();
/// kernel::define_pendsv_dynamic!(STRATEGY);
/// ```
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
macro_rules! define_pendsv_dynamic {
    ($strategy:ident) => {
        /// Rust shim called from the PendSV assembly to program dynamic
        /// MPU regions R4-R7.  Exposed as `#[no_mangle]` so the assembly
        /// can `bl` to it.
        #[no_mangle]
        extern "C" fn __pendsv_program_mpu() {
            // SAFETY: PendSV is the lowest-priority exception, so no
            // other exception can preempt us while writing MPU registers.
            // `steal()` is sound because we have exclusive access to the
            // MPU peripheral at this priority level.
            let p = unsafe { cortex_m::Peripherals::steal() };
            $strategy.program_regions(&p.MPU);
        }

        #[cfg(target_arch = "arm")]
        // SAFETY: PendSV exception handler (dynamic-MPU) — accesses kernel via
        // Rust shims and __pendsv_program_mpu for MPU.
        // No aliasing: PendSV is lowest priority and cannot preempt itself.
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            /* ================================================================
             * Context Save Macro (shared with define_pendsv!)
             * ================================================================
             * Expects: r1 = current partition index (not 0xFF)
             * Uses: r0, r1, r2, r3
             * Clobbers lr (caller must push/pop)
             *
             * The save path:
             *   1. mrs r3, psp
             *   2. stmdb r3!, {{r4-r11}}
             *   3. mov r0, <current_idx>; bl get_partition_sp  (validate)
             *   4. mov r0, <idx>; mov r1, <sp>; bl set_partition_sp
             */
            .macro pendsv_context_save
            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}     /* r3 now points to saved context */
            mov     r2, r3              /* r2 = sp to save (preserve across call) */

            /* get_partition_sp(current_idx) — validation call */
            mov     r0, r1              /* r0 = current partition index */
            push    {{r1, r2, lr}}
            bl      get_partition_sp
            pop     {{r1, r2, lr}}
            /* validation result in r0 (unused, but validates index) */

            /* set_partition_sp(current_idx, sp) */
            mov     r0, r1              /* r0 = current partition index */
            mov     r1, r2              /* r1 = stack pointer to save */
            push    {{lr}}
            bl      set_partition_sp
            pop     {{lr}}
            .endm

            /* ================================================================
             * Context Restore Macro (shared with define_pendsv!)
             * ================================================================
             * Expects: r1 = next partition index
             * Uses: r0, r1, r3
             * Clobbers lr (caller must push/pop)
             * On exit: psp updated, r4-r11 restored
             */
            .macro pendsv_context_restore
            /* get_partition_sp(next_idx) */
            mov     r0, r1              /* r0 = next partition index */
            push    {{lr}}
            bl      get_partition_sp
            pop     {{lr}}
            mov     r3, r0              /* r3 = saved stack pointer */

            ldmia   r3!, {{r4-r11}}
            msr     psp, r3
            .endm

            /* ================================================================
             * Unprivileged Return Macro (shared with define_pendsv!)
             * ================================================================
             * Sets CONTROL.nPRIV = 1, issues ISB, returns via EXC_RETURN.
             */
            .macro pendsv_return_unprivileged
            mrs     r0, CONTROL
            orr     r0, r0, #1
            msr     CONTROL, r0
            isb
            ldr     lr, =0xFFFFFFFD
            bx      lr
            .endm

            .global PendSV
            .type PendSV, %function

        PendSV:
            push    {{lr}}
            bl      get_current_partition
            pop     {{lr}}
            mov     r1, r0              /* r1 = current partition index */

            cmp     r1, #0xFF
            beq     .Lpendsv_dyn_skip_save

            pendsv_context_save

        .Lpendsv_dyn_skip_save:
            push    {{lr}}
            bl      __pendsv_program_mpu
            bl      get_next_partition
            pop     {{lr}}
            mov     r1, r0              /* r1 = next partition index */

            push    {{r1, lr}}
            mov     r0, r1
            bl      set_current_partition
            pop     {{r1, lr}}

            pendsv_context_restore
            pendsv_return_unprivileged

            .size PendSV, . - PendSV
        "#
        );
    };
}
