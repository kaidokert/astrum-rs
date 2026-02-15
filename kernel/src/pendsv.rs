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
//! The binary must define `PARTITION_SP: [u32; N]` as `#[no_mangle] static mut`.
//! The binary must also invoke [`define_unified_kernel!`] which provides the
//! `get_current_partition()`, `get_next_partition()`, and `set_current_partition()`
//! shims for accessing kernel state.
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
/// 2. Saves r4-r11 via `stmdb`; stores PSP to `PARTITION_SP[current]`.
/// 3. Calls `get_next_partition()`, then `set_current_partition(next)`.
/// 4. Restores r4-r11 via `ldmia`; sets PSP.
/// 5. Sets `CONTROL.nPRIV = 1` + ISB; returns with EXC_RETURN=0xFFFFFFFD.
#[macro_export]
macro_rules! define_pendsv {
    () => {
        #[cfg(target_arch = "arm")]
        // SAFETY: PendSV exception handler — accesses kernel via Rust shims
        // (interrupt::free) and static mut PARTITION_SP owned by binary crate.
        // No aliasing: PendSV cannot preempt itself.
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            .global PendSV
            .type PendSV, %function

        PendSV:
            push    {{lr}}
            bl      get_current_partition
            pop     {{lr}}
            mov     r1, r0

            cmp     r1, #0xFF
            beq     .Lpendsv_skip_save

            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            str     r3, [r2, r0]

        .Lpendsv_skip_save:
            push    {{lr}}
            bl      get_next_partition
            pop     {{lr}}
            mov     r1, r0

            push    {{r1, lr}}
            mov     r0, r1
            bl      set_current_partition
            pop     {{r1, lr}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            ldr     r3, [r2, r0]

            ldmia   r3!, {{r4-r11}}

            msr     psp, r3

            /* Set CONTROL.nPRIV so the partition runs unprivileged. */
            mrs     r0, CONTROL
            orr     r0, r0, #1
            msr     CONTROL, r0
            isb

            ldr     lr, =0xFFFFFFFD
            bx      lr

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
        // Rust shims, static mut PARTITION_SP, and __pendsv_program_mpu for MPU.
        // No aliasing: PendSV is lowest priority and cannot preempt itself.
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            .global PendSV
            .type PendSV, %function

        PendSV:
            push    {{lr}}
            bl      get_current_partition
            pop     {{lr}}
            mov     r1, r0

            cmp     r1, #0xFF
            beq     .Lpendsv_dyn_skip_save

            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            str     r3, [r2, r0]

        .Lpendsv_dyn_skip_save:
            push    {{lr}}
            bl      __pendsv_program_mpu
            bl      get_next_partition
            pop     {{lr}}
            mov     r1, r0

            push    {{r1, lr}}
            mov     r0, r1
            bl      set_current_partition
            pop     {{r1, lr}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            ldr     r3, [r2, r0]

            ldmia   r3!, {{r4-r11}}

            msr     psp, r3

            /* Set CONTROL.nPRIV so the partition runs unprivileged. */
            mrs     r0, CONTROL
            orr     r0, r0, #1
            msr     CONTROL, r0
            isb

            ldr     lr, =0xFFFFFFFD
            bx      lr

            .size PendSV, . - PendSV
        "#
        );
    };
}
