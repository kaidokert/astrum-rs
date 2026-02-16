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
//! The context save/restore logic is implemented as linkable functions in
//! [`pendsv_asm`](crate::pendsv_asm):
//! - `pendsv_context_save(idx)` — saves r4-r11 and PSP
//! - `pendsv_context_restore(idx)` — restores r4-r11 and PSP (with null-pointer check)
//! - `pendsv_return_unprivileged()` — sets CONTROL.nPRIV=1 and returns
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
/// 2. Calls `pendsv_context_save(idx)` to save r4-r11 and PSP.
/// 3. Calls `get_next_partition()`, then `set_current_partition(next)`.
/// 4. Calls `pendsv_context_restore(idx)` to restore r4-r11 and PSP.
/// 5. Calls `pendsv_return_unprivileged()` to set CONTROL.nPRIV=1 and return.
///
/// # Implementation Note
///
/// The context save/restore logic is implemented as linkable functions in
/// [`pendsv_asm`](crate::pendsv_asm). These functions are shared between
/// `define_pendsv!` and `define_pendsv_dynamic!`.
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

            .global PendSV
            .type PendSV, %function
            .thumb_func

        PendSV:
            bl      get_current_partition
            mov     r4, r0              /* r4 = current partition index (callee-saved) */

            cmp     r4, #0xFF
            beq     .Lpendsv_skip_save

            mov     r0, r4
            bl      pendsv_context_save

        .Lpendsv_skip_save:
            bl      get_next_partition
            mov     r4, r0              /* r4 = next partition index */

            bl      set_current_partition

            mov     r0, r4
            bl      pendsv_context_restore

            /* pendsv_return_unprivileged does not return */
            bl      pendsv_return_unprivileged

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
/// The context save/restore logic is implemented as linkable functions in
/// [`pendsv_asm`](crate::pendsv_asm). These functions are shared between
/// `define_pendsv!` and `define_pendsv_dynamic!`.
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

            .global PendSV
            .type PendSV, %function
            .thumb_func

        PendSV:
            bl      get_current_partition
            mov     r4, r0              /* r4 = current partition index (callee-saved) */

            cmp     r4, #0xFF
            beq     .Lpendsv_dyn_skip_save

            mov     r0, r4
            bl      pendsv_context_save

        .Lpendsv_dyn_skip_save:
            bl      __pendsv_program_mpu
            bl      get_next_partition
            mov     r4, r0              /* r4 = next partition index */

            bl      set_current_partition

            mov     r0, r4
            bl      pendsv_context_restore

            /* pendsv_return_unprivileged does not return */
            bl      pendsv_return_unprivileged

            .size PendSV, . - PendSV
        "#
        );
    };
}
