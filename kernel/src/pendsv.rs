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
//! The binary must define the following `#[no_mangle] static mut` symbols:
//!
//! - `CURRENT_PARTITION: u32` — index of the currently running partition
//!   (initialise to `u32::MAX` to indicate "no partition yet").
//! - `NEXT_PARTITION: u32` — index of the partition to switch to next.
//! - `PARTITION_SP: [u32; N]` — saved stack-pointer (PSP) for each partition.
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
/// 1. Skips saving if `CURRENT_PARTITION == 0xFFFF_FFFF` (first switch).
/// 2. Saves r4-r11 onto the current partition's stack via `stmdb`.
/// 3. Stores the updated PSP into `PARTITION_SP[current]`.
/// 4. Loads `NEXT_PARTITION`, sets `CURRENT_PARTITION = NEXT_PARTITION`.
/// 5. Restores r4-r11 from the next partition's stack via `ldmia`.
/// 6. Sets PSP.
/// 7. Sets `CONTROL.nPRIV = 1` so the partition executes unprivileged,
///    followed by an `ISB` to ensure the pipeline observes the new
///    privilege level before the `bx lr`.
/// 8. Returns with `EXC_RETURN = 0xFFFFFFFD` (Thread/PSP).
#[macro_export]
macro_rules! define_pendsv {
    () => {
        #[cfg(target_arch = "arm")]
        // SAFETY: This inline assembly implements the PendSV exception handler,
        // which is the sole context-switch path for partitions. It executes at
        // exception priority and accesses only the global symbols
        // CURRENT_PARTITION, NEXT_PARTITION, and PARTITION_SP — all of which
        // are `#[no_mangle] static mut` owned by the binary crate and
        // exclusively mutated inside this handler.  The handler saves/restores
        // r4-r11 and PSP following the ARM calling convention for exception
        // entry, and sets CONTROL.nPRIV before returning to Thread mode so the
        // partition runs unprivileged.  No Rust aliasing rules are violated
        // because PendSV cannot preempt itself and SysTick only writes
        // NEXT_PARTITION (a plain u32 store, atomic on Cortex-M).
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            .global PendSV
            .type PendSV, %function

        PendSV:
            ldr     r0, =CURRENT_PARTITION
            ldr     r1, [r0]
            ldr     r2, =0xFFFFFFFF
            cmp     r1, r2
            beq     .Lpendsv_skip_save

            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            str     r3, [r2, r0]

        .Lpendsv_skip_save:
            ldr     r0, =NEXT_PARTITION
            ldr     r1, [r0]

            ldr     r0, =CURRENT_PARTITION
            str     r1, [r0]

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
        // SAFETY: This inline assembly implements the PendSV exception handler
        // for the dynamic-MPU variant.  Same invariants as `define_pendsv!`
        // apply: it runs at exception priority, exclusively accesses the
        // `#[no_mangle] static mut` symbols CURRENT_PARTITION,
        // NEXT_PARTITION, and PARTITION_SP, and saves/restores r4-r11 and PSP
        // per the ARM exception-entry convention.  Additionally, it calls the
        // Rust shim `__pendsv_program_mpu` (defined above) to reconfigure MPU
        // regions while still in handler mode, which is safe because PendSV is
        // the lowest-priority exception and holds exclusive access to the MPU
        // peripheral.  CONTROL.nPRIV is set before returning to Thread mode so
        // the partition runs unprivileged.
        core::arch::global_asm!(
            r#"
            .syntax unified
            .thumb

            .global PendSV
            .type PendSV, %function

        PendSV:
            ldr     r0, =CURRENT_PARTITION
            ldr     r1, [r0]
            ldr     r2, =0xFFFFFFFF
            cmp     r1, r2
            beq     .Lpendsv_dyn_skip_save

            mrs     r3, psp
            stmdb   r3!, {{r4-r11}}

            ldr     r2, =PARTITION_SP
            lsl     r0, r1, #2
            str     r3, [r2, r0]

        .Lpendsv_dyn_skip_save:
            /* Program dynamic MPU regions for the incoming partition. */
            push    {{lr}}
            bl      __pendsv_program_mpu
            pop     {{lr}}

            ldr     r0, =NEXT_PARTITION
            ldr     r1, [r0]

            ldr     r0, =CURRENT_PARTITION
            str     r1, [r0]

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
