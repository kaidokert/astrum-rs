//! PendSV context-switch handler for Cortex-M partitions.
//!
//! Provides a reusable macro [`define_pendsv!`] that emits the standard
//! PendSV handler assembly. This avoids each example duplicating the
//! context-switch logic while keeping the assembly out of binaries that
//! do not need it (since `global_asm!` in a library crate is linked into
//! every binary).
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
//! kernel::define_pendsv!();
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
/// 6. Sets PSP and returns with `EXC_RETURN = 0xFFFFFFFD` (Thread/PSP).
#[macro_export]
macro_rules! define_pendsv {
    () => {
        #[cfg(target_arch = "arm")]
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

            ldr     lr, =0xFFFFFFFD
            bx      lr

            .size PendSV, . - PendSV
        "#
        );
    };
}
