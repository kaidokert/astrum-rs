//! Cortex-M exception and context switch frame definitions.

use crate::partition::EntryAddr;

/// Hardware-stacked exception frame (ascending address order: r0 at lowest).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExceptionFrame {
    pub r0: u32,
    pub r1: u32,
    pub r2: u32,
    pub r3: u32,
    pub r12: u32,
    pub lr: u32,
    pub pc: u32,
    pub xpsr: u32,
}

/// Software-saved callee-saved registers (r4-r11).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SavedContext {
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
}

// SavedContext must be exactly 32 bytes (8 × u32) per architecture.md.
// Checked here (not in define_unified_kernel!) because SavedContext is a
// fixed architecture-defined struct independent of any kernel configuration.
crate::const_assert!(
    core::mem::size_of::<SavedContext>() == 32,
    "SavedContext must be exactly 32 bytes (8 x u32)"
);

/// Number of u32 words in a SavedContext (r4-r11).
pub const SAVED_CONTEXT_WORDS: usize = 8;

/// Total words in a SavedContext + ExceptionFrame pair (8 + 8).
pub const CONTEXT_FRAME_WORDS: usize = 16;

pub const XPSR_THUMB: u32 = 1 << 24; // Thumb bit — required on all Cortex-M

/// EXC_RETURN value: return to Thread mode using PSP.
///
/// On Cortex-M, when the processor enters an exception, the LR is loaded with
/// a special EXC_RETURN value. Writing 0xFFFFFFFD to LR and executing BX LR
/// causes an exception return to Thread mode, restoring context from the PSP.
pub const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;

// ---------------------------------------------------------------------------
// FPU-aware frame definitions (Cortex-M4F+)
// ---------------------------------------------------------------------------

/// Software-saved FPU callee-saved registers (s16-s31).
#[cfg(feature = "fpu-context")]
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FpuSavedContext {
    pub s16: u32,
    pub s17: u32,
    pub s18: u32,
    pub s19: u32,
    pub s20: u32,
    pub s21: u32,
    pub s22: u32,
    pub s23: u32,
    pub s24: u32,
    pub s25: u32,
    pub s26: u32,
    pub s27: u32,
    pub s28: u32,
    pub s29: u32,
    pub s30: u32,
    pub s31: u32,
}

#[cfg(feature = "fpu-context")]
crate::const_assert!(
    core::mem::size_of::<FpuSavedContext>() == 64,
    "FpuSavedContext must be exactly 64 bytes (16 x u32)"
);

/// EXC_RETURN for Thread mode, PSP, with FPU frame (bit 4 cleared).
///
/// Bit 4 = 0 indicates that the stacked frame includes the extended FPU
/// registers (s0-s15, FPSCR). 0xFFFFFFED = 0xFFFFFFFD with bit 4 cleared.
#[cfg(feature = "fpu-context")]
pub const EXC_RETURN_THREAD_PSP_FPU: u32 = 0xFFFF_FFED;

/// Number of u32 words in an FpuSavedContext (s16-s31).
#[cfg(feature = "fpu-context")]
pub const FPU_SAVED_CONTEXT_WORDS: usize = 16;

/// Extended hardware frame words: s0-s15 (16) + FPSCR (1) + reserved (1) = 18.
// TODO: consider making public if external assembly or debugging tools need the frame size
#[cfg(feature = "fpu-context")]
const FPU_HW_EXTENDED_WORDS: usize = 18;

/// Total words in a full FPU context frame:
/// 8 (r4-r11) + 16 (s16-s31) + 8 (r0-r3,r12,lr,pc,xpsr) + 18 (s0-s15,FPSCR,reserved) = 50.
#[cfg(feature = "fpu-context")]
pub const FPU_CONTEXT_FRAME_WORDS: usize = 50;

// TODO: populate FpuSavedContext through a type-safe interface instead of raw offset writes

/// Write an initial context-switch frame at the top of `stack` so PendSV can
/// "return" into `entry_point`. Returns the stack-pointer index, or `None`
/// if too small.
///
/// Standard layout (no FPU):
///   `[r4..r11 | r0..r3, r12, lr, pc, xpsr]`
///
/// FPU layout (fpu-context enabled):
///   `[s16..s31 | r4..r11 | r0..r3, r12, lr, pc, xpsr | s0..s15, FPSCR, reserved]`
pub fn init_stack_frame(
    stack: &mut [u32],
    entry_point: impl Into<EntryAddr>,
    r0_arg: Option<u32>,
) -> Option<usize> {
    let addr = entry_point.into().raw();
    let len = stack.len();

    #[cfg(feature = "fpu-context")]
    let frame_words = FPU_CONTEXT_FRAME_WORDS;
    #[cfg(not(feature = "fpu-context"))]
    let frame_words = CONTEXT_FRAME_WORDS;

    if len < frame_words {
        return None;
    }
    let base = len - frame_words;

    // FPU: write s16-s31 software-saved registers at base
    #[cfg(feature = "fpu-context")]
    stack.get_mut(base..base + FPU_SAVED_CONTEXT_WORDS)?.fill(0);

    // Offset for core registers depends on whether FPU callee block precedes them
    #[cfg(feature = "fpu-context")]
    let core_base = base + FPU_SAVED_CONTEXT_WORDS;
    #[cfg(not(feature = "fpu-context"))]
    let core_base = base;

    // SavedContext (r4-r11): all zero
    stack
        .get_mut(core_base..core_base + SAVED_CONTEXT_WORDS)?
        .fill(0);

    // ExceptionFrame: r0, r1, r2, r3, r12, lr, pc, xpsr
    let ef = core_base + SAVED_CONTEXT_WORDS;

    #[cfg(feature = "fpu-context")]
    let lr_val = EXC_RETURN_THREAD_PSP_FPU;
    #[cfg(not(feature = "fpu-context"))]
    let lr_val = EXC_RETURN_THREAD_PSP;

    *stack.get_mut(ef)? = r0_arg.unwrap_or(0); // r0
    *stack.get_mut(ef + 1)? = 0; // r1
    *stack.get_mut(ef + 2)? = 0; // r2
    *stack.get_mut(ef + 3)? = 0; // r3
    *stack.get_mut(ef + 4)? = 0; // r12
    *stack.get_mut(ef + 5)? = lr_val; // lr
    *stack.get_mut(ef + 6)? = addr; // pc
    *stack.get_mut(ef + 7)? = XPSR_THUMB; // xpsr

    // FPU: extended hw frame (s0-s15, FPSCR, reserved) — all zero
    #[cfg(feature = "fpu-context")]
    {
        let fpu_hw = ef + 8;
        stack
            .get_mut(fpu_hw..fpu_hw + FPU_HW_EXTENDED_WORDS)?
            .fill(0);
    }

    Some(base)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem;

    #[test]
    fn struct_sizes_are_32_bytes() {
        assert_eq!(mem::size_of::<ExceptionFrame>(), 32);
        assert_eq!(mem::size_of::<SavedContext>(), 32);
    }

    #[test]
    fn field_offsets_match_arm_stacking_order() {
        assert_eq!(mem::offset_of!(ExceptionFrame, r0), 0);
        assert_eq!(mem::offset_of!(ExceptionFrame, r3), 12);
        assert_eq!(mem::offset_of!(ExceptionFrame, r12), 16);
        assert_eq!(mem::offset_of!(ExceptionFrame, lr), 20);
        assert_eq!(mem::offset_of!(ExceptionFrame, pc), 24);
        assert_eq!(mem::offset_of!(ExceptionFrame, xpsr), 28);
        assert_eq!(mem::offset_of!(SavedContext, r4), 0);
        assert_eq!(mem::offset_of!(SavedContext, r7), 12);
        assert_eq!(mem::offset_of!(SavedContext, r8), 16);
        assert_eq!(mem::offset_of!(SavedContext, r11), 28);
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_stack_frame_writes_correct_values() {
        let mut stack = [0xDEAD_BEEFu32; 32];
        let entry = 0x0800_0101u32;
        let sp = init_stack_frame(&mut stack, entry, None).unwrap();
        assert_eq!(sp, 16); // 32 - 16
                            // SavedContext: r4-r11 all zero
        for i in 0..SAVED_CONTEXT_WORDS {
            assert_eq!(stack[sp + i], 0, "saved r{} != 0", i + 4);
        }
        // ExceptionFrame fields
        let ef = sp + SAVED_CONTEXT_WORDS;
        assert_eq!(stack[ef], 0); // r0
        assert_eq!(stack[ef + 5], EXC_RETURN_THREAD_PSP); // lr
        assert_eq!(stack[ef + 6], entry); // pc
        assert_eq!(stack[ef + 7], 0x0100_0000); // xpsr
        assert_eq!(stack[0], 0xDEAD_BEEF); // below frame untouched
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_stack_frame_with_r0_arg() {
        let mut stack = [0u32; 32];
        let sp = init_stack_frame(&mut stack, 0x0800_0201, Some(42)).unwrap();
        assert_eq!(stack[sp + SAVED_CONTEXT_WORDS], 42); // r0
        assert_eq!(stack[sp + SAVED_CONTEXT_WORDS + 6], 0x0800_0201); // pc
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_stack_frame_exact_fit() {
        let mut stack = [0u32; CONTEXT_FRAME_WORDS];
        let sp = init_stack_frame(&mut stack, 0x100, None).unwrap();
        assert_eq!(sp, 0);
        assert_eq!(stack[SAVED_CONTEXT_WORDS + 6], 0x100); // pc
        assert_eq!(stack[SAVED_CONTEXT_WORDS + 7], 0x0100_0000); // xpsr
    }

    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn init_stack_frame_too_small_and_empty() {
        let mut small = [0u32; CONTEXT_FRAME_WORDS - 1];
        assert!(init_stack_frame(&mut small, 0x100, None).is_none());
        let mut empty: [u32; 0] = [];
        assert!(init_stack_frame(&mut empty, 0x100, None).is_none());
    }

    #[test]
    fn exc_return_thread_psp_bit_fields() {
        // Exact value: return to Thread mode using PSP.
        assert_eq!(EXC_RETURN_THREAD_PSP, 0xFFFF_FFFD);

        // Bit 0 set: return to Thread mode (not Handler mode).
        assert_ne!(
            EXC_RETURN_THREAD_PSP & (1 << 0),
            0,
            "bit 0 (Thread mode) must be set"
        );

        // Bit 2 set: restore context from PSP (not MSP).
        assert_ne!(
            EXC_RETURN_THREAD_PSP & (1 << 2),
            0,
            "bit 2 (PSP) must be set"
        );

        // Bits [31:4] must all be ones (EXC_RETURN magic prefix).
        assert_eq!(
            EXC_RETURN_THREAD_PSP & 0xFFFF_FFF0,
            0xFFFF_FFF0,
            "bits [31:4] must all be ones"
        );
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn fpu_saved_context_size_and_offsets() {
        assert_eq!(mem::size_of::<FpuSavedContext>(), 64);
        assert_eq!(mem::offset_of!(FpuSavedContext, s16), 0);
        assert_eq!(mem::offset_of!(FpuSavedContext, s17), 4);
        assert_eq!(mem::offset_of!(FpuSavedContext, s23), 28);
        assert_eq!(mem::offset_of!(FpuSavedContext, s24), 32);
        assert_eq!(mem::offset_of!(FpuSavedContext, s31), 60);
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn exc_return_fpu_bit_fields() {
        assert_eq!(EXC_RETURN_THREAD_PSP_FPU, 0xFFFF_FFED);
        // Bit 4 cleared: extended FPU frame.
        assert_eq!(
            EXC_RETURN_THREAD_PSP_FPU & (1 << 4),
            0,
            "bit 4 must be cleared"
        );
        // Bit 0 set: Thread mode.
        assert_ne!(
            EXC_RETURN_THREAD_PSP_FPU & (1 << 0),
            0,
            "bit 0 (Thread) must be set"
        );
        // Bit 2 set: PSP.
        assert_ne!(
            EXC_RETURN_THREAD_PSP_FPU & (1 << 2),
            0,
            "bit 2 (PSP) must be set"
        );
        // Only bit 4 differs from non-FPU variant.
        assert_eq!(EXC_RETURN_THREAD_PSP ^ EXC_RETURN_THREAD_PSP_FPU, 1 << 4);
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn fpu_context_frame_words_constant() {
        assert_eq!(FPU_CONTEXT_FRAME_WORDS, 50);
        assert_eq!(FPU_SAVED_CONTEXT_WORDS, 16);
        assert_eq!(
            FPU_CONTEXT_FRAME_WORDS,
            SAVED_CONTEXT_WORDS + FPU_SAVED_CONTEXT_WORDS + 8 + FPU_HW_EXTENDED_WORDS
        );
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn init_stack_frame_fpu_layout() {
        let mut stack = [0xDEAD_BEEFu32; 64];
        let entry = 0x0800_0101u32;
        let sp = init_stack_frame(&mut stack, entry, Some(7)).unwrap();
        assert_eq!(sp, 64 - FPU_CONTEXT_FRAME_WORDS); // 14

        // s16-s31 (16 words) at base — all zero
        for i in 0..FPU_SAVED_CONTEXT_WORDS {
            assert_eq!(stack[sp + i], 0, "s{} != 0", i + 16);
        }

        // r4-r11 (8 words) after FPU saved — all zero
        let core_base = sp + FPU_SAVED_CONTEXT_WORDS;
        for i in 0..SAVED_CONTEXT_WORDS {
            assert_eq!(stack[core_base + i], 0, "r{} != 0", i + 4);
        }

        // ExceptionFrame
        let ef = core_base + SAVED_CONTEXT_WORDS;
        assert_eq!(stack[ef], 7); // r0
        assert_eq!(stack[ef + 1], 0); // r1
        assert_eq!(stack[ef + 4], 0); // r12
        assert_eq!(stack[ef + 5], EXC_RETURN_THREAD_PSP_FPU); // lr
        assert_eq!(stack[ef + 6], entry); // pc
        assert_eq!(stack[ef + 7], XPSR_THUMB); // xpsr

        // Extended FPU hw frame: s0-s15, FPSCR, reserved (18 words) — all zero
        let fpu_hw = ef + 8;
        for i in 0..18 {
            assert_eq!(stack[fpu_hw + i], 0, "fpu hw word {} != 0", i);
        }

        // Verify frame ends at top of stack
        assert_eq!(fpu_hw + 18, 64);

        // Below the frame is untouched
        assert_eq!(stack[sp - 1], 0xDEAD_BEEF);
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn init_stack_frame_fpu_exact_fit() {
        let mut stack = [0u32; FPU_CONTEXT_FRAME_WORDS];
        let sp = init_stack_frame(&mut stack, 0x100, None).unwrap();
        assert_eq!(sp, 0);
        // pc at expected offset
        let ef = FPU_SAVED_CONTEXT_WORDS + SAVED_CONTEXT_WORDS;
        assert_eq!(stack[ef + 6], 0x100);
        assert_eq!(stack[ef + 7], XPSR_THUMB);
    }

    #[cfg(feature = "fpu-context")]
    #[test]
    fn init_stack_frame_fpu_too_small() {
        let mut small = [0u32; FPU_CONTEXT_FRAME_WORDS - 1];
        assert!(init_stack_frame(&mut small, 0x100, None).is_none());
    }
}
