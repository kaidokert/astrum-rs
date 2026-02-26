//! Cortex-M exception and context switch frame definitions.

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

/// Number of u32 words in a SavedContext (r4-r11).
pub const SAVED_CONTEXT_WORDS: usize = 8;

/// Total words in a SavedContext + ExceptionFrame pair (8 + 8).
pub const CONTEXT_FRAME_WORDS: usize = 16;

const XPSR_THUMB: u32 = 1 << 24; // Thumb bit — required on all Cortex-M

/// EXC_RETURN value: return to Thread mode using PSP.
///
/// On Cortex-M, when the processor enters an exception, the LR is loaded with
/// a special EXC_RETURN value. Writing 0xFFFFFFFD to LR and executing BX LR
/// causes an exception return to Thread mode, restoring context from the PSP.
pub const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;

/// Write an initial context-switch frame at the top of `stack` so PendSV can
/// "return" into `entry_point`. Returns the stack-pointer index, or `None`
/// if too small. Layout: `[r4..r11 | r0..r3, r12, lr, pc, xpsr]`.
pub fn init_stack_frame(stack: &mut [u32], entry_point: u32, r0_arg: Option<u32>) -> Option<usize> {
    let len = stack.len();
    if len < CONTEXT_FRAME_WORDS {
        return None;
    }
    let base = len - CONTEXT_FRAME_WORDS;
    // SavedContext (r4-r11): all zero
    stack[base..base + SAVED_CONTEXT_WORDS].fill(0);
    // ExceptionFrame: r0, r1, r2, r3, r12, lr, pc, xpsr
    let ef = base + SAVED_CONTEXT_WORDS;
    stack[ef] = r0_arg.unwrap_or(0); // r0
    stack[ef + 1] = 0; // r1
    stack[ef + 2] = 0; // r2
    stack[ef + 3] = 0; // r3
    stack[ef + 4] = 0; // r12
    stack[ef + 5] = EXC_RETURN_THREAD_PSP; // lr
    stack[ef + 6] = entry_point; // pc
    stack[ef + 7] = XPSR_THUMB; // xpsr
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

    #[test]
    fn init_stack_frame_with_r0_arg() {
        let mut stack = [0u32; 32];
        let sp = init_stack_frame(&mut stack, 0x0800_0201, Some(42)).unwrap();
        assert_eq!(stack[sp + SAVED_CONTEXT_WORDS], 42); // r0
        assert_eq!(stack[sp + SAVED_CONTEXT_WORDS + 6], 0x0800_0201); // pc
    }

    #[test]
    fn init_stack_frame_exact_fit() {
        let mut stack = [0u32; CONTEXT_FRAME_WORDS];
        let sp = init_stack_frame(&mut stack, 0x100, None).unwrap();
        assert_eq!(sp, 0);
        assert_eq!(stack[SAVED_CONTEXT_WORDS + 6], 0x100); // pc
        assert_eq!(stack[SAVED_CONTEXT_WORDS + 7], 0x0100_0000); // xpsr
    }

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
}
