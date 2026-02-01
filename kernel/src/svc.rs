use crate::context::ExceptionFrame;
use crate::syscall::SyscallId;

// TODO: cortex-m-rt's #[exception] macro requires SVCall handlers to have
// signature `fn() [-> !]` — it cannot pass the exception frame. Because we
// need the PSP-based ExceptionFrame for syscall dispatch, an assembly
// trampoline is architecturally required here. If a future cortex-m-rt
// version adds frame support for SVCall (as it does for HardFault), this
// should be migrated to #[exception].
#[cfg(all(target_arch = "arm", not(test)))]
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".global SVCall",
    ".type SVCall, %function",
    "SVCall:",
    "mrs r0, psp",
    "push {{lr}}",
    "bl dispatch_svc",
    "pop {{pc}}",
    ".size SVCall, . - SVCall",
);

/// Reference this function pointer to ensure the linker includes the SVCall
/// assembly trampoline and `dispatch_svc` in the final binary. Without an
/// explicit Rust-level reference, the linker may discard the object.
pub static SVC_HANDLER: unsafe extern "C" fn(&mut ExceptionFrame) = dispatch_svc;

/// Dispatch an SVC call based on the syscall number in `frame.r0`.
///
/// # Safety
///
/// The caller must pass a valid pointer to the hardware-stacked
/// `ExceptionFrame` on the process stack (PSP). This is guaranteed
/// when called from the SVCall assembly trampoline above.
#[no_mangle]
pub unsafe extern "C" fn dispatch_svc(frame: &mut ExceptionFrame) {
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(_) => 1,
        None => u32::MAX,
    };
}

fn handle_yield() -> u32 {
    #[cfg(not(test))]
    {
        cortex_m::peripheral::SCB::set_pendsv();
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syscall::{SYS_EVT_WAIT, SYS_YIELD};

    #[rustfmt::skip]
    fn f(r0: u32) -> ExceptionFrame {
        ExceptionFrame { r0, r1: 0xAA, r2: 0xBB, r3: 0xCC, r12: 0, lr: 0, pc: 0, xpsr: 0 }
    }
    #[test]
    fn yield_returns_zero_and_preserves_regs() {
        let mut ef = f(SYS_YIELD);
        unsafe { dispatch_svc(&mut ef) };
        assert_eq!((ef.r0, ef.r1, ef.r2, ef.r3), (0, 0xAA, 0xBB, 0xCC));
    }
    #[test]
    fn invalid_syscall_returns_max() {
        let mut ef = f(0xFFFF);
        unsafe { dispatch_svc(&mut ef) };
        assert_eq!(ef.r0, u32::MAX);
    }
    #[test]
    fn unimplemented_syscall_returns_one() {
        let mut ef = f(SYS_EVT_WAIT);
        unsafe { dispatch_svc(&mut ef) };
        assert_eq!(ef.r0, 1);
    }
}
