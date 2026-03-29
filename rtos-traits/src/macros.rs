//! User-space SVC macros for issuing system calls.

/// Issue an SVC #0 system call.  Returns the kernel's `r0` value.
/// On non-ARM hosts returns `0` (allows `cargo test` on the host).
#[macro_export]
macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: Inline asm issues an SVC #0 with the caller-supplied
        // syscall ID and arguments in r0-r3.  The kernel's SVC handler
        // preserves all registers except r0 (return value) and r12
        // (clobbered by the Cortex-M exception entry).  The operand
        // constraints match this ABI exactly.
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") $id => r, in("r1") $a,
                in("r2") $b, in("r3") $c, out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        { let _ = ($id, $a, $b, $c); r = 0; }
        r
    }};
}

/// Issue an SVC #0 returning `(r0, r1)`.  Host stub returns `(0, 0)`.
#[macro_export]
macro_rules! svc_r01 {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r0: u32;
        let r1: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: Same ABI contract as `svc!` above, but r1 is also an
        // output (the kernel returns a second value in r1 for two-word
        // results).  Constraints mirror the kernel's SVC return ABI.
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") $id => r0, inout("r1") $a => r1,
                in("r2") $b, in("r3") $c, out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        { let _ = ($id, $a, $b, $c); r0 = 0; r1 = 0; }
        (r0, r1)
    }};
}

/// Issue an SVC #0 returning `(r0, r1, r2, r3)`.  Host stub returns `(0, 0, 0, 0)`.
#[macro_export]
macro_rules! svc_r0123 {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r0: u32;
        let r1: u32;
        let r2: u32;
        let r3: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: Same ABI contract as `svc!` above, but r1-r3 are also
        // outputs (the kernel returns four values in r0-r3 for quad-word
        // results).  Constraints mirror the kernel's SVC return ABI.
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") $id => r0, inout("r1") $a => r1,
                inout("r2") $b => r2, inout("r3") $c => r3, out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        { let _ = ($id, $a, $b, $c); r0 = 0; r1 = 0; r2 = 0; r3 = 0; }
        (r0, r1, r2, r3)
    }};
}

#[cfg(test)]
mod tests {
    #[test]
    fn svc_returns_zero_on_host() {
        assert_eq!(svc!(1u32, 2u32, 3u32, 4u32), 0);
    }

    #[test]
    fn svc_r01_returns_zero_pair_on_host() {
        let (r0, r1) = svc_r01!(1u32, 2u32, 3u32, 4u32);
        assert_eq!(r0, 0);
        assert_eq!(r1, 0);
    }

    #[test]
    fn svc_r0123_returns_zero_quad_on_host() {
        let (r0, r1, r2, r3) = svc_r0123!(1u32, 2u32, 3u32, 4u32);
        assert_eq!(r0, 0);
        assert_eq!(r1, 0);
        assert_eq!(r2, 0);
        assert_eq!(r3, 0);
    }
}
