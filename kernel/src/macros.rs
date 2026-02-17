//! Shared user-space helper macros for issuing SVCs and reading partition
//! entry arguments.
//!
//! These macros are `#[macro_export]`-ed so that examples (and eventually
//! real partitions) can call `kernel::svc!` / `kernel::unpack_r0!` instead
//! of duplicating inline-asm boilerplate.

/// Issue an SVC #0 system call, passing a syscall ID in `r0` and up to
/// three arguments in `r1`–`r3`.  Returns the value the kernel placed in
/// `r0` after dispatch.
///
/// On non-ARM hosts the macro is a no-op that returns `0`, which lets
/// the examples compile (but not run) for `cargo check` / `cargo clippy`.
///
/// # Examples
///
/// ```ignore
/// let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
/// ```
#[macro_export]
macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: The inline `svc #0` instruction triggers the SVCall
        // exception whose handler (`SVC_HANDLER`) inspects and validates
        // the arguments.  The register constraints match the kernel's
        // documented SVC ABI (id in r0, args in r1-r3, r12 clobbered).
        unsafe {
            core::arch::asm!(
                "svc #0",
                inout("r0") $id => r,
                in("r1") $a,
                in("r2") $b,
                in("r3") $c,
                out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        {
            let _ = ($id, $a, $b, $c);
            r = 0;
        }
        r
    }};
}

/// Read the value the kernel placed in `r0` before entering this partition.
///
/// Partition entry arguments (port IDs, resource handles, etc.) are packed
/// into a single `u32` and passed via `r0` at partition entry.  This macro
/// extracts that value so partition code can unpack it.
///
/// On non-ARM hosts the macro returns `0`.
#[macro_export]
macro_rules! unpack_r0 {
    () => {{
        let p: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: At partition entry the kernel has placed the
        // initialisation argument in r0.  Reading the register here,
        // before any other code has clobbered it, is the defined ABI.
        unsafe {
            core::arch::asm!("", out("r0") p)
        }
        #[cfg(not(target_arch = "arm"))]
        {
            p = 0;
        }
        p
    }};
}

/// Complete kernel runtime setup macro.
///
/// Generates kernel storage, accessor functions, and PendSV handler in a
/// single invocation. This macro wraps [`define_unified_kernel!`] to provide
/// a complete, foolproof runtime setup.
///
/// # Syntax
///
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
/// ```
///
/// # Generated Items
///
/// | Item | Description |
/// |------|-------------|
/// | `KERNEL` static | Mutex-protected kernel storage |
/// | `store_kernel()` | Stores kernel instance and installs SVC hook |
/// | `with_kernel()` | Immutable access within critical section |
/// | `with_kernel_mut()` | Mutable access within critical section |
/// | `get_current_partition()` | C ABI accessor for PendSV |
/// | `get_next_partition()` | C ABI accessor for PendSV |
/// | `get_partition_sp()` | C ABI accessor for PendSV |
/// | `set_partition_sp()` | C ABI accessor for PendSV |
/// | `set_current_partition()` | C ABI accessor for PendSV |
///
/// # Example
///
/// ```ignore
/// use kernel::config::KernelConfig;
/// use kernel::svc::Kernel;
///
/// struct MyConfig;
/// impl KernelConfig for MyConfig {
///     // ... configuration constants ...
/// }
///
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
///
/// fn main() {
///     let k = Kernel::<MyConfig>::new_empty();
///     store_kernel(k);
/// }
/// ```
#[macro_export]
macro_rules! define_kernel_runtime {
    ($name:ident : Kernel<$Config:ty>) => {
        $crate::define_unified_kernel!($name : Kernel<$Config>);
    };
}

#[cfg(test)]
mod tests {
    // On non-ARM test hosts both macros must compile and return 0.

    #[test]
    fn svc_returns_zero_on_host() {
        let r = svc!(1u32, 2u32, 3u32, 4u32);
        assert_eq!(r, 0);
    }

    #[test]
    fn unpack_r0_returns_zero_on_host() {
        let p = unpack_r0!();
        assert_eq!(p, 0);
    }

    // Tests for define_kernel_runtime! macro require ARM target since the
    // macro generates code using cortex_m::interrupt::free and other ARM
    // intrinsics. The QEMU integration tests verify the actual functionality.
    // See svc.rs::unified_kernel_macro_tests for the gating pattern.
}
