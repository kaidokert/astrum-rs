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
        {
            // The `compiler_fence(Acquire)` after the read prevents the
            // compiler from reordering subsequent memory accesses before
            // the register capture.  Without it, the compiler is free to
            // move later loads/stores above the asm block, potentially
            // reading stale or uninitialised data.
            //
            // SAFETY: At partition entry the kernel has placed the
            // initialisation argument in r0.  We use an explicit
            // `mov {0}, r0` to capture it into a general-purpose register
            // because `out("r0")` with an empty asm template lets the
            // compiler assume r0 is already in the output, which can be
            // optimised away.  Reading r0 here, before any other code has
            // clobbered it, is the defined ABI.
            // `options(nomem, nostack, preserves_flags)` tells the compiler
            // this asm block does not touch memory, the stack, or flags.
            unsafe {
                core::arch::asm!(
                    "mov {0}, r0",
                    out(reg) p,
                    options(nomem, nostack, preserves_flags),
                );
            }
            core::sync::atomic::compiler_fence(
                core::sync::atomic::Ordering::Acquire,
            );
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
/// single invocation. This macro wraps [`define_unified_kernel!`] and
/// [`define_pendsv!`] to provide a complete, foolproof runtime setup.
///
/// # Syntax
///
/// Static mode (standard context switch):
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
/// ```
///
/// Static mode with SysTick handler:
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, systick);
/// ```
///
/// Dynamic mode (with MPU region programming):
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY);
/// ```
///
/// Dynamic mode with SysTick handler:
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY, systick);
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
/// | `PendSV` | Context-switch exception handler |
/// | `SysTick` | (with systick) Exception handler for scheduler tick |
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
/// // Static mode
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
///
/// // Static mode with SysTick handler
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, systick);
///
/// // Or dynamic mode with MPU strategy
/// // kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY);
///
/// // Dynamic mode with SysTick handler
/// // kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY, systick);
///
/// fn main() {
///     let k = Kernel::<MyConfig>::new_empty();
///     store_kernel(k);
/// }
/// ```
#[macro_export]
macro_rules! define_kernel_runtime {
    // Unified arm: handles static/dynamic mode and optional systick
    ($name:ident : Kernel<$Config:ty> $(, dynamic: $strategy:ident)? $(, systick)?) => {
        $crate::define_unified_kernel!($name : Kernel<$Config>);
        $crate::define_kernel_runtime!(@pendsv_impl $(dynamic: $strategy)?);
        $crate::define_kernel_runtime!(@systick_impl $Config $(, systick)?);
    };

    // Internal: emit PendSV for static mode
    (@pendsv_impl) => {
        $crate::define_pendsv!();
    };
    // Internal: emit PendSV for dynamic mode
    (@pendsv_impl dynamic: $strategy:ident) => {
        $crate::define_pendsv!(dynamic: $strategy);
    };
    // Internal: no SysTick handler
    (@systick_impl $Config:ty) => {};
    // Internal: emit SysTick handler
    (@systick_impl $Config:ty, systick) => {
        $crate::define_systick!($Config);
    };
}

/// Generate a `#[naked] extern "C" fn() -> !` trampoline that tail-calls a
/// partition body function of type `extern "C" fn(u32) -> !`.
///
/// On ARM targets the trampoline is a single `b` (branch) instruction.
/// Because the function is `#[naked]`, the compiler emits no prologue and
/// `r0` is guaranteed to still hold the value the kernel placed there at
/// partition entry.  This eliminates the compiler-reordering hazard that
/// would exist if `unpack_r0!` were used in a non-naked function.
///
/// On non-ARM test hosts the trampoline calls `$body(0)`, matching the
/// existing `unpack_r0!` host behaviour (always returns 0).
///
/// # Usage
///
/// ```ignore
/// extern "C" fn my_body(r0: u32) -> ! { loop {} }
/// kernel::partition_trampoline!(my_trampoline => my_body);
/// ```
#[macro_export]
macro_rules! partition_trampoline {
    ($name:ident => $body:path) => {
        #[cfg(target_arch = "arm")]
        #[no_mangle]
        #[naked]
        pub extern "C" fn $name() -> ! {
            // SAFETY: naked function — no prologue, so r0 is untouched.
            // The `b` instruction tail-calls $body which expects r0 as its
            // first argument per AAPCS.
            unsafe { core::arch::naked_asm!("b {0}", sym $body) }
        }

        #[cfg(not(target_arch = "arm"))]
        #[no_mangle]
        pub extern "C" fn $name() -> ! {
            // On host targets, call the body with 0 (matching unpack_r0!
            // host behaviour).
            let body: extern "C" fn(u32) -> ! = $body;
            body(0)
        }
    };
}

/// Generate a SysTick exception handler that advances the scheduler.
///
/// This macro emits a `SysTick` exception handler that calls
/// [`tick::systick_handler`](crate::tick::systick_handler) within a critical
/// section to advance the scheduler and handle partition switches.
///
/// # Usage
///
/// Typically invoked automatically by [`define_kernel_runtime!`] with the
/// `systick` option. Can also be used standalone:
///
/// ```ignore
/// kernel::define_systick!(MyConfig);
/// ```
///
/// # Generated Items
///
/// | Item | Description |
/// |------|-------------|
/// | `SysTick` | Exception handler that calls `systick_handler` |
#[macro_export]
macro_rules! define_systick {
    ($Config:ty) => {
        #[cfg(target_arch = "arm")]
        #[cortex_m_rt::exception]
        fn SysTick() {
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                $crate::tick::systick_handler::<$Config>(k);
            });
        }
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

    #[allow(clippy::empty_loop)]
    extern "C" fn test_body(_r0: u32) -> ! {
        loop {}
    }

    partition_trampoline!(test_trampoline => test_body);

    #[test]
    fn partition_trampoline_has_correct_fn_type() {
        // The trampoline must be `extern "C" fn() -> !`, compatible with
        // boot() which expects that signature for partition entry points.
        let _: extern "C" fn() -> ! = test_trampoline;
    }

    #[test]
    fn partition_trampoline_is_not_null() {
        let ptr = test_trampoline as *const ();
        assert!(!ptr.is_null());
    }

    // Tests for define_kernel_runtime! macro require ARM target since the
    // macro generates code using cortex_m::interrupt::free and other ARM
    // intrinsics. The QEMU integration tests verify the actual functionality.
    // See svc.rs::unified_kernel_macro_tests for the gating pattern.
}
