//! Kernel state access via [`with_kernel`] / [`with_kernel_mut`].
//!
//! The kernel is constructed on the caller's stack (in a `-> !` function
//! that never returns) and its address is published via
//! [`store_kernel_ptr`](crate::kernel_ptr::store_kernel_ptr).
//! All runtime access goes through [`with_kernel`] / [`with_kernel_mut`],
//! which wrap the closure in a critical section.

// ---- Kernel-size feature selection: mutual exclusivity guards ----
//
// Pairwise checks ensure that any combination of two or three kernel-size
// features is a compile error. Users must select exactly one.
// The `_kernel-size-any` escape hatch suppresses these guards so that
// `--all-features` can compile (kernel-16k wins in that case).

#[cfg(all(
    feature = "kernel-4k",
    feature = "kernel-8k",
    not(feature = "_kernel-size-any")
))]
compile_error!(
    "Features `kernel-4k` and `kernel-8k` are mutually exclusive. Select only one kernel size."
);

#[cfg(all(
    feature = "kernel-4k",
    feature = "kernel-16k",
    not(feature = "_kernel-size-any")
))]
compile_error!(
    "Features `kernel-4k` and `kernel-16k` are mutually exclusive. Select only one kernel size."
);

#[cfg(all(
    feature = "kernel-8k",
    feature = "kernel-16k",
    not(feature = "_kernel-size-any")
))]
compile_error!(
    "Features `kernel-8k` and `kernel-16k` are mutually exclusive. Select only one kernel size."
);

use crate::config::KernelConfig;
use crate::svc::Kernel;

/// Unified kernel state containing all kernel subsystems.
///
/// Merges: partitions, schedule, current/next indices, partition SPs, tick,
/// yield flag, and all IPC pools (events, semaphores, mutexes, messages,
/// queuing ports, sampling ports, blackboards). The `Kernel<'mem, C>` struct
/// already contains all these via sub-struct composition.
pub type UnifiedKernel<'mem, C> = Kernel<'mem, C>;

/// Access the unified kernel state immutably within a critical section.
///
/// Obtains the kernel pointer via [`load_kernel_ptr`](crate::kernel_ptr::load_kernel_ptr)
/// (AtomicPtr with Acquire ordering), then wraps the closure call in
/// `cortex_m::interrupt::free()` for exclusive access on single-core Cortex-M.
///
/// # Safety Invariants
///
/// This function is safe to call provided the following invariants hold:
///
/// 1. **Initialization before use**: `store_kernel_ptr()` must be called
///    during the boot sequence before interrupts are enabled. This is done
///    by `store_kernel()` before `boot_preconfigured()` is called.
///
/// 2. **Single-core execution**: Cortex-M is single-core; the critical section
///    via `interrupt::free()` masks all configurable interrupts, preventing
///    concurrent access from exception handlers.
///
/// 3. **Exception priority prevents reentrancy**: The three-tier priority
///    model ensures that lower-priority exceptions cannot preempt higher-priority
///    ones. SVCall runs at priority 0x00 (highest), SysTick at 0x10,
///    app IRQs at >= 0x20, and PendSV at 0xFF (lowest). This priority
///    ordering prevents reentrancy within kernel code paths.
///
/// # Returns
///
/// The result of applying closure `f` to a shared reference to the kernel.
///
/// # Safety
///
/// Calling this function before `store_kernel_ptr()` has been called results
/// in an `Err` return (not UB), because `load_kernel_ptr` returns `None`.
#[cfg(not(test))]
pub fn with_kernel<C, F, R>(f: F) -> Result<R, &'static str>
where
    C: KernelConfig,
    F: FnOnce(&Kernel<'static, C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
{
    // SAFETY: We only cast the NonNull to a shared reference inside a
    // critical section, ensuring no aliasing &mut exists. The Acquire
    // ordering in load_kernel_ptr guarantees visibility of the kernel
    // initialization performed before the matching Release store.
    let nn = unsafe { crate::kernel_ptr::load_kernel_ptr::<C>() };
    let ptr = match nn {
        Some(p) => p.as_ptr(),
        None => return Err("kernel not initialized"),
    };
    Ok(cortex_m::interrupt::free(|_cs| {
        // SAFETY: load_kernel_ptr returned Some, so the pointer is non-null
        // and was stored via store_kernel_ptr after a valid Kernel was
        // initialized. The critical section prevents concurrent access.
        let kernel = unsafe { &*ptr };
        f(kernel)
    }))
}

/// Access the unified kernel state mutably within a critical section.
///
/// Obtains the kernel pointer via [`load_kernel_ptr`](crate::kernel_ptr::load_kernel_ptr)
/// (AtomicPtr with Acquire ordering), then wraps the closure call in
/// `cortex_m::interrupt::free()` for exclusive access on single-core Cortex-M.
///
/// # Safety Invariants
///
/// This function is safe to call provided the following invariants hold:
///
/// 1. **Initialization before use**: `store_kernel_ptr()` must be called
///    during the boot sequence before interrupts are enabled. This is done
///    by `store_kernel()` before `boot_preconfigured()` is called.
///
/// 2. **Single-core execution**: Cortex-M is single-core; the critical section
///    via `interrupt::free()` masks all configurable interrupts, preventing
///    concurrent access from exception handlers.
///
/// 3. **Exception priority prevents reentrancy**: The three-tier priority
///    model ensures that lower-priority exceptions cannot preempt higher-priority
///    ones. SVCall runs at priority 0x00 (highest), SysTick at 0x10,
///    app IRQs at >= 0x20, and PendSV at 0xFF (lowest). This priority
///    ordering prevents reentrancy within kernel code paths.
///
/// # Returns
///
/// The result of applying closure `f` to a mutable reference to the kernel.
///
/// # Safety
///
/// Calling this function before `store_kernel_ptr()` has been called results
/// in an `Err` return (not UB), because `load_kernel_ptr` returns `None`.
#[cfg(not(test))]
pub fn with_kernel_mut<C, F, R>(f: F) -> Result<R, &'static str>
where
    C: KernelConfig,
    F: FnOnce(&mut Kernel<'static, C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
{
    // SAFETY: We only cast the NonNull to a mutable reference inside a
    // critical section, ensuring no other references exist. The Acquire
    // ordering in load_kernel_ptr guarantees visibility of the kernel
    // initialization performed before the matching Release store.
    let nn = unsafe { crate::kernel_ptr::load_kernel_ptr::<C>() };
    let ptr = match nn {
        Some(p) => p.as_ptr(),
        None => return Err("kernel not initialized"),
    };
    Ok(cortex_m::interrupt::free(|_cs| {
        // SAFETY: load_kernel_ptr returned Some, so the pointer is non-null
        // and was stored via store_kernel_ptr after a valid Kernel was
        // initialized. The critical section ensures exclusive access.
        let kernel = unsafe { &mut *ptr };
        f(kernel)
    }))
}

/// Test-only version of [`with_kernel`] that skips `cortex_m::interrupt::free`
/// (unavailable on host). Interrupt-based exclusion is unnecessary in host tests;
/// callers must serialize access themselves (e.g., via a `std::sync::Mutex`).
#[cfg(test)]
pub fn with_kernel<C, F, R>(f: F) -> Result<R, &'static str>
where
    C: KernelConfig,
    F: FnOnce(&Kernel<'static, C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
{
    // SAFETY: Same invariants as the production version. In tests there are
    // no interrupts, so the critical section is replaced by caller-managed
    // serialization (e.g., TEST_LOCK mutex).
    let nn = unsafe { crate::kernel_ptr::load_kernel_ptr::<C>() };
    let ptr = match nn {
        Some(p) => p.as_ptr(),
        None => return Err("kernel not initialized"),
    };
    // SAFETY: load_kernel_ptr returned Some, so the pointer is non-null and
    // was stored via store_kernel_ptr after a valid Kernel was initialized.
    // No interrupts exist in the test environment; caller serializes access.
    let kernel = unsafe { &*ptr };
    Ok(f(kernel))
}

/// Test-only version of [`with_kernel_mut`] that skips `cortex_m::interrupt::free`
/// (unavailable on host). Interrupt-based exclusion is unnecessary in host tests;
/// callers must serialize access themselves (e.g., via a `std::sync::Mutex`).
#[cfg(test)]
pub fn with_kernel_mut<C, F, R>(f: F) -> Result<R, &'static str>
where
    C: KernelConfig,
    F: FnOnce(&mut Kernel<'static, C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
{
    // SAFETY: Same invariants as the production version. In tests there are
    // no interrupts, so the critical section is replaced by caller-managed
    // serialization (e.g., TEST_LOCK mutex).
    let nn = unsafe { crate::kernel_ptr::load_kernel_ptr::<C>() };
    let ptr = match nn {
        Some(p) => p.as_ptr(),
        None => return Err("kernel not initialized"),
    };
    // SAFETY: load_kernel_ptr returned Some, so the pointer is non-null and
    // was stored via store_kernel_ptr after a valid Kernel was initialized.
    // No interrupts exist in the test environment; caller serializes access.
    let kernel = unsafe { &mut *ptr };
    Ok(f(kernel))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        compose_kernel_config, DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
    };

    compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

    /// Helper to create a test kernel instance.
    ///
    /// Abstracts cfg-gated construction logic for DRY compliance.
    fn create_test_kernel() -> Kernel<'static, TestConfig> {
        let reg = crate::virtual_device::DeviceRegistry::default();
        #[allow(deprecated)]
        Kernel::new_empty(reg)
    }

    #[test]
    fn unified_kernel_is_kernel_alias() {
        fn assert_same_type<T>(_: T, _: T) {}
        let k1: Kernel<'static, TestConfig> = create_test_kernel();
        let k2: UnifiedKernel<'static, TestConfig> = create_test_kernel();
        assert_same_type(k1, k2);
    }

    #[test]
    fn run_bottom_half_macro_sets_and_clears_flag() {
        let mut k = create_test_kernel();
        assert!(!k.in_bottom_half);
        let _bh = crate::run_bottom_half!(k, 0, &k.dynamic_strategy).unwrap();
        assert!(!k.in_bottom_half);
    }

    #[test]
    fn run_bottom_half_macro_returns_err_on_nested_call() {
        let mut k = create_test_kernel();
        k.in_bottom_half = true;
        let result = crate::run_bottom_half!(k, 0, &k.dynamic_strategy);
        assert_eq!(
            result.unwrap_err(),
            "nested bottom-half invocation detected"
        );
    }

    /// Mutex to serialize tests that share the global `KERNEL_PTR`.
    static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Verifies the full store_kernel_ptr → with_kernel_mut path.
    ///
    /// This test confirms the chicken-and-egg fix: after store_kernel_ptr populates
    /// the AtomicPtr, `with_kernel_mut` can successfully access the kernel and
    /// returns `Ok`.
    #[test]
    fn store_kernel_ptr_enables_with_kernel_mut() {
        let _guard = TEST_LOCK.lock().unwrap();

        let mut kernel = create_test_kernel();

        // Store the kernel pointer in the AtomicPtr global.
        // SAFETY: kernel lives for the duration of this test and we
        // clear the pointer before it is dropped.
        unsafe { crate::kernel_ptr::store_kernel_ptr(&mut kernel) };

        // Exercise the real with_kernel_mut API and verify it returns Ok.
        let result = with_kernel_mut::<TestConfig, _, _>(|k| {
            assert_eq!(
                k.current_partition, 255,
                "current_partition sentinel must be accessible via with_kernel_mut"
            );
            assert!(
                k.active_partition.is_none(),
                "active_partition must be None"
            );
        });
        assert!(
            result.is_ok(),
            "with_kernel_mut must return Ok after store_kernel_ptr"
        );

        // Clean up global state.
        crate::kernel_ptr::clear_kernel_ptr();

        // Verify cleanup: with_kernel_mut should now return Err.
        let after_clear = with_kernel_mut::<TestConfig, _, ()>(|_| {});
        assert!(
            after_clear.is_err(),
            "with_kernel_mut must return Err after clear_kernel_ptr"
        );
    }
}
