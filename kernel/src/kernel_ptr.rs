//! AtomicPtr-based ISR-safe kernel access for Pattern A (main-local kernel).
//!
//! Provides a global [`AtomicPtr`] that ISR handlers can use to access a
//! [`Kernel`] instance owned by `main()`. Call [`store_kernel_ptr`] before
//! enabling interrupts, then ISRs call [`load_kernel_ptr`] to obtain access.
//!
//! # Pattern A usage (main-local kernel)
//!
//! ```rust,ignore
//! fn main() -> ! {
//!     let mut kernel = Kernel::new(/* ... */);
//!     // SAFETY: kernel lives for the remainder of main() (forever in embedded).
//!     unsafe { store_kernel_ptr(&mut kernel) };
//!     // Enable interrupts — ISRs can now access the kernel.
//!     // ...
//!     loop { /* idle */ }
//! }
//!
//! #[interrupt]
//! fn UART0() {
//!     // SAFETY: C matches the KernelConfig used in main(); kernel is alive.
//!     // Caller must ensure no higher-priority ISR creates an aliasing &mut
//!     // (e.g., by masking interrupts while the reference is held).
//!     let p = unsafe { load_kernel_ptr::<MyConfig>() };
//!     if !p.is_null() { let k = unsafe { &mut *p }; k.handle_uart_irq(); }
//! }
//! ```

use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};

use crate::config::KernelConfig;
use crate::svc::Kernel;

/// Type-erased global kernel pointer for ISR access.
static KERNEL_PTR: AtomicPtr<()> = AtomicPtr::new(ptr::null_mut());

/// Store a kernel pointer for ISR access (Release ordering).
///
/// # Safety
///
/// The caller must ensure that the referenced `Kernel` outlives all
/// subsequent calls to [`load_kernel_ptr`]. Because the pointer escapes
/// to a global static, a non-`'static` reference can dangle if the
/// kernel is dropped or moved before [`clear_kernel_ptr`] is called.
pub unsafe fn store_kernel_ptr<C: KernelConfig>(k: &mut Kernel<'_, C>)
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
{
    KERNEL_PTR.store(k as *mut Kernel<'_, C> as *mut (), Ordering::Release);
}

/// Load the kernel pointer (Acquire ordering). Returns null if no
/// kernel has been stored (or after [`clear_kernel_ptr`]).
///
/// Returns a raw pointer rather than a reference because the true lifetime
/// is *not* `'static` (the kernel may live on the stack of `main()`), and
/// on Cortex-M with priority-based nesting, multiple ISRs could call this
/// concurrently — returning `&mut` would instantly create aliasing UB.
/// The caller must convert to a reference only when they can guarantee
/// exclusive access for the duration of that borrow.
///
/// # Safety
///
/// - `C` must match the concrete `KernelConfig` used in the preceding
///   [`store_kernel_ptr`] call.
/// - The kernel must still be live (not dropped or moved).
/// - The caller must ensure no other `&mut` reference to the kernel
///   exists at the same time (e.g., between `main` and an ISR, or in
///   nested ISRs). Dereferencing the returned pointer while another
///   mutable reference is live is undefined behaviour.
pub unsafe fn load_kernel_ptr<C: KernelConfig>() -> *mut Kernel<'static, C>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
{
    let ptr = KERNEL_PTR.load(Ordering::Acquire);
    ptr as *mut Kernel<'static, C>
}

/// Clear the stored kernel pointer (Release ordering).
pub fn clear_kernel_ptr() {
    KERNEL_PTR.store(ptr::null_mut(), Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        compose_kernel_config, DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
    };

    compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

    /// Mutex to serialize tests that share the global `KERNEL_PTR`.
    static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn create_test_kernel() -> Kernel<'static, TestConfig> {
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            #[allow(deprecated)]
            Kernel::new_empty()
        }
        #[cfg(feature = "dynamic-mpu")]
        {
            let reg = crate::virtual_device::DeviceRegistry::default();
            #[allow(deprecated)]
            Kernel::new_empty(reg)
        }
    }

    #[test]
    fn load_returns_null_before_store() {
        let _guard = TEST_LOCK.lock().unwrap();
        clear_kernel_ptr();
        // SAFETY: No pointer stored; returns null without dereferencing.
        let result = unsafe { load_kernel_ptr::<TestConfig>() };
        assert!(result.is_null(), "expected null before any store");
    }

    #[test]
    fn store_load_round_trip() {
        let _guard = TEST_LOCK.lock().unwrap();
        let mut kernel = create_test_kernel();
        // SAFETY: kernel lives for the duration of this test and we
        // clear the pointer before it is dropped.
        unsafe { store_kernel_ptr(&mut kernel) };

        // SAFETY: Pointer just stored, kernel alive, no aliasing &mut.
        let loaded = unsafe { load_kernel_ptr::<TestConfig>() };
        assert!(
            !loaded.is_null(),
            "load_kernel_ptr must return non-null after store"
        );
        // SAFETY: loaded pointer is valid and we hold no other &mut.
        let k = unsafe { &*loaded };
        assert_eq!(k.current_partition, 255, "sentinel must survive round-trip");
        assert!(
            k.active_partition.is_none(),
            "active_partition must be None"
        );
        assert_eq!(k.ticks_dropped, 0, "ticks_dropped must be zero after init");

        clear_kernel_ptr();
    }

    #[test]
    fn clear_resets_to_null() {
        let _guard = TEST_LOCK.lock().unwrap();
        let mut kernel = create_test_kernel();
        // SAFETY: kernel lives for the duration of this test.
        unsafe { store_kernel_ptr(&mut kernel) };
        clear_kernel_ptr();

        // SAFETY: Pointer cleared; returns null.
        let result = unsafe { load_kernel_ptr::<TestConfig>() };
        assert!(result.is_null(), "expected null after clear");
    }
}
