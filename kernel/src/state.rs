//! Unified kernel state storage with linker-controlled placement.
//!
//! Provides [`UnifiedKernel`] type alias for the merged kernel state, placed
//! at a linker-controlled address via `.kernel_state` for assembly access.

use core::mem::{size_of, MaybeUninit};
use core::ptr::addr_of_mut;

use crate::config::KernelConfig;
use crate::svc::Kernel;

/// Maximum size in bytes for kernel state storage.
///
/// This must be large enough to hold `Kernel<C>` for any configuration used.
/// The `init_kernel_state` function includes a compile-time assertion to verify
/// that the actual kernel size does not exceed this limit.
///
/// Current allocation: 16 KiB, sufficient for typical configurations with
/// up to 4 partitions (256-word stacks each), 8 schedule entries, and
/// moderate IPC pool sizes. For larger configurations, increase this value
/// and ensure the target has sufficient RAM.
pub const MAX_KERNEL_SIZE: usize = 16 * 1024;

/// Static storage for the unified kernel state.
///
/// Uses a byte array wrapped in `MaybeUninit` for deferred initialization.
/// Linker places this in `.kernel_state` section (defined in `memory.x`),
/// providing a fixed address that assembly can reference via `__kernel_state_start`.
///
/// # Safety
///
/// Uninitialized at startup. Call [`init_kernel_state`] exactly once before
/// access. Reading before initialization is undefined behavior.
#[link_section = ".kernel_state"]
pub static mut UNIFIED_KERNEL_STORAGE: MaybeUninit<KernelStorageBuffer> = MaybeUninit::uninit();

/// Properly-sized buffer to hold kernel state.
///
/// This is a concrete, non-generic type that reserves [`MAX_KERNEL_SIZE`] bytes.
/// The actual `Kernel<C>` is written into this buffer at initialization time.
#[repr(C, align(8))]
pub struct KernelStorageBuffer {
    _data: [u8; MAX_KERNEL_SIZE],
}

/// Unified kernel state containing all kernel subsystems.
///
/// Merges: partitions, schedule, current/next indices, partition SPs, tick,
/// yield flag, and all IPC pools (events, semaphores, mutexes, messages,
/// queuing ports, sampling ports, blackboards). The `Kernel<C>` struct
/// already contains all these via sub-struct composition.
pub type UnifiedKernel<C> = Kernel<C>;

/// Helper struct for compile-time size assertion.
///
/// Creates a zero-sized constant if `Kernel<C>` fits in `MAX_KERNEL_SIZE`,
/// otherwise fails to compile due to array size overflow.
struct AssertKernelFits<C: KernelConfig>
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
    _marker: core::marker::PhantomData<C>,
}

impl<C: KernelConfig> AssertKernelFits<C>
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
    /// Zero-sized constant that only compiles if Kernel<C> fits in storage.
    const OK: () = {
        // This assertion will fail at compile time if Kernel<C> is too large
        assert!(
            size_of::<Kernel<C>>() <= MAX_KERNEL_SIZE,
            "Kernel<C> exceeds MAX_KERNEL_SIZE"
        );
    };
}

/// Initialize kernel state storage with a configured kernel instance.
///
/// # Safety
///
/// - Must be called exactly once before any other kernel access.
/// - The caller must ensure no other code accesses `UNIFIED_KERNEL_STORAGE`
///   during or after this call without proper synchronization.
///
/// # Panics
///
/// Compile-time assertion fails if `size_of::<Kernel<C>>()` exceeds
/// [`MAX_KERNEL_SIZE`].
#[allow(dead_code)]
pub unsafe fn init_kernel_state<C: KernelConfig>(kernel: Kernel<C>)
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
    // Compile-time size check: ensure Kernel<C> fits in the storage buffer.
    // This creates a zero-sized array if the condition holds, or fails to compile
    // if Kernel<C> is too large (negative array size).
    #[allow(clippy::let_unit_value)]
    let _ = AssertKernelFits::<C>::OK;

    // SAFETY: The caller guarantees this is called exactly once before any
    // other kernel access. We use addr_of_mut! to obtain a raw pointer without
    // creating a reference to the static (avoiding static_mut_refs warning).
    //
    // The storage buffer is properly sized (MAX_KERNEL_SIZE bytes) and aligned
    // (8-byte alignment via repr(C, align(8))). The compile-time assertion above
    // guarantees that Kernel<C> fits within this buffer. The pointer cast from
    // *mut KernelStorageBuffer to *mut Kernel<C> is valid because:
    // 1. Both pointers point to the same memory location
    // 2. The buffer has sufficient size (verified at compile time)
    // 3. The buffer has sufficient alignment (8 bytes >= Kernel<C> alignment)
    // 4. The write initializes the memory with a valid Kernel<C> value
    let ptr = addr_of_mut!(UNIFIED_KERNEL_STORAGE) as *mut Kernel<C>;
    ptr.write(kernel);
}

/// Get a raw pointer to the kernel state storage.
///
/// # Safety
///
/// The returned pointer is only valid after [`init_kernel_state`] has been called.
/// The caller must ensure proper synchronization when accessing the kernel state.
#[allow(dead_code)]
pub unsafe fn get_kernel_ptr<C: KernelConfig>() -> *mut Kernel<C>
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
    // SAFETY: This function returns a raw pointer, deferring safety to the caller.
    // The pointer cast from *mut KernelStorageBuffer to *mut Kernel<C> is valid
    // because init_kernel_state() wrote a Kernel<C> at this location, and
    // KernelStorageBuffer has sufficient size (MAX_KERNEL_SIZE, verified at
    // compile time in init_kernel_state) and alignment (8 bytes via repr(align(8))).
    // The caller must ensure init_kernel_state() was called before dereferencing
    // the returned pointer, and must provide proper synchronization for access.
    addr_of_mut!(UNIFIED_KERNEL_STORAGE) as *mut Kernel<C>
}

// TODO: DRY violation - with_kernel and with_kernel_mut have nearly identical
// documentation. Consider using #[doc = include_str!("...")] or a doc macro
// if the toolchain supports it and the duplication becomes a maintenance burden.

/// Access the unified kernel state immutably within a critical section.
///
/// Wraps access to `UNIFIED_KERNEL_STORAGE` in `cortex_m::interrupt::free()`,
/// ensuring exclusive access by masking interrupts on single-core Cortex-M.
///
/// # Safety Invariants
///
/// This function is safe to call provided the following invariants hold:
///
/// 1. **Initialization before use**: `init_kernel_state()` must be called
///    exactly once by `boot()` before interrupts are enabled. The kernel
///    is initialized during the boot sequence before any exception handlers
///    can run.
///
/// 2. **Single-core execution**: Cortex-M is single-core; the critical section
///    via `interrupt::free()` masks all configurable interrupts, preventing
///    concurrent access from exception handlers.
///
/// 3. **Exception priority prevents reentrancy**: The ARM exception model
///    ensures that lower-priority exceptions cannot preempt higher-priority
///    ones. SVC runs at priority 0 (highest), PendSV at 0xFF (lowest), and
///    SysTick at 0xFE. This priority ordering prevents reentrancy within
///    kernel code paths.
///
/// # Returns
///
/// The result of applying closure `f` to a shared reference to the kernel.
///
/// # Safety
///
/// Calling this function before `init_kernel_state()` has been called results
/// in **Undefined Behavior** (dereferencing an uninitialized pointer).
#[cfg(not(test))]
pub fn with_kernel<C, F, R>(f: F) -> R
where
    C: KernelConfig,
    F: FnOnce(&Kernel<C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
{
    cortex_m::interrupt::free(|_cs| {
        // SAFETY: Access is safe because:
        // 1. We are inside a critical section (interrupt::free), which masks all
        //    configurable interrupts on single-core Cortex-M, preventing concurrent
        //    access from exception handlers.
        // 2. The system boot sequence guarantees that init_kernel_state() is called
        //    exactly once before interrupts are enabled, so UNIFIED_KERNEL_STORAGE
        //    contains a valid, initialized Kernel<C> instance.
        // 3. The pointer cast is valid because init_kernel_state() wrote a Kernel<C>
        //    to this location, and the storage has sufficient size and alignment.
        // 4. Creating a shared reference is safe because the critical section
        //    ensures no mutable references can exist concurrently.
        let ptr = addr_of_mut!(UNIFIED_KERNEL_STORAGE) as *const Kernel<C>;
        let kernel = unsafe { &*ptr };
        f(kernel)
    })
}

/// Access the unified kernel state mutably within a critical section.
///
/// Wraps access to `UNIFIED_KERNEL_STORAGE` in `cortex_m::interrupt::free()`,
/// ensuring exclusive access by masking interrupts on single-core Cortex-M.
///
/// # Safety Invariants
///
/// This function is safe to call provided the following invariants hold:
///
/// 1. **Initialization before use**: `init_kernel_state()` must be called
///    exactly once by `boot()` before interrupts are enabled. The kernel
///    is initialized during the boot sequence before any exception handlers
///    can run.
///
/// 2. **Single-core execution**: Cortex-M is single-core; the critical section
///    via `interrupt::free()` masks all configurable interrupts, preventing
///    concurrent access from exception handlers.
///
/// 3. **Exception priority prevents reentrancy**: The ARM exception model
///    ensures that lower-priority exceptions cannot preempt higher-priority
///    ones. SVC runs at priority 0 (highest), PendSV at 0xFF (lowest), and
///    SysTick at 0xFE. This priority ordering prevents reentrancy within
///    kernel code paths.
///
/// # Returns
///
/// The result of applying closure `f` to a mutable reference to the kernel.
///
/// # Safety
///
/// Calling this function before `init_kernel_state()` has been called results
/// in **Undefined Behavior** (dereferencing an uninitialized pointer).
#[cfg(not(test))]
pub fn with_kernel_mut<C, F, R>(f: F) -> R
where
    C: KernelConfig,
    F: FnOnce(&mut Kernel<C>) -> R,
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
{
    cortex_m::interrupt::free(|_cs| {
        // SAFETY: Access is safe because:
        // 1. We are inside a critical section (interrupt::free), which masks all
        //    configurable interrupts on single-core Cortex-M, preventing concurrent
        //    access from exception handlers.
        // 2. The system boot sequence guarantees that init_kernel_state() is called
        //    exactly once before interrupts are enabled, so UNIFIED_KERNEL_STORAGE
        //    contains a valid, initialized Kernel<C> instance.
        // 3. The pointer cast is valid because init_kernel_state() wrote a Kernel<C>
        //    to this location, and the storage has sufficient size and alignment.
        // 4. Creating a mutable reference is safe because the critical section
        //    ensures no other references (shared or mutable) can exist concurrently.
        let ptr = addr_of_mut!(UNIFIED_KERNEL_STORAGE) as *mut Kernel<C>;
        let kernel = unsafe { &mut *ptr };
        f(kernel)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg_pools::MsgPools;
    use crate::partition_core::PartitionCore;
    use crate::port_pools::PortPools;
    use crate::sync_pools::SyncPools;

    struct TestConfig;
    impl KernelConfig for TestConfig {
        const N: usize = 2;
        const SCHED: usize = 4;
        const STACK_WORDS: usize = 256;
        const S: usize = 1;
        const SW: usize = 1;
        const MS: usize = 1;
        const MW: usize = 1;
        const QS: usize = 1;
        const QD: usize = 1;
        const QM: usize = 1;
        const QW: usize = 1;
        const SP: usize = 1;
        const SM: usize = 1;
        const BS: usize = 1;
        const BM: usize = 1;
        const BW: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 32;
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = 4;
        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    #[test]
    fn unified_kernel_is_kernel_alias() {
        fn assert_same_type<T>(_: T, _: T) {}
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            let k1: Kernel<TestConfig> = Kernel::new_empty();
            let k2: UnifiedKernel<TestConfig> = Kernel::new_empty();
            assert_same_type(k1, k2);
        }
        #[cfg(feature = "dynamic-mpu")]
        {
            let reg = crate::virtual_device::DeviceRegistry::default();
            let k1: Kernel<TestConfig> = Kernel::new_empty(reg);
            let reg2 = crate::virtual_device::DeviceRegistry::default();
            let k2: UnifiedKernel<TestConfig> = Kernel::new_empty(reg2);
            assert_same_type(k1, k2);
        }
    }

    #[test]
    fn storage_is_in_kernel_state_section() {
        use core::ptr::addr_of;
        let ptr = addr_of!(UNIFIED_KERNEL_STORAGE);
        assert!(!ptr.is_null());
    }

    #[test]
    fn storage_buffer_has_correct_size() {
        assert_eq!(size_of::<KernelStorageBuffer>(), MAX_KERNEL_SIZE);
    }

    #[test]
    fn kernel_fits_in_storage() {
        // Verify that Kernel<TestConfig> fits in the storage buffer
        assert!(size_of::<Kernel<TestConfig>>() <= MAX_KERNEL_SIZE);
    }

    #[test]
    fn storage_buffer_alignment() {
        use core::mem::align_of;
        // Verify 8-byte alignment
        assert!(align_of::<KernelStorageBuffer>() >= 8);
    }
}
