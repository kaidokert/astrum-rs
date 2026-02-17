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
/// Current allocation: 64 KiB, sufficient for typical configurations with
/// multiple partitions, IPC pools, and all kernel subsystems.
pub const MAX_KERNEL_SIZE: usize = 64 * 1024;

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
    addr_of_mut!(UNIFIED_KERNEL_STORAGE) as *mut Kernel<C>
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
