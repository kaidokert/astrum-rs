//! SysTick handler registration API for user callbacks after kernel tick processing.
//!
//! This module provides two mechanisms for handling SysTick exceptions:
//!
//! ## Simple Handler Function
//!
//! For examples that just need standard SysTick behavior without custom hooks,
//! use [`handle_systick`]:
//!
//! ```ignore
//! #[exception]
//! fn SysTick() {
//!     kernel::systick::handle_systick::<MyConfig>();
//! }
//! ```
//!
//! This wraps `with_kernel_mut()` and calls `systick_handler()` to advance the
//! schedule and trigger PendSV on partition switches.
//!
//! ## Custom Callback Registration
//!
//! For more control, register a callback that receives `&mut Kernel<C>` and the
//! tick count. The callback runs after internal SysTick processing completes.
//!
//! # Type Erasure
//!
//! Because `Kernel<C>` is generic over `KernelConfig`, but static storage must be
//! non-generic, this module uses type erasure:
//! - The user's `fn(&mut Kernel<C>, u64)` is stored as a raw pointer
//! - A raw `*mut ()` pointer to the kernel is stored alongside
//! - A monomorphized trampoline function handles the typed invocation
//!
//! # Safety
//!
//! The `invoke_handler` function must only be called with a valid `&mut Kernel<C>`
//! reference that matches the type used during `register_handler`. This is enforced
//! by requiring the same `C` type parameter at both registration and invocation.
//!
//! # Example
//!
//! ```ignore
//! fn my_hook(kernel: &mut Kernel<MyConfig>, tick: u64) {
//!     if tick % 100 == 0 {
//!         // Access kernel state here
//!     }
//! }
//!
//! // During initialization, before boot():
//! kernel::systick::register_handler::<MyConfig>(&mut kernel, my_hook);
//! ```

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use crate::config::KernelConfig;
use crate::svc::Kernel;

/// Simple SysTick handler that advances the schedule and triggers PendSV.
///
/// This function is designed to be called from an `#[exception]` SysTick handler.
/// It wraps access to the kernel state in a critical section, calls the internal
/// `systick_handler`, and triggers PendSV when a partition switch is needed.
///
/// # Usage
///
/// ```ignore
/// use cortex_m_rt::exception;
///
/// #[exception]
/// fn SysTick() {
///     kernel::systick::handle_systick::<MyConfig>();
/// }
/// ```
///
/// # Requirements
///
/// - The kernel must be initialized via `state::init_kernel_state()` before
///   this function is called (typically done in `boot()`).
/// - This function must only be called from handler mode (exception context).
///
/// # Note
///
/// For examples that need a custom SysTick hook, use `define_unified_harness!`
/// with the extended form instead, or use `register_handler()` to add a callback.
#[cfg(not(test))]
pub fn handle_systick<C: KernelConfig>()
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    crate::state::with_kernel_mut::<C, _, _>(|kernel| {
        crate::tick::systick_handler::<C>(kernel);
    });
}

#[cfg(not(test))]
fn with_cs<F, R>(f: F) -> R
where
    F: FnOnce(&cortex_m::interrupt::CriticalSection) -> R,
{
    cortex_m::interrupt::free(f)
}

#[cfg(test)]
fn with_cs<F, R>(f: F) -> R
where
    F: FnOnce(&cortex_m::interrupt::CriticalSection) -> R,
{
    // SAFETY: In test mode, we're single-threaded and don't have real interrupts.
    // Creating a fake CriticalSection is safe for testing purposes.
    f(unsafe { &cortex_m::interrupt::CriticalSection::new() })
}

/// Type-erased handler that stores all pointers needed for invocation.
///
/// The `trampoline` field is a monomorphized function that:
/// 1. Casts `kernel_ptr` back to `*mut Kernel<C>`
/// 2. Casts `handler_ptr` back to `fn(&mut Kernel<C>, u64)`
/// 3. Dereferences the kernel and calls the handler
struct ErasedHandler {
    /// Monomorphized trampoline: `fn(*mut (), *const (), u64)` that internally
    /// knows the concrete type C and performs the type-safe cast and call.
    trampoline: fn(*mut (), *const (), u64),
    /// Raw pointer to the Kernel instance, stored as `*mut ()` for type erasure.
    kernel_ptr: *mut (),
    /// Raw pointer to the user's handler function, stored as `*const ()` for type erasure.
    /// This is actually a `fn(&mut Kernel<C>, u64)` cast to a raw pointer.
    handler_ptr: *const (),
}

// SAFETY: ErasedHandler is only accessed within critical sections (interrupt-free),
// so it's safe to mark as Send + Sync. The kernel pointer remains valid for the
// lifetime of the kernel (which is 'static in embedded contexts).
unsafe impl Send for ErasedHandler {}
// SAFETY: Same justification as Send - access is serialized via critical sections.
unsafe impl Sync for ErasedHandler {}

static SYSTICK_HANDLER: Mutex<RefCell<Option<ErasedHandler>>> = Mutex::new(RefCell::new(None));

/// The trampoline function that reconstructs typed pointers and invokes the handler.
///
/// This is a generic function that gets monomorphized for each `C: KernelConfig`.
/// When stored, the specific monomorphization is captured as a plain fn pointer.
///
/// # Safety
///
/// - `kernel_ptr` must be a valid pointer to a `Kernel<C>` that was cast to `*mut ()`
/// - `handler_ptr` must be a valid `fn(&mut Kernel<C>, u64)` cast to `*const ()`
/// - The caller must ensure no other mutable references to the kernel exist
fn trampoline<C: KernelConfig>(kernel_ptr: *mut (), handler_ptr: *const (), tick: u64)
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
    // SAFETY: The kernel_ptr was created from a valid &mut Kernel<C> in register_handler.
    // The caller of invoke_handler guarantees the pointer is still valid and the type matches.
    // We're in a critical section (interrupt-disabled) when this runs.
    let kernel = unsafe { &mut *(kernel_ptr as *mut Kernel<C>) };
    // SAFETY: The handler_ptr was created from a valid fn(&mut Kernel<C>, u64) in register_handler.
    // Function pointers have the same size/alignment as *const (), and the type is preserved.
    let handler: fn(&mut Kernel<C>, u64) =
        unsafe { core::mem::transmute::<*const (), fn(&mut Kernel<C>, u64)>(handler_ptr) };
    handler(kernel, tick);
}

/// Register a SysTick handler that receives `&mut Kernel<C>` and the tick count.
///
/// The handler is called after the kernel's internal SysTick processing completes.
/// Replaces any previously registered handler. Call before `boot()`.
///
/// # Type Safety
///
/// The handler is stored with type erasure. The caller must ensure that
/// `invoke_handler` is called with the same `Kernel<C>` instance that was
/// passed to this function.
///
/// # Safety
///
/// This function is safe to call, but the internal implementation relies on
/// unsafe pointer casts. The safety invariants are:
/// 1. The kernel reference must remain valid for the duration of handler invocations
/// 2. `invoke_handler` must be called with matching type parameter `C`
/// 3. Handler invocation must occur in a context where `&mut Kernel<C>` is valid
///    (typically within a critical section in the SysTick exception)
pub fn register_handler<C: KernelConfig>(kernel: &mut Kernel<C>, handler: fn(&mut Kernel<C>, u64))
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
    let kernel_ptr = kernel as *mut Kernel<C> as *mut ();
    // Cast the fn pointer to a raw pointer for type-erased storage
    let handler_ptr = handler as *const ();

    let erased = ErasedHandler {
        trampoline: trampoline::<C>,
        kernel_ptr,
        handler_ptr,
    };

    with_cs(|cs| {
        SYSTICK_HANDLER.borrow(cs).replace(Some(erased));
    });
}

/// Clear any registered SysTick handler.
pub fn clear_handler() {
    with_cs(|cs| {
        SYSTICK_HANDLER.borrow(cs).replace(None);
    });
}

/// Check if a SysTick handler is currently registered.
pub fn has_handler() -> bool {
    with_cs(|cs| SYSTICK_HANDLER.borrow(cs).borrow().is_some())
}

/// Invoke the registered SysTick handler if any.
///
/// This is called by the SysTick exception handler after kernel tick processing.
/// The handler receives the tick count; the kernel reference was captured at
/// registration time.
///
/// # Safety
///
/// The caller must ensure that:
/// 1. This is called from a context where the kernel reference captured during
///    `register_handler` is still valid (not dropped or moved)
/// 2. No other code holds a mutable reference to the kernel simultaneously
///
/// In practice, this is called from within the SysTick exception handler where
/// the kernel is borrowed mutably for the entire exception, satisfying these
/// requirements.
pub fn invoke_handler(tick: u64) {
    with_cs(|cs| {
        if let Some(ref handler) = *SYSTICK_HANDLER.borrow(cs).borrow() {
            // SAFETY: See the safety requirements documented on this function.
            // The trampoline will cast kernel_ptr and handler_ptr back to the
            // correct types and invoke the user's handler.
            (handler.trampoline)(handler.kernel_ptr, handler.handler_ptr, tick);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU64, Ordering};

    use crate::config::KernelConfig;
    use crate::msg_pools::MsgPools;
    use crate::partition::PartitionConfig;
    use crate::partition_core::{AlignedStack1K, PartitionCore};
    use crate::port_pools::PortPools;
    use crate::scheduler::{ScheduleEntry, ScheduleTable};
    use crate::sync_pools::SyncPools;

    struct TestConfig;

    impl KernelConfig for TestConfig {
        const N: usize = 2;
        const SCHED: usize = 4;
        const STACK_WORDS: usize = 256;
        const S: usize = 2;
        const SW: usize = 2;
        const MS: usize = 2;
        const MW: usize = 2;
        const QS: usize = 2;
        const QD: usize = 4;
        const QM: usize = 16;
        const QW: usize = 2;
        const SP: usize = 2;
        const SM: usize = 16;
        const BS: usize = 2;
        const BM: usize = 16;
        const BW: usize = 2;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 64;
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = 4;

        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    fn make_test_kernel() -> Kernel<TestConfig> {
        use crate::partition::MpuRegion;
        let mut schedule = ScheduleTable::new();
        schedule.add(ScheduleEntry::new(0, 10)).unwrap();
        schedule.add(ScheduleEntry::new(1, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        let configs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 1024, 0x03),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_2000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 1024, 0x03),
                peripheral_regions: heapless::Vec::new(),
            },
        ];
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            Kernel::<TestConfig>::new(schedule, &configs).expect("kernel creation failed")
        }
        #[cfg(feature = "dynamic-mpu")]
        {
            let registry = crate::virtual_device::DeviceRegistry::new();
            Kernel::<TestConfig>::new(schedule, &configs, registry).expect("kernel creation failed")
        }
    }

    /// Mutex to serialize tests that manipulate the global `SYSTICK_HANDLER`.
    static TEST_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

    static COUNTER: AtomicU64 = AtomicU64::new(0);
    static TICK: AtomicU64 = AtomicU64::new(0);

    fn reset() {
        COUNTER.store(0, Ordering::SeqCst);
        TICK.store(0, Ordering::SeqCst);
        clear_handler();
    }

    fn handler(_kernel: &mut Kernel<TestConfig>, t: u64) {
        COUNTER.fetch_add(1, Ordering::SeqCst);
        TICK.store(t, Ordering::SeqCst);
    }

    #[test]
    fn register_clear_has() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();
        assert!(!has_handler());

        let mut kernel = make_test_kernel();
        register_handler(&mut kernel, handler);
        assert!(has_handler());
        clear_handler();
        assert!(!has_handler());
    }

    #[test]
    fn invoke_behavior() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();

        // Invoke when no handler registered - should be a no-op
        invoke_handler(99);
        assert_eq!(COUNTER.load(Ordering::SeqCst), 0);

        let mut kernel = make_test_kernel();
        register_handler(&mut kernel, handler);
        invoke_handler(42);
        assert_eq!(COUNTER.load(Ordering::SeqCst), 1);
        assert_eq!(TICK.load(Ordering::SeqCst), 42);
        clear_handler();
    }

    #[test]
    fn replaces_previous() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();

        static N: AtomicU64 = AtomicU64::new(0);
        fn h2(_kernel: &mut Kernel<TestConfig>, _: u64) {
            N.fetch_add(1, Ordering::SeqCst);
        }
        N.store(0, Ordering::SeqCst);

        let mut kernel = make_test_kernel();
        register_handler(&mut kernel, handler);
        invoke_handler(1);
        register_handler(&mut kernel, h2);
        invoke_handler(2);
        assert_eq!(COUNTER.load(Ordering::SeqCst), 1);
        assert_eq!(N.load(Ordering::SeqCst), 1);
        clear_handler();
    }

    #[test]
    fn handler_receives_kernel_reference() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();

        static TICK_FROM_KERNEL: AtomicU64 = AtomicU64::new(0);

        fn kernel_reader(kernel: &mut Kernel<TestConfig>, _tick: u64) {
            // Read the tick count from the kernel to verify we have real access
            let tick = kernel.tick().get();
            TICK_FROM_KERNEL.store(tick, Ordering::SeqCst);
        }
        TICK_FROM_KERNEL.store(0, Ordering::SeqCst);

        let mut kernel = make_test_kernel();
        // Set a known tick value
        kernel.tick.sync(12345);

        register_handler(&mut kernel, kernel_reader);
        invoke_handler(0);

        assert_eq!(TICK_FROM_KERNEL.load(Ordering::SeqCst), 12345);
        clear_handler();
    }

    #[test]
    fn handler_can_modify_kernel() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();

        fn kernel_modifier(kernel: &mut Kernel<TestConfig>, tick: u64) {
            // Modify the kernel's tick counter
            kernel.tick.sync(tick * 2);
        }

        let mut kernel = make_test_kernel();
        kernel.tick.sync(100);

        register_handler(&mut kernel, kernel_modifier);
        invoke_handler(50);

        // Verify the kernel was modified
        assert_eq!(kernel.tick().get(), 100); // 50 * 2 = 100
        clear_handler();
    }
}
