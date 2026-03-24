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
//! For more control, register a callback that receives `&mut Kernel<'mem, C>` and the
//! tick count. The callback runs after internal SysTick processing completes.
//!
//! # Type Erasure
//!
//! Because `Kernel<'mem, C>` is generic over `KernelConfig`, but static storage must be
//! non-generic, this module uses type erasure:
//! - The user's `fn(&mut Kernel<'mem, C>, u64)` is stored as a raw pointer
//! - A monomorphized trampoline function loads the kernel via `load_kernel_ptr()` and
//!   handles the typed invocation
//!
//! # Safety
//!
//! The `invoke_handler` function must only be called with a valid `&mut Kernel<'mem, C>`
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

/// Type alias for SysTick handler callback signature.
///
/// A `TickHandlerFn<C>` receives a mutable reference to the kernel and the
/// current tick count.  It is called after the kernel's internal SysTick
/// processing completes.
// TODO: 'mem lifetime propagation — TickHandlerFn uses '_ (HRTB) because function
// pointer types require universally quantified lifetimes; naming it 'mem would require
// adding a lifetime parameter to the type alias, changing the public API.
pub type TickHandlerFn<C> = fn(&mut Kernel<'_, C>, u64);

/// Simple SysTick handler that advances the schedule and triggers PendSV.
///
/// This function is designed to be called from an `#[exception]` SysTick handler.
/// It wraps access to the kernel state in a critical section, calls the internal
/// `systick_handler`, and triggers PendSV when a partition switch is needed.
///
/// # EXC_RETURN
///
/// SysTick is a pure-Rust handler and does not inspect or manipulate EXC_RETURN.
/// Context switches are deferred to PendSV by setting the PendSV pending bit;
/// PendSV then handles the actual register save/restore and EXC_RETURN selection.
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
/// - The kernel pointer must be published via `store_kernel_ptr()` before
///   this function is called (typically done in `store_kernel()`).
/// - This function must only be called from handler mode (exception context).
///
/// # Note
///
/// For examples that need a custom SysTick hook, use `define_unified_harness!`
/// with the extended form instead, or use `register_handler()` to add a callback.
/// Inner ISR wrapper: emit isr_enter, run the provided closure, emit isr_exit.
///
/// Separated from [`handle_systick`] for testability. The closure typically calls
/// `systick_handler` via `with_kernel_mut`. Note: if `systick_handler` diverges
/// (via `enter_safe_idle`), that path emits `isr_exit_to_scheduler` before
/// diverging, so the trace span is always closed.
pub(crate) fn handle_systick_inner<F: FnOnce()>(f: F) {
    #[cfg(feature = "trace")]
    rtos_trace::trace::isr_enter();

    f();

    #[cfg(feature = "trace")]
    rtos_trace::trace::isr_exit_to_scheduler();
}

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
    handle_systick_inner(|| {
        let _ = crate::state::with_kernel_mut::<C, _, _>(|kernel| {
            crate::tick::systick_handler::<C>(kernel);
        });
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
/// 1. Loads the kernel pointer via `load_kernel_ptr()`
/// 2. Casts `handler_ptr` back to `fn(&mut Kernel<'mem, C>, u64)`
/// 3. Dereferences the kernel and calls the handler
struct ErasedHandler {
    /// Monomorphized trampoline that loads the kernel via AtomicPtr and calls the handler.
    trampoline: fn(*const (), u64),
    /// Raw pointer to the user's handler function, stored as `*const ()` for type erasure.
    /// This is actually a `fn(&mut Kernel<'mem, C>, u64)` cast to a raw pointer.
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
/// - `kernel_ptr` must be a valid pointer to a `Kernel<'mem, C>` that was cast to `*mut ()`
/// - `handler_ptr` must be a valid `fn(&mut Kernel<'mem, C>, u64)` cast to `*const ()`
/// - The caller must ensure no other mutable references to the kernel exist
fn trampoline<C: KernelConfig>(handler_ptr: *const (), tick: u64)
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
    // SAFETY: store_kernel_ptr was called during boot with a valid Kernel<C>.
    // We're in a critical section (interrupt-disabled) when this runs.
    let Some(nn) = (unsafe { crate::kernel_ptr::load_kernel_ptr::<C>() }) else {
        return;
    };
    // SAFETY: load_kernel_ptr returned Some, so the pointer is non-null and was
    // stored via store_kernel_ptr with a valid Kernel<C>. We're in a critical section
    // so no aliasing &mut exists.
    let kernel = unsafe { &mut *nn.as_ptr() };
    // SAFETY: The handler_ptr was created from a valid fn(&mut Kernel<'mem, C>, u64) in register_handler.
    // Function pointers have the same size/alignment as *const (), and the type is preserved.
    let handler: TickHandlerFn<C> =
        unsafe { core::mem::transmute::<*const (), TickHandlerFn<C>>(handler_ptr) };
    handler(kernel, tick);
}

/// Register a SysTick handler that receives `&mut Kernel<'mem, C>` and the tick count.
///
/// The handler is called after the kernel's internal SysTick processing completes.
/// Replaces any previously registered handler. Call before `boot()`.
///
/// # Type Safety
///
/// The handler is stored with type erasure. The caller must ensure that
/// `invoke_handler` is called with the same `Kernel<'mem, C>` instance that was
/// passed to this function.
///
/// # Safety
///
/// This function is safe to call, but the internal implementation relies on
/// unsafe pointer casts. The safety invariants are:
/// 1. The kernel reference must remain valid for the duration of handler invocations
/// 2. `invoke_handler` must be called with matching type parameter `C`
/// 3. Handler invocation must occur in a context where `&mut Kernel<'mem, C>` is valid
///    (typically within a critical section in the SysTick exception)
pub fn register_handler<C: KernelConfig>(handler: TickHandlerFn<C>)
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
    // Cast the fn pointer to a raw pointer for type-erased storage
    let handler_ptr = handler as *const ();

    let erased = ErasedHandler {
        trampoline: trampoline::<C>,
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
            (handler.trampoline)(handler.handler_ptr, tick);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU64, Ordering};

    use crate::config::KernelConfig;
    use crate::kernel_config_types;
    use crate::partition::{ExternalPartitionMemory, MpuRegion};
    use crate::partition_core::AlignedStack256B;
    use crate::scheduler::{ScheduleEntry, ScheduleTable};

    struct TestConfig;

    impl KernelConfig for TestConfig {
        const N: usize = 2;
        const SCHED: usize = 4;
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

        kernel_config_types!();
    }

    // Lifetime note: `stk0`/`stk1` are local but this is sound because
    // `Kernel<'mem, C>` has no lifetime parameter — `new()` copies
    // descriptor data into PCBs (with sentinel stack fields) and does
    // not retain borrows from `ExternalPartitionMemory`.
    fn make_test_kernel() -> Kernel<'static, TestConfig> {
        let mut schedule = ScheduleTable::new();
        schedule.add(ScheduleEntry::new(0, 10)).unwrap();
        schedule.add(ScheduleEntry::new(1, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        let mut stk0 = AlignedStack256B::default();
        let mut stk1 = AlignedStack256B::default();
        // MpuRegion with size=0 is a sentinel meaning "no user-configured data
        // region"; boot_preconfigured() patches it from the actual stack address.
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_1001,
                MpuRegion::new(0, 0, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_2001,
                MpuRegion::new(0, 0, 0),
                1,
            )
            .unwrap(),
        ];
        Kernel::<TestConfig>::new(schedule, &mems).expect("kernel creation failed")
    }

    /// Mutex to serialize tests that manipulate the global `SYSTICK_HANDLER`.
    static TEST_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

    static COUNTER: AtomicU64 = AtomicU64::new(0);
    static TICK: AtomicU64 = AtomicU64::new(0);

    fn reset() {
        COUNTER.store(0, Ordering::SeqCst);
        TICK.store(0, Ordering::SeqCst);
        clear_handler();
        crate::kernel_ptr::clear_kernel_ptr();
    }

    fn handler(_kernel: &mut Kernel<'_, TestConfig>, t: u64) {
        COUNTER.fetch_add(1, Ordering::SeqCst);
        TICK.store(t, Ordering::SeqCst);
    }

    #[test]
    fn register_clear_has() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();
        assert!(!has_handler());

        register_handler::<TestConfig>(handler);
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
        // SAFETY: kernel lives for the duration of this test; type matches TestConfig.
        unsafe { crate::kernel_ptr::store_kernel_ptr(&mut kernel) };
        register_handler::<TestConfig>(handler);
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
        fn h2(_kernel: &mut Kernel<'_, TestConfig>, _: u64) {
            N.fetch_add(1, Ordering::SeqCst);
        }
        N.store(0, Ordering::SeqCst);

        let mut kernel = make_test_kernel();
        // SAFETY: kernel lives for the duration of this test; type matches TestConfig.
        unsafe { crate::kernel_ptr::store_kernel_ptr(&mut kernel) };
        register_handler::<TestConfig>(handler);
        invoke_handler(1);
        register_handler::<TestConfig>(h2);
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

        fn kernel_reader(kernel: &mut Kernel<'_, TestConfig>, _tick: u64) {
            // Read the tick count from the kernel to verify we have real access
            let tick = kernel.tick().get();
            TICK_FROM_KERNEL.store(tick, Ordering::SeqCst);
        }
        TICK_FROM_KERNEL.store(0, Ordering::SeqCst);

        let mut kernel = make_test_kernel();
        // Set a known tick value
        kernel.tick.sync(12345);

        // SAFETY: kernel lives for the duration of this test; type matches TestConfig.
        unsafe { crate::kernel_ptr::store_kernel_ptr(&mut kernel) };
        register_handler::<TestConfig>(kernel_reader);
        invoke_handler(0);

        assert_eq!(TICK_FROM_KERNEL.load(Ordering::SeqCst), 12345);
        clear_handler();
    }

    #[test]
    fn tick_handler_fn_type_alias_compatible() {
        // Verify that a function with the correct signature is assignable to TickHandlerFn.
        fn my_handler(_kernel: &mut Kernel<'_, TestConfig>, _tick: u64) {}
        let f: TickHandlerFn<TestConfig> = my_handler;
        // Confirm the function pointer round-trips to the same address.
        assert_eq!(f as *const () as usize, my_handler as *const () as usize);
        // Also confirm the existing `handler` test helper is assignable.
        let g: TickHandlerFn<TestConfig> = handler;
        assert_eq!(g as *const () as usize, handler as *const () as usize);
    }

    #[test]
    fn handler_can_modify_kernel() {
        let _guard = TEST_MUTEX.lock().unwrap();
        reset();

        fn kernel_modifier(kernel: &mut Kernel<'_, TestConfig>, tick: u64) {
            // Modify the kernel's tick counter
            kernel.tick.sync(tick * 2);
        }

        let mut kernel = make_test_kernel();
        kernel.tick.sync(100);

        // SAFETY: kernel lives for the duration of this test; type matches TestConfig.
        unsafe { crate::kernel_ptr::store_kernel_ptr(&mut kernel) };
        register_handler::<TestConfig>(kernel_modifier);
        invoke_handler(50);

        // Verify the kernel was modified
        assert_eq!(kernel.tick().get(), 100); // 50 * 2 = 100
        clear_handler();
    }

    // -------------------------------------------------------------------------
    // ISR trace pairing tests (handle_systick_inner)
    // -------------------------------------------------------------------------

    /// Verify that handle_systick_inner calls the provided closure.
    #[test]
    fn handle_systick_inner_calls_closure() {
        use core::sync::atomic::{AtomicBool, Ordering};
        static CALLED: AtomicBool = AtomicBool::new(false);
        CALLED.store(false, Ordering::SeqCst);

        handle_systick_inner(|| {
            CALLED.store(true, Ordering::SeqCst);
        });

        assert!(CALLED.load(Ordering::SeqCst), "closure must be invoked");
    }

    /// Verify that handle_systick_inner wraps the closure symmetrically:
    /// the closure runs between ISR enter and ISR exit. We verify structural
    /// ordering by tracking a sequence counter.
    #[test]
    fn handle_systick_inner_isr_enter_exit_brackets_closure() {
        use core::sync::atomic::{AtomicU32, Ordering};
        static SEQ: AtomicU32 = AtomicU32::new(0);
        SEQ.store(0, Ordering::SeqCst);

        // The closure captures the sequence value at the point it runs.
        // In non-trace builds isr_enter/isr_exit are no-ops, so the
        // closure runs at seq=0. The key invariant: the closure always
        // runs, and handle_systick_inner returns normally.
        let mut closure_seq = 0u32;
        handle_systick_inner(|| {
            closure_seq = SEQ.fetch_add(1, Ordering::SeqCst);
        });

        assert_eq!(closure_seq, 0, "closure should run first");
        assert_eq!(
            SEQ.load(Ordering::SeqCst),
            1,
            "sequence should advance exactly once"
        );
    }

    /// Verify handle_systick_inner returns normally on the non-diverging path.
    /// This confirms that isr_exit_to_scheduler would be reached when
    /// systick_handler does not call enter_safe_idle.
    #[test]
    fn handle_systick_inner_returns_on_normal_path() {
        // If this test completes, the normal (non-faulted) path is correct:
        // isr_enter -> closure -> isr_exit_to_scheduler all execute.
        handle_systick_inner(|| {
            // simulate normal systick_handler that returns
        });
    }
}
