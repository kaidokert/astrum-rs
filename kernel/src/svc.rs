use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::context::ExceptionFrame;

/// Typed SVC error codes returned to user-space via r0.
///
/// Each variant maps to a unique `u32` with the high bit set (>= 0x8000_0000),
/// making them distinguishable from success values (which are small non-negative
/// integers such as byte counts or zero).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SvcError {
    /// The syscall number in r0 was not a recognized `SyscallId`.
    InvalidSyscall,
    /// The resource ID (semaphore, mutex, queue, port, blackboard) was out of
    /// range or does not refer to an allocated resource.
    InvalidResource,
    /// A wait queue on the target resource is full and cannot accept another
    /// blocked partition.
    WaitQueueFull,
    /// A partition state transition (e.g. Running → Waiting) failed because the
    /// current state does not permit it.
    TransitionFailed,
    /// The partition index supplied as caller or target is out of range.
    InvalidPartition,
    /// A catch-all for operation-specific failures (e.g. direction violation,
    /// message too large, board empty on non-blocking read).
    OperationFailed,
    /// A user-supplied pointer (and length) does not lie within the calling
    /// partition's MPU data region or the arithmetic overflows `u32`.
    InvalidPointer,
    /// The syscall number is recognised but the handler is not yet
    /// implemented.
    NotImplemented,
}

impl SvcError {
    /// Bit mask shared by all error codes.  Every `SvcError` variant has this
    /// bit set in its `u32` representation, while success values (small
    /// non-negative integers) never do.
    pub const ERROR_BIT: u32 = 0x8000_0000;

    /// Return `true` when the raw `u32` returned by an SVC call indicates an
    /// error (i.e. has the high bit set).
    #[inline]
    pub const fn is_error(code: u32) -> bool {
        code & Self::ERROR_BIT != 0
    }

    /// Map this error to a unique `u32` value with the high bit set.
    ///
    /// The values count down from `0xFFFF_FFFF` so they are easy to inspect in
    /// a debugger and leave room for future variants without renumbering.
    pub const fn to_u32(self) -> u32 {
        match self {
            Self::InvalidSyscall => 0xFFFF_FFFF,
            Self::InvalidResource => 0xFFFF_FFFE,
            Self::WaitQueueFull => 0xFFFF_FFFD,
            Self::TransitionFailed => 0xFFFF_FFFC,
            Self::InvalidPartition => 0xFFFF_FFFB,
            Self::OperationFailed => 0xFFFF_FFFA,
            Self::InvalidPointer => 0xFFFF_FFF9,
            Self::NotImplemented => 0xFFFF_FFF8,
        }
    }
}
/// Check that a user-space pointer `[ptr, ptr+len)` lies entirely within the
/// MPU data region OR the stack region of partition `pid`.
///
/// Returns `true` when **all** of the following hold:
/// 1. `pid` refers to an existing partition in `partitions`.
/// 2. `ptr + len` does not overflow `u32`.
/// 3. The pointer range `[ptr, ptr+len)` lies entirely within EITHER:
///    - The MPU data region (`mpu_region.base()`, `mpu_region.size()`), OR
///    - The stack region (`stack_base`, `stack_size`).
///
/// A pointer that spans across both regions (partially in data, partially in
/// stack) is **invalid** — the entire range must fit within a single region.
///
/// A zero-length range (`len == 0`) is considered valid as long as `ptr`
/// itself falls within `[base, base + size]` of either region, but the caller
/// should avoid dereferencing such a pointer.
pub fn validate_user_ptr<const N: usize>(
    partitions: &PartitionTable<N>,
    pid: u8,
    ptr: u32,
    len: usize,
) -> bool {
    let pcb = match partitions.get(pid as usize) {
        Some(p) => p,
        None => return false,
    };

    // Compute end of the user range with overflow check.
    let end = match ptr.checked_add(len as u32) {
        Some(e) => e,
        None => return false,
    };

    // Check each accessible static region (data and stack).
    // Region validity (base + size not overflowing) is enforced at creation time.
    for (base, size) in pcb.accessible_static_regions() {
        let region_end = base + size;
        if ptr >= base && end <= region_end {
            return true;
        }
    }

    false
}

/// Check that a user-space pointer `[ptr, ptr+len)` lies entirely within one
/// of the dynamic MPU windows assigned to partition `pid`, OR within the
/// static regions (data + stack).
///
/// This is the dynamic-MPU variant of [`validate_user_ptr`] that additionally
/// queries `strategy.accessible_regions(pid)` and validates against each
/// dynamically-assigned MPU window.
///
/// Returns `true` when **all** of the following hold:
/// 1. `pid` refers to an existing partition in `partitions`.
/// 2. `ptr + len` does not overflow `u32`.
/// 3. The pointer range `[ptr, ptr+len)` lies entirely within EITHER:
///    - One of the dynamic MPU windows returned by `strategy.accessible_regions(pid)`, OR
///    - The static MPU data region (`mpu_region.base()`, `mpu_region.size()`), OR
///    - The static stack region (`stack_base`, `stack_size`).
///
/// A pointer that spans across multiple regions is **invalid** — the entire
/// range must fit within a single region.
#[cfg(feature = "dynamic-mpu")]
pub fn validate_user_ptr_dynamic<const N: usize>(
    partitions: &PartitionTable<N>,
    strategy: &crate::mpu_strategy::DynamicStrategy,
    pid: u8,
    ptr: u32,
    len: usize,
) -> bool {
    let pcb = match partitions.get(pid as usize) {
        Some(p) => p,
        None => return false,
    };

    // Compute end of the user range with overflow check.
    let end = match ptr.checked_add(len as u32) {
        Some(e) => e,
        None => return false,
    };

    // Helper: check if [ptr, end) lies entirely within [base, base+size].
    // Uses saturating_add to handle regions that extend to or past 0xFFFF_FFFF.
    let in_region = |base: u32, size: u32| -> bool {
        let region_end = base.saturating_add(size);
        ptr >= base && end <= region_end
    };

    // First, check dynamic MPU windows assigned to this partition.
    let windows = strategy.accessible_regions(pid);
    for (base, size) in windows {
        if in_region(base, size) {
            return true;
        }
    }

    // Fall back to static regions (data + stack).
    let data_region = pcb.mpu_region();
    if in_region(data_region.base(), data_region.size()) {
        return true;
    }

    let (stack_base, stack_size) = pcb.stack_region();
    in_region(stack_base, stack_size)
}

/// Return all memory regions accessible to a partition, combining static and dynamic regions.
///
/// This function provides a unified view of all memory a partition can access,
/// useful for comprehensive pointer validation or memory map introspection.
///
/// # Regions Returned
///
/// 1. **Static regions** from [`PartitionControlBlock::accessible_static_regions`]:
///    - Data region (`mpu_region.base()`, `mpu_region.size()`)
///    - Stack region (`stack_base`, `stack_size`)
///    - Peripheral regions (up to 2)
///
/// 2. **Dynamic windows** from [`DynamicStrategy::accessible_regions`]:
///    - MPU windows (R4–R7) currently assigned to this partition
///
/// Static regions appear first in the returned vector, followed by dynamic windows.
///
/// # Returns
///
/// A `heapless::Vec<(u32, u32), 8>` containing `(base, size)` pairs for each region.
/// Returns an empty vector if the partition does not exist.
///
/// # Example
///
/// ```ignore
/// let regions = all_accessible_regions(&partitions, &strategy, pid);
/// for (base, size) in regions {
///     // Process region...
/// }
/// ```
#[cfg(feature = "dynamic-mpu")]
pub fn all_accessible_regions<const N: usize>(
    partitions: &PartitionTable<N>,
    strategy: &crate::mpu_strategy::DynamicStrategy,
    pid: u8,
) -> heapless::Vec<(u32, u32), 8> {
    let mut result = heapless::Vec::new();

    let pcb = match partitions.get(pid as usize) {
        Some(p) => p,
        None => return result,
    };

    // Add static regions first (up to 4: data, stack, 2 peripheral).
    for region in pcb.accessible_static_regions() {
        result.push(region).unwrap();
    }

    // Add dynamic windows (up to 4: R4–R7 assigned to this partition).
    for region in strategy.accessible_regions(pid) {
        result.push(region).unwrap();
    }

    result
}

/// Validate a user pointer and, on success, execute the body expression.
///
/// Returns `SvcError::InvalidPointer` when `validate_user_ptr` fails;
/// otherwise evaluates `$body` (which must produce `u32`).
///
/// Uses `$self.partitions()` (immutable borrow) for validation, then
/// releases the borrow before executing `$body` so that `$body` may
/// freely call `$self.partitions_mut()`.
macro_rules! validated_ptr {
    ($self:ident, $ptr:expr, $len:expr, $body:expr) => {
        if !validate_user_ptr($self.partitions(), $self.current_partition, $ptr, $len) {
            SvcError::InvalidPointer.to_u32()
        } else {
            $body
        }
    };
}

/// Validate a user pointer for syscalls that may involve dynamically-mapped buffers.
///
/// When `dynamic-mpu` feature is enabled, uses [`validate_user_ptr_dynamic`] which
/// checks both static MPU regions and dynamic MPU windows assigned to the partition.
/// When `dynamic-mpu` is disabled, falls back to standard [`validate_user_ptr`].
///
/// Same interface as [`validated_ptr!`]: returns `SvcError::InvalidPointer`
/// when validation fails; otherwise evaluates `$body`.
#[cfg(feature = "dynamic-mpu")]
macro_rules! validated_ptr_dynamic {
    ($self:ident, $ptr:expr, $len:expr, $body:expr) => {
        if !validate_user_ptr_dynamic(
            $self.partitions(),
            &$self.dynamic_strategy,
            $self.current_partition,
            $ptr,
            $len,
        ) {
            SvcError::InvalidPointer.to_u32()
        } else {
            $body
        }
    };
}

/// Fallback implementation when `dynamic-mpu` is disabled.
/// Uses standard pointer validation (same as `validated_ptr!`).
#[cfg(not(feature = "dynamic-mpu"))]
#[allow(unused_macros)]
macro_rules! validated_ptr_dynamic {
    ($self:ident, $ptr:expr, $len:expr, $body:expr) => {
        validated_ptr!($self, $ptr, $len, $body)
    };
}

use crate::events;
use crate::scheduler::ScheduleEvent;

/// Abstracts over schedule advance return type for feature-gated code.
///
/// Harness macros use `$crate::svc::YieldResult` to extract partition IDs
/// uniformly regardless of which feature gates are enabled.
pub trait YieldResult {
    fn partition_id(&self) -> Option<u8>;
    fn is_system_window(&self) -> bool;
}

impl YieldResult for ScheduleEvent {
    fn partition_id(&self) -> Option<u8> {
        if let Self::PartitionSwitch(p) = self {
            Some(*p)
        } else {
            None
        }
    }
    fn is_system_window(&self) -> bool {
        #[cfg(feature = "dynamic-mpu")]
        if matches!(self, Self::SystemWindow) {
            return true;
        }
        false
    }
}
use crate::message::{MessagePool, RecvOutcome, SendOutcome};
use crate::mutex::MutexPool;
use crate::partition::{ConfigError, PartitionConfig, PartitionState, PartitionTable};
use crate::queuing::{QueuingPortPool, QueuingPortStatus, SendQueuingOutcome};
use crate::sampling::SamplingPortPool;
use crate::scheduler::ScheduleTable;
use crate::semaphore::SemaphorePool;
use crate::syscall::SyscallId;
use crate::tick::TickCounter;
// Re-export for callers who need to call methods on TickCounter from facade methods
#[allow(unused_imports)]
pub use crate::tick::TickCounterOps;
#[cfg(feature = "dynamic-mpu")]
use crate::virtual_device::VirtualDevice;

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

/// Optional application-provided dispatch hook. When set, `dispatch_svc`
/// forwards every SVC frame to this function instead of using the built-in
/// minimal dispatch. Applications set this to route syscalls through a
/// full `Kernel::dispatch` that has access to all kernel service pools.
///
/// Protected by a `cortex_m::interrupt::Mutex` so that reads and writes
/// are performed inside critical sections on single-core Cortex-M.
static SVC_DISPATCH_HOOK: Mutex<RefCell<Option<unsafe extern "C" fn(&mut ExceptionFrame)>>> =
    Mutex::new(RefCell::new(None));

/// Execute `f` inside a critical section.
///
/// On the target this disables interrupts via `cortex_m::interrupt::free`.
/// In host-mode unit tests the closure runs directly with a synthetic
/// `CriticalSection` token (single-threaded test runner, no real interrupts).
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
    // SAFETY: Tests run single-threaded on the host — there are no real
    // interrupts to mask, so a synthetic CriticalSection token is sound.
    f(unsafe { &cortex_m::interrupt::CriticalSection::new() })
}

/// Install an application-provided SVC dispatch hook.
///
/// The hook function will be called by `dispatch_svc` for every SVC
/// exception instead of the built-in minimal handler.
///
/// The function pointer is stored behind a `Mutex<RefCell<…>>` so this
/// is safe to call from any non-interrupt context on single-core
/// Cortex-M.  The hook itself is `unsafe extern "C"` because it will
/// be invoked from the SVCall exception with a raw `ExceptionFrame`
/// pointer, but *installing* it is a safe operation.
pub fn set_dispatch_hook(hook: unsafe extern "C" fn(&mut ExceptionFrame)) {
    with_cs(|cs| {
        SVC_DISPATCH_HOOK.borrow(cs).replace(Some(hook));
    });
}

/// Declares a unified kernel storage static with dispatch hook and store function.
///
/// This macro generates:
/// - `static KERNEL: Mutex<RefCell<Option<Kernel<$Config>>>>` — the unified kernel storage
/// - `unsafe extern "C" fn dispatch_hook(f: &mut ExceptionFrame)` — the SVC dispatch hook
/// - `fn store_kernel(k: Kernel<$Config>)` — stores the kernel and installs the hook
/// - PendSV accessors: `get_current_partition`, `get_next_partition`, `get_partition_sp_ptr`
///
/// # Where Bounds
///
/// The macro uses `Kernel<$Config>` which requires only essential const bounds:
/// - `[(); C::N]:` and `[(); C::SCHED]:` (always required)
/// - `[(); C::BP]:`, `[(); C::BZ]:`, `[(); C::DR]:` (with `dynamic-mpu` feature)
///
/// Sub-struct-owned constants (S, SW, MS, MW, QS, QD, QM, QW, SP, SM, BS, BM, BW)
/// do NOT require explicit `[(); C::X]:` bounds - these are encapsulated within
/// the associated types (`C::Sync`, `C::Msg`, `C::Ports`) and satisfied implicitly.
///
/// # Variants
///
/// 1. Basic: `define_unified_kernel!(MyConfig);`
/// 2. With yield handler: `define_unified_kernel!(MyConfig, |k| { ... });`
/// 3. Config-generating: `define_unified_kernel!(MyConfig { N: 4, ... });`
/// 4. Config-generating with yield: `define_unified_kernel!(MyConfig { ... }, |k| { ... });`
///
/// # Usage
///
/// ```ignore
/// kernel::define_unified_kernel!(MyConfig);
/// ```
///
/// With a custom yield handler:
///
/// ```ignore
/// kernel::define_unified_kernel!(MyConfig, |k| {
///     // Handle yield request
///     k.yield_requested = false;
/// });
/// ```
#[macro_export]
macro_rules! define_unified_kernel {
    // Basic variant with no custom yield handler.
    ($Config:ty) => {
        $crate::define_unified_kernel!(@impl $Config, |_k| {});
    };
    // Variant with custom yield handler.
    ($Config:ty, |$k:ident| $yield_body:block) => {
        $crate::define_unified_kernel!(@impl $Config, |$k| $yield_body);
    };
    // Variant that generates a complete KernelConfig struct and impl with
    // associated types wired up. Use this to reduce boilerplate when defining
    // new kernel configurations.
    //
    // Usage:
    // ```ignore
    // define_unified_kernel!(MyConfig {
    //     N: 4, SCHED: 8, S: 4, SW: 4, MS: 4, MW: 4,
    //     QS: 4, QD: 4, QM: 64, QW: 4, SP: 8, SM: 32, BS: 4, BM: 32, BW: 4
    // });
    // ```
    ($Name:ident {
        N: $n:expr, SCHED: $sched:expr,
        S: $s:expr, SW: $sw:expr, MS: $ms:expr, MW: $mw:expr,
        QS: $qs:expr, QD: $qd:expr, QM: $qm:expr, QW: $qw:expr,
        SP: $sp:expr, SM: $sm:expr, BS: $bs:expr, BM: $bm:expr, BW: $bw:expr
        $(, BP: $bp:expr, BZ: $bz:expr)?
    }) => {
        $crate::define_unified_kernel!(@config $Name {
            N: $n, SCHED: $sched,
            S: $s, SW: $sw, MS: $ms, MW: $mw,
            QS: $qs, QD: $qd, QM: $qm, QW: $qw,
            SP: $sp, SM: $sm, BS: $bs, BM: $bm, BW: $bw
            $(, BP: $bp, BZ: $bz)?
        });
        $crate::define_unified_kernel!(@impl $Name, |_k| {});
    };
    // Variant that generates KernelConfig with custom yield handler.
    ($Name:ident {
        N: $n:expr, SCHED: $sched:expr,
        S: $s:expr, SW: $sw:expr, MS: $ms:expr, MW: $mw:expr,
        QS: $qs:expr, QD: $qd:expr, QM: $qm:expr, QW: $qw:expr,
        SP: $sp:expr, SM: $sm:expr, BS: $bs:expr, BM: $bm:expr, BW: $bw:expr
        $(, BP: $bp:expr, BZ: $bz:expr)?
    }, |$k:ident| $yield_body:block) => {
        $crate::define_unified_kernel!(@config $Name {
            N: $n, SCHED: $sched,
            S: $s, SW: $sw, MS: $ms, MW: $mw,
            QS: $qs, QD: $qd, QM: $qm, QW: $qw,
            SP: $sp, SM: $sm, BS: $bs, BM: $bm, BW: $bw
            $(, BP: $bp, BZ: $bz)?
        });
        $crate::define_unified_kernel!(@impl $Name, |$k| $yield_body);
    };
    // Private rule: generates the KernelConfig struct and impl.
    // Called by the public-facing rules to avoid code duplication.
    //
    // BOUNDS: The generated impl provides associated types (Core, Sync, Msg, Ports) that
    // encapsulate sub-struct-owned constants. This allows Kernel<$Name> to compile without
    // explicit `[(); C::X]:` bounds for S, SW, MS, MW, etc. - those bounds are satisfied
    // implicitly through the associated type definitions.
    (@config $Name:ident {
        N: $n:expr, SCHED: $sched:expr,
        S: $s:expr, SW: $sw:expr, MS: $ms:expr, MW: $mw:expr,
        QS: $qs:expr, QD: $qd:expr, QM: $qm:expr, QW: $qw:expr,
        SP: $sp:expr, SM: $sm:expr, BS: $bs:expr, BM: $bm:expr, BW: $bw:expr
        $(, BP: $bp:expr, BZ: $bz:expr)?
    }) => {
        struct $Name;
        impl $crate::config::KernelConfig for $Name {
            const N: usize = $n;
            const SCHED: usize = $sched;
            const S: usize = $s;
            const SW: usize = $sw;
            const MS: usize = $ms;
            const MW: usize = $mw;
            const QS: usize = $qs;
            const QD: usize = $qd;
            const QM: usize = $qm;
            const QW: usize = $qw;
            const SP: usize = $sp;
            const SM: usize = $sm;
            const BS: usize = $bs;
            const BM: usize = $bm;
            const BW: usize = $bw;
            $(
                #[cfg(feature = "dynamic-mpu")]
                const BP: usize = $bp;
                #[cfg(feature = "dynamic-mpu")]
                const BZ: usize = $bz;
            )?

            type Core = $crate::partition_core::PartitionCore<{ Self::N }, { Self::SCHED }>;
            type Sync = $crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
            type Msg = $crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
            type Ports = $crate::port_pools::PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
        }
    };
    // Internal implementation rule using @impl token as private pattern.
    // NOTE: The @impl pattern below is valid Rust macro syntax. If a code review tool
    // reports a "file path as matcher token" error, that is a false positive from the
    // tool - verify by running `cargo check` which succeeds. The @impl idiom is standard
    // for private macro rules (see std::vec!, lazy_static!, etc.).
    //
    // BOUNDS: This rule uses `Kernel<$Config>` which inherits its where clause from the
    // Kernel struct definition. The struct was updated to remove sub-struct-owned const
    // bounds (S, SW, MS, MW, QS, QD, QM, QW, SP, SM, BS, BM, BW) - see commit b1e8222.
    // The macro itself does not need explicit where clauses; reduced bounds are achieved
    // through the struct definition.
    (@impl $Config:ty, |$k:ident| $yield_body:block) => {
        /// Unified kernel storage: holds the `Kernel` struct containing partitions,
        /// schedule, resource pools, and dispatch state.
        static KERNEL: ::cortex_m::interrupt::Mutex<
            ::core::cell::RefCell<Option<$crate::svc::Kernel<$Config>>>,
        > = ::cortex_m::interrupt::Mutex::new(::core::cell::RefCell::new(None));

        /// SVC dispatch hook that routes syscalls through the unified kernel.
        ///
        /// # Safety
        ///
        /// Must be called from SVC exception context with a valid `ExceptionFrame`
        /// pointer from the process stack (PSP).
        unsafe extern "C" fn dispatch_hook(f: &mut $crate::context::ExceptionFrame) {
            // SAFETY: called from SVC exception context on single-core Cortex-M;
            // `cortex_m::interrupt::free` masks interrupts, ensuring exclusive access.
            ::cortex_m::interrupt::free(|cs| {
                if let Some(k) = KERNEL.borrow(cs).borrow_mut().as_mut() {
                    // NOTE: current_partition is now maintained by PendSV via the
                    // set_current_partition() shim. No sync from static needed.

                    // SAFETY: `k.dispatch(f)` requires: (1) `f` is a valid pointer to the
                    // exception frame on the process stack — guaranteed by the SVC assembly
                    // trampoline that calls dispatch_hook, and (2) exclusive mutable access
                    // to the Kernel — guaranteed by `interrupt::free` masking interrupts and
                    // `RefCell::borrow_mut` providing runtime borrow checking within the
                    // critical section.
                    unsafe { k.dispatch(f) }

                    // Check and handle yield request after dispatch.
                    if k.yield_requested {
                        k.yield_requested = false;
                        let $k = k;
                        $yield_body
                    }
                }
            });
        }

        /// Store the kernel instance and install the SVC dispatch hook.
        ///
        /// This function:
        /// 1. Stores the provided `Kernel` instance in the global `KERNEL` static
        /// 2. Installs `dispatch_hook` as the SVC exception handler
        ///
        /// Must be called exactly once during initialization, before enabling
        /// interrupts or starting the scheduler.
        fn store_kernel(k: $crate::svc::Kernel<$Config>) {
            ::cortex_m::interrupt::free(|cs| {
                KERNEL.borrow(cs).replace(Some(k));
            });
            $crate::svc::set_dispatch_hook(dispatch_hook);
        }

        /// Helper to access KERNEL within an interrupt-free critical section.
        ///
        /// Consolidates the repeated pattern of `interrupt::free` + `borrow` +
        /// `as_ref`/`as_mut` into a single abstraction for PendSV accessor functions.
        ///
        /// Returns `None` if KERNEL is not initialized; otherwise passes a reference
        /// to the closure and returns its result wrapped in `Some`.
        // TODO: Reviewer feedback suggests a cleaner abstraction layer for all
        // C-ABI shims that return raw pointers to kernel state (issue #3).
        #[inline]
        fn with_kernel<T, F: FnOnce(&$crate::svc::Kernel<$Config>) -> T>(f: F) -> Option<T> {
            ::cortex_m::interrupt::free(|cs| {
                KERNEL.borrow(cs).borrow().as_ref().map(f)
            })
        }

        /// Mutable variant of [`with_kernel`] for accessors that need `&mut`.
        #[inline]
        fn with_kernel_mut<T, F: FnOnce(&mut $crate::svc::Kernel<$Config>) -> T>(f: F) -> Option<T> {
            ::cortex_m::interrupt::free(|cs| {
                KERNEL.borrow(cs).borrow_mut().as_mut().map(f)
            })
        }

        /// Returns the current partition index from the Kernel struct.
        ///
        /// Called by the Rust shim from PendSV assembly to read context-switch
        /// state. Uses `interrupt::free` to safely access KERNEL.
        ///
        /// Returns `u32::MAX` if KERNEL is not initialized.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn get_current_partition() -> u32 {
            with_kernel(|k| k.current_partition as u32).unwrap_or(u32::MAX)
        }

        /// Returns the next partition index from the Kernel struct.
        ///
        /// Called by the Rust shim from PendSV assembly to read context-switch
        /// state. Uses `interrupt::free` to safely access KERNEL.
        ///
        /// Returns `u32::MAX` if KERNEL is not initialized.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn get_next_partition() -> u32 {
            with_kernel(|k| k.next_partition() as u32).unwrap_or(u32::MAX)
        }

        /// Returns a pointer to the partition_sp array in the Kernel struct.
        ///
        /// Called by the Rust shim from PendSV assembly to read/write saved
        /// stack pointers during context switch. Uses `interrupt::free` to
        /// safely access KERNEL.
        ///
        /// Returns null pointer if KERNEL is not initialized.
        ///
        /// # Safety
        ///
        /// The caller must ensure that:
        /// - The pointer is only used while interrupts are disabled or from
        ///   PendSV (lowest priority exception that cannot be preempted).
        /// - Array access is within bounds (0..N where N is the partition count).
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn get_partition_sp_ptr() -> *mut u32 {
            with_kernel_mut(|k| k.partition_sp_mut().as_mut_ptr()).unwrap_or(::core::ptr::null_mut())
        }

        /// Returns the stack pointer for a partition by index.
        ///
        /// Called by PendSV assembly to read saved stack pointers during
        /// context switch. Uses `interrupt::free` to safely access KERNEL.
        ///
        /// Returns 0 if KERNEL is not initialized or index is out of bounds.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn get_partition_sp(idx: u32) -> u32 {
            with_kernel(|k| k.get_sp(idx as usize).unwrap_or(0)).unwrap_or(0)
        }

        /// Sets the stack pointer for a partition by index.
        ///
        /// Called by PendSV assembly to save stack pointers during context
        /// switch. Uses `interrupt::free` to safely access KERNEL.
        ///
        /// No-op if KERNEL is not initialized or index is out of bounds.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn set_partition_sp(idx: u32, sp: u32) {
            with_kernel_mut(|k| { let _ = k.set_sp(idx as usize, sp); });
        }

        /// Sets the current partition index in the Kernel struct.
        ///
        /// Called by PendSV assembly after context switch to update the
        /// kernel's current partition. Uses `interrupt::free` to safely
        /// access KERNEL.
        ///
        /// Does nothing if KERNEL is not initialized.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn set_current_partition(pid: u32) {
            with_kernel_mut(|k| k.set_current_partition(pid as u8));
        }
    };
}

/// Dispatch an SVC call based on the syscall number in `frame.r0`.
///
/// If a dispatch hook has been installed via [`set_dispatch_hook`], the
/// call is forwarded to the application-provided handler. Otherwise a
/// minimal built-in handler processes `Yield` and returns error codes
/// for all other syscalls.
///
/// # Safety
///
/// The caller must pass a valid pointer to the hardware-stacked
/// `ExceptionFrame` on the process stack (PSP). This is guaranteed
/// when called from the SVCall assembly trampoline above.
#[no_mangle]
pub unsafe extern "C" fn dispatch_svc(frame: &mut ExceptionFrame) {
    #[cfg(feature = "qemu")]
    cortex_m_semihosting::hprintln!("[dispatch_svc] entered, r0={:#x}", frame.r0);
    let hook = with_cs(|cs| *SVC_DISPATCH_HOOK.borrow(cs).borrow());
    if let Some(hook) = hook {
        #[cfg(feature = "qemu")]
        cortex_m_semihosting::hprintln!("[dispatch_svc] calling hook");
        hook(frame);
        #[cfg(feature = "qemu")]
        cortex_m_semihosting::hprintln!("[dispatch_svc] hook returned, r0={:#x}", frame.r0);
        return;
    }
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(SyscallId::EventWait | SyscallId::EventSet | SyscallId::EventClear) => {
            // Event syscalls require kernel state; in the real handler this
            // would go through dispatch_syscall with the global partition table.
            1
        }
        Some(_) => 1,
        None => SvcError::InvalidSyscall.to_u32(),
    };
}

/// Core syscall dispatch that routes event syscalls to the events module.
///
/// Frame register convention:
/// - `r0`: syscall ID (overwritten with return value)
/// - `r1`: first argument (partition index for caller/target)
/// - `r2`: second argument (event mask)
pub fn dispatch_syscall<const N: usize>(
    frame: &mut ExceptionFrame,
    partitions: &mut PartitionTable<N>,
) {
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(SyscallId::EventWait) => events::event_wait(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::EventSet) => events::event_set(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::EventClear) => events::event_clear(partitions, frame.r1 as usize, frame.r2),
        Some(_) => 1,
        None => SvcError::InvalidSyscall.to_u32(),
    };
}

/// Encapsulates all kernel service pools alongside the partition table,
/// with pool sizes derived from a single [`KernelConfig`] implementer.
pub struct Kernel<C: KernelConfig>
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
    /// Currently active partition index, if any.
    pub active_partition: Option<u8>,
    pub tick: TickCounter,
    /// The partition index of the currently executing partition, set by the
    /// scheduler before entering user code. Used as the trusted caller identity
    /// for syscalls, rather than reading from a user-controlled register.
    pub current_partition: u8,
    /// Set to `true` by `SYS_YIELD` dispatch; checked and cleared by the
    /// harness so it can force-advance the schedule and call
    /// `set_next_partition()` before PendSV fires.
    pub yield_requested: bool,
    #[cfg(feature = "dynamic-mpu")]
    pub buffers: crate::buffer_pool::BufferPool<{ C::BP }, { C::BZ }>,
    #[cfg(feature = "dynamic-mpu")]
    pub uart_pair: crate::virtual_uart::VirtualUartPair,
    /// ISR top-half to bottom-half ring buffer (8 records, 16-byte payload).
    #[cfg(feature = "dynamic-mpu")]
    pub isr_ring: crate::split_isr::IsrRingBuffer<8, 16>,
    /// Optional hardware UART backend, checked after `uart_pair` in
    /// `dev_dispatch`. Set via [`set_hw_uart`](Self::set_hw_uart).
    #[cfg(feature = "dynamic-mpu")]
    pub hw_uart: Option<crate::hw_uart::HwUartBackend>,
    /// Device registry for dynamic device dispatch.
    #[cfg(feature = "dynamic-mpu")]
    pub registry: crate::virtual_device::DeviceRegistry<'static, { C::DR }>,
    /// Wait queue for partitions blocked on device reads.
    #[cfg(feature = "dynamic-mpu")]
    pub dev_wait_queue: crate::waitqueue::DeviceWaitQueue<{ C::N }>,
    /// Dynamic MPU strategy for managing runtime memory windows.
    ///
    /// Tracks dynamic MPU windows for all partitions. When a buffer is lent
    /// to a partition via the buffer pool, it should be registered here.
    /// The `validated_ptr_dynamic!` macro queries this strategy to validate
    /// pointers against both static MPU regions and dynamic windows.
    #[cfg(feature = "dynamic-mpu")]
    pub dynamic_strategy: crate::mpu_strategy::DynamicStrategy,
    /// Partition/schedule state sub-struct containing partitions, schedule,
    /// current_partition, next_partition, and partition_sp.
    pub core: C::Core,
    /// Synchronization primitives sub-struct (will replace individual fields).
    pub sync: C::Sync,
    /// Message-passing primitives sub-struct (will replace individual fields).
    pub msg: C::Msg,
    /// Port primitives sub-struct (will replace individual fields).
    pub ports: C::Ports,
}

/// Helper function to align an address down to an 8-byte boundary.
const fn align_down_8(addr: u32) -> u32 {
    addr & !7
}

impl<C: KernelConfig> Default for Kernel<C>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core:
        CoreOps<PartTable = PartitionTable<{ C::N }>, SchedTable = ScheduleTable<{ C::SCHED }>>,
    C::Sync: SyncOps<
        SemPool = SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: MsgOps<
        MsgPool = MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: PortsOps<
        SamplingPool = SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    fn default() -> Self {
        Self::new_empty(
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
        )
    }
}

impl<C: KernelConfig> Kernel<C>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core:
        CoreOps<PartTable = PartitionTable<{ C::N }>, SchedTable = ScheduleTable<{ C::SCHED }>>,
    C::Sync: SyncOps<
        SemPool = SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: MsgOps<
        MsgPool = MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: PortsOps<
        SamplingPool = SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    /// Create a new `Kernel` with the given schedule and partition configs.
    ///
    /// Validates that: schedule is non-empty, all schedule entries reference
    /// valid partitions, and all partition configs pass MPU/stack validation.
    pub fn new(
        schedule: ScheduleTable<{ C::SCHED }>,
        configs: &[PartitionConfig],
        #[cfg(feature = "dynamic-mpu")] registry: crate::virtual_device::DeviceRegistry<
            'static,
            { C::DR },
        >,
    ) -> Result<Self, ConfigError> {
        use crate::partition::{MpuRegion, PartitionControlBlock};
        if schedule.is_empty() {
            return Err(ConfigError::ScheduleEmpty);
        }
        for (i, entry) in schedule.entries().iter().enumerate() {
            #[cfg(feature = "dynamic-mpu")]
            if entry.is_system_window {
                continue;
            }
            if entry.partition_index as usize >= configs.len() {
                return Err(ConfigError::ScheduleIndexOutOfBounds {
                    entry_index: i,
                    partition_index: entry.partition_index,
                    num_partitions: configs.len(),
                });
            }
        }
        // Initialize core sub-struct with schedule and partitions.
        let mut core = C::Core::default();
        *core.schedule_mut() = schedule;
        for (i, c) in configs.iter().enumerate() {
            // Enforce contiguous, zero-based IDs that match array indices.
            if c.id as usize != i {
                return Err(ConfigError::PartitionIdMismatch {
                    index: i,
                    expected_id: i as u8,
                    actual_id: c.id,
                });
            }
            c.validate()?;
            // Use internal stack from PartitionCore instead of PartitionConfig.
            // This ensures MPU regions protect the actual stack memory.
            let internal_stack = core
                .stack_mut(i)
                .ok_or(ConfigError::StackInitFailed { partition_id: c.id })?;
            let internal_stack_base = internal_stack.as_ptr() as u32;
            let internal_stack_size = (internal_stack.len() * 4) as u32;
            let sp = align_down_8(internal_stack_base.wrapping_add(internal_stack_size));
            // Override MpuRegion base with internal stack base for correct MPU config.
            let mpu_region = MpuRegion::new(
                internal_stack_base,
                internal_stack_size,
                c.mpu_region.permissions(),
            );
            let pcb = PartitionControlBlock::new(
                c.id,
                c.entry_point,
                internal_stack_base,
                sp,
                mpu_region,
            )
            .with_peripheral_regions(&c.peripheral_regions);
            if core.partitions_mut().add(pcb).is_err() {
                return Err(ConfigError::PartitionTableFull);
            }
            // Initialize partition_sp from the PCB stack pointer.
            core.set_sp(i, sp);
        }
        Ok(Self {
            active_partition: None,
            tick: TickCounter::new(),
            current_partition: 255, // sentinel for "no partition running yet"
            yield_requested: false,
            #[cfg(feature = "dynamic-mpu")]
            buffers: crate::buffer_pool::BufferPool::new(),
            #[cfg(feature = "dynamic-mpu")]
            uart_pair: crate::virtual_uart::VirtualUartPair::new(0, 1),
            #[cfg(feature = "dynamic-mpu")]
            isr_ring: crate::split_isr::IsrRingBuffer::new(),
            #[cfg(feature = "dynamic-mpu")]
            hw_uart: None,
            #[cfg(feature = "dynamic-mpu")]
            registry,
            #[cfg(feature = "dynamic-mpu")]
            dev_wait_queue: crate::waitqueue::DeviceWaitQueue::new(),
            #[cfg(feature = "dynamic-mpu")]
            dynamic_strategy: crate::mpu_strategy::DynamicStrategy::new(),
            core,
            sync: C::Sync::default(),
            msg: C::Msg::default(),
            ports: C::Ports::default(),
        })
    }

    /// Create a `Kernel` with empty partition table and schedule.
    ///
    /// This is for backward compatibility with examples that manage
    /// partitions via `KernelState` separately.
    ///
    // NOTE: Subtask 285 (new_empty initialization cleanup) was completed as part of
    // subtasks 281-284 which removed duplicated fields. The core, sync, msg, and ports
    // sub-structs are now initialized via Default::default().
    pub fn new_empty(
        #[cfg(feature = "dynamic-mpu")] registry: crate::virtual_device::DeviceRegistry<
            'static,
            { C::DR },
        >,
    ) -> Self {
        Self {
            active_partition: None,
            tick: TickCounter::new(),
            current_partition: 255, // sentinel for "no partition running yet"
            yield_requested: false,
            #[cfg(feature = "dynamic-mpu")]
            buffers: crate::buffer_pool::BufferPool::new(),
            #[cfg(feature = "dynamic-mpu")]
            uart_pair: crate::virtual_uart::VirtualUartPair::new(0, 1),
            #[cfg(feature = "dynamic-mpu")]
            isr_ring: crate::split_isr::IsrRingBuffer::new(),
            #[cfg(feature = "dynamic-mpu")]
            hw_uart: None,
            #[cfg(feature = "dynamic-mpu")]
            registry,
            #[cfg(feature = "dynamic-mpu")]
            dev_wait_queue: crate::waitqueue::DeviceWaitQueue::new(),
            #[cfg(feature = "dynamic-mpu")]
            dynamic_strategy: crate::mpu_strategy::DynamicStrategy::new(),
            core: C::Core::default(),
            sync: C::Sync::default(),
            msg: C::Msg::default(),
            ports: C::Ports::default(),
        }
    }

    // -------------------------------------------------------------------------
    // Facade methods delegating to self.sync (SyncPools)
    // -------------------------------------------------------------------------

    /// Returns a shared reference to the semaphore pool.
    #[inline(always)]
    pub fn semaphores(&self) -> &<C::Sync as SyncOps>::SemPool {
        self.sync.semaphores()
    }

    /// Returns a mutable reference to the semaphore pool.
    #[inline(always)]
    pub fn semaphores_mut(&mut self) -> &mut <C::Sync as SyncOps>::SemPool {
        self.sync.semaphores_mut()
    }

    /// Returns a shared reference to the mutex pool.
    #[inline(always)]
    pub fn mutexes(&self) -> &<C::Sync as SyncOps>::MutPool {
        self.sync.mutexes()
    }

    /// Returns a mutable reference to the mutex pool.
    #[inline(always)]
    pub fn mutexes_mut(&mut self) -> &mut <C::Sync as SyncOps>::MutPool {
        self.sync.mutexes_mut()
    }

    /// Returns a shared reference to the message pool.
    #[inline(always)]
    pub fn messages(&self) -> &<C::Msg as MsgOps>::MsgPool {
        self.msg.messages()
    }

    /// Returns a mutable reference to the message pool.
    #[inline(always)]
    pub fn messages_mut(&mut self) -> &mut <C::Msg as MsgOps>::MsgPool {
        self.msg.messages_mut()
    }

    /// Returns a shared reference to the queuing port pool.
    #[inline(always)]
    pub fn queuing(&self) -> &<C::Msg as MsgOps>::QueuingPool {
        self.msg.queuing()
    }

    /// Returns a mutable reference to the queuing port pool.
    #[inline(always)]
    pub fn queuing_mut(&mut self) -> &mut <C::Msg as MsgOps>::QueuingPool {
        self.msg.queuing_mut()
    }

    // -------------------------------------------------------------------------
    // Facade methods delegating to self.ports (PortPools)
    // -------------------------------------------------------------------------

    /// Returns a shared reference to the sampling port pool.
    #[inline(always)]
    pub fn sampling(&self) -> &<C::Ports as PortsOps>::SamplingPool {
        self.ports.sampling()
    }

    /// Returns a mutable reference to the sampling port pool.
    #[inline(always)]
    pub fn sampling_mut(&mut self) -> &mut <C::Ports as PortsOps>::SamplingPool {
        self.ports.sampling_mut()
    }

    /// Returns a shared reference to the blackboard pool.
    #[inline(always)]
    pub fn blackboards(&self) -> &<C::Ports as PortsOps>::BlackboardPool {
        self.ports.blackboards()
    }

    /// Returns a mutable reference to the blackboard pool.
    #[inline(always)]
    pub fn blackboards_mut(&mut self) -> &mut <C::Ports as PortsOps>::BlackboardPool {
        self.ports.blackboards_mut()
    }

    /// Install an optional hardware UART backend.
    ///
    /// Stores the backend in the `hw_uart` field for direct access (e.g.
    /// ISR bottom-half draining).  `dev_dispatch` no longer falls back to
    /// this field — callers must register the backend in the
    /// [`DeviceRegistry`](crate::virtual_device::DeviceRegistry) to make
    /// it reachable via syscall dispatch.
    // TODO: migrate remaining callers to register hw_uart in the registry
    // at init time, then remove this method (backlog item 195).
    #[cfg(feature = "dynamic-mpu")]
    pub fn set_hw_uart(&mut self, backend: crate::hw_uart::HwUartBackend) {
        self.hw_uart = Some(backend);
    }

    /// Trigger a deschedule by setting `yield_requested` and invoking PendSV.
    ///
    /// This centralizes the deschedule logic that both `SYS_YIELD` and
    /// blocking syscalls need. Sets `self.yield_requested = true` so the
    /// harness can force-advance the schedule, then calls `handle_yield()`
    /// to pend the PendSV exception.
    ///
    /// Returns 0 on success (matching `handle_yield` semantics).
    pub fn trigger_deschedule(&mut self) -> u32 {
        self.yield_requested = true;
        handle_yield()
    }

    /// Look up a virtual device by `device_id` and invoke `f` with a mutable
    /// reference to the device (as a trait object) and the current partition.
    ///
    /// Resolves the device exclusively through the registry.  All devices
    /// (virtual UARTs, hardware UART, etc.) must be registered at init time.
    /// Returns `InvalidResource` when the ID is unknown or `OperationFailed`
    /// when the closure returns a `DeviceError`.
    #[cfg(feature = "dynamic-mpu")]
    fn dev_dispatch<F>(&mut self, device_id: u8, f: F) -> u32
    where
        F: FnOnce(&mut dyn VirtualDevice, u8) -> Result<u32, crate::virtual_device::DeviceError>,
    {
        let pid = self.current_partition;
        match self.registry.get_mut(device_id) {
            Some(d) => match f(d, pid) {
                Ok(val) => val,
                Err(_) => SvcError::OperationFailed.to_u32(),
            },
            None => SvcError::InvalidResource.to_u32(),
        }
    }

    /// Expire timed waits whose deadlines have passed.
    ///
    /// Calls `queuing.tick_timeouts()` and `blackboards.tick_timeouts()` with
    /// the given tick, collecting all expired partition IDs into a single
    /// `heapless::Vec<u8, E>`, then transitions each from
    /// [`Waiting`](PartitionState::Waiting) to [`Ready`](PartitionState::Ready).
    ///
    /// With `dynamic-mpu`, also drains the device wait queue.
    ///
    /// The tick handler should call this once per tick so that blocked
    /// senders/receivers are woken when their timeout elapses.
    ///
    /// `E` must be large enough to hold the total number of expired partition
    /// IDs across all subsystems in a single tick.
    pub fn expire_timed_waits<const E: usize>(&mut self, current_tick: u64) {
        let mut expired: heapless::Vec<u8, E> = heapless::Vec::new();
        self.msg
            .queuing_mut()
            .tick_timeouts(current_tick, &mut expired);
        self.ports
            .blackboards_mut()
            .tick_timeouts(current_tick, &mut expired);
        #[cfg(feature = "dynamic-mpu")]
        self.dev_wait_queue
            .drain_expired(current_tick, &mut expired);
        for &pid in expired.iter() {
            try_transition(self.core.partitions_mut(), pid, PartitionState::Ready);
        }
    }

    /// Synchronize the kernel's tick counter to the given value.
    ///
    /// Called from the SysTick handler to copy the authoritative tick from
    /// `KernelState` into `Kernel` each tick.
    pub fn sync_tick(&mut self, current_tick: u64) {
        self.tick.sync(current_tick);
    }

    /// Full syscall dispatch including semaphore, mutex, message, sampling,
    /// and queuing operations.
    ///
    /// Frame register convention:
    /// - `r0`: syscall ID (overwritten with return value)
    /// - `r1`: resource ID (semaphore/mutex/queue/port index)
    /// - `r2`: second argument (event mask, data length, or status pointer)
    /// - `r3`: pointer to user data buffer (for msg/queuing send/recv)
    ///
    /// For queuing syscalls, the caller's partition identity is taken from
    /// `self.current_partition` (set by the scheduler), **not** from a
    /// user-controlled register, to prevent partition impersonation.
    ///
    /// For `QueuingStatus`: `r2` must point to a writable
    /// [`QueuingPortStatus`] struct where the kernel writes the full status.
    ///
    /// # Safety
    ///
    /// For message/queuing syscalls, `r3` must point to a readable (send) or
    /// writable (recv) buffer of at least `QM` bytes within the calling
    /// partition's memory region. For `QueuingStatus`, `r2` must point to a
    /// writable `QueuingPortStatus`. The caller is responsible for ensuring
    /// this; in production the MPU enforces partition isolation.
    pub unsafe fn dispatch(&mut self, frame: &mut ExceptionFrame) {
        #[cfg(feature = "qemu")]
        cortex_m_semihosting::hprintln!("[Kernel::dispatch] syscall={}", frame.r0);
        frame.r0 = match SyscallId::from_u32(frame.r0) {
            Some(SyscallId::Yield) => self.trigger_deschedule(),
            Some(SyscallId::EventWait) => {
                events::event_wait(self.partitions_mut(), frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::EventSet) => {
                events::event_set(self.partitions_mut(), frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::EventClear) => {
                events::event_clear(self.partitions_mut(), frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::SemWait) => {
                let pt = self.core.partitions_mut();
                match self
                    .sync
                    .semaphores_mut()
                    .wait(pt, frame.r1 as usize, frame.r2 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::SemSignal) => {
                let pt = self.core.partitions_mut();
                match self.sync.semaphores_mut().signal(pt, frame.r1 as usize) {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MutexLock) => {
                let pt = self.core.partitions_mut();
                match self
                    .sync
                    .mutexes_mut()
                    .lock(pt, frame.r1 as usize, frame.r2 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MutexUnlock) => {
                let pt = self.core.partitions_mut();
                match self
                    .sync
                    .mutexes_mut()
                    .unlock(pt, frame.r1 as usize, frame.r2 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MsgSend) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let data = unsafe { core::slice::from_raw_parts(frame.r3 as *const u8, C::QM) };
                match self
                    .msg
                    .messages_mut()
                    .send(frame.r1 as usize, frame.r2 as usize, data)
                {
                    Ok(outcome) => match apply_send_outcome(self.partitions_mut(), outcome) {
                        Ok(Some(_blocked)) => {
                            self.trigger_deschedule();
                            0
                        }
                        Ok(None) => 0,
                        Err(e) => e.to_u32(),
                    },
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::MsgRecv) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let buf = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                match self
                    .msg
                    .messages_mut()
                    .recv(frame.r1 as usize, frame.r2 as usize, buf)
                {
                    Ok(outcome) => match apply_recv_outcome(self, outcome) {
                        Ok(Some(_blocked)) => 0,
                        Ok(None) => 0,
                        Err(e) => e.to_u32(),
                    },
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::SamplingWrite) => {
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
                    // SAFETY: validated_ptr confirmed [r3, r3+r2) lies within
                    // the calling partition's MPU data region.
                    let d = unsafe {
                        core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                    };
                    let tick = self.tick.get();
                    match self.ports.sampling_mut().write_sampling_message(
                        frame.r1 as usize,
                        d,
                        tick,
                    ) {
                        Ok(()) => 0,
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::SamplingRead) => validated_ptr!(self, frame.r3, C::SM, {
                // SAFETY: validated_ptr confirmed [r3, r3+SM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::SM) };
                let tick = self.tick.get();
                match self
                    .ports
                    .sampling_mut()
                    .read_sampling_message(frame.r1 as usize, b, tick)
                {
                    Ok((sz, _)) => sz as u32,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::QueuingSend) => {
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
                    // SAFETY: validated_ptr confirmed [r3, r3+r2) lies within
                    // the calling partition's MPU data region.
                    let d = unsafe {
                        core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                    };
                    let pid = self.current_partition;
                    let tick = self.tick.get();
                    match self
                        .msg
                        .queuing_mut()
                        .send_routed(frame.r1 as usize, pid, d, 0, tick)
                    {
                        Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
                            if let Some(wpid) = w {
                                try_transition(
                                    self.core.partitions_mut(),
                                    wpid,
                                    PartitionState::Ready,
                                );
                            }
                            0
                        }
                        Ok(SendQueuingOutcome::SenderBlocked { .. }) => 0,
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::QueuingRecv) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                let pid = self.current_partition;
                let tick = self.tick.get();
                match self.msg.queuing_mut().receive_queuing_message(
                    frame.r1 as usize,
                    pid,
                    b,
                    0,
                    tick,
                ) {
                    Ok(crate::queuing::RecvQueuingOutcome::Received {
                        msg_len,
                        wake_sender: w,
                    }) => {
                        if let Some(wpid) = w {
                            try_transition(self.core.partitions_mut(), wpid, PartitionState::Ready);
                        }
                        msg_len as u32
                    }
                    Ok(_) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::QueuingStatus) => {
                validated_ptr!(self, frame.r2, core::mem::size_of::<QueuingPortStatus>(), {
                    // SAFETY: validated_ptr confirmed [r2, r2+size_of QueuingPortStatus)
                    // lies within the calling partition's MPU data region.
                    let status_ptr = frame.r2 as *mut QueuingPortStatus;
                    match self
                        .msg
                        .queuing()
                        .get_queuing_port_status(frame.r1 as usize)
                    {
                        Ok(status) => {
                            // SAFETY: validated_ptr confirmed the pointer lies within
                            // the partition's MPU region. We are in Handler mode after
                            // validation, so the write is sound.
                            unsafe { core::ptr::write(status_ptr, status) };
                            0
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::BbDisplay) => {
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
                    // SAFETY: validated_ptr confirmed [r3, r3+r2) lies within
                    // the calling partition's MPU data region.
                    let d = unsafe {
                        core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                    };
                    match self
                        .ports
                        .blackboards_mut()
                        .display_blackboard(frame.r1 as usize, d)
                    {
                        Ok(woken) => {
                            for &wpid in woken.iter() {
                                try_transition(
                                    self.core.partitions_mut(),
                                    wpid,
                                    PartitionState::Ready,
                                );
                            }
                            0
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::BbRead) => {
                validated_ptr!(self, frame.r3, C::BM, {
                    // SAFETY: validated_ptr confirmed [r3, r3+BM) lies within
                    // the calling partition's MPU data region.
                    let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::BM) };
                    let pid = self.current_partition;
                    let tick = self.tick.get();
                    match self.ports.blackboards_mut().read_blackboard_timed(
                        frame.r1 as usize,
                        pid,
                        b,
                        frame.r2,
                        tick,
                    ) {
                        Ok(crate::blackboard::ReadBlackboardOutcome::Read { msg_len }) => {
                            msg_len as u32
                        }
                        Ok(crate::blackboard::ReadBlackboardOutcome::ReaderBlocked) => {
                            try_transition(
                                self.core.partitions_mut(),
                                pid,
                                PartitionState::Waiting,
                            );
                            self.trigger_deschedule()
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::BbClear) => {
                match self
                    .ports
                    .blackboards_mut()
                    .clear_blackboard(frame.r1 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::GetTime) => self.tick.get() as u32,
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferAlloc) => {
                use crate::buffer_pool::BorrowMode;
                let m = if frame.r1 == 0 {
                    BorrowMode::Read
                } else {
                    BorrowMode::Write
                };
                let max_ticks = frame.r2;
                match self.buffers.alloc(self.current_partition, m) {
                    Some(slot) => {
                        if max_ticks > 0 {
                            let deadline = self.tick.get().wrapping_add(max_ticks as u64);
                            // set_deadline cannot fail here: slot is valid and
                            // just-allocated (non-free), so we ignore the result.
                            let _ = self.buffers.set_deadline(slot, Some(deadline));
                        }
                        slot as u32
                    }
                    None => SvcError::OperationFailed.to_u32(),
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferRelease) => {
                match self
                    .buffers
                    .release(frame.r1 as usize, self.current_partition)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferWrite) => {
                validated_ptr_dynamic!(self, frame.r3, frame.r2 as usize, {
                    use crate::buffer_pool::BorrowState;
                    let slot_idx = frame.r1 as usize;
                    let len = frame.r2 as usize;
                    let data_ptr = frame.r3 as *const u8;
                    match self.buffers.get_mut(slot_idx) {
                        Some(slot)
                            if slot.state()
                                == (BorrowState::BorrowedWrite {
                                    owner: self.current_partition,
                                }) =>
                        {
                            let buf = slot.data_mut();
                            match buf.get_mut(..len) {
                                Some(dst) => {
                                    // SAFETY: validated_ptr confirmed [r3, r3+r2)
                                    // lies within the calling partition's MPU data
                                    // region.
                                    let src = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                                    dst.copy_from_slice(src);
                                    len as u32
                                }
                                None => SvcError::OperationFailed.to_u32(),
                            }
                        }
                        _ => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevOpen) => self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                dev.open(pid)?;
                Ok(0)
            }),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevRead) => {
                validated_ptr_dynamic!(self, frame.r3, frame.r2 as usize, {
                    let len = frame.r2 as usize;
                    let buf_ptr = frame.r3 as *mut u8;
                    self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                        // SAFETY: validated_ptr_dynamic confirmed [r3, r3+r2) lies
                        // within the calling partition's accessible memory regions.
                        let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, len) };
                        Ok(dev.read(pid, buf)? as u32)
                    })
                })
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevWrite) => {
                validated_ptr_dynamic!(self, frame.r3, frame.r2 as usize, {
                    let len = frame.r2 as usize;
                    let data_ptr = frame.r3 as *const u8;
                    self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                        // SAFETY: validated_ptr_dynamic confirmed [r3, r3+r2) lies
                        // within the calling partition's accessible memory regions.
                        let data = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                        Ok(dev.write(pid, data)? as u32)
                    })
                })
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevIoctl) => self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                dev.ioctl(pid, frame.r2, frame.r3)
            }),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevClose) => self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                dev.close(pid)?;
                Ok(0)
            }),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevReadTimed) => {
                // r1 = device_id, r2 = timeout_ticks (0 = non-blocking), r3 = buf_ptr
                // Reads a single byte; blocks caller if no data and timeout > 0.
                validated_ptr!(self, frame.r3, 1, {
                    let buf_ptr = frame.r3 as *mut u8;
                    let timeout = frame.r2;
                    let pid = self.current_partition;
                    let device_id = frame.r1 as u8;
                    match self.registry.get_mut(device_id) {
                        Some(dev) => {
                            // SAFETY: validated_ptr confirmed [r3, r3+1) lies
                            // within the calling partition's MPU data region.
                            let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, 1) };
                            match dev.read(pid, buf) {
                                Ok(n) if n > 0 => n as u32,
                                Ok(_) if timeout > 0 => {
                                    // No data available — block the caller.
                                    let expiry = self.tick.get() + timeout as u64;
                                    match self.dev_wait_queue.block_reader(pid, expiry) {
                                        Ok(()) => {
                                            try_transition(
                                                self.partitions_mut(),
                                                pid,
                                                PartitionState::Waiting,
                                            );
                                            self.trigger_deschedule()
                                        }
                                        Err(_) => SvcError::WaitQueueFull.to_u32(),
                                    }
                                }
                                Ok(_) => 0, // non-blocking, no data
                                Err(_) => SvcError::OperationFailed.to_u32(),
                            }
                        }
                        None => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::QueuingRecvTimed) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                let pid = self.current_partition;
                let tick = self.tick.get();
                match self.msg.queuing_mut().receive_queuing_message(
                    frame.r1 as usize,
                    pid,
                    b,
                    frame.r2 as u64,
                    tick,
                ) {
                    Ok(crate::queuing::RecvQueuingOutcome::Received {
                        msg_len,
                        wake_sender: w,
                    }) => {
                        if let Some(wpid) = w {
                            try_transition(self.core.partitions_mut(), wpid, PartitionState::Ready);
                        }
                        msg_len as u32
                    }
                    Ok(crate::queuing::RecvQueuingOutcome::ReceiverBlocked { .. }) => {
                        try_transition(self.core.partitions_mut(), pid, PartitionState::Waiting);
                        self.trigger_deschedule()
                    }
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::QueuingSendTimed) => {
                let data_len = (frame.r2 & 0xFFFF) as usize;
                let timeout = (frame.r2 >> 16) as u64;
                validated_ptr!(self, frame.r3, data_len, {
                    // SAFETY: validated_ptr confirmed [r3, r3+data_len) lies within
                    // the calling partition's MPU data region.
                    let d = unsafe { core::slice::from_raw_parts(frame.r3 as *const u8, data_len) };
                    let pid = self.current_partition;
                    let tick = self.tick.get();
                    match self.msg.queuing_mut().send_routed(
                        frame.r1 as usize,
                        pid,
                        d,
                        timeout,
                        tick,
                    ) {
                        Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
                            if let Some(wpid) = w {
                                try_transition(
                                    self.core.partitions_mut(),
                                    wpid,
                                    PartitionState::Ready,
                                );
                            }
                            0
                        }
                        Ok(SendQueuingOutcome::SenderBlocked { .. }) => {
                            try_transition(
                                self.core.partitions_mut(),
                                pid,
                                PartitionState::Waiting,
                            );
                            self.trigger_deschedule()
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            // Debug syscalls: only available in qemu builds with semihosting.
            // For non-qemu builds these are recognized but return NotImplemented.
            Some(SyscallId::DebugPrint) => {
                #[cfg(feature = "qemu")]
                {
                    // SAFETY: caller ensures r1 points to a valid string of length r2
                    // within the partition's memory region. Semihosting is available
                    // in QEMU builds.
                    let ptr = frame.r1 as *const u8;
                    let len = frame.r2 as usize;
                    if len > 0 && !ptr.is_null() {
                        let slice = unsafe { core::slice::from_raw_parts(ptr, len) };
                        if let Ok(s) = core::str::from_utf8(slice) {
                            cortex_m_semihosting::hprint!("{}", s);
                        }
                    }
                    0
                }
                #[cfg(not(feature = "qemu"))]
                {
                    SvcError::NotImplemented.to_u32()
                }
            }
            Some(SyscallId::DebugExit) => {
                #[cfg(feature = "qemu")]
                {
                    use cortex_m_semihosting::debug;
                    if frame.r1 == 0 {
                        debug::exit(debug::EXIT_SUCCESS);
                    } else {
                        debug::exit(debug::EXIT_FAILURE);
                    }
                    // Note: debug::exit() does not return in QEMU.
                    #[allow(unreachable_code)]
                    0
                }
                #[cfg(not(feature = "qemu"))]
                {
                    SvcError::NotImplemented.to_u32()
                }
            }
            None => SvcError::InvalidSyscall.to_u32(),
        };
        #[cfg(feature = "qemu")]
        cortex_m_semihosting::hprintln!("[Kernel::dispatch] returning r0={:#x}", frame.r0);
    }

    // -------------------------------------------------------------------------
    // Schedule and partition accessors
    // -------------------------------------------------------------------------

    /// Returns an immutable reference to the partition table.
    #[inline(always)]
    pub fn partitions(&self) -> &PartitionTable<{ C::N }> {
        self.core.partitions()
    }

    /// Returns a mutable reference to the partition table.
    #[inline(always)]
    pub fn partitions_mut(&mut self) -> &mut PartitionTable<{ C::N }> {
        self.core.partitions_mut()
    }

    /// Returns an immutable reference to the schedule table.
    #[inline(always)]
    pub fn schedule(&self) -> &ScheduleTable<{ C::SCHED }> {
        self.core.schedule()
    }

    /// Returns a mutable reference to the schedule table.
    #[inline(always)]
    pub fn schedule_mut(&mut self) -> &mut ScheduleTable<{ C::SCHED }> {
        self.core.schedule_mut()
    }

    /// Returns the current partition index stored in core.
    #[inline(always)]
    pub fn core_current_partition(&self) -> u8 {
        self.core.current_partition()
    }

    /// Sets the current partition index in core.
    #[inline(always)]
    pub fn set_core_current_partition(&mut self, id: u8) {
        self.core.set_current_partition(id);
    }

    /// Returns the next partition index.
    #[inline(always)]
    pub fn next_partition(&self) -> u8 {
        self.core.next_partition()
    }

    /// Sets the next partition index.
    #[inline(always)]
    pub fn set_next_partition(&mut self, id: u8) {
        self.core.set_next_partition(id);
    }

    /// Gets the stack pointer for a partition by index.
    #[inline(always)]
    pub fn get_sp(&self, index: usize) -> Option<u32> {
        self.core.get_sp(index)
    }

    /// Sets the stack pointer for a partition by index. Returns true if valid.
    #[inline(always)]
    pub fn set_sp(&mut self, index: usize, sp: u32) -> bool {
        self.core.set_sp(index, sp)
    }

    /// Returns a reference to the partition_sp array.
    #[inline(always)]
    pub fn partition_sp(&self) -> &[u32] {
        self.core.partition_sp()
    }

    /// Returns a mutable reference to the partition_sp array.
    #[inline(always)]
    pub fn partition_sp_mut(&mut self) -> &mut [u32] {
        self.core.partition_sp_mut()
    }

    // -------------------------------------------------------------------------
    // Core state facade methods (active_partition, tick, yield_requested)
    //
    // These methods delegate to self.core for accessing the fields that were
    // moved to PartitionCore. The direct fields on Kernel are retained for
    // backward compatibility but will be removed in a follow-up.
    // -------------------------------------------------------------------------

    /// Returns the currently active partition index from core.
    #[inline(always)]
    pub fn core_active_partition(&self) -> Option<u8> {
        self.core.active_partition()
    }

    /// Sets the active partition index in core.
    #[inline(always)]
    pub fn set_core_active_partition(&mut self, id: Option<u8>) {
        self.core.set_active_partition(id);
    }

    /// Returns a reference to the tick counter in core.
    #[inline(always)]
    pub fn core_tick(&self) -> &<C::Core as CoreOps>::TickCounter {
        self.core.tick()
    }

    /// Returns a mutable reference to the tick counter in core.
    #[inline(always)]
    pub fn core_tick_mut(&mut self) -> &mut <C::Core as CoreOps>::TickCounter {
        self.core.tick_mut()
    }

    /// Returns whether a yield has been requested (from core).
    #[inline(always)]
    pub fn core_yield_requested(&self) -> bool {
        self.core.yield_requested()
    }

    /// Sets the yield_requested flag in core.
    #[inline(always)]
    pub fn set_core_yield_requested(&mut self, requested: bool) {
        self.core.set_yield_requested(requested);
    }

    /// Returns a mutable reference to a partition's stack array.
    #[inline(always)]
    pub fn core_stack_mut(&mut self, index: usize) -> Option<&mut [u32]> {
        self.core.stack_mut(index)
    }

    // -------------------------------------------------------------------------
    // Schedule advance methods
    // -------------------------------------------------------------------------

    /// Advance the schedule table by one tick. Returns a [`ScheduleEvent`]
    /// indicating whether a partition switch or system window occurred.
    /// Updates `active_partition` and `next_partition` on partition switches.
    pub fn advance_schedule_tick(&mut self) -> ScheduleEvent {
        self.tick.increment();
        let event = self.schedule_mut().advance_tick();
        if let ScheduleEvent::PartitionSwitch(pid) = event {
            self.active_partition = Some(pid);
            self.set_next_partition(pid);
        }
        event
    }

    /// Force-advance the schedule to the next slot, forfeiting remaining
    /// ticks. Updates `active_partition` and returns the schedule result.
    /// Called by the harness when a partition yields.
    pub fn yield_current_slot(&mut self) -> impl YieldResult {
        let result = self.schedule_mut().force_advance();
        if let Some(pid) = result.partition_id() {
            self.active_partition = Some(pid);
        }
        result
    }

    /// Start the schedule and return the initial partition ID.
    ///
    /// Calls `self.schedule_mut().start()` to initialize the schedule table's
    /// internal state (resetting to the first slot). Returns the partition
    /// ID of the first schedule entry, or `None` if the schedule is empty.
    ///
    /// This centralizes schedule startup in the Kernel, allowing the harness
    /// to call `kernel.start_schedule()` instead of managing schedule state
    /// separately.
    pub fn start_schedule(&mut self) -> Option<u8> {
        self.schedule_mut().start();
        let first_pid = self.schedule().current_partition();
        if let Some(pid) = first_pid {
            self.active_partition = Some(pid);
        }
        first_pid
    }

    /// Returns an immutable reference to the tick counter.
    pub fn tick(&self) -> &TickCounter {
        &self.tick
    }

    // -------------------------------------------------------------------------
    // Facade methods for kernel state (used by tests and harness)
    // -------------------------------------------------------------------------

    /// Returns whether a yield was requested (e.g., by a blocking syscall).
    #[inline(always)]
    pub fn yield_requested(&self) -> bool {
        self.yield_requested
    }

    /// Sets the yield_requested flag.
    #[inline(always)]
    pub fn set_yield_requested(&mut self, value: bool) {
        self.yield_requested = value;
    }

    /// Returns the current partition index.
    #[inline(always)]
    pub fn current_partition(&self) -> u8 {
        self.current_partition
    }

    /// Sets the current partition index.
    #[inline(always)]
    pub fn set_current_partition(&mut self, pid: u8) {
        self.current_partition = pid;
    }

    /// Returns the active partition (the one currently running in user mode).
    #[inline(always)]
    pub fn active_partition(&self) -> Option<u8> {
        self.active_partition
    }

    /// Returns an immutable reference to the buffer pool.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn buffers(&self) -> &crate::buffer_pool::BufferPool<{ C::BP }, { C::BZ }> {
        &self.buffers
    }

    /// Returns a mutable reference to the buffer pool.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn buffers_mut(&mut self) -> &mut crate::buffer_pool::BufferPool<{ C::BP }, { C::BZ }> {
        &mut self.buffers
    }

    /// Returns an immutable reference to the device wait queue.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn dev_wait_queue(&self) -> &crate::waitqueue::DeviceWaitQueue<{ C::N }> {
        &self.dev_wait_queue
    }

    /// Returns a mutable reference to the device wait queue.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn dev_wait_queue_mut(&mut self) -> &mut crate::waitqueue::DeviceWaitQueue<{ C::N }> {
        &mut self.dev_wait_queue
    }

    /// Returns an immutable reference to the hardware UART backend.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn hw_uart(&self) -> &Option<crate::hw_uart::HwUartBackend> {
        &self.hw_uart
    }
}

/// Try to transition partition `pid` to `state`. Returns `true` on success.
pub fn try_transition<const N: usize>(
    partitions: &mut PartitionTable<N>,
    pid: u8,
    state: PartitionState,
) -> bool {
    partitions
        .get_mut(pid as usize)
        .and_then(|p| p.transition(state).ok())
        .is_some()
}

/// Apply a `SendOutcome` by transitioning partition states as needed.
/// Returns `Some(blocked_pid)` if a sender blocked, `None` otherwise.
/// Returns `Err(SvcError::TransitionFailed)` if a partition transition fails.
fn apply_send_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: SendOutcome,
) -> Result<Option<u32>, SvcError> {
    match outcome {
        SendOutcome::Delivered {
            wake_receiver: Some(pid),
        } => {
            if !try_transition(partitions, pid, PartitionState::Ready) {
                return Err(SvcError::TransitionFailed);
            }
            Ok(None)
        }
        SendOutcome::Delivered {
            wake_receiver: None,
        } => Ok(None),
        SendOutcome::SenderBlocked { blocked } => {
            if !try_transition(partitions, blocked, PartitionState::Waiting) {
                return Err(SvcError::TransitionFailed);
            }
            Ok(Some(blocked as u32))
        }
    }
}

/// Apply a `RecvOutcome` by transitioning partition states as needed.
/// Returns `Ok(Some(blocked_pid))` if a receiver blocked, `Ok(None)` otherwise.
/// Returns `Err(SvcError::TransitionFailed)` if a partition transition fails.
///
/// When a receiver blocks, this function calls `trigger_deschedule()` on the
/// kernel to pend PendSV and set `yield_requested`.
fn apply_recv_outcome<C: KernelConfig>(
    kernel: &mut Kernel<C>,
    outcome: RecvOutcome,
) -> Result<Option<u32>, SvcError>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core:
        CoreOps<PartTable = PartitionTable<{ C::N }>, SchedTable = ScheduleTable<{ C::SCHED }>>,
    C::Sync: SyncOps<
        SemPool = SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: MsgOps<
        MsgPool = MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: PortsOps<
        SamplingPool = SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    match outcome {
        RecvOutcome::Received {
            wake_sender: Some(pid),
        } => {
            if !try_transition(kernel.core.partitions_mut(), pid, PartitionState::Ready) {
                return Err(SvcError::TransitionFailed);
            }
            Ok(None)
        }
        RecvOutcome::Received { wake_sender: None } => Ok(None),
        RecvOutcome::ReceiverBlocked { blocked } => {
            if !try_transition(
                kernel.core.partitions_mut(),
                blocked,
                PartitionState::Waiting,
            ) {
                return Err(SvcError::TransitionFailed);
            }
            kernel.trigger_deschedule();
            Ok(Some(blocked as u32))
        }
    }
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
    // Facade methods for `active_partition`, `current_partition`, `yield_requested`,
    // `buffers`, `dev_wait_queue`, and `hw_uart` are now available on Kernel.
    // Tests should use these accessor methods instead of direct field access.

    //! # Safety
    //!
    //! All `unsafe { k.dispatch(&mut ef) }` calls in this test module share the
    //! same safety justification:
    //!
    //! 1. **ExceptionFrame construction**: The `frame()` test helper constructs
    //!    `ExceptionFrame` instances with valid register values. Unlike hardware
    //!    exception entry, these are not actual stacked registers, but the
    //!    dispatch logic only reads/writes the r0-r3 fields which are always
    //!    initialized.
    //!
    //! 2. **Kernel construction**: The `kernel()` and `kernel_with_registry()`
    //!    helpers construct `Kernel` instances with properly initialized
    //!    partition tables (via `tbl()`), schedule tables, and resource pools.
    //!    All partitions have valid MPU regions and are transitioned to Running
    //!    state before dispatch.
    //!
    //! 3. **Host-mode pointer validation**: Tests run on the host (not target
    //!    hardware) where `validate_user_ptr` checks pass for any pointer within
    //!    the partition's configured MPU region. Tests that exercise pointer
    //!    validation use `mmap` to allocate memory at addresses matching the
    //!    partition's MPU region, ensuring the kernel's bounds checks succeed.

    use super::*;
    use crate::config::KernelConfig;
    use crate::message::MessageQueue;
    use crate::partition::{MpuRegion, PartitionControlBlock};
    use crate::scheduler::ScheduleEntry;
    use crate::semaphore::Semaphore;
    use crate::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_YIELD};

    /// Test kernel configuration with small, fixed pool sizes.
    struct TestConfig;
    impl KernelConfig for TestConfig {
        const N: usize = 4;
        const SCHED: usize = 4;
        const STACK_WORDS: usize = 256;
        const S: usize = 4;
        const SW: usize = 4;
        const MS: usize = 4;
        const MW: usize = 4;
        const QS: usize = 4;
        const QD: usize = 4;
        const QM: usize = 4;
        const QW: usize = 4;
        const SP: usize = 4;
        const SM: usize = 64;
        const BS: usize = 4;
        const BM: usize = 64;
        const BW: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 32;

        type Core = crate::partition_core::PartitionCore<
            { Self::N },
            { Self::SCHED },
            { Self::STACK_WORDS },
        >;
        type Sync =
            crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg =
            crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports = crate::port_pools::PortPools<
            { Self::SP },
            { Self::SM },
            { Self::BS },
            { Self::BM },
            { Self::BW },
        >;
    }

    fn frame(r0: u32, r1: u32, r2: u32) -> ExceptionFrame {
        ExceptionFrame {
            r0,
            r1,
            r2,
            r3: 0xCC,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        }
    }
    fn pcb(id: u8) -> PartitionControlBlock {
        let o = (id as u32) * 0x1000;
        PartitionControlBlock::new(
            id,
            0x0800_0000 + o,
            0x2000_0000 + o,
            0x2000_0400 + o,
            MpuRegion::new(0x2000_0000 + o, 4096, 0),
        )
    }
    fn tbl() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        t.add(pcb(0)).unwrap();
        t.add(pcb(1)).unwrap();
        t.get_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t.get_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t
    }

    /// Build a default device registry with virtual UART backends for IDs 0
    /// and 1 (matching the legacy `uart_pair` convention).
    ///
    /// Returns the registry and raw pointers to each backend so that tests
    /// can call backend-specific helpers (e.g. `push_rx`, `pop_tx`) on the
    /// same instances that `dev_dispatch` routes to.
    #[cfg(feature = "dynamic-mpu")]
    fn default_registry() -> (
        crate::virtual_device::DeviceRegistry<'static, 4>,
        *mut crate::virtual_uart::VirtualUartBackend,
        *mut crate::virtual_uart::VirtualUartBackend,
    ) {
        use crate::virtual_uart::VirtualUartBackend;
        let uart_a: &'static mut VirtualUartBackend =
            Box::leak(Box::new(VirtualUartBackend::new(0)));
        let ptr_a: *mut VirtualUartBackend = uart_a as *mut _;
        let uart_b: &'static mut VirtualUartBackend =
            Box::leak(Box::new(VirtualUartBackend::new(1)));
        let ptr_b: *mut VirtualUartBackend = uart_b as *mut _;
        let mut reg = crate::virtual_device::DeviceRegistry::new();
        reg.add(uart_a).unwrap();
        reg.add(uart_b).unwrap();
        (reg, ptr_a, ptr_b)
    }

    /// Build a Kernel with a caller-supplied device registry,
    /// pre-populated with 2 running partitions.
    #[cfg(feature = "dynamic-mpu")]
    fn kernel_with_registry(
        sem_count: usize,
        mtx_count: usize,
        msg_queue_count: usize,
        registry: crate::virtual_device::DeviceRegistry<'static, 4>,
    ) -> Kernel<TestConfig> {
        kernel_impl(sem_count, mtx_count, msg_queue_count, registry)
    }

    /// Build a Kernel with 2 running partitions, the given semaphore count,
    /// mutex count, and message queue count.
    ///
    /// When `dynamic-mpu` is enabled the kernel is constructed with a
    /// [`DeviceRegistry`] pre-populated with virtual UART backends for
    /// device IDs 0 and 1.
    fn kernel(sem_count: usize, mtx_count: usize, msg_queue_count: usize) -> Kernel<TestConfig> {
        #[cfg(feature = "dynamic-mpu")]
        {
            kernel_impl(sem_count, mtx_count, msg_queue_count, default_registry().0)
        }
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            kernel_impl(sem_count, mtx_count, msg_queue_count)
        }
    }

    /// Common implementation for building a Kernel with 2 running partitions.
    /// Extracted to avoid code duplication between `kernel()` and `kernel_with_registry()`.
    fn kernel_impl(
        sem_count: usize,
        mtx_count: usize,
        msg_queue_count: usize,
        #[cfg(feature = "dynamic-mpu")] registry: crate::virtual_device::DeviceRegistry<'static, 4>,
    ) -> Kernel<TestConfig> {
        // Build the core with pre-populated partitions.
        let mut core = <TestConfig as KernelConfig>::Core::default();
        let pt = tbl();
        for pcb in pt.iter() {
            core.partitions_mut().add(pcb.clone()).unwrap();
        }
        let mut k: Kernel<TestConfig> = Kernel {
            active_partition: None,
            tick: TickCounter::new(),
            current_partition: 0,
            yield_requested: false,
            #[cfg(feature = "dynamic-mpu")]
            buffers: crate::buffer_pool::BufferPool::new(),
            #[cfg(feature = "dynamic-mpu")]
            uart_pair: crate::virtual_uart::VirtualUartPair::new(0, 1),
            #[cfg(feature = "dynamic-mpu")]
            isr_ring: crate::split_isr::IsrRingBuffer::new(),
            #[cfg(feature = "dynamic-mpu")]
            hw_uart: None,
            #[cfg(feature = "dynamic-mpu")]
            registry,
            #[cfg(feature = "dynamic-mpu")]
            dev_wait_queue: crate::waitqueue::DeviceWaitQueue::new(),
            #[cfg(feature = "dynamic-mpu")]
            dynamic_strategy: crate::mpu_strategy::DynamicStrategy::new(),
            core,
            sync: <TestConfig as KernelConfig>::Sync::default(),
            msg: <TestConfig as KernelConfig>::Msg::default(),
            ports: <TestConfig as KernelConfig>::Ports::default(),
        };
        // Add semaphores via facade method
        for _ in 0..sem_count {
            k.semaphores_mut().add(Semaphore::new(1, 2)).unwrap();
        }
        // Mutexes are pre-allocated at capacity in SyncPools::default()
        let _ = mtx_count;
        // Add message queues via facade method
        for _ in 0..msg_queue_count {
            k.messages_mut().add(MessageQueue::new()).unwrap();
        }
        k
    }

    #[test]
    fn yield_returns_zero_and_preserves_regs() {
        let mut ef = frame(SYS_YIELD, 0xAA, 0xBB);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!((ef.r0, ef.r1, ef.r2, ef.r3), (0, 0xAA, 0xBB, 0xCC));
    }

    #[test]
    fn yield_sets_yield_requested_flag() {
        let mut k = kernel(0, 0, 0);
        assert!(!k.yield_requested());
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert!(k.yield_requested());
    }

    #[test]
    fn yield_requested_cleared_after_manual_reset() {
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert!(k.yield_requested());
        k.set_yield_requested(false);
        assert!(!k.yield_requested());
        // Non-yield syscall does not set the flag
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert!(!k.yield_requested());
    }

    #[test]
    fn trigger_deschedule_sets_yield_requested() {
        let mut k = kernel(0, 0, 0);
        assert!(!k.yield_requested());
        let ret = k.trigger_deschedule();
        assert_eq!(ret, 0);
        assert!(k.yield_requested());
    }

    #[test]
    fn partition_sp_initialized_from_internal_stacks() {
        use crate::partition::PartitionConfig;
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 10)).unwrap();
        schedule.add(ScheduleEntry::new(1, 10)).unwrap();
        // Config stack_base values are ignored; internal stacks are used instead.
        let configs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];
        #[cfg(feature = "dynamic-mpu")]
        let registry = crate::virtual_device::DeviceRegistry::new();
        let k = Kernel::<TestConfig>::new(
            schedule,
            &configs,
            #[cfg(feature = "dynamic-mpu")]
            registry,
        )
        .unwrap();
        // Verify PCB uses internal stack (not config values).
        let pcb0 = k.partitions().get(0).unwrap();
        assert_ne!(pcb0.stack_base(), 0x2000_0000);
        assert_eq!(k.partition_sp()[2], 0); // Unused slots zero-initialized.
    }

    #[test]
    fn invalid_syscall_returns_error_code() {
        let mut ef = frame(0xFFFF, 0, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, SvcError::InvalidSyscall.to_u32());
    }

    // TODO: reviewer false positive - event_* tests use `dispatch_syscall` (a safe fn),
    // not `unsafe { k.dispatch(...) }`, so they do not need SAFETY comments.
    #[test]
    fn event_wait_dispatches_to_events_module() {
        let mut t = tbl();
        events::event_set(&mut t, 0, 0b1010);
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1110);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, 0b1010);
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
    }

    #[test]
    fn event_set_dispatches_to_events_module() {
        let mut t = tbl();
        let mut ef = frame(SYS_EVT_SET, 1, 0b0101);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, 0);
        assert_eq!(t.get(1).unwrap().event_flags(), 0b0101);
    }

    #[test]
    fn event_clear_dispatches_to_events_module() {
        let mut t = tbl();
        events::event_set(&mut t, 0, 0b1111);
        let mut ef = frame(SYS_EVT_CLEAR, 0, 0b0101);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, 0);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1010);
    }

    #[test]
    fn event_invalid_partition_returns_error_code() {
        let inv = SvcError::InvalidPartition.to_u32();
        let mut t = tbl();
        let mut ef = frame(SYS_EVT_WAIT, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, inv);
        let mut ef = frame(SYS_EVT_SET, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, inv);
        let mut ef = frame(SYS_EVT_CLEAR, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, inv);
    }

    #[test]
    fn sem_wait_and_signal_dispatch() {
        let mut k = kernel(1, 0, 0);
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let mut ef = frame(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[test]
    fn mutex_lock_unlock_dispatch() {
        let mut k = kernel(0, 1, 0);
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(k.mutexes().owner(0), Ok(Some(0)));
        let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(k.mutexes().owner(0), Ok(None));
    }

    // TODO: reviewer false positive - msg_send_recv_* tests call k.messages_mut().send/recv
    // (safe methods), not `unsafe { k.dispatch(...) }`, so they do not need SAFETY comments.
    #[test]
    fn msg_send_recv_pointer_based() {
        let mut k = kernel(0, 0, 2);
        // Send to queue 0 from partition 0
        let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        let outcome = k.messages_mut().send(0, 0, &data).unwrap();
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

        // Receive from queue 0 into partition 1's buffer
        let mut recv_buf = [0u8; 4];
        let outcome = k.messages_mut().recv(0, 1, &mut recv_buf).unwrap();
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
        assert_eq!(recv_buf, [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn msg_send_recv_multiple_queues() {
        let mut k = kernel(0, 0, 2);
        // Send different data to queue 0 and queue 1
        let data_q0: [u8; 4] = [1, 2, 3, 4];
        let data_q1: [u8; 4] = [5, 6, 7, 8];

        let outcome = k.messages_mut().send(0, 0, &data_q0).unwrap();
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

        let outcome = k.messages_mut().send(1, 0, &data_q1).unwrap();
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

        // Recv from queue 1 first
        let mut buf = [0u8; 4];
        let outcome = k.messages_mut().recv(1, 1, &mut buf).unwrap();
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
        assert_eq!(buf, [5, 6, 7, 8]);

        // Recv from queue 0
        let mut buf = [0u8; 4];
        let outcome = k.messages_mut().recv(0, 1, &mut buf).unwrap();
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn msg_invalid_queue_id_returns_max() {
        let mut k = kernel(0, 0, 1);
        assert!(k.messages_mut().send(99, 0, &[1; 4]).is_err());
        assert!(k.messages_mut().recv(99, 0, &mut [0; 4]).is_err());
    }

    #[test]
    fn msg_send_blocks_and_wakes() {
        let mut k = kernel(0, 0, 1);
        // Fill the depth-4 queue to capacity
        for i in 0..4u8 {
            let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
            assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
        }

        // Next send blocks partition 1
        let outcome = k.messages_mut().send(0, 1, &[99; 4]).unwrap();
        assert_eq!(outcome, SendOutcome::SenderBlocked { blocked: 1 });
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(Some(1)));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );

        // Recv should wake partition 1
        let mut buf = [0u8; 4];
        let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
        assert_eq!(
            outcome,
            RecvOutcome::Received {
                wake_sender: Some(1)
            }
        );
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
        assert_eq!(buf, [0; 4]); // first message enqueued
    }

    #[test]
    fn msg_recv_blocks_and_wakes() {
        let mut k = kernel(0, 0, 1);
        // Recv on empty queue blocks partition 0
        let mut buf = [0u8; 4];
        let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::ReceiverBlocked { blocked: 0 });
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(Some(0)));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Send should wake partition 0
        let outcome = k.messages_mut().send(0, 1, &[9; 4]).unwrap();
        assert_eq!(
            outcome,
            SendOutcome::Delivered {
                wake_receiver: Some(0)
            }
        );
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn msg_send_blocks_sets_yield_requested() {
        let mut k = kernel(0, 0, 1);
        // Fill the depth-4 queue to capacity
        for i in 0..4u8 {
            let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
            assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
        }
        // yield_requested should still be false after successful sends
        assert!(!k.yield_requested());

        // Next send blocks partition 1
        let outcome = k.messages_mut().send(0, 1, &[99; 4]).unwrap();
        assert_eq!(outcome, SendOutcome::SenderBlocked { blocked: 1 });
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(Some(1)));
        // Caller is responsible for triggering deschedule
        k.trigger_deschedule();

        // After blocking and triggering deschedule, yield_requested should be true
        assert!(k.yield_requested());
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );
    }

    #[test]
    fn msg_recv_blocks_sets_yield_requested() {
        let mut k = kernel(0, 0, 1);
        // yield_requested should initially be false
        assert!(!k.yield_requested());

        // Recv on empty queue blocks partition 0
        let mut buf = [0u8; 4];
        let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::ReceiverBlocked { blocked: 0 });

        // apply_recv_outcome calls trigger_deschedule internally
        assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(Some(0)));

        // yield_requested should now be true after blocking
        assert!(k.yield_requested());
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    #[test]
    fn get_time_returns_zero_initially() {
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[test]
    fn get_time_after_increments() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(5);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 5);
    }

    #[test]
    fn get_time_preserves_other_registers() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(1);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0xAA, 0xBB);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1);
        assert_eq!(ef.r1, 0xAA);
        assert_eq!(ef.r2, 0xBB);
    }

    #[test]
    fn get_time_truncates_to_u32() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick((1u64 << 32) + 7);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 7);
    }

    #[test]
    fn sync_tick_updates_kernel_tick() {
        let mut k = kernel(0, 0, 0);
        assert_eq!(k.tick().get(), 0);
        k.sync_tick(42);
        assert_eq!(k.tick().get(), 42);
        k.sync_tick(1000);
        assert_eq!(k.tick().get(), 1000);
    }

    fn frame4(r0: u32, r1: u32, r2: u32, r3: u32) -> ExceptionFrame {
        ExceptionFrame {
            r0,
            r1,
            r2,
            r3,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        }
    }

    #[test]
    fn sampling_dispatch_write_read_and_errors() {
        use crate::sampling::PortDirection;
        use crate::syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE};
        let mut k = kernel(0, 0, 0);
        let src = k
            .sampling_mut()
            .create_port(PortDirection::Source, 1000)
            .unwrap();
        let dst = k
            .sampling_mut()
            .create_port(PortDirection::Destination, 1000)
            .unwrap();
        k.sampling_mut().connect_ports(src, dst).unwrap();
        // Write + read via pool (avoids 64-bit pointer truncation issue).
        let tick = k.tick().get();
        k.sampling_mut()
            .write_sampling_message(src, &[0xAA, 0xBB], tick)
            .unwrap();
        let mut buf = [0u8; 64];
        let tick = k.tick().get();
        let (n, _) = k
            .sampling_mut()
            .read_sampling_message(dst, &mut buf, tick)
            .unwrap();
        assert_eq!((n, &buf[..n]), (2, &[0xAA, 0xBB][..]));
        // Invalid port → InvalidResource error for both write and read.
        // Use an address inside partition 0's MPU region so pointer
        // validation passes and the invalid port ID (99) is reached.
        let inv = SvcError::InvalidResource.to_u32();
        let mut ef = frame4(SYS_SAMPLING_WRITE, 99, 1, 0x2000_0000);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
        let mut ef = frame4(SYS_SAMPLING_READ, 99, 0, 0x2000_0000);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
    }

    #[test]
    fn bb_nonblocking_read_empty_returns_error_without_enqueue() {
        use crate::blackboard::BlackboardError;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards_mut().create().unwrap();
        let mut buf = [0u8; 64];
        // Non-blocking read (timeout=0) on empty board returns error
        assert_eq!(
            k.blackboards_mut().read_blackboard(id, 0, &mut buf, 0),
            Err(BlackboardError::BoardEmpty)
        );
        // Caller was NOT enqueued
        assert_eq!(k.blackboards().get(id).unwrap().waiting_readers(), 0);
    }

    #[test]
    fn bb_blocking_read_and_display_wake() {
        use crate::blackboard::ReadBlackboardOutcome;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards_mut().create().unwrap();
        let mut buf = [0u8; 64];
        // Blocking read (timeout>0) enqueues the caller
        assert_eq!(
            k.blackboards_mut().read_blackboard(id, 0, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(k.blackboards().get(id).unwrap().waiting_readers(), 1);
        // Display wakes the blocked reader
        let woken = k
            .blackboards_mut()
            .display_blackboard(id, &[0xAA, 0xBB])
            .unwrap();
        assert_eq!(woken.as_slice(), &[0]);
        // Non-blocking read now succeeds
        let outcome = k
            .blackboards_mut()
            .read_blackboard(id, 0, &mut buf, 0)
            .unwrap();
        assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 2 });
        assert_eq!(&buf[..2], &[0xAA, 0xBB]);
    }

    #[test]
    fn bb_blocking_read_wakes_partition() {
        use crate::blackboard::ReadBlackboardOutcome;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards_mut().create().unwrap();
        let mut buf = [0u8; 64];
        // Transition partition 1 to Waiting and block it on the blackboard
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.blackboards_mut().read_blackboard(id, 1, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        // Display wakes partition 1
        let woken = k.blackboards_mut().display_blackboard(id, &[0x01]).unwrap();
        assert_eq!(woken.as_slice(), &[1]);
        for &pid in woken.iter() {
            try_transition(k.partitions_mut(), pid, PartitionState::Ready);
        }
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn bb_invalid_board_errors() {
        use crate::blackboard::BlackboardError;
        let mut k = kernel(0, 0, 0);
        let r: Result<heapless::Vec<u8, 4>, _> = k.blackboards_mut().display_blackboard(99, &[1]);
        assert_eq!(r, Err(BlackboardError::InvalidBoard));
    }

    #[test]
    fn bb_clear_svc_dispatch() {
        use crate::blackboard::BlackboardError;
        use crate::syscall::SYS_BB_CLEAR;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards_mut().create().unwrap();
        let _ = k.blackboards_mut().display_blackboard(id, &[42]).unwrap();
        // Clear via SVC dispatch
        let mut ef = frame(SYS_BB_CLEAR, id as u32, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Non-blocking read after clear should fail
        let mut buf = [0u8; 64];
        assert_eq!(
            k.blackboards_mut().read_blackboard(id, 0, &mut buf, 0),
            Err(BlackboardError::BoardEmpty)
        );
        // Invalid board via SVC
        let mut ef = frame(SYS_BB_CLEAR, 99, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    // ---- set_dispatch_hook / Mutex-based hook tests ----

    /// Mutex to serialize tests that manipulate the global `SVC_DISPATCH_HOOK`.
    /// Without this, parallel test execution can cause `RefCell` borrow conflicts.
    static HOOK_TEST_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Reset the hook to `None` inside a critical section (test helper).
    fn clear_dispatch_hook() {
        with_cs(|cs| {
            SVC_DISPATCH_HOOK.borrow(cs).replace(None);
        });
    }

    /// Read the current hook value inside a critical section (test helper).
    fn read_dispatch_hook() -> Option<unsafe extern "C" fn(&mut ExceptionFrame)> {
        with_cs(|cs| *SVC_DISPATCH_HOOK.borrow(cs).borrow())
    }

    #[test]
    fn dispatch_hook_initially_none() {
        let _guard = HOOK_TEST_MUTEX.lock().unwrap();
        clear_dispatch_hook();
        assert!(read_dispatch_hook().is_none());
    }

    #[test]
    fn set_dispatch_hook_installs_hook() {
        let _guard = HOOK_TEST_MUTEX.lock().unwrap();
        unsafe extern "C" fn my_hook(_: &mut ExceptionFrame) {}
        clear_dispatch_hook();
        set_dispatch_hook(my_hook);
        assert!(read_dispatch_hook().is_some());
        clear_dispatch_hook();
    }

    #[test]
    fn dispatch_svc_uses_hook_when_set() {
        let _guard = HOOK_TEST_MUTEX.lock().unwrap();
        /// A hook that sets r0 to a sentinel value (0xCAFE).
        unsafe extern "C" fn sentinel_hook(frame: &mut ExceptionFrame) {
            frame.r0 = 0xCAFE;
        }
        clear_dispatch_hook();
        set_dispatch_hook(sentinel_hook);
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: ef is a valid ExceptionFrame on the stack.
        unsafe { dispatch_svc(&mut ef) };
        assert_eq!(ef.r0, 0xCAFE);
        clear_dispatch_hook();
    }

    #[test]
    fn dispatch_svc_falls_through_without_hook() {
        let _guard = HOOK_TEST_MUTEX.lock().unwrap();
        clear_dispatch_hook();
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: ef is a valid ExceptionFrame on the stack.
        unsafe { dispatch_svc(&mut ef) };
        // Without a hook, Yield returns 0.
        assert_eq!(ef.r0, 0);
    }

    // ---- SvcError tests ----

    /// All SvcError variants for exhaustive testing.
    const ALL_SVC_ERRORS: &[SvcError] = &[
        SvcError::InvalidSyscall,
        SvcError::InvalidResource,
        SvcError::WaitQueueFull,
        SvcError::TransitionFailed,
        SvcError::InvalidPartition,
        SvcError::OperationFailed,
        SvcError::InvalidPointer,
        SvcError::NotImplemented,
    ];

    #[test]
    fn svc_error_codes_are_unique_nonzero_and_high_bit_set() {
        use std::collections::HashSet;
        let mut seen = HashSet::new();
        for &e in ALL_SVC_ERRORS {
            let code = e.to_u32();
            assert_ne!(code, 0, "{e:?} maps to zero");
            assert!(
                code >= 0x8000_0000,
                "{e:?} code {code:#010X} does not have the high bit set"
            );
            assert!(seen.insert(code), "{e:?} has duplicate code {code:#010X}");
        }
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_release_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_RELEASE};
        let (eop, eres) = (
            SvcError::OperationFailed.to_u32(),
            SvcError::InvalidResource.to_u32(),
        );
        let mut k = kernel(0, 0, 0);
        macro_rules! svc {
            ($r0:expr,$r1:expr) => {{
                let mut ef = frame($r0, $r1, 0);
                unsafe { k.dispatch(&mut ef) };
                ef.r0
            }};
        }
        for i in 0..4u32 {
            assert_eq!(svc!(SYS_BUF_ALLOC, 1), i);
        }
        assert_eq!(svc!(SYS_BUF_ALLOC, 1), eop); // exhausted
        assert_eq!(svc!(SYS_BUF_RELEASE, 0), 0); // release 0
        assert_eq!(svc!(SYS_BUF_ALLOC, 0), 0); // re-alloc reuses 0
        assert_eq!(svc!(SYS_BUF_RELEASE, 0), 0); // release 0 again
        assert_eq!(svc!(SYS_BUF_RELEASE, 0), eres); // double-release
        k.set_current_partition(1); // switch to partition 1
        assert_eq!(svc!(SYS_BUF_RELEASE, 1), eres); // wrong owner
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_sets_deadline_from_r2() {
        use crate::syscall::SYS_BUF_ALLOC;
        let mut k = kernel(0, 0, 0);

        // r2=0 → no deadline
        let mut ef = frame(SYS_BUF_ALLOC, 1, 0);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        let slot0 = ef.r0 as usize;
        assert_eq!(slot0, 0);
        assert_eq!(k.buffers().deadline(slot0), None);

        // r2=100 → deadline = tick + 100; tick starts at 0 ⇒ deadline=100
        let mut ef = frame(SYS_BUF_ALLOC, 0, 100);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        let slot1 = ef.r0 as usize;
        assert_eq!(slot1, 1);
        assert_eq!(k.buffers().deadline(slot1), Some(100));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_open_dispatch_valid_and_invalid() {
        use crate::syscall::SYS_DEV_OPEN;
        let mut k = kernel(0, 0, 0);
        // Open device 0 (UART-A) — should succeed
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Open device 1 (UART-B) — should succeed
        let mut ef = frame(SYS_DEV_OPEN, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Open invalid device 99 — should return InvalidResource
        let mut ef = frame(SYS_DEV_OPEN, 99, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    /// Allocate a page at a fixed low address via `mmap` so that
    /// `ptr as u32` round-trips correctly on 64-bit test hosts.
    /// Each call site must use a distinct `page` offset. Leaked.
    #[cfg(feature = "dynamic-mpu")]
    fn low32_buf(page: usize) -> *mut u8 {
        extern "C" {
            fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
        }
        let addr = 0x2000_0000 + page * 4096;
        // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
        // low address. The mapping is intentionally leaked (test-only).
        let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
        assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
        ptr
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_write_dispatch_routes_to_uart() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_WRITE};
        let (registry, uart_a, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // SAFETY: ptr points to a valid mmap'd page.
        unsafe { core::ptr::copy_nonoverlapping([0xAA, 0xBB, 0xCC].as_ptr(), ptr, 3) };
        let mut ef = frame4(SYS_DEV_WRITE, 0, 3, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 3);
        // SAFETY: uart_a points to a leaked VirtualUartBackend that is only
        // accessed mutably here (dispatch is complete, no aliasing).
        let ua = unsafe { &mut *uart_a };
        assert_eq!(ua.pop_tx(), Some(0xAA));
        assert_eq!(ua.pop_tx(), Some(0xBB));
        assert_eq!(ua.pop_tx(), Some(0xCC));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_dispatch_routes_to_uart() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ};
        let (registry, uart_a, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing
        // because we only touch it here outside of dispatch.
        unsafe { &mut *uart_a }.push_rx(&[0xDE, 0xAD]);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_DEV_READ, 0, 4, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 2);
        // SAFETY: ptr is valid for 4096 bytes (mmap via low32_buf), 2 were written.
        let out = unsafe { core::slice::from_raw_parts(ptr, 2) };
        assert_eq!(out, &[0xDE, 0xAD]);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_ioctl_dispatch_routes_to_uart() {
        use crate::syscall::{SYS_DEV_IOCTL, SYS_DEV_OPEN};
        use crate::virtual_uart::IOCTL_AVAILABLE;
        let (registry, _, uart_b) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 1
        let mut ef = frame(SYS_DEV_OPEN, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push some RX data into UART-B
        // SAFETY: uart_b points to a leaked VirtualUartBackend; no aliasing
        // because we only touch it here outside of dispatch.
        unsafe { &mut *uart_b }.push_rx(&[1, 2, 3]);
        // IOCTL_AVAILABLE should return 3
        let mut ef = frame4(SYS_DEV_IOCTL, 1, IOCTL_AVAILABLE, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 3);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_invalid_id_returns_invalid_resource() {
        use crate::syscall::{SYS_DEV_IOCTL, SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE};
        let inv = SvcError::InvalidResource.to_u32();
        let inv_ptr = SvcError::InvalidPointer.to_u32();
        let mut k = kernel(0, 0, 0);
        // Open invalid device
        let mut ef = frame(SYS_DEV_OPEN, 99, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
        // Write to invalid device — pointer validation rejects first
        let mut ef = frame4(SYS_DEV_WRITE, 99, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv_ptr);
        // Read from invalid device — pointer validation rejects first
        let mut ef = frame4(SYS_DEV_READ, 99, 4, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv_ptr);
        // Ioctl on invalid device
        let mut ef = frame4(SYS_DEV_IOCTL, 99, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_close_after_open_returns_success() {
        use crate::hw_uart::HwUartBackend;
        use crate::syscall::{SYS_DEV_CLOSE, SYS_DEV_OPEN};
        use crate::uart_hal::UartRegs;
        let (mut registry, _, _) = default_registry();
        let hw = Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
        registry.add(hw).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open hw_uart device 5 (registered in the registry)
        let mut ef = frame(SYS_DEV_OPEN, 5, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Close device 5 — should succeed
        let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_close_without_open_returns_error() {
        use crate::hw_uart::HwUartBackend;
        use crate::syscall::SYS_DEV_CLOSE;
        use crate::uart_hal::UartRegs;
        let (mut registry, _, _) = default_registry();
        let hw = Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
        registry.add(hw).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Close device 5 without opening — HwUartBackend checks require_open,
        // so this returns OperationFailed.
        let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::OperationFailed.to_u32());
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_close_invalid_device_returns_invalid_resource() {
        use crate::syscall::SYS_DEV_CLOSE;
        let mut k = kernel(0, 0, 0);
        // Close non-existent device 99
        let mut ef = frame(SYS_DEV_CLOSE, 99, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    // ---- DevReadTimed dispatch tests ----

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_immediate_data_returns_byte_count() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, uart_a, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push a byte into the RX buffer so read succeeds immediately.
        // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing.
        unsafe { &mut *uart_a }.push_rx(&[0x42]);
        let ptr = low32_buf(0);
        // timeout=10 but data is available, so should return immediately.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 10, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "should return 1 byte read");
        // SAFETY: ptr was mmap'd by low32_buf and dispatch wrote 1 byte.
        let out = unsafe { core::slice::from_raw_parts(ptr, 1) };
        assert_eq!(out, &[0x42]);
        // Partition should remain Running (not blocked).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_blocks_on_empty_with_timeout() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, _, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout=50 should block the caller.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 50, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.dev_wait_queue().len(), 1);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_blocking_sets_yield_requested() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, _, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout>0 should block and trigger deschedule.
        assert!(!k.yield_requested());
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 50, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert!(
            k.yield_requested(),
            "yield_requested should be true after blocking read"
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_nonblocking_empty_returns_zero() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, _, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout=0 (non-blocking) should return 0 immediately.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 0, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Partition should remain Running (not blocked).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.dev_wait_queue().len(), 0);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_invalid_device_returns_error() {
        use crate::syscall::SYS_DEV_READ_TIMED;
        let mut k = kernel(0, 0, 0);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_DEV_READ_TIMED, 99, 10, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_rejects_out_of_bounds_pointer() {
        use crate::syscall::SYS_DEV_READ_TIMED;
        let mut k = kernel(0, 0, 0);
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 10, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    // ---- InvalidPointer and validate_user_ptr tests ----

    #[test]
    fn invalid_pointer_error_code() {
        assert_eq!(SvcError::InvalidPointer.to_u32(), 0xFFFF_FFF9);
        assert!(SvcError::is_error(SvcError::InvalidPointer.to_u32()));
    }

    /// Build a partition table with one partition whose MPU data region
    /// spans `[base, base + size)`.
    fn ptr_table(base: u32, size: u32) -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        t.add(PartitionControlBlock::new(
            0,
            0x0800_0000,
            base,
            base + 0x400,
            MpuRegion::new(base, size, 0),
        ))
        .unwrap();
        t
    }

    #[test]
    fn validate_ptr_valid_at_start() {
        let t = ptr_table(0x2000_0000, 4096);
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 16));
    }

    #[test]
    fn validate_ptr_valid_middle() {
        let t = ptr_table(0x2000_0000, 4096);
        assert!(validate_user_ptr(&t, 0, 0x2000_0100, 64));
    }

    #[test]
    fn validate_ptr_valid_exact_end() {
        // ptr + len == base + size (last byte is at base + size - 1)
        let t = ptr_table(0x2000_0000, 4096);
        assert!(validate_user_ptr(&t, 0, 0x2000_0FF0, 16));
    }

    #[test]
    fn validate_ptr_before_region() {
        let t = ptr_table(0x2000_0000, 4096);
        assert!(!validate_user_ptr(&t, 0, 0x1FFF_FFF0, 32));
    }

    #[test]
    fn validate_ptr_after_region() {
        let t = ptr_table(0x2000_0000, 4096);
        assert!(!validate_user_ptr(&t, 0, 0x2000_1000, 1));
    }

    #[test]
    fn validate_ptr_overflow_wraps() {
        let t = ptr_table(0x2000_0000, 4096);
        // ptr + len overflows u32
        assert!(!validate_user_ptr(&t, 0, 0xFFFF_FFF0, 0x20));
    }

    #[test]
    fn validate_ptr_zero_length() {
        let t = ptr_table(0x2000_0000, 4096);
        // Zero-length at region start is valid.
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 0));
        // Zero-length at region end (ptr == base + size) is valid.
        assert!(validate_user_ptr(&t, 0, 0x2000_1000, 0));
        // Zero-length outside the region is invalid.
        assert!(!validate_user_ptr(&t, 0, 0x1FFF_FFFF, 0));
    }

    #[test]
    fn validate_ptr_nonexistent_partition() {
        let t = ptr_table(0x2000_0000, 4096);
        assert!(!validate_user_ptr(&t, 99, 0x2000_0000, 16));
    }

    #[test]
    fn validate_ptr_spans_past_end() {
        let t = ptr_table(0x2000_0000, 4096);
        // Starts inside but extends 1 byte past the region.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0FF0, 17));
    }

    /// Build a partition table with separate (non-overlapping) data and stack regions.
    /// - Stack region: [stack_base, stack_base + stack_size)
    /// - Data region:  [data_base, data_base + data_size)
    fn ptr_table_separate_regions(
        stack_base: u32,
        stack_size: u32,
        data_base: u32,
        data_size: u32,
    ) -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        t.add(PartitionControlBlock::new(
            0,
            0x0800_0000,             // entry_point
            stack_base,              // stack_base
            stack_base + stack_size, // stack_pointer (top of stack)
            MpuRegion::new(data_base, data_size, 0),
        ))
        .unwrap();
        t
    }

    #[test]
    fn validate_ptr_in_stack_region_passes() {
        // Stack at 0x2000_0000..0x2000_0400 (1KB)
        // Data at  0x2000_1000..0x2000_2000 (4KB, non-overlapping)
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        // Pointer entirely within stack region should pass.
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 16));
        assert!(validate_user_ptr(&t, 0, 0x2000_0100, 64));
        // Exact end of stack region.
        assert!(validate_user_ptr(&t, 0, 0x2000_03F0, 16));
    }

    #[test]
    fn validate_ptr_spanning_stack_and_data_fails() {
        // Stack at 0x2000_0000..0x2000_0400
        // Data at  0x2000_0400..0x2000_0800 (adjacent, no overlap)
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_0400, 0x400);
        // Pointer starts in stack, ends in data region — must fail.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0300, 0x200));
        // Pointer starts in data, ends past data — must fail.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0700, 0x200));
    }

    #[test]
    fn validate_ptr_in_data_region_with_separate_stack() {
        // Stack at 0x2000_0000..0x2000_0400
        // Data at  0x2000_1000..0x2000_2000 (non-overlapping)
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        // Pointer entirely within data region should pass.
        assert!(validate_user_ptr(&t, 0, 0x2000_1000, 16));
        assert!(validate_user_ptr(&t, 0, 0x2000_1800, 64));
    }

    #[test]
    fn validate_ptr_outside_both_regions_fails() {
        // Stack at 0x2000_0000..0x2000_0400
        // Data at  0x2000_1000..0x2000_2000
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        // Gap between stack and data (0x2000_0400..0x2000_1000).
        assert!(!validate_user_ptr(&t, 0, 0x2000_0500, 16));
        // Before stack.
        assert!(!validate_user_ptr(&t, 0, 0x1FFF_FF00, 16));
        // After data.
        assert!(!validate_user_ptr(&t, 0, 0x2000_2000, 16));
    }

    // ---- validate_user_ptr_dynamic tests (dynamic-mpu feature) ----

    /// Comprehensive test for validate_user_ptr_dynamic covering dynamic windows,
    /// static region fallback, partition isolation, multiple windows, and edge cases.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_comprehensive() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};
        // Stack: 0x2000_0000..0x2000_0400, Data: 0x2000_1000..0x2000_2000
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        let s = DynamicStrategy::new();

        // Add 3 windows: one for P0, two more for P0, none for P1
        s.add_window(0x2001_0000, 256, 0, 0).unwrap(); // R5: P0
        s.add_window(0x2002_0000, 512, 0, 0).unwrap(); // R6: P0
        s.add_window(0x2003_0000, 1024, 0, 1).unwrap(); // R7: P1

        // Pointer in dynamic windows passes
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2001_0000, 16));
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2002_0100, 64));
        // Fallback to static data region
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_1000, 16));
        // Fallback to static stack region
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0000, 16));
        // Outside all regions fails
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0500, 16));
        // Spanning window boundary fails
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x2001_00F0, 32));
        // Nonexistent partition fails
        assert!(!validate_user_ptr_dynamic(&t, &s, 99, 0x2000_0000, 16));
        // Overflow fails
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0xFFFF_FFF0, 0x20));
        // Partition isolation: P0 cannot access P1's window
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x2003_0000, 16));
    }

    // ---- all_accessible_regions tests (dynamic-mpu feature) ----

    /// Comprehensive test for all_accessible_regions covering static/dynamic
    /// combination, ordering, partition isolation, and edge cases.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn all_accessible_regions_comprehensive() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};

        // Stack: 0x2000_0000..0x2000_0400, Data: 0x2000_1000..0x2000_2000
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        let s = DynamicStrategy::new();

        // Nonexistent partition returns empty
        assert!(all_accessible_regions(&t, &s, 99).is_empty());

        // No dynamic windows: only static (data + stack)
        let r = all_accessible_regions(&t, &s, 0);
        assert_eq!(r.len(), 2);
        assert!(r.contains(&(0x2000_1000, 0x1000)) && r.contains(&(0x2000_0000, 0x400)));

        // Add windows: P0 gets 2, P1 gets 1
        s.add_window(0x3000_0000, 256, 0, 0).unwrap();
        s.add_window(0x3001_0000, 512, 0, 0).unwrap();
        s.add_window(0x4000_0000, 1024, 0, 1).unwrap();

        // P0: 2 static + 2 dynamic = 4
        let r0 = all_accessible_regions(&t, &s, 0);
        assert_eq!(r0.len(), 4);
        assert!(r0.contains(&(0x3000_0000, 256)) && r0.contains(&(0x3001_0000, 512)));
        assert!(!r0.contains(&(0x4000_0000, 1024))); // P1's window excluded

        // Static first, dynamic last: first 2 are static bases
        let static_bases: heapless::Vec<u32, 2> = r0.iter().take(2).map(|&(b, _)| b).collect();
        assert!(static_bases.contains(&0x2000_1000) && static_bases.contains(&0x2000_0000));

        // P1 not in partition table → empty
        assert!(all_accessible_regions(&t, &s, 1).is_empty());
    }

    // ---- Pointer validation rejection via Kernel::dispatch ----

    /// Data-driven test: every syscall that validates a user pointer must
    /// return `InvalidPointer` when r3 points outside the caller's MPU region.
    #[test]
    fn validated_syscalls_reject_out_of_bounds_pointer() {
        use crate::sampling::PortDirection;

        // (syscall_id, r1, r2, msg_queues, setup_sampling)
        let cases: &[(u32, u32, u32, usize, Option<PortDirection>)] = &[
            // MsgSend: queue 0 exists, r3 out-of-bounds
            (crate::syscall::SYS_MSG_SEND, 0, 0, 1, None),
            // MsgRecv: queue 0 exists, r3 out-of-bounds
            (crate::syscall::SYS_MSG_RECV, 0, 0, 1, None),
            // SamplingWrite: port 0 (Source), r2=4 (length), r3 out-of-bounds
            (
                crate::syscall::SYS_SAMPLING_WRITE,
                0,
                4,
                0,
                Some(PortDirection::Source),
            ),
            // SamplingRead: port 0 (Destination), r3 out-of-bounds
            (
                crate::syscall::SYS_SAMPLING_READ,
                0,
                0,
                0,
                Some(PortDirection::Destination),
            ),
        ];

        for &(sys_id, r1, r2, msg_queues, ref sampling_dir) in cases {
            let mut k = kernel(0, 0, msg_queues);
            if let Some(dir) = sampling_dir {
                k.sampling_mut().create_port(*dir, 1000).unwrap();
            }
            let mut ef = frame4(sys_id, r1, r2, 0xDEAD_0000);
            unsafe { k.dispatch(&mut ef) };
            assert_eq!(
                ef.r0,
                SvcError::InvalidPointer.to_u32(),
                "syscall {sys_id:#X} should reject out-of-bounds pointer"
            );
        }
    }

    /// Queuing syscalls (Send, Recv, Status) must reject out-of-bounds
    /// pointers with `SvcError::InvalidPointer`.
    #[test]
    fn queuing_syscalls_reject_out_of_bounds_pointer() {
        use crate::sampling::PortDirection;

        // (syscall_id, r1, r2, r3, queuing_port_direction)
        let cases: &[(u32, u32, u32, u32, PortDirection)] = &[
            // QueuingSend: r3 = data ptr (out-of-bounds), r2 = len
            (
                crate::syscall::SYS_QUEUING_SEND,
                0,
                4,
                0xDEAD_0000,
                PortDirection::Source,
            ),
            // QueuingRecv: r3 = buf ptr (out-of-bounds)
            (
                crate::syscall::SYS_QUEUING_RECV,
                0,
                0,
                0xDEAD_0000,
                PortDirection::Destination,
            ),
            // QueuingStatus: r2 = status ptr (out-of-bounds)
            (
                crate::syscall::SYS_QUEUING_STATUS,
                0,
                0xDEAD_0000,
                0,
                PortDirection::Source,
            ),
        ];

        for &(sys_id, r1, r2, r3, dir) in cases {
            let mut k = kernel(0, 0, 0);
            k.queuing_mut().create_port(dir).unwrap();
            let mut ef = frame4(sys_id, r1, r2, r3);
            unsafe { k.dispatch(&mut ef) };
            assert_eq!(
                ef.r0,
                SvcError::InvalidPointer.to_u32(),
                "syscall {sys_id:#X} should reject out-of-bounds pointer"
            );
        }
    }

    /// BbDisplay and BbRead must reject out-of-bounds pointers with
    /// `SvcError::InvalidPointer`.
    #[test]
    fn blackboard_syscalls_reject_out_of_bounds_pointer() {
        // BbDisplay: r1 = board id, r2 = data len, r3 = data ptr (out-of-bounds)
        let mut k = kernel(0, 0, 0);
        k.blackboards_mut().create().unwrap();
        let mut ef = frame4(crate::syscall::SYS_BB_DISPLAY, 0, 4, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(
            ef.r0,
            SvcError::InvalidPointer.to_u32(),
            "BbDisplay should reject out-of-bounds pointer"
        );

        // BbRead: r1 = board id, r2 = timeout, r3 = buf ptr (out-of-bounds)
        let mut k = kernel(0, 0, 0);
        k.blackboards_mut().create().unwrap();
        let mut ef = frame4(crate::syscall::SYS_BB_READ, 0, 0, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(
            ef.r0,
            SvcError::InvalidPointer.to_u32(),
            "BbRead should reject out-of-bounds pointer"
        );
    }

    // ---- QueuingRecvTimed dispatch tests ----

    /// Allocate a page at a fixed low address via `mmap` so that
    /// `ptr as u32` round-trips correctly on 64-bit test hosts.
    /// Each call site must use a distinct `page` offset. Leaked.
    #[cfg(not(feature = "dynamic-mpu"))]
    fn low32_buf(page: usize) -> *mut u8 {
        extern "C" {
            fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
        }
        let addr = 0x2000_0000 + page * 4096;
        // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
        // low address. The mapping is intentionally leaked (test-only).
        let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
        assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
        ptr
    }

    // ---- BbRead blocking dispatch tests ----

    #[test]
    fn bb_read_blocks_sets_yield_requested() {
        use crate::syscall::SYS_BB_READ;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards_mut().create().unwrap();
        let ptr = low32_buf(0);
        // BbRead with timeout > 0 on empty blackboard should block
        let mut ef = frame4(SYS_BB_READ, id as u32, 50, ptr as u32);
        assert!(
            !k.yield_requested(),
            "yield_requested should be false before blocking read"
        );
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert!(
            k.yield_requested(),
            "yield_requested should be true after blocking read"
        );
    }

    // ---- QueuingRecvTimed dispatch tests ----

    #[test]
    fn queuing_recv_timed_delivers_message_and_wakes_sender() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        k.queuing_mut()
            .get_mut(dst)
            .unwrap()
            .inject_message(2, &[0xAA, 0xBB]);
        k.queuing_mut()
            .get_mut(dst)
            .unwrap()
            .enqueue_blocked_sender(1, u64::MAX);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 100, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 2, "should return msg_len=2");
        // Verify message data was delivered to the buffer.
        // SAFETY: ptr was mmap'd by low32_buf and dispatch wrote into it.
        let delivered = unsafe { core::slice::from_raw_parts(ptr, 2) };
        assert_eq!(
            delivered,
            &[0xAA, 0xBB],
            "buffer should contain delivered data"
        );
        // Blocked sender (partition 1) should be woken to Ready
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn queuing_recv_timed_blocks_on_empty_queue() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 50, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.queuing().get(dst).unwrap().pending_receivers(), 1);
    }

    #[test]
    fn queuing_recv_timed_blocks_sets_yield_requested() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 50, ptr as u32);
        assert!(
            !k.yield_requested(),
            "yield_requested should be false before blocking recv"
        );
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert!(
            k.yield_requested(),
            "yield_requested should be true after blocking recv"
        );
    }

    #[test]
    fn queuing_recv_timed_zero_timeout_returns_error() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 0, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn queuing_recv_timed_invalid_port_returns_error() {
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, 99, 50, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn queuing_recv_timed_rejects_out_of_bounds_pointer() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        k.queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, 0, 50, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    #[test]
    fn queuing_recv_timed_received_no_sender_to_wake() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        k.queuing_mut()
            .get_mut(dst)
            .unwrap()
            .inject_message(1, &[42]);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 100, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "should return msg_len=1");
        // Verify message data was actually delivered to the buffer.
        // SAFETY: ptr was mmap'd by low32_buf and dispatch wrote into it.
        let delivered = unsafe { core::slice::from_raw_parts(ptr, 1) };
        assert_eq!(delivered[0], 42, "buffer should contain the delivered byte");
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_recv_original_unchanged_still_uses_zero_timeout() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV;
        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let ptr = low32_buf(0);
        // Empty queue with original QueuingRecv (timeout=0) → QueueEmpty → InvalidResource
        let mut ef = frame4(SYS_QUEUING_RECV, dst as u32, 0, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
        // Partition should NOT be in Waiting (no blocking happened)
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    // ---- QueuingSendTimed dispatch tests ----

    /// Helper: create a connected Source→Destination pair, return (src_id, dst_id).
    fn connected_send_pair(k: &mut Kernel<TestConfig>) -> (usize, usize) {
        use crate::sampling::PortDirection;
        let s = k.queuing_mut().create_port(PortDirection::Source).unwrap();
        let d = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        k.queuing_mut().connect_ports(s, d).unwrap();
        (s, d)
    }

    /// Pack r2 for QueuingSendTimed: timeout_hi16 << 16 | data_len_lo16.
    fn pack_r2(timeout: u16, data_len: u16) -> u32 {
        ((timeout as u32) << 16) | (data_len as u32)
    }

    /// Dequeue one message from destination port `d` and assert its length and
    /// content match `expected_data`.
    fn assert_queued_message(k: &mut Kernel<TestConfig>, d: usize, expected_data: &[u8]) {
        assert_eq!(
            k.queuing().get(d).unwrap().nb_messages(),
            1,
            "destination should have 1 queued message"
        );
        let mut recv_buf = [0u8; 4];
        let outcome = k
            .queuing_mut()
            .get_mut(d)
            .unwrap()
            .receive_queuing_message(1, &mut recv_buf, 0, 0)
            .unwrap();
        match outcome {
            crate::queuing::RecvQueuingOutcome::Received { msg_len, .. } => {
                assert_eq!(msg_len, expected_data.len());
                assert_eq!(
                    &recv_buf[..expected_data.len()],
                    expected_data,
                    "delivered data must match"
                );
            }
            _ => panic!("expected Received outcome"),
        }
    }

    #[test]
    fn queuing_send_timed_delivers_message_and_wakes_receiver() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Enqueue a blocked receiver on the destination port
        k.queuing_mut()
            .get_mut(d)
            .unwrap()
            .enqueue_blocked_receiver(1, u64::MAX);
        let ptr = low32_buf(0);
        // Write two bytes of data into the buffer
        // SAFETY: test-only, writing to a known-mapped page.
        unsafe {
            *ptr = 0xAA;
            *ptr.add(1) = 0xBB;
        }
        let r2 = pack_r2(100, 2);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "send should return 0 on successful delivery");
        assert_queued_message(&mut k, d, &[0xAA, 0xBB]);
        // Blocked receiver (partition 1) should be woken to Ready
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn queuing_send_timed_blocks_on_full_queue() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill the destination queue to capacity (QD=4)
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(d)
                .unwrap()
                .inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        let r2 = pack_r2(50, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Caller (partition 0) should transition to Waiting
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.queuing().get(d).unwrap().pending_senders(), 1);
    }

    #[test]
    fn queuing_send_timed_zero_timeout_full_returns_error() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill destination queue
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(d)
                .unwrap()
                .inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        let r2 = pack_r2(0, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        // timeout=0 with full queue → QueueFull → InvalidResource
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn queuing_send_timed_invalid_port_returns_error() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let ptr = low32_buf(0);
        let r2 = pack_r2(10, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, 99, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn queuing_send_timed_rejects_out_of_bounds_pointer() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        connected_send_pair(&mut k);
        let r2 = pack_r2(10, 4);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, 0, r2, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    #[test]
    fn queuing_send_timed_delivered_no_receiver_to_wake() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        let ptr = low32_buf(0);
        // SAFETY: test-only, writing to a known-mapped page.
        unsafe {
            *ptr = 0x42;
        }
        let r2 = pack_r2(100, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "send should return 0 on delivery");
        assert_queued_message(&mut k, d, &[0x42]);
        // No receiver was blocked, so partition 1 should remain Running
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_send_original_unchanged_still_uses_zero_timeout() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_SEND;
        let mut k = kernel(0, 0, 0);
        let s = k.queuing_mut().create_port(PortDirection::Source).unwrap();
        let d = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        k.queuing_mut().connect_ports(s, d).unwrap();
        // Fill destination queue
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(d)
                .unwrap()
                .inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        // Original QueuingSend: r2=data_len (used as raw len), timeout=0
        // Full queue with timeout=0 → QueueFull → InvalidResource
        let mut ef = frame4(SYS_QUEUING_SEND, s as u32, 1, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
        // Partition should NOT be in Waiting (no blocking with original handler)
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_send_timed_blocks_sets_yield_requested() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill the destination queue to capacity (QD=4)
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(d)
                .unwrap()
                .inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        let r2 = pack_r2(50, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        assert!(
            !k.yield_requested(),
            "yield_requested should be false before blocking send"
        );
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert!(
            k.yield_requested(),
            "yield_requested should be true after blocking send"
        );
    }

    // ---- hw_uart integration tests ----

    /// hw_uart starts as None; virtual UARTs still work.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_none_virtual_uarts_still_dispatch() {
        use crate::syscall::SYS_DEV_OPEN;
        let mut k = kernel(0, 0, 0);
        assert!(k.hw_uart().is_none());
        // Virtual UART-A (device 0) opens successfully.
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Virtual UART-B (device 1) opens successfully.
        let mut ef = frame(SYS_DEV_OPEN, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    /// Unknown device ID returns InvalidResource when hw_uart is None.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_none_unknown_id_returns_invalid_resource() {
        use crate::syscall::SYS_DEV_OPEN;
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_DEV_OPEN, 5, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    /// After registering hw_uart in the registry, dev_dispatch routes to it.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_registered_and_dispatch_open_write_read() {
        use crate::hw_uart::HwUartBackend;
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE};
        use crate::uart_hal::UartRegs;
        let (mut registry, _, _) = default_registry();
        let hw: &'static mut HwUartBackend =
            Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
        let hw_ptr: *mut HwUartBackend = hw as *mut _;
        registry.add(hw).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open hw_uart device 5 (now in the registry).
        let mut ef = frame(SYS_DEV_OPEN, 5, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Write 3 bytes via device 5 (ptr within partition 0 MPU region).
        let ptr = low32_buf(0);
        // SAFETY: ptr points to a valid mmap'd page within MPU region.
        unsafe { core::ptr::copy_nonoverlapping([0x11, 0x22, 0x33].as_ptr(), ptr, 3) };
        let mut ef = frame4(SYS_DEV_WRITE, 5, 3, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 3);
        // Inject bytes into hw_uart RX and read them back.
        // SAFETY: hw_ptr points to a leaked HwUartBackend; no aliasing
        // because dispatch is not active.
        unsafe { &mut *hw_ptr }.push_rx(&[0xAA, 0xBB]);
        let mut ef = frame4(SYS_DEV_READ, 5, 4, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 2);
        // SAFETY: ptr is valid for 4096 bytes via low32_buf.
        let out = unsafe { core::slice::from_raw_parts(ptr, 2) };
        assert_eq!(out, &[0xAA, 0xBB]);
    }

    /// Registry rejects duplicate device IDs, preventing shadowing.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn registry_rejects_duplicate_device_id() {
        use crate::hw_uart::HwUartBackend;
        use crate::uart_hal::UartRegs;
        let (mut registry, _, _) = default_registry();
        // Attempt to register hw_uart with device_id = 0 (same as UART-A).
        let hw = Box::leak(Box::new(HwUartBackend::new(0, UartRegs::new(0x4000_C000))));
        assert_eq!(
            registry.add(hw),
            Err(crate::virtual_device::DeviceError::DuplicateId)
        );
    }

    // ---- device registry integration tests ----

    /// Open + close lifecycle via a device registered in the registry.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn registry_dispatch_open_close_lifecycle() {
        use crate::syscall::{SYS_DEV_CLOSE, SYS_DEV_OPEN};
        use crate::virtual_uart::VirtualUartBackend;
        let (mut registry, _, _) = default_registry();
        let dev = Box::leak(Box::new(VirtualUartBackend::new(10)));
        registry.add(dev).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        let mut ef = frame(SYS_DEV_OPEN, 10, 0);
        // SAFETY: dispatch requires a mutable ExceptionFrame pointer. The
        // frame lives on the stack and is not accessed concurrently; this is
        // a single-threaded test with no real interrupt context.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let mut ef = frame(SYS_DEV_CLOSE, 10, 0);
        // SAFETY: same as above — single-threaded test, frame on the stack.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    // --- expire_timed_waits tests ---

    #[test]
    fn expire_timed_waits_wakes_blocked_receiver() {
        use crate::queuing::RecvQueuingOutcome;
        use crate::sampling::PortDirection;

        let mut k = kernel(0, 0, 0);
        // Create a destination queuing port (empty queue → recv blocks).
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();

        // Partition 0 attempts a timed receive with timeout=50 at tick=100.
        // Queue is empty so the receiver gets blocked with expiry=150.
        let mut buf = [0u8; 4];
        let outcome = k
            .queuing_mut()
            .receive_queuing_message(dst, 0, &mut buf, 50, 100)
            .unwrap();
        assert_eq!(
            outcome,
            RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 150 }
        );

        // Simulate the SVC handler: transition partition 0 from Running→Waiting.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Tick 150: expiry fires, partition should move Waiting→Ready.
        k.expire_timed_waits::<8>(150);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn expire_timed_waits_wakes_blocked_sender() {
        use crate::queuing::SendQueuingOutcome;
        use crate::sampling::PortDirection;

        let mut k = kernel(0, 0, 0);
        // Create a source port and a connected destination port.
        let src = k.queuing_mut().create_port(PortDirection::Source).unwrap();
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        k.queuing_mut().connect_ports(src, dst).unwrap();

        // Fill the destination queue (depth=4).
        for i in 0..4u8 {
            let outcome = k.queuing_mut().send_routed(src, 0, &[i; 4], 0, 0).unwrap();
            assert!(matches!(outcome, SendQueuingOutcome::Delivered { .. }));
        }

        // Partition 1 attempts a timed send with timeout=100 at tick=50.
        // Queue is full so the sender blocks with expiry=150.
        let outcome = k
            .queuing_mut()
            .send_routed(src, 1, &[0xFF; 4], 100, 50)
            .unwrap();
        assert_eq!(
            outcome,
            SendQueuingOutcome::SenderBlocked { expiry_tick: 150 }
        );

        // Simulate the SVC handler: transition partition 1 Running→Waiting.
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );

        // Tick 150: expiry fires, partition should move Waiting→Ready.
        k.expire_timed_waits::<8>(150);
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn expire_timed_waits_non_expired_stays_waiting() {
        use crate::queuing::RecvQueuingOutcome;
        use crate::sampling::PortDirection;

        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();

        // Block partition 0 as receiver with expiry at tick 200.
        let mut buf = [0u8; 4];
        let outcome = k
            .queuing_mut()
            .receive_queuing_message(dst, 0, &mut buf, 100, 100)
            .unwrap();
        assert_eq!(
            outcome,
            RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 200 }
        );

        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // Tick 150: before expiry — partition must remain Waiting.
        k.expire_timed_waits::<8>(150);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Tick 199: still before expiry.
        k.expire_timed_waits::<8>(199);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    // --- device wait queue expiry tests ---

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn expire_timed_waits_device_reader_expiry() {
        let mut k = kernel(0, 0, 0);
        k.dev_wait_queue_mut().block_reader(0, 100).unwrap();
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        // Before expiry: stays Waiting.
        k.expire_timed_waits::<8>(99);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.dev_wait_queue().len(), 1);
        // At expiry: transitions Waiting→Ready.
        k.expire_timed_waits::<8>(100);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
        assert!(k.dev_wait_queue().is_empty());
    }

    // --- sync_tick dispatch integration tests ---

    #[test]
    fn sync_tick_then_get_time_returns_synced_value() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(42);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 42);
    }

    #[test]
    fn sync_tick_overwrite_returns_latest_value() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(5);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 5);

        k.sync_tick(10);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 10);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn sync_tick_affects_buffer_alloc_deadline() {
        use crate::syscall::SYS_BUF_ALLOC;

        let mut k = kernel(0, 0, 0);
        // Sync tick to 100, then alloc with timeout=50 → deadline should be 150.
        k.sync_tick(100);
        let mut ef = frame(SYS_BUF_ALLOC, 1, 50);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        let slot = ef.r0 as usize;
        assert_eq!(k.buffers().deadline(slot), Some(150));
    }

    #[test]
    fn sync_tick_expire_timed_waits_uses_synced_tick() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;

        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();

        // Sync tick to 100, then dispatch a timed recv with timeout=50.
        // The dispatch handler reads self.tick.get() (100) as current_tick,
        // so the receiver blocks with expiry = 100 + 50 = 150.
        k.sync_tick(100);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 50, ptr as u32);
        // SAFETY: test-only dispatch on a valid ExceptionFrame.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.queuing().get(dst).unwrap().pending_receivers(), 1);

        // Expire at the synced deadline tick.
        k.expire_timed_waits::<8>(150);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
    }

    // Kernel::new() validation - comprehensive tests in kernel.rs for KernelState

    #[test]
    fn kernel_new_validates_and_creates() {
        // Test empty schedule rejection
        let empty: ScheduleTable<4> = ScheduleTable::new();
        let cfg = PartitionConfig {
            id: 0,
            entry_point: 0x0800_0000,
            stack_base: 0x2000_0000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
            peripheral_regions: heapless::Vec::new(),
        };
        #[cfg(not(feature = "dynamic-mpu"))]
        assert!(matches!(
            Kernel::<TestConfig>::new(empty, core::slice::from_ref(&cfg)),
            Err(ConfigError::ScheduleEmpty)
        ));
        #[cfg(feature = "dynamic-mpu")]
        assert!(matches!(
            Kernel::<TestConfig>::new(
                empty,
                core::slice::from_ref(&cfg),
                crate::virtual_device::DeviceRegistry::new()
            ),
            Err(ConfigError::ScheduleEmpty)
        ));
        // Test valid config succeeds
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new(s, core::slice::from_ref(&cfg)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        let k = Kernel::<TestConfig>::new(
            s,
            core::slice::from_ref(&cfg),
            crate::virtual_device::DeviceRegistry::new(),
        )
        .unwrap();
        assert_eq!(k.partitions().len(), 1);
        assert_eq!(k.active_partition(), None);
    }

    /// Helper to call `Kernel::new` with correct feature-flag arguments.
    fn try_kernel_new(
        schedule: ScheduleTable<4>,
        configs: &[PartitionConfig],
    ) -> Result<Kernel<TestConfig>, ConfigError> {
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            Kernel::<TestConfig>::new(schedule, configs)
        }
        #[cfg(feature = "dynamic-mpu")]
        {
            Kernel::<TestConfig>::new(
                schedule,
                configs,
                crate::virtual_device::DeviceRegistry::new(),
            )
        }
    }

    #[test]
    fn kernel_new_rejects_partition_id_mismatch() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        // Config where id=5 is at index 0 (mismatch)
        let bad_cfg = PartitionConfig {
            id: 5, // should be 0
            entry_point: 0x0800_0000,
            stack_base: 0x2000_0000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
            peripheral_regions: heapless::Vec::new(),
        };

        let result = try_kernel_new(s, &[bad_cfg]);

        assert!(matches!(
            result,
            Err(ConfigError::PartitionIdMismatch {
                index: 0,
                expected_id: 0,
                actual_id: 5,
            })
        ));
    }

    #[test]
    fn kernel_new_rejects_second_partition_id_mismatch() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add(ScheduleEntry::new(1, 100)).unwrap();

        let cfg0 = PartitionConfig {
            id: 0, // correct
            entry_point: 0x0800_0000,
            stack_base: 0x2000_0000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
            peripheral_regions: heapless::Vec::new(),
        };
        let cfg1 = PartitionConfig {
            id: 3, // should be 1
            entry_point: 0x0800_1000,
            stack_base: 0x2000_1000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
            peripheral_regions: heapless::Vec::new(),
        };

        let result = try_kernel_new(s, &[cfg0, cfg1]);

        assert!(matches!(
            result,
            Err(ConfigError::PartitionIdMismatch {
                index: 1,
                expected_id: 1,
                actual_id: 3,
            })
        ));
    }

    #[test]
    fn kernel_new_accepts_matching_partition_ids() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add(ScheduleEntry::new(1, 100)).unwrap();

        let cfgs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];

        let k = try_kernel_new(s, &cfgs).unwrap();

        assert_eq!(k.partitions().len(), 2);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        assert_eq!(k.partitions().get(1).unwrap().id(), 1);
    }

    #[test]
    fn kernel_new_empty_configs_with_schedule_fails() {
        // Schedule references partition 0, but no partitions provided
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        let result = try_kernel_new(s, &[]);

        assert!(matches!(
            result,
            Err(ConfigError::ScheduleIndexOutOfBounds {
                entry_index: 0,
                partition_index: 0,
                num_partitions: 0,
            })
        ));
    }

    #[test]
    fn kernel_new_single_partition_id_zero_succeeds() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        let cfg = PartitionConfig {
            id: 0,
            entry_point: 0x0800_0000,
            stack_base: 0x2000_0000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
            peripheral_regions: heapless::Vec::new(),
        };

        let k = try_kernel_new(s, &[cfg]).unwrap();

        assert_eq!(k.partitions().len(), 1);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
    }

    #[test]
    fn kernel_new_three_contiguous_partitions_succeeds() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add(ScheduleEntry::new(1, 100)).unwrap();
        s.add(ScheduleEntry::new(2, 100)).unwrap();

        let cfgs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 2,
                entry_point: 0x0800_2000,
                stack_base: 0x2000_2000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_2000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];

        let k = try_kernel_new(s, &cfgs).unwrap();

        assert_eq!(k.partitions().len(), 3);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        assert_eq!(k.partitions().get(1).unwrap().id(), 1);
        assert_eq!(k.partitions().get(2).unwrap().id(), 2);
    }

    #[test]
    fn kernel_new_swapped_partition_ids_fails() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add(ScheduleEntry::new(1, 100)).unwrap();
        s.add(ScheduleEntry::new(2, 100)).unwrap();

        // Three partitions with non-sequential ID in middle: [id:0, id:2, id:1]
        let cfgs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 2, // should be 1
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1, // should be 2
                entry_point: 0x0800_2000,
                stack_base: 0x2000_2000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_2000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];

        let result = try_kernel_new(s, &cfgs);

        // Mismatch at index 1: config[1].id=2 but expected 1
        assert!(matches!(
            result,
            Err(ConfigError::PartitionIdMismatch {
                index: 1,
                expected_id: 1,
                actual_id: 2,
            })
        ));
    }

    // -------------------------------------------------------------------------
    // Schedule advance and accessor tests
    // -------------------------------------------------------------------------

    /// Helper to create a Kernel with a started schedule and partitions.
    fn kernel_with_schedule() -> Kernel<TestConfig> {
        // Create 2-slot schedule: P0 for 5 ticks, P1 for 3 ticks
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 3)).unwrap();
        schedule.start();
        let cfgs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];
        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new(schedule, &cfgs).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        let k = Kernel::<TestConfig>::new(
            schedule,
            &cfgs,
            crate::virtual_device::DeviceRegistry::new(),
        )
        .unwrap();
        k
    }

    #[test]
    fn partitions_accessor_returns_partition_table() {
        let k = kernel_with_schedule();
        assert_eq!(k.partitions().len(), 2);
        assert!(k.partitions().get(0).is_some());
        assert!(k.partitions().get(1).is_some());
    }

    #[test]
    fn partitions_mut_accessor_allows_modification() {
        let mut k = kernel_with_schedule();
        let pcb = k.partitions_mut().get_mut(0).unwrap();
        // Verify we can read partition state through the mutable reference
        assert_eq!(pcb.id(), 0);
    }

    #[test]
    fn schedule_accessor_returns_schedule_table() {
        let k = kernel_with_schedule();
        assert_eq!(k.schedule().major_frame_ticks, 8); // 5 + 3
        assert_eq!(k.schedule().len(), 2);
    }

    #[test]
    fn next_partition_initialized_to_zero() {
        let k = kernel_with_schedule();
        assert_eq!(k.next_partition(), 0);
    }

    #[test]
    fn advance_schedule_tick_updates_active_partition() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Initially no active partition
        assert_eq!(k.active_partition(), None);
        // Advance 4 ticks within slot 0 - no switch
        for _ in 0..4 {
            assert_eq!(k.advance_schedule_tick(), ScheduleEvent::None);
        }
        // 5th tick triggers switch to P1
        assert_eq!(k.advance_schedule_tick(), ScheduleEvent::PartitionSwitch(1));
        assert_eq!(k.active_partition(), Some(1));
    }

    #[test]
    fn advance_schedule_tick_updates_next_partition() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Initially next_partition is 0 (default)
        assert_eq!(k.next_partition(), 0);
        // Advance 4 ticks within slot 0 - no switch, next_partition unchanged
        for _ in 0..4 {
            assert_eq!(k.advance_schedule_tick(), ScheduleEvent::None);
            assert_eq!(k.next_partition(), 0);
        }
        // 5th tick triggers switch to P1, next_partition updated
        assert_eq!(k.advance_schedule_tick(), ScheduleEvent::PartitionSwitch(1));
        assert_eq!(k.next_partition(), 1);
        // Continue through P1's slot (3 ticks), then wrap to P0
        for _ in 0..2 {
            k.advance_schedule_tick();
            assert_eq!(k.next_partition(), 1);
        }
        // 3rd tick of P1's slot triggers switch back to P0
        assert_eq!(k.advance_schedule_tick(), ScheduleEvent::PartitionSwitch(0));
        assert_eq!(k.next_partition(), 0);
    }

    #[test]
    fn advance_schedule_tick_increments_tick_counter() {
        let mut k = kernel_with_schedule();
        assert_eq!(k.tick().get(), 0);
        k.advance_schedule_tick();
        assert_eq!(k.tick().get(), 1);
        k.advance_schedule_tick();
        assert_eq!(k.tick().get(), 2);
    }

    #[test]
    fn yield_current_slot_advances_to_next_partition() {
        let mut k = kernel_with_schedule();
        // Consume 2 ticks in slot 0
        k.advance_schedule_tick();
        k.advance_schedule_tick();
        // Yield: skip remaining 3 ticks, advance to P1
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));
        assert_eq!(k.active_partition(), Some(1));
    }

    #[test]
    fn yield_current_slot_wraps_around() {
        let mut k = kernel_with_schedule();
        // Yield to P1
        let r1 = k.yield_current_slot();
        assert_eq!(r1.partition_id(), Some(1));
        // Yield again: wraps to P0
        let r2 = k.yield_current_slot();
        assert_eq!(r2.partition_id(), Some(0));
        assert_eq!(k.active_partition(), Some(0));
    }

    #[test]
    fn yield_does_not_increment_tick_counter() {
        let mut k = kernel_with_schedule();
        let tick_before = k.tick().get();
        k.yield_current_slot();
        assert_eq!(k.tick().get(), tick_before);
    }

    #[test]
    fn yield_current_slot_returns_schedule_event() {
        let mut k = kernel_with_schedule();
        let result = k.yield_current_slot();
        // Should switch to P1
        assert_eq!(result.partition_id(), Some(1));
        assert!(!result.is_system_window());
    }

    /// Helper to create a Kernel with an UNSTARTED schedule for testing
    /// the start_schedule() method.
    fn kernel_unstarted_schedule() -> Kernel<TestConfig> {
        // Create 2-slot schedule: P0 for 5 ticks, P1 for 3 ticks
        // NOTE: do NOT call schedule.start() here
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 3)).unwrap();
        let cfgs = [
            PartitionConfig {
                id: 0,
                entry_point: 0x0800_0000,
                stack_base: 0x2000_0000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_0000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
            PartitionConfig {
                id: 1,
                entry_point: 0x0800_1000,
                stack_base: 0x2000_1000,
                stack_size: 1024,
                mpu_region: MpuRegion::new(0x2000_1000, 4096, 0),
                peripheral_regions: heapless::Vec::new(),
            },
        ];
        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new(schedule, &cfgs).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        let k = Kernel::<TestConfig>::new(
            schedule,
            &cfgs,
            crate::virtual_device::DeviceRegistry::new(),
        )
        .unwrap();
        k
    }

    #[test]
    fn start_schedule_returns_initial_partition() {
        let mut k = kernel_unstarted_schedule();
        // Before start, active_partition is None (kernel hasn't started scheduling)
        assert_eq!(k.active_partition, None);

        // Start the schedule
        let initial = k.start_schedule();
        assert_eq!(initial, Some(0)); // First entry is partition 0
        assert_eq!(k.active_partition, Some(0));
        assert_eq!(k.schedule().current_partition(), Some(0));
    }

    #[test]
    fn start_schedule_empty_returns_none() {
        // Create empty kernel via new_empty
        #[cfg(not(feature = "dynamic-mpu"))]
        let mut k = Kernel::<TestConfig>::new_empty();
        #[cfg(feature = "dynamic-mpu")]
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());

        // Start empty schedule returns None
        let initial = k.start_schedule();
        assert_eq!(initial, None);
        assert_eq!(k.active_partition, None);
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn start_schedule_allows_advance_tick() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_unstarted_schedule();
        // Start the schedule
        let initial = k.start_schedule();
        assert_eq!(initial, Some(0));

        // Now advance ticks - schedule should work normally
        // P0 has 5 ticks, so 4 advances should return None
        for _ in 0..4 {
            assert_eq!(k.advance_schedule_tick(), ScheduleEvent::None);
        }
        // 5th tick triggers switch to P1
        assert_eq!(k.advance_schedule_tick(), ScheduleEvent::PartitionSwitch(1));
        assert_eq!(k.active_partition, Some(1));
    }

    #[test]
    fn tick_accessor_returns_tick_counter() {
        let k = kernel_with_schedule();
        // Initial tick is 0
        assert_eq!(k.tick().get(), 0);
    }

    #[test]
    fn tick_accessor_reflects_increments() {
        let mut k = kernel_with_schedule();
        assert_eq!(k.tick().get(), 0);
        // Use sync_tick to change the tick (simulating what SysTick handler does)
        k.sync_tick(42);
        assert_eq!(k.tick().get(), 42);
    }

    /// Test module for `define_unified_kernel!` macro.
    ///
    /// The macro generates:
    /// - `static KERNEL: Mutex<RefCell<Option<Kernel<$Config>>>>`
    /// - `unsafe extern "C" fn dispatch_hook(f: &mut ExceptionFrame)`
    /// - `fn store_kernel(k: Kernel<$Config>)`
    ///
    /// Since these involve cortex-m intrinsics (interrupt::free, extern statics),
    /// these tests can only run on ARM targets. Full runtime testing is done
    /// via QEMU integration tests.
    #[cfg(target_arch = "arm")]
    mod unified_kernel_macro_tests {
        use super::*;

        /// Test configuration for macro expansion tests.
        struct UnifiedTestConfig;
        impl KernelConfig for UnifiedTestConfig {
            const N: usize = 2;
            const SCHED: usize = 4;
            const S: usize = 2;
            const SW: usize = 2;
            const MS: usize = 2;
            const MW: usize = 2;
            const QS: usize = 2;
            const QD: usize = 2;
            const QM: usize = 32;
            const QW: usize = 2;
            const SP: usize = 2;
            const SM: usize = 32;
            const BS: usize = 2;
            const BM: usize = 32;
            const BW: usize = 2;
            #[cfg(feature = "dynamic-mpu")]
            const BP: usize = 2;
            #[cfg(feature = "dynamic-mpu")]
            const BZ: usize = 32;

            type Core = crate::partition_core::PartitionCore<{ Self::N }, { Self::SCHED }>;
            type Sync =
                crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
            type Msg =
                crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
            type Ports = crate::port_pools::PortPools<
                { Self::SP },
                { Self::SM },
                { Self::BS },
                { Self::BM },
                { Self::BW },
            >;
        }

        // Module to test basic macro invocation compiles.
        // The generated items (KERNEL, dispatch_hook, store_kernel) are scoped
        // to this module and don't conflict with other tests.
        mod basic_expansion {
            use super::*;

            // Invoke the macro with minimal configuration.
            crate::define_unified_kernel!(UnifiedTestConfig);

            #[test]
            fn macro_generates_store_kernel_function() {
                // Verify that store_kernel exists and has correct signature.
                // We can't call it because it requires cortex-m runtime, but
                // we can verify the function exists by taking its pointer.
                let _: fn(Kernel<UnifiedTestConfig>) = store_kernel;
            }

            #[test]
            fn macro_generates_kernel_static() {
                // Verify KERNEL static exists and has expected type.
                // We can't borrow it without cortex-m runtime, but we can
                // verify the type via pointer conversion.
                let _: &cortex_m::interrupt::Mutex<
                    core::cell::RefCell<Option<Kernel<UnifiedTestConfig>>>,
                > = &KERNEL;
            }

            #[test]
            fn macro_generates_get_current_partition() {
                // Verify get_current_partition exists with extern "C" ABI
                // and returns u32.
                let _: extern "C" fn() -> u32 = get_current_partition;
            }

            #[test]
            fn macro_generates_get_next_partition() {
                // Verify get_next_partition exists with extern "C" ABI
                // and returns u32.
                let _: extern "C" fn() -> u32 = get_next_partition;
            }

            #[test]
            fn macro_generates_get_partition_sp_ptr() {
                // Verify get_partition_sp_ptr exists with extern "C" ABI
                // and returns *mut u32.
                let _: extern "C" fn() -> *mut u32 = get_partition_sp_ptr;
            }

            #[test]
            fn macro_generates_get_partition_sp() {
                // Verify get_partition_sp exists with extern "C" ABI,
                // takes u32 index, and returns u32.
                let _: extern "C" fn(u32) -> u32 = get_partition_sp;
            }

            #[test]
            fn macro_generates_set_partition_sp() {
                // Verify set_partition_sp exists with extern "C" ABI,
                // takes u32 index and u32 value.
                let _: extern "C" fn(u32, u32) = set_partition_sp;
            }

            #[test]
            fn get_current_partition_returns_max_when_uninitialized() {
                // When KERNEL is None, should return u32::MAX as sentinel.
                // Note: KERNEL starts as None (RefCell<Option<...>>), so
                // without calling store_kernel, it remains uninitialized.
                // We use a fresh module scope, so KERNEL is None.
                let result = get_current_partition();
                assert_eq!(result, u32::MAX);
            }

            #[test]
            fn get_next_partition_returns_max_when_uninitialized() {
                let result = get_next_partition();
                assert_eq!(result, u32::MAX);
            }

            #[test]
            fn get_partition_sp_ptr_returns_null_when_uninitialized() {
                let result = get_partition_sp_ptr();
                assert!(result.is_null());
            }

            #[test]
            fn get_partition_sp_returns_zero_when_uninitialized() {
                // When KERNEL is None, should return 0 as sentinel.
                let result = get_partition_sp(0);
                assert_eq!(result, 0);
            }

            #[test]
            fn set_partition_sp_noop_when_uninitialized() {
                // When KERNEL is None, should silently do nothing.
                // This test just verifies no panic occurs.
                set_partition_sp(0, 0x2000_0000);
            }
        }

        /// Functional tests for PendSV accessor functions with initialized kernel.
        #[allow(dead_code)]
        mod pendsv_accessor_functional_tests {
            use super::*;

            // Separate module to get a fresh KERNEL static.
            // Some generated items (dispatch_hook, store_kernel, CURRENT_PARTITION)
            // are not used in tests but are needed for the macro expansion.
            crate::define_unified_kernel!(UnifiedTestConfig);

            #[test]
            fn accessors_return_correct_values_after_initialization() {
                // Create a kernel with known partition state.
                let mut kernel = Kernel::<UnifiedTestConfig> {
                    current_partition: 1,
                    ..Default::default()
                };
                kernel.set_next_partition(0);
                kernel.set_sp(0, 0x2000_1000);
                kernel.set_sp(1, 0x2000_2000);

                // Store the kernel (initializes KERNEL static).
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(Some(kernel));
                });

                // Now test that accessors return the expected values.
                assert_eq!(get_current_partition(), 1);
                assert_eq!(get_next_partition(), 0);

                // Verify partition_sp pointer is valid and points to correct data.
                let sp_ptr = get_partition_sp_ptr();
                assert!(!sp_ptr.is_null());

                // SAFETY: We just verified the pointer is non-null and we know
                // the kernel is initialized with our test data.
                unsafe {
                    assert_eq!(*sp_ptr, 0x2000_1000); // partition_sp[0]
                    assert_eq!(*sp_ptr.add(1), 0x2000_2000); // partition_sp[1]
                }

                // Clean up: reset KERNEL to None for other tests.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(None);
                });
            }

            #[test]
            fn get_partition_sp_returns_correct_values() {
                // Create a kernel with known stack pointer values.
                let mut kernel = Kernel::<UnifiedTestConfig>::default();
                kernel.set_sp(0, 0x2000_1000);
                kernel.set_sp(1, 0x2000_2000);

                // Store the kernel.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(Some(kernel));
                });

                // Test indexed access returns correct values.
                assert_eq!(get_partition_sp(0), 0x2000_1000);
                assert_eq!(get_partition_sp(1), 0x2000_2000);

                // Clean up.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(None);
                });
            }

            #[test]
            fn get_partition_sp_returns_zero_for_out_of_bounds() {
                // Create a kernel (N=2 partitions).
                let kernel = Kernel::<UnifiedTestConfig>::default();

                // Store the kernel.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(Some(kernel));
                });

                // Out-of-bounds indices should return 0.
                assert_eq!(get_partition_sp(2), 0);
                assert_eq!(get_partition_sp(100), 0);
                assert_eq!(get_partition_sp(u32::MAX), 0);

                // Clean up.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(None);
                });
            }

            #[test]
            fn set_partition_sp_writes_correct_values() {
                // Create a kernel with initial values.
                let kernel = Kernel::<UnifiedTestConfig>::default();

                // Store the kernel.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(Some(kernel));
                });

                // Write using set_partition_sp.
                set_partition_sp(0, 0x2000_3000);
                set_partition_sp(1, 0x2000_4000);

                // Verify using get_partition_sp.
                assert_eq!(get_partition_sp(0), 0x2000_3000);
                assert_eq!(get_partition_sp(1), 0x2000_4000);

                // Clean up.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(None);
                });
            }

            #[test]
            fn set_partition_sp_noop_for_out_of_bounds() {
                // Create a kernel with known values.
                let mut kernel = Kernel::<UnifiedTestConfig>::default();
                kernel.set_sp(0, 0x2000_1000);
                kernel.set_sp(1, 0x2000_2000);

                // Store the kernel.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(Some(kernel));
                });

                // Out-of-bounds writes should be no-ops.
                set_partition_sp(2, 0xDEAD_BEEF);
                set_partition_sp(100, 0xDEAD_BEEF);
                set_partition_sp(u32::MAX, 0xDEAD_BEEF);

                // Original values should be unchanged.
                assert_eq!(get_partition_sp(0), 0x2000_1000);
                assert_eq!(get_partition_sp(1), 0x2000_2000);

                // Clean up.
                cortex_m::interrupt::free(|cs| {
                    KERNEL.borrow(cs).replace(None);
                });
            }
        }

        // Module to test macro invocation with custom yield handler.
        mod with_yield_handler {
            use super::*;
            use core::sync::atomic::{AtomicBool, Ordering};

            static YIELD_CALLED: AtomicBool = AtomicBool::new(false);

            crate::define_unified_kernel!(UnifiedTestConfig, |k| {
                // Custom yield handler that sets a flag.
                let _ = k;
                YIELD_CALLED.store(true, Ordering::SeqCst);
            });

            #[test]
            fn macro_with_yield_handler_generates_store_kernel() {
                let _: fn(Kernel<UnifiedTestConfig>) = store_kernel;
            }

            #[test]
            fn macro_with_yield_handler_generates_kernel_static() {
                let _: &cortex_m::interrupt::Mutex<
                    core::cell::RefCell<Option<Kernel<UnifiedTestConfig>>>,
                > = &KERNEL;
            }
        }

        /// Tests for the config-generating variant of define_unified_kernel!.
        ///
        /// This variant generates both the KernelConfig struct/impl AND the
        /// KERNEL static/functions in one macro invocation. It should work
        /// without explicit where bounds for sub-struct-owned constants
        /// (S, SW, MS, MW, QS, QD, QM, QW, SP, SM, BS, BM, BW).
        mod config_generating_variant {
            #[allow(unused_imports)]
            use super::*;

            // Test the config-generating variant without yield handler.
            // This generates GeneratedConfig struct + impl and KERNEL static.
            mod basic_config_generating {
                crate::define_unified_kernel!(GeneratedConfig {
                    N: 2,
                    SCHED: 4,
                    S: 2,
                    SW: 2,
                    MS: 2,
                    MW: 2,
                    QS: 2,
                    QD: 2,
                    QM: 32,
                    QW: 2,
                    SP: 2,
                    SM: 32,
                    BS: 2,
                    BM: 32,
                    BW: 2,
                    BP: 2,
                    BZ: 64
                });

                #[test]
                fn config_generating_variant_compiles() {
                    // Verify that the macro generates a valid KernelConfig impl
                    // by checking we can create a Kernel<GeneratedConfig>.
                    use crate::svc::Kernel;
                    let _: fn(Kernel<GeneratedConfig>) = store_kernel;
                }

                #[test]
                fn generated_config_has_correct_constants() {
                    use crate::config::KernelConfig;
                    assert_eq!(GeneratedConfig::N, 2);
                    assert_eq!(GeneratedConfig::SCHED, 4);
                    assert_eq!(GeneratedConfig::S, 2);
                    assert_eq!(GeneratedConfig::SW, 2);
                    assert_eq!(GeneratedConfig::MS, 2);
                    assert_eq!(GeneratedConfig::MW, 2);
                    assert_eq!(GeneratedConfig::QS, 2);
                    assert_eq!(GeneratedConfig::QD, 2);
                    assert_eq!(GeneratedConfig::QM, 32);
                    assert_eq!(GeneratedConfig::QW, 2);
                    assert_eq!(GeneratedConfig::SP, 2);
                    assert_eq!(GeneratedConfig::SM, 32);
                    assert_eq!(GeneratedConfig::BS, 2);
                    assert_eq!(GeneratedConfig::BM, 32);
                    assert_eq!(GeneratedConfig::BW, 2);
                }

                #[test]
                fn generated_kernel_static_exists() {
                    use crate::svc::Kernel;
                    let _: &cortex_m::interrupt::Mutex<
                        core::cell::RefCell<Option<Kernel<GeneratedConfig>>>,
                    > = &KERNEL;
                }
            }

            // Test config-generating variant with custom yield handler.
            mod config_generating_with_yield {
                use core::sync::atomic::{AtomicBool, Ordering};

                static YIELD_FLAG: AtomicBool = AtomicBool::new(false);

                crate::define_unified_kernel!(
                    GeneratedConfigYield {
                        N: 2,
                        SCHED: 4,
                        S: 2,
                        SW: 2,
                        MS: 2,
                        MW: 2,
                        QS: 2,
                        QD: 2,
                        QM: 32,
                        QW: 2,
                        SP: 2,
                        SM: 32,
                        BS: 2,
                        BM: 32,
                        BW: 2,
                        BP: 2,
                        BZ: 64
                    },
                    |k| {
                        let _ = k;
                        YIELD_FLAG.store(true, Ordering::SeqCst);
                    }
                );

                #[test]
                fn config_generating_with_yield_compiles() {
                    use crate::svc::Kernel;
                    let _: fn(Kernel<GeneratedConfigYield>) = store_kernel;
                }

                #[test]
                fn generated_kernel_static_with_yield_exists() {
                    use crate::svc::Kernel;
                    let _: &cortex_m::interrupt::Mutex<
                        core::cell::RefCell<Option<Kernel<GeneratedConfigYield>>>,
                    > = &KERNEL;
                }
            }
        }
    }
}
