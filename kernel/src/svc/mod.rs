pub mod accessors;
#[cfg(feature = "ipc-blackboard")]
pub mod blackboard;
#[cfg(feature = "dynamic-mpu")]
pub mod buf;
#[cfg(feature = "dynamic-mpu")]
pub mod buffer;
pub mod debug;
pub mod events;
pub mod irq;
#[cfg(feature = "ipc-message")]
pub mod msg;
#[cfg(feature = "ipc-queuing")]
pub mod queuing;
#[cfg(feature = "ipc-sampling")]
pub mod sampling;
pub mod scheduler;
pub mod sleep;
pub mod sync;

use core::cell::RefCell;
use core::marker::PhantomData;

use cortex_m::interrupt::Mutex;

use self::scheduler as svc_scheduler;
use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::context::ExceptionFrame;

// Re-export SvcError from shared traits crate for ABI isolation
pub use rtos_traits::syscall::SvcError;

/// Function-pointer type for SVC dispatch hooks and handlers.
///
/// Every function that the kernel invokes from the SVCall exception — whether
/// the built-in `dispatch_svc` or an application-installed hook — must match
/// this signature: it receives a mutable reference to the saved exception
/// frame so it can inspect syscall arguments and write back return values.
// TODO: define_irq_table! macro does not exist yet; adopt SvcDispatchFn there once it is added.
pub type SvcDispatchFn = unsafe extern "C" fn(&mut ExceptionFrame);

// ---------------------------------------------------------------------------
// Kernel struct-move invariant
// ---------------------------------------------------------------------------
//
// `Kernel` is constructed on the stack in `Kernel::new()` and then moved into
// the static `UNIFIED_KERNEL_STORAGE` via `core::ptr::write()`.  Because the
// struct changes address during this move, any pointer or address derived from
// a field *during* `new()` becomes stale once the struct lands in its final
// location.
//
// Affected fields (as of this writing):
//   - `PartitionControlBlock.mpu_region.base`  — patched by `fix_mpu_data_region()`
//   - `PartitionControlBlock.stack_base`        — set via `ExternalPartitionMemory` at construction
//
// Correct pattern:
//   1. Construct with real stack/config values via `ExternalPartitionMemory`.
//   2. Place into `UNIFIED_KERNEL_STORAGE` with `ptr::write()`.
//   3. Call `fix_mpu_data_region()` **after** placement so it computes
//      addresses from the struct's final location.
//   4. Verify with `invariants::assert_pcb_addresses_in_storage()`.
//
// See `boot.rs` for the post-placement fixup sequence and `invariants.rs` for
// the runtime assertion that all PCB-embedded addresses fall within the static
// storage region.
// ---------------------------------------------------------------------------

// ---- Kernel Memory Region Constants ----

/// End address of kernel code region in flash (exclusive).
///
/// Kernel code occupies the lower portion of flash starting at 0x0000_0000.
/// This 64 KiB estimate covers the kernel binary, vector table, and read-only
/// data. Pointers in `[0, KERNEL_CODE_END)` are rejected by `validate_user_ptr`.
pub const KERNEL_CODE_END: u32 = 0x0001_0000;

/// End address of kernel data region in SRAM (exclusive).
///
/// Kernel data (BSS, statics) occupies the lower portion of SRAM starting at
/// 0x2000_0000. Pointers in `[0x2000_0000, KERNEL_DATA_END)` are rejected.
/// Set to 0x2000_0000 to disable this check (code region check remains active).
pub const KERNEL_DATA_END: u32 = 0x2000_0000;

/// Sentinel value used by the `frame()` test helper for the `r3` register.
///
/// The 3-argument `frame(r0, r1, r2)` helper fills `r3` with this value.
/// The SYS_BUF_LEND handler treats it as "no deadline" so that tests written
/// before the r3-deadline feature continue to work unchanged.
#[cfg(any(test, feature = "dynamic-mpu"))]
const LEGACY_TEST_FRAME_R3_SENTINEL: u32 = 0xCC;

/// Return the runtime end-address of the kernel data region.
///
/// On ARM targets the linker defines `__kernel_state_end` marking where kernel
/// state ends in SRAM. On host/test builds the compile-time constant
/// `KERNEL_DATA_END` (0x2000_0000) is returned, keeping the check disabled.
#[cfg(target_arch = "arm")]
#[inline]
fn kernel_data_end() -> u32 {
    extern "C" {
        static __kernel_state_end: u8;
    }
    // SAFETY: We only take the address of the linker symbol, never dereference it.
    // The unsafe block is required on ARM targets where the extern static is real.
    #[allow(unused_unsafe)]
    unsafe {
        core::ptr::addr_of!(__kernel_state_end) as u32
    }
}

// Per-test override for kernel_data_end(), allowing tests to inject a
// non-trivial kernel data boundary and verify Guard 3 through validate_user_ptr.
#[cfg(test)]
std::thread_local! {
    static KERNEL_DATA_END_OVERRIDE: std::cell::Cell<Option<u32>> =
        const { std::cell::Cell::new(None) };
}

/// Host/test fallback: returns the compile-time constant (disabled),
/// unless a test override is active.
#[cfg(not(target_arch = "arm"))]
#[inline]
fn kernel_data_end() -> u32 {
    #[cfg(test)]
    if let Some(v) = KERNEL_DATA_END_OVERRIDE.with(|c| c.get()) {
        return v;
    }
    KERNEL_DATA_END
}

/// Check if the range `[ptr, end)` overlaps kernel data `[0x2000_0000, kernel_end)`.
///
/// Parameterized so unit tests can inject an arbitrary `kernel_end` value.
/// When `kernel_end == 0x2000_0000` the region is empty and no addresses are rejected.
#[inline]
fn overlaps_kernel_data(ptr: u32, end: u32, kernel_end: u32) -> bool {
    ptr < kernel_end && end > 0x2000_0000
}

/// Unpack a packed r2 register used by timed syscalls.
///
/// ABI: `r2 = (timeout_ticks_hi16 << 16) | buf_len_lo16`.
/// Returns `(timeout_ticks, buf_len)`.
#[inline]
pub fn unpack_packed_r2(r2: u32) -> (u16, u16) {
    ((r2 >> 16) as u16, (r2 & 0xFFFF) as u16)
}

/// Validate that `[ptr, ptr+len)` fits within a static region of partition `pid`.
///
/// Guards: (1) flash rejects `[0, KERNEL_CODE_END)`, (2) grant accepts if range
/// fits a partition region, (3) SRAM rejects `[0x2000_0000, kernel_data_end())`.
/// Grant (2) runs before SRAM guard (3) so granted regions are accepted first.
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

    // Guard 1 — Flash: unconditionally reject pointers in kernel code region.
    if ptr < KERNEL_CODE_END && end > 0 {
        return false;
    }

    // Guard 2 — Accessible-regions grant (runs before SRAM guard).
    for (base, size) in pcb.accessible_static_regions() {
        let region_end = base + size;
        if ptr >= base && end <= region_end {
            return true;
        }
    }

    // Guard 3 — SRAM kernel-data: reject overlap with kernel data in SRAM.
    if overlaps_kernel_data(ptr, end, kernel_data_end()) {
        return false;
    }

    false
}

/// Dynamic-MPU variant of [`validate_user_ptr`].
///
/// Same three-guard ordering as `validate_user_ptr` (flash → grant → SRAM),
/// but the grant step also checks dynamic MPU windows from
/// `strategy.accessible_regions(pid)` before falling back to static regions.
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

    // Guard 1 — Flash: unconditionally reject pointers in kernel code region.
    if ptr < KERNEL_CODE_END && end > 0 {
        return false;
    }

    // Helper: check if [ptr, end) lies entirely within [base, base+size].
    // Uses saturating_add to handle regions that extend to or past 0xFFFF_FFFF.
    let in_region = |base: u32, size: u32| -> bool {
        let region_end = base.saturating_add(size);
        ptr >= base && end <= region_end
    };

    // Guard 2a — Dynamic MPU windows assigned to this partition.
    let windows = strategy.accessible_regions(pid);
    for (base, size) in windows {
        if in_region(base, size) {
            return true;
        }
    }

    // Guard 2b — Static regions (data, stack, and peripheral regions).
    for (base, size) in pcb.accessible_static_regions() {
        if in_region(base, size) {
            return true;
        }
    }

    // Guard 3 — SRAM kernel-data: reject overlap with kernel data in SRAM.
    if overlaps_kernel_data(ptr, end, kernel_data_end()) {
        return false;
    }

    false
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

#[cfg(test)]
use crate::events as ev_data;
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
use crate::message::MessagePool;
#[cfg(test)]
use crate::message::{RecvOutcome, SendOutcome};
use crate::mutex::MutexPool;
use crate::partition::{
    ConfigError, PartitionConfig, PartitionControlBlock, PartitionState, PartitionTable,
};
use crate::queuing::{QueuingPortPool, QueuingPortStatus, RecvQueuingOutcome, SendQueuingOutcome};
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
//
// SAFETY: This assembly block defines the SVC exception handler trampoline that is sound
// because:
//
// 1. **PSP-Based Frame Pointer Extraction**:
//    - `mrs r0, psp` reads the Process Stack Pointer (PSP) which points to the exception
//      frame that hardware pushed on SVC entry. Partitions always use PSP in Thread mode.
//    - The PSP points to an 8-word hardware-stacked frame: r0, r1, r2, r3, r12, lr, pc, xpsr.
//    - Passing PSP in r0 to `dispatch_svc` allows the Rust handler to access syscall
//      arguments (frame.r0-r3) and modify return values (frame.r0) in place.
//    - The kernel uses MSP (Main Stack Pointer) in Handler mode, so reading PSP does not
//      affect the kernel's own stack.
//
// 2. **Register Clobber Conventions (AAPCS)**:
//    - Per ARM AAPCS, r0-r3 are caller-saved argument/scratch registers. The `bl dispatch_svc`
//      call may clobber r0-r3 freely; hardware will restore the *modified* frame values on
//      exception return (allowing syscall return values in r0).
//    - r12 is a scratch register (ip) that AAPCS allows callees to clobber.
//    - lr is saved with `push {lr}` before `bl` and restored with `pop {pc}` after, preserving
//      the EXC_RETURN value needed for proper exception return.
//    - r4-r11 are callee-saved; `dispatch_svc` (Rust code) preserves them per AAPCS, so no
//      explicit save/restore is needed in this trampoline.
//
// 3. **Exception Return Behavior**:
//    - On SVC entry, hardware loads lr with an EXC_RETURN value (0xFFFFFFFD for Thread mode
//      using PSP, no FPU context).
//    - `push {lr}` saves EXC_RETURN before the `bl` clobbers lr with the return address.
//    - `pop {pc}` loads the saved EXC_RETURN into pc, triggering the hardware exception
//      return sequence: hardware unstacks r0-r3, r12, lr, pc, xpsr from PSP and resumes
//      the partition at the instruction after `svc`.
//    - The modified frame.r0 (syscall return value) is what hardware restores to r0.
//
// 4. **Calling Context From Exception Mode**:
//    - SVCall executes in Handler mode at the priority configured in SHPR2 (default 0).
//    - Handler mode always uses MSP, never PSP, so kernel stack is separate from partition.
//    - The trampoline runs with interrupts at the SVC priority level; higher-priority
//      interrupts can preempt, but lower-priority cannot.
//    - `dispatch_svc` is a safe Rust `extern "C"` function that receives a mutable reference
//      to the exception frame. The frame remains valid for the entire handler duration
//      because the partition is suspended until exception return.
#[cfg(all(target_arch = "arm", not(test)))]
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".global SVCall",
    ".type SVCall, %function",
    "SVCall:",
    // Defense-in-depth: validate lr == EXC_RETURN_THREAD_PSP (0xFFFFFFFD).
    // If SVC is entered from Handler mode, lr would be a different EXC_RETURN
    // value and PSP would not point to the caller's exception frame.
    "movw r0, #0xFFFD",
    "movt r0, #0xFFFF",
    "cmp lr, r0",
    "bne .Lsvc_bad_exc_return",
    // Defense-in-depth: clear both PRIMASK and BASEPRI so Thread mode
    // runs with all configurable interrupts unblocked. PendSV now uses
    // PRIMASK (cpsid i / cpsie i) for its critical section; the BASEPRI
    // clear is retained for belt-and-suspenders safety. On the normal
    // path where both are already 0, these are harmless no-ops (~2 cycles
    // each).
    //
    // SAFETY: CPSIE I clears PRIMASK, enabling all configurable exceptions.
    // Writing 0 to BASEPRI disables priority masking (ARMv7-M B5.2.3).
    // r0 is used as scratch; `mrs r0, psp` on the very next instruction
    // overwrites r0 unconditionally, so no prior value is lost.
    "cpsie i",
    "mov r0, #0",
    "msr BASEPRI, r0",
    "mrs r0, psp",
    "push {{lr}}",
    "bl dispatch_svc",
    "pop {{pc}}",
    ".Lsvc_bad_exc_return:",
    "b .Lsvc_bad_exc_return",
    ".size SVCall, . - SVCall",
);

/// Reference this function pointer to ensure the linker includes the SVCall
/// assembly trampoline and `dispatch_svc` in the final binary. Without an
/// explicit Rust-level reference, the linker may discard the object.
pub static SVC_HANDLER: SvcDispatchFn = dispatch_svc;

/// Optional application-provided dispatch hook. When set, `dispatch_svc`
/// forwards every SVC frame to this function instead of using the built-in
/// minimal dispatch. Applications set this to route syscalls through a
/// full `Kernel::dispatch` that has access to all kernel service pools.
///
/// Protected by a `cortex_m::interrupt::Mutex` so that reads and writes
/// are performed inside critical sections on single-core Cortex-M.
static SVC_DISPATCH_HOOK: Mutex<RefCell<Option<SvcDispatchFn>>> = Mutex::new(RefCell::new(None));

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
pub fn set_dispatch_hook(hook: SvcDispatchFn) {
    with_cs(|cs| {
        SVC_DISPATCH_HOOK.borrow(cs).replace(Some(hook));
    });
}

/// Declares a unified kernel storage static with dispatch hook and store function.
///
/// This macro generates:
/// - `unsafe extern "C" fn dispatch_hook(f: &mut ExceptionFrame)` — the SVC dispatch hook
/// - `fn store_kernel(k: Kernel<$Config>)` — stores the kernel and installs the hook (logs via `klog!` on failure)
/// - PendSV accessors: `get_partition_sp_ptr`, `get_partition_sp`, `set_partition_sp`
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
        $crate::define_unified_kernel!(@impl_named KERNEL, $Config, |_k| {});
    };
    // Variant with custom yield handler.
    ($Config:ty, |$k:ident| $yield_body:block) => {
        $crate::define_unified_kernel!(@impl_named KERNEL, $Config, |$k| $yield_body);
    };
    // Named variant: `define_unified_kernel!(MY_KERNEL: Kernel<MyConfig>);`
    // Allows specifying a custom name for the kernel static.
    ($name:ident : Kernel<$Config:ty>) => {
        $crate::define_unified_kernel!(@impl_named $name, $Config, |_k| {});
    };
    // Named variant with custom yield handler.
    ($name:ident : Kernel<$Config:ty>, |$k:ident| $yield_body:block) => {
        $crate::define_unified_kernel!(@impl_named $name, $Config, |$k| $yield_body);
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
        $crate::define_unified_kernel!(@impl_named KERNEL, $Name, |_k| {});
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
        $crate::define_unified_kernel!(@impl_named KERNEL, $Name, |$k| $yield_body);
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

            $crate::kernel_config_types!();
        }
    };
    // Internal implementation rule using @impl_named token as private pattern.
    // NOTE: The @impl_named pattern below is valid Rust macro syntax. If a code review tool
    // reports a "file path as matcher token" error, that is a false positive from the
    // tool - verify by running `cargo check` which succeeds. The @impl idiom is standard
    // for private macro rules (see std::vec!, lazy_static!, etc.).
    //
    // BOUNDS: This rule uses `Kernel<$Config>` which inherits its where clause from the
    // Kernel struct definition. The struct was updated to remove sub-struct-owned const
    // bounds (S, SW, MS, MW, QS, QD, QM, QW, SP, SM, BS, BM, BW).
    // The macro itself does not need explicit where clauses; reduced bounds are achieved
    // through the struct definition.
    (@impl_named $name:ident, $Config:ty, |$k:ident| $yield_body:block) => {
        // ============================================================
        // Kernel struct field offsets for direct assembly access.
        //
        // These constants are computed at compile time using offset_of!
        // and exported as #[no_mangle] symbols. PendSV assembly loads
        // __kernel_state_start and adds these offsets to access fields
        // directly, avoiding function call overhead during context switch.
        // ============================================================

        /// Offset of `current_partition` field in `Kernel<'mem, C>`.
        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static KERNEL_CURRENT_PARTITION_OFFSET: usize =
            ::core::mem::offset_of!($crate::svc::Kernel<'static, $Config>, current_partition);

        /// Offset of `ticks_dropped` field in `Kernel<'mem, C>`.
        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static KERNEL_TICKS_DROPPED_OFFSET: usize =
            ::core::mem::offset_of!($crate::svc::Kernel<'static, $Config>, ticks_dropped);

        /// Offset of `core` field in `Kernel<'mem, C>`.
        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static KERNEL_CORE_OFFSET: usize =
            ::core::mem::offset_of!($crate::svc::Kernel<'static, $Config>, core);

        /// Offset of `next_partition` field within `PartitionCore`.
        /// To get the absolute offset from kernel base, add KERNEL_CORE_OFFSET.
        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static CORE_NEXT_PARTITION_OFFSET: usize =
            ::core::mem::offset_of!(<$Config as $crate::config::KernelConfig>::Core, next_partition);

        /// Offset of `partition_sp` array within `PartitionCore`.
        /// To get the absolute offset from kernel base, add KERNEL_CORE_OFFSET.
        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static CORE_PARTITION_SP_OFFSET: usize =
            ::core::mem::offset_of!(<$Config as $crate::config::KernelConfig>::Core, partition_sp);

        #[cfg_attr(not(test), no_mangle)]
        #[cfg_attr(not(test), link_section = ".rodata")]
        #[used]
        static CORE_PARTITION_STACK_LIMIT_OFFSET: usize =
            ::core::mem::offset_of!(<$Config as $crate::config::KernelConfig>::Core, partition_stack_limits);

        // Shared offset-limit, field-ordering, alignment, and stride checks.
        $crate::assert_kernel_layout!($Config);

        // SVC-specific: verify exported ABI statics match offset_of! and
        // check type constraints for ldrb/strb instructions.
        const _: () = {
            type K = $crate::svc::Kernel<'static, $Config>;
            type C = <$Config as $crate::config::KernelConfig>::Core;

            // Verify the exported ABI constants match the actual struct layout.
            assert!(
                KERNEL_CURRENT_PARTITION_OFFSET == ::core::mem::offset_of!(K, current_partition),
                "KERNEL_CURRENT_PARTITION_OFFSET does not match struct layout"
            );
            assert!(
                KERNEL_TICKS_DROPPED_OFFSET == ::core::mem::offset_of!(K, ticks_dropped),
                "KERNEL_TICKS_DROPPED_OFFSET does not match struct layout"
            );
            assert!(
                KERNEL_CORE_OFFSET == ::core::mem::offset_of!(K, core),
                "KERNEL_CORE_OFFSET does not match struct layout"
            );
            assert!(
                CORE_NEXT_PARTITION_OFFSET == ::core::mem::offset_of!(C, next_partition),
                "CORE_NEXT_PARTITION_OFFSET does not match struct layout"
            );
            assert!(
                CORE_PARTITION_SP_OFFSET == ::core::mem::offset_of!(C, partition_sp),
                "CORE_PARTITION_SP_OFFSET does not match struct layout"
            );
            assert!(
                CORE_PARTITION_STACK_LIMIT_OFFSET == ::core::mem::offset_of!(C, partition_stack_limits),
                "CORE_PARTITION_STACK_LIMIT_OFFSET does not match struct layout"
            );
            // PendSV uses ldrb/strb for current_partition — must be u8.
            #[allow(unused)]
            fn _assert_cp_is_u8(k: &K) { let _: u8 = k.current_partition; }

            // PendSV uses ldrb for next_partition — must be u8.
            #[allow(unused)]
            fn _assert_np_is_u8(c: &C) { let _: u8 = c.next_partition; }
        };

        /// SVC dispatch hook that routes syscalls through the unified kernel.
        ///
        /// # Safety
        ///
        /// Must be called from SVC exception context with a valid `ExceptionFrame`
        /// pointer from the process stack (PSP).
        unsafe extern "C" fn dispatch_hook(f: &mut $crate::context::ExceptionFrame) {
            // Delegate to state module for unified kernel storage.
            let _ = $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                // SAFETY: `f` is a valid exception frame pointer from PSP.
                unsafe { k.dispatch(f) }
                if k.yield_requested {
                    k.yield_requested = false;
                    let $k = k;
                    $yield_body
                }
            });
        }

        /// Store the kernel instance and install the SVC dispatch hook.
        ///
        /// This function:
        /// 1. Stores the provided `Kernel` instance in the global unified kernel storage
        /// 2. Installs `dispatch_hook` as the SVC exception handler
        ///
        /// On failure, logs the error via `klog!` before panicking.
        ///
        /// Must be called exactly once during initialization, before enabling
        /// interrupts or starting the scheduler.
        fn store_kernel(k: $crate::svc::Kernel<'static, $Config>) {
            // SAFETY: Called once during init before interrupts enabled.
            if let Err(e) = unsafe { $crate::state::init_kernel_state(k) } {
                $crate::klog!("store_kernel failed: {:?}", e);
                panic!("{}", e);
            }
            // SAFETY: init_kernel_state succeeded so UNIFIED_KERNEL_STORAGE
            // contains a valid Kernel<'static, $Config>. Called exactly once
            // before interrupts are enabled, satisfying store_kernel_ptr's
            // requirement that the kernel outlives all load_kernel_ptr callers.
            unsafe {
                let kp = $crate::state::get_kernel_ptr::<$Config>();
                $crate::kernel_ptr::store_kernel_ptr(&mut *kp);
            }
            $crate::svc::set_dispatch_hook(dispatch_hook);
        }

        /// Helper to access the kernel within an interrupt-free critical section.
        ///
        /// Consolidates the repeated pattern of `interrupt::free` + `borrow` +
        /// `as_ref`/`as_mut` into a single abstraction for PendSV accessor functions.
        ///
        /// Returns `None` if the kernel is not initialized; otherwise passes a
        /// reference to the closure and returns its result wrapped in `Some`.
        // TODO: Reviewer feedback suggests a cleaner abstraction layer for all
        // C-ABI shims that return raw pointers to kernel state (issue #3).
        #[inline]
        fn with_kernel<T, F: FnOnce(&$crate::svc::Kernel<'static, $Config>) -> T>(f: F) -> Option<T> {
            $crate::state::with_kernel::<$Config, _, _>(f).ok()
        }

        /// Mutable variant of [`with_kernel`] for accessors that need `&mut`.
        #[inline]
        fn with_kernel_mut<T, F: FnOnce(&mut $crate::svc::Kernel<'static, $Config>) -> T>(f: F) -> Option<T> {
            $crate::state::with_kernel_mut::<$Config, _, _>(f).ok()
        }

        /// Returns a pointer to the partition_sp array in the Kernel struct.
        ///
        /// Called by the Rust shim from PendSV assembly to read/write saved
        /// stack pointers during context switch. Uses `interrupt::free` to
        /// safely access kernel state.
        ///
        /// Returns null pointer if the kernel is not initialized.
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
        /// context switch. Uses `interrupt::free` to safely access kernel state.
        ///
        /// Returns 0 if the kernel is not initialized or index is out of bounds.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn get_partition_sp(idx: u32) -> u32 {
            with_kernel(|k| k.get_sp(idx as usize).unwrap_or(0)).unwrap_or(0)
        }

        /// Sets the stack pointer for a partition by index.
        ///
        /// Called by PendSV assembly to save stack pointers during context
        /// switch. Uses `interrupt::free` to safely access kernel state.
        ///
        /// No-op if the kernel is not initialized or index is out of bounds.
        #[cfg_attr(not(test), no_mangle)]
        #[allow(dead_code)] // Called from assembly, not Rust
        extern "C" fn set_partition_sp(idx: u32, sp: u32) {
            with_kernel_mut(|k| { let _ = k.set_sp(idx as usize, sp); });
        }

    };
}

/// Dispatch an SVC call based on the syscall number in `frame.r0`.
///
/// Production systems install a dispatch hook via [`set_dispatch_hook`]
/// that forwards syscalls to [`Kernel::dispatch`], which has access to
/// the partition table and can route event, IPC, and IRQ syscalls to
/// their real implementations. When a hook is installed, `dispatch_svc`
/// simply delegates to it and returns.
///
/// The built-in fallback (no hook installed) only handles `Yield`;
/// all other recognised syscall IDs return
/// [`SvcError::InvalidSyscall`]. This fallback exists so that unit
/// tests and early boot can exercise the SVC entry path without
/// a fully configured kernel.
///
/// # Safety
///
/// The caller must pass a valid pointer to the hardware-stacked
/// `ExceptionFrame` on the process stack (PSP). This is guaranteed
/// when called from the SVCall assembly trampoline above.
#[no_mangle]
pub unsafe extern "C" fn dispatch_svc(frame: &mut ExceptionFrame) {
    let hook = with_cs(|cs| *SVC_DISPATCH_HOOK.borrow(cs).borrow());
    if let Some(hook) = hook {
        hook(frame);
        return;
    }
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(_) => SvcError::InvalidSyscall.to_u32(),
        None => SvcError::InvalidSyscall.to_u32(),
    };
}

/// Core syscall dispatch that routes event and IRQ syscalls to their
/// respective modules.
///
/// `caller` is the kernel-trusted partition index of the calling partition,
/// used by syscalls that operate on the caller's own partition (e.g.
/// EventWait, EventClear).
///
/// Frame register convention:
/// - `r0`: syscall ID (overwritten with return value)
/// - `r1`: first argument (interpretation varies per syscall)
/// - `r2`: second argument (event mask)
///
/// # ARM target note
///
/// On success (`r0 == 0`), `IrqAck` callers must additionally call
/// `NVIC::unmask` on the acknowledged IRQ number to re-enable it in
/// hardware. [`Kernel::dispatch`] handles this automatically; direct
/// callers of `dispatch_syscall` are responsible for doing so
/// themselves.
pub fn dispatch_syscall<const N: usize>(
    frame: &mut ExceptionFrame,
    partitions: &mut PartitionTable<N>,
    caller: usize,
) {
    use events as ev;
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(SyscallId::GetPartitionId) => caller as u32,
        // TODO: dispatch_syscall cannot trigger descheduling; blocking
        // state is silently dropped here. Callers needing full EventWait
        // semantics should use Kernel::dispatch (handle_svc) instead.
        Some(SyscallId::EventWait) => ev::handle_event_wait(partitions, caller, frame.r2).0,
        Some(SyscallId::EventSet) => ev::handle_event_set(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::EventClear) => ev::handle_event_clear(partitions, caller, frame.r2),
        Some(SyscallId::IrqAck) => SvcError::InvalidResource.to_u32(),
        Some(_) => SvcError::InvalidSyscall.to_u32(),
        None => SvcError::InvalidSyscall.to_u32(),
    };
}

// ---- Struct-Move Invariant ----
//
// `Kernel` is constructed on the stack in `Kernel::new()` and then moved into
// `UNIFIED_KERNEL_STORAGE` via `ptr.write(kernel)` (see `state.rs`).  Any
// address captured from internal fields during `new()` — stack buffer
// pointers, MPU region bases — becomes **stale** after the move because the
// struct now lives at a different address.
//
// Fields that store self-referential addresses must therefore be **patched
// after placement**, not during construction.  The correct pattern is:
//
//   1. Construct with real stack/config values via `ExternalPartitionMemory`.
//   2. After placement in `boot_preconfigured()`, call the appropriate `fix_*()` method
//      which recomputes the address from the live storage location.
//   3. Verify the patched address falls within the `UNIFIED_KERNEL_STORAGE`
//      range.
//
// Currently affected fields and their fixup methods:
//   - `PartitionControlBlock.mpu_region.base`  → `fix_mpu_data_region()`
//   - `PartitionControlBlock.stack_base`        → set via `ExternalPartitionMemory` at construction
//
// See `boot.rs` for the post-placement fixup sequence that calls these
// methods after the kernel has been moved into its final storage.

/// Encapsulates all kernel service pools alongside the partition table,
/// with pool sizes derived from a single [`KernelConfig`] implementer.
///
/// # Representation
///
/// This struct uses `#[repr(C)]` to ensure deterministic field layout for
/// direct memory access from PendSV assembly. The assembly loads
/// `__kernel_state_start` and uses compile-time field offsets to access
/// `current_partition`, `core.next_partition`, and `core.partition_sp[]`
/// without calling Rust shim functions.
#[repr(C)]
pub struct Kernel<'mem, C: KernelConfig>
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
    /// Monotonic counter of SysTick ticks that were dropped (i.e. a tick
    /// arrived while the previous one was still pending). Incremented by
    /// the SysTick handler when it detects PENDSTSET was already set.
    pub ticks_dropped: u32,
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
    /// The `check_user_ptr_dynamic` method queries this strategy to validate
    /// pointers against both static MPU regions and dynamic windows.
    #[cfg(feature = "dynamic-mpu")]
    pub dynamic_strategy: crate::mpu_strategy::DynamicStrategy,
    /// Counts ticks since the last bottom-half (system window) processing.
    ///
    /// Incremented on each call to `advance_schedule_tick()`, reset to 0
    /// when `ScheduleEvent::SystemWindow` is returned. Used to monitor
    /// bottom-half staleness at runtime.
    #[cfg(feature = "dynamic-mpu")]
    pub ticks_since_bottom_half: u32,
    /// Diagnostic flag: true when `ticks_since_bottom_half` exceeds
    /// `C::SYSTEM_WINDOW_MAX_GAP_TICKS`, indicating bottom-half processing
    /// is overdue. Used for runtime health monitoring.
    #[cfg(feature = "dynamic-mpu")]
    bottom_half_stale: bool,
    /// Guard flag for nested bottom-half detection. Set/cleared by `run_bottom_half!`.
    #[cfg(feature = "dynamic-mpu")]
    pub in_bottom_half: bool,
    /// Partition/schedule state sub-struct containing partitions, schedule,
    /// current_partition, next_partition, and partition_sp.
    pub core: C::Core,
    /// Synchronization primitives sub-struct (will replace individual fields).
    pub sync: C::Sync,
    /// Message-passing primitives sub-struct (will replace individual fields).
    pub msg: C::Msg,
    /// Port primitives sub-struct (will replace individual fields).
    pub ports: C::Ports,
    pub irq_bindings: &'static [crate::irq_dispatch::IrqBinding], // IrqAck dispatch table
    /// Sorted sleep timer queue for O(1) amortised wakeup of sleeping
    /// partitions. Entries are inserted by `SYS_SLEEP` and drained each
    /// tick in [`expire_timed_waits`](Self::expire_timed_waits).
    pub sleep_queue: crate::waitqueue::SleepQueue<{ C::N }>,
    _memory: PhantomData<&'mem ()>,
}

#[cfg(test)]
impl<'mem, C: KernelConfig> Default for Kernel<'mem, C>
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
        #[allow(deprecated)]
        Self::new_empty(
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
        )
    }
}

impl<'mem, C: KernelConfig> Kernel<'mem, C>
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
    /// Primary constructor: create a `Kernel` from [`ExternalPartitionMemory`] descriptors.
    ///
    /// Partition IDs are derived from array indices (0, 1, 2, …).
    /// PCBs are created with sentinel stack values (0, 0) that
    /// [`boot_preconfigured()`](crate::boot::boot_preconfigured) patches later.
    /// Entry points and MPU regions are preserved from each descriptor.
    pub fn new(
        schedule: ScheduleTable<{ C::SCHED }>,
        memories: &[crate::partition::ExternalPartitionMemory<'_>],
    ) -> Result<Self, ConfigError> {
        if memories.len() > C::N {
            return Err(ConfigError::PartitionTableFull);
        }
        // TODO: memories.len() < C::N creates fewer partitions than C::N.
        // This is intentional: with_config creates exactly memories.len()
        // partitions, each fully initialized.  Schedule validation catches
        // any entry referencing a non-existent partition index.
        let mut configs: heapless::Vec<PartitionConfig, { C::N }> = heapless::Vec::new();
        for (i, m) in memories.iter().enumerate() {
            let cfg = PartitionConfig {
                id: u8::try_from(i).map_err(|_| ConfigError::PartitionTableFull)?,
                entry_point: m.entry_point(),
                mpu_region: *m.mpu_region(),
                peripheral_regions: m.peripheral_regions().clone(),
                r0_hint: m.r0_hint(),
                code_mpu_region: m.code_mpu_region().copied(),
                stack_base: m.stack_base(),
                stack_size: m.stack_size_bytes(),
            };
            configs
                .push(cfg)
                .map_err(|_| ConfigError::PartitionTableFull)?;
        }
        Self::with_config(
            schedule,
            &configs,
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
            &[],
        )
    }

    /// Create a `Kernel` with `C::N` sentinel partitions.
    ///
    /// Primary constructor for custom kernel configurations.
    ///
    /// Validates that: schedule is non-empty, all schedule entries reference
    /// valid partitions, and all partition configs pass MPU/stack validation.
    /// Use this when explicitly supplying a `KernelConfig` type parameter.
    pub fn with_config(
        schedule: ScheduleTable<{ C::SCHED }>,
        configs: &[PartitionConfig],
        #[cfg(feature = "dynamic-mpu")] registry: crate::virtual_device::DeviceRegistry<
            'static,
            { C::DR },
        >,
        irq_bindings: &'static [crate::irq_dispatch::IrqBinding],
    ) -> Result<Self, ConfigError> {
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
        // Validate system window presence for dynamic-mpu builds.
        #[cfg(feature = "dynamic-mpu")]
        if !schedule.has_system_window() {
            return Err(ConfigError::NoSystemWindow);
        }
        // Validate system window frequency for dynamic-mpu builds.
        #[cfg(feature = "dynamic-mpu")]
        {
            let max_gap = schedule.max_ticks_without_system_window();
            if max_gap > C::SYSTEM_WINDOW_MAX_GAP_TICKS {
                return Err(ConfigError::SystemWindowTooInfrequent {
                    max_gap_ticks: max_gap,
                    threshold_ticks: C::SYSTEM_WINDOW_MAX_GAP_TICKS,
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
            // Note: We skip c.validate() here. We must still validate
            // mpu_region and peripheral_regions, which ARE used in the final PCB.
            // Size==0 is a sentinel meaning "no user-configured data region".
            if c.mpu_region.size() > 0 {
                crate::mpu::validate_mpu_region(c.mpu_region.base(), c.mpu_region.size()).map_err(
                    |detail| ConfigError::MpuRegionInvalid {
                        partition_id: c.id,
                        detail,
                    },
                )?;
            }
            if let Some(ref code) = c.code_mpu_region {
                if code.size() > 0 {
                    crate::mpu::validate_mpu_region(code.base(), code.size()).map_err(
                        |detail| ConfigError::CodeRegionInvalid {
                            partition_id: c.id,
                            detail,
                        },
                    )?;
                }
                // Verify the entry point (Thumb bit stripped) falls within
                // the code MPU region [base, base+size).
                let effective_entry = c.entry_point.raw() & !1;
                if effective_entry.wrapping_sub(code.base()) >= code.size() {
                    return Err(ConfigError::EntryPointOutsideCodeRegion {
                        partition_id: c.id,
                        entry_point: c.entry_point.raw(),
                        region_base: code.base(),
                        region_size: code.size(),
                    });
                }
            }
            for (j, region) in c.peripheral_regions.iter().enumerate() {
                crate::mpu::validate_mpu_region(region.base(), region.size()).map_err(
                    |detail| ConfigError::PeripheralRegionInvalid {
                        partition_id: c.id,
                        region_index: j,
                        detail,
                    },
                )?;
            }
            // Stack values default to 0 (sentinel) when not provided.
            // boot_preconfigured() patches sentinels with real addresses
            // once the kernel is in its final storage location.
            let mpu_region = c.mpu_region;
            let stack_base = c.stack_base;
            let stack_pointer = stack_base.wrapping_add(c.stack_size);
            let mut pcb = PartitionControlBlock::new(
                c.id,
                c.entry_point,
                stack_base,
                stack_pointer,
                mpu_region,
            )
            .with_peripheral_regions(&c.peripheral_regions);
            if let Some(code_region) = c.code_mpu_region {
                pcb = pcb.with_code_mpu_region(code_region);
            }
            pcb.set_r0_hint(c.r0_hint);
            if core.partitions_mut().add(pcb).is_err() {
                return Err(ConfigError::PartitionTableFull);
            }
            core.set_sp(i, stack_pointer);
        }
        #[cfg(debug_assertions)]
        crate::invariants::assert_no_overlapping_mpu_regions(core.partitions().as_slice());
        Ok(Self::init_kernel_struct(
            core,
            irq_bindings,
            #[cfg(feature = "dynamic-mpu")]
            registry,
        ))
    }

    /// Construct a `Kernel` from a fully-populated `Core` and IRQ bindings.
    ///
    /// This is the single place where the `Kernel` struct literal is built.
    /// `with_config()` delegates here after populating the core's
    /// partition table and schedule.
    fn init_kernel_struct(
        core: C::Core,
        irq_bindings: &'static [crate::irq_dispatch::IrqBinding],
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
            ticks_dropped: 0,
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
            #[cfg(feature = "dynamic-mpu")]
            ticks_since_bottom_half: 0,
            #[cfg(feature = "dynamic-mpu")]
            bottom_half_stale: false,
            #[cfg(feature = "dynamic-mpu")]
            in_bottom_half: false,
            core,
            sync: C::Sync::default(),
            msg: C::Msg::default(),
            ports: C::Ports::default(),
            irq_bindings,
            sleep_queue: crate::waitqueue::SleepQueue::new(),
            _memory: PhantomData,
        }
    }

    /// Create a `Kernel` with empty partition table and schedule.
    ///
    /// Only available in test builds for backward-compatible test helpers.
    ///
    /// # Deprecation
    ///
    /// Use [`Kernel::new()`] instead.  `new_empty()` exists only for legacy
    /// test helpers; new tests should construct a fully-configured kernel
    /// via `Kernel::new()` with appropriate test fixtures.
    #[deprecated(since = "0.1.0", note = "use Kernel::new() instead")]
    #[cfg(test)]
    pub(crate) fn new_empty(
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
            ticks_dropped: 0,
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
            #[cfg(feature = "dynamic-mpu")]
            ticks_since_bottom_half: 0,
            #[cfg(feature = "dynamic-mpu")]
            bottom_half_stale: false,
            #[cfg(feature = "dynamic-mpu")]
            in_bottom_half: false,
            core: C::Core::default(),
            sync: C::Sync::default(),
            msg: C::Msg::default(),
            ports: C::Ports::default(),
            irq_bindings: &[],
            sleep_queue: crate::waitqueue::SleepQueue::new(),
            _memory: PhantomData,
        }
    }

    // -------------------------------------------------------------------------
    // Runtime alignment assertion
    // -------------------------------------------------------------------------

    /// Assert that `self` is properly aligned at runtime.
    ///
    /// This check catches linker script or manual placement errors that could
    /// misalign the kernel storage. Compile-time `align_of::<T>()` checks only
    /// verify type requirements, not whether the linker or manual placement
    /// actually honored those requirements for the specific memory symbol.
    ///
    /// In debug builds, this panics if the kernel instance is misaligned.
    /// In release builds, this compiles to nothing.
    #[inline(always)]
    pub fn debug_assert_self_aligned(&self) {
        debug_assert!(
            (self as *const Self as usize).is_multiple_of(core::mem::align_of::<Self>()),
            "Kernel<'mem, C> instance at {:p} is not aligned to {} bytes",
            self as *const Self,
            core::mem::align_of::<Self>()
        );
    }

    // -------------------------------------------------------------------------
    // User-pointer validation helpers
    // -------------------------------------------------------------------------

    /// Validate that `ptr..ptr+len` lies in memory accessible to the current
    /// partition's static MPU regions. Returns `Ok(())` on success or
    /// `Err(SvcError::InvalidPointer.to_u32())` on failure.
    #[inline(always)]
    pub fn check_user_ptr(&self, ptr: u32, len: usize) -> Result<(), u32> {
        validate_user_ptr(self.partitions(), self.current_partition, ptr, len)
            .then_some(())
            .ok_or(SvcError::InvalidPointer.to_u32())
    }

    /// Validate that `ptr..ptr+len` lies in memory accessible to the current
    /// partition, including dynamically-mapped MPU windows when `dynamic-mpu`
    /// is enabled. Falls back to [`check_user_ptr`](Self::check_user_ptr)
    /// when the feature is disabled.
    #[inline(always)]
    #[cfg(feature = "dynamic-mpu")]
    pub fn check_user_ptr_dynamic(&self, ptr: u32, len: usize) -> Result<(), u32> {
        validate_user_ptr_dynamic(
            self.partitions(),
            &self.dynamic_strategy,
            self.current_partition,
            ptr,
            len,
        )
        .then_some(())
        .ok_or(SvcError::InvalidPointer.to_u32())
    }

    /// Fallback: delegates to [`check_user_ptr`](Self::check_user_ptr) when
    /// `dynamic-mpu` is disabled.
    #[inline(always)]
    #[cfg(not(feature = "dynamic-mpu"))]
    pub fn check_user_ptr_dynamic(&self, ptr: u32, len: usize) -> Result<(), u32> {
        self.check_user_ptr(ptr, len)
    }

    // -------------------------------------------------------------------------
    // Facade methods delegating to self.sync (SyncPools)
    // -------------------------------------------------------------------------

    /// Returns a shared reference to the semaphore pool.
    #[inline(always)]
    pub fn semaphores(&self) -> &<C::Sync as SyncOps>::SemPool {
        self.debug_assert_self_aligned();
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

    /// Store a per-kernel IRQ binding table for `IrqAck` dispatch.
    pub fn store_irq_bindings(&mut self, bindings: &'static [crate::irq_dispatch::IrqBinding]) {
        self.irq_bindings = bindings;
    }

    /// Trigger a deschedule by setting `yield_requested` and invoking PendSV.
    ///
    /// This centralizes the deschedule logic that both `SYS_YIELD` and
    /// blocking syscalls need. Sets `self.yield_requested = true` so the
    /// harness can force-advance the schedule, then calls `handle_yield()`
    /// to pend the PendSV exception.
    ///
    /// The outgoing Running → Ready transition is intentionally **not**
    /// performed here.  It is deferred to `yield_current_slot()`, which
    /// only fires the transition after confirming a valid runnable partner
    /// exists.  Doing it eagerly here would leave the partition in Ready
    /// even when no context switch actually occurs, violating the invariant
    /// that exactly one partition is Running at all times.
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
    /// Calls `semaphores.tick_timeouts()`, `queuing.tick_timeouts()`, and
    /// `blackboards.tick_timeouts()` with the given tick, collecting all
    /// expired partition IDs into a single `heapless::Vec<u8, E>`, then
    /// transitions each from [`Waiting`](PartitionState::Waiting) to
    /// [`Ready`](PartitionState::Ready).
    ///
    /// With `dynamic-mpu`, also drains the device wait queue.
    ///
    /// The tick handler should call this once per tick so that blocked
    /// senders/receivers are woken when their timeout elapses.
    ///
    /// `E` must be large enough to hold the total number of expired partition
    /// IDs across all subsystems in a single tick. Callers should set `E` to
    /// at least `N` (max partitions) to guarantee no overflow; overflow is
    /// safe (expired entries are re-enqueued by `drain_expired`) but delays
    /// wakeup by one tick.
    // TODO: Consider tying E to the KernelConfig::N const so callers cannot
    // under-size the buffer. Currently E is caller-chosen.
    pub fn expire_timed_waits<const E: usize>(&mut self, current_tick: u64) {
        let mut expired: heapless::Vec<u8, E> = heapless::Vec::new();
        self.sync
            .semaphores_mut()
            .tick_timeouts(current_tick, &mut expired);
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
        // Drain expired sleep timers from the sorted sleep queue.
        let mut sleep_expired: heapless::Vec<u8, E> = heapless::Vec::new();
        self.sleep_queue
            .drain_expired(current_tick, &mut sleep_expired);
        for &pid in sleep_expired.iter() {
            if try_transition(self.core.partitions_mut(), pid, PartitionState::Ready) {
                if let Some(pcb) = self.core.partitions_mut().get_mut(pid as usize) {
                    pcb.set_sleep_until(0);
                }
            }
        }
        if !expired.is_empty() || !sleep_expired.is_empty() {
            self.yield_requested = true;
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
        use events as ev;
        // SAFETY: Snapshot all argument registers before the match writes
        // frame.r0 with the return value.  This prevents any theoretical
        // compiler reordering between reads of r1/r2/r3 and the r0 write,
        // since all field accesses happen before the mutable borrow for r0.
        let syscall_id = frame.r0;
        let arg1 = frame.r1;
        let arg2 = frame.r2;
        let arg3 = frame.r3;
        let caller = self.current_partition as usize;
        self.assert_dispatch_invariants();
        #[cfg(any(debug_assertions, test))]
        let entry_states = {
            let parts = self.core.partitions().as_slice();
            let mut buf = [crate::partition::PartitionState::Ready; C::N];
            for (dst, src) in buf[..parts.len()].iter_mut().zip(parts.iter()) {
                *dst = src.state();
            }
            (buf, parts.len())
        };
        frame.r0 = match SyscallId::from_u32(syscall_id) {
            Some(SyscallId::Yield) => self.trigger_deschedule(),
            Some(SyscallId::GetPartitionId) => caller as u32,
            Some(SyscallId::EventWait) => {
                let (result, block) = ev::handle_event_wait(self.partitions_mut(), caller, arg2);
                if block {
                    self.trigger_deschedule();
                }
                result
            }
            Some(SyscallId::EventSet) => {
                ev::handle_event_set(self.partitions_mut(), arg1 as usize, arg2)
            }
            Some(SyscallId::EventClear) => {
                ev::handle_event_clear(self.partitions_mut(), caller, arg2)
            }
            Some(SyscallId::SemWait) => {
                let pt = self.core.partitions_mut();
                let (r, block) =
                    sync::handle_sem_wait(self.sync.semaphores_mut(), pt, arg1 as usize, caller);
                if block {
                    self.trigger_deschedule();
                }
                r
            }
            Some(SyscallId::SemSignal) => {
                let pt = self.core.partitions_mut();
                sync::handle_sem_signal(self.sync.semaphores_mut(), pt, frame.r1 as usize)
            }
            Some(SyscallId::MutexLock) => {
                let pt = self.core.partitions_mut();
                let (r, block) =
                    sync::handle_mtx_lock(self.sync.mutexes_mut(), pt, arg1 as usize, caller);
                if block {
                    self.trigger_deschedule();
                }
                r
            }
            Some(SyscallId::MutexUnlock) => {
                let pt = self.core.partitions_mut();
                sync::handle_mtx_unlock(self.sync.mutexes_mut(), pt, frame.r1 as usize, caller)
            }
            #[cfg(feature = "ipc-message")]
            Some(SyscallId::MsgSend) => match self.check_user_ptr(arg3, C::QM) {
                Err(e) => e,
                Ok(()) => {
                    // SAFETY: check_user_ptr confirmed [r3, r3+QM) lies within
                    // the calling partition's MPU data region.
                    let (r, blk) = unsafe {
                        msg::handle_msg_send(
                            self.msg.messages_mut(),
                            self.core.partitions_mut(),
                            arg1 as usize,
                            arg2 as usize,
                            arg3 as *const u8,
                            C::QM,
                        )
                    }
                    .into_svc_return();
                    if blk {
                        self.trigger_deschedule();
                    }
                    r
                }
            },
            #[cfg(feature = "ipc-message")]
            Some(SyscallId::MsgRecv) => match self.check_user_ptr(arg3, C::QM) {
                Err(e) => e,
                Ok(()) => {
                    // SAFETY: check_user_ptr confirmed [r3, r3+QM) lies within
                    // the calling partition's MPU data region.
                    let (r, blk) = unsafe {
                        msg::handle_msg_recv(
                            self.msg.messages_mut(),
                            self.core.partitions_mut(),
                            arg1 as usize,
                            arg2 as usize,
                            arg3 as *mut u8,
                            C::QM,
                        )
                    }
                    .into_svc_return();
                    if blk {
                        self.trigger_deschedule();
                    }
                    r
                }
            },
            #[cfg(feature = "ipc-sampling")]
            Some(SyscallId::SamplingWrite) => {
                match self.check_user_ptr(frame.r3, frame.r2 as usize) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r3, r3+r2) lies within
                        // the calling partition's MPU data region. (2) Slice length
                        // is user-provided but validated against MPU bounds.
                        // (3) The partition owns this memory as enforced by MPU.
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
                    }
                }
            }
            #[cfg(feature = "ipc-sampling")]
            Some(SyscallId::SamplingRead) => match self.check_user_ptr(frame.r3, C::SM) {
                Err(e) => e,
                Ok(()) => {
                    let tick = self.tick.get();
                    // SAFETY: check_user_ptr confirmed [r3, r3+SM) is in-bounds.
                    let r = unsafe {
                        sampling::handle_sampling_read(
                            self.ports.sampling_mut(),
                            frame.r1 as usize,
                            frame.r3 as *mut u8,
                            C::SM,
                            tick,
                        )
                    };
                    match r {
                        Ok((sz, v)) => {
                            frame.r1 = v as u32;
                            sz
                        }
                        Err(code) => code,
                    }
                }
            },
            #[cfg(feature = "ipc-queuing")]
            Some(SyscallId::QueuingSend) => {
                match self.check_user_ptr(frame.r3, frame.r2 as usize) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r3, r3+r2) lies within
                        // the calling partition's MPU data region. (2) Slice length
                        // is user-provided but validated against MPU bounds.
                        // (3) The partition owns this memory as enforced by MPU.
                        let d = unsafe {
                            core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                        };
                        queuing::handle_queuing_send(
                            self.msg.queuing_mut(),
                            self.core.partitions_mut(),
                            self.current_partition,
                            self.tick.get(),
                            frame.r1 as usize,
                            d,
                        )
                    }
                }
            }
            #[cfg(feature = "ipc-queuing")]
            Some(SyscallId::QueuingRecv) => match self.check_user_ptr(frame.r3, C::QM) {
                Err(e) => e,
                Ok(()) => {
                    // SAFETY: (1) check_user_ptr confirmed [r3, r3+QM) lies within
                    // the calling partition's MPU data region. (2) Slice length is
                    // C::QM, a KernelConfig constant. (3) The partition owns this
                    // memory as enforced by MPU isolation.
                    let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                    queuing::handle_queuing_receive(
                        self.msg.queuing_mut(),
                        self.core.partitions_mut(),
                        self.current_partition,
                        self.tick.get(),
                        frame.r1 as usize,
                        b,
                    )
                }
            },
            #[cfg(feature = "ipc-queuing")]
            Some(SyscallId::QueuingStatus) => {
                match self.check_user_ptr(frame.r2, core::mem::size_of::<QueuingPortStatus>()) {
                    Err(e) => e,
                    Ok(())
                        if !(frame.r2 as usize)
                            .is_multiple_of(core::mem::align_of::<QueuingPortStatus>()) =>
                    {
                        SvcError::InvalidPointer.to_u32()
                    }
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r2, r2+size_of
                        // QueuingPortStatus) lies within the calling partition's MPU
                        // data region. (2) Alignment of r2 for QueuingPortStatus is
                        // verified by the guard on the preceding arm. (3) The partition
                        // owns this memory as enforced by MPU isolation.
                        unsafe {
                            queuing::handle_queuing_status(
                                self.msg.queuing(),
                                frame.r1 as usize,
                                frame.r2 as *mut QueuingPortStatus,
                            )
                        }
                    }
                }
            }
            #[cfg(feature = "ipc-blackboard")]
            Some(SyscallId::BbDisplay) => {
                match self.check_user_ptr(frame.r3, frame.r2 as usize) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r3, r3+r2) lies within
                        // the calling partition's MPU data region. (2) Slice length
                        // is user-provided but validated against MPU bounds.
                        // (3) The partition owns this memory as enforced by MPU.
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
                    }
                }
            }
            #[cfg(feature = "ipc-blackboard")]
            Some(SyscallId::BbRead) => {
                match self.check_user_ptr(frame.r3, C::BM) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r3, r3+BM) lies within
                        // the calling partition's MPU data region. (2) Slice length is
                        // C::BM, a KernelConfig constant. (3) The partition owns this
                        // memory as enforced by MPU isolation.
                        let b =
                            unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::BM) };
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
                    }
                }
            }
            #[cfg(feature = "ipc-blackboard")]
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
            Some(SyscallId::SleepTicks) => {
                // NOTE: self.core.partitions_mut() is intentional – split borrow
                // needed because self.sleep_queue is also mutably borrowed.
                let outcome = self::sleep::handle_sleep_ticks(
                    &mut self.sleep_queue,
                    self.core.partitions_mut(),
                    self.current_partition,
                    arg1,
                    self.tick.get(),
                );
                let (r, deschedule) = outcome.into_svc_return();
                if deschedule {
                    self.trigger_deschedule();
                }
                r
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferAlloc) => buf::handle_buf_alloc(
                &mut self.buffers,
                self.current_partition,
                frame.r1,
                frame.r2,
                self.tick.get(),
            ),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferRelease) => buf::handle_buf_release(
                &mut self.buffers,
                frame.r1 as usize,
                self.current_partition,
            ),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferLend) => {
                let pc = self.partition_count();
                let tick = self.tick.get();
                let (r0, r1_ov) = buffer::handle_buffer_lend(
                    &mut self.buffers,
                    &self.dynamic_strategy,
                    self.current_partition,
                    pc,
                    tick,
                    frame.r1,
                    frame.r2,
                    frame.r3,
                    LEGACY_TEST_FRAME_R3_SENTINEL,
                );
                if let Some(v) = r1_ov {
                    frame.r1 = v;
                }
                r0
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferRevoke) => {
                let slot = frame.r1 as usize;
                let target_raw = frame.r2 as usize;
                if target_raw >= self.partition_count() {
                    SvcError::InvalidPartition.to_u32()
                } else {
                    let target = target_raw as u8;
                    match self.buffers.unshare_from_partition(
                        slot,
                        self.current_partition,
                        target,
                        &self.dynamic_strategy,
                    ) {
                        Ok(()) => 0,
                        Err(e) => e.to_svc_error().to_u32(),
                    }
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferTransfer) => {
                let slot = frame.r1 as usize;
                let new_owner_raw = frame.r2 as usize;
                match u8::try_from(new_owner_raw) {
                    Ok(new_owner) if (new_owner as usize) < self.partition_count() => {
                        match self.buffers.transfer_ownership(
                            slot,
                            self.current_partition,
                            new_owner,
                        ) {
                            Ok(()) => 0,
                            Err(e) => e.to_svc_error().to_u32(),
                        }
                    }
                    _ => SvcError::InvalidPartition.to_u32(),
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferWrite) => {
                match self.check_user_ptr_dynamic(frame.r3, frame.r2 as usize) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: check_user_ptr_dynamic confirmed [r3, r3+r2) is in caller's MPU region.
                        unsafe {
                            buf::handle_buf_write(
                                &mut self.buffers,
                                frame.r1 as usize,
                                self.current_partition,
                                frame.r3 as *const u8,
                                frame.r2 as usize,
                            )
                        }
                    }
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::BufferRead) => {
                match self.check_user_ptr_dynamic(frame.r3, frame.r2 as usize) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: check_user_ptr_dynamic confirmed [r3, r3+r2) is in caller's MPU region.
                        unsafe {
                            buf::handle_buf_read(
                                &mut self.buffers,
                                frame.r1 as usize,
                                self.current_partition,
                                frame.r3 as *mut u8,
                                frame.r2 as usize,
                            )
                        }
                    }
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevOpen) => self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                dev.open(pid)?;
                Ok(0)
            }),
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevRead) => {
                let len = frame.r2 as usize;
                match self.check_user_ptr_dynamic(frame.r3, len) {
                    Err(e) => e,
                    Ok(()) => {
                        let buf_ptr = frame.r3 as *mut u8;
                        self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                            // SAFETY: check_user_ptr_dynamic confirmed [r3, r3+r2) lies
                            // within the calling partition's accessible memory regions.
                            let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, len) };
                            Ok(dev.read(pid, buf)? as u32)
                        })
                    }
                }
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevWrite) => {
                let len = frame.r2 as usize;
                match self.check_user_ptr_dynamic(frame.r3, len) {
                    Err(e) => e,
                    Ok(()) => {
                        let data_ptr = frame.r3 as *const u8;
                        self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                            // SAFETY: check_user_ptr_dynamic confirmed [r3, r3+r2) lies
                            // within the calling partition's accessible memory regions.
                            let data = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                            Ok(dev.write(pid, data)? as u32)
                        })
                    }
                }
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
                // r1 = device_id, r2 = (timeout_ticks << 16 | buf_len), r3 = buf_ptr
                // Canonical caller: plib::sys_dev_read_timed (plib/src/lib.rs) packs r2.
                let (timeout_ticks, buf_len) = unpack_packed_r2(frame.r2);
                let buf_len = buf_len as usize;
                match self.check_user_ptr_dynamic(frame.r3, buf_len) {
                    Err(e) => e,
                    Ok(()) => {
                        let buf_ptr = frame.r3 as *mut u8;
                        let timeout = timeout_ticks as u32;
                        let pid = self.current_partition;
                        let device_id = frame.r1 as u8;
                        match self.registry.get_mut(device_id) {
                            Some(dev) => {
                                // SAFETY: check_user_ptr_dynamic confirmed
                                // [r3, r3+buf_len) lies within the calling
                                // partition's accessible memory regions.
                                let buf =
                                    unsafe { core::slice::from_raw_parts_mut(buf_ptr, buf_len) };
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
                    }
                }
            }
            // TODO: QueryBottomHalf gated behind dynamic-mpu because underlying
            // ticks_since_bottom_half/is_bottom_half_stale are feature-gated. Future
            // refactor should make bottom-half health monitoring unconditional.
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::QueryBottomHalf) => {
                frame.r1 = u32::from(self.is_bottom_half_stale());
                self.ticks_since_bottom_half
            }
            #[cfg(feature = "ipc-queuing")]
            Some(SyscallId::QueuingRecvTimed) => {
                let (timeout_ticks, buf_len) = unpack_packed_r2(arg2);
                let rlen = core::cmp::min(buf_len as usize, C::QM);
                match self.check_user_ptr(arg3, rlen) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: (1) check_user_ptr confirmed [r3, r3+rlen) lies
                        // within the calling partition's MPU data region.
                        // (2) len clamped to C::QM. (3) The partition owns this
                        // memory as enforced by MPU isolation.
                        let b = unsafe { core::slice::from_raw_parts_mut(arg3 as *mut u8, rlen) };
                        let pid = self.current_partition;
                        let tick = self.tick.get();
                        match self.msg.queuing_mut().receive_queuing_message(
                            arg1 as usize,
                            pid,
                            b,
                            timeout_ticks as u64,
                            tick,
                        ) {
                            Ok(RecvQueuingOutcome::Received {
                                msg_len,
                                wake_sender: w,
                            }) => {
                                if let Some(wpid) = w {
                                    try_transition(
                                        self.core.partitions_mut(),
                                        wpid,
                                        PartitionState::Ready,
                                    );
                                }
                                msg_len as u32
                            }
                            Ok(RecvQueuingOutcome::ReceiverBlocked { .. }) => {
                                try_transition(
                                    self.core.partitions_mut(),
                                    pid,
                                    PartitionState::Waiting,
                                );
                                self.trigger_deschedule()
                            }
                            Err(_) => SvcError::InvalidResource.to_u32(),
                        }
                    }
                }
            }
            #[cfg(feature = "ipc-queuing")]
            Some(SyscallId::QueuingSendTimed) => {
                let (timeout_ticks, buf_len) = unpack_packed_r2(frame.r2);
                let data_len = buf_len as usize;
                let timeout = timeout_ticks as u64;
                match self.check_user_ptr(frame.r3, data_len) {
                    Err(e) => e,
                    Ok(()) => {
                        // SAFETY: check_user_ptr confirmed [r3, r3+data_len) lies within
                        // the calling partition's MPU data region.
                        let d =
                            unsafe { core::slice::from_raw_parts(frame.r3 as *const u8, data_len) };
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
                    }
                }
            }
            Some(SyscallId::DebugPrint) => {
                let (ptr, len) = (frame.r1, frame.r2 as usize);
                if !validate_user_ptr(self.partitions(), self.current_partition, ptr, len) {
                    SvcError::InvalidPointer.to_u32()
                } else {
                    // SAFETY: validate_user_ptr confirmed [ptr, ptr+len) in partition memory.
                    let data = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
                    debug::handle_debug_print(data)
                }
            }
            Some(SyscallId::DebugExit) => debug::handle_debug_exit(frame.r1),
            #[cfg(feature = "partition-debug")]
            Some(SyscallId::DebugNotify) => {
                let pid = self.current_partition as usize;
                debug::handle_debug_notify(self.partitions_mut(), pid)
            }
            #[cfg(feature = "partition-debug")]
            Some(SyscallId::DebugWrite) => {
                let (ptr, len) = (frame.r1, frame.r2 as usize);
                if !validate_user_ptr(self.partitions(), self.current_partition, ptr, len) {
                    SvcError::InvalidPointer.to_u32()
                } else {
                    let pid = self.current_partition as usize;
                    // SAFETY: validate_user_ptr confirmed [ptr, ptr+len) in partition memory.
                    let data = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
                    debug::handle_debug_write(self.partitions_mut(), pid, data)
                }
            }
            Some(SyscallId::IrqAck) => {
                let irq_num = arg1 as u8;
                let caller_id = self.current_partition;
                self::irq::handle_irq_ack(self.irq_bindings, caller_id, irq_num)
            }
            #[allow(unreachable_patterns)]
            Some(_) => SvcError::InvalidSyscall.to_u32(),
            None => SvcError::InvalidSyscall.to_u32(),
        };
        #[cfg(any(debug_assertions, test))]
        {
            let (states, len) = entry_states;
            crate::invariants::assert_waiting_implies_yield_requested(
                self.core.partitions().as_slice(),
                &states[..len],
                self.yield_requested,
            );
        }
        self.assert_dispatch_invariants();
    }

    /// Assert kernel invariants at dispatch entry/exit (no-op in release).
    #[cfg(any(debug_assertions, test))]
    fn assert_dispatch_invariants(&self) {
        // In real builds, verify linker-placed 4096-byte alignment. Skipped in
        // tests because Box only guarantees the type's natural alignment; the
        // test harness checks type alignment separately.
        #[cfg(not(test))]
        crate::invariants::assert_storage_alignment(
            self as *const Self as usize,
            crate::state::KERNEL_ALIGNMENT,
        );
        // Unit tests may leave active_partition as None; in the real kernel
        // it is always Some when an SVC fires.  Skip the check only in tests.
        #[cfg(test)]
        if self.active_partition.is_none() {
            return;
        }
        let parts = self.core.partitions().as_slice();
        let pid = self.current_partition as usize;
        // yield_requested ⇒ active_partition/next_partition may be stale.
        let active = if self.yield_requested
            && parts.get(pid).map(|p| p.state()) != Some(PartitionState::Running)
        {
            None
        } else {
            self.active_partition
        };
        let mut sem_pairs = [(0u32, 0u32); C::S];
        let mut sem_len = 0;
        for (i, pair) in sem_pairs.iter_mut().enumerate() {
            match self.sync.semaphores().get(i) {
                Some(s) => {
                    *pair = (s.count(), s.max_count());
                    sem_len = i + 1;
                }
                None => break,
            }
        }
        let next = if self.yield_requested {
            None
        } else {
            Some(self.core.next_partition())
        };
        let sp = match self.core.partition_sp().get(..parts.len()) {
            Some(s) => s,
            None => return, // mismatched lengths — skip rather than panic
        };
        crate::invariants::assert_kernel_invariants(parts, active, &sem_pairs[..sem_len], next, sp);
    }

    #[cfg(not(any(debug_assertions, test)))]
    #[inline(always)]
    fn assert_dispatch_invariants(&self) {}

    /// Increment starvation counters for all Ready partitions that are not
    /// the currently active partition. Called when a schedule slot is wasted
    /// on a Waiting partition.
    pub fn increment_starvation_for_ready_partitions(&mut self) {
        let active = self.active_partition;
        for pcb in self.partitions_mut().iter_mut() {
            if Some(pcb.id()) == active {
                continue;
            }
            if pcb.state() == PartitionState::Ready {
                pcb.increment_starvation();
                if pcb.is_starved() {
                    crate::klog!(
                        "partition {} starved (count={})",
                        pcb.id(),
                        pcb.starvation_count()
                    );
                }
            }
        }
    }

    /// Updates the PCB stack region fields for a partition.
    ///
    /// Returns `true` if the partition exists and the region was updated,
    /// `false` if the partition index is out of bounds or the MPU validation
    /// fails.
    ///
    /// Syncs the PCB stack_limit into the mirrored `partition_stack_limits`
    /// array for PendSV overflow pre-check.
    #[inline(always)]
    pub fn sync_stack_limit(&mut self, index: usize) -> bool {
        if let Some(pcb) = self.core.partition_slice().get(index) {
            let limit = pcb.stack_limit();
            if let Some(slot) = self.core.partition_stack_limits_mut().get_mut(index) {
                *slot = limit;
                return true;
            }
        }
        false
    }

    /// Updates the PCB MPU data region base for a partition.
    ///
    /// Returns `true` if the partition exists and was updated, `false` if the
    /// partition index is out of bounds.
    #[inline(always)]
    pub fn fix_mpu_data_region(&mut self, index: usize, base: u32) -> bool {
        if let Some(pcb) = self.core.partitions_mut().get_mut(index) {
            pcb.fix_mpu_data_region(base);
            true
        } else {
            false
        }
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

    #[cfg(feature = "dynamic-mpu")]
    pub(crate) fn mpu_tick_bookkeeping(&mut self, system_window: bool) {
        if system_window {
            self.ticks_since_bottom_half = 0;
            self.bottom_half_stale = false;
        } else {
            self.ticks_since_bottom_half = self.ticks_since_bottom_half.saturating_add(1);
            if self.ticks_since_bottom_half > C::SYSTEM_WINDOW_MAX_GAP_TICKS {
                self.bottom_half_stale = true;
            }
        }
    }

    /// Force-advance the schedule to the next slot, forfeiting remaining
    /// ticks. Updates `active_partition` and returns the schedule result.
    /// Called by the harness when a partition yields.
    pub fn yield_current_slot(&mut self) -> impl YieldResult {
        #[allow(unused_variables)]
        let (result, skipped) = self.schedule_mut().force_advance_to_partition();
        #[cfg(feature = "dynamic-mpu")]
        if skipped > 0 {
            self.ticks_since_bottom_half = 0;
            self.bottom_half_stale = false;
        }
        if let Some(pid) = result.partition_id() {
            let is_waiting = self
                .pcb(pid as usize)
                .map(|pcb| pcb.state() == PartitionState::Waiting)
                .unwrap_or(false);
            if is_waiting {
                self.increment_starvation_for_ready_partitions();
                // Bug 06: restore active partition to Running if it is Waiting.
                // Note: `is_waiting` checks `pid` (next scheduled), not `ap`
                // (active partition) — they can differ in multi-partition
                // schedules, so this guard is intentionally not redundant.
                if let Some(ap) = self.active_partition {
                    if self
                        .pcb(ap as usize)
                        .is_some_and(|p| p.state() == PartitionState::Waiting)
                        && try_transition(self.partitions_mut(), ap, PartitionState::Ready)
                    {
                        self.set_next_partition(ap);
                    }
                }
                return ScheduleEvent::None;
            }
            svc_scheduler::transition_outgoing_ready(self);
            // Reset starvation: the incoming partition is now running.
            if let Some(pcb) = self.pcb_mut(pid as usize) {
                pcb.reset_starvation();
            }
            self.active_partition = Some(pid);
        } else {
            // Bug 05: force_advance returned no partition (empty/unstarted
            // schedule or SystemWindow slot). If the active partition is
            // Waiting, restore it to Running so it remains schedulable.
            // Transition chain: Waiting → Ready → Running (no direct path).
            // active_partition is intentionally left unchanged here — the
            // schedule advanced past system-only windows but no partition was
            // found, so we keep the current partition active.
            if let Some(ap) = self.active_partition {
                if try_transition(self.partitions_mut(), ap, PartitionState::Ready) {
                    let _ = try_transition(self.partitions_mut(), ap, PartitionState::Running);
                }
            }
            return ScheduleEvent::None;
        }
        result
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

    /// Returns whether bottom-half processing is stale.
    ///
    /// This flag is set to `true` when `ticks_since_bottom_half` exceeds
    /// `C::SYSTEM_WINDOW_MAX_GAP_TICKS`, indicating that the system window
    /// has not been reached within the expected time. The flag is cleared
    /// when a `SystemWindow` event occurs.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn is_bottom_half_stale(&self) -> bool {
        self.bottom_half_stale
    }

    /// Clears the bottom-half stale diagnostic flag.
    ///
    /// This allows external monitoring code to reset the flag after observing
    /// and logging a stale condition, enabling detection of subsequent stale
    /// events.
    #[cfg(feature = "dynamic-mpu")]
    #[inline(always)]
    pub fn clear_bottom_half_stale(&mut self) {
        self.bottom_half_stale = false;
    }

    /// Returns the current value of the ticks-dropped counter.
    #[inline(always)]
    pub fn ticks_dropped(&self) -> u32 {
        self.ticks_dropped
    }

    /// Increments the ticks-dropped counter by 1 (saturating).
    #[inline(always)]
    pub fn increment_ticks_dropped(&mut self) {
        self.ticks_dropped = self.ticks_dropped.saturating_add(1);
    }

    /// Atomically resets the ticks-dropped counter to zero, returning the
    /// previous value.
    #[inline(always)]
    pub fn reset_ticks_dropped(&mut self) -> u32 {
        let prev = self.ticks_dropped;
        self.ticks_dropped = 0;
        prev
    }

    /// Safety-critical fallback: revoke expired buffer lending deadlines.
    ///
    /// When `bottom_half_stale` is true (system windows are too infrequent),
    /// this method enforces buffer deadlines by revoking any buffer slots
    /// whose deadline has passed. Only buffer revocation runs as fallback —
    /// UART transfer and ISR ring draining are not safety-critical and are
    /// deferred to the next system window.
    ///
    /// Returns the number of buffers revoked, or 0 if the stale flag is not set.
    #[cfg(feature = "dynamic-mpu")]
    pub fn fallback_revoke_expired_buffers(&mut self) -> usize {
        if !self.bottom_half_stale {
            return 0;
        }
        let current_tick = self.tick.get();
        self.buffers
            .revoke_expired(current_tick, &self.dynamic_strategy)
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

    /// Drains debug output from all partitions with pending debug data.
    ///
    /// Iterates through all partitions, and for each with `debug_pending=true`,
    /// calls `drain_partition` with the given budget. The budget is applied
    /// per-partition, not globally.
    ///
    /// Returns the total number of bytes drained across all partitions.
    ///
    // TODO: Real-time determinism - worst-case execution time scales linearly
    // with partition count. Consider adding a global aggregate budget cap for
    // low-latency scenarios.
    #[cfg(feature = "partition-debug")]
    pub fn drain_debug_pending(
        &mut self,
        ctx: &mut crate::partition_debug::DrainContext,
        budget: usize,
    ) -> usize {
        let mut total = 0;
        for pcb in self.partitions_mut().iter_mut() {
            if pcb.debug_pending() {
                total += ctx.drain_partition(pcb, budget);
            }
        }
        total
    }

    /// Drain pending debug output using the budget from `KernelConfig`.
    ///
    /// When `partition-debug` is enabled and `DEBUG_AUTO_DRAIN_BUDGET > 0`,
    /// creates a stack-local `DrainContext` and forwards to
    /// `drain_debug_pending`. Otherwise compiles to nothing (zero-cost).
    pub fn drain_debug_auto(&mut self) {
        #[cfg(feature = "partition-debug")]
        {
            if C::DEBUG_AUTO_DRAIN_BUDGET > 0 {
                let mut ctx = crate::partition_debug::DrainContext::new();
                self.drain_debug_pending(&mut ctx, C::DEBUG_AUTO_DRAIN_BUDGET);
            }
        }
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

#[cfg(test)]
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

#[cfg(test)]
fn apply_recv_outcome<'mem, C: KernelConfig>(
    kernel: &mut Kernel<'mem, C>,
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
#[allow(clippy::undocumented_unsafe_blocks, deprecated)]
mod tests {
    // Facade methods for `active_partition`, `current_partition`, `yield_requested`,
    // `buffers`, `dev_wait_queue`, and `hw_uart` are now available on Kernel.
    // Tests should use these accessor methods instead of direct field access.

    //! # SAFETY — Test Dispatch Justification
    //!
    //! All `unsafe { k.dispatch(&mut ef) }` calls in this test module share
    //! the same safety justification:
    //!
    //! ## Test Isolation
    //!
    //! Each `#[test]` function creates its own `Kernel` and `ExceptionFrame`
    //! instances on the stack. No global state is shared between tests, so
    //! tests cannot interfere with each other's kernel or partition state.
    //!
    //! ## Single-Threaded Execution
    //!
    //! Rust's default test runner executes tests in a single-threaded manner
    //! (unless `--test-threads=N` is specified, which these tests do not
    //! rely on). Even with parallel test execution, each test has isolated
    //! stack-local state. The `dispatch()` function is not re-entrant, but
    //! since each test owns its own `Kernel` instance, concurrent test
    //! execution does not cause data races.
    //!
    //! ## Valid ExceptionFrame Construction
    //!
    //! The `frame()` test helper constructs `ExceptionFrame` instances with
    //! valid register values. Unlike hardware exception entry, these are not
    //! actual stacked registers, but the dispatch logic only reads/writes
    //! the r0-r3 fields which are always initialized. The remaining fields
    //! (r12, lr, pc, xpsr) are set to zero, which is safe because dispatch
    //! does not use them.
    //!
    //! ## Kernel Construction
    //!
    //! The `kernel()` and `kernel_with_registry()` helpers construct `Kernel`
    //! instances with properly initialized partition tables (via `tbl()`),
    //! schedule tables, and resource pools. All partitions have valid MPU
    //! regions and are transitioned to Running state before dispatch.
    //!
    //! ## Host-Mode Pointer Validation
    //!
    //! Tests run on the host (not target hardware) where `validate_user_ptr`
    //! checks pass for any pointer within the partition's configured MPU
    //! region. Tests that exercise pointer validation use `mmap` to allocate
    //! memory at addresses matching the partition's MPU region, ensuring
    //! the kernel's bounds checks succeed.

    mod helpers;
    pub use helpers::*;

    mod time;

    mod macro_config;

    mod irq;

    pub use super::*;
    pub use crate::config::KernelConfig;
    pub use crate::kernel_config_types;
    pub use crate::message::MessageQueue;
    pub use crate::mpu::MpuError;
    #[cfg(feature = "dynamic-mpu")]
    pub use crate::mpu_strategy::MpuStrategy;
    pub use crate::partition::{ExternalPartitionMemory, MpuRegion, PartitionControlBlock};
    pub use crate::partition_core::AlignedStack1K;
    pub use crate::scheduler::ScheduleEntry;
    pub use crate::scheduler::ScheduleEvent;
    pub use crate::semaphore::Semaphore;
    pub use crate::syscall::SYS_GET_PARTITION_ID;
    pub use crate::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_IRQ_ACK, SYS_YIELD};

    // ---- Kernel::new() is the canonical constructor ----

    #[test]
    fn kernel_new_is_canonical_constructor() {
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 5)).unwrap();
        schedule.start();

        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mpu0 = MpuRegion::new(0x2000_0000, 4096, 0);
        let mpu1 = MpuRegion::new(0x2000_1000, 4096, 0);
        let m0 = ExternalPartitionMemory::new(&mut stk0.0, 0x0800_0001, mpu0, 0).unwrap();
        let m1 = ExternalPartitionMemory::new(&mut stk1.0, 0x0800_1001, mpu1, 1).unwrap();
        let k = Kernel::<TestConfig>::new(schedule, &[m0, m1]).unwrap();
        assert_eq!(k.partitions().len(), 2);
        assert_eq!(k.partitions().get(0).unwrap().entry_point(), 0x0800_0001);
        assert_eq!(k.partitions().get(1).unwrap().entry_point(), 0x0800_1001);
    }

    #[test]
    fn kernel_new_forwards_code_mpu_region() {
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.start();

        let mut stk0 = AlignedStack1K::default();
        let data_region = MpuRegion::new(0x2000_0000, 4096, 0);
        let code_region = MpuRegion::new(0x0800_0000, 8192, 0);
        let m0 = ExternalPartitionMemory::new(&mut stk0.0, 0x0800_0001, data_region, 0)
            .unwrap()
            .with_code_mpu_region(code_region)
            .unwrap();
        let k = Kernel::<TestConfig>::new(schedule, &[m0]).unwrap();
        let pcb = k.partitions().get(0).unwrap();
        let got = pcb
            .code_mpu_region()
            .expect("code_mpu_region should be Some");
        assert_eq!(got.base(), 0x0800_0000);
        assert_eq!(got.size(), 8192);
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
    ) -> Kernel<'static, TestConfig> {
        kernel_impl(sem_count, mtx_count, msg_queue_count, registry)
    }

    /// Build a Kernel with 2 running partitions, the given semaphore count,
    /// mutex count, and message queue count.
    ///
    /// When `dynamic-mpu` is enabled the kernel is constructed with a
    /// [`DeviceRegistry`] pre-populated with virtual UART backends for
    /// device IDs 0 and 1.
    fn kernel(
        sem_count: usize,
        mtx_count: usize,
        msg_queue_count: usize,
    ) -> Kernel<'static, TestConfig> {
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
    ) -> Kernel<'static, TestConfig> {
        // Build the core with pre-populated partitions.
        let mut core = <TestConfig as KernelConfig>::Core::default();
        let pt = tbl();
        for pcb in pt.iter() {
            core.partitions_mut().add(pcb.clone()).unwrap();
        }
        let mut k: Kernel<'static, TestConfig> = Kernel {
            active_partition: None,
            tick: TickCounter::new(),
            current_partition: 0,
            yield_requested: false,
            ticks_dropped: 0,
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
            #[cfg(feature = "dynamic-mpu")]
            ticks_since_bottom_half: 0,
            #[cfg(feature = "dynamic-mpu")]
            bottom_half_stale: false,
            #[cfg(feature = "dynamic-mpu")]
            in_bottom_half: false,
            core,
            sync: <TestConfig as KernelConfig>::Sync::default(),
            msg: <TestConfig as KernelConfig>::Msg::default(),
            ports: <TestConfig as KernelConfig>::Ports::default(),
            irq_bindings: &[],
            sleep_queue: crate::waitqueue::SleepQueue::new(),
            _memory: PhantomData,
        };
        // Add semaphores via facade method
        for _ in 0..sem_count {
            k.semaphores_mut().add(Semaphore::new(1, 2)).unwrap();
        }
        // MutexPool is pre-allocated at full capacity by SyncPools::default();
        // no add() API exists, so mtx_count is unused here.
        let _ = mtx_count;
        // Add message queues via facade method
        for _ in 0..msg_queue_count {
            k.messages_mut().add(MessageQueue::new()).unwrap();
        }
        k
    }

    /// Safe wrapper for `Kernel::dispatch` in test code.
    ///
    /// Consolidates the repeated `unsafe { k.dispatch(ef) }` pattern behind
    /// a safe interface. The safety justification is documented once here
    /// rather than at each call site.
    fn dispatch_checked(k: &mut Kernel<'static, TestConfig>, ef: &mut ExceptionFrame) {
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(ef) }
    }

    /// Dispatch a 3-register SVC and return `r0`.
    ///
    /// Builds an [`ExceptionFrame`] from the given register values, dispatches
    /// it through `dispatch_checked`, and returns the resulting `r0`.
    fn dispatch_r0(k: &mut Kernel<'static, TestConfig>, r0: u32, r1: u32, r2: u32) -> u32 {
        let mut ef = frame(r0, r1, r2);
        dispatch_checked(k, &mut ef);
        ef.r0
    }

    /// Dispatch a 3-register SVC and return `(r0, r1)`.
    // TODO: consider whether dynamic-mpu feature-gating on these helpers is
    // too aggressive — callers outside dynamic-mpu tests may need them.
    #[cfg(feature = "dynamic-mpu")]
    fn dispatch_r01(k: &mut Kernel<'static, TestConfig>, r0: u32, r1: u32, r2: u32) -> (u32, u32) {
        let mut ef = frame(r0, r1, r2);
        dispatch_checked(k, &mut ef);
        (ef.r0, ef.r1)
    }

    /// Dispatch a 4-register SVC and return `r0`.
    ///
    /// Like [`dispatch_r0`] but passes all four argument registers.
    #[cfg(feature = "dynamic-mpu")]
    fn dispatch_r04(
        k: &mut Kernel<'static, TestConfig>,
        r0: u32,
        r1: u32,
        r2: u32,
        r3: u32,
    ) -> u32 {
        let mut ef = frame4(r0, r1, r2, r3);
        dispatch_checked(k, &mut ef);
        ef.r0
    }

    /// Register a new partition and transition it to `Running`.
    ///
    /// Useful for tests that need additional partitions beyond the default P0/P1
    /// created by [`kernel()`].
    #[cfg(feature = "dynamic-mpu")]
    fn add_running_partition(k: &mut Kernel<'static, TestConfig>, id: u8) {
        k.partitions_mut().add(pcb(id)).unwrap();
        k.partitions_mut()
            .get_mut(id as usize)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
    }

    // -------------------------------------------------------------------------
    // EXC_RETURN guard constant tests
    // -------------------------------------------------------------------------

    #[test]
    fn svc_exc_return_guard_matches_context_constant() {
        use crate::context::EXC_RETURN_THREAD_PSP;

        // The assembly trampoline hardcodes 0xFFFFFFFD via movw/movt.
        // Verify the context module constant matches the expected value.
        assert_eq!(EXC_RETURN_THREAD_PSP, 0xFFFF_FFFD);

        // Bit 0 set: return to Thread mode (not Handler mode).
        assert_ne!(
            EXC_RETURN_THREAD_PSP & (1 << 0),
            0,
            "bit 0 (Thread mode) must be set"
        );

        // Bit 2 set: restore context from PSP (not MSP).
        assert_ne!(
            EXC_RETURN_THREAD_PSP & (1 << 2),
            0,
            "bit 2 (PSP) must be set"
        );

        // Bit 1 clear: no FPU context (basic frame).
        assert_eq!(
            EXC_RETURN_THREAD_PSP & (1 << 4),
            1 << 4,
            "bit 4 (no FPU stacking) must be set for basic frame"
        );

        // Bits [31:4] must all be ones (EXC_RETURN magic prefix).
        assert_eq!(
            EXC_RETURN_THREAD_PSP & 0xFFFF_FFF0,
            0xFFFF_FFF0,
            "bits [31:4] must all be ones"
        );
    }

    // -------------------------------------------------------------------------
    // Runtime alignment assertion tests
    // -------------------------------------------------------------------------

    #[test]
    fn debug_assert_self_aligned_does_not_panic_for_aligned_kernel() {
        // Create a kernel on the stack - Rust guarantees proper alignment.
        let k = kernel(0, 0, 0);
        // This should not panic since the kernel is properly aligned.
        k.debug_assert_self_aligned();
    }

    #[test]
    fn debug_assert_self_aligned_verifies_runtime_address() {
        use core::mem::align_of;
        let k = kernel(0, 0, 0);
        // Verify the actual address is aligned (same check the method performs).
        let addr = &k as *const _ as usize;
        let required = align_of::<Kernel<'static, TestConfig>>();
        assert!(
            addr.is_multiple_of(required),
            "Kernel at 0x{:x} not aligned to {} bytes",
            addr,
            required
        );
        // The method should succeed for this aligned instance.
        k.debug_assert_self_aligned();
    }

    #[test]
    fn yield_returns_zero_and_preserves_regs() {
        let mut ef = frame(SYS_YIELD, 0xAA, 0xBB);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!((ef.r0, ef.r1, ef.r2, ef.r3), (0, 0xAA, 0xBB, 0xCC));
    }

    #[test]
    fn get_partition_id_returns_caller() {
        let mut t = tbl();
        for caller in [0usize, 1] {
            let mut ef = frame(SYS_GET_PARTITION_ID, 0, 0);
            dispatch_syscall(&mut ef, &mut t, caller);
            assert_eq!(ef.r0, caller as u32);
        }
    }

    #[test]
    fn yield_sets_yield_requested_flag() {
        let mut k = kernel(0, 0, 0);
        k.active_partition = Some(0);
        // Fix invariant: only P0 should be Running.
        try_transition(k.partitions_mut(), 1, PartitionState::Ready);
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
        k.active_partition = Some(0);
        // Fix invariant: only P0 should be Running.
        try_transition(k.partitions_mut(), 1, PartitionState::Ready);
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert!(k.yield_requested());
        k.set_yield_requested(false);
        assert!(!k.yield_requested());
        // Restore P0 to Running (yield transitioned it to Ready).
        try_transition(k.partitions_mut(), 0, PartitionState::Running);
        // Non-yield syscall does not set the flag
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert!(!k.yield_requested());
    }

    #[test]
    fn trigger_deschedule_sets_yield_requested() {
        let mut k = kernel(0, 0, 0);
        k.active_partition = Some(0);
        // Fix invariant: only P0 should be Running.
        try_transition(k.partitions_mut(), 1, PartitionState::Ready);
        assert!(!k.yield_requested());
        let ret = k.trigger_deschedule();
        assert_eq!(ret, 0);
        assert!(k.yield_requested());
    }

    #[test]
    fn trigger_deschedule_preserves_running_state() {
        let mut k = kernel_with_schedule();

        // Put P0 into Running and mark it as active.
        k.set_next_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        k.active_partition = Some(0);

        let ret = k.trigger_deschedule();

        assert_eq!(ret, 0);
        assert!(k.yield_requested());
        // The partition must still be Running — the transition to Ready
        // is deferred to yield_current_slot(), not done eagerly here.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn trigger_deschedule_noop_for_waiting_partition() {
        let mut k = kernel_with_schedule();

        // Put P0 into Running, then transition to Waiting.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        let ret = k.trigger_deschedule();

        assert_eq!(ret, 0);
        assert!(k.yield_requested());
        // Partition remains Waiting — transition_outgoing_ready is a no-op.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    /// Build a single-partition Kernel whose only variable is `mpu_region`.
    fn kernel_with_mpu_region(
        region: MpuRegion,
    ) -> Result<Kernel<'static, TestConfig>, ConfigError> {
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stk, 0x0800_0001, region, 0)?;
        Kernel::<TestConfig>::new(schedule, core::slice::from_ref(&mem))
    }

    /// Kernel::new() succeeds when a partition uses mpu_region size==0
    /// as a sentinel for "no user-configured data region".
    #[test]
    fn kernel_new_accepts_zero_size_mpu_region_sentinel() {
        let kernel = kernel_with_mpu_region(MpuRegion::new(0, 0, 0))
            .expect("Kernel::new() should accept mpu_region size==0 sentinel");
        let pcb = kernel.partitions().get(0).expect("partition 0 must exist");

        assert_eq!(
            pcb.mpu_region().base(),
            0,
            "sentinel mpu_region base must equal config-provided value (0)"
        );
        assert_eq!(
            pcb.mpu_region().size(),
            0,
            "sentinel mpu_region size must remain 0"
        );
    }

    /// Kernel::new() rejects a partition with non-zero but invalid mpu_region
    /// (e.g. size=17, not a power of two).
    #[test]
    fn kernel_new_rejects_invalid_nonzero_mpu_region() {
        let err = kernel_with_mpu_region(MpuRegion::new(0, 17, 0))
            .err()
            .expect("Kernel::new() should reject non-zero invalid mpu_region");
        assert!(
            matches!(
                err,
                ConfigError::MpuRegionInvalid {
                    partition_id: 0,
                    detail: MpuError::SizeTooSmall,
                }
            ),
            "Expected MpuRegionInvalid with SizeTooSmall, got: {:?}",
            err,
        );
    }

    /// Kernel::new() rejects a partition whose mpu_region has a valid
    /// power-of-two size but a base address that is not aligned to that size.
    #[test]
    fn kernel_new_rejects_misaligned_user_mpu_region() {
        // base=64 is not aligned to size=256 (64 & 255 != 0).
        let err = kernel_with_mpu_region(MpuRegion::new(64, 256, 0))
            .err()
            .expect("Kernel::new() should reject misaligned mpu_region base");
        assert!(
            matches!(
                err,
                ConfigError::MpuRegionInvalid {
                    partition_id: 0,
                    detail: MpuError::BaseNotAligned,
                }
            ),
            "Expected MpuRegionInvalid with BaseNotAligned, got: {:?}",
            err,
        );
    }

    /// Helper: build a single-partition kernel via `new`.
    fn mem_kernel(entry: u32, mpu: MpuRegion) -> Result<Kernel<'static, TestConfig>, ConfigError> {
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let mut stk = AlignedStack1K::default();
        let m = ExternalPartitionMemory::from_aligned_stack(&mut stk, entry, mpu, 0)?;
        Kernel::<TestConfig>::new(sched, &[m])
    }

    #[test]
    fn mem_kernel_pcb_fields_and_sentinels() {
        let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
        let k = mem_kernel(0x0800_1001, mpu).expect("should succeed");
        let pcb = k.partitions().get(0).expect("partition 0 must exist");
        assert_eq!(pcb.id(), 0);
        assert_eq!(pcb.entry_point(), 0x0800_1001);
        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 1024);
        assert_ne!(pcb.stack_base(), 0, "stack_base populated by Kernel::new");
        assert_ne!(pcb.stack_size(), 0, "stack_size populated by Kernel::new");
        // Sentinel mpu_region (size==0) also accepted
        let k2 = mem_kernel(1, MpuRegion::new(0, 0, 0)).unwrap();
        assert_eq!(k2.partitions().get(0).unwrap().mpu_region().size(), 0);
    }

    #[test]
    fn mem_kernel_validates_mpu_and_schedule() {
        // Invalid MPU region rejected
        let err = mem_kernel(1, MpuRegion::new(0, 17, 0))
            .err()
            .expect("should fail");
        assert!(matches!(
            err,
            ConfigError::MpuRegionInvalid {
                partition_id: 0,
                ..
            }
        ));
        // Empty schedule rejected
        let mut stk = AlignedStack1K::default();
        let m =
            ExternalPartitionMemory::from_aligned_stack(&mut stk, 1, MpuRegion::new(0, 0, 0), 0)
                .unwrap();
        let err = Kernel::<TestConfig>::new(ScheduleTable::<4>::new(), &[m]);
        assert!(matches!(err, Err(ConfigError::ScheduleEmpty)));
    }

    // ---- Kernel::new() with ExternalPartitionMemory unit tests ----

    /// Helper: build a single-partition kernel via `new`.
    fn ext_kernel(entry: u32, mpu: MpuRegion) -> Result<Kernel<'static, TestConfig>, ConfigError> {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let mut stack = AlignedStack256B::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, entry, mpu, 0)?;
        Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
    }

    #[test]
    fn kernel_new_pcb_fields_and_sentinels() {
        let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
        let k = ext_kernel(0x0800_1001, mpu).expect("should succeed");
        let pcb = k.partitions().get(0).expect("partition 0 must exist");
        assert_eq!(pcb.id(), 0);
        assert_eq!(pcb.entry_point(), 0x0800_1001);
        assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
        assert_eq!(pcb.mpu_region().size(), 1024);
        assert_ne!(pcb.stack_base(), 0, "stack_base populated by Kernel::new");
        assert_eq!(pcb.stack_size(), 256, "stack_size from AlignedStack256B");
        assert!(
            pcb.peripheral_regions().is_empty(),
            "no peripheral regions configured"
        );
    }

    #[test]
    fn kernel_new_validates_schedule() {
        use crate::partition_core::AlignedStack256B;
        let mut stack = AlignedStack256B::default();
        let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 1, mpu, 0).unwrap();
        let err = Kernel::<TestConfig>::new(ScheduleTable::<4>::new(), core::slice::from_ref(&mem));
        assert!(matches!(err, Err(ConfigError::ScheduleEmpty)));
    }

    #[test]
    fn kernel_new_two_partitions_ids_from_indices() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 5)).unwrap();
        sched.add(ScheduleEntry::new(1, 5)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
        let mut stack0 = AlignedStack256B::default();
        let mut stack1 = AlignedStack256B::default();
        let m0 =
            ExternalPartitionMemory::from_aligned_stack(&mut stack0, 0x0800_0001, mpu, 0).unwrap();
        let m1 =
            ExternalPartitionMemory::from_aligned_stack(&mut stack1, 0x0800_1001, mpu, 1).unwrap();
        let k = Kernel::<TestConfig>::new(sched, &[m0, m1]).expect("two partitions should succeed");
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        assert_eq!(k.partitions().get(0).unwrap().entry_point(), 0x0800_0001);
        assert_eq!(k.partitions().get(1).unwrap().id(), 1);
        assert_eq!(k.partitions().get(1).unwrap().entry_point(), 0x0800_1001);
    }

    #[test]
    fn kernel_new_partition_count_limit() {
        use crate::partition_core::AlignedStack256B;
        let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
        // Build 5 ExternalPartitionMemory descriptors — exceeds TestConfig::N (4).
        let mut stacks: [AlignedStack256B; 5] = [AlignedStack256B::default(); 5];
        let mut mems: heapless::Vec<ExternalPartitionMemory<'_>, 5> = heapless::Vec::new();
        for (i, s) in stacks.iter_mut().enumerate() {
            let m = ExternalPartitionMemory::from_aligned_stack(s, 1, mpu, i as u8).unwrap();
            mems.push(m).unwrap();
        }
        // Schedule only needs to reference partition 0 — the failure is in
        // the config vec push, not schedule validation.
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let err = Kernel::<TestConfig>::new(sched, &mems);
        assert!(matches!(err, Err(ConfigError::PartitionTableFull)));
    }

    // ---- Kernel::new() code_mpu_region validation tests ----

    /// Invalid code_mpu_region is now rejected at builder time
    /// (with_code_mpu_region), so Kernel::new() never sees it.
    #[test]
    fn kernel_new_rejects_invalid_code_mpu_region() {
        use crate::partition_core::AlignedStack256B;
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let bad_code = MpuRegion::new(0x0800_0000, 100, 0); // 100 is not power-of-two
        let mut stack = AlignedStack256B::default();
        let err = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0001, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(bad_code)
            .unwrap_err();
        assert!(
            matches!(
                err,
                ConfigError::CodeRegionInvalid {
                    partition_id: 0,
                    detail: MpuError::SizeNotPowerOfTwo,
                }
            ),
            "Expected CodeRegionInvalid, got: {:?}",
            err,
        );
    }

    /// Kernel::new() accepts a partition with a valid code_mpu_region.
    #[test]
    fn kernel_new_accepts_valid_code_mpu_region() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let code_mpu = MpuRegion::new(0x0800_0000, 8192, 0);
        let mut stack = AlignedStack256B::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0001, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(code_mpu)
            .unwrap();
        let k = Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
            .expect("valid code_mpu_region should be accepted");
        let pcb = k.partitions().get(0).expect("partition 0 must exist");
        let got = pcb.code_mpu_region().expect("code region should be set");
        assert_eq!(got.base(), 0x0800_0000);
        assert_eq!(got.size(), 8192);
    }

    /// Kernel::new() succeeds when no code_mpu_region is configured.
    #[test]
    fn kernel_new_accepts_no_code_mpu_region() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let mut stack = AlignedStack256B::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0001, data_mpu, 0)
            .unwrap();
        let k = Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
            .expect("no code_mpu_region should be accepted");
        let pcb = k.partitions().get(0).expect("partition 0 must exist");
        assert!(
            pcb.code_mpu_region().is_none(),
            "code region should be None when not configured"
        );
    }

    // ---- Entry-point-in-code-region validation tests ----
    // TODO: These tests now exercise ExternalPartitionMemory::with_code_mpu_region()
    // directly. Consider adding back Kernel::new()-level integration tests to ensure
    // the Kernel path also rejects invalid entry points with the correct partition_id,
    // guarding against cases where the builder validation could be bypassed.

    #[test]
    fn kernel_new_rejects_entry_below_code_region() {
        use crate::partition_core::AlignedStack256B;
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let code_mpu = MpuRegion::new(0x0800_1000, 4096, 0);
        let mut stack = AlignedStack256B::default();
        let err = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0001, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(code_mpu)
            .expect_err("should reject entry below code region");
        assert!(matches!(
            err,
            ConfigError::EntryPointOutsideCodeRegion {
                partition_id: 0,
                entry_point: 0x0800_0001,
                region_base: 0x0800_1000,
                region_size: 4096,
            }
        ));
    }

    #[test]
    fn kernel_new_rejects_entry_at_code_region_end() {
        use crate::partition_core::AlignedStack256B;
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let code_mpu = MpuRegion::new(0x0800_0000, 4096, 0);
        let mut stack = AlignedStack256B::default();
        let err = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_1001, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(code_mpu)
            .expect_err("should reject entry at code region end");
        assert!(matches!(
            err,
            ConfigError::EntryPointOutsideCodeRegion {
                partition_id: 0,
                entry_point: 0x0800_1001,
                region_base: 0x0800_0000,
                region_size: 4096,
            }
        ));
    }

    #[test]
    fn kernel_new_accepts_entry_within_code_region() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let code_mpu = MpuRegion::new(0x0800_0000, 8192, 0);
        let mut stack = AlignedStack256B::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0101, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(code_mpu)
            .unwrap();
        let k = Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
            .expect("entry within code region should be accepted");
        assert_eq!(k.partitions().get(0).unwrap().entry_point(), 0x0800_0101);
    }

    #[test]
    fn kernel_new_strips_thumb_bit_for_code_region_check() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let code_mpu = MpuRegion::new(0x0800_0000, 8192, 0);
        let mut stack = AlignedStack256B::default();
        // entry 0x0800_0101 has Thumb bit; effective = 0x0800_0100
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0800_0101, data_mpu, 0)
            .unwrap()
            .with_code_mpu_region(code_mpu)
            .unwrap();
        let k = Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
            .expect("entry with Thumb bit within code region should be accepted");
        assert_eq!(k.partitions().get(0).unwrap().entry_point(), 0x0800_0101);
    }

    #[test]
    fn kernel_new_skips_entry_check_when_no_code_region() {
        use crate::partition_core::AlignedStack256B;
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let data_mpu = MpuRegion::new(0x2000_0000, 1024, 0);
        let mut stack = AlignedStack256B::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(&mut stack, 0x0FFF_0001, data_mpu, 0)
            .unwrap();
        Kernel::<TestConfig>::new(sched, core::slice::from_ref(&mem))
            .expect("no code region means no entry point bounds check");
    }

    #[test]
    fn invalid_syscall_returns_error_code() {
        let mut ef = frame(0xFFFF, 0, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, SvcError::InvalidSyscall.to_u32());
    }

    #[test]
    fn event_wait_dispatches_to_events_module() {
        let mut t = tbl();
        ev_data::event_set(&mut t, 0, 0b1010);
        let mut ef = frame(SYS_EVT_WAIT, 0xDEADBEEF, 0b1110);
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, 0b1010);
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
    }

    #[test]
    fn event_set_dispatches_to_events_module() {
        let mut t = tbl();
        let mut ef = frame(SYS_EVT_SET, 1, 0b0101);
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, 0);
        assert_eq!(t.get(1).unwrap().event_flags(), 0b0101);
    }

    #[test]
    fn event_clear_dispatches_to_events_module() {
        let mut t = tbl();
        ev_data::event_set(&mut t, 0, 0b1111);
        let mut ef = frame(SYS_EVT_CLEAR, 0xDEADBEEF, 0b0101);
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, 0b1111, "event_clear must return previous flags");
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1010);
    }

    #[test]
    fn event_invalid_partition_returns_error_code() {
        let inv = SvcError::InvalidPartition.to_u32();
        let mut t = tbl();
        let mut ef = frame(SYS_EVT_WAIT, 0xDEADBEEF, 0b0001);
        dispatch_syscall(&mut ef, &mut t, 99);
        assert_eq!(ef.r0, inv);
        let mut ef = frame(SYS_EVT_SET, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, inv);
        let mut ef = frame(SYS_EVT_CLEAR, 0xDEADBEEF, 0b0001);
        dispatch_syscall(&mut ef, &mut t, 99);
        assert_eq!(ef.r0, inv);
    }

    // ---- dispatch_syscall IrqAck tests ----
    // dispatch_syscall has no binding table; IrqAck always returns
    // InvalidResource directly.

    #[test]
    fn dispatch_syscall_irq_ack_success() {
        use crate::syscall::SYS_IRQ_ACK;
        let mut ef = frame(SYS_IRQ_ACK, 5, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn dispatch_syscall_irq_ack_wrong_partition() {
        use crate::syscall::SYS_IRQ_ACK;
        let mut ef = frame(SYS_IRQ_ACK, 5, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 1);
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn dispatch_syscall_irq_ack_kernel_clears_rejected() {
        use crate::syscall::SYS_IRQ_ACK;
        let mut ef = frame(SYS_IRQ_ACK, 20, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn dispatch_syscall_irq_ack_missing_binding() {
        use crate::syscall::SYS_IRQ_ACK;
        let mut ef = frame(SYS_IRQ_ACK, 99, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t, 0);
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[test]
    fn dispatch_event_wait_blocking_triggers_deschedule() {
        let mut k = kernel(0, 0, 0);
        // No bits set — event_wait should block (return 0) and trigger deschedule.
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
        assert!(
            k.yield_requested(),
            "blocking EventWait must trigger deschedule"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "blocked partition must be in Waiting state"
        );
    }

    #[test]
    fn dispatch_event_wait_immediate_no_deschedule() {
        let mut k = kernel(0, 0, 0);
        // Pre-set bits so event_wait returns immediately with matched bits.
        ev_data::event_set(k.partitions_mut(), 0, 0b1010);
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1110);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0b1010);
        assert!(!k.yield_requested());
    }

    #[test]
    fn dispatch_event_wait_blocking_saves_wait_mask() {
        let mut k = kernel(0, 0, 0);
        // No bits set — EventWait should block and save the wait mask in the PCB.
        let mask = 0b1100_0011;
        let mut ef = frame(SYS_EVT_WAIT, 0, mask);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocked EventWait must return 0");
        assert!(
            k.yield_requested(),
            "blocking EventWait must trigger deschedule"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "blocked partition must be Waiting"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().event_wait_mask(),
            mask,
            "PCB must save the wait mask for wake-up"
        );
    }

    /// Confused-deputy regression: EventWait must use current_partition, not r1.
    /// If r1=1 but current_partition=0, the wait must operate on partition 0.
    #[test]
    fn event_wait_uses_current_partition_not_r1() {
        let mut k = kernel(0, 0, 0);
        // r1=1 (attacker tries to operate on partition 1), mask in r2
        let mut ef = frame(SYS_EVT_WAIT, 1, 0b0011);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        // current_partition=0 has no bits set, so it blocks
        assert_eq!(ef.r0, 0, "EventWait must block on partition 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "partition 0 must be Waiting (not partition 1)"
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running,
            "partition 1 must be unaffected"
        );
    }

    /// Confused-deputy regression: EventClear must use current_partition, not r1.
    /// If r1=1 but current_partition=0, the clear must operate on partition 0.
    #[test]
    fn event_clear_uses_current_partition_not_r1() {
        let mut k = kernel(0, 0, 0);
        // Pre-set bits on both partitions
        ev_data::event_set(k.partitions_mut(), 0, 0b1111);
        ev_data::event_set(k.partitions_mut(), 1, 0b1111);
        // r1=1 (attacker tries to clear partition 1's flags), mask in r2
        let mut ef = frame(SYS_EVT_CLEAR, 1, 0b0101);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0b1111, "EventClear must return previous flags");
        assert_eq!(
            k.partitions().get(0).unwrap().event_flags(),
            0b1010,
            "partition 0 flags must be cleared (not partition 1)"
        );
        assert_eq!(
            k.partitions().get(1).unwrap().event_flags(),
            0b1111,
            "partition 1 flags must be unaffected"
        );
    }

    /// EventSet must still route to the target in r1 (by design — you signal
    /// another partition). Verify r1=1 targets partition 1.
    #[test]
    fn event_set_still_targets_r1() {
        let mut k = kernel(0, 0, 0);
        // current_partition=0, r1=1 → should set flags on partition 1
        let mut ef = frame(SYS_EVT_SET, 1, 0b0110);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "EventSet must succeed");
        assert_eq!(
            k.partitions().get(1).unwrap().event_flags(),
            0b0110,
            "partition 1 must have the flags set via r1"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().event_flags(),
            0,
            "partition 0 must be unaffected"
        );
    }

    /// Confused-deputy regression: SemWait must use kernel-derived `caller`,
    /// not user-supplied r2. If r2=1 but current_partition=0, sem blocks partition 0.
    #[test]
    fn sem_wait_uses_caller_not_r2() {
        // 1 semaphore with count=0 so wait will block.
        let mut k = kernel(1, 0, 0);
        // Drain the semaphore: count starts at 1, first wait acquires.
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "first SemWait must acquire (count was 1)");

        // Now count=0. Attacker sets r2=1 to impersonate partition 1.
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 1);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "SemWait must block (count=0)");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "partition 0 must be Waiting (not partition 1)"
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running,
            "partition 1 must be unaffected"
        );
    }

    /// Confused-deputy regression: MutexLock must use kernel-derived `caller`,
    /// not user-supplied r2. If r2=1 but current_partition=0, mutex is owned by partition 0.
    #[test]
    fn mutex_lock_uses_caller_not_r2() {
        let mut k = kernel(0, 1, 0);
        // Attacker sets r2=1 to impersonate partition 1.
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 1);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "MutexLock must acquire immediately");
        assert_eq!(
            k.mutexes().owner(0),
            Ok(Some(0)),
            "mutex must be owned by partition 0 (not partition 1)"
        );
    }

    /// Confused-deputy regression: MutexUnlock must use kernel-derived `caller`,
    /// not user-supplied r2. If r2=1 but current_partition=0, unlock acts as partition 0.
    #[test]
    fn mutex_unlock_uses_caller_not_r2() {
        let mut k = kernel(0, 1, 0);
        // Partition 0 acquires the mutex.
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "MutexLock must acquire");
        assert_eq!(k.mutexes().owner(0), Ok(Some(0)));

        // Attacker sets r2=1 to impersonate partition 1 during unlock.
        let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 1);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "MutexUnlock must succeed for partition 0");
        assert_eq!(
            k.mutexes().owner(0),
            Ok(None),
            "mutex must be unlocked by partition 0 (not rejected as not-owner)"
        );
    }

    #[test]
    fn sem_wait_and_signal_dispatch() {
        let mut k = kernel(1, 0, 0);
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1);
        let mut ef = frame(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[test]
    fn dispatch_sem_wait_blocking_triggers_deschedule() {
        let mut k = kernel(1, 0, 0);
        // First wait: semaphore has count=1, so this acquires immediately (Ok(true)).
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1);
        assert!(!k.yield_requested());

        // Second wait: count is now 0, so partition 0 blocks (Ok(false)).
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert!(k.yield_requested());
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    #[test]
    fn dispatch_sem_wait_targets_correct_index_via_r1() {
        // Create 3 semaphores (indices 0, 1, 2) each with count=1, max=2.
        let mut k = kernel(3, 0, 0);
        // Dispatch SemWait with r1=2 to target semaphore index 2.
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 2, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        // r0=1 means acquired immediately (Ok(true)).
        assert_eq!(ef.r0, 1, "SemWait must return 1 (acquired)");
        // Semaphore 2 should have count decremented from 1 to 0.
        assert_eq!(k.semaphores().get(2).unwrap().count(), 0);
        // Semaphores 0 and 1 must remain untouched at count=1.
        assert_eq!(k.semaphores().get(0).unwrap().count(), 1);
        assert_eq!(k.semaphores().get(1).unwrap().count(), 1);
    }

    #[test]
    fn mutex_lock_unlock_dispatch() {
        let mut k = kernel(0, 1, 0);
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1); // 1 = acquired immediately
        assert_eq!(k.mutexes().owner(0), Ok(Some(0)));
        let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(k.mutexes().owner(0), Ok(None));
    }

    #[test]
    fn dispatch_mutex_lock_blocking_triggers_deschedule() {
        let mut k = kernel(0, 1, 0);
        // Partition 0 acquires mutex 0 immediately.
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "uncontested MutexLock must return 1 (acquired)");
        assert!(!k.yield_requested());

        // Switch to partition 1 and attempt to lock the same mutex.
        k.set_current_partition(1);
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 1);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking MutexLock must return 0");
        assert!(
            k.yield_requested(),
            "blocking MutexLock must trigger deschedule"
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "blocked partition must be in Waiting state"
        );
    }

    #[test]
    fn dispatch_mutex_lock_immediate_no_deschedule() {
        let mut k = kernel(0, 1, 0);
        // Mutex 0 is free — MutexLock should acquire immediately without deschedule.
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "immediate MutexLock must return 1 (acquired)");
        assert!(
            !k.yield_requested(),
            "immediate MutexLock must not trigger deschedule"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "acquiring partition must stay Running"
        );
        assert_eq!(
            k.mutexes().owner(0),
            Ok(Some(0)),
            "mutex must be owned by acquiring partition"
        );
    }

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
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
        // test Kernel with valid partition tables.
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
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
        // test Kernel with valid partition tables.
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
    fn read_dispatch_hook() -> Option<SvcDispatchFn> {
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
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields. The dispatch hook is set to
        // `sentinel_hook` which safely writes to r0.
        unsafe { dispatch_svc(&mut ef) };
        assert_eq!(ef.r0, 0xCAFE);
        clear_dispatch_hook();
    }

    #[test]
    fn dispatch_svc_falls_through_without_hook() {
        let _guard = HOOK_TEST_MUTEX.lock().unwrap();
        clear_dispatch_hook();
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields. No hook is installed, so
        // dispatch_svc falls through to the default Yield handler.
        unsafe { dispatch_svc(&mut ef) };
        // Without a hook, Yield returns 0.
        assert_eq!(ef.r0, 0);

        // Event and IRQ syscalls must return InvalidSyscall in the fallback path.
        let expected = SvcError::InvalidSyscall.to_u32();
        for &sys_id in &[SYS_EVT_WAIT, SYS_EVT_SET, SYS_EVT_CLEAR, SYS_IRQ_ACK] {
            let mut ef = frame(sys_id, 0, 0);
            // SAFETY: same as above — valid stack-allocated ExceptionFrame,
            // no hook installed.
            unsafe { dispatch_svc(&mut ef) };
            assert_eq!(
                ef.r0, expected,
                "syscall {sys_id} should return InvalidSyscall without hook"
            );
        }
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
        SvcError::PermissionDenied,
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
                // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed
                // by `frame()` with initialized r0-r3 fields; `k` is a properly
                // initialized test Kernel with valid partition tables.
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
        assert_eq!(
            svc!(SYS_BUF_RELEASE, 1),
            SvcError::PermissionDenied.to_u32()
        ); // wrong owner
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_alloc_sets_deadline_from_r2() {
        use crate::syscall::SYS_BUF_ALLOC;
        let mut k = kernel(0, 0, 0);

        // r2=0 → no deadline
        let mut ef = frame(SYS_BUF_ALLOC, 1, 0);
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
        // test Kernel with valid partition tables.
        unsafe { k.dispatch(&mut ef) };
        let slot0 = ef.r0 as usize;
        assert_eq!(slot0, 0);
        assert_eq!(k.buffers().deadline(slot0), None);

        // r2=100 → deadline = tick + 100; tick starts at 0 ⇒ deadline=100
        let mut ef = frame(SYS_BUF_ALLOC, 0, 100);
        // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
        // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
        // test Kernel with valid partition tables.
        unsafe { k.dispatch(&mut ef) };
        let slot1 = ef.r0 as usize;
        assert_eq!(slot1, 1);
        assert_eq!(k.buffers().deadline(slot1), Some(100));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_revoke_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let eop = SvcError::OperationFailed.to_u32();
        let epart = SvcError::InvalidPartition.to_u32();
        let mut k = kernel(0, 0, 0);
        // Allocate a writable buffer (mode=1) as partition 0
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Lend to partition 1 and verify r0=base_addr, r1=region_id
        let expected_addr = k.buffers().slot_base_address(slot as usize).unwrap();
        {
            let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
            assert_eq!(
                base_addr, expected_addr,
                "r0 should contain buffer base address"
            );
            // TODO: reviewer asked for `< 0x8000_0000` but host-test addresses
            // are truncated 64-bit pointers and may have bit 31 set; use
            // from_u32 to confirm the value is not a known SvcError instead.
            assert!(
                SvcError::from_u32(base_addr).is_none(),
                "r0 must not be a known SvcError"
            );
            k.dynamic_strategy
                .slot(region_id as u8)
                .expect("r1 region_id must be a valid dynamic MPU slot");
        }
        // Revoke from partition 1
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1), 0);
        // Error: revoke when not lent → OperationFailed (NotLent)
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1), eop);
        // Error: invalid partition (only 2 exist); r2 bits[7:0]=99
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_LEND, slot, 99), epart);
        // r2=0x100 → target=0 (self), flags=WRITABLE → SelfLend → OperationFailed
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_LEND, slot, 256), eop);
        // r2=0x101 → target=1, flags=WRITABLE → writable lend succeeds
        {
            let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 257);
            assert!(
                base_addr < 0x8000_0000,
                "writable lend r0 must be a valid address"
            );
            k.dynamic_strategy
                .slot(region_id as u8)
                .expect("writable lend r1 must be a valid dynamic MPU slot");
        }
        // Revoke the writable lend before further tests
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1), 0);
        // Error: self-lend (partition 0 lending to itself) → OperationFailed
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_LEND, slot, 0), eop);
        // Error: invalid partition in BufferRevoke
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 99), epart);
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 256), epart);
    }

    /// SYS_BUF_LEND must reject r2 values with reserved bits 9-15 set.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_reserved_bits_rejected() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND};
        let eop = SvcError::OperationFailed.to_u32();
        let mut k = kernel(0, 0, 0);
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");

        // (1) bit 9 set alone → target=1, reserved bit 9 set
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_LEND, slot, 0x0201), eop);

        // (2) all reserved bits set (0xFE00) → target=1
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_LEND, slot, 0xFE01), eop);

        // (3) valid WRITABLE flag (0x100) with target=1 still works
        {
            let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 0x0101);
            assert!(
                base_addr < 0x8000_0000,
                "writable lend with no reserved bits should succeed"
            );
            k.dynamic_strategy
                .slot(region_id as u8)
                .expect("r1 must be a valid dynamic MPU slot");
        }
    }

    /// SYS_BUF_LEND r3 deadline-ticks: r3=0 → no deadline, r3>0 → deadline
    /// set to current_tick + r3, and lend deadline overrides alloc deadline.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_deadline_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let mut k = kernel(0, 0, 0);
        k.sync_tick(100);
        // Allocate a writable buffer as partition 0 (no alloc deadline: r2=0)
        let slot_r0 = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot_r0 < 0x8000_0000, "alloc should succeed");
        let slot = slot_r0 as usize;

        // (1) r3=0 → lend sets no deadline
        {
            let mut ef = frame4(SYS_BUF_LEND, slot as u32, 1, 0);
            dispatch_checked(&mut k, &mut ef);
            assert!(ef.r0 < 0x8000_0000, "lend should succeed");
            k.dynamic_strategy
                .slot(ef.r1 as u8)
                .expect("r1 must be a valid dynamic MPU slot");
            assert_eq!(
                k.buffers().deadline(slot),
                None,
                "r3=0 must not set deadline"
            );
            // Revoke so we can re-lend
            assert_eq!(
                dispatch_r0(&mut k, SYS_BUF_REVOKE, slot as u32, 1),
                0,
                "revoke should succeed"
            );
        }

        // (2) r3=500 → deadline = tick(100) + 500 = 600
        {
            let mut ef = frame4(SYS_BUF_LEND, slot as u32, 1, 500);
            dispatch_checked(&mut k, &mut ef);
            assert!(ef.r0 < 0x8000_0000, "lend should succeed");
            k.dynamic_strategy
                .slot(ef.r1 as u8)
                .expect("r1 must be a valid dynamic MPU slot");
            assert_eq!(
                k.buffers().deadline(slot),
                Some(600),
                "deadline should be current_tick(100) + 500"
            );
            assert_eq!(dispatch_r0(&mut k, SYS_BUF_REVOKE, slot as u32, 1), 0);
        }

        // (3) Lend deadline overrides prior alloc-time deadline.
        //     Clear existing deadline, set one via alloc pattern, then override via lend.
        k.buffers.set_deadline(slot, Some(9999)).unwrap();
        assert_eq!(k.buffers().deadline(slot), Some(9999));
        {
            let mut ef = frame4(SYS_BUF_LEND, slot as u32, 1, 200);
            dispatch_checked(&mut k, &mut ef);
            assert!(ef.r0 < 0x8000_0000, "lend should succeed");
            k.dynamic_strategy
                .slot(ef.r1 as u8)
                .expect("r1 must be a valid dynamic MPU slot");
            assert_eq!(
                k.buffers().deadline(slot),
                Some(300),
                "lend deadline (100+200=300) must override prior alloc deadline (9999)"
            );
        }
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_transfer_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_TRANSFER};
        let eperm = SvcError::PermissionDenied.to_u32();
        let epart = SvcError::InvalidPartition.to_u32();
        let eop = SvcError::OperationFailed.to_u32();
        let mut k = kernel(0, 0, 0);
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0); // writable buffer, partition 0
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Successful transfer: partition 0 → partition 1
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 1), 0);
        // Wrong owner: partition 0 no longer owns it → PermissionDenied
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 1), eperm);
        // Transfer back via partition 1
        k.current_partition = 1;
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 0), 0);
        k.current_partition = 0;
        // Self-transfer → OperationFailed
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 0), eop);
        // Transfer while lent → OperationFailed (AlreadyLent)
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 1), eop);
        // Invalid partition ID → InvalidPartition
        assert_eq!(dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 99), epart);
    }

    /// Verify that an unauthorized third partition (P2) cannot revoke a lend
    /// it does not own, nor transfer a buffer it does not own.  Also checks
    /// that the authorized owner (P0) can still revoke after the failed attempt,
    /// ensuring no state corruption occurred.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_unauthorized_revoke_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE, SYS_BUF_TRANSFER};
        let eperm = SvcError::PermissionDenied.to_u32();
        let mut k = kernel(0, 0, 0);
        add_running_partition(&mut k, 2);

        // P0 allocates a writable buffer
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(!SvcError::is_error(slot), "alloc should succeed");
        // P0 lends to P1
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");

        // P2 tries to revoke P0→P1 lend — must get PermissionDenied
        k.current_partition = 2;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            eperm,
            "P2 revoking P0's lend must fail with PermissionDenied"
        );
        // P2 tries to transfer P0's buffer — must get PermissionDenied
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_TRANSFER, slot, 1),
            eperm,
            "P2 transferring P0's buffer must fail with PermissionDenied"
        );

        // Authorized owner (P0) can still revoke — no state corruption
        k.current_partition = 0;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "P0 should still be able to revoke its own lend"
        );
    }

    /// Verify that the lendee (P1) cannot re-lend a buffer it was lent by P0.
    /// Also checks that the owner (P0) can still revoke after the failed
    /// re-lend attempt.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lendee_cannot_relend_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let eperm = SvcError::PermissionDenied.to_u32();
        let mut k = kernel(0, 0, 0);
        add_running_partition(&mut k, 2);

        // P0 allocates a writable buffer
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(!SvcError::is_error(slot), "alloc should succeed");
        // P0 lends to P1
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");

        // P1 tries to re-lend the same slot to P2 — must get PermissionDenied
        k.current_partition = 1;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_LEND, slot, 2),
            eperm,
            "lendee P1 re-lending to P2 must fail with PermissionDenied"
        );

        // Owner (P0) can still revoke — no state corruption from failed re-lend
        k.current_partition = 0;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "P0 should still be able to revoke after P1's failed re-lend"
        );
    }

    /// Verify one-level lend depth enforcement via `dispatch_r0`:
    /// the lendee (P1) cannot re-lend a borrowed buffer back to P0 or forward
    /// to a third partition (P2).
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn target_cannot_relend_shared_buffer() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let eperm = SvcError::PermissionDenied.to_u32();
        let eres = SvcError::InvalidResource.to_u32();
        let epart = SvcError::InvalidPartition.to_u32();
        let mut k = kernel(0, 0, 0);
        add_running_partition(&mut k, 2);

        // P0 allocates and lends to P1
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");

        // Switch to lendee P1
        k.current_partition = 1;
        // Sanity: invalid slot → InvalidResource (not PermissionDenied)
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_LEND, 99, 0),
            eres,
            "invalid slot must return InvalidResource"
        );
        // Sanity: out-of-bounds partition → InvalidPartition
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_LEND, slot, 99),
            epart,
            "out-of-bounds partition must return InvalidPartition"
        );

        // P1 tries to re-lend back to P0 — must fail with ownership check
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_LEND, slot, 0),
            eperm,
            "lendee P1 re-lending back to owner P0 must fail with PermissionDenied"
        );

        // P1 tries to re-lend to a third partition P2 — must also fail
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_LEND, slot, 2),
            eperm,
            "lendee P1 re-lending to P2 must fail with PermissionDenied"
        );

        // Owner can still revoke — no state corruption
        k.current_partition = 0;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "P0 must still revoke after P1's failed re-lend attempts"
        );
    }

    /// Verify that the lendee (P1) cannot revoke a buffer it was lent by P0.
    /// Only the owner may revoke a lend.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn target_cannot_revoke_shared_buffer() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let eperm = SvcError::PermissionDenied.to_u32();
        let eres = SvcError::InvalidResource.to_u32();
        let epart = SvcError::InvalidPartition.to_u32();
        let mut k = kernel(0, 0, 0);

        // P0 allocates and lends to P1
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");

        // Switch to lendee P1
        k.current_partition = 1;
        // Sanity: invalid slot → InvalidResource (not PermissionDenied)
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, 99, 1),
            eres,
            "invalid slot must return InvalidResource"
        );
        // Sanity: out-of-bounds partition → InvalidPartition
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 99),
            epart,
            "out-of-bounds partition must return InvalidPartition"
        );

        // P1 tries to revoke — must fail with ownership check
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            eperm,
            "lendee P1 revoking P0's lend must fail with PermissionDenied"
        );

        // Owner can still revoke — lend state intact
        k.current_partition = 0;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "P0 must still revoke after P1's failed revoke attempt"
        );
    }

    /// Verify that the lendee (P1) cannot release a buffer owned by P0.
    /// Only the owner may release, and only when the buffer is not lent.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn target_cannot_release_shared_buffer() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_RELEASE, SYS_BUF_REVOKE};
        let eperm = SvcError::PermissionDenied.to_u32();
        let eres = SvcError::InvalidResource.to_u32();
        let mut k = kernel(0, 0, 0);

        // P0 allocates and lends to P1
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");

        // Switch to lendee P1
        k.current_partition = 1;
        // Sanity: invalid slot → InvalidResource (not PermissionDenied)
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_RELEASE, 99, 0),
            eres,
            "invalid slot must return InvalidResource"
        );

        // P1 tries to release — must fail (not owner)
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_RELEASE, slot, 0),
            eperm,
            "lendee P1 releasing P0's buffer must fail with PermissionDenied"
        );

        // Owner can revoke and then release — no state corruption
        k.current_partition = 0;
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "P0 must still revoke after P1's failed release"
        );
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_RELEASE, slot, 0),
            0,
            "P0 must release after revoking"
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_READ};
        let eres = SvcError::InvalidResource.to_u32();
        let eperm = SvcError::PermissionDenied.to_u32();
        let mut k = kernel(0, 0, 0);
        macro_rules! svc {
            ($r0:expr, $r1:expr, $r2:expr, $r3:expr) => {{
                let mut ef = frame4($r0, $r1, $r2, $r3);
                unsafe { k.dispatch(&mut ef) }; // SAFETY: see module docs
                ef.r0
            }};
        }
        let slot = svc!(SYS_BUF_ALLOC, 1, 0, 0); // alloc writable buffer
        assert!(slot < 0x8000_0000, "alloc should succeed");
        let pat: [u8; 32] = core::array::from_fn(|i| i as u8);
        k.buffers_mut()
            .get_mut(slot as usize)
            .unwrap()
            .data_mut()
            .copy_from_slice(&pat);
        let ptr = low32_buf(0);
        assert_eq!(svc!(SYS_BUF_READ, slot, 32, ptr as u32), 32);
        // SAFETY: ptr valid for 4096 bytes (mmap), 32 written.
        let out = unsafe { core::slice::from_raw_parts(ptr, 32) };
        assert_eq!(out, &pat, "read data must match written pattern");
        k.current_partition = 1; // wrong owner → PermissionDenied
        assert_eq!(svc!(SYS_BUF_READ, slot, 32, low32_buf(1) as u32), eperm);
        k.current_partition = 0; // invalid slot
        assert_eq!(svc!(SYS_BUF_READ, 99, 32, ptr as u32), eres);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_write_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_WRITE};
        let eres = SvcError::InvalidResource.to_u32();
        let eperm = SvcError::PermissionDenied.to_u32();
        let mut k = kernel(0, 0, 0);
        macro_rules! svc {
            ($r0:expr, $r1:expr, $r2:expr, $r3:expr) => {{
                let mut ef = frame4($r0, $r1, $r2, $r3);
                unsafe { k.dispatch(&mut ef) }; // SAFETY: see module docs
                ef.r0
            }};
        }
        // Allocate a writable buffer (r1=1 → BorrowedWrite)
        let slot = svc!(SYS_BUF_ALLOC, 1, 0, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Write a known pattern via SYS_BUF_WRITE
        let pat: [u8; 32] = core::array::from_fn(|i| (i as u8).wrapping_add(0xA0));
        let ptr = low32_buf(0);
        // SAFETY: ptr valid for 4096 bytes (mmap), writing 32.
        unsafe { core::ptr::copy_nonoverlapping(pat.as_ptr(), ptr, 32) };
        assert_eq!(svc!(SYS_BUF_WRITE, slot, 32, ptr as u32), 32);
        // Verify backing data matches
        let backing = k.buffers().get(slot as usize).unwrap().data();
        assert_eq!(
            &backing[..32],
            &pat,
            "backing data must match written pattern"
        );
        // Wrong owner → PermissionDenied
        k.current_partition = 1;
        assert_eq!(svc!(SYS_BUF_WRITE, slot, 32, low32_buf(1) as u32), eperm);
        // Invalid slot → InvalidResource
        k.current_partition = 0;
        assert_eq!(svc!(SYS_BUF_WRITE, 99, 32, ptr as u32), eres);
        // BorrowedRead slot → PermissionDenied (not write-owner)
        let rd_slot = svc!(SYS_BUF_ALLOC, 0, 0, 0); // r1=0 → BorrowedRead
        assert!(rd_slot < 0x8000_0000, "read alloc should succeed");
        assert_eq!(svc!(SYS_BUF_WRITE, rd_slot, 32, ptr as u32), eperm);
    }

    /// SYS_BUF_WRITE and SYS_BUF_READ must reject out-of-bounds pointers
    /// with `SvcError::InvalidPointer`.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buffer_syscalls_reject_out_of_bounds_pointer() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_READ, SYS_BUF_WRITE};
        let eptr = SvcError::InvalidPointer.to_u32();
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_BUF_ALLOC, 1, 0); // alloc writable
        unsafe { k.dispatch(&mut ef) }; // SAFETY: see module docs
        let slot = ef.r0;
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // BUF_WRITE with out-of-bounds pointer
        let mut ef = frame4(SYS_BUF_WRITE, slot, 4, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) }; // SAFETY: see module docs
        assert_eq!(ef.r0, eptr, "BufWrite should reject out-of-bounds pointer");
        // BUF_READ with out-of-bounds pointer
        let mut ef = frame4(SYS_BUF_READ, slot, 4, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) }; // SAFETY: see module docs
        assert_eq!(ef.r0, eptr, "BufRead should reject out-of-bounds pointer");
    }

    /// Lend with r2=1 (no WRITABLE flag) must produce an AP_RO_RO window.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_readonly_dispatch() {
        use crate::mpu::{decode_rasr_ap, AP_RO_RO};
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let mut k = kernel(0, 0, 0);
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Lend read-only: r2 = 1 (target=1, no WRITABLE flag)
        let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!(
            base_addr < 0x8000_0000,
            "r0 should contain valid base address"
        );
        let desc = k
            .dynamic_strategy
            .slot(region_id as u8)
            .expect("window descriptor must exist after lend");
        assert_eq!(
            decode_rasr_ap(desc.permissions),
            AP_RO_RO,
            "read-only lend must set AP_RO_RO"
        );
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "revoke must succeed"
        );
    }

    /// Lend with WRITABLE flag must produce an AP_FULL_ACCESS window.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_writable_ap_dispatch() {
        use crate::mpu::{decode_rasr_ap, AP_FULL_ACCESS};
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let mut k = kernel(0, 0, 0);
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Lend writable: r2 = target(1) | WRITABLE(0x100)
        let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1 | 0x100);
        assert!(
            base_addr < 0x8000_0000,
            "r0 should contain valid base address"
        );
        let desc = k
            .dynamic_strategy
            .slot(region_id as u8)
            .expect("window descriptor must exist after writable lend");
        assert_eq!(
            decode_rasr_ap(desc.permissions),
            AP_FULL_ACCESS,
            "writable lend must set AP_FULL_ACCESS"
        );
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "revoke must succeed"
        );
    }

    /// BorrowedRead owner can lend read-only through the SVC handler.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_read_owner_share_dispatch() {
        use crate::mpu::{decode_rasr_ap, AP_RO_RO};
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_REVOKE};
        let mut k = kernel(0, 0, 0);
        // Allocate BorrowedRead buffer: r1=0
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 0, 0);
        assert!(slot < 0x8000_0000, "alloc BorrowedRead should succeed");
        // Lend read-only to partition 1
        let (base_addr, region_id) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!(
            base_addr < 0x8000_0000,
            "r0 should contain valid base address"
        );
        let desc = k
            .dynamic_strategy
            .slot(region_id as u8)
            .expect("window descriptor must exist after lend");
        assert_eq!(
            decode_rasr_ap(desc.permissions),
            AP_RO_RO,
            "BorrowedRead read-only lend must set AP_RO_RO"
        );
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "revoke must succeed"
        );
    }

    /// BorrowedRead owner cannot lend with WRITABLE flag — must get PermissionDenied.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_lend_read_owner_writable_rejected_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND};
        let mut k = kernel(0, 0, 0);
        // Allocate BorrowedRead buffer: r1=0
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 0, 0);
        assert!(slot < 0x8000_0000, "alloc BorrowedRead should succeed");
        // Attempt writable lend — must be rejected
        let result = dispatch_r0(&mut k, SYS_BUF_LEND, slot, 1 | 0x100);
        assert_eq!(
            result,
            SvcError::PermissionDenied.to_u32(),
            "BorrowedRead writable lend must return PermissionDenied"
        );
    }

    /// Owner can still read a buffer via SYS_BUF_READ while it is lent.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_read_while_lent_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_READ, SYS_BUF_WRITE};
        let mut k = kernel(0, 0, 0);
        // Allocate writable buffer
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        // Write a magic pattern via SYS_BUF_WRITE
        let pat: [u8; 32] = core::array::from_fn(|i| (i as u8) ^ 0x5A);
        let ptr = low32_buf(0);
        // SAFETY: ptr valid for 4096 bytes (mmap via low32_buf), writing 32.
        unsafe { core::ptr::copy_nonoverlapping(pat.as_ptr(), ptr, 32) };
        assert_eq!(
            dispatch_r04(&mut k, SYS_BUF_WRITE, slot, 32, ptr as u32),
            32,
            "write should return 32 bytes"
        );
        // Lend to partition 1
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");
        // Clear the user buffer, then read back as owner while lent
        // SAFETY: ptr valid for 4096 bytes (mmap via low32_buf), zeroing 32.
        unsafe { core::ptr::write_bytes(ptr, 0, 32) };
        assert_eq!(
            dispatch_r04(&mut k, SYS_BUF_READ, slot, 32, ptr as u32),
            32,
            "owner must read while buffer is lent"
        );
        // SAFETY: ptr valid for 4096 bytes (mmap via low32_buf), dispatch wrote 32 bytes.
        let out = unsafe { core::slice::from_raw_parts(ptr, 32) };
        assert_eq!(out, &pat, "read data must match written pattern");
    }

    /// SYS_BUF_RELEASE must fail while the buffer is lent, succeed after revoke.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn buf_release_while_lent_dispatch() {
        use crate::syscall::{SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_RELEASE, SYS_BUF_REVOKE};
        let eop = SvcError::OperationFailed.to_u32();
        let mut k = kernel(0, 0, 0);
        let slot = dispatch_r0(&mut k, SYS_BUF_ALLOC, 1, 0);
        assert!(slot < 0x8000_0000, "alloc should succeed");
        let (_, rid) = dispatch_r01(&mut k, SYS_BUF_LEND, slot, 1);
        assert!((4..=7).contains(&rid), "lend should succeed");
        // Release while lent must fail
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_RELEASE, slot, 0),
            eop,
            "release while lent must return OperationFailed"
        );
        // Revoke the lend
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_REVOKE, slot, 1),
            0,
            "revoke must succeed"
        );
        // Release after revoke must succeed
        assert_eq!(
            dispatch_r0(&mut k, SYS_BUF_RELEASE, slot, 0),
            0,
            "release after revoke must succeed"
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_open_dispatch_valid_and_invalid() {
        use crate::syscall::SYS_DEV_OPEN;
        let mut k = kernel(0, 0, 0);
        // Open device 0 (UART-A) — should succeed
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Open device 1 (UART-B) — should succeed
        let mut ef = frame(SYS_DEV_OPEN, 1, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Open invalid device 99 — should return InvalidResource
        let mut ef = frame(SYS_DEV_OPEN, 99, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[cfg(feature = "dynamic-mpu")]
    fn low32_buf(page: usize) -> *mut u8 {
        crate::test_mmap::low32_buf(page)
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_write_dispatch_routes_to_uart() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_WRITE};
        let (registry, uart_a, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // SAFETY: ptr points to a valid mmap'd page.
        unsafe { core::ptr::copy_nonoverlapping([0xAA, 0xBB, 0xCC].as_ptr(), ptr, 3) };
        let mut ef = frame4(SYS_DEV_WRITE, 0, 3, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing
        // because we only touch it here outside of dispatch.
        unsafe { &mut *uart_a }.push_rx(&[0xDE, 0xAD]);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_DEV_READ, 0, 4, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push some RX data into UART-B
        // SAFETY: uart_b points to a leaked VirtualUartBackend; no aliasing
        // because we only touch it here outside of dispatch.
        unsafe { &mut *uart_b }.push_rx(&[1, 2, 3]);
        // IOCTL_AVAILABLE should return 3
        let mut ef = frame4(SYS_DEV_IOCTL, 1, IOCTL_AVAILABLE, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
        // Write to invalid device — pointer validation rejects first
        let mut ef = frame4(SYS_DEV_WRITE, 99, 1, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv_ptr);
        // Read from invalid device — pointer validation rejects first
        let mut ef = frame4(SYS_DEV_READ, 99, 4, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv_ptr);
        // Ioctl on invalid device
        let mut ef = frame4(SYS_DEV_IOCTL, 99, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // Type annotation needed: const generic N cannot be inferred through dyn VirtualDevice.
        let hw: &mut HwUartBackend<8> =
            Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
        registry.add(hw).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open hw_uart device 5 (registered in the registry)
        let mut ef = frame(SYS_DEV_OPEN, 5, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Close device 5 — should succeed
        let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // Type annotation needed: const generic N cannot be inferred through dyn VirtualDevice.
        let hw: &mut HwUartBackend<8> =
            Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
        registry.add(hw).unwrap();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Close device 5 without opening — HwUartBackend checks require_open,
        // so this returns OperationFailed.
        let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push a byte into the RX buffer so read succeeds immediately.
        // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing.
        unsafe { &mut *uart_a }.push_rx(&[0x42]);
        let ptr = low32_buf(0);
        // timeout=10, buf_len=1; data is available, so should return immediately.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (10 << 16) | 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout=50, buf_len=1 should block the caller.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (50 << 16) | 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout>0 should block and trigger deschedule.
        assert!(!k.yield_requested());
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (50 << 16) | 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; timeout=0, buf_len=1 (non-blocking) should return 0 immediately.
        // Packed r2: (timeout << 16) | buf_len = (0 << 16) | 1 = 1.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
    fn dev_read_timed_nonblocking_packed_len64_returns_zero() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, _, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // No RX data; packed r2 with timeout=0, buf_len=64 must NOT block.
        // Packed r2: (timeout << 16) | buf_len = (0 << 16) | 64 = 64.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, 64, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "non-blocking read must return 0 immediately");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "partition must remain Running for non-blocking read"
        );
        assert_eq!(k.dev_wait_queue().len(), 0, "must not enqueue a waiter");
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_invalid_device_returns_error() {
        use crate::syscall::SYS_DEV_READ_TIMED;
        let mut k = kernel(0, 0, 0);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_DEV_READ_TIMED, 99, (10 << 16) | 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_rejects_out_of_bounds_pointer() {
        use crate::syscall::SYS_DEV_READ_TIMED;
        let mut k = kernel(0, 0, 0);
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (10 << 16) | 1, 0xDEAD_0000);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_timed_multibyte_reads_correct_buffer_length() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, uart_a, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);
        // Open device 0
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push 16 bytes with a known pattern into the RX buffer.
        let pattern: [u8; 16] = [
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
            0x1E, 0x1F,
        ];
        // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing.
        unsafe { &mut *uart_a }.push_rx(&pattern);
        let ptr = low32_buf(0);
        // timeout=10, buf_len=16; data is available, so should return immediately.
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (10 << 16) | 16, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 16, "should return 16 bytes read");
        // SAFETY: ptr was mmap'd by low32_buf and dispatch wrote 16 bytes.
        let out = unsafe { core::slice::from_raw_parts(ptr, 16) };
        assert_eq!(out, &pattern, "buffer contents must match pushed pattern");
        // Partition should remain Running (not blocked).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
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
    fn validate_ptr_in_kernel_code_fails() {
        // Even if MPU region covers flash, kernel code region is rejected.
        let t = ptr_table(0x0000_0000, 0x0002_0000);
        assert!(!validate_user_ptr(&t, 0, 0x0000_0000, 16));
        assert!(!validate_user_ptr(&t, 0, 0x0000_8000, 64));
        assert!(!validate_user_ptr(&t, 0, KERNEL_CODE_END - 16, 16));
        // Spanning boundary: starts in kernel code, ends outside.
        assert!(!validate_user_ptr(&t, 0, KERNEL_CODE_END - 8, 16));
    }

    #[test]
    fn validate_ptr_kernel_data_rejected() {
        // Override kernel_data_end so Guard 3 is active with a non-trivial
        // kernel data region [0x2000_0000, 0x2000_2000).
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(Some(0x2000_2000)));
        // MPU region covers SRAM from 0x2000_2000 onward (above kernel data).
        let t = ptr_table(0x2000_2000, 0x0000_E000);
        // Pointer inside kernel data region — rejected by Guard 3.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0000, 16));
        assert!(!validate_user_ptr(&t, 0, 0x2000_1000, 64));
        // Spanning boundary: starts in kernel data, ends outside.
        assert!(!validate_user_ptr(&t, 0, 0x2000_1FF0, 32));
        // Pointer at kernel_data_end — accepted (outside kernel data, in grant).
        assert!(validate_user_ptr(&t, 0, 0x2000_2000, 16));
        // Clean up override.
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(None));
    }

    #[test]
    fn validate_ptr_own_stack_in_kernel_data_accepted() {
        // Override kernel_data_end so Guard 3 covers [0x2000_0000, 0x2000_2000).
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(Some(0x2000_2000)));
        // Stack at 0x2000_0000..0x2000_0800 — inside kernel data region.
        // Data at 0x2000_2000..0x2000_3000 — above kernel data.
        let t = ptr_table_separate_regions(0x2000_0000, 0x800, 0x2000_2000, 0x1000);
        // Guard 2 (grant) must accept own-stack pointers before Guard 3 (SRAM
        // rejection) fires.  This exercises the guard ordering fix.
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 4)); // start of stack
        assert!(validate_user_ptr(&t, 0, 0x2000_0400, 16)); // middle of stack
        assert!(validate_user_ptr(&t, 0, 0x2000_07F0, 16)); // near end of stack

        // Clean up override.
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(None));
    }

    #[test]
    fn validate_ptr_null_and_low_kernel_code_rejected() {
        let t = ptr_table(0x2000_0000, 4096);
        // Null pointer (address 0) — rejected.
        assert!(!validate_user_ptr(&t, 0, 0, 1));
        // Null with zero length: not in any grant region — rejected.
        assert!(!validate_user_ptr(&t, 0, 0, 0));
        // Non-zero address in kernel code region — verifies range check, not
        // just a null check.
        assert!(!validate_user_ptr(&t, 0, 0x8000, 16));
    }

    #[test]
    fn validate_ptr_length_wraps_address_space() {
        let t = ptr_table(0x2000_0000, 0x0001_0000);
        // ptr + len wraps u32 — rejected by checked_add overflow.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0100, usize::MAX));
        assert!(!validate_user_ptr(&t, 0, 1, u32::MAX as usize));
    }

    // ---- overlaps_kernel_data parameterized tests ----

    #[test]
    fn overlaps_kernel_data_rejects_within_kernel_region() {
        let kernel_end: u32 = 0x2000_2000;
        // Pointer at SRAM start, fully inside [0x2000_0000, 0x2000_2000).
        assert!(overlaps_kernel_data(0x2000_0000, 0x2000_0010, kernel_end));
        // Pointer in the middle of the kernel region.
        assert!(overlaps_kernel_data(0x2000_1000, 0x2000_1010, kernel_end));
        // Pointer at last byte of kernel region.
        assert!(overlaps_kernel_data(0x2000_1FFF, 0x2000_2000, kernel_end));
        // Pointer spanning the kernel_end boundary.
        assert!(overlaps_kernel_data(0x2000_1FF0, 0x2000_2010, kernel_end));
    }

    #[test]
    fn overlaps_kernel_data_accepts_above_kernel_end() {
        let kernel_end: u32 = 0x2000_2000;
        // Pointer starts exactly at kernel_end — no overlap.
        assert!(!overlaps_kernel_data(0x2000_2000, 0x2000_2010, kernel_end));
        // Pointer well above kernel_end.
        assert!(!overlaps_kernel_data(0x2000_8000, 0x2000_9000, kernel_end));
    }

    #[test]
    fn overlaps_kernel_data_disabled_when_kernel_end_equals_sram_start() {
        // Disabled: kernel_end equals SRAM start, so the region is empty.
        let kernel_end: u32 = 0x2000_0000;
        assert!(!overlaps_kernel_data(0x2000_0000, 0x2000_1000, kernel_end));
        assert!(!overlaps_kernel_data(0x2000_0000, 0x2001_0000, kernel_end));
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

    // ---- validate_user_ptr peripheral region tests ----

    /// Build a partition table with data, stack, and peripheral regions.
    fn ptr_table_with_peripherals(
        stack_base: u32,
        stack_size: u32,
        data_base: u32,
        data_size: u32,
        peripheral_regions: &[MpuRegion],
    ) -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,             // entry_point
            stack_base,              // stack_base
            stack_base + stack_size, // stack_pointer (top of stack)
            MpuRegion::new(data_base, data_size, 0),
        )
        .with_peripheral_regions(peripheral_regions);
        t.add(pcb).unwrap();
        t
    }

    #[test]
    fn validate_ptr_in_peripheral_region_passes() {
        // Stack: 0x2000_0000..0x2000_0400
        // Data:  0x2000_1000..0x2000_2000
        // Peripheral: 0x4000_0000..0x4000_0100 (UART)
        let periph = [MpuRegion::new(0x4000_0000, 0x100, 0)];
        let t = ptr_table_with_peripherals(0x2000_0000, 0x400, 0x2000_1000, 0x1000, &periph);

        // Pointer at start of peripheral region.
        assert!(validate_user_ptr(&t, 0, 0x4000_0000, 4));
        // Pointer in middle of peripheral region.
        assert!(validate_user_ptr(&t, 0, 0x4000_0080, 16));
        // Pointer at exact end of peripheral region (ptr + len == base + size).
        assert!(validate_user_ptr(&t, 0, 0x4000_00F0, 16));
        // Zero-length at peripheral region start.
        assert!(validate_user_ptr(&t, 0, 0x4000_0000, 0));
        // Zero-length at peripheral region end.
        assert!(validate_user_ptr(&t, 0, 0x4000_0100, 0));
    }

    #[test]
    fn validate_ptr_outside_peripheral_region_fails() {
        // Stack: 0x2000_0000..0x2000_0400
        // Data:  0x2000_1000..0x2000_2000
        // Peripheral: 0x4000_0000..0x4000_0100
        let periph = [MpuRegion::new(0x4000_0000, 0x100, 0)];
        let t = ptr_table_with_peripherals(0x2000_0000, 0x400, 0x2000_1000, 0x1000, &periph);

        // Before peripheral region.
        assert!(!validate_user_ptr(&t, 0, 0x3FFF_FF00, 16));
        // After peripheral region.
        assert!(!validate_user_ptr(&t, 0, 0x4000_0100, 16));
        // Spanning past peripheral region end.
        assert!(!validate_user_ptr(&t, 0, 0x4000_00F8, 16));
        // In gap between data and peripheral.
        assert!(!validate_user_ptr(&t, 0, 0x3000_0000, 16));
    }

    #[test]
    fn validate_ptr_multiple_peripheral_regions() {
        // Stack: 0x2000_0000..0x2000_0400
        // Data:  0x2000_1000..0x2000_2000
        // Peripheral 1: 0x4000_0000..0x4000_0100 (UART)
        // Peripheral 2: 0x4001_0000..0x4001_0200 (SPI)
        let periph = [
            MpuRegion::new(0x4000_0000, 0x100, 0),
            MpuRegion::new(0x4001_0000, 0x200, 0),
        ];
        let t = ptr_table_with_peripherals(0x2000_0000, 0x400, 0x2000_1000, 0x1000, &periph);

        // First peripheral region.
        assert!(validate_user_ptr(&t, 0, 0x4000_0000, 16));
        assert!(validate_user_ptr(&t, 0, 0x4000_00F0, 16));
        // Second peripheral region.
        assert!(validate_user_ptr(&t, 0, 0x4001_0000, 32));
        assert!(validate_user_ptr(&t, 0, 0x4001_01E0, 32));
        // Gap between peripherals fails.
        assert!(!validate_user_ptr(&t, 0, 0x4000_0800, 16));
        // Spanning both peripherals fails.
        assert!(!validate_user_ptr(&t, 0, 0x4000_0000, 0x1_0100));
    }

    #[test]
    fn validate_ptr_no_peripheral_regions_configured() {
        // Partition without any peripheral regions.
        let t = ptr_table_with_peripherals(0x2000_0000, 0x400, 0x2000_1000, 0x1000, &[]);

        // Pointer in typical peripheral address range should fail.
        assert!(!validate_user_ptr(&t, 0, 0x4000_0000, 16));
        // Data and stack regions still work.
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 16)); // stack
        assert!(validate_user_ptr(&t, 0, 0x2000_1000, 16)); // data
    }

    // ---- Dedicated pointer validation unit tests ----
    //
    // Pointer validation is covered by the tests below (null, boundary,
    // misalignment) and by `validate_mpu_region` tests in mpu.rs.

    #[test]
    fn validate_ptr_null_pointer_rejected() {
        let t = ptr_table(0x2000_0000, 4096);
        // Null pointer (0x0) is in kernel code region — always rejected.
        assert!(!validate_user_ptr(&t, 0, 0x0, 1));
        assert!(!validate_user_ptr(&t, 0, 0x0, 4));
        // Null with zero length: ptr=0 is outside accessible regions.
        assert!(!validate_user_ptr(&t, 0, 0x0, 0));
    }

    #[test]
    fn validate_ptr_at_u32_max_boundary() {
        let t = ptr_table(0x2000_0000, 4096);
        // ptr = 0xFFFF_FFFF, len = 1 → overflow (wraps past u32::MAX).
        assert!(!validate_user_ptr(&t, 0, 0xFFFF_FFFF, 1));
        // ptr = 0xFFFF_FFFF, len = 0 → no overflow, but outside all regions.
        assert!(!validate_user_ptr(&t, 0, 0xFFFF_FFFF, 0));
        // ptr = 0xFFFF_FFFE, len = 2 → end = 0x1_0000_0000 overflows u32.
        assert!(!validate_user_ptr(&t, 0, 0xFFFF_FFFE, 2));
        // No overflow but far outside partition region.
        assert!(!validate_user_ptr(&t, 0, 0xFFFF_FF00, 0xFF));
    }

    #[test]
    fn validate_ptr_misaligned_within_valid_region() {
        let t = ptr_table(0x2000_0000, 4096);
        // validate_user_ptr does NOT enforce alignment — it only checks
        // region membership. Misaligned pointers inside a valid region
        // are accepted by this function. Alignment rejection is enforced
        // at the MPU configuration layer (validate_mpu_region / PartitionConfig::validate).
        assert!(validate_user_ptr(&t, 0, 0x2000_0001, 1)); // odd byte
        assert!(validate_user_ptr(&t, 0, 0x2000_0003, 2)); // halfword-misaligned
        assert!(validate_user_ptr(&t, 0, 0x2000_0005, 4)); // word-misaligned
    }

    /// Misaligned pointer rejection: `validate_mpu_region` rejects bases
    /// that are not aligned to their size, which is the kernel's mechanism
    /// for preventing misaligned memory regions.
    #[test]
    fn validate_mpu_region_rejects_misaligned_base_subtask273() {
        use crate::mpu::{validate_mpu_region, MpuError};
        // Base 0x100 with size 4096 (0x1000) — base not aligned to size.
        assert_eq!(
            validate_mpu_region(0x0000_0100, 4096),
            Err(MpuError::BaseNotAligned)
        );
        // Base 0x2000_0001 with size 32 — odd base, not 32-byte aligned.
        assert_eq!(
            validate_mpu_region(0x2000_0001, 32),
            Err(MpuError::BaseNotAligned)
        );
        // Properly aligned base is accepted.
        assert_eq!(validate_mpu_region(0x2000_0000, 4096), Ok(()));
    }

    #[test]
    fn validate_ptr_exact_full_region() {
        let t = ptr_table(0x2000_0000, 4096);
        // Pointer covers the ENTIRE region [base, base + size).
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 4096));
        // One byte more than the full region — rejected.
        assert!(!validate_user_ptr(&t, 0, 0x2000_0000, 4097));
    }

    #[test]
    fn validate_ptr_single_byte_at_region_edges() {
        let t = ptr_table(0x2000_0000, 4096);
        // First byte of region: valid.
        assert!(validate_user_ptr(&t, 0, 0x2000_0000, 1));
        // Last valid byte of region (base + size - 1).
        assert!(validate_user_ptr(&t, 0, 0x2000_0FFF, 1));
        // First byte past region end: invalid.
        assert!(!validate_user_ptr(&t, 0, 0x2000_1000, 1));
    }

    #[test]
    fn validate_ptr_kernel_code_boundary() {
        // Region starts at 0 covering 128KB of flash.
        let t = ptr_table(0x0000_0000, 0x0002_0000);
        // Pointer at exactly KERNEL_CODE_END is valid (outside kernel code).
        assert!(validate_user_ptr(&t, 0, KERNEL_CODE_END, 16));
        // One byte below KERNEL_CODE_END is in kernel code — rejected.
        assert!(!validate_user_ptr(&t, 0, KERNEL_CODE_END - 1, 1));
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

    /// Test validate_user_ptr_dynamic with peripheral regions.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_in_peripheral_region_passes() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};
        // Stack: 0x2000_0000..0x2000_0400
        // Data:  0x2000_1000..0x2000_2000
        // Peripheral: 0x4000_0000..0x4000_0100 (UART)
        let periph = [MpuRegion::new(0x4000_0000, 0x100, 0)];
        let t = ptr_table_with_peripherals(0x2000_0000, 0x400, 0x2000_1000, 0x1000, &periph);
        let s = DynamicStrategy::new();
        // Add a dynamic window for P0.
        s.add_window(0x2001_0000, 256, 0, 0).unwrap();

        // Dynamic window passes.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2001_0000, 16));
        // Static data region passes.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_1000, 16));
        // Static stack region passes.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0000, 16));
        // Peripheral region passes.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x4000_0000, 4));
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x4000_0080, 16));
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x4000_00F0, 16));
        // Outside peripheral region fails.
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x4000_0100, 16));
        // Spanning peripheral boundary fails.
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x4000_00F8, 16));
    }

    /// Test that validate_user_ptr_dynamic rejects pointers in kernel code region.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_in_kernel_code_fails() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};
        // MPU region covers flash including kernel code area
        let t = ptr_table(0x0000_0000, 0x0002_0000);
        let s = DynamicStrategy::new();
        // Add a dynamic window that also covers kernel code
        s.add_window(0x0000_0000, 0x0001_0000, 0, 0).unwrap();

        // Kernel code region [0, KERNEL_CODE_END) must be rejected
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x0000_0000, 16));
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x0000_8000, 64));
        assert!(!validate_user_ptr_dynamic(
            &t,
            &s,
            0,
            KERNEL_CODE_END - 16,
            16
        ));
        // Spanning boundary: starts in kernel code, ends outside
        assert!(!validate_user_ptr_dynamic(
            &t,
            &s,
            0,
            KERNEL_CODE_END - 8,
            16
        ));
    }

    /// Test kernel data region check when KERNEL_DATA_END equals SRAM start (empty range).
    ///
    /// KERNEL_DATA_END is currently 0x2000_0000 (SRAM start), making the kernel data
    /// region [0x2000_0000, 0x2000_0000) empty. This test verifies SRAM pointers are
    /// allowed when no kernel data region is configured.
    ///
    /// TODO: Add rejection tests when KERNEL_DATA_END is set to an actual value
    /// (e.g., 0x2000_1000) to reserve kernel BSS/data at SRAM start.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_kernel_data_empty_range() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};
        // MPU region covers SRAM
        let t = ptr_table(0x2000_0000, 0x0001_0000);
        let s = DynamicStrategy::new();
        s.add_window(0x2000_0000, 0x0001_0000, 0, 0).unwrap();

        // With KERNEL_DATA_END == 0x2000_0000, the kernel data region is empty,
        // so SRAM addresses are allowed (assuming valid MPU coverage).
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0000, 16));
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0100, 16));
    }

    /// Test that revoking a dynamic MPU window invalidates pointers that were
    /// previously valid in that window.
    ///
    /// This verifies the dynamic nature of pointer validation: when a window
    /// is removed via `remove_window`, `validate_user_ptr_dynamic` correctly
    /// rejects pointers that were valid before revocation.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_window_revocation() {
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};

        // Stack: 0x2000_0000..0x2000_0400, Data: 0x2000_1000..0x2000_2000
        let t = ptr_table_separate_regions(0x2000_0000, 0x400, 0x2000_1000, 0x1000);
        let s = DynamicStrategy::new();

        // Add a dynamic window for partition 0 at 0x3000_0000 (256 bytes).
        let region_id = s.add_window(0x3000_0000, 256, 0, 0).unwrap();
        assert_eq!(region_id, 5); // First dynamic slot is R5

        // Pointer at start of window is valid.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x3000_0000, 16));
        // Pointer in middle of window is valid.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x3000_0080, 64));
        // Pointer at end of window is valid.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x3000_00F0, 16));

        // Revoke the window.
        s.remove_window(region_id);

        // Verify window is removed from the strategy.
        assert!(s.slot(region_id).is_none());

        // Same pointers that were valid before are now invalid.
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x3000_0000, 16));
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x3000_0080, 64));
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x3000_00F0, 16));

        // Static regions still work after window revocation.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0000, 16)); // stack
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_1000, 16)); // data
    }

    /// Test that validate_user_ptr_dynamic accepts own-stack pointers even when
    /// the stack region overlaps the kernel data area [0x2000_0000, kernel_data_end).
    ///
    /// Mirrors `validate_ptr_own_stack_in_kernel_data_accepted` but exercises
    /// the dynamic-MPU code path: Guard 2b (static grant) must fire before
    /// Guard 3 (SRAM kernel-data rejection).
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn validate_ptr_dynamic_own_stack_in_kernel_data_accepted() {
        use crate::mpu_strategy::DynamicStrategy;
        // Override kernel_data_end so Guard 3 covers [0x2000_0000, 0x2000_2000).
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(Some(0x2000_2000)));
        // Stack at 0x2000_0000..0x2000_0800 — inside kernel data region.
        // Data at 0x2000_2000..0x2000_3000 — above kernel data.
        let t = ptr_table_separate_regions(0x2000_0000, 0x800, 0x2000_2000, 0x1000);
        let s = DynamicStrategy::new();
        // No dynamic windows — only static regions.

        // Guard 2b (grant) must accept own-stack pointers before Guard 3
        // (SRAM rejection) fires.
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0000, 4)); // start
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0400, 16)); // middle
        assert!(validate_user_ptr_dynamic(&t, &s, 0, 0x2000_07F0, 16)); // near end

        // Pointer inside kernel data but outside all granted regions → rejected.
        assert!(!validate_user_ptr_dynamic(&t, &s, 0, 0x2000_0900, 4));

        // Clean up override.
        KERNEL_DATA_END_OVERRIDE.with(|c| c.set(None));
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

    // ---- check_user_ptr / check_user_ptr_dynamic method tests ----

    #[test]
    fn check_user_ptr_ok_and_err() {
        let k = kernel(0, 0, 0);
        // Static regions accepted by both check_user_ptr and check_user_ptr_dynamic.
        assert_eq!(k.check_user_ptr(0x2000_0000, 4), Ok(()));
        assert_eq!(k.check_user_ptr(0x2000_0800, 16), Ok(()));
        assert_eq!(k.check_user_ptr_dynamic(0x2000_0000, 4), Ok(()));
        assert_eq!(k.check_user_ptr_dynamic(0x2000_0800, 16), Ok(()));
        // Out-of-bounds addresses rejected.
        assert_eq!(
            k.check_user_ptr(0x0000_1000, 4),
            Err(SvcError::InvalidPointer.to_u32())
        );
        assert_eq!(
            k.check_user_ptr(0x4000_0000, 4),
            Err(SvcError::InvalidPointer.to_u32())
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn check_user_ptr_dynamic_accepts_dynamic_window() {
        let k = kernel(0, 0, 0);
        let window_base: u32 = 0x3000_0000;
        let window_size: u32 = 256;
        k.dynamic_strategy
            .add_window(window_base, window_size, 0, 0)
            .expect("add_window failed: MPU window slots exhausted or invalid config");
        // Static-only rejects the dynamic window.
        assert_eq!(
            k.check_user_ptr(window_base, 4),
            Err(SvcError::InvalidPointer.to_u32())
        );
        // Dynamic validation accepts it.
        assert_eq!(k.check_user_ptr_dynamic(window_base, 4), Ok(()));
        assert_eq!(
            k.check_user_ptr_dynamic(window_base, window_size as usize),
            Ok(())
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn check_user_ptr_dynamic_rejects_wrong_owner_and_revocation() {
        let k = kernel(0, 0, 0);
        // Window owned by partition 1 — inaccessible to partition 0.
        k.dynamic_strategy
            .add_window(0x3000_0000, 256, 0, 1)
            .expect("add_window for partition 1 failed unexpectedly");
        assert_eq!(
            k.check_user_ptr_dynamic(0x3000_0000, 4),
            Err(SvcError::InvalidPointer.to_u32())
        );

        // Window owned by partition 0, then revoked.
        let k2 = kernel(0, 0, 0);
        let rid = k2
            .dynamic_strategy
            .add_window(0x3000_0000, 256, 0, 0)
            .expect("add_window for partition 0 failed unexpectedly");
        assert_eq!(k2.check_user_ptr_dynamic(0x3000_0000, 4), Ok(()));
        k2.dynamic_strategy.remove_window(rid);
        assert_eq!(
            k2.check_user_ptr_dynamic(0x3000_0000, 4),
            Err(SvcError::InvalidPointer.to_u32())
        );
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
            // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
            // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
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
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(
            ef.r0,
            SvcError::InvalidPointer.to_u32(),
            "BbRead should reject out-of-bounds pointer"
        );
    }

    // ---- QueuingRecvTimed dispatch tests ----

    #[cfg(not(feature = "dynamic-mpu"))]
    fn low32_buf(page: usize) -> *mut u8 {
        crate::test_mmap::low32_buf(page)
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
        let page = low32_buf(0);
        // Byte offset within the shared mmap page, chosen to avoid data
        // races with parallel tests that also use low32_buf(0).
        // The resulting address (MMAP_BASE + RECV_BUF_OFFSET = 0x2000_0200)
        // falls within partition 0's data region.
        const RECV_BUF_OFFSET: usize = 512;
        // SAFETY: page is a 4096-byte mmap; RECV_BUF_OFFSET (512) + max
        // message length (4) is well within bounds.
        let ptr = unsafe { page.add(RECV_BUF_OFFSET) };
        // SAFETY: ptr is within the mmap page; zeroing 2 bytes is in-bounds.
        unsafe { core::ptr::write_bytes(ptr, 0, 2) };
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 100, ptr as u32);
        // SAFETY: ef is a valid ExceptionFrame and k is a valid Kernel.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 2, "should return msg_len=2");
        // SAFETY: ptr points into a valid mmap page and dispatch wrote 2 bytes.
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
        let r2 = (50u32 << 16) | 4;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
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
        let r2 = (50u32 << 16) | 4;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
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
        // Empty queue with original QueuingRecv (timeout=0) → QueueEmpty → returns 0 (transient)
        let mut ef = frame4(SYS_QUEUING_RECV, dst as u32, 0, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Partition should NOT be in Waiting (no blocking happened)
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_recv_timed_extracts_timeout_from_upper_r2() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;

        let mut k = kernel(0, 0, 0);
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();

        // Pack r2: timeout=75 in upper 16 bits, buf_len=4 in lower 16 bits.
        // If the handler incorrectly used the full r2 as timeout, expiry
        // would be 0 + ((75 << 16) | 4) = 4_915_204 instead of 75.
        k.sync_tick(0);
        let ptr = low32_buf(0);
        let r2 = (75u32 << 16) | 4;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Expire at tick=75 (the correctly extracted timeout).
        // If the handler used the full r2, this would NOT expire the waiter.
        k.expire_timed_waits::<8>(75);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "timeout must come from upper 16 bits of r2, not the full value"
        );
    }

    // ---- QueuingSendTimed dispatch tests ----

    /// Helper: create a connected Source→Destination pair, return (src_id, dst_id).
    fn connected_send_pair(k: &mut Kernel<'static, TestConfig>) -> (usize, usize) {
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
    fn assert_queued_message(k: &mut Kernel<'static, TestConfig>, d: usize, expected_data: &[u8]) {
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
            RecvQueuingOutcome::Received { msg_len, .. } => {
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
        // TODO: Offset change (64 -> 512) to avoid test interference is outside original
        // subtask scope; consider moving to separate commit.
        // SAFETY: test-only, ptr.add within known-mapped page bounds.
        let data_ptr = unsafe { ptr.add(512) };
        // SAFETY: test-only, writing to a known-mapped page.
        unsafe {
            *data_ptr = 0xAA;
            *data_ptr.add(1) = 0xBB;
        }
        let r2 = pack_r2(100, 2);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, data_ptr as u32);
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
        // TODO: Offset change (0 -> 768) and variable rename (*ptr -> *data_ptr) to avoid
        // test interference is outside original subtask scope; consider moving to separate commit.
        // SAFETY: test-only, ptr.add within known-mapped page bounds.
        let data_ptr = unsafe { ptr.add(768) };
        // SAFETY: test-only, writing to a known-mapped page.
        unsafe {
            *data_ptr = 0x42;
        }
        let r2 = pack_r2(100, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, data_ptr as u32);
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
        // Full queue with timeout=0 → QueueFull → returns 0 (transient, not an error)
        let mut ef = frame4(SYS_QUEUING_SEND, s as u32, 1, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Partition should NOT be in Waiting (no blocking with original handler)
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_recv_connected_empty_nonblocking_stays_running() {
        use crate::syscall::SYS_QUEUING_RECV;
        let mut k = kernel(0, 0, 0);
        let (_s, d) = connected_send_pair(&mut k);
        let ptr = low32_buf(0);
        assert!(
            !k.yield_requested(),
            "yield_requested should be false before non-blocking recv"
        );
        // Connected empty queue, non-blocking (timeout=0): returns 0 immediately.
        let mut ef = frame4(SYS_QUEUING_RECV, d as u32, 0, ptr as u32);
        // SAFETY: frame4 built a valid ExceptionFrame and kernel is in test
        // mode with partition 0 running; dispatch only mutates kernel state.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert!(
            !k.yield_requested(),
            "non-blocking recv must not trigger deschedule"
        );
    }

    #[test]
    fn queuing_send_connected_full_nonblocking_stays_running() {
        use crate::syscall::SYS_QUEUING_SEND;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill destination queue to capacity (QD=4).
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(d)
                .unwrap()
                .inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        assert!(
            !k.yield_requested(),
            "yield_requested should be false before non-blocking send"
        );
        // Connected full queue, non-blocking (timeout=0): returns 0 immediately.
        let mut ef = frame4(SYS_QUEUING_SEND, s as u32, 1, ptr as u32);
        // SAFETY: frame4 built a valid ExceptionFrame and kernel is in test
        // mode with partition 0 running; dispatch only mutates kernel state.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert!(
            !k.yield_requested(),
            "non-blocking send must not trigger deschedule"
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

    // ---- blocking-path completeness meta-test ----

    /// Regression gate: every blocking syscall MUST transition to Waiting
    /// and set yield_requested (proving trigger_deschedule was called).
    /// If a new blocking path is added and this test is not updated, the
    /// coverage gap should be caught during code review.
    #[test]
    fn all_blocking_paths_set_waiting_and_yield_requested() {
        use crate::sampling::PortDirection;
        use crate::syscall::{
            SYS_BB_READ, SYS_EVT_WAIT, SYS_MSG_RECV, SYS_MSG_SEND, SYS_MTX_LOCK,
            SYS_QUEUING_RECV_TIMED, SYS_QUEUING_SEND_TIMED, SYS_SEM_WAIT,
        };

        // Shared assertion: after a blocking dispatch the caller partition
        // must be Waiting and yield_requested must be true.
        macro_rules! assert_blocked {
            ($k:expr, $pid:expr, $label:expr) => {
                assert_eq!(
                    $k.partitions().get($pid).unwrap().state(),
                    PartitionState::Waiting,
                    "{}: partition must be Waiting",
                    $label
                );
                assert!(
                    $k.yield_requested(),
                    "{}: yield_requested must be true",
                    $label
                );
            };
        }

        // --- EventWait: no bits set → blocks immediately ---
        {
            let mut k = kernel(0, 0, 0);
            let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "EventWait");
        }

        // --- SemWait: count starts at 1; second wait blocks ---
        {
            let mut k = kernel(1, 0, 0);
            let mut ef = frame(SYS_SEM_WAIT, 0, 0);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 1, "SemWait: first wait should acquire");
            let mut ef = frame(SYS_SEM_WAIT, 0, 0);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "SemWait");
        }

        // --- MutexLock: partition 0 acquires; partition 1 blocks ---
        {
            let mut k = kernel(0, 1, 0);
            let mut ef = frame(SYS_MTX_LOCK, 0, 0);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 1, "MutexLock: first lock should acquire");
            k.set_current_partition(1);
            let mut ef = frame(SYS_MTX_LOCK, 0, 1);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 1, "MutexLock");
        }

        // --- QueuingRecvTimed: empty destination port → blocks ---
        // NOTE: low32_buf(0) reuse is intentional — each sub-test runs in its
        // own block with a fresh Kernel, so the buffer identity does not matter;
        // only the address (which must fall within the partition's MPU region).
        {
            let mut k = kernel(0, 0, 0);
            let _dst = k
                .queuing_mut()
                .create_port(PortDirection::Destination)
                .unwrap();
            let ptr = low32_buf(0);
            let r2 = (50u32 << 16) | 4;
            let mut ef = frame4(SYS_QUEUING_RECV_TIMED, _dst as u32, r2, ptr as u32);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "QueuingRecvTimed");
        }

        // --- QueuingSendTimed: full destination queue → blocks ---
        {
            let mut k = kernel(0, 0, 0);
            let (s, d) = connected_send_pair(&mut k);
            for _ in 0..4 {
                k.queuing_mut()
                    .get_mut(d)
                    .unwrap()
                    .inject_message(1, &[0x42]);
            }
            let ptr = low32_buf(0);
            let r2 = pack_r2(50, 1);
            let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "QueuingSendTimed");
        }

        // --- BbRead: empty blackboard with timeout → blocks ---
        {
            let mut k = kernel(0, 0, 0);
            let id = k.blackboards_mut().create().unwrap();
            let ptr = low32_buf(0);
            let mut ef = frame4(SYS_BB_READ, id as u32, 50, ptr as u32);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "BbRead");
        }

        // --- MsgSend: full queue → blocks sender ---
        {
            let mut k = kernel(0, 0, 1);
            // Fill the depth-4 queue to capacity via direct API
            for i in 0..4u8 {
                let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
                assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
            }
            let ptr = low32_buf(0);
            // r1=queue 0, r2=sender partition 0, r3=data pointer
            let mut ef = frame4(SYS_MSG_SEND, 0, 0, ptr as u32);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "MsgSend");
        }

        // --- MsgRecv: empty queue → blocks receiver ---
        {
            let mut k = kernel(0, 0, 1);
            let ptr = low32_buf(0);
            // r1=queue 0, r2=receiver partition 0, r3=buffer pointer
            let mut ef = frame4(SYS_MSG_RECV, 0, 0, ptr as u32);
            dispatch_checked(&mut k, &mut ef);
            assert_eq!(ef.r0, 0);
            assert_blocked!(k, 0, "MsgRecv");
        }
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
        // Type annotation needed: const generic N cannot be inferred through dyn VirtualDevice.
        let hw: &mut HwUartBackend<8> =
            Box::leak(Box::new(HwUartBackend::new(0, UartRegs::new(0x4000_C000))));
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

    // --- sleep timer expiry tests ---

    #[test]
    fn expire_timed_waits_wakes_sleeping_partition() {
        let mut k = kernel(0, 0, 0);
        // Partition 0: enqueue sleep until tick 100, transition to Waiting.
        k.partitions_mut().get_mut(0).unwrap().set_sleep_until(100);
        k.sleep_queue.push(0, 100).unwrap();
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Before expiry: stays Waiting.
        k.expire_timed_waits::<8>(99);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 100);

        // At expiry: transitions Waiting→Ready, sleep_until cleared.
        k.expire_timed_waits::<8>(100);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
        assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 0);
        assert!(k.yield_requested);
    }

    #[test]
    fn expire_timed_waits_sleep_no_false_wake() {
        let mut k = kernel(0, 0, 0);
        // Partition in Waiting with no sleep queue entry should not be affected.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        k.expire_timed_waits::<8>(1000);
        // Partition stays Waiting — no entry in sleep queue.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    #[test]
    fn expire_timed_waits_sleep_transition_fail_preserves_sleep_until() {
        let mut k = kernel(0, 0, 0);
        // Partition 0 starts Running. Transition to Ready so that the
        // Ready→Ready transition in expire_timed_waits will fail.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Ready)
            .unwrap();
        k.partitions_mut().get_mut(0).unwrap().set_sleep_until(50);
        k.sleep_queue.push(0, 50).unwrap();
        // Partition is Ready, so try_transition(Ready→Ready) is invalid.
        // sleep_until must NOT be cleared when the transition fails.
        k.expire_timed_waits::<8>(50);
        assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 50);
        assert!(k.sleep_queue.is_empty());
    }

    // --- SYS_SLEEP_TICKS dispatch tests ---

    #[test]
    fn sleep_ticks_zero_returns_immediately() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(50);
        let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 0);
        assert!(k.sleep_queue.is_empty());
        assert!(!k.yield_requested);
    }

    #[test]
    fn sleep_ticks_nonzero_blocks_partition() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(100);
        let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 50, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 150);
        assert!(!k.sleep_queue.is_empty());
        assert!(k.yield_requested);
    }

    #[test]
    fn sleep_ticks_queue_full_returns_error() {
        let mut k = kernel(0, 0, 0);
        k.sync_tick(10);
        // Fill the sleep queue to capacity (N = 4).
        for i in 0..4u8 {
            let _ = k.sleep_queue.push(i, 1000 + i as u64);
        }
        let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 50, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::WaitQueueFull.to_u32());
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    // Kernel::new() validation - comprehensive tests in kernel.rs for KernelState

    #[test]
    fn kernel_new_validates_and_creates() {
        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap();
        // Test empty schedule rejection
        let empty: ScheduleTable<4> = ScheduleTable::new();
        assert!(matches!(
            Kernel::<TestConfig>::new(empty, core::slice::from_ref(&mem)),
            Err(ConfigError::ScheduleEmpty)
        ));
        // Test valid config succeeds
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        s.add_system_window(1).unwrap();
        let k = Kernel::<TestConfig>::new(s, core::slice::from_ref(&mem)).unwrap();
        assert_eq!(k.partitions().len(), 1);
        assert_eq!(k.active_partition(), None);
    }

    #[test]
    fn kernel_create_succeeds_with_valid_config() {
        let mut s: ScheduleTable<4> = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        s.add_system_window(1).unwrap();
        let mut stk = AlignedStack1K::default();
        let mem =
            ExternalPartitionMemory::from_aligned_stack(&mut stk, 1, MpuRegion::new(0, 0, 0), 0)
                .unwrap();
        let k = Kernel::<TestConfig>::new(s, core::slice::from_ref(&mem)).unwrap();
        assert_eq!(k.partitions().len(), 1);
        assert_eq!(k.active_partition(), None);
    }

    /// Kernel::new() preserves mpu_region from config.
    #[test]
    fn kernel_new_mpu_region_base_matches_config() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 50)).unwrap();
        s.add(ScheduleEntry::new(1, 50)).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 1024, 0x0306_0000),
                1,
            )
            .unwrap(),
        ];
        let k = try_kernel_new_mem(s, &mems).unwrap();
        for (i, mem) in mems.iter().enumerate() {
            let pcb = k.partitions().get(i).unwrap();
            // User-configured (size>0): base preserved from config.
            assert_eq!(pcb.mpu_region().base(), mem.mpu_region().base());
            assert_eq!(pcb.mpu_region().size(), mem.mpu_region().size());
            assert_eq!(
                pcb.mpu_region().permissions(),
                mem.mpu_region().permissions()
            );
        }
    }

    #[test]
    fn kernel_new_allows_overlapping_data_regions() {
        // Only the data (MPU) regions overlap here. Since Data-vs-Data is the
        // only overlap and is permitted for shared-memory IPC, this should succeed.
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 50)).unwrap();
        s.add(ScheduleEntry::new(1, 50)).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 16384, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_2001,
                MpuRegion::new(0x2000_2000, 4096, 0),
                1,
            )
            .unwrap(),
        ];
        let _k = try_kernel_new_mem(s, &mems).unwrap();
    }

    #[test]
    fn kernel_new_non_overlapping_mpu_regions_succeeds() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 50)).unwrap();
        s.add(ScheduleEntry::new(1, 50)).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
        ];
        let k = try_kernel_new_mem(s, &mems).unwrap();
        assert_eq!(k.partitions().len(), 2);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        assert_eq!(k.partitions().get(1).unwrap().id(), 1);
        // MPU regions must not overlap: region 0 ends at base0+4096 == base1.
        let b0 = k.partitions().get(0).unwrap().mpu_region().base();
        let b1 = k.partitions().get(1).unwrap().mpu_region().base();
        assert!(b0 + 4096 <= b1, "regions must be non-overlapping");
    }

    /// Helper to call `create_from_memory` with correct feature-flag arguments.
    ///
    /// For `dynamic-mpu` builds, this adds a 1-tick system window to the schedule.
    /// Tests using this helper must ensure total partition ticks ≤
    /// `SYSTEM_WINDOW_MAX_GAP_TICKS` (100).
    fn try_kernel_new_mem(
        schedule: ScheduleTable<4>,
        memories: &[ExternalPartitionMemory<'_>],
    ) -> Result<Kernel<'static, TestConfig>, ConfigError> {
        #[cfg(not(feature = "dynamic-mpu"))]
        {
            Kernel::<TestConfig>::new(schedule, memories)
        }
        #[cfg(feature = "dynamic-mpu")]
        {
            let mut schedule = schedule;
            schedule
                .add_system_window(1)
                .expect("schedule must have room for system window");
            Kernel::<TestConfig>::new(schedule, memories)
        }
    }

    #[test]
    fn kernel_new_accepts_matching_partition_ids() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 50)).unwrap();
        s.add(ScheduleEntry::new(1, 50)).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
        ];
        let k = try_kernel_new_mem(s, &mems).unwrap();
        assert_eq!(k.partitions().len(), 2);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        assert_eq!(k.partitions().get(1).unwrap().id(), 1);
        // Verify data-carrying fields were mapped correctly from ExternalPartitionMemory.
        assert_eq!(k.partitions().get(0).unwrap().entry_point(), 0x0800_0001);
        assert_eq!(k.partitions().get(1).unwrap().entry_point(), 0x0800_1001);
    }

    #[test]
    fn kernel_new_empty_configs_with_schedule_fails() {
        // Schedule references partition 0, but no partitions provided
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        let result = try_kernel_new_mem(s, &[]);

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
    fn init_kernel_struct_returns_valid_kernel() {
        let mut core = <TestConfig as KernelConfig>::Core::default();
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        *core.schedule_mut() = s;
        let pcb = PartitionControlBlock::new(0, 0, 0, 0, MpuRegion::new(0, 0, 0));
        assert!(core.partitions_mut().add(pcb).is_ok());
        let k = Kernel::<TestConfig>::init_kernel_struct(
            core,
            &[],
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
        );
        assert_eq!(k.current_partition, 255);
        assert!(k.active_partition.is_none());
        assert_eq!(k.partitions().len(), 1);
        assert_eq!(k.partitions().get(0).unwrap().id(), 0);
        // Verify schedule was preserved through init_kernel_struct
        assert_eq!(k.schedule().entries().len(), 1);
        assert_eq!(k.schedule().entries()[0].partition_index, 0);
        assert_eq!(k.schedule().entries()[0].duration_ticks, 100);
    }

    #[test]
    fn kernel_new_single_partition_id_zero_succeeds() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap();

        let k = try_kernel_new_mem(s, core::slice::from_ref(&mem)).unwrap();

        assert_eq!(k.partitions().len(), 1);
        let pcb = k.partitions().get(0).unwrap();
        assert_eq!(pcb.id(), 0);
        assert_eq!(pcb.entry_point(), mem.entry_point());
        // User-configured (size>0): base preserved from config.
        assert_eq!(pcb.mpu_region().base(), mem.mpu_region().base());
        assert_eq!(pcb.mpu_region().size(), mem.mpu_region().size());
        assert_eq!(
            pcb.mpu_region().permissions(),
            mem.mpu_region().permissions()
        );
    }

    #[test]
    fn kernel_new_three_contiguous_partitions_succeeds() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 30)).unwrap();
        s.add(ScheduleEntry::new(1, 30)).unwrap();
        s.add(ScheduleEntry::new(2, 30)).unwrap();

        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mut stk2 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk2,
                0x0800_2001,
                MpuRegion::new(0x2000_2000, 4096, 0),
                2,
            )
            .unwrap(),
        ];

        let k = try_kernel_new_mem(s, &mems).unwrap();

        assert_eq!(k.partitions().len(), 3);
        for (i, mem) in mems.iter().enumerate() {
            let pcb = k.partitions().get(i).unwrap();
            assert_eq!(pcb.id(), i as u8);
            assert_eq!(pcb.entry_point(), mem.entry_point());
            // User-configured (size>0): base preserved from config.
            assert_eq!(pcb.mpu_region().base(), mem.mpu_region().base());
            assert_eq!(pcb.mpu_region().size(), mem.mpu_region().size());
            assert_eq!(
                pcb.mpu_region().permissions(),
                mem.mpu_region().permissions()
            );
        }
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn kernel_new_rejects_no_system_window() {
        // Schedule with only partition entries (no system window)
        let mut s = ScheduleTable::<4>::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();

        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap();

        // Call new directly (not try_kernel_new_mem which adds a system window)
        let result = Kernel::<TestConfig>::new(s, core::slice::from_ref(&mem));

        assert!(matches!(result, Err(ConfigError::NoSystemWindow)));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn kernel_new_rejects_system_window_too_infrequent() {
        // Create schedule with system window gap exceeding SYSTEM_WINDOW_MAX_GAP_TICKS (100).
        // P0 runs for 150 ticks, then system window for 1 tick. Gap is 150 > 100.
        let mut s = ScheduleTable::<4>::new();
        s.add(ScheduleEntry::new(0, 150)).unwrap();
        s.add_system_window(1).unwrap();
        // Note: s.start() is intentionally not called here; Kernel::new calls it internally.

        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap();

        let result = Kernel::<TestConfig>::new(s, core::slice::from_ref(&mem));

        assert!(matches!(
            result,
            Err(ConfigError::SystemWindowTooInfrequent {
                max_gap_ticks: 150,
                threshold_ticks: 100,
            })
        ));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn kernel_new_accepts_system_window_at_exact_threshold() {
        // Create schedule with system window gap exactly equal to SYSTEM_WINDOW_MAX_GAP_TICKS (100).
        // P0 runs for 100 ticks, then system window for 1 tick. Gap is 100 == 100.
        // The check is `max_gap > threshold`, so equality should be accepted.
        let mut s = ScheduleTable::<4>::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add_system_window(1).unwrap();

        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap();

        let result = Kernel::<TestConfig>::new(s, core::slice::from_ref(&mem));

        // Should succeed: max_gap (100) is not greater than threshold (100).
        assert!(result.is_ok());
    }

    // -------------------------------------------------------------------------
    // Kernel::new peripheral region validation tests
    // -------------------------------------------------------------------------

    #[test]
    fn kernel_new_rejects_peripheral_region_size_too_small() {
        let mut stk = AlignedStack1K::default();
        let result = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap()
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 16, 0x03)]);
        assert!(matches!(
            result,
            Err(ConfigError::PeripheralRegionInvalid {
                partition_id: 0,
                region_index: 0,
                detail: MpuError::SizeTooSmall,
            })
        ));
    }

    #[test]
    fn kernel_new_rejects_peripheral_region_not_power_of_two() {
        let mut stk = AlignedStack1K::default();
        let result = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap()
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 100, 0x03)]);
        assert!(matches!(
            result,
            Err(ConfigError::PeripheralRegionInvalid {
                partition_id: 0,
                region_index: 0,
                detail: MpuError::SizeNotPowerOfTwo,
            })
        ));
    }

    #[test]
    fn kernel_new_rejects_peripheral_region_base_misaligned() {
        let mut stk = AlignedStack1K::default();
        // base 0x4000_0100 is not aligned to size 4096 (0x1000)
        let result = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap()
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0100, 4096, 0x03)]);
        assert!(matches!(
            result,
            Err(ConfigError::PeripheralRegionInvalid {
                partition_id: 0,
                region_index: 0,
                detail: MpuError::BaseNotAligned,
            })
        ));
    }

    #[test]
    fn kernel_new_accepts_valid_peripheral_region() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        let mut stk = AlignedStack1K::default();
        let mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stk,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap()
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 4096, 0x03)])
        .unwrap();
        let k = try_kernel_new_mem(s, core::slice::from_ref(&mem)).unwrap();
        assert_eq!(k.partitions().get(0).unwrap().peripheral_regions().len(), 1);
        assert_eq!(
            k.partitions().get(0).unwrap().peripheral_regions()[0].base(),
            0x4000_0000
        );
        assert_eq!(
            k.partitions().get(0).unwrap().peripheral_regions()[0].size(),
            4096
        );
    }

    // -------------------------------------------------------------------------
    // Schedule advance and accessor tests
    // -------------------------------------------------------------------------

    /// Number of ticks to advance from the first P1 boundary tick through
    /// the rest of P1's slot and the full P0 slot to reach the next P1
    /// boundary.  Under `dynamic-mpu` the schedule inserts an extra
    /// SystemWindow(1) between P1 and P0.
    #[cfg(not(feature = "dynamic-mpu"))]
    const INTER_P1_TICKS: usize = 2 + 5;
    #[cfg(feature = "dynamic-mpu")]
    const INTER_P1_TICKS: usize = 2 + 1 + 5;

    /// Helper to create a Kernel with a started schedule and partitions.
    // NOTE: local stacks are safe here — Kernel has no lifetime parameter and
    // new() copies all data out of ExternalPartitionMemory.
    fn kernel_with_schedule() -> Kernel<'static, TestConfig> {
        // Create 2-slot schedule: P0 for 5 ticks, P1 for 3 ticks
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 3)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        schedule.start();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
        ];
        let mut k = Kernel::<TestConfig>::new(schedule, &mems).unwrap();
        let _ = &mut k;
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
    fn pcb_returns_correct_partition_by_index() {
        let k = kernel_with_schedule();
        let pcb0 = k.pcb(0).expect("pcb(0) should exist");
        assert_eq!(pcb0.id(), 0);
        let pcb1 = k.pcb(1).expect("pcb(1) should exist");
        assert_eq!(pcb1.id(), 1);
        assert_eq!(k.partition_count(), 2);
    }

    #[test]
    fn pcb_out_of_bounds_returns_none() {
        let k = kernel_with_schedule();
        assert!(k.pcb(2).is_none());
        assert!(k.pcb(100).is_none());
        assert!(k.pcb(usize::MAX).is_none());
    }

    #[test]
    fn pcb_mut_allows_mutation() {
        let mut k = kernel_with_schedule();
        let flags_before = k.pcb(0).unwrap().event_flags();
        assert_eq!(flags_before, 0);
        k.pcb_mut(0).unwrap().set_event_flags(0xAB);
        assert_eq!(k.pcb(0).unwrap().event_flags(), 0xAB);
    }

    #[test]
    fn schedule_accessor_returns_schedule_table() {
        let k = kernel_with_schedule();
        #[cfg(not(feature = "dynamic-mpu"))]
        assert_eq!((k.schedule().major_frame_ticks, k.schedule().len()), (8, 2));
        #[cfg(feature = "dynamic-mpu")]
        assert_eq!((k.schedule().major_frame_ticks, k.schedule().len()), (9, 3));
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
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // 5th tick triggers switch to P1
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
        assert_eq!(k.active_partition(), Some(1));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn advance_schedule_tick_updates_next_partition() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Initially next_partition is 0 (default)
        assert_eq!(k.next_partition(), 0);
        // Advance 4 ticks within slot 0 - no switch, next_partition unchanged
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
            assert_eq!(k.next_partition(), 0);
        }
        // 5th tick triggers switch to P1, next_partition updated
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
        assert_eq!(k.next_partition(), 1);
        // Continue through P1's slot (3 ticks), then wrap to P0
        for _ in 0..2 {
            svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(k.next_partition(), 1);
        }
        // 3rd tick of P1's slot triggers switch back to P0
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(0)
        );
        assert_eq!(k.next_partition(), 0);
    }

    #[test]
    fn advance_schedule_tick_increments_tick_counter() {
        let mut k = kernel_with_schedule();
        assert_eq!(k.tick().get(), 0);
        svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(k.tick().get(), 1);
        svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(k.tick().get(), 2);
    }

    /// Tests advance_schedule_tick skips Waiting partitions (returns None,
    /// doesn't update active_partition or next_partition).
    #[test]
    fn advance_schedule_tick_skips_waiting_partition() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        let (initial_next, initial_active) = (k.next_partition(), k.active_partition());

        // Advance to boundary before P1's slot
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // Transition P1 to Waiting (Ready -> Running -> Waiting)
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // Tick that would switch to P1 returns None instead
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.active_partition(), initial_active);
        assert_eq!(k.next_partition(), initial_next);
    }

    /// Verifies that skipping a Waiting partition does NOT transition the
    /// current active partition from Running to Ready. Covers the early-return
    /// at svc.rs:2270-2273 when an active Running partition exists.
    #[test]
    fn advance_schedule_tick_skips_waiting_preserves_running_state() {
        let mut k = kernel_with_schedule();

        // Make P0 the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );

        // Advance to boundary before P1's slot (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // Transition P1 to Waiting (Ready -> Running -> Waiting).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // 5th tick would switch to P1, but P1 is Waiting => skip.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.active_partition(), Some(0));
        // P0 must still be Running — no Running→Ready transition fired.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    /// Waiting partitions do not accumulate starvation — they voluntarily
    /// blocked, so skipping them is expected.
    #[test]
    fn advance_schedule_tick_waiting_not_starved() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance to boundary before P1's slot (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // Transition P1 to Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(pcb1.starvation_count(), 0);

        // P1's slot fires — P1 is Waiting so it is skipped.
        // Only P0 exists and it is Running (active), so no Ready
        // partition accumulates starvation.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);

        // Advance through remaining P1 ticks + full P0 slot to reach
        // another P1 boundary. P1 has 3 ticks, then P0 has 5 ticks.
        // We already consumed 1 of P1's 3 ticks above.
        for _ in 0..INTER_P1_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        // Next tick hits P1 boundary again — still Waiting, still 0.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// Starvation count resets to 0 when a partition starts running.
    /// Artificially sets starvation to verify the scheduler resets it
    /// on PartitionSwitch.
    #[test]
    fn advance_schedule_tick_resets_starvation_on_run() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance to boundary before P1's slot.
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // Artificially give P1 a non-zero starvation count.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.increment_starvation();
        pcb1.increment_starvation();
        assert_eq!(pcb1.starvation_count(), 2);

        // This tick switches to P1 (Ready) — starvation resets to 0.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// Starvation count stays 0 for partitions that are never skipped.
    #[test]
    fn advance_schedule_tick_starvation_stays_zero_when_not_skipped() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Both partitions are Ready. Advance through a full cycle.
        // P0(5 ticks) -> P1(3 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // Switch to P1.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
        assert_eq!(k.partitions().get(0).unwrap().starvation_count(), 0);
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// Helper: 3-partition schedule for starvation tests.
    /// P0: 5 ticks, P1: 3 ticks, P2: 4 ticks.
    fn kernel_with_3_partition_schedule() -> Kernel<'static, TestConfig> {
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 3)).unwrap();
        schedule.add(ScheduleEntry::new(2, 4)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        {
            schedule.add_system_window(1).unwrap();
        }
        schedule.start();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mut stk2 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk2,
                0x0800_2001,
                MpuRegion::new(0x2000_2000, 4096, 0),
                2,
            )
            .unwrap(),
        ];
        Kernel::<TestConfig>::new(schedule, &mems).unwrap()
    }

    /// Ready partition accumulates starvation when another partition's
    /// Waiting slot is skipped.  3-partition schedule: P0(5), P1(3), P2(4).
    /// P0 runs, P1 is Waiting (skipped), P2 is Ready → P2 starvation++.
    #[test]
    fn advance_schedule_tick_ready_partition_starved_on_skip() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_3_partition_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance to boundary before P1's slot (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // Transition P1 to Waiting so it will be skipped.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P2 is Ready (default state).
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 0);

        // P1's slot fires — P1 is Waiting → skipped.
        // P2 is Ready and not active, so P2.starvation_count increments.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 1);
    }

    /// Starvation count for a Ready partition reaches STARVATION_THRESHOLD
    /// after repeated skips of a Waiting partition's slot.
    #[test]
    fn advance_schedule_tick_starvation_reaches_threshold() {
        use crate::partition::STARVATION_THRESHOLD;
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_3_partition_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance to boundary before P1's slot (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // Transition P1 to Waiting so it gets skipped on every boundary.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P2 is Ready — it will accumulate starvation each time P1's slot
        // fires and is skipped.
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 0);
        assert!(!k.partitions().get(2).unwrap().is_starved());

        // Each cycle: P1 boundary (skip) → advance through remaining
        // P1 ticks + P2 slot + P0 slot to reach next P1 boundary.
        // P1 has 3 ticks, P2 has 4 ticks, P0 has 5 ticks.
        // After hitting P1 boundary (1 tick consumed), remaining = 2 + 4 + 5 = 11.
        // Under dynamic-mpu there is an extra SystemWindow tick.
        #[cfg(not(feature = "dynamic-mpu"))]
        const CYCLE_TICKS: usize = 2 + 4 + 5;
        #[cfg(feature = "dynamic-mpu")]
        const CYCLE_TICKS: usize = 2 + 4 + 1 + 5;

        for skip in 1..=STARVATION_THRESHOLD {
            // Tick that hits P1 boundary — P1 is Waiting, P2 is Ready → P2.starvation++.
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
            let count = k.partitions().get(2).unwrap().starvation_count();
            assert_eq!(
                count, skip,
                "after skip #{skip}, expected count={skip}, got {count}"
            );

            if skip < STARVATION_THRESHOLD {
                assert!(!k.partitions().get(2).unwrap().is_starved());
                // Advance through remaining P1 ticks + P2 slot + P0 slot.
                // P2's slot will fire PartitionSwitch(2) — P2 is Ready, so it
                // runs and its starvation resets. We must re-setup for next skip.
                //
                // Instead, also put P2 in Waiting so that P2's slot is skipped
                // too, and P2's starvation is not reset by running. Wait — that
                // would make P2 Waiting, not Ready, so it wouldn't accumulate
                // starvation. We need P2 to stay Ready but NOT run.
                //
                // The only way P2 stays Ready but doesn't run is if we don't
                // advance through P2's slot boundary. Instead, skip through
                // P1's remaining ticks, then P2's slot (P2 runs — starvation
                // resets), then P0's slot, then re-set starvation manually
                // to preserve the count for the next skip.
                for _ in 0..CYCLE_TICKS {
                    svc_scheduler::advance_schedule_tick(&mut k);
                }
                // P2 ran during its slot, resetting starvation. Re-apply
                // the accumulated count for the next iteration.
                let pcb2 = k.partitions_mut().get_mut(2).unwrap();
                for _ in 0..skip {
                    pcb2.increment_starvation();
                }
            }
        }

        // After exactly STARVATION_THRESHOLD skips, P2 is_starved() must be true.
        assert!(k.partitions().get(2).unwrap().is_starved());
        assert_eq!(
            k.partitions().get(2).unwrap().starvation_count(),
            STARVATION_THRESHOLD
        );
    }

    #[test]
    fn advance_schedule_tick_sleeping_not_starved() {
        let mut k = kernel_with_schedule();
        k.set_next_partition(0);
        k.active_partition = Some(0);
        for _ in 0..4 {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        pcb1.set_sleep_until(9999);
        svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// ticks_dropped starts at zero on a new kernel.
    #[test]
    fn ticks_dropped_initially_zero() {
        let k = kernel_with_schedule();
        assert_eq!(k.ticks_dropped(), 0);
    }

    /// increment_ticks_dropped increases the counter by one.
    #[test]
    fn increment_ticks_dropped_adds_one() {
        let mut k = kernel_with_schedule();
        k.increment_ticks_dropped();
        assert_eq!(k.ticks_dropped(), 1);
        k.increment_ticks_dropped();
        assert_eq!(k.ticks_dropped(), 2);
    }

    /// increment_ticks_dropped saturates at u32::MAX.
    #[test]
    fn increment_ticks_dropped_saturates() {
        let mut k = kernel_with_schedule();
        k.ticks_dropped = u32::MAX;
        k.increment_ticks_dropped();
        assert_eq!(k.ticks_dropped(), u32::MAX);
    }

    /// reset_ticks_dropped returns previous value and zeroes the counter.
    #[test]
    fn reset_ticks_dropped_returns_prev_and_zeroes() {
        let mut k = kernel_with_schedule();
        k.increment_ticks_dropped();
        k.increment_ticks_dropped();
        k.increment_ticks_dropped();
        assert_eq!(k.reset_ticks_dropped(), 3);
        assert_eq!(k.ticks_dropped(), 0);
    }

    /// reset_ticks_dropped returns zero when counter is already zero.
    #[test]
    fn reset_ticks_dropped_when_already_zero() {
        let mut k = kernel_with_schedule();
        assert_eq!(k.reset_ticks_dropped(), 0);
        assert_eq!(k.ticks_dropped(), 0);
    }

    /// Tests advance_schedule_tick switches to Ready partitions normally.
    #[test]
    fn advance_schedule_tick_switches_to_ready_partition() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );

        // Advance to P1's slot boundary
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // Switch to P1 (Ready partition)
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
        assert_eq!(k.active_partition(), Some(1));
    }

    /// Verifies that advance_schedule_tick transitions the outgoing partition
    /// from Running to Ready before switching to the incoming partition.
    #[test]
    fn advance_schedule_tick_transitions_outgoing_to_ready() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Put P0 into Running via set_next_partition.
        k.set_next_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        k.active_partition = Some(0);

        // Advance to the P1 slot boundary (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // 5th tick triggers switch to P1.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );

        // P0 should now be Ready, P1 should be Running.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );

        // At most one Running partition.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Multi-cycle consistency: exercises 3+ consecutive partition switches
    /// (P0→P1→P0→P1) via advance_schedule_tick to catch regressions where
    /// transition_outgoing_ready works on the first switch but fails on
    /// subsequent ones due to stale state.
    #[test]
    fn advance_schedule_tick_multi_cycle_consistency() {
        use crate::invariants::{
            assert_partition_state_consistency, assert_running_matches_active,
        };
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Bootstrap: put P0 into Running so the first switch has an outgoing.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );

        // Schedule layout: P0(5 ticks) → P1(3 ticks) [→ SystemWindow(1)]
        // We will drive through: switch1 P0→P1, switch2 P1→P0, switch3 P0→P1.
        struct Boundary {
            interior_ticks: u32,
            outgoing: u8,
            incoming: u8,
        }
        #[cfg(not(feature = "dynamic-mpu"))]
        let boundaries = [
            Boundary {
                interior_ticks: 4,
                outgoing: 0,
                incoming: 1,
            }, // P0→P1
            Boundary {
                interior_ticks: 2,
                outgoing: 1,
                incoming: 0,
            }, // P1→P0
            Boundary {
                interior_ticks: 4,
                outgoing: 0,
                incoming: 1,
            }, // P0→P1
        ];
        #[cfg(feature = "dynamic-mpu")]
        let boundaries = [
            Boundary {
                interior_ticks: 4,
                outgoing: 0,
                incoming: 1,
            }, // P0→P1
            // P1(3 ticks) then SystemWindow(1 tick) before P0
            Boundary {
                interior_ticks: 2,
                outgoing: 1,
                incoming: 0,
            }, // P1→SW→P0
            Boundary {
                interior_ticks: 4,
                outgoing: 0,
                incoming: 1,
            }, // P0→P1
        ];

        for (i, b) in boundaries.iter().enumerate() {
            // Advance through interior ticks (no switch expected).
            for _ in 0..b.interior_ticks {
                let ev = svc_scheduler::advance_schedule_tick(&mut k);
                assert!(
                    ev != ScheduleEvent::PartitionSwitch(b.incoming),
                    "unexpected early switch at boundary {i}"
                );
            }

            // On dynamic-mpu, the P1→P0 boundary has a SystemWindow tick first.
            #[cfg(feature = "dynamic-mpu")]
            if i == 1 {
                assert_eq!(
                    svc_scheduler::advance_schedule_tick(&mut k),
                    ScheduleEvent::SystemWindow
                );
            }

            // The next tick triggers the partition switch.
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::PartitionSwitch(b.incoming),
                "expected switch to P{} at boundary {i}",
                b.incoming
            );

            // (1) Outgoing partition must be Ready.
            assert_eq!(
                k.partitions().get(b.outgoing as usize).unwrap().state(),
                PartitionState::Ready,
                "outgoing P{} not Ready at boundary {i}",
                b.outgoing
            );
            // (2) Incoming partition must be Running.
            assert_eq!(
                k.partitions().get(b.incoming as usize).unwrap().state(),
                PartitionState::Running,
                "incoming P{} not Running at boundary {i}",
                b.incoming
            );
            // (3) At most one Running partition.
            assert_partition_state_consistency(k.partitions().as_slice());
            // (4) Running partition matches active_partition.
            assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
        }
    }

    /// Bug 06 regression for advance_schedule_tick: P0 active+Waiting,
    /// P1 Waiting. Tick reaches schedule boundary nominating P1 (Waiting),
    /// guard restores P0 to Running via set_next_partition.
    #[test]
    fn bug06_advance_tick_restores_active_when_waiting() {
        use crate::invariants::{
            assert_next_partition_not_waiting, assert_partition_state_consistency,
            assert_running_matches_active,
        };
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance to 1 tick before P1's slot boundary (P0 has 5 ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }

        // P0: Running → Waiting (simulates blocking syscall).
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // P1: Ready → Running → Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // 5th tick triggers schedule boundary nominating P1 (Waiting).
        // Bug 06 guard should restore P0 to Running.
        let ev = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(ev, ScheduleEvent::None);

        // P0 must be restored to Running.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));

        // P1 still Waiting.
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );

        // Invariant checks.
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
        assert_next_partition_not_waiting(k.partitions().as_slice(), k.next_partition());
    }

    /// Regression: SYS_YIELD dispatch sets yield_requested only when the
    /// current partition is Running (not Ready or Waiting).
    #[test]
    fn dispatch_yield_sets_flag_when_partition_running() {
        let mut k = kernel(0, 0, 0);
        k.active_partition = Some(0);
        // Fix invariant: only P0 should be Running.
        try_transition(k.partitions_mut(), 1, PartitionState::Ready);
        // kernel() creates partitions via tbl(), which transitions them to Running.
        assert_eq!(
            k.partitions()
                .get(k.current_partition() as usize)
                .unwrap()
                .state(),
            PartitionState::Running,
        );
        assert!(!k.yield_requested());
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert!(k.yield_requested());
    }

    /// Regression: a Waiting partition must not execute when its schedule
    /// slot arrives. The scheduler returns None and preserves the previous
    /// active/next partition values throughout the entire skipped slot.
    #[test]
    fn waiting_partition_skipped_for_entire_slot() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Schedule: P0(5 ticks), P1(3 ticks). Partitions start Ready.

        // Advance through P0's slot (4 interior ticks).
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // Before P1's slot: transition P1 Ready -> Running -> Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );

        let prev_active = k.active_partition();
        let prev_next = k.next_partition();

        // Tick 5 would switch to P1 — must be suppressed.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );
        assert_eq!(k.active_partition(), prev_active);
        assert_eq!(k.next_partition(), prev_next);

        // Remaining 2 ticks of P1's slot: still no switch.
        for _ in 0..2 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
            assert_eq!(k.active_partition(), prev_active);
        }
    }

    /// Regression: once a Waiting partition transitions back to Ready, it
    /// must be scheduled normally when its next slot comes up.
    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn waiting_partition_resumes_after_transition_to_ready() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Schedule: P0(5 ticks), P1(3 ticks), major frame = 8 ticks.

        // Advance 4 ticks into P0's first slot.
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // Put P1 into Waiting before its slot.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // Tick 5: P1's slot boundary — skipped because Waiting.
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::None
        );

        // Advance through rest of P1's slot (2 ticks) + P0's next slot (5 ticks).
        // Total: 7 more ticks to reach P1's next slot boundary.
        for _ in 0..7 {
            svc_scheduler::advance_schedule_tick(&mut k);
        }

        // Unblock P1: Waiting -> Ready.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Ready).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );

        // Next tick enters P1's second slot — should switch to P1.
        // We are at tick 12, which is 4 ticks into P0's second slot.
        // Need to reach P1's next boundary at tick 13 (5+3+5 = 13).
        // Actually let me recalculate: we did 4+1+7 = 12 ticks total.
        // Major frame = 8, so 12 mod 8 = 4 ticks into 2nd major frame.
        // P0 slot ends at tick 5 of each frame. We need 1 more tick.

        // The schedule wraps: P0(5), P1(3), P0(5), P1(3)...
        // Tick 1-4: interior of P0 slot 1
        // Tick 5: switch to P1 (skipped)
        // Tick 6-7: interior of P1 slot
        // Tick 8: switch to P0 (wraps to slot 0)
        // Tick 9-12: interior of P0 slot 2
        // Tick 13: switch to P1

        // We've done 12 ticks. One more should switch to P1.
        let event = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(event, ScheduleEvent::PartitionSwitch(1));
        assert_eq!(k.active_partition(), Some(1));
    }

    /// Regression: multiple simultaneously Waiting partitions must all be
    /// skipped. Only non-Waiting partitions produce PartitionSwitch events.
    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn multi_partition_waiting_scheduler_skip() {
        use crate::scheduler::ScheduleEvent;

        // Build a 4-partition schedule: P0(3), P1(2), P2(2), P3(3) = 10 ticks
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 3)).unwrap();
        schedule.add(ScheduleEntry::new(1, 2)).unwrap();
        schedule.add(ScheduleEntry::new(2, 2)).unwrap();
        schedule.add(ScheduleEntry::new(3, 3)).unwrap();
        schedule.start();

        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mut stk2 = AlignedStack1K::default();
        let mut stk3 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk2,
                0x0800_2001,
                MpuRegion::new(0x2000_2000, 4096, 0),
                2,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk3,
                0x0800_3001,
                MpuRegion::new(0x2000_3000, 4096, 0),
                3,
            )
            .unwrap(),
        ];
        let mut k = kernel_from_ext(schedule, &mems);

        // Transition P1 and P2 to Waiting (Ready -> Running -> Waiting).
        for pid in [1u8, 2] {
            let pcb = k.partitions_mut().get_mut(pid as usize).unwrap();
            pcb.transition(PartitionState::Running).unwrap();
            pcb.transition(PartitionState::Waiting).unwrap();
            assert_eq!(pcb.state(), PartitionState::Waiting);
        }

        // Walk through a full major frame (10 ticks) and record events.
        // Schedule layout:
        //   tick 1-2: interior of P0 slot (None)
        //   tick 3:   boundary -> P1 slot (Waiting -> None)
        //   tick 4:   interior of P1 slot (None)
        //   tick 5:   boundary -> P2 slot (Waiting -> None)
        //   tick 6:   interior of P2 slot (None)
        //   tick 7:   boundary -> P3 slot (PartitionSwitch(3))
        //   tick 8-9: interior of P3 slot (None)
        //   tick 10:  boundary -> P0 slot (PartitionSwitch(0)) [wraps]
        let mut events = [ScheduleEvent::None; 10];
        for event in &mut events {
            *event = svc_scheduler::advance_schedule_tick(&mut k);
            // active_partition must never point to a Waiting partition.
            if let Some(ap) = k.active_partition() {
                assert!(ap != 1 && ap != 2, "active_partition was set to P{ap}");
            }
        }

        // P0 slot (3 ticks): ticks 1-2 interior, tick 3 = boundary to P1.
        assert_eq!(events[0], ScheduleEvent::None);
        assert_eq!(events[1], ScheduleEvent::None);
        // P1 boundary (Waiting -> skipped).
        assert_eq!(events[2], ScheduleEvent::None, "P1 slot must be skipped");
        // P1 interior tick.
        assert_eq!(events[3], ScheduleEvent::None);
        // P2 boundary (Waiting -> skipped).
        assert_eq!(events[4], ScheduleEvent::None, "P2 slot must be skipped");
        // P2 interior tick.
        assert_eq!(events[5], ScheduleEvent::None);
        // P3 boundary (Ready -> PartitionSwitch).
        assert_eq!(
            events[6],
            ScheduleEvent::PartitionSwitch(3),
            "P3 slot must produce PartitionSwitch(3)"
        );
        // P3 interior ticks.
        assert_eq!(events[7], ScheduleEvent::None);
        assert_eq!(events[8], ScheduleEvent::None);
        // Major frame wraps -> P0 boundary (Ready -> PartitionSwitch).
        assert_eq!(
            events[9],
            ScheduleEvent::PartitionSwitch(0),
            "P0 slot must produce PartitionSwitch(0) on wrap"
        );
        // Final state: active_partition must be P0 (last successful switch).
        assert_eq!(k.active_partition(), Some(0));
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    fn ticks_since_bottom_half_tracks_staleness() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();
        // Initially 0
        assert_eq!(k.ticks_since_bottom_half, 0);
        // Schedule: P0(5 ticks), P1(3 ticks), SystemWindow(1 tick)
        // Advance through P0's slot (5 ticks) - counter increments each tick
        for i in 1..=5 {
            let event = svc_scheduler::advance_schedule_tick(&mut k);
            if i < 5 {
                assert_eq!(event, ScheduleEvent::None);
            } else {
                assert_eq!(event, ScheduleEvent::PartitionSwitch(1));
            }
            assert_eq!(k.ticks_since_bottom_half, i);
        }
        // Advance through P1's slot (3 ticks)
        for i in 6..=8 {
            let event = svc_scheduler::advance_schedule_tick(&mut k);
            if i < 8 {
                assert_eq!(event, ScheduleEvent::None);
            } else {
                assert_eq!(event, ScheduleEvent::SystemWindow);
            }
            if i < 8 {
                assert_eq!(k.ticks_since_bottom_half, i);
            } else {
                // SystemWindow resets to 0
                assert_eq!(k.ticks_since_bottom_half, 0);
            }
        }
        // Continue advancing - counter should increment again from 0
        let event = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(event, ScheduleEvent::PartitionSwitch(0));
        assert_eq!(k.ticks_since_bottom_half, 1);
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn bottom_half_stale_flag_set_when_threshold_exceeded() {
        // Create a kernel with a schedule that has NO system window.
        // This allows ticks_since_bottom_half to exceed the threshold.
        // Note: new() would fail validation if system window gap > threshold,
        // so we use new_empty() and manually set up the schedule.
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Initially both counter and flag are at default values
        assert_eq!(k.ticks_since_bottom_half, 0);
        assert!(!k.is_bottom_half_stale());

        // Advance ticks up to the threshold - flag should remain false
        for i in 1..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(k.ticks_since_bottom_half, i);
            assert!(
                !k.is_bottom_half_stale(),
                "flag should be false at tick {}",
                i
            );
        }

        // Advance one more tick past the threshold - flag should become true
        svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(
            k.ticks_since_bottom_half,
            TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS + 1
        );
        assert!(
            k.is_bottom_half_stale(),
            "flag should be true after exceeding threshold"
        );

        // Flag should remain true on subsequent ticks
        svc_scheduler::advance_schedule_tick(&mut k);
        assert!(k.is_bottom_half_stale());
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn clear_bottom_half_stale_resets_flag() {
        // Set up a kernel where the stale flag can be triggered
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Initially flag is false
        assert!(!k.is_bottom_half_stale());

        // Advance past threshold to set the flag
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        assert!(
            k.is_bottom_half_stale(),
            "flag should be true after exceeding threshold"
        );

        // Clear the flag
        k.clear_bottom_half_stale();
        assert!(
            !k.is_bottom_half_stale(),
            "flag should be false after clear"
        );

        // Advancing more ticks past threshold should set it again
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        assert!(
            k.is_bottom_half_stale(),
            "flag should be true again after exceeding threshold"
        );
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn fallback_revoke_noop_when_not_stale() {
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Lend a buffer with an already-expired deadline
        let ds = &k.dynamic_strategy;
        k.buffers.lend_to_partition(0, 1, false, ds).unwrap();
        k.buffers.set_deadline(0, Some(0)).unwrap();

        // bottom_half_stale is false — fallback should be a no-op
        assert!(!k.is_bottom_half_stale());
        let revoked = k.fallback_revoke_expired_buffers();
        assert_eq!(revoked, 0, "should not revoke when stale flag is false");

        // Buffer should still be borrowed
        assert_eq!(
            k.buffers.get(0).unwrap().state(),
            crate::buffer_pool::BorrowState::BorrowedRead { owner: 1 },
        );
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn fallback_revoke_expired_buffers_when_stale() {
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Lend slot 0 with deadline=10
        let ds = &k.dynamic_strategy;
        k.buffers.lend_to_partition(0, 1, true, ds).unwrap();
        k.buffers.set_deadline(0, Some(10)).unwrap();

        // Advance past threshold to trigger stale flag
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        assert!(k.is_bottom_half_stale());

        // Current tick is now > 10, so buffer should be revoked
        let revoked = k.fallback_revoke_expired_buffers();
        assert_eq!(revoked, 1, "expired buffer should be revoked");
        assert_eq!(
            k.buffers.get(0).unwrap().state(),
            crate::buffer_pool::BorrowState::Free,
        );
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn fallback_revoke_skips_non_expired_buffers() {
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Lend slot 0 with a deadline far in the future
        let ds = &k.dynamic_strategy;
        k.buffers.lend_to_partition(0, 1, false, ds).unwrap();
        k.buffers.set_deadline(0, Some(999_999)).unwrap();

        // Advance past threshold to trigger stale flag
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        assert!(k.is_bottom_half_stale());

        // Current tick (~101) is well below deadline (999_999)
        let revoked = k.fallback_revoke_expired_buffers();
        assert_eq!(revoked, 0, "non-expired buffer should not be revoked");
        assert_eq!(
            k.buffers.get(0).unwrap().state(),
            crate::buffer_pool::BorrowState::BorrowedRead { owner: 1 },
        );
    }

    /// Verify that systick_handler calls fallback_revoke_expired_buffers
    /// during tick processing when bottom_half_stale is set, revoking
    /// buffers with expired deadlines even without a SystemWindow event.
    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn systick_handler_fallback_revokes_expired_buffers() {
        use crate::buffer_pool::BorrowState;

        // Create kernel with no system window so stale flag will trigger.
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut().add(ScheduleEntry::new(0, 50)).unwrap();
        k.schedule_mut().add(ScheduleEntry::new(1, 60)).unwrap();
        k.schedule_mut().start();

        // Lend slot 0 with deadline=10 (will expire quickly).
        let ds = &k.dynamic_strategy;
        k.buffers.lend_to_partition(0, 1, true, ds).unwrap();
        k.buffers.set_deadline(0, Some(10)).unwrap();

        // Confirm buffer is currently borrowed.
        assert_eq!(
            k.buffers.get(0).unwrap().state(),
            BorrowState::BorrowedWrite { owner: 1 },
        );

        // Drive ticks past the stale threshold via systick_handler.
        // Threshold is 100 ticks; deadline is 10. After 101 ticks,
        // stale flag is set and deadline is long past.
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            crate::tick::systick_handler::<TestConfig>(&mut k);
        }

        // The fallback path inside systick_handler should have revoked
        // the expired buffer.
        assert_eq!(
            k.buffers.get(0).unwrap().state(),
            BorrowState::Free,
            "expired buffer should be revoked by fallback in systick_handler",
        );
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    fn query_bottom_half_returns_ticks_in_r0() {
        let mut k = kernel_with_schedule();
        // Initially both values are zero
        let mut ef = frame(crate::syscall::SYS_QUERY_BOTTOM_HALF, 0xAA, 0xBB);
        // SAFETY: `ef` is a valid stack-local ExceptionFrame with initialized r0-r3
        // fields. `k` is a properly constructed Kernel with partitions in Running
        // state. Single-threaded test execution prevents data races.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "r0 should contain ticks_since_bottom_half (0)");
        assert_eq!(ef.r1, 0, "r1 should contain stale flag (false -> 0)");

        // Advance a few ticks and check again
        svc_scheduler::advance_schedule_tick(&mut k);
        svc_scheduler::advance_schedule_tick(&mut k);
        let mut ef2 = frame(crate::syscall::SYS_QUERY_BOTTOM_HALF, 0, 0);
        // SAFETY: `ef2` is a valid stack-local ExceptionFrame with initialized r0-r3
        // fields. `k` remains a valid Kernel. Single-threaded test execution.
        unsafe { k.dispatch(&mut ef2) };
        assert_eq!(ef2.r0, 2, "r0 should contain ticks_since_bottom_half (2)");
        assert_eq!(ef2.r1, 0, "r1 should contain stale flag (false -> 0)");
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    #[allow(deprecated)]
    fn query_bottom_half_returns_stale_flag_in_r1() {
        // Use new_empty so we can exceed the threshold without system window reset
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());
        k.schedule_mut()
            .add(ScheduleEntry::new(
                0,
                TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS + 10,
            ))
            .unwrap();
        k.schedule_mut().start();

        // Advance past the threshold to trigger the stale flag
        for _ in 0..=TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        assert!(k.is_bottom_half_stale(), "flag should be set");

        let mut ef = frame(crate::syscall::SYS_QUERY_BOTTOM_HALF, 0, 0);
        // SAFETY: `ef` is a valid stack-local ExceptionFrame with initialized r0-r3
        // fields. `k` is a properly constructed Kernel (via new_empty) with a valid
        // schedule. Single-threaded test execution prevents data races.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(
            ef.r0,
            TestConfig::SYSTEM_WINDOW_MAX_GAP_TICKS + 1,
            "r0 should contain ticks_since_bottom_half"
        );
        assert_eq!(ef.r1, 1, "r1 should contain stale flag (true -> 1)");
    }

    #[test]
    fn yield_current_slot_advances_to_next_partition() {
        let mut k = kernel_with_schedule();
        // Consume 2 ticks in slot 0
        svc_scheduler::advance_schedule_tick(&mut k);
        svc_scheduler::advance_schedule_tick(&mut k);
        // Yield: skip remaining 3 ticks, advance to P1
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));
        assert_eq!(k.active_partition(), Some(1));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
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

    /// Tests yield_current_slot skips Waiting partitions (returns None,
    /// doesn't update active_partition).
    #[test]
    fn yield_current_slot_skips_waiting_partition() {
        let mut k = kernel_with_schedule();
        let initial_active = k.active_partition();

        // Transition P1 to Waiting (Ready -> Running -> Waiting)
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // Yield from P0 slot: force_advance returns P1, but P1 is Waiting
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert_eq!(k.active_partition(), initial_active);
    }

    /// Verifies that skipping a Waiting partition on yield does NOT
    /// transition the current active partition from Running to Ready.
    /// Covers the early-return at svc.rs:2301-2302.
    #[test]
    fn yield_current_slot_skips_waiting_preserves_running_state() {
        let mut k = kernel_with_schedule();

        // Make P0 the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );

        // Transition P1 to Waiting (Ready -> Running -> Waiting).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // Yield from P0: force_advance targets P1, but P1 is Waiting => skip.
        // yield_current_slot returns `impl YieldResult` (opaque), so verify
        // ScheduleEvent::None semantics via both trait methods.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert!(!result.is_system_window());
        assert_eq!(k.active_partition(), Some(0));
        // P0 must still be Running — no Running→Ready transition fired.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    /// yield_current_slot does NOT increment starvation for a Waiting
    /// partition — it voluntarily blocked.
    #[test]
    fn yield_current_slot_waiting_not_starved() {
        let mut k = kernel_with_schedule();

        // Make P0 the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Transition P1 to Waiting (Ready -> Running -> Waiting).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(pcb1.starvation_count(), 0);

        // Yield from P0: force_advance targets P1, but P1 is Waiting =>
        // skipped. With only 2 partitions (P0 Running, P1 Waiting), no
        // Ready partition exists to accumulate starvation.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// yield_current_slot resets starvation count on the incoming partition
    /// when the yield succeeds (target is Ready).
    #[test]
    fn yield_current_slot_resets_starvation_on_success() {
        let mut k = kernel_with_schedule();

        // Make P0 the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Artificially give P1 a non-zero starvation count.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.increment_starvation();
        pcb1.increment_starvation();
        assert_eq!(pcb1.starvation_count(), 2);

        // Yield from P0: force_advance targets P1 which is Ready =>
        // switch succeeds, starvation resets to 0.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
    }

    /// yield_current_slot increments starvation for Ready non-active
    /// partitions when the target partition is Waiting.
    /// 3-partition schedule: P0(5), P1(3), P2(4).
    /// P0 active/Running, P1 Waiting (target), P2 Ready → P2 starvation++.
    #[test]
    fn yield_current_slot_increments_starvation_on_waiting_target() {
        let mut k = kernel_with_3_partition_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Transition P1 to Waiting (Ready -> Running -> Waiting).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P2 is Ready (default after Kernel::new).
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 0);

        // Yield from P0: force_advance targets P1 (Waiting) → skipped.
        // P2 is Ready and not active, so P2.starvation_count increments.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 1);
    }

    /// yield_current_slot does NOT increment starvation when the target
    /// partition is Ready (successful switch — no wasted slot).
    #[test]
    fn yield_current_slot_no_starvation_when_target_ready() {
        let mut k = kernel_with_3_partition_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // P1 and P2 are both Ready (default).
        assert_eq!(k.partitions().get(1).unwrap().starvation_count(), 0);
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 0);

        // Yield from P0: force_advance targets P1 (Ready) → switch succeeds.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));

        // No starvation incremented for any partition on a successful switch.
        assert_eq!(k.partitions().get(0).unwrap().starvation_count(), 0);
        assert_eq!(k.partitions().get(2).unwrap().starvation_count(), 0);
    }

    /// Tests that yield_current_slot transitions the outgoing partition
    /// from Running to Ready before switching to the next partition.
    #[test]
    fn yield_current_slot_transitions_outgoing_to_ready() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Put P0 into Running via set_next_partition and mark it active.
        k.set_next_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        k.active_partition = Some(0);

        // Yield: should advance to P1 and transition P0 Running→Ready.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));

        // P0 should now be Ready.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );

        // At most one Running partition.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    #[test]
    fn transition_outgoing_ready_transitions_running_to_ready() {
        let mut k = kernel_with_schedule();

        // Put P0 into Running via set_next_partition and mark it active.
        k.set_next_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        k.active_partition = Some(0);

        // Call the helper directly.
        svc_scheduler::transition_outgoing_ready(&mut k);

        // P0 should now be Ready.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    fn transition_outgoing_ready_noop_when_no_active_partition() {
        let mut k = kernel_with_schedule();

        // Ensure active_partition is None (default after construction).
        assert_eq!(k.active_partition, None);

        // Call the helper — should not panic or change any state.
        svc_scheduler::transition_outgoing_ready(&mut k);

        // Both partitions should remain in their initial state (Ready).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
    }

    #[test]
    #[should_panic(expected = "2 partitions in Running state")]
    fn transition_outgoing_ready_asserts_on_double_running() {
        let mut k = kernel_with_schedule();

        // Force both P0 and P1 into Running (violating the invariant).
        k.set_next_partition(0);
        k.set_next_partition(1);

        // With active_partition = None the transition is a no-op,
        // so both partitions remain Running and the debug_assert fires.
        k.active_partition = None;
        svc_scheduler::transition_outgoing_ready(&mut k);
    }

    #[test]
    fn transition_outgoing_ready_noop_when_active_partition_waiting() {
        let mut k = kernel_with_schedule();

        // Put P0 into Running, then block it (Running → Waiting).
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // transition_outgoing_ready should be a no-op: P0 is Waiting, not Running.
        svc_scheduler::transition_outgoing_ready(&mut k);

        // P0 must remain Waiting — not spuriously moved to Ready.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
    }

    #[test]
    fn transition_outgoing_ready_at_major_frame_boundary() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Schedule: P0(5 ticks) | P1(3 ticks). Major frame = 8 ticks.
        // Make P0 Running in slot 0.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Advance 5 ticks to exhaust P0's slot → switch to P1.
        for _ in 0..4 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(ev.partition_id(), None);
        }
        let ev = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(ev.partition_id(), Some(1));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );

        // Advance 3 ticks to exhaust P1's slot → wrap to slot 0 (P0).
        for _ in 0..2 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(ev.partition_id(), None);
        }
        // With dynamic-mpu, the system window sits between P1 and P0.
        #[cfg(feature = "dynamic-mpu")]
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::SystemWindow
        );
        // Next tick triggers major frame boundary wrap-around: P1 → P0.
        let ev = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(ev.partition_id(), Some(0));

        // P1 (outgoing) transitioned Running → Ready at major frame boundary.
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
        // P0 (incoming) is now Running.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    #[test]
    fn transition_after_yield_at_end_of_major_frame() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Schedule: P0(5 ticks) | P1(3 ticks). Major frame = 8 ticks.
        // Make P0 Running in slot 0, then advance to P1's slot.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        for _ in 0..5 {
            svc_scheduler::advance_schedule_tick(&mut k);
        }
        // P1 is now Running in the last slot before wrap-around.
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(1));

        // Yield in the last slot — force_advance wraps to slot 0 (P0).
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(0));

        // P1 (outgoing) transitioned Running → Ready via wrap-around yield.
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );
        // P0 is now the active partition.
        assert_eq!(k.active_partition(), Some(0));
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: trigger_deschedule() then yield_current_slot()
    /// when no runnable partner exists must leave the caller Running.
    ///
    /// Scenario: P0 is Running, P1 is Waiting. trigger_deschedule() sets
    /// yield_requested but defers the Running→Ready transition.
    /// yield_current_slot() sees the next slot targets a Waiting partition,
    /// so it returns None and must NOT demote P0 to Ready.
    #[test]
    fn bug05_trigger_deschedule_then_yield_no_partner_preserves_running() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Transition P1 to Waiting first (Ready → Running → Waiting)
        // so that promoting P0 to Running never violates at-most-one-Running.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Now set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 1: trigger_deschedule() — sets yield_requested, keeps P0 Running.
        let ret = k.trigger_deschedule();
        assert_eq!(ret, 0, "trigger_deschedule must return 0");
        assert!(
            k.yield_requested(),
            "yield_requested must be set after trigger_deschedule"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "P0 must remain Running after trigger_deschedule"
        );

        // Step 2: yield_current_slot() — no runnable partner, returns None.
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // yield_requested must still be set: yield_current_slot() does NOT
        // clear the flag — only the dispatch hook does.
        assert!(
            k.yield_requested(),
            "yield_requested must remain set after yield_current_slot (cleared only by dispatch hook)"
        );

        // P0 must still be Running — this is the Bug 05 invariant.
        // TODO: we verify P0 is Running before and after yield_current_slot();
        // mid-call state cannot be observed in a unit test without instrumentation.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must stay Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (criterion #5): a blocking syscall (SYS_EVT_WAIT
    /// with no events set) transitions P0 to Waiting, then
    /// yield_current_slot() finds no runnable partner and returns None.
    /// P0 must remain Waiting — not revert to Ready.
    #[test]
    fn bug05_blocking_syscall_stays_waiting_after_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Transition P1 to Waiting (Ready → Running → Waiting)
        // so yield_current_slot() finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Set up P0 as active Running partition.
        k.set_current_partition(0);
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Dispatch blocking SYS_EVT_WAIT (no events set → blocks, returns 0).
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
        assert!(
            k.yield_requested(),
            "blocking EventWait must set yield_requested"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking EventWait"
        );

        // Simulate harness yield handling: clear flag, call yield_current_slot().
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Bug 05: P0 must be restored to Running — no runnable partner.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (criteria #3/#6): dispatch SYS_YIELD, simulate
    /// harness yield handling, then dispatch SYS_GET_TIME. The second
    /// dispatch calls assert_dispatch_invariants at entry — if Bug 05
    /// were present, P0 would be Ready (not Running) and the invariant
    /// check would panic with "active 0 is Ready".
    #[test]
    fn bug05_dispatch_yield_then_second_svc_no_invariant_panic() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Transition P1 to Waiting (Ready → Running → Waiting).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // Set up P0 as active Running partition with matching current_partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 1: Dispatch SYS_YIELD — goes through assert_dispatch_invariants.
        let mut ef = frame(SYS_YIELD, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "SYS_YIELD must return 0");
        assert!(k.yield_requested(), "SYS_YIELD must set yield_requested");

        // Step 2: Simulate harness yield handling.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // P0 must still be Running after yield found no partner.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "P0 must stay Running after yield with no partner"
        );

        // Step 3: Dispatch SYS_GET_TIME — assert_dispatch_invariants runs
        // at entry. If Bug 05 were present, P0 would be Ready and the
        // invariant would panic.
        let mut ef2 = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef2) };
        // GET_TIME returns the current tick (0 for a fresh kernel).
        assert_eq!(ef2.r0, 0, "SYS_GET_TIME must return current tick low word");

        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (tick-driven path): advance_schedule_tick() must
    /// skip a PartitionSwitch when the target partition is Waiting.
    ///
    /// Scenario: P0 (5 ticks) is Running, P1 (3 ticks) is Waiting.
    /// Advance 5 ticks to exhaust slot 0. The schedule table yields
    /// PartitionSwitch(1), but advance_schedule_tick must detect P1 is
    /// Waiting and return ScheduleEvent::None instead, leaving P0
    /// Running and active_partition unchanged.
    #[test]
    fn bug05_advance_schedule_tick_skips_waiting_target() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::partition::PartitionState;
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Both partitions start Ready (kernel_with_schedule default).
        // Transition P1 to Waiting (Ready → Running → Waiting) first,
        // while P0 is still Ready, so at-most-one-Running is never violated.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // set_next_partition() transitions P0 Ready → Running internally
        // (see Kernel::set_next_partition which calls try_transition).
        k.set_next_partition(0);
        // TODO: active_partition is a public field; direct assignment is the
        // established test pattern (no setter API exists).
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Advance 4 ticks — all within slot 0 (P0 has 5 ticks).
        for i in 0..4 {
            let event = svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(
                event,
                ScheduleEvent::None,
                "tick {}: must be None (still within P0 slot)",
                i + 1
            );
        }

        // 5th tick exhausts slot 0 → schedule table would return
        // PartitionSwitch(1), but P1 is Waiting so advance_schedule_tick
        // must return None and skip the switch.
        let event = svc_scheduler::advance_schedule_tick(&mut k);
        assert_eq!(
            event,
            ScheduleEvent::None,
            "Bug 05: advance_schedule_tick must return None when target P1 is Waiting"
        );

        // P0 must still be Running — not demoted to Ready.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must stay Running when tick-driven switch skips Waiting target"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (idempotency): two consecutive trigger_deschedule()
    /// calls without an intervening yield_current_slot() must not introduce
    /// hidden side-effects.
    #[test]
    fn bug05_trigger_deschedule_idempotent_double_call() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // P1 → Waiting so yield finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_partition_state_consistency(k.partitions().as_slice());

        // First trigger_deschedule.
        let ret1 = k.trigger_deschedule();
        assert_eq!(ret1, 0, "first trigger_deschedule must return 0");
        assert!(k.yield_requested(), "yield_requested after first call");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Second trigger_deschedule — must be idempotent.
        let ret2 = k.trigger_deschedule();
        assert_eq!(ret2, 0, "second trigger_deschedule must return 0");
        assert!(k.yield_requested(), "yield_requested after second call");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (consecutive block): block→yield→wake→block→yield
    /// cycle must preserve invariants. P0 blocks via SYS_EVT_WAIT, yield
    /// finds no partner, woken via event_set, promoted to Running, blocks
    /// again via SYS_SEM_WAIT, yield again finds no partner.
    #[test]
    fn bug05_consecutive_block_no_partner_preserves_invariant() {
        use crate::invariants::assert_partition_state_consistency;
        let mut k = kernel_with_schedule();

        // Semaphore with count 0 so SYS_SEM_WAIT will block.
        k.semaphores_mut().add(Semaphore::new(0, 1)).unwrap();

        // P1 → Waiting so yield finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 1: SYS_EVT_WAIT — P0 blocks (no events set).
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 2: Simulate harness yield (no partner).
        k.set_yield_requested(false);
        let r1 = k.yield_current_slot();
        assert_eq!(r1.partition_id(), None, "no runnable partner");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running // Bug 05: restored from Waiting
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 3: event_set — P0 is already Running, no state change.
        let ret = crate::events::event_set(k.partitions_mut(), 0, 0b1010);
        assert_eq!(ret, 0, "event_set must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 4: Promote P0 back to Running.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 5: SYS_SEM_WAIT — sem count 0, P0 blocks again.
        let mut ef2 = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef2) };
        assert_eq!(ef2.r0, 0, "blocking SemWait must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 6: Simulate harness yield (no partner).
        k.set_yield_requested(false);
        let r2 = k.yield_current_slot();
        assert_eq!(r2.partition_id(), None, "no runnable partner");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running // Bug 05: restored from Waiting
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (wakeup-reblock cycle): exercises the full
    /// block→switch→wake→run→reblock→yield lifecycle.
    ///
    /// Scenario: P0 Running, P1 Ready, schedule [P0:5, P1:3].
    /// P0 blocks on SYS_SEM_WAIT. yield switches to P1. P1 signals
    /// the semaphore (wakes P0 to Ready). Tick-driven advance exhausts
    /// P1's slot, switching back to P0. P0 blocks on SYS_EVT_WAIT.
    /// yield finds P1 Ready (budget exhausted but force_advance grants
    /// a fresh slot) and switches to P1. Verifies the at-most-one-Running
    /// invariant throughout.
    #[test]
    fn bug05_wakeup_reblock_cycle_full_lifecycle() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // Semaphore with count 0 so SYS_SEM_WAIT will block.
        k.semaphores_mut().add(Semaphore::new(0, 1)).unwrap();

        // P0 as active Running partition, P1 stays Ready.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 1: P0 SYS_SEM_WAIT — blocks (Waiting).
        let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking SemWait must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking SemWait"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 2: yield_current_slot → P1 Ready, switches to P1.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1), "must switch to P1");
        assert_eq!(k.active_partition(), Some(1));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must stay Waiting (transition_outgoing_ready is no-op)"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 3: Promote P1 to Running (harness context-switch).
        k.set_next_partition(1);
        k.set_current_partition(1);
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 4: P1 SYS_SEM_SIGNAL — wakes P0 (Waiting → Ready).
        let mut ef_sig = frame(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef_sig) };
        assert_eq!(ef_sig.r0, 0, "SemSignal must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must be Ready after sem signal wake"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 5: Advance ticks to exhaust P1's slot → switch to P0.
        // With dynamic-mpu the schedule is [P0:5, P1:3, SW:1], so
        // after P1's 3 ticks we hit a SystemWindow before P0.
        // TODO: cfg-split is driven by kernel_with_schedule() adding
        // add_system_window(1) under dynamic-mpu. If schedule layout
        // changes, these tick counts must be re-derived from the layout.
        for i in 0..2 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(ev, ScheduleEvent::None, "tick {}: interior", i + 1);
        }
        let ev = svc_scheduler::advance_schedule_tick(&mut k);
        #[cfg(feature = "dynamic-mpu")]
        assert_eq!(
            ev,
            ScheduleEvent::SystemWindow,
            "3rd tick hits system window"
        );
        #[cfg(not(feature = "dynamic-mpu"))]
        assert_eq!(
            ev,
            ScheduleEvent::PartitionSwitch(0),
            "3rd tick switches to P0"
        );
        #[cfg(feature = "dynamic-mpu")]
        {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_eq!(
                ev,
                ScheduleEvent::PartitionSwitch(0),
                "4th tick switches to P0"
            );
        }
        assert_eq!(k.active_partition(), Some(0));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready,
            "P1 transitioned to Ready by transition_outgoing_ready"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 6: Harness context-switch — advance_schedule_tick already
        // promoted P0 to Running via set_next_partition(0); update current.
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Ready,
            "P1 must remain Ready (budget exhausted, not Waiting)"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 7: P0 SYS_EVT_WAIT — no events set, blocks again (Waiting).
        let mut ef2 = frame(crate::syscall::SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef2) };
        assert_eq!(ef2.r0, 0, "blocking EventWait must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking EventWait"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // Step 8: yield_current_slot — P1 is Ready, force_advance gives
        // P1 a fresh slot and yield switches to it.
        // TODO: yield_current_slot does not currently check per-frame budget;
        // force_advance always grants a fresh slot.  If per-frame budget
        // tracking is added, this assertion should change to expect None.
        k.set_yield_requested(false);
        let r2 = k.yield_current_slot();
        assert_eq!(
            r2.partition_id(),
            Some(1),
            "P1 is Ready — yield must switch to it"
        );
        assert_eq!(k.active_partition(), Some(1));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must stay Waiting — transition_outgoing_ready is no-op"
        );
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression (major-frame wrap): advancing through full major
    /// frames must not violate the at-most-one-Running invariant when the
    /// schedule wraps back to an already-Running partition while another
    /// partition is Waiting.
    #[test]
    fn bug05_major_frame_wrap_waiting_partition_invariant() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::partition::PartitionState;
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_with_schedule();

        // P1: Ready → Running → Waiting (before P0 becomes Running).
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P0: Ready → Running (explicit transition for the active partition).
        let pcb0 = k.partitions_mut().get_mut(0).unwrap();
        pcb0.transition(PartitionState::Running).unwrap();

        // Set scheduler fields to track P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_partition_state_consistency(k.partitions().as_slice());

        // Schedule: P0(5) | P1(3) [+ SystemWindow(1) with dynamic-mpu].
        //
        // Without dynamic-mpu (major frame = 8 ticks):
        //   Tick  5: P1 slot boundary → P1 Waiting → None
        //   Tick  8: P0 slot boundary → PartitionSwitch(0) (wrap #1)
        //   Tick 13: P1 slot boundary → P1 Waiting → None
        //   Tick 16: P0 slot boundary → PartitionSwitch(0) (wrap #2)
        //
        // With dynamic-mpu (major frame = 9 ticks):
        //   Tick  5: P1 slot boundary → P1 Waiting → None
        //   Tick  8: SystemWindow boundary → SystemWindow
        //   Tick  9: P0 slot boundary → PartitionSwitch(0) (wrap #1)
        //   Tick 14: P1 slot boundary → P1 Waiting → None
        //   Tick 17: SystemWindow boundary → SystemWindow
        //   Tick 18: P0 slot boundary → PartitionSwitch(0) (wrap #2)
        #[cfg(not(feature = "dynamic-mpu"))]
        let major_frame: u32 = 8;
        #[cfg(feature = "dynamic-mpu")]
        let major_frame: u32 = 9;

        let mut wrap_count = 0u32;
        for tick in 1..=major_frame * 2 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);

            // 1-based position within the current major frame.
            let offset = ((tick - 1) % major_frame) + 1;

            // Determine the exact expected event for this tick.
            #[cfg(not(feature = "dynamic-mpu"))]
            let expected = match offset {
                5 => ScheduleEvent::None,               // P1 boundary, Waiting → suppressed
                8 => ScheduleEvent::PartitionSwitch(0), // major-frame wrap
                _ => ScheduleEvent::None,
            };
            #[cfg(feature = "dynamic-mpu")]
            let expected = match offset {
                5 => ScheduleEvent::None, // P1 boundary, Waiting → suppressed
                8 => ScheduleEvent::SystemWindow,
                9 => ScheduleEvent::PartitionSwitch(0), // major-frame wrap
                _ => ScheduleEvent::None,
            };

            assert_eq!(
                ev, expected,
                "tick {tick} (offset {offset}): unexpected event"
            );

            if ev == ScheduleEvent::PartitionSwitch(0) {
                wrap_count += 1;
            }

            // P0 must stay Running throughout.
            assert_eq!(
                k.partitions().get(0).unwrap().state(),
                PartitionState::Running,
                "tick {tick}: P0 must remain Running"
            );
            // P1 must stay Waiting throughout (no wakeup source).
            assert_eq!(
                k.partitions().get(1).unwrap().state(),
                PartitionState::Waiting,
                "tick {tick}: P1 must remain Waiting"
            );
            assert_eq!(
                k.active_partition(),
                Some(0),
                "tick {tick}: active_partition must remain P0"
            );
            assert_partition_state_consistency(k.partitions().as_slice());
        }

        // Exactly 2 major-frame wraps must have occurred at the expected ticks.
        assert_eq!(
            wrap_count, 2,
            "expected exactly 2 major-frame wraps, got {wrap_count}"
        );
    }

    /// Bug 05 regression: SYS_EVT_WAIT with no matching event bits blocks
    /// P0 via trigger_deschedule(), then yield_current_slot() finds no
    /// runnable partner (P1 is Waiting).  P0 must be restored to Running.
    /// Covers the SYS_EVT_WAIT→trigger_deschedule path.
    #[test]
    fn bug05_evt_wait_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::syscall::SYS_EVT_WAIT;
        let mut k = kernel_with_schedule();

        // Step 1: Transition P1 to Waiting (Ready → Running → Waiting)
        // so yield_current_slot() finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_EVT_WAIT with mask 0b1010 — no events set,
        // so event_wait returns 0 (no matching bits) and blocks.
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
        assert!(
            k.yield_requested(),
            "blocking EventWait must set yield_requested"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking EventWait"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_MTX_LOCK blocks P0 (mutex held by P1),
    /// trigger_deschedule() fires, then yield_current_slot() finds no
    /// runnable partner (P1 is also Waiting).  P0 must be restored to
    /// Running.  Covers the SYS_MTX_LOCK→trigger_deschedule path not
    /// exercised by the other Bug 05 tests.
    #[test]
    fn bug05_mutex_lock_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::syscall::SYS_MTX_LOCK;
        let mut k = kernel_with_schedule();

        // Step 1: P1 acquires mutex 0 while active_partition is still None
        // (dispatch invariants skip when active_partition is None in tests).
        k.set_current_partition(1);
        let mut ef = frame(SYS_MTX_LOCK, 0, 1);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "uncontested MutexLock must return 1 (acquired)");
        assert_eq!(
            k.mutexes().owner(0),
            Ok(Some(1)),
            "mutex 0 must be owned by P1"
        );

        // Step 2: Transition P1 to Waiting (Ready → Running → Waiting)
        // so yield_current_slot() finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 3: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 4: P0 dispatches SYS_MTX_LOCK on mutex 0 (held by P1) → blocks.
        let mut ef = frame(SYS_MTX_LOCK, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking MutexLock must return 0");
        assert!(
            k.yield_requested(),
            "blocking MutexLock must set yield_requested"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking MutexLock"
        );

        // Step 5: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 6: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_SEM_WAIT on a semaphore with count=0 blocks
    /// the caller via trigger_deschedule(), then yield_current_slot() finds
    /// no runnable partner (P1 is Waiting).  P0 must be restored to Running.
    /// Covers the SYS_SEM_WAIT→trigger_deschedule path.
    #[test]
    fn bug05_sem_wait_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::semaphore::Semaphore;
        use crate::syscall::SYS_SEM_WAIT;
        let mut k = kernel_with_schedule();

        // Step 1: Add a semaphore with count=0 so SYS_SEM_WAIT will block.
        k.semaphores_mut().add(Semaphore::new(0, 4)).unwrap();

        // Step 2: Transition P1 to Waiting (Ready → Running → Waiting)
        // so yield_current_slot() finds no runnable partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 3: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 4: P0 dispatches SYS_SEM_WAIT on semaphore 0 (count=0) → blocks.
        let mut ef = frame(SYS_SEM_WAIT, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking SemWait must return 0");
        assert!(
            k.yield_requested(),
            "blocking SemWait must set yield_requested"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking SemWait"
        );

        // Step 5: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 6: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_MSG_SEND on a full message queue blocks the
    /// sender via trigger_deschedule(), then yield_current_slot() finds no
    /// runnable partner (P1 is Waiting).  P0 must be restored to Running.
    /// Covers the SYS_MSG_SEND→trigger_deschedule path.
    #[cfg(feature = "ipc-message")]
    #[test]
    fn bug05_msg_send_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::message::MessageQueue;
        use crate::syscall::SYS_MSG_SEND;
        let mut k = kernel_with_schedule();

        // Step 0: Create one message queue (kernel_with_schedule has none).
        k.messages_mut().add(MessageQueue::new()).unwrap();

        // Step 1: Fill message queue 0 to capacity (QD=4) via direct API.
        for i in 0..4u8 {
            let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
            assert_eq!(
                apply_send_outcome(k.partitions_mut(), outcome),
                Ok(None),
                "filling queue: message {} must enqueue without blocking",
                i
            );
        }

        // Step 2: Transition P1 to Waiting so yield finds no partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 3: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 4: Dispatch SYS_MSG_SEND on full queue → sender blocks.
        let ptr = low32_buf(0);
        // r0=SYS_MSG_SEND, r1=queue 0, r2=sender P0, r3=data pointer
        let mut ef = frame4(SYS_MSG_SEND, 0, 0, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking MsgSend must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking MsgSend"
        );
        assert!(
            k.yield_requested(),
            "blocking MsgSend must set yield_requested"
        );

        // Step 5: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 6: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_MSG_RECV on an empty message queue blocks the
    /// receiver via apply_recv_outcome → trigger_deschedule(), then
    /// yield_current_slot() finds no runnable partner (P1 is Waiting).
    /// P0 must be restored to Running.  Covers the
    /// SYS_MSG_RECV→ReceiverBlocked→trigger_deschedule path.
    #[cfg(feature = "ipc-message")]
    #[test]
    fn bug05_msg_recv_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::message::MessageQueue;
        use crate::syscall::SYS_MSG_RECV;
        let mut k = kernel_with_schedule();

        // Step 0: Create one message queue (kernel_with_schedule has none).
        // Leave it empty so SYS_MSG_RECV will block the receiver.
        k.messages_mut().add(MessageQueue::new()).unwrap();

        // Step 1: Transition P1 to Waiting so yield finds no partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_MSG_RECV on empty queue → receiver blocks.
        let ptr = low32_buf(0);
        // r0=SYS_MSG_RECV, r1=queue 0, r2=receiver P0, r3=buffer pointer
        let mut ef = frame4(SYS_MSG_RECV, 0, 0, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking MsgRecv must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking MsgRecv"
        );
        assert!(
            k.yield_requested(),
            "blocking MsgRecv must set yield_requested"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_BB_READ on an empty blackboard blocks the
    /// reader via ReaderBlocked → trigger_deschedule(), then
    /// yield_current_slot() finds no runnable partner (P1 is Waiting).
    /// P0 must be restored to Running.  Covers the
    /// SYS_BB_READ→ReaderBlocked→trigger_deschedule path.
    #[cfg(feature = "ipc-blackboard")]
    #[test]
    fn bug05_bb_read_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::syscall::SYS_BB_READ;
        let mut k = kernel_with_schedule();

        // Step 0: Create one blackboard (kernel_with_schedule has none).
        // Leave it empty so SYS_BB_READ will block the reader.
        let bb_id = k.blackboards_mut().create().unwrap();

        // Step 1: Transition P1 to Waiting so yield finds no partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_BB_READ on empty blackboard → reader blocks.
        let ptr = low32_buf(0);
        // r0=SYS_BB_READ, r1=board id, r2=timeout (>0), r3=buffer pointer
        let mut ef = frame4(SYS_BB_READ, bb_id as u32, 50, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking BbRead must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking BbRead"
        );
        assert!(
            k.yield_requested(),
            "blocking BbRead must set yield_requested"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 05 regression: SYS_QUEUING_RECV_TIMED on an empty destination
    /// queue with timeout > 0 blocks the caller (ReceiverBlocked).  After
    /// yield_current_slot() finds no runnable partner and returns None,
    /// P0 must be restored to Running.
    #[cfg(feature = "ipc-queuing")]
    #[test]
    fn bug05_queuing_recv_timed_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel_with_schedule();

        // Step 0: Create a Destination queuing port (empty queue).
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();

        // Step 1: Transition P1 to Waiting so yield finds no partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_QUEUING_RECV_TIMED on empty queue → blocks.
        // Use P0's MPU region base as the buffer pointer (same approach
        // as partition_lifecycle_roundtrip_via_blocking_ipc).
        let mpu_base = k.partitions().get(0).unwrap().mpu_region().base();
        let ptr = mpu_base as *mut u8;
        let r2 = (50u32 << 16) | 4;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking queuing recv must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking queuing recv"
        );
        assert!(
            k.yield_requested(),
            "blocking queuing recv must set yield_requested"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    #[test]
    fn bug05_queuing_send_timed_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel_with_schedule();

        // Step 0: Create a connected Source→Destination queuing port pair.
        let (src, dst) = connected_send_pair(&mut k);

        // Fill the destination queue to capacity (QD=4) so send blocks.
        for _ in 0..4 {
            k.queuing_mut()
                .get_mut(dst)
                .unwrap()
                .inject_message(1, &[0x42]);
        }

        // Step 1: Transition P1 to Waiting so yield finds no partner.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_QUEUING_SEND_TIMED on full queue → SenderBlocked.
        // Use P0's MPU region base as the data pointer.
        let mpu_base = k.partitions().get(0).unwrap().mpu_region().base();
        let ptr = mpu_base as *mut u8;
        let r2 = pack_r2(50, 1); // timeout_hi16=50, data_len_lo16=1
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, src as u32, r2, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking queuing send must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking queuing send"
        );
        assert!(
            k.yield_requested(),
            "blocking queuing send must set yield_requested"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    #[test]
    #[cfg(feature = "dynamic-mpu")]
    fn bug05_dev_read_timed_blocks_then_yield_no_partner() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED};
        let (registry, _, _) = default_registry();
        let mut k = kernel_with_registry(0, 0, 0, registry);

        // Step 0: Open device 0 so SYS_DEV_READ_TIMED can address it.
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "SYS_DEV_OPEN must succeed");

        // Step 1: Transition P1 to Waiting so yield finds no partner.
        // P1 starts as Running (from tbl()), so transition directly.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "precondition: P1 must be Waiting"
        );

        // Step 2: Set up P0 as the active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "precondition: P0 must be Running"
        );

        // Step 3: Dispatch SYS_DEV_READ_TIMED with empty RX buffer, timeout>0.
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_DEV_READ_TIMED, 0, (50 << 16) | 1, ptr as u32);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking device read must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking device read"
        );
        assert!(
            k.yield_requested(),
            "blocking device read must set yield_requested"
        );

        // Step 4: Simulate harness yield handling: clear flag, yield.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when no runnable partner exists"
        );

        // Step 5: Bug 05: P0 must be restored to Running (no runnable partner).
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 05: P0 must be restored to Running when yield finds no runnable partner"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must still be P0"
        );

        // At-most-one-Running invariant must hold.
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Bug 06 regression: P0 Running, P1 Waiting. yield_current_slot()
    /// nominates P1 (Waiting), is_waiting guard fires but active P0 is
    /// Running (not Waiting) — guard is a no-op. Verifies Bug 05
    /// behavior is preserved by the Bug 06 guard.
    #[test]
    fn bug06_yield_guard_noop_when_active_running() {
        use crate::invariants::{
            assert_partition_state_consistency, assert_running_matches_active,
        };
        let mut k = kernel_with_schedule();

        // P1: Ready → Running → Waiting
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // force_advance → P1 (Waiting), guard checks P0 (Running) → no-op.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
    }

    /// Bug 06 regression: P0 blocks (SYS_EVT_WAIT), P1 is Ready.
    /// yield nominates P1 (Ready, not Waiting) → normal switch path.
    /// Bug 06 guard never fires. P0 stays Waiting, P1 becomes Running.
    #[test]
    fn bug06_blocking_with_ready_partner_normal_switch() {
        use crate::invariants::{
            assert_partition_state_consistency, assert_running_matches_active,
        };
        use crate::syscall::SYS_EVT_WAIT;
        let mut k = kernel_with_schedule();

        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);

        // Dispatch blocking SYS_EVT_WAIT (no events → blocks).
        let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Harness yield: P1 is Ready → normal switch.
        k.set_yield_requested(false);
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), Some(1));
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.active_partition(), Some(1));

        // Harness completes switch.
        k.set_next_partition(1);
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
    }

    /// Bug 06 regression: single-partition kernel. P0 blocks, yield
    /// wraps back to P0 (Waiting), guard restores P0 to Running.
    /// Covers the hal_gpio_blink single-partition pattern.
    #[test]
    fn bug06_single_partition_blocking_no_panic() {
        use crate::invariants::{
            assert_partition_state_consistency, assert_running_matches_active,
        };

        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        schedule.start();
        let mut stk0 = AlignedStack1K::default();
        let mems = [ExternalPartitionMemory::from_aligned_stack(
            &mut stk0,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            0,
        )
        .unwrap()];
        let mut k = kernel_from_ext(schedule, &mems);

        k.set_next_partition(0);
        k.active_partition = Some(0);

        // Simulate blocking: Running → Waiting.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // force_advance wraps to P0 (Waiting), guard restores to Running.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
    }

    /// Bug 06 regression: both P0 (active) and P1 are Waiting.
    /// yield_current_slot() nominates P1 (Waiting), is_waiting guard fires,
    /// then the inner Bug 06 guard detects P0 is also Waiting, restores P0
    /// to Running via set_next_partition. Returns ScheduleEvent::None.
    #[test]
    fn bug06_yield_both_waiting_restores_active() {
        use crate::invariants::{
            assert_partition_state_consistency, assert_running_matches_active,
        };
        let mut k = kernel_with_schedule();

        // Bootstrap P0 as active Running partition.
        k.set_next_partition(0);
        k.active_partition = Some(0);

        // P0: Running → Waiting.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // P1: Ready → Running → Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();

        // force_advance → P1 (Waiting), Bug 06 guard restores P0 to Running.
        let result = k.yield_current_slot();
        assert_eq!(
            result.partition_id(),
            None,
            "yield must return None when nominated partition is Waiting"
        );
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "Bug 06: P0 must be restored to Running when both partitions are Waiting"
        );
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "P1 must remain Waiting"
        );
        assert_eq!(
            k.active_partition(),
            Some(0),
            "active_partition must remain P0"
        );
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
    }

    /// Bug 06 regression (tick path, 3 partitions): P0 active+Waiting,
    /// P1 Waiting, P2 Waiting. Tick boundary nominates P1 (Waiting),
    /// guard restores P0 to Running. P1 and P2 stay Waiting.
    #[test]
    fn bug06_tick_three_partitions_all_waiting_restores_active() {
        use crate::invariants::{
            assert_next_partition_not_waiting, assert_partition_state_consistency,
            assert_running_matches_active,
        };
        use crate::scheduler::ScheduleEvent;
        use crate::test_harness::KernelTestHarness;

        let mut h = KernelTestHarness::with_partitions(3).expect("harness setup");
        let k = h.kernel_mut();
        svc_scheduler::start_schedule(k);
        k.set_next_partition(0);

        // P0: Running → Waiting.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        // P1: Ready → Running → Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        // P2: Ready → Running → Waiting.
        let pcb2 = k.partitions_mut().get_mut(2).unwrap();
        pcb2.transition(PartitionState::Running).unwrap();
        pcb2.transition(PartitionState::Waiting).unwrap();

        // Advance 9 ticks (P0's slot = 10 ticks; boundary at tick 10).
        for _ in 0..9 {
            assert_eq!(svc_scheduler::advance_schedule_tick(k), ScheduleEvent::None);
        }
        // 10th tick: boundary nominates P1 (Waiting), guard restores P0.
        assert_eq!(svc_scheduler::advance_schedule_tick(k), ScheduleEvent::None);

        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(
            k.partitions().get(2).unwrap().state(),
            PartitionState::Waiting
        );
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
        assert_next_partition_not_waiting(k.partitions().as_slice(), k.next_partition());
    }

    /// Bug 06 regression (yield path, 3 partitions): P0 active+Waiting,
    /// P1 Waiting, P2 Ready. Yield from P0 → nominates P1 (Waiting),
    /// guard restores P0 to Running. P2 stays Ready (untouched).
    #[test]
    fn bug06_yield_three_partitions_nominee_waiting_restores_active() {
        use crate::invariants::{
            assert_next_partition_not_waiting, assert_partition_state_consistency,
            assert_running_matches_active,
        };
        use crate::test_harness::KernelTestHarness;

        let mut h = KernelTestHarness::with_partitions(3).expect("harness setup");
        let k = h.kernel_mut();
        svc_scheduler::start_schedule(k);
        k.set_next_partition(0);

        // P0: Running → Waiting.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        // P1: Ready → Running → Waiting.
        let pcb1 = k.partitions_mut().get_mut(1).unwrap();
        pcb1.transition(PartitionState::Running).unwrap();
        pcb1.transition(PartitionState::Waiting).unwrap();
        // P2: stays Ready (untouched).

        // Yield → force_advance nominates P1 (Waiting), guard restores P0.
        let result = k.yield_current_slot();
        assert_eq!(result.partition_id(), None, "must not switch to Waiting P1");

        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(k.active_partition(), Some(0));
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(
            k.partitions().get(2).unwrap().state(),
            PartitionState::Ready
        );
        assert_partition_state_consistency(k.partitions().as_slice());
        assert_running_matches_active(k.partitions().as_slice(), k.active_partition());
        assert_next_partition_not_waiting(k.partitions().as_slice(), k.next_partition());
    }

    /// Helper to create a Kernel with an UNSTARTED schedule for testing
    /// the start_schedule() method.
    fn kernel_unstarted_schedule() -> Kernel<'static, TestConfig> {
        // Create 2-slot schedule: P0 for 5 ticks, P1 for 3 ticks
        // NOTE: do NOT call schedule.start() here
        let mut schedule = ScheduleTable::<4>::new();
        schedule.add(ScheduleEntry::new(0, 5)).unwrap();
        schedule.add(ScheduleEntry::new(1, 3)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(1).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0x2000_0000, 4096, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2000_1000, 4096, 0),
                1,
            )
            .unwrap(),
        ];
        kernel_from_ext(schedule, &mems)
    }

    #[test]
    fn start_schedule_returns_initial_partition() {
        let mut k = kernel_unstarted_schedule();
        // Before start, active_partition is None (kernel hasn't started scheduling)
        assert_eq!(k.active_partition, None);

        // Start the schedule
        let initial = svc_scheduler::start_schedule(&mut k);
        assert_eq!(initial, Some(0)); // First entry is partition 0
        assert_eq!(k.active_partition, Some(0));
        assert_eq!(k.schedule().current_partition(), Some(0));
    }

    #[test]
    #[allow(deprecated)]
    fn start_schedule_empty_returns_none() {
        // Create empty kernel via new_empty
        #[cfg(not(feature = "dynamic-mpu"))]
        let mut k = Kernel::<TestConfig>::new_empty();
        #[cfg(feature = "dynamic-mpu")]
        let mut k = Kernel::<TestConfig>::new_empty(crate::virtual_device::DeviceRegistry::new());

        // Start empty schedule returns None
        let initial = svc_scheduler::start_schedule(&mut k);
        assert_eq!(initial, None);
        assert_eq!(k.active_partition, None);
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn start_schedule_allows_advance_tick() {
        use crate::scheduler::ScheduleEvent;
        let mut k = kernel_unstarted_schedule();
        // Start the schedule
        let initial = svc_scheduler::start_schedule(&mut k);
        assert_eq!(initial, Some(0));

        // Now advance ticks - schedule should work normally
        // P0 has 5 ticks, so 4 advances should return None
        for _ in 0..4 {
            assert_eq!(
                svc_scheduler::advance_schedule_tick(&mut k),
                ScheduleEvent::None
            );
        }
        // 5th tick triggers switch to P1
        assert_eq!(
            svc_scheduler::advance_schedule_tick(&mut k),
            ScheduleEvent::PartitionSwitch(1)
        );
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

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_notify_sets_debug_pending_flag() {
        use crate::syscall::SYS_DEBUG_NOTIFY;
        let mut k = kernel(0, 0, 0);
        // Verify debug_pending is initially false
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        // Dispatch SYS_DEBUG_NOTIFY syscall
        let mut ef = frame(SYS_DEBUG_NOTIFY, 0, 0);
        // SAFETY: See module-level SAFETY docs for test dispatch justification.
        unsafe { k.dispatch(&mut ef) };
        // Should return 0 on success
        assert_eq!(ef.r0, 0);
        // debug_pending flag should now be set on current partition (0)
        assert!(k.partitions().get(0).unwrap().debug_pending());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_pending_iterates_and_clears() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};
        use crate::partition_debug::DrainContext;

        static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
        static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

        let mut k = kernel(0, 0, 0);

        // Set up debug buffers on both partitions
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF0);
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .set_debug_buffer(&BUF1);

        // Write records to both buffers
        BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
        BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");

        // Signal debug pending on both partitions
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .signal_debug_pending();

        // Verify both flags are set
        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(k.partitions().get(1).unwrap().debug_pending());

        // Drain with sufficient budget
        let mut ctx = DrainContext::new();
        let drained = k.drain_debug_pending(&mut ctx, 256);

        // Should have drained records from both partitions (4 + 2 bytes each)
        assert_eq!(drained, 12);

        // Both debug_pending flags should be cleared
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(!k.partitions().get(1).unwrap().debug_pending());

        // Buffers should be empty
        assert!(BUF0.is_empty());
        assert!(BUF1.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_pending_skips_non_pending() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};
        use crate::partition_debug::DrainContext;

        static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
        static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

        let mut k = kernel(0, 0, 0);

        // Set up debug buffers on both partitions
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF0);
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .set_debug_buffer(&BUF1);

        // Write records to both buffers but only signal partition 1
        BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
        BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .signal_debug_pending();

        // Only partition 1 has debug_pending set
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(k.partitions().get(1).unwrap().debug_pending());

        let mut ctx = DrainContext::new();
        let drained = k.drain_debug_pending(&mut ctx, 256);

        // Should only drain partition 1's buffer (4 + 2 = 6 bytes)
        assert_eq!(drained, 6);

        // Partition 0's buffer should still have data
        assert!(!BUF0.is_empty());
        // Partition 1's buffer should be empty
        assert!(BUF1.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_auto_drains_pending_output() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut k = kernel(0, 0, 0);

        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"hello");
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();

        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(!BUF.is_empty());

        k.drain_debug_auto();

        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(BUF.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_auto_budget_zero_is_noop() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        struct NoDrainConfig;
        impl KernelConfig for NoDrainConfig {
            const N: usize = 4;
            const S: usize = 4;
            const SW: usize = 4;
            const MS: usize = 4;
            const MW: usize = 4;
            const QS: usize = 4;
            const QD: usize = 4;
            const QM: usize = 4;
            const QW: usize = 4;
            const SP: usize = 4;
            const BS: usize = 4;
            const BW: usize = 4;
            const DEBUG_AUTO_DRAIN_BUDGET: usize = 0;
            #[cfg(feature = "dynamic-mpu")]
            const BP: usize = 4;

            kernel_config_types!();
        }

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut k: Kernel<'static, NoDrainConfig> = Kernel::default();
        k.partitions_mut().add(pcb(0)).unwrap();

        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"stay");
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();

        assert!(k.partitions().get(0).unwrap().debug_pending());

        k.drain_debug_auto();

        // Budget is 0 so buffer must remain untouched.
        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(!BUF.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn systick_handler_drains_debug_output() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut k = kernel(0, 0, 0);

        // Set up debug buffer on partition 0
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"test");
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();
        assert!(k.partitions().get(0).unwrap().debug_pending());

        // Call systick_handler - should drain debug output
        crate::tick::systick_handler::<TestConfig>(&mut k);

        // debug_pending should be cleared and buffer empty
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(BUF.is_empty());
    }

    /// Mirrors the harness yield closure: yield_current_slot then
    /// drain_debug_auto clears pending debug output.
    #[cfg(feature = "partition-debug")]
    #[test]
    fn yield_then_drain_debug_auto_clears_pending() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut k = kernel_with_schedule();

        // Attach debug buffer to partition 0 and write a record.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"yield-drain");
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();

        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(!BUF.is_empty());

        // Execute the same sequence as the harness yield closure:
        // yield_current_slot followed by drain_debug_auto.
        let result = k.yield_current_slot();
        assert!(result.partition_id().is_some());
        k.drain_debug_auto();

        // Debug buffer must be fully drained.
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(BUF.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_auto_multi_partition_clears_all() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        static BUF0: DebugRingBuffer<64> = DebugRingBuffer::new();
        static BUF1: DebugRingBuffer<64> = DebugRingBuffer::new();

        let mut k = kernel(0, 0, 0);

        // Attach debug buffers to both partitions.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF0);
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .set_debug_buffer(&BUF1);

        // Write distinct records so we can verify per-partition drain.
        BUF0.write_record(LOG_INFO, KIND_TEXT, b"p0");
        BUF1.write_record(LOG_INFO, KIND_TEXT, b"p1");

        // Signal pending on both partitions.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();
        k.partitions_mut()
            .get_mut(1)
            .unwrap()
            .signal_debug_pending();

        // Pre-conditions: both pending, both non-empty.
        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(k.partitions().get(1).unwrap().debug_pending());
        assert!(!BUF0.is_empty());
        assert!(!BUF1.is_empty());

        // Exercise the auto-drain entry point (not drain_debug_pending directly).
        k.drain_debug_auto();

        // Both debug_pending flags must be cleared.
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(!k.partitions().get(1).unwrap().debug_pending());

        // Both buffers must be fully drained.
        assert!(BUF0.is_empty());
        assert!(BUF1.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn drain_debug_auto_idempotent_second_call_noop() {
        use crate::debug::{DebugRingBuffer, KIND_TEXT, LOG_INFO};

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();

        let mut k = kernel(0, 0, 0);

        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"once");
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .signal_debug_pending();

        assert!(k.partitions().get(0).unwrap().debug_pending());
        assert!(!BUF.is_empty());

        // First call drains the buffer and clears pending.
        k.drain_debug_auto();
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(BUF.is_empty());

        // Second call is a no-op: pending is already false so nothing runs.
        k.drain_debug_auto();
        assert!(!k.partitions().get(0).unwrap().debug_pending());
        assert!(BUF.is_empty());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_rejects_invalid_pointer() {
        use crate::syscall::SYS_DEBUG_WRITE;
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_DEBUG_WRITE, 0xDEAD_BEEF, 5);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    /// Allocate a page at a fixed low address for partition-debug tests.
    /// Page 0 corresponds to partition 0's MPU data region at 0x2000_0000.
    #[cfg(feature = "partition-debug")]
    fn debug_test_buf(page: usize) -> *mut u8 {
        extern "C" {
            fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
        }
        // Partition 0 has MPU data region at 0x2000_0000 with size 4096.
        // Use page 0 for all partition-debug tests (shared with dynamic-mpu tests).
        let _ = page; // All tests use partition 0's region.
        let addr = 0x2000_0000_usize;
        // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
        // low address. The mapping is intentionally leaked (test-only).
        let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
        assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
        ptr
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_success_writes_data_to_buffer() {
        use crate::debug::DebugRingBuffer;
        use crate::syscall::SYS_DEBUG_WRITE;

        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut k = kernel(0, 0, 0);

        // Configure debug buffer for partition 0.
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);

        // Write a known pattern into partition memory.
        let ptr = debug_test_buf(0);
        let pattern = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE];
        unsafe { core::ptr::copy_nonoverlapping(pattern.as_ptr(), ptr, pattern.len()) };

        // Invoke SYS_DEBUG_WRITE with ptr and len.
        let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, pattern.len() as u32);
        unsafe { k.dispatch(&mut ef) };

        // Verify success: r0 == bytes written.
        assert_eq!(ef.r0, pattern.len() as u32);

        // Verify data integrity: drain buffer and compare.
        let mut out = [0u8; 64];
        let n = BUF.drain(&mut out, 64);
        assert_eq!(n, pattern.len());
        assert_eq!(&out[..n], &pattern);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_returns_not_supported_when_no_buffer() {
        use crate::syscall::SYS_DEBUG_WRITE;

        let mut k = kernel(0, 0, 0);
        // Do NOT set a debug buffer.

        let ptr = debug_test_buf(1);
        let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, 4);
        unsafe { k.dispatch(&mut ef) };

        assert_eq!(ef.r0, SvcError::NotSupported.to_u32());
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn debug_write_returns_zero_on_overflow() {
        use crate::debug::DebugRingBuffer;
        use crate::syscall::SYS_DEBUG_WRITE;

        // Small buffer that will overflow.
        static BUF: DebugRingBuffer<16> = DebugRingBuffer::new();
        let mut k = kernel(0, 0, 0);

        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&BUF);

        // Try to write more than the buffer can hold — returns 0 (no bytes written).
        let ptr = debug_test_buf(2);
        let mut ef = frame(SYS_DEBUG_WRITE, ptr as u32, 32);
        unsafe { k.dispatch(&mut ef) };

        assert_eq!(ef.r0, 0);
    }

    /// Full state lifecycle round-trip: Running→Waiting→Ready→Running via
    /// blocking queuing recv and timed-wait expiry.
    #[test]
    fn partition_lifecycle_roundtrip_via_blocking_ipc() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::sampling::PortDirection;
        use crate::scheduler::ScheduleEvent;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;

        let mut k = kernel_with_schedule();

        // --- Step 1: P0 starts Running via set_next_partition ---
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 2: P0 dispatches blocking queuing recv (Running→Waiting) ---
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let r2 = (50u32 << 16) | 4;
        // Use P0's actual MPU region base as the buffer pointer.
        // Kernel::new replaces the config base with the internal stack
        // address, so low32_buf won't pass pointer validation.
        let mpu_base = k.partitions().get(0).unwrap().mpu_region().base();
        let ptr = mpu_base as *mut u8;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
        // SAFETY: ef is a valid ExceptionFrame and k is a valid Kernel.
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "blocking recv should return 0");

        // --- Step 3: Verify P0 is Waiting and yield_requested is true ---
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert!(
            k.yield_requested(),
            "yield_requested must be true after blocking recv"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 4: expire_timed_waits at timeout tick (Waiting→Ready) ---
        // The recv was dispatched at tick=1 (set_next_partition didn't advance;
        // the kernel tick is still 0 when dispatch runs, so expiry = 0 + 50 = 50).
        let expiry_tick = k.tick().get() + 50;
        k.expire_timed_waits::<8>(expiry_tick);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 should be Ready after timeout expiry"
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 5: Advance schedule until P0's next slot (Ready→Running) ---
        // Schedule layout: P0(5 ticks) → P1(3 ticks) [→ SystemWindow on dynamic-mpu]
        // P0 is currently in slot 0.  Advance through the remaining P0 ticks
        // and all of P1's slot so the schedule wraps back to P0.
        let mut switched_to_p0 = false;
        // Upper bound: one full cycle is 8 ticks (+ 1 system window tick with
        // dynamic-mpu). Use 12 as a safe ceiling.
        for _ in 0..12 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            if ev == ScheduleEvent::PartitionSwitch(0) {
                switched_to_p0 = true;
                break;
            }
        }
        assert!(switched_to_p0, "schedule must eventually switch back to P0");

        // --- Step 6: Verify P0 is Running again ---
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "P0 should be Running after schedule switches to it"
        );
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Regression test for bug03: a blocking IPC syscall sets Waiting, and
    /// the partition must remain Waiting through a full schedule cycle wrap.
    /// Without the fix in `transition_outgoing_ready` (which skips the
    /// Running→Ready demotion for non-Running partitions), the Waiting state
    /// would be overwritten to Ready when the schedule switched away from P0.
    #[test]
    fn bug03_blocking_ipc_preserves_waiting_across_schedule_wrap() {
        use crate::invariants::assert_partition_state_consistency;
        use crate::sampling::PortDirection;
        use crate::scheduler::ScheduleEvent;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;

        let mut k = kernel_with_schedule();

        // --- Step 1: P0 starts Running ---
        k.set_next_partition(0);
        k.active_partition = Some(0);
        k.set_current_partition(0);
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Running,
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 2: P0 dispatches blocking QueuingRecvTimed → Waiting ---
        let dst = k
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .unwrap();
        let r2 = (500u32 << 16) | 4; // large timeout; must not expire during test
        let mpu_base = k.partitions().get(0).unwrap().mpu_region().base();
        let ptr = mpu_base as *mut u8;
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
        // SAFETY: `ef` is a valid stack-local ExceptionFrame with initialized
        // r0-r3 fields. `k` is a properly constructed test Kernel with a valid
        // partition in Running state. Single-threaded test; no data races.
        unsafe { k.dispatch(&mut ef) };
        // TODO: dispatch() overwrites ef.r0 with a kernel-internal
        // "descheduled" indicator (0) even though P0 is now Waiting and
        // the user-visible r0 should hold the eventual syscall return
        // value.  This is an architectural state leak in the dispatch
        // path that should be fixed separately.
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking recv",
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 3: Advance through P0's remaining slot ticks → P1 ---
        // Schedule: P0(5 ticks) | P1(3 ticks). set_next_partition consumed
        // tick 0 implicitly; the schedule cursor is at the start of P0's
        // slot. Advance until we see PartitionSwitch(1).
        let mut switched_to_p1 = false;
        for _ in 0..5 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_partition_state_consistency(k.partitions().as_slice());
            // P0 must stay Waiting throughout — never demoted to Ready.
            assert_eq!(
                k.partitions().get(0).unwrap().state(),
                PartitionState::Waiting,
                "P0 must remain Waiting during P0's slot",
            );
            if ev == ScheduleEvent::PartitionSwitch(1) {
                switched_to_p1 = true;
                break;
            }
        }
        assert!(switched_to_p1, "schedule must switch to P1 after P0's slot");
        assert_eq!(
            k.partitions().get(1).unwrap().state(),
            PartitionState::Running,
            "P1 must be Running after switch",
        );
        assert_partition_state_consistency(k.partitions().as_slice());

        // --- Step 4: Advance through P1's slot until wrap to P0 boundary ---
        // P1 has 3 ticks. After those ticks (plus an optional system window
        // tick on dynamic-mpu), the schedule wraps back to P0's slot.
        let mut p0_boundary_event = Option::<ScheduleEvent>::None;
        for _ in 0..6 {
            let ev = svc_scheduler::advance_schedule_tick(&mut k);
            assert_partition_state_consistency(k.partitions().as_slice());
            // P0 must stay Waiting the entire time.
            assert_eq!(
                k.partitions().get(0).unwrap().state(),
                PartitionState::Waiting,
                "P0 must remain Waiting during P1's slot / wrap",
            );
            #[cfg(feature = "dynamic-mpu")]
            if ev == ScheduleEvent::SystemWindow {
                continue;
            }
            if matches!(ev, ScheduleEvent::PartitionSwitch(0) | ScheduleEvent::None) {
                // We reached the boundary where the schedule *would* switch
                // to P0. Because P0 is Waiting, advance_schedule_tick must
                // return None (skip).
                p0_boundary_event = Some(ev);
                break;
            }
        }

        // --- Step 5: Verify the skip ---
        let ev = p0_boundary_event.expect("schedule must reach P0's slot boundary");
        assert_eq!(
            ev,
            ScheduleEvent::None,
            "advance_schedule_tick must return None when P0 is Waiting (skip)",
        );

        // --- Step 6: Final state assertions ---
        assert_eq!(
            k.partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must still be Waiting after full schedule wrap",
        );
        assert_partition_state_consistency(k.partitions().as_slice());
    }

    /// Boot guard: fix_mpu_data_region only for sentinel (size==0).
    #[test]
    fn fix_mpu_data_region_skipped_for_user_configured_partition() {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 50)).unwrap();
        s.add(ScheduleEntry::new(1, 50)).unwrap();
        let mut stk0 = AlignedStack1K::default();
        let mut stk1 = AlignedStack1K::default();
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk0,
                0x0800_0001,
                MpuRegion::new(0, 0, 0),
                0,
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut stk1,
                0x0800_1001,
                MpuRegion::new(0x2004_0000, 2048, 0x0306_0000),
                1,
            )
            .unwrap(),
        ];
        let mut k = try_kernel_new_mem(s, &mems).unwrap();
        for i in 0..2 {
            let sz = k.partitions().get(i).unwrap().mpu_region().size();
            if sz == 0 {
                let base = k.partitions().get(i).unwrap().stack_base();
                assert!(k.fix_mpu_data_region(i, base));
            }
        }
        let p0 = k.partitions().get(0).unwrap();
        assert_eq!(p0.mpu_region().base(), p0.stack_base());
        let p1 = k.partitions().get(1).unwrap();
        assert_eq!(p1.mpu_region().base(), 0x2004_0000);
        assert_eq!(p1.mpu_region().size(), 2048);
    }

    // ---- PendSV ABI constant verification (compile-time) ----
    //
    // Replaces the former pendsv_abi_offsets_match_expected_layout runtime test
    // with const assertions that fire during compilation.  Verifies offset
    // ranges, field ordering, alignment, and element stride for TestConfig.

    // Compile-time version of the former pendsv_abi_offsets_match_expected_layout
    // runtime test.  These assertions fire during `cargo test` compilation,
    // catching layout regressions without needing to run the binary.
    const _: () = {
        type K = Kernel<'static, TestConfig>;
        type C = <TestConfig as KernelConfig>::Core;

        // Field ordering: current_partition before core.
        assert!(core::mem::offset_of!(K, current_partition) < core::mem::offset_of!(K, core));
        // Field ordering: next_partition before partition_sp.
        assert!(core::mem::offset_of!(C, next_partition) < core::mem::offset_of!(C, partition_sp));

        // partition_sp combined offset must be 4-byte aligned for word-sized ldr/str.
        assert!((core::mem::offset_of!(K, core) + core::mem::offset_of!(C, partition_sp)) % 4 == 0);

        // partition_sp element stride must be 4 (u32).
        #[allow(unused)]
        fn _assert_sp_elem_is_u32_stride_4(c: &C) {
            let _: u32 = c.partition_sp[0];
            let _: [u8; 4] = c.partition_sp[0].to_ne_bytes();
        }
    };

    #[test]
    fn combined_pendsv_offsets_within_literal_pool_limit() {
        use crate::pendsv::LITERAL_POOL_OFFSET_LIMIT;

        // Small config with AlignedStack1K (align 1024) so core fits within limit.
        // TestConfig uses AlignedStack4K which pushes core to offset 4096.
        struct SmallConfig;
        impl KernelConfig for SmallConfig {
            const N: usize = 2;
            const S: usize = 2;
            const SW: usize = 2;
            const MS: usize = 2;
            const MW: usize = 2;
            const QS: usize = 2;
            const QD: usize = 2;
            const QM: usize = 2;
            const QW: usize = 2;
            const SP: usize = 2;
            const BS: usize = 2;
            const BW: usize = 2;
            #[cfg(feature = "dynamic-mpu")]
            const BP: usize = 2;

            kernel_config_types!();
        }

        type K = Kernel<'static, SmallConfig>;
        type C = <SmallConfig as KernelConfig>::Core;

        let co = core::mem::offset_of!(K, core);
        let cp = core::mem::offset_of!(K, current_partition);
        let td = core::mem::offset_of!(K, ticks_dropped);
        let np = co + core::mem::offset_of!(C, next_partition);
        let sp = co + core::mem::offset_of!(C, partition_sp);
        let sl = co + core::mem::offset_of!(C, partition_stack_limits);

        // Every offset accessed by PendSV assembly must fit in literal-pool offset limit.
        for (off, name) in [
            (cp, "current_partition"),
            (td, "ticks_dropped"),
            (co, "core"),
            (np, "core+next_partition"),
            (sp, "core+partition_sp"),
            (sl, "core+partition_stack_limits"),
        ] {
            assert!(off < LITERAL_POOL_OFFSET_LIMIT, "{name} offset {off}");
        }
        assert_eq!(sp % 4, 0, "partition_sp not 4-byte aligned");
        assert!(cp < co && np < sp);
    }
}
