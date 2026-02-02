use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use crate::blackboard::BlackboardPool;
use crate::config::KernelConfig;
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
/// MPU data region of partition `pid`.
///
/// Returns `true` when **all** of the following hold:
/// 1. `pid` refers to an existing partition in `partitions`.
/// 2. `ptr >= region_base`.
/// 3. `ptr + len <= region_base + region_size` (the end is inclusive of the
///    last valid byte, so `ptr + len` may equal `base + size`).
/// 4. `ptr + len` does not overflow `u32`.
///
/// A zero-length range (`len == 0`) is considered valid as long as `ptr`
/// itself falls within `[base, base + size]`, but the caller should avoid
/// dereferencing such a pointer.
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
    let region = pcb.mpu_region();
    let base = region.base();
    let size = region.size();

    // Compute end of the user range with overflow check.
    let end = match ptr.checked_add(len as u32) {
        Some(e) => e,
        None => return false,
    };

    // MPU region validity (base + size not overflowing) is enforced at
    // region creation time, so a simple add is correct here.
    let region_end = base + size;

    ptr >= base && end <= region_end
}

/// Validate a user pointer and, on success, execute the body expression.
///
/// Returns `SvcError::InvalidPointer` when `validate_user_ptr` fails;
/// otherwise evaluates `$body` (which must produce `u32`).
macro_rules! validated_ptr {
    ($self:ident, $ptr:expr, $len:expr, $body:expr) => {
        if !validate_user_ptr(&$self.partitions, $self.current_partition, $ptr, $len) {
            SvcError::InvalidPointer.to_u32()
        } else {
            $body
        }
    };
}

use crate::events;
use crate::message::{MessagePool, RecvOutcome, SendOutcome};
use crate::mutex::MutexPool;
use crate::partition::{PartitionState, PartitionTable};
use crate::queuing::{QueuingPortPool, QueuingPortStatus, SendQueuingOutcome};
use crate::sampling::SamplingPortPool;
use crate::semaphore::SemaphorePool;
use crate::syscall::SyscallId;
use crate::tick::TickCounter;
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

/// Declare a `Mutex<RefCell<Option<Kernel<$Config>>>>` static, an SVC
/// dispatch hook that forwards to [`Kernel::dispatch`], and a helper to
/// store the kernel instance.
///
/// This is the companion to [`define_pendsv!`](crate::define_pendsv): it
/// eliminates the boilerplate that every QEMU example otherwise duplicates
/// for the safe `Mutex`-based kernel storage and dispatch pattern.
///
/// # Generated items
///
/// | Item | Description |
/// |------|-------------|
/// | `static KERN: Mutex<RefCell<Option<Kernel<$Config>>>>` | Thread-safe kernel storage |
/// | `unsafe extern "C" fn dispatch_hook(f: &mut ExceptionFrame)` | SVC hook that dispatches via `KERN` |
/// | `fn store_kernel(k: Kernel<$Config>)` | Store the kernel and install the hook |
///
/// # Yield handler
///
/// An optional second argument is a block `{ |k| … }` that is called
/// after `k.dispatch(f)` when `k.yield_requested` is set.  The harness
/// uses this to force-advance the schedule without duplicating the
/// entire macro body.
///
/// # Usage
///
/// ```ignore
/// // Standalone (no yield handling):
/// kernel::define_dispatch_hook!(DemoConfig);
///
/// // With yield handler (used by define_harness!):
/// kernel::define_dispatch_hook!(DemoConfig, |k| { … });
/// ```
#[macro_export]
macro_rules! define_dispatch_hook {
    ($Config:ty $(, |$k:ident| $yield_body:block )? ) => {
        static KERN: ::cortex_m::interrupt::Mutex<
            ::core::cell::RefCell<Option<$crate::svc::Kernel<$Config>>>,
        > = ::cortex_m::interrupt::Mutex::new(::core::cell::RefCell::new(None));

        unsafe extern "C" fn dispatch_hook(f: &mut $crate::context::ExceptionFrame) {
            // SAFETY: called from SVC exception context on single-core Cortex-M;
            // `cortex_m::interrupt::free` masks interrupts, ensuring exclusive access.
            ::cortex_m::interrupt::free(|cs| {
                if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                    // Synchronise the kernel's caller identity from the
                    // assembly-level CURRENT_PARTITION written by PendSV.
                    // SAFETY: CURRENT_PARTITION is a #[no_mangle] static mut u32
                    // written by the PendSV handler; reading it here is safe
                    // because interrupts are masked by `interrupt::free` on
                    // single-core Cortex-M, so no concurrent write can occur.
                    extern "C" { static CURRENT_PARTITION: u32; }
                    // SAFETY: read-only access; value is always a valid u8
                    // partition index (set by PendSV from NEXT_PARTITION).
                    k.current_partition =
                        unsafe { core::ptr::read_volatile(&raw const CURRENT_PARTITION) } as u8;

                    // SAFETY: `dispatch_hook` is only installed via `set_dispatch_hook`
                    // which stores it in SVC_DISPATCH_HOOK; it is only ever called from
                    // `SVC_HANDLER` with a valid `ExceptionFrame` pointer derived from
                    // the hardware-stacked exception frame. Interrupts are masked by
                    // `interrupt::free`, so `k` has exclusive access and no data races
                    // are possible on single-core Cortex-M.
                    unsafe { k.dispatch(f) }

                    $(
                        if k.yield_requested {
                            k.yield_requested = false;
                            let $k = k;
                            $yield_body
                        }
                    )?
                }
            });
        }

        /// Store the kernel instance and install the SVC dispatch hook.
        fn store_kernel(k: $crate::svc::Kernel<$Config>) {
            ::cortex_m::interrupt::free(|cs| {
                KERN.borrow(cs).replace(Some(k));
            });
            $crate::svc::set_dispatch_hook(dispatch_hook);
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
    let hook = with_cs(|cs| *SVC_DISPATCH_HOOK.borrow(cs).borrow());
    if let Some(hook) = hook {
        return hook(frame);
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
    [(); C::S]:,
    [(); C::SW]:,
    [(); C::MS]:,
    [(); C::MW]:,
    [(); C::QS]:,
    [(); C::QD]:,
    [(); C::QM]:,
    [(); C::QW]:,
    [(); C::SP]:,
    [(); C::SM]:,
    [(); C::BS]:,
    [(); C::BM]:,
    [(); C::BW]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
{
    pub partitions: PartitionTable<{ C::N }>,
    pub semaphores: SemaphorePool<{ C::S }, { C::SW }>,
    pub mutexes: MutexPool<{ C::MS }, { C::MW }>,
    pub messages: MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    pub tick: TickCounter,
    pub sampling: SamplingPortPool<{ C::SP }, { C::SM }>,
    pub queuing: QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    pub blackboards: BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    /// The partition index of the currently executing partition, set by the
    /// scheduler before entering user code. Used as the trusted caller identity
    /// for syscalls, rather than reading from a user-controlled register.
    pub current_partition: u8,
    /// Set to `true` by `SYS_YIELD` dispatch; checked and cleared by the
    /// harness so it can force-advance the schedule and update
    /// `NEXT_PARTITION` before PendSV fires.
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
}

impl<C: KernelConfig> Default for Kernel<C>
where
    [(); C::N]:,
    [(); C::S]:,
    [(); C::SW]:,
    [(); C::MS]:,
    [(); C::MW]:,
    [(); C::QS]:,
    [(); C::QD]:,
    [(); C::QM]:,
    [(); C::QW]:,
    [(); C::SP]:,
    [(); C::SM]:,
    [(); C::BS]:,
    [(); C::BM]:,
    [(); C::BW]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<C: KernelConfig> Kernel<C>
where
    [(); C::N]:,
    [(); C::S]:,
    [(); C::SW]:,
    [(); C::MS]:,
    [(); C::MW]:,
    [(); C::QS]:,
    [(); C::QD]:,
    [(); C::QM]:,
    [(); C::QW]:,
    [(); C::SP]:,
    [(); C::SM]:,
    [(); C::BS]:,
    [(); C::BM]:,
    [(); C::BW]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
{
    /// Create a new `Kernel` with empty resource pools and partition table.
    pub fn new() -> Self {
        Self {
            partitions: PartitionTable::new(),
            semaphores: SemaphorePool::new(),
            mutexes: MutexPool::new(0),
            messages: MessagePool::new(),
            tick: TickCounter::new(),
            sampling: SamplingPortPool::new(),
            queuing: QueuingPortPool::new(),
            blackboards: BlackboardPool::new(),
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
        }
    }

    /// Install an optional hardware UART backend.
    ///
    /// Once set, `dev_dispatch` will check this backend when the requested
    /// `device_id` does not match either virtual UART in `uart_pair`.
    #[cfg(feature = "dynamic-mpu")]
    pub fn set_hw_uart(&mut self, backend: crate::hw_uart::HwUartBackend) {
        self.hw_uart = Some(backend);
    }

    /// Look up a virtual device by `device_id` and invoke `f` with a mutable
    /// reference to the device (as a trait object) and the current partition.
    ///
    /// Checks `uart_pair` first, then falls through to `hw_uart`.
    /// Returns `InvalidResource` when the ID is unknown or `OperationFailed`
    /// when the closure returns a `DeviceError`.
    #[cfg(feature = "dynamic-mpu")]
    fn dev_dispatch<F>(&mut self, device_id: u8, f: F) -> u32
    where
        F: FnOnce(&mut dyn VirtualDevice, u8) -> Result<u32, crate::virtual_device::DeviceError>,
    {
        let pid = self.current_partition;
        // Resolve the target device: virtual UART pair first, then hw_uart.
        let dev: Option<&mut dyn VirtualDevice> = self
            .uart_pair
            .get_mut(device_id)
            .map(|d| d as _)
            .or_else(|| {
                self.hw_uart
                    .as_mut()
                    .filter(|hw| hw.device_id() == device_id)
                    .map(|hw| hw as _)
            });
        match dev {
            Some(d) => match f(d, pid) {
                Ok(val) => val,
                Err(_) => SvcError::OperationFailed.to_u32(),
            },
            None => SvcError::InvalidResource.to_u32(),
        }
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
        frame.r0 = match SyscallId::from_u32(frame.r0) {
            Some(SyscallId::Yield) => {
                self.yield_requested = true;
                handle_yield()
            }
            Some(SyscallId::EventWait) => {
                events::event_wait(&mut self.partitions, frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::EventSet) => {
                events::event_set(&mut self.partitions, frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::EventClear) => {
                events::event_clear(&mut self.partitions, frame.r1 as usize, frame.r2)
            }
            Some(SyscallId::SemWait) => {
                match self.semaphores.wait(
                    &mut self.partitions,
                    frame.r1 as usize,
                    frame.r2 as usize,
                ) {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::SemSignal) => {
                match self
                    .semaphores
                    .signal(&mut self.partitions, frame.r1 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MutexLock) => {
                match self
                    .mutexes
                    .lock(&mut self.partitions, frame.r1 as usize, frame.r2 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MutexUnlock) => {
                match self.mutexes.unlock(
                    &mut self.partitions,
                    frame.r1 as usize,
                    frame.r2 as usize,
                ) {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MsgSend) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let data = unsafe { core::slice::from_raw_parts(frame.r3 as *const u8, C::QM) };
                match self
                    .messages
                    .send(frame.r1 as usize, frame.r2 as usize, data)
                {
                    Ok(outcome) => apply_send_outcome(&mut self.partitions, outcome),
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::MsgRecv) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let buf = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                match self
                    .messages
                    .recv(frame.r1 as usize, frame.r2 as usize, buf)
                {
                    Ok(outcome) => apply_recv_outcome(&mut self.partitions, outcome),
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::SamplingWrite) => validated_ptr!(self, frame.r3, frame.r2 as usize, {
                // SAFETY: validated_ptr confirmed [r3, r3+r2) lies within
                // the calling partition's MPU data region.
                let d = unsafe {
                    core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                };
                match self
                    .sampling
                    .write_sampling_message(frame.r1 as usize, d, self.tick.get())
                {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::SamplingRead) => validated_ptr!(self, frame.r3, C::SM, {
                // SAFETY: validated_ptr confirmed [r3, r3+SM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::SM) };
                match self
                    .sampling
                    .read_sampling_message(frame.r1 as usize, b, self.tick.get())
                {
                    Ok((sz, _)) => sz as u32,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::QueuingSend) => validated_ptr!(self, frame.r3, frame.r2 as usize, {
                // SAFETY: validated_ptr confirmed [r3, r3+r2) lies within
                // the calling partition's MPU data region.
                let d = unsafe {
                    core::slice::from_raw_parts(frame.r3 as *const u8, frame.r2 as usize)
                };
                match self.queuing.send_routed(
                    frame.r1 as usize,
                    self.current_partition,
                    d,
                    0,
                    self.tick.get(),
                ) {
                    Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
                        if let Some(pid) = w {
                            try_transition(&mut self.partitions, pid, PartitionState::Ready);
                        }
                        0
                    }
                    Ok(SendQueuingOutcome::SenderBlocked { .. }) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }),
            Some(SyscallId::QueuingRecv) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                match self.queuing.receive_queuing_message(
                    frame.r1 as usize,
                    self.current_partition,
                    b,
                    0,
                    self.tick.get(),
                ) {
                    Ok(crate::queuing::RecvQueuingOutcome::Received {
                        msg_len,
                        wake_sender: w,
                    }) => {
                        if let Some(pid) = w {
                            try_transition(&mut self.partitions, pid, PartitionState::Ready);
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
                    match self.queuing.get_queuing_port_status(frame.r1 as usize) {
                        Ok(status) => {
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
                    match self.blackboards.display_blackboard(frame.r1 as usize, d) {
                        Ok(woken) => {
                            for &pid in woken.iter() {
                                try_transition(&mut self.partitions, pid, PartitionState::Ready);
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
                    match self.blackboards.read_blackboard_timed(
                        frame.r1 as usize,
                        self.current_partition,
                        b,
                        frame.r2,
                        self.tick.get(),
                    ) {
                        Ok(crate::blackboard::ReadBlackboardOutcome::Read { msg_len }) => {
                            msg_len as u32
                        }
                        Ok(crate::blackboard::ReadBlackboardOutcome::ReaderBlocked) => {
                            try_transition(
                                &mut self.partitions,
                                self.current_partition,
                                PartitionState::Waiting,
                            );
                            0
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            Some(SyscallId::BbClear) => {
                match self.blackboards.clear_blackboard(frame.r1 as usize) {
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
                match self.buffers.alloc(self.current_partition, m) {
                    Some(slot) => slot as u32,
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
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
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
                            if len > buf.len() {
                                SvcError::OperationFailed.to_u32()
                            } else {
                                // SAFETY: validated_ptr confirmed [r3, r3+r2)
                                // lies within the calling partition's MPU data
                                // region.
                                let src = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                                buf[..len].copy_from_slice(src);
                                len as u32
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
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
                    let len = frame.r2 as usize;
                    let buf_ptr = frame.r3 as *mut u8;
                    self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                        // SAFETY: validated_ptr confirmed [r3, r3+r2) lies
                        // within the calling partition's MPU data region.
                        let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, len) };
                        Ok(dev.read(pid, buf)? as u32)
                    })
                })
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevWrite) => {
                validated_ptr!(self, frame.r3, frame.r2 as usize, {
                    let len = frame.r2 as usize;
                    let data_ptr = frame.r3 as *const u8;
                    self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                        // SAFETY: validated_ptr confirmed [r3, r3+r2) lies
                        // within the calling partition's MPU data region.
                        let data = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                        Ok(dev.write(pid, data)? as u32)
                    })
                })
            }
            #[cfg(feature = "dynamic-mpu")]
            Some(SyscallId::DevIoctl) => self.dev_dispatch(frame.r1 as u8, |dev, pid| {
                dev.ioctl(pid, frame.r2, frame.r3)
            }),
            Some(SyscallId::QueuingRecvTimed) => validated_ptr!(self, frame.r3, C::QM, {
                // SAFETY: validated_ptr confirmed [r3, r3+QM) lies within
                // the calling partition's MPU data region.
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::QM) };
                match self.queuing.receive_queuing_message(
                    frame.r1 as usize,
                    self.current_partition,
                    b,
                    frame.r2 as u64,
                    self.tick.get(),
                ) {
                    Ok(crate::queuing::RecvQueuingOutcome::Received {
                        msg_len,
                        wake_sender: w,
                    }) => {
                        if let Some(pid) = w {
                            try_transition(&mut self.partitions, pid, PartitionState::Ready);
                        }
                        msg_len as u32
                    }
                    Ok(crate::queuing::RecvQueuingOutcome::ReceiverBlocked { .. }) => {
                        try_transition(
                            &mut self.partitions,
                            self.current_partition,
                            PartitionState::Waiting,
                        );
                        0
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
                    match self.queuing.send_routed(
                        frame.r1 as usize,
                        self.current_partition,
                        d,
                        timeout,
                        self.tick.get(),
                    ) {
                        Ok(SendQueuingOutcome::Delivered { wake_receiver: w }) => {
                            if let Some(pid) = w {
                                try_transition(&mut self.partitions, pid, PartitionState::Ready);
                            }
                            0
                        }
                        Ok(SendQueuingOutcome::SenderBlocked { .. }) => {
                            try_transition(
                                &mut self.partitions,
                                self.current_partition,
                                PartitionState::Waiting,
                            );
                            0
                        }
                        Err(_) => SvcError::InvalidResource.to_u32(),
                    }
                })
            }
            None => SvcError::InvalidSyscall.to_u32(),
        };
    }
}

/// Try to transition partition `pid` to `state`. Returns `true` on success.
fn try_transition<const N: usize>(
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
/// Returns 0 on success or `u32::MAX` if a partition transition fails.
fn apply_send_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: SendOutcome,
) -> u32 {
    match outcome {
        SendOutcome::Delivered {
            wake_receiver: Some(pid),
        } => {
            if !try_transition(partitions, pid, PartitionState::Ready) {
                return SvcError::TransitionFailed.to_u32();
            }
            0
        }
        SendOutcome::Delivered {
            wake_receiver: None,
        } => 0,
        SendOutcome::SenderBlocked { blocked } => {
            if !try_transition(partitions, blocked, PartitionState::Waiting) {
                return SvcError::TransitionFailed.to_u32();
            }
            0
        }
    }
}

/// Apply a `RecvOutcome` by transitioning partition states as needed.
/// Returns 0 on success, or `u32::MAX` if a partition transition fails.
fn apply_recv_outcome<const N: usize>(
    partitions: &mut PartitionTable<N>,
    outcome: RecvOutcome,
) -> u32 {
    match outcome {
        RecvOutcome::Received {
            wake_sender: Some(pid),
        } => {
            if !try_transition(partitions, pid, PartitionState::Ready) {
                return SvcError::TransitionFailed.to_u32();
            }
            0
        }
        RecvOutcome::Received { wake_sender: None } => 0,
        RecvOutcome::ReceiverBlocked { blocked } => {
            if !try_transition(partitions, blocked, PartitionState::Waiting) {
                return SvcError::TransitionFailed.to_u32();
            }
            0
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
    use super::*;
    use crate::config::KernelConfig;
    use crate::message::MessageQueue;
    use crate::partition::{MpuRegion, PartitionControlBlock};
    use crate::semaphore::Semaphore;
    use crate::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_YIELD};

    /// Test kernel configuration with small, fixed pool sizes.
    struct TestConfig;
    impl KernelConfig for TestConfig {
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
        const SM: usize = 64;
        const BS: usize = 4;
        const BM: usize = 64;
        const BW: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 32;
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

    /// Build a Kernel with 2 running partitions, the given semaphore count,
    /// mutex count, and message queue count.
    fn kernel(sem_count: usize, mtx_count: usize, msg_queue_count: usize) -> Kernel<TestConfig> {
        let t = tbl();
        let s = SemaphorePool::<4, 4>::new();
        let m = MutexPool::<4, 4>::new(mtx_count);
        let mut msgs = MessagePool::<4, 4, 4, 4>::new();
        for _ in 0..msg_queue_count {
            msgs.add(MessageQueue::new()).unwrap();
        }
        let mut k = Kernel {
            partitions: t,
            semaphores: s,
            mutexes: m,
            messages: msgs,
            tick: TickCounter::new(),
            sampling: SamplingPortPool::new(),
            queuing: QueuingPortPool::new(),
            blackboards: BlackboardPool::new(),
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
        };
        // Add semaphores
        for _ in 0..sem_count {
            k.semaphores.add(Semaphore::new(1, 2)).unwrap();
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
        assert!(!k.yield_requested);
        let mut ef = frame(SYS_YIELD, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert!(k.yield_requested);
    }

    #[test]
    fn yield_requested_cleared_after_manual_reset() {
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_YIELD, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert!(k.yield_requested);
        k.yield_requested = false;
        assert!(!k.yield_requested);
        // Non-yield syscall does not set the flag
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert!(!k.yield_requested);
    }

    #[test]
    fn invalid_syscall_returns_error_code() {
        let mut ef = frame(0xFFFF, 0, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, SvcError::InvalidSyscall.to_u32());
    }

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
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let mut ef = frame(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[test]
    fn mutex_lock_unlock_dispatch() {
        let mut k = kernel(0, 1, 0);
        let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(k.mutexes.owner(0), Ok(Some(0)));
        let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(k.mutexes.owner(0), Ok(None));
    }

    #[test]
    fn msg_send_recv_pointer_based() {
        let mut k = kernel(0, 0, 2);
        // Send to queue 0 from partition 0
        let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        let outcome = k.messages.send(0, 0, &data).unwrap();
        let r0 = apply_send_outcome(&mut k.partitions, outcome);
        assert_eq!(r0, 0);

        // Receive from queue 0 into partition 1's buffer
        let mut recv_buf = [0u8; 4];
        let outcome = k.messages.recv(0, 1, &mut recv_buf).unwrap();
        let r0 = apply_recv_outcome(&mut k.partitions, outcome);
        assert_eq!(r0, 0);
        assert_eq!(recv_buf, [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn msg_send_recv_multiple_queues() {
        let mut k = kernel(0, 0, 2);
        // Send different data to queue 0 and queue 1
        let data_q0: [u8; 4] = [1, 2, 3, 4];
        let data_q1: [u8; 4] = [5, 6, 7, 8];

        let outcome = k.messages.send(0, 0, &data_q0).unwrap();
        assert_eq!(apply_send_outcome(&mut k.partitions, outcome), 0);

        let outcome = k.messages.send(1, 0, &data_q1).unwrap();
        assert_eq!(apply_send_outcome(&mut k.partitions, outcome), 0);

        // Recv from queue 1 first
        let mut buf = [0u8; 4];
        let outcome = k.messages.recv(1, 1, &mut buf).unwrap();
        assert_eq!(apply_recv_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(buf, [5, 6, 7, 8]);

        // Recv from queue 0
        let mut buf = [0u8; 4];
        let outcome = k.messages.recv(0, 1, &mut buf).unwrap();
        assert_eq!(apply_recv_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn msg_invalid_queue_id_returns_max() {
        let mut k = kernel(0, 0, 1);
        assert!(k.messages.send(99, 0, &[1; 4]).is_err());
        assert!(k.messages.recv(99, 0, &mut [0; 4]).is_err());
    }

    #[test]
    fn msg_send_blocks_and_wakes() {
        let mut k = kernel(0, 0, 1);
        // Fill the depth-4 queue to capacity
        for i in 0..4u8 {
            let outcome = k.messages.send(0, 0, &[i; 4]).unwrap();
            assert_eq!(apply_send_outcome(&mut k.partitions, outcome), 0);
        }

        // Next send blocks partition 1
        let outcome = k.messages.send(0, 1, &[99; 4]).unwrap();
        assert_eq!(outcome, SendOutcome::SenderBlocked { blocked: 1 });
        assert_eq!(apply_send_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(
            k.partitions.get(1).unwrap().state(),
            PartitionState::Waiting
        );

        // Recv should wake partition 1
        let mut buf = [0u8; 4];
        let outcome = k.messages.recv(0, 0, &mut buf).unwrap();
        assert_eq!(
            outcome,
            RecvOutcome::Received {
                wake_sender: Some(1)
            }
        );
        assert_eq!(apply_recv_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(k.partitions.get(1).unwrap().state(), PartitionState::Ready);
        assert_eq!(buf, [0; 4]); // first message enqueued
    }

    #[test]
    fn msg_recv_blocks_and_wakes() {
        let mut k = kernel(0, 0, 1);
        // Recv on empty queue blocks partition 0
        let mut buf = [0u8; 4];
        let outcome = k.messages.recv(0, 0, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::ReceiverBlocked { blocked: 0 });
        assert_eq!(apply_recv_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(
            k.partitions.get(0).unwrap().state(),
            PartitionState::Waiting
        );

        // Send should wake partition 0
        let outcome = k.messages.send(0, 1, &[9; 4]).unwrap();
        assert_eq!(
            outcome,
            SendOutcome::Delivered {
                wake_receiver: Some(0)
            }
        );
        assert_eq!(apply_send_outcome(&mut k.partitions, outcome), 0);
        assert_eq!(k.partitions.get(0).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn get_time_returns_zero_initially() {
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
    }

    #[test]
    fn get_time_after_increments() {
        let mut k = kernel(0, 0, 0);
        for _ in 0..5 {
            k.tick.increment();
        }
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 5);
    }

    #[test]
    fn get_time_preserves_other_registers() {
        let mut k = kernel(0, 0, 0);
        k.tick.increment();
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0xAA, 0xBB);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1);
        assert_eq!(ef.r1, 0xAA);
        assert_eq!(ef.r2, 0xBB);
    }

    #[test]
    fn get_time_truncates_to_u32() {
        let mut k = kernel(0, 0, 0);
        k.tick.set((1u64 << 32) + 7);
        let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 7);
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
        let src = k.sampling.create_port(PortDirection::Source, 1000).unwrap();
        let dst = k
            .sampling
            .create_port(PortDirection::Destination, 1000)
            .unwrap();
        k.sampling.connect_ports(src, dst).unwrap();
        // Write + read via pool (avoids 64-bit pointer truncation issue).
        k.sampling
            .write_sampling_message(src, &[0xAA, 0xBB], k.tick.get())
            .unwrap();
        let mut buf = [0u8; 64];
        let (n, _) = k
            .sampling
            .read_sampling_message(dst, &mut buf, k.tick.get())
            .unwrap();
        assert_eq!((n, &buf[..n]), (2, &[0xAA, 0xBB][..]));
        // Invalid port → InvalidResource error for both write and read.
        // Use an address inside partition 0's MPU region so pointer
        // validation passes and the invalid port ID (99) is reached.
        let inv = SvcError::InvalidResource.to_u32();
        let mut ef = frame4(SYS_SAMPLING_WRITE, 99, 1, 0x2000_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
        let mut ef = frame4(SYS_SAMPLING_READ, 99, 0, 0x2000_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
    }

    #[test]
    fn bb_nonblocking_read_empty_returns_error_without_enqueue() {
        use crate::blackboard::BlackboardError;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards.create().unwrap();
        let mut buf = [0u8; 64];
        // Non-blocking read (timeout=0) on empty board returns error
        assert_eq!(
            k.blackboards.read_blackboard(id, 0, &mut buf, 0),
            Err(BlackboardError::BoardEmpty)
        );
        // Caller was NOT enqueued
        assert_eq!(k.blackboards.get(id).unwrap().waiting_readers(), 0);
    }

    #[test]
    fn bb_blocking_read_and_display_wake() {
        use crate::blackboard::ReadBlackboardOutcome;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards.create().unwrap();
        let mut buf = [0u8; 64];
        // Blocking read (timeout>0) enqueues the caller
        assert_eq!(
            k.blackboards.read_blackboard(id, 0, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(k.blackboards.get(id).unwrap().waiting_readers(), 1);
        // Display wakes the blocked reader
        let woken = k.blackboards.display_blackboard(id, &[0xAA, 0xBB]).unwrap();
        assert_eq!(woken.as_slice(), &[0]);
        // Non-blocking read now succeeds
        let outcome = k.blackboards.read_blackboard(id, 0, &mut buf, 0).unwrap();
        assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 2 });
        assert_eq!(&buf[..2], &[0xAA, 0xBB]);
    }

    #[test]
    fn bb_blocking_read_wakes_partition() {
        use crate::blackboard::ReadBlackboardOutcome;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards.create().unwrap();
        let mut buf = [0u8; 64];
        // Transition partition 1 to Waiting and block it on the blackboard
        k.partitions
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert_eq!(
            k.blackboards.read_blackboard(id, 1, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        // Display wakes partition 1
        let woken = k.blackboards.display_blackboard(id, &[0x01]).unwrap();
        assert_eq!(woken.as_slice(), &[1]);
        for &pid in woken.iter() {
            try_transition(&mut k.partitions, pid, PartitionState::Ready);
        }
        assert_eq!(k.partitions.get(1).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn bb_invalid_board_errors() {
        use crate::blackboard::BlackboardError;
        let mut k = kernel(0, 0, 0);
        let r: Result<heapless::Vec<u8, 4>, _> = k.blackboards.display_blackboard(99, &[1]);
        assert_eq!(r, Err(BlackboardError::InvalidBoard));
    }

    #[test]
    fn bb_clear_svc_dispatch() {
        use crate::blackboard::BlackboardError;
        use crate::syscall::SYS_BB_CLEAR;
        let mut k = kernel(0, 0, 0);
        let id = k.blackboards.create().unwrap();
        let _ = k.blackboards.display_blackboard(id, &[42]).unwrap();
        // Clear via SVC dispatch
        let mut ef = frame(SYS_BB_CLEAR, id as u32, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Non-blocking read after clear should fail
        let mut buf = [0u8; 64];
        assert_eq!(
            k.blackboards.read_blackboard(id, 0, &mut buf, 0),
            Err(BlackboardError::BoardEmpty)
        );
        // Invalid board via SVC
        let mut ef = frame(SYS_BB_CLEAR, 99, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    }

    // ---- set_dispatch_hook / Mutex-based hook tests ----

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
        clear_dispatch_hook();
        assert!(read_dispatch_hook().is_none());
    }

    #[test]
    fn set_dispatch_hook_installs_hook() {
        unsafe extern "C" fn my_hook(_: &mut ExceptionFrame) {}
        clear_dispatch_hook();
        set_dispatch_hook(my_hook);
        assert!(read_dispatch_hook().is_some());
        clear_dispatch_hook();
    }

    #[test]
    fn dispatch_svc_uses_hook_when_set() {
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
        k.current_partition = 1; // switch to partition 1
        assert_eq!(svc!(SYS_BUF_RELEASE, 1), eres); // wrong owner
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
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        let ptr = low32_buf(0);
        // SAFETY: ptr points to a valid mmap'd page.
        unsafe { core::ptr::copy_nonoverlapping([0xAA, 0xBB, 0xCC].as_ptr(), ptr, 3) };
        let mut ef = frame4(SYS_DEV_WRITE, 0, 3, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 3);
        assert_eq!(k.uart_pair.a.pop_tx(), Some(0xAA));
        assert_eq!(k.uart_pair.a.pop_tx(), Some(0xBB));
        assert_eq!(k.uart_pair.a.pop_tx(), Some(0xCC));
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn dev_read_dispatch_routes_to_uart() {
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ};
        let mut k = kernel(0, 0, 0);
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        k.uart_pair.a.push_rx(&[0xDE, 0xAD]);
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
        let mut k = kernel(0, 0, 0);
        // Open device 1
        let mut ef = frame(SYS_DEV_OPEN, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Push some RX data into UART-B
        k.uart_pair.b.push_rx(&[1, 2, 3]);
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
                k.sampling.create_port(*dir, 1000).unwrap();
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
            k.queuing.create_port(dir).unwrap();
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
        k.blackboards.create().unwrap();
        let mut ef = frame4(crate::syscall::SYS_BB_DISPLAY, 0, 4, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(
            ef.r0,
            SvcError::InvalidPointer.to_u32(),
            "BbDisplay should reject out-of-bounds pointer"
        );

        // BbRead: r1 = board id, r2 = timeout, r3 = buf ptr (out-of-bounds)
        let mut k = kernel(0, 0, 0);
        k.blackboards.create().unwrap();
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

    #[test]
    fn queuing_recv_timed_delivers_message_and_wakes_sender() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing
            .get_mut(dst)
            .unwrap()
            .inject_message(2, &[0xAA, 0xBB]);
        k.queuing
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
        assert_eq!(k.partitions.get(1).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn queuing_recv_timed_blocks_on_empty_queue() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k.queuing.create_port(PortDirection::Destination).unwrap();
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 50, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        assert_eq!(
            k.partitions.get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.queuing.get(dst).unwrap().pending_receivers(), 1);
    }

    #[test]
    fn queuing_recv_timed_zero_timeout_returns_error() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k.queuing.create_port(PortDirection::Destination).unwrap();
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
        k.queuing.create_port(PortDirection::Destination).unwrap();
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, 0, 50, 0xDEAD_0000);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
    }

    #[test]
    fn queuing_recv_timed_received_no_sender_to_wake() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;
        let mut k = kernel(0, 0, 0);
        let dst = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing.get_mut(dst).unwrap().inject_message(1, &[42]);
        let ptr = low32_buf(0);
        let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, 100, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1, "should return msg_len=1");
        // Verify message data was actually delivered to the buffer.
        // SAFETY: ptr was mmap'd by low32_buf and dispatch wrote into it.
        let delivered = unsafe { core::slice::from_raw_parts(ptr, 1) };
        assert_eq!(delivered[0], 42, "buffer should contain the delivered byte");
        assert_eq!(
            k.partitions.get(1).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_recv_original_unchanged_still_uses_zero_timeout() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV;
        let mut k = kernel(0, 0, 0);
        let dst = k.queuing.create_port(PortDirection::Destination).unwrap();
        let ptr = low32_buf(0);
        // Empty queue with original QueuingRecv (timeout=0) → QueueEmpty → InvalidResource
        let mut ef = frame4(SYS_QUEUING_RECV, dst as u32, 0, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
        // Partition should NOT be in Waiting (no blocking happened)
        assert_eq!(
            k.partitions.get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    // ---- QueuingSendTimed dispatch tests ----

    /// Helper: create a connected Source→Destination pair, return (src_id, dst_id).
    fn connected_send_pair(k: &mut Kernel<TestConfig>) -> (usize, usize) {
        use crate::sampling::PortDirection;
        let s = k.queuing.create_port(PortDirection::Source).unwrap();
        let d = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing.connect_ports(s, d).unwrap();
        (s, d)
    }

    /// Pack r2 for QueuingSendTimed: timeout_hi16 << 16 | data_len_lo16.
    fn pack_r2(timeout: u16, data_len: u16) -> u32 {
        ((timeout as u32) << 16) | (data_len as u32)
    }

    #[test]
    fn queuing_send_timed_delivers_message_and_wakes_receiver() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Enqueue a blocked receiver on the destination port
        k.queuing
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
        assert_eq!(ef.r0, 0);
        // Blocked receiver (partition 1) should be woken to Ready
        assert_eq!(k.partitions.get(1).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn queuing_send_timed_blocks_on_full_queue() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill the destination queue to capacity (QD=4)
        for _ in 0..4 {
            k.queuing.get_mut(d).unwrap().inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        let r2 = pack_r2(50, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Caller (partition 0) should transition to Waiting
        assert_eq!(
            k.partitions.get(0).unwrap().state(),
            PartitionState::Waiting
        );
        assert_eq!(k.queuing.get(d).unwrap().pending_senders(), 1);
    }

    #[test]
    fn queuing_send_timed_zero_timeout_full_returns_error() {
        use crate::syscall::SYS_QUEUING_SEND_TIMED;
        let mut k = kernel(0, 0, 0);
        let (s, d) = connected_send_pair(&mut k);
        // Fill destination queue
        for _ in 0..4 {
            k.queuing.get_mut(d).unwrap().inject_message(1, &[0x42]);
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
        let (s, _d) = connected_send_pair(&mut k);
        let ptr = low32_buf(0);
        let r2 = pack_r2(100, 1);
        let mut ef = frame4(SYS_QUEUING_SEND_TIMED, s as u32, r2, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // No receiver was blocked, so partition 1 should remain Running
        assert_eq!(
            k.partitions.get(1).unwrap().state(),
            PartitionState::Running
        );
    }

    #[test]
    fn queuing_send_original_unchanged_still_uses_zero_timeout() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_SEND;
        let mut k = kernel(0, 0, 0);
        let s = k.queuing.create_port(PortDirection::Source).unwrap();
        let d = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing.connect_ports(s, d).unwrap();
        // Fill destination queue
        for _ in 0..4 {
            k.queuing.get_mut(d).unwrap().inject_message(1, &[0x42]);
        }
        let ptr = low32_buf(0);
        // Original QueuingSend: r2=data_len (used as raw len), timeout=0
        // Full queue with timeout=0 → QueueFull → InvalidResource
        let mut ef = frame4(SYS_QUEUING_SEND, s as u32, 1, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
        // Partition should NOT be in Waiting (no blocking with original handler)
        assert_eq!(
            k.partitions.get(0).unwrap().state(),
            PartitionState::Running
        );
    }

    // ---- hw_uart integration tests ----

    /// hw_uart starts as None; virtual UARTs still work.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_none_virtual_uarts_still_dispatch() {
        use crate::syscall::SYS_DEV_OPEN;
        let mut k = kernel(0, 0, 0);
        assert!(k.hw_uart.is_none());
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

    /// After set_hw_uart, dev_dispatch routes to the hw backend.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_set_and_dispatch_open_write_read() {
        use crate::hw_uart::HwUartBackend;
        use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE};
        use crate::uart_hal::UartRegs;
        let mut k = kernel(0, 0, 0);
        k.set_hw_uart(HwUartBackend::new(5, UartRegs::new(0x4000_C000)));
        assert!(k.hw_uart.is_some());
        // Open hw_uart device 5.
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
        k.hw_uart.as_mut().unwrap().push_rx(&[0xAA, 0xBB]);
        let mut ef = frame4(SYS_DEV_READ, 5, 4, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 2);
        // SAFETY: ptr is valid for 4096 bytes via low32_buf.
        let out = unsafe { core::slice::from_raw_parts(ptr, 2) };
        assert_eq!(out, &[0xAA, 0xBB]);
    }

    /// Virtual UARTs take priority over hw_uart when IDs overlap.
    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn hw_uart_does_not_shadow_virtual_uart() {
        use crate::hw_uart::HwUartBackend;
        use crate::syscall::SYS_DEV_OPEN;
        use crate::uart_hal::UartRegs;
        let mut k = kernel(0, 0, 0);
        // Install hw_uart with device_id = 0 (same as virtual UART-A).
        k.set_hw_uart(HwUartBackend::new(0, UartRegs::new(0x4000_C000)));
        // Open device 0 — should route to virtual UART-A, not hw_uart.
        let mut ef = frame(SYS_DEV_OPEN, 0, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // Write via device 0 goes to virtual UART-A TX.
        let ptr = low32_buf(0);
        // SAFETY: ptr points to a valid mmap'd page within MPU region.
        unsafe { core::ptr::copy_nonoverlapping([0xFF].as_ptr(), ptr, 1) };
        let mut ef = frame4(crate::syscall::SYS_DEV_WRITE, 0, 1, ptr as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 1);
        // Byte should be in virtual UART-A's TX, not hw_uart's TX.
        assert_eq!(k.uart_pair.a.pop_tx(), Some(0xFF));
        assert_eq!(k.hw_uart.as_ref().unwrap().tx_len(), 0);
    }
}
