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
}

impl SvcError {
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
        }
    }
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

/// Safely install an application-provided SVC dispatch hook.
///
/// The hook function will be called by `dispatch_svc` for every SVC
/// exception instead of the built-in minimal handler.
///
/// # Safety
///
/// The caller must ensure `hook` points to a function with the same safety
/// requirements as `dispatch_svc` and that it remains valid for the lifetime
/// of the program (typically a `static` function).
pub unsafe fn set_dispatch_hook(hook: unsafe extern "C" fn(&mut ExceptionFrame)) {
    with_cs(|cs| {
        SVC_DISPATCH_HOOK.borrow(cs).replace(Some(hook));
    });
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
            Some(SyscallId::Yield) => handle_yield(),
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
            Some(SyscallId::MsgSend) => {
                let data_ptr = frame.r3 as *const u8;
                let data = unsafe { core::slice::from_raw_parts(data_ptr, C::QM) };
                match self
                    .messages
                    .send(frame.r1 as usize, frame.r2 as usize, data)
                {
                    Ok(outcome) => apply_send_outcome(&mut self.partitions, outcome),
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::MsgRecv) => {
                let buf_ptr = frame.r3 as *mut u8;
                let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, C::QM) };
                match self
                    .messages
                    .recv(frame.r1 as usize, frame.r2 as usize, buf)
                {
                    Ok(outcome) => apply_recv_outcome(&mut self.partitions, outcome),
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::SamplingWrite) => {
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
            }
            Some(SyscallId::SamplingRead) => {
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, C::SM) };
                match self
                    .sampling
                    .read_sampling_message(frame.r1 as usize, b, self.tick.get())
                {
                    Ok((sz, _)) => sz as u32,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::QueuingSend) => {
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
            }
            Some(SyscallId::QueuingRecv) => {
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
            }
            Some(SyscallId::QueuingStatus) => {
                let status_ptr = frame.r2 as *mut QueuingPortStatus;
                match self.queuing.get_queuing_port_status(frame.r1 as usize) {
                    Ok(status) => {
                        unsafe { core::ptr::write(status_ptr, status) };
                        0
                    }
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::BbDisplay) => {
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
            }
            Some(SyscallId::BbRead) => {
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
            }
            Some(SyscallId::BbClear) => {
                match self.blackboards.clear_blackboard(frame.r1 as usize) {
                    Ok(()) => 0,
                    Err(_) => SvcError::InvalidResource.to_u32(),
                }
            }
            Some(SyscallId::GetTime) => self.tick.get() as u32,
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
        let inv = SvcError::InvalidResource.to_u32();
        let d: [u8; 1] = [1];
        let mut ef = frame4(SYS_SAMPLING_WRITE, 99, 1, d.as_ptr() as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, inv);
        let mut rb = [0u8; 64];
        let mut ef = frame4(SYS_SAMPLING_READ, 99, 0, rb.as_mut_ptr() as u32);
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
        // SAFETY: my_hook is a valid static function.
        unsafe { set_dispatch_hook(my_hook) };
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
        // SAFETY: sentinel_hook is a valid static function.
        unsafe { set_dispatch_hook(sentinel_hook) };
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
}
