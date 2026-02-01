use crate::context::ExceptionFrame;
use crate::events;
use crate::message::{MessagePool, RecvOutcome, SendOutcome};
use crate::mutex::MutexPool;
use crate::partition::{PartitionState, PartitionTable};
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

/// Dispatch an SVC call based on the syscall number in `frame.r0`.
///
/// # Safety
///
/// The caller must pass a valid pointer to the hardware-stacked
/// `ExceptionFrame` on the process stack (PSP). This is guaranteed
/// when called from the SVCall assembly trampoline above.
///
/// # Note
///
/// Event syscalls require a partition table. In production, the caller
/// must wire a global `KernelState` so that `dispatch_svc` can obtain
/// `&mut PartitionTable`. See [`dispatch_syscall`] for the core logic.
// TODO: wire a global KernelState pointer so dispatch_svc can forward
// event syscalls in the real (non-test) interrupt handler path.
#[no_mangle]
pub unsafe extern "C" fn dispatch_svc(frame: &mut ExceptionFrame) {
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(SyscallId::EventWait | SyscallId::EventSet | SyscallId::EventClear) => {
            // Event syscalls require kernel state; in the real handler this
            // would go through dispatch_syscall with the global partition table.
            1
        }
        Some(_) => 1,
        None => u32::MAX,
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
        None => u32::MAX,
    };
}

/// Encapsulates all kernel service pools alongside the partition table,
/// reducing const-generic parameter explosion in `dispatch_all`.
pub struct Kernel<
    const N: usize,
    const S: usize,
    const SW: usize,
    const MS: usize,
    const MW: usize,
    const QS: usize,
    const QD: usize,
    const QM: usize,
    const QW: usize,
    const SP: usize,
    const SM: usize,
> {
    pub partitions: PartitionTable<N>,
    pub semaphores: SemaphorePool<S, SW>,
    pub mutexes: MutexPool<MS, MW>,
    pub messages: MessagePool<QS, QD, QM, QW>,
    pub tick: TickCounter,
    pub sampling: SamplingPortPool<SP, SM>,
}

impl<
        const N: usize,
        const S: usize,
        const SW: usize,
        const MS: usize,
        const MW: usize,
        const QS: usize,
        const QD: usize,
        const QM: usize,
        const QW: usize,
        const SP: usize,
        const SM: usize,
    > Kernel<N, S, SW, MS, MW, QS, QD, QM, QW, SP, SM>
{
    /// Full syscall dispatch including semaphore, mutex, and message operations.
    ///
    /// Frame register convention:
    /// - `r0`: syscall ID (overwritten with return value)
    /// - `r1`: resource ID (semaphore/mutex/queue index)
    /// - `r2`: caller partition index
    /// - `r3`: pointer to user data buffer (for msg_send/msg_recv)
    ///
    /// For `MsgSend`: the kernel copies `QM` bytes from `r3` (data_ptr) into
    /// the queue. For `MsgRecv`: the kernel copies `QM` bytes from the queue
    /// into `r3` (buf_ptr). This implements true pointer-based inter-partition
    /// data transfer.
    ///
    /// # Safety
    ///
    /// For message syscalls, `r3` must point to a readable (send) or writable
    /// (recv) buffer of at least `QM` bytes within the calling partition's
    /// memory region. The caller is responsible for ensuring this; in
    /// production the MPU enforces partition isolation.
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
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::SemSignal) => {
                match self
                    .semaphores
                    .signal(&mut self.partitions, frame.r1 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::MutexLock) => {
                match self
                    .mutexes
                    .lock(&mut self.partitions, frame.r1 as usize, frame.r2 as usize)
                {
                    Ok(()) => 0,
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::MutexUnlock) => {
                match self.mutexes.unlock(
                    &mut self.partitions,
                    frame.r1 as usize,
                    frame.r2 as usize,
                ) {
                    Ok(()) => 0,
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::MsgSend) => {
                let data_ptr = frame.r3 as *const u8;
                let data = unsafe { core::slice::from_raw_parts(data_ptr, QM) };
                match self
                    .messages
                    .send(frame.r1 as usize, frame.r2 as usize, data)
                {
                    Ok(outcome) => apply_send_outcome(&mut self.partitions, outcome),
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::MsgRecv) => {
                let buf_ptr = frame.r3 as *mut u8;
                let buf = unsafe { core::slice::from_raw_parts_mut(buf_ptr, QM) };
                match self
                    .messages
                    .recv(frame.r1 as usize, frame.r2 as usize, buf)
                {
                    Ok(outcome) => apply_recv_outcome(&mut self.partitions, outcome),
                    Err(_) => u32::MAX,
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
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::SamplingRead) => {
                let b = unsafe { core::slice::from_raw_parts_mut(frame.r3 as *mut u8, SM) };
                match self
                    .sampling
                    .read_sampling_message(frame.r1 as usize, b, self.tick.get())
                {
                    Ok((sz, _)) => sz as u32,
                    Err(_) => u32::MAX,
                }
            }
            Some(SyscallId::GetTime) => self.tick.get() as u32,
            None => u32::MAX,
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
                return u32::MAX;
            }
            0
        }
        SendOutcome::Delivered {
            wake_receiver: None,
        } => 0,
        SendOutcome::SenderBlocked { blocked } => {
            if !try_transition(partitions, blocked, PartitionState::Waiting) {
                return u32::MAX;
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
                return u32::MAX;
            }
            0
        }
        RecvOutcome::Received { wake_sender: None } => 0,
        RecvOutcome::ReceiverBlocked { blocked } => {
            if !try_transition(partitions, blocked, PartitionState::Waiting) {
                return u32::MAX;
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
    use crate::message::MessageQueue;
    use crate::partition::{MpuRegion, PartitionControlBlock};
    use crate::semaphore::Semaphore;
    use crate::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_YIELD};

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
    fn kernel(
        sem_count: usize,
        mtx_count: usize,
        msg_queue_count: usize,
    ) -> Kernel<4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 64> {
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
    fn invalid_syscall_returns_max() {
        let mut ef = frame(0xFFFF, 0, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
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
    fn event_invalid_partition_returns_max() {
        let mut t = tbl();
        let mut ef = frame(SYS_EVT_WAIT, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
        let mut ef = frame(SYS_EVT_SET, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
        let mut ef = frame(SYS_EVT_CLEAR, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
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
        // Invalid port → u32::MAX for both write and read.
        let d: [u8; 1] = [1];
        let mut ef = frame4(SYS_SAMPLING_WRITE, 99, 1, d.as_ptr() as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, u32::MAX);
        let mut rb = [0u8; 64];
        let mut ef = frame4(SYS_SAMPLING_READ, 99, 0, rb.as_mut_ptr() as u32);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, u32::MAX);
    }
}
