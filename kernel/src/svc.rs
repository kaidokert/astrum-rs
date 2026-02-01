use crate::context::ExceptionFrame;
use crate::events;
use crate::partition::PartitionTable;
use crate::semaphore::SemaphorePool;
use crate::syscall::SyscallId;

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

/// Full syscall dispatch including semaphore operations.
/// r1 = semaphore id, r2 = caller partition index (for SemWait).
pub fn dispatch_all<const N: usize, const S: usize, const W: usize>(
    frame: &mut ExceptionFrame,
    partitions: &mut PartitionTable<N>,
    semaphores: &mut SemaphorePool<S, W>,
) {
    frame.r0 = match SyscallId::from_u32(frame.r0) {
        Some(SyscallId::Yield) => handle_yield(),
        Some(SyscallId::EventWait) => events::event_wait(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::EventSet) => events::event_set(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::EventClear) => events::event_clear(partitions, frame.r1 as usize, frame.r2),
        Some(SyscallId::SemWait) => {
            match semaphores.wait(partitions, frame.r1 as usize, frame.r2 as usize) {
                Ok(()) => 0,
                Err(_) => u32::MAX,
            }
        }
        Some(SyscallId::SemSignal) => match semaphores.signal(partitions, frame.r1 as usize) {
            Ok(()) => 0,
            Err(_) => u32::MAX,
        },
        Some(_) => 1,
        None => u32::MAX,
    };
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
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};
    use crate::semaphore::Semaphore;
    use crate::syscall::{SYS_EVT_CLEAR, SYS_EVT_SET, SYS_EVT_WAIT, SYS_YIELD};

    #[rustfmt::skip]
    fn f(r0: u32, r1: u32, r2: u32) -> ExceptionFrame {
        ExceptionFrame { r0, r1, r2, r3: 0xCC, r12: 0, lr: 0, pc: 0, xpsr: 0 }
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

    #[test]
    fn yield_returns_zero_and_preserves_regs() {
        let mut ef = f(SYS_YIELD, 0xAA, 0xBB);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!((ef.r0, ef.r1, ef.r2, ef.r3), (0, 0xAA, 0xBB, 0xCC));
    }
    #[test]
    fn invalid_syscall_returns_max() {
        let mut ef = f(0xFFFF, 0, 0);
        let mut t = tbl();
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
    }
    #[test]
    fn event_wait_dispatches_to_events_module() {
        let mut t = tbl();
        // Set flags on partition 0, then dispatch EventWait via the SVC path
        events::event_set(&mut t, 0, 0b1010);
        let mut ef = f(SYS_EVT_WAIT, 0, 0b1110);
        dispatch_syscall(&mut ef, &mut t);
        // Should return matched bits and clear them
        assert_eq!(ef.r0, 0b1010);
        assert_eq!(t.get(0).unwrap().event_flags(), 0);
    }
    #[test]
    fn event_set_dispatches_to_events_module() {
        let mut t = tbl();
        let mut ef = f(SYS_EVT_SET, 1, 0b0101);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, 0);
        assert_eq!(t.get(1).unwrap().event_flags(), 0b0101);
    }
    #[test]
    fn event_clear_dispatches_to_events_module() {
        let mut t = tbl();
        events::event_set(&mut t, 0, 0b1111);
        let mut ef = f(SYS_EVT_CLEAR, 0, 0b0101);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, 0);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1010);
    }
    #[test]
    fn event_invalid_partition_returns_max() {
        let mut t = tbl();
        let mut ef = f(SYS_EVT_WAIT, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
        let mut ef = f(SYS_EVT_SET, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
        let mut ef = f(SYS_EVT_CLEAR, 99, 0b0001);
        dispatch_syscall(&mut ef, &mut t);
        assert_eq!(ef.r0, u32::MAX);
    }
    #[test]
    fn sem_wait_and_signal_dispatch() {
        let mut t = tbl();
        let mut s = SemaphorePool::<4, 4>::new();
        s.add(Semaphore::new(1, 2)).unwrap();
        let mut ef = f(crate::syscall::SYS_SEM_WAIT, 0, 0);
        dispatch_all(&mut ef, &mut t, &mut s);
        assert_eq!(ef.r0, 0);
        assert_eq!(s.get(0).unwrap().count(), 0);
        let mut ef = f(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
        dispatch_all(&mut ef, &mut t, &mut s);
        assert_eq!(ef.r0, 0);
        assert_eq!(s.get(0).unwrap().count(), 1);
    }
}
