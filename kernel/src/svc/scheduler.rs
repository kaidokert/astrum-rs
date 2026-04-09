use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::invariants::assert_partition_state_consistency;
use crate::message::MessagePool;
use crate::mutex::MutexPool;
use crate::partition::{PartitionState, PartitionTable};
use crate::queuing::QueuingPortPool;
use crate::sampling::SamplingPortPool;
use crate::scheduler::ScheduleEvent;
use crate::scheduler::ScheduleTable;
use crate::semaphore::SemaphorePool;
use crate::svc::{try_transition, Kernel};
use core::sync::atomic::{AtomicU8, Ordering};
use rtos_traits::ids::PartitionId;
#[cfg(feature = "intra-threads")]
use rtos_traits::ids::ThreadId;

// ---------------------------------------------------------------------------
// Pending thread-switch flag
// ---------------------------------------------------------------------------

/// Sentinel meaning "no thread switch pending".
const NO_PENDING: u8 = 0xFF;

/// Atomic flag holding the outgoing thread ID of a pending intra-partition
/// thread switch, or `0xFF` when no switch is pending.
static PENDING_THREAD_SWITCH: AtomicU8 = AtomicU8::new(NO_PENDING);

/// Record that an intra-partition thread switch is pending for `outgoing_tid`.
pub fn set_pending_thread_switch(outgoing_tid: u8) {
    PENDING_THREAD_SWITCH.store(outgoing_tid, Ordering::Release);
}

/// Atomically read and clear the pending thread-switch flag.
///
/// Returns `Some(outgoing_tid)` if a switch was pending, or `None` if no
/// switch was pending (the flag was already `0xFF`).
pub fn take_pending_thread_switch() -> Option<u8> {
    let val = PENDING_THREAD_SWITCH.swap(NO_PENDING, Ordering::AcqRel);
    if val == NO_PENDING {
        None
    } else {
        Some(val)
    }
}

/// Check whether a thread switch is pending without consuming the flag.
pub fn is_thread_switch_pending() -> bool {
    PENDING_THREAD_SWITCH.load(Ordering::Acquire) != NO_PENDING
}

// TODO: The where clause duplicates the full Kernel bounds from svc.rs. This is
// required by Rust because partitions()/schedule() are defined in the main impl
// block which carries these bounds. The C::Sync, C::Msg, and C::Ports bounds are
// not semantically needed by these functions but are required for method resolution.

/// Transition the active partition from `Running` to `Ready`.
///
/// If `active_partition` is `Some` and that partition is currently
/// `Running`, it is moved to `Ready`. Otherwise this is a no-op.
pub(crate) fn transition_outgoing_ready<'mem, C: KernelConfig>(kernel: &mut Kernel<'mem, C>)
where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
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
    if let Some(old_pid) = kernel.active_partition {
        let state = kernel.pcb(old_pid as usize).map(|pcb| pcb.state());
        if state == Some(PartitionState::Running) {
            try_transition(
                kernel.partitions_mut(),
                PartitionId::new(old_pid as u32),
                PartitionState::Ready,
            );
        }
        // Faulted is terminal — do not attempt any transition.
    }
    debug_assert!(
        {
            assert_partition_state_consistency(kernel.partitions().as_slice());
            true
        },
        "at-most-one-Running invariant violated after transition_outgoing_ready"
    );
}

/// Start the schedule and return the initial partition ID.
///
/// Calls `kernel.schedule_mut().start()` to initialize the schedule table's
/// internal state (resetting to the first slot). Returns the partition
/// ID of the first schedule entry, or `None` if the schedule is empty.
///
/// The first partition's `run_count` is incremented here so that
/// `sys_get_partition_run_count` returns ≥ 1 during the partition's
/// very first execution window — matching the invariant that every
/// scheduled execution is counted (see `advance_schedule_tick`).
pub fn start_schedule<'mem, C: KernelConfig>(kernel: &mut Kernel<'mem, C>) -> Option<u8>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
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
    kernel.schedule_mut().start();
    let first_pid = kernel.schedule().current_partition();
    if let Some(pid) = first_pid {
        kernel.active_partition = Some(pid);
        if let Some(pcb) = kernel.pcb_mut(pid as usize) {
            pcb.increment_run_count();
        }
    }
    first_pid
}

pub fn advance_schedule_tick<'mem, C: KernelConfig>(kernel: &mut Kernel<'mem, C>) -> ScheduleEvent
where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
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
    kernel.tick.increment();
    kernel.mpu_tick_bookkeeping(false);
    let event = kernel.schedule_mut().advance_tick();
    match event {
        ScheduleEvent::PartitionSwitch(pid) => {
            let target_state = kernel.pcb(pid as usize).map(|pcb| pcb.state());
            let target_skippable = matches!(
                target_state,
                Some(PartitionState::Waiting) | Some(PartitionState::Faulted)
            );
            if target_skippable {
                kernel.increment_starvation_for_ready_partitions();
                if let Some(ap) = kernel.active_partition {
                    if kernel
                        .pcb(ap as usize)
                        .is_some_and(|p| p.state() == PartitionState::Waiting)
                        && try_transition(
                            kernel.partitions_mut(),
                            PartitionId::new(ap as u32),
                            PartitionState::Ready,
                        )
                    {
                        kernel.set_next_partition(ap);
                    }
                }
                return ScheduleEvent::None;
            }
            transition_outgoing_ready(kernel);
            if let Some(pcb) = kernel.pcb_mut(pid as usize) {
                pcb.reset_starvation();
                pcb.increment_run_count();
            }
            kernel.active_partition = Some(pid);
            kernel.set_next_partition(pid);
            event
        }
        ScheduleEvent::SystemWindow => {
            kernel.mpu_tick_bookkeeping(true);
            event
        }
        _ => event,
    }
}

#[cfg(feature = "intra-threads")]
/// Advance the intra-partition thread schedule for the active partition.
///
/// If the active partition has more than one runnable thread, this function:
/// 1. Returns false early if a thread switch is already pending (re-advance guard)
/// 2. Saves `partition_sp[pid]` into the outgoing thread's `TCB.stack_pointer`
/// 3. Calls `advance_intra_schedule()` on the partition's `ThreadTable`
/// 4. Sets the pending thread-switch flag with the outgoing thread ID
///
/// The actual SP swap into `partition_sp` is deferred to PendSV, which reads
/// the pending flag via `take_pending_thread_switch()`.
///
/// Returns `true` if a thread switch was scheduled (the active thread changed),
/// so the caller can trigger PendSV to perform the hardware context switch.
///
/// Partitions with fewer than 2 runnable threads are skipped (zero overhead).
// TODO: If a partition switch just occurred in advance_schedule_tick, the newly
// selected partition's partition_sp (its last saved state) is saved into the
// outgoing thread's TCB and then the schedule advances, causing the partition to
// skip a thread upon resumption. This is currently acceptable for fairness but
// should be revisited as an explicit design choice.
pub(crate) fn advance_intra_thread_schedule<'mem, C: KernelConfig>(
    kernel: &mut Kernel<'mem, C>,
) -> bool
where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
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
    // Guard: if a thread switch is already pending, do not re-advance.
    if is_thread_switch_pending() {
        return false;
    }

    let pid = match kernel.active_partition {
        Some(pid) => pid as usize,
        None => return false,
    };

    // Read partition_sp before borrowing PCB (avoids overlapping borrows).
    let current_sp = match kernel.get_sp(pid) {
        Some(sp) => sp,
        None => return false,
    };

    // Phase 1: access PCB, check runnable count, save outgoing SP, advance,
    // determine whether a thread switch occurred.
    let outgoing_tid = {
        let pcb = match kernel.pcb_mut(pid) {
            Some(pcb) => pcb,
            None => return false,
        };

        // TODO: runnable_count() performs a linear O(N) scan on every SysTick.
        // A cached counter in ThreadTable would be more efficient for high-frequency
        // interrupt context. Deferred: acceptable for current max_threads (≤4).
        if pcb.thread_table().runnable_count() <= 1 {
            return false; // Zero or one runnable thread: nothing to switch to.
        }

        let prev_tid = pcb.thread_table().current_thread_id();

        // Save current partition_sp to the outgoing (currently running) thread's TCB.
        if let Some(tid) = prev_tid {
            if let Some(tcb) = pcb.thread_table_mut().get_mut(tid) {
                tcb.stack_pointer = current_sp;
            }
        }

        // Advance the intra-partition schedule (round-robin or priority).
        let new_tid = pcb.thread_table_mut().advance_intra_schedule();

        // Only flag a switch if the thread actually changed.
        match (prev_tid, new_tid) {
            (Some(prev), Some(next)) if prev != next => Some(prev),
            _ => None,
        }
    }; // PCB borrow ends here.

    // Phase 2: set the pending flag so PendSV can perform the actual SP swap.
    if let Some(tid) = outgoing_tid {
        set_pending_thread_switch(tid.into());
        true
    } else {
        false
    }
}

#[cfg(feature = "intra-threads")]
#[allow(dead_code)] // will be called from PendSV handler in a follow-up
/// Apply a pending intra-partition thread switch.
///
/// If `take_pending_thread_switch()` yields an outgoing thread ID:
/// 1. Saves the current `partition_sp[pid]` (which `context_save` just wrote)
///    into the outgoing thread's `TCB.stack_pointer`.
/// 2. Reads the incoming (current) thread's `TCB.stack_pointer`.
/// 3. Writes that SP into `partition_sp[pid]` so PendSV restores it.
///
/// This is a no-op when no switch is pending.
pub(crate) fn apply_pending_thread_switch<'mem, C: KernelConfig>(kernel: &mut Kernel<'mem, C>)
where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
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
    let outgoing_tid = match take_pending_thread_switch() {
        Some(tid) => tid,
        None => return,
    };

    let pid = match kernel.active_partition {
        Some(pid) => pid as usize,
        None => return,
    };

    // Read partition_sp[pid] — the live PSP that context_save just wrote.
    let saved_sp = match kernel.get_sp(pid) {
        Some(sp) => sp,
        None => return,
    };

    // Save partition_sp into the outgoing thread's TCB, then read incoming
    // thread's SP and write it to partition_sp. Split into two phases to
    // avoid overlapping borrows on the PCB.
    let incoming_sp = {
        let pcb = match kernel.pcb_mut(pid) {
            Some(pcb) => pcb,
            None => return,
        };

        // Save the live PSP into the outgoing thread's TCB.
        if let Some(tcb) = pcb.thread_table_mut().get_mut(ThreadId::new(outgoing_tid)) {
            tcb.stack_pointer = saved_sp;
        }

        // Read the incoming (now-current) thread's SP.
        let current_tid = pcb.thread_table().current_thread_id();
        match current_tid.and_then(|tid| pcb.thread_table().get(tid)) {
            Some(tcb) => tcb.stack_pointer,
            None => return,
        }
    };

    // Write the incoming thread's SP to partition_sp so PendSV restores it.
    kernel.set_sp(pid, incoming_sp);
}
