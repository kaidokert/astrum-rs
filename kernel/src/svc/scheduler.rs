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
use rtos_traits::ids::PartitionId;

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

/// Advance the intra-partition thread schedule for the active partition.
///
/// If the active partition has more than one runnable thread, this function:
/// 1. Saves `partition_sp[pid]` into the outgoing thread's `TCB.stack_pointer`
/// 2. Calls `advance_intra_schedule()` on the partition's `ThreadTable`
/// 3. Loads the incoming thread's `TCB.stack_pointer` into `partition_sp[pid]`
///
/// Returns `true` if a thread switch actually occurred (the active thread changed),
/// so the caller can trigger PendSV to perform the hardware context switch.
///
/// Partitions with fewer than 2 runnable threads are skipped (zero overhead).
///
/// This must run after `advance_schedule_tick` so that `active_partition` reflects
/// the partition that is about to execute, and before PendSV fires so that
/// `partition_sp` contains the correct thread's SP for context restore.
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
    let pid = match kernel.active_partition {
        Some(pid) => pid as usize,
        None => return false,
    };

    // Read partition_sp before borrowing PCB (avoids overlapping borrows).
    let current_sp = match kernel.get_sp(pid) {
        Some(sp) => sp,
        None => return false,
    };

    // Phase 1: access PCB, check runnable count, save outgoing SP, advance, read incoming SP.
    let switch_sp = {
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

        // Only produce a new SP if the thread actually changed.
        match (prev_tid, new_tid) {
            (Some(prev), Some(next)) if prev != next => {
                pcb.thread_table().get(next).map(|t| t.stack_pointer)
            }
            _ => None,
        }
    }; // PCB borrow ends here.

    // Phase 2: update partition_sp so PendSV restores the correct thread.
    if let Some(sp) = switch_sp {
        kernel.set_sp(pid, sp);
        true
    } else {
        false
    }
}
