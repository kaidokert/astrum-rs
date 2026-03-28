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
            try_transition(kernel.partitions_mut(), old_pid, PartitionState::Ready);
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
                        && try_transition(kernel.partitions_mut(), ap, PartitionState::Ready)
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
