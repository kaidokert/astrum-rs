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
pub(crate) fn transition_outgoing_ready<C: KernelConfig>(kernel: &mut Kernel<C>)
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
    if let Some(old_pid) = kernel.active_partition {
        let is_running = kernel
            .pcb(old_pid as usize)
            .map(|pcb| pcb.state() == PartitionState::Running)
            .unwrap_or(false);
        if is_running {
            try_transition(kernel.partitions_mut(), old_pid, PartitionState::Ready);
        }
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
pub fn start_schedule<C: KernelConfig>(kernel: &mut Kernel<C>) -> Option<u8>
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
    kernel.schedule_mut().start();
    let first_pid = kernel.schedule().current_partition();
    if let Some(pid) = first_pid {
        kernel.active_partition = Some(pid);
    }
    first_pid
}

pub fn advance_schedule_tick<C: KernelConfig>(kernel: &mut Kernel<C>) -> ScheduleEvent
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
    kernel.tick.increment();
    #[cfg(feature = "dynamic-mpu")]
    kernel.mpu_tick_bookkeeping(false);
    let event = kernel.schedule_mut().advance_tick();
    match event {
        ScheduleEvent::PartitionSwitch(pid) => {
            let target_waiting = kernel
                .pcb(pid as usize)
                .is_some_and(|pcb| pcb.state() == PartitionState::Waiting);
            if target_waiting {
                // Increment starvation for Ready partitions that are not
                // currently active — they want to run but the schedule slot
                // was wasted on a Waiting partition.
                let active = kernel.active_partition;
                for pcb in kernel.partitions_mut().iter_mut() {
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
            }
            kernel.active_partition = Some(pid);
            kernel.set_next_partition(pid);
            event
        }
        #[cfg(feature = "dynamic-mpu")]
        ScheduleEvent::SystemWindow => {
            kernel.mpu_tick_bookkeeping(true);
            event
        }
        _ => event,
    }
}
