use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::invariants::assert_partition_state_consistency;
use crate::message::MessagePool;
use crate::mutex::MutexPool;
use crate::partition::{PartitionState, PartitionTable};
use crate::queuing::QueuingPortPool;
use crate::sampling::SamplingPortPool;
use crate::scheduler::ScheduleTable;
use crate::semaphore::SemaphorePool;
use crate::svc::{try_transition, Kernel};

// TODO: The where clause duplicates the full Kernel bounds from svc.rs. This is
// required by Rust because partitions()/schedule() are defined in the main impl
// block which carries these bounds. The C::Sync, C::Msg, and C::Ports bounds are
// not semantically needed by these methods but are required for method resolution.
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
    /// Transition the active partition from `Running` to `Ready`.
    ///
    /// If `active_partition` is `Some` and that partition is currently
    /// `Running`, it is moved to `Ready`. Otherwise this is a no-op.
    pub(crate) fn transition_outgoing_ready(&mut self) {
        if let Some(old_pid) = self.active_partition {
            let is_running = self
                .partitions()
                .get(old_pid as usize)
                .map(|pcb| pcb.state() == PartitionState::Running)
                .unwrap_or(false);
            if is_running {
                try_transition(self.partitions_mut(), old_pid, PartitionState::Ready);
            }
        }
        debug_assert!(
            {
                assert_partition_state_consistency(self.partitions().as_slice());
                true
            },
            "at-most-one-Running invariant violated after transition_outgoing_ready"
        );
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

    pub fn advance_schedule_tick(&mut self) -> crate::scheduler::ScheduleEvent {
        use crate::scheduler::ScheduleEvent;
        self.tick.increment();
        #[cfg(feature = "dynamic-mpu")]
        self.mpu_tick_bookkeeping(false);
        let event = self.schedule_mut().advance_tick();
        match event {
            ScheduleEvent::PartitionSwitch(pid) => {
                let target_waiting = self
                    .partitions()
                    .get(pid as usize)
                    .is_some_and(|pcb| pcb.state() == PartitionState::Waiting);
                if target_waiting {
                    if let Some(pcb) = self.partitions_mut().get_mut(pid as usize) {
                        pcb.increment_starvation();
                        if pcb.is_starved() {
                            crate::klog!(
                                "partition {} starved (count={})",
                                pid,
                                pcb.starvation_count()
                            );
                        }
                    }
                    if let Some(ap) = self.active_partition {
                        if self
                            .partitions()
                            .get(ap as usize)
                            .is_some_and(|p| p.state() == PartitionState::Waiting)
                            && try_transition(self.partitions_mut(), ap, PartitionState::Ready)
                        {
                            self.set_next_partition(ap);
                        }
                    }
                    return ScheduleEvent::None;
                }
                self.transition_outgoing_ready();
                if let Some(pcb) = self.partitions_mut().get_mut(pid as usize) {
                    pcb.reset_starvation();
                }
                self.active_partition = Some(pid);
                self.set_next_partition(pid);
                event
            }
            #[cfg(feature = "dynamic-mpu")]
            ScheduleEvent::SystemWindow => {
                self.mpu_tick_bookkeeping(true);
                event
            }
            _ => event,
        }
    }
}
