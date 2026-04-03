use super::{try_transition, Kernel};
use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::message::MessagePool;
use crate::mutex::MutexPool;
use crate::partition::{PartitionControlBlock, PartitionState, PartitionTable};
use crate::queuing::QueuingPortPool;
use crate::sampling::SamplingPortPool;
use crate::scheduler::ScheduleTable;
use crate::semaphore::SemaphorePool;
use rtos_traits::ids::PartitionId;

impl<'mem, C: KernelConfig> Kernel<'mem, C>
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
    /// Returns an immutable reference to the partition table.
    #[inline(always)]
    pub fn partitions(&self) -> &PartitionTable<{ C::N }> {
        self.core.partitions()
    }

    /// Returns a mutable reference to the partition table.
    #[inline(always)]
    pub fn partitions_mut(&mut self) -> &mut PartitionTable<{ C::N }> {
        self.core.partitions_mut()
    }

    /// Returns a reference to the partition control block at `index`, if valid.
    #[inline(always)]
    pub fn pcb(&self, index: usize) -> Option<&PartitionControlBlock> {
        self.partitions().get(index)
    }

    /// Returns a mutable reference to the partition control block at `index`, if valid.
    #[inline(always)]
    pub fn pcb_mut(&mut self, index: usize) -> Option<&mut PartitionControlBlock> {
        self.partitions_mut().get_mut(index)
    }

    /// Returns the number of partitions currently registered.
    #[inline(always)]
    pub fn partition_count(&self) -> usize {
        self.partitions().len()
    }

    /// Returns a slice of all partition control blocks.
    #[inline(always)]
    pub fn partition_slice(&self) -> &[PartitionControlBlock] {
        CoreOps::partition_slice(&self.core)
    }

    /// Returns a mutable slice of all partition control blocks.
    #[inline(always)]
    pub fn partition_slice_mut(&mut self) -> &mut [PartitionControlBlock] {
        CoreOps::partition_slice_mut(&mut self.core)
    }

    /// Returns an immutable reference to the schedule table.
    #[inline(always)]
    pub fn schedule(&self) -> &ScheduleTable<{ C::SCHED }> {
        self.core.schedule()
    }

    /// Returns a mutable reference to the schedule table.
    #[inline(always)]
    pub fn schedule_mut(&mut self) -> &mut ScheduleTable<{ C::SCHED }> {
        self.core.schedule_mut()
    }

    /// Returns the current partition index stored in core.
    #[inline(always)]
    pub fn core_current_partition(&self) -> u8 {
        self.core.current_partition()
    }

    /// Sets the current partition index in core.
    #[inline(always)]
    pub fn set_core_current_partition(&mut self, id: u8) {
        self.core.set_current_partition(id);
    }

    /// Returns the next partition index.
    #[inline(always)]
    pub fn next_partition(&self) -> u8 {
        self.core.next_partition()
    }

    /// Sets the next partition index and transitions it to Running state.
    ///
    /// Single point where the scheduler selects a partition to run. The state
    /// transition to Running happens exactly once per scheduling decision.
    ///
    /// If the partition cannot transition (already Running or incompatible
    /// state), the transition is silently skipped — a Running partition needs
    /// no transition, and a Waiting one will yield and be rescheduled.
    pub fn set_next_partition(&mut self, id: u8) {
        // Transition the incoming partition to Running so syscalls can block it.
        // This is the authoritative location for this state transition.
        let _ = try_transition(
            self.partitions_mut(),
            PartitionId::new(id as u32),
            PartitionState::Running,
        );
        self.core.set_next_partition(id);
    }

    /// Gets the stack pointer for a partition by index.
    #[inline(always)]
    pub fn get_sp(&self, index: usize) -> Option<u32> {
        self.core.get_sp(index)
    }

    /// Sets the stack pointer for a partition by index. Returns true if valid.
    #[inline(always)]
    pub fn set_sp(&mut self, index: usize, sp: u32) -> bool {
        self.core.set_sp(index, sp)
    }
}

#[cfg(test)]
mod tests {
    use crate::partition::PartitionState;
    use crate::test_harness::KernelTestHarness;

    #[test]
    fn all_runnable_faulted_false_when_partitions_are_ready_or_running() {
        let h = KernelTestHarness::with_partitions(2).unwrap();
        // Partition 0 is Running, partition 1 is Ready — not all faulted.
        assert!(!h.kernel().all_runnable_faulted());
    }

    #[test]
    fn all_runnable_faulted_true_when_all_faulted() {
        let mut h = KernelTestHarness::with_partitions(2).unwrap();
        // Transition both partitions to Faulted.
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Faulted)
            .unwrap();
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Faulted)
            .unwrap();
        assert!(h.kernel().all_runnable_faulted());
    }

    #[test]
    fn all_runnable_faulted_false_when_one_ready_one_faulted() {
        let mut h = KernelTestHarness::with_partitions(2).unwrap();
        // Fault partition 0 (Running -> Faulted), leave partition 1 as Ready.
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Faulted)
            .unwrap();
        assert!(!h.kernel().all_runnable_faulted());
    }

    #[test]
    fn all_runnable_faulted_true_with_single_faulted_partition() {
        let mut h = KernelTestHarness::with_partitions(1).unwrap();
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Faulted)
            .unwrap();
        assert!(h.kernel().all_runnable_faulted());
    }

    #[test]
    fn all_runnable_faulted_false_with_waiting_partition() {
        let mut h = KernelTestHarness::with_partitions(2).unwrap();
        // Fault partition 0, make partition 1 Waiting (Ready -> Running -> Waiting).
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Faulted)
            .unwrap();
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        assert!(!h.kernel().all_runnable_faulted());
    }
}
