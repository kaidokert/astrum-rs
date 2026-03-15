use super::Kernel;
use crate::blackboard::BlackboardPool;
use crate::config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps};
use crate::message::MessagePool;
use crate::mutex::MutexPool;
use crate::partition::{PartitionControlBlock, PartitionTable};
use crate::queuing::QueuingPortPool;
use crate::sampling::SamplingPortPool;
use crate::scheduler::ScheduleTable;
use crate::semaphore::SemaphorePool;

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
}
