//! Kernel boot initialization.

#[cfg(not(test))]
use crate::{
    blackboard::BlackboardPool,
    config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps},
    message::MessagePool,
    mutex::MutexPool,
    partition::PartitionTable,
    queuing::QueuingPortPool,
    sampling::SamplingPortPool,
    scheduler::ScheduleTable,
    semaphore::SemaphorePool,
};

/// Boot initialization errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// Stack frame init failed.
    StackInitFailed { partition_index: usize },
    /// No partition ready at boot.
    NoReadyPartition,
}

impl core::fmt::Display for BootError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::StackInitFailed { partition_index } => {
                write!(f, "stack init failed: partition {partition_index}")
            }
            Self::NoReadyPartition => write!(f, "no partition ready at boot"),
        }
    }
}

/// Uninhabited type for diverging functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Never {}

/// Initialize stacks, priorities, start schedule, enable SysTick, enter idle loop.
#[cfg(not(test))]
pub fn boot<C: KernelConfig>(
    partitions: &[(extern "C" fn() -> !, u32)],
    peripherals: &mut cortex_m::Peripherals,
) -> Result<Never, BootError>
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
    use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, SCB};
    crate::state::with_kernel_mut::<C, _, _>(|k| {
        for (i, &(ep, hint)) in partitions.iter().enumerate() {
            let stk = k
                .core_stack_mut(i)
                .ok_or(BootError::StackInitFailed { partition_index: i })?;
            let base = stk.as_ptr() as u32;
            let ix = crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                .ok_or(BootError::StackInitFailed { partition_index: i })?;
            k.set_sp(i, base + (ix as u32) * 4);
        }
        Ok::<(), BootError>(())
    })?;

    const { crate::config::assert_priority_order::<C>() }
    // SAFETY: Called once before scheduler starts; single-core exclusive access to SCB.
    unsafe {
        peripherals
            .SCB
            .set_priority(SystemHandler::SVCall, C::SVCALL_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::PendSV, C::PENDSV_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::SysTick, C::SYSTICK_PRIORITY);
    }

    let first = crate::state::with_kernel_mut::<C, _, _>(|k| {
        k.start_schedule().inspect(|&pid| k.set_next_partition(pid))
    })
    .ok_or(BootError::NoReadyPartition)?;
    let _ = first;

    peripherals.SYST.set_clock_source(SystClkSource::Core);
    peripherals.SYST.set_reload(C::SYSTICK_CYCLES - 1);
    peripherals.SYST.clear_current();
    peripherals.SYST.enable_counter();
    peripherals.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    #[test]
    fn boot_error_construction_and_display() {
        let err = BootError::StackInitFailed { partition_index: 2 };
        assert_eq!(err, BootError::StackInitFailed { partition_index: 2 });
        assert_ne!(err, BootError::NoReadyPartition);
        assert_eq!(std::format!("{err}"), "stack init failed: partition 2");
        assert_eq!(
            std::format!("{}", BootError::NoReadyPartition),
            "no partition ready at boot"
        );
        let (e2, e3) = (err, err);
        assert_eq!(e2, e3);
    }
}
