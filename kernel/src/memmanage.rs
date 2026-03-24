//! MemManage fault handler macro and pure-logic handler for Cortex-M partitions.

/// Emit a `MemoryManagement` exception handler for the given kernel config.
#[macro_export]
macro_rules! define_memmanage_handler {
    ($Config:ty) => {
        #[cfg(target_arch = "arm")]
        #[cortex_m_rt::exception]
        fn MemoryManagement() {
            // SAFETY: Privileged exception handler context for all unsafe calls below.
            let cfsr = unsafe { $crate::fault::read_cfsr() };
            let mmfar = unsafe { $crate::fault::read_mmfar() };
            let psp = cortex_m::register::psp::read() as u32;
            let exc_return: u32;
            // SAFETY: Reading LR in handler mode yields EXC_RETURN.
            unsafe { core::arch::asm!("mov {}, lr", out(reg) exc_return, options(nomem, nostack)); }

            // Kernel-mode MPU violation is fatal — halt rather than attempting recovery.
            if !$crate::fault::exc_return_uses_psp(exc_return) {
                panic!(
                    "MemManage in kernel mode: CFSR={:#010x} MMFAR={:#010x}",
                    cfsr, mmfar
                );
            }

            let faulting_pc = unsafe {
                // SAFETY: PSP points to a valid hardware exception frame.
                $crate::fault::faulting_pc_from_psp(exc_return, psp)
            }.unwrap_or(0);

            let _ = $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                match $crate::memmanage::handle_memmanage_fault::<$Config>(k, cfsr, mmfar, faulting_pc) {
                    Some(d) => $crate::klog!(
                        "[MemManage] partition={} CFSR={:#010x} MMFAR={:#010x} PC={:#010x}",
                        d.partition_id, d.cfsr, d.mmfar, d.faulting_pc),
                    None => $crate::klog!(
                        "[MemManage] no active partition CFSR={:#010x} MMFAR={:#010x} PC={:#010x}",
                        cfsr, mmfar, faulting_pc),
                }
            });

            // Redirect stacked PC to WFI trampoline to prevent re-execution.
            // SAFETY: PSP exception frame is valid; fault_trampoline is a valid code address.
            unsafe { $crate::fault::write_stacked_pc(psp, $crate::fault::fault_trampoline as u32); }
            // SAFETY: Privileged context; CFSR is write-1-to-clear.
            unsafe { $crate::fault::clear_mmfsr(); }
            // TODO: reviewer false positive — SCB::set_pendsv() is a static method in cortex-m 0.7
            cortex_m::peripheral::SCB::set_pendsv();
        }
    };
}

/// Pure-logic fault handler: partition lookup, transition to Faulted, detail capture.
pub fn handle_memmanage_fault<C: crate::config::KernelConfig>(
    kernel: &mut crate::svc::Kernel<'_, C>,
    cfsr: u32,
    mmfar: u32,
    faulting_pc: u32,
) -> Option<crate::fault::FaultDetails>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    let pid = kernel.active_partition()?;
    let details = crate::fault::FaultDetails::new(pid, cfsr, mmfar, faulting_pc);
    kernel.fault_partition(pid as usize);

    Some(details)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelConfig;
    use crate::fault::{
        FaultDetails, CFSR_DACCVIOL, CFSR_IACCVIOL, CFSR_MMARVALID, CFSR_MMFSR_MASK,
    };
    use crate::kernel_config_types;
    use crate::partition::{ExternalPartitionMemory, MpuRegion, PartitionState};
    use crate::partition_core::AlignedStack256B;
    use crate::scheduler::{ScheduleEntry, ScheduleTable};

    struct TestConfig;
    impl KernelConfig for TestConfig {
        const N: usize = 2;
        const SCHED: usize = 4;
        const S: usize = 2;
        const SW: usize = 2;
        const MS: usize = 2;
        const MW: usize = 2;
        const QS: usize = 2;
        const QD: usize = 4;
        const QM: usize = 16;
        const QW: usize = 2;
        const SP: usize = 2;
        const SM: usize = 16;
        const BS: usize = 2;
        const BM: usize = 16;
        const BW: usize = 2;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 4;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 64;
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = 4;
        kernel_config_types!();
    }

    fn make_test_kernel() -> crate::svc::Kernel<'static, TestConfig> {
        let mut sched = ScheduleTable::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        sched.add(ScheduleEntry::new(1, 10)).unwrap();
        #[cfg(feature = "dynamic-mpu")]
        sched.add_system_window(1).unwrap();
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let m = MpuRegion::new(0, 0, 0);
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(&mut s0, 0x0800_1001, m, 0).unwrap(),
            ExternalPartitionMemory::from_aligned_stack(&mut s1, 0x0800_2001, m, 1).unwrap(),
        ];
        crate::svc::Kernel::<TestConfig>::new(sched, &mems).expect("kernel init")
    }

    #[test]
    fn handle_returns_none_when_no_active_partition() {
        let mut kernel = make_test_kernel();
        let result = handle_memmanage_fault::<TestConfig>(
            &mut kernel,
            CFSR_DACCVIOL | CFSR_MMARVALID,
            0x2000_F000,
            0x0800_1234,
        );
        assert!(result.is_none());
    }

    #[test]
    fn handle_fault_transitions_and_returns_details() {
        let mut kernel = make_test_kernel();
        kernel
            .pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        kernel.active_partition = Some(0);
        let cfsr = CFSR_DACCVIOL | CFSR_MMARVALID;
        let details =
            handle_memmanage_fault::<TestConfig>(&mut kernel, cfsr, 0x2000_F000, 0x0800_1234)
                .expect("should return fault details");
        assert_eq!(
            details,
            FaultDetails::new(0, cfsr, 0x2000_F000, 0x0800_1234)
        );
        assert!(details.is_daccviol() && details.is_mmarvalid());
        assert_eq!(details.mmfsr(), (cfsr & CFSR_MMFSR_MASK) as u8);
        assert_eq!(kernel.pcb(0).unwrap().state(), PartitionState::Faulted);
        assert_eq!(kernel.pcb(1).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn handle_fault_sets_partition_sp_sentinel() {
        let mut kernel = make_test_kernel();
        kernel
            .pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        kernel.active_partition = Some(0);
        // Pre-condition: partition_sp is a valid stack pointer (not sentinel).
        let original_sp = kernel.partition_sp()[0];
        assert_ne!(
            original_sp,
            crate::partition_core::SP_SENTINEL_FAULT,
            "pre-condition: SP must not be sentinel"
        );

        handle_memmanage_fault::<TestConfig>(
            &mut kernel,
            CFSR_DACCVIOL | CFSR_MMARVALID,
            0x2000_F000,
            0x0800_1234,
        )
        .expect("should return fault details");

        // partition_sp must be set to sentinel so PendSV skips context save.
        assert_eq!(
            kernel.partition_sp()[0],
            crate::partition_core::SP_SENTINEL_FAULT,
            "partition_sp must be sentinel after fault"
        );
    }

    #[test]
    fn handle_fault_on_partition_1_sets_sentinel() {
        let mut kernel = make_test_kernel();
        kernel
            .pcb_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        kernel.active_partition = Some(1);

        handle_memmanage_fault::<TestConfig>(&mut kernel, CFSR_IACCVIOL, 0, 0x0800_2000)
            .expect("should return fault details");

        assert_eq!(kernel.pcb(1).unwrap().state(), PartitionState::Faulted);
        assert_eq!(
            kernel.partition_sp()[1],
            crate::partition_core::SP_SENTINEL_FAULT,
            "partition_sp[1] must be sentinel after fault"
        );
        // partition_sp[0] should be unaffected.
        assert_ne!(
            kernel.partition_sp()[0],
            crate::partition_core::SP_SENTINEL_FAULT
        );
    }

    #[test]
    fn handle_fault_on_partition_1() {
        let mut kernel = make_test_kernel();
        kernel
            .pcb_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        kernel.active_partition = Some(1);
        let details =
            handle_memmanage_fault::<TestConfig>(&mut kernel, CFSR_IACCVIOL, 0, 0x0800_2000)
                .expect("should return fault details");
        assert_eq!(details.partition_id, 1);
        assert!(details.is_iaccviol() && !details.is_daccviol());
        assert_eq!(kernel.pcb(1).unwrap().state(), PartitionState::Faulted);
    }

    #[test]
    fn handle_ready_partition_can_fault() {
        let mut kernel = make_test_kernel();
        kernel.active_partition = Some(0);
        let result = handle_memmanage_fault::<TestConfig>(&mut kernel, CFSR_DACCVIOL, 0, 0);
        assert!(result.is_some());
        assert_eq!(kernel.pcb(0).unwrap().state(), PartitionState::Faulted);
    }

    #[test]
    fn handle_double_fault_is_noop_transition() {
        let mut kernel = make_test_kernel();
        kernel
            .pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        kernel.active_partition = Some(0);
        handle_memmanage_fault::<TestConfig>(&mut kernel, CFSR_DACCVIOL, 0, 0);
        assert_eq!(kernel.pcb(0).unwrap().state(), PartitionState::Faulted);
        let details =
            handle_memmanage_fault::<TestConfig>(&mut kernel, CFSR_IACCVIOL, 0x1000, 0x2000)
                .unwrap();
        assert_eq!(details.partition_id, 0);
        assert_eq!(kernel.pcb(0).unwrap().state(), PartitionState::Faulted);
    }
}
