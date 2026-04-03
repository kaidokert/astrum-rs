//! MemManage fault handler macro and pure-logic handler for Cortex-M partitions.

/// Emit a `MemoryManagement` exception handler for the given kernel config.
#[macro_export]
macro_rules! define_memmanage_handler {
    ($Config:ty) => {
        #[cfg(target_arch = "arm")]
        #[cortex_m_rt::exception]
        fn MemoryManagement() {
            // Capture EXC_RETURN first: the bl instructions in read_cfsr/read_mmfar/psp::read clobber LR.
            let exc_return: u32;
            // SAFETY: Reading LR in handler mode yields EXC_RETURN.
            unsafe { core::arch::asm!("mov {}, lr", out(reg) exc_return, options(nomem, nostack)); }

            // SAFETY: Privileged exception handler context for all unsafe calls below.
            let cfsr = unsafe { $crate::fault::read_cfsr() };
            let mmfar = unsafe { $crate::fault::read_mmfar() };
            let psp = cortex_m::register::psp::read() as u32;

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

            let restart_sp = $crate::state::with_kernel_mut::<$Config, _, Option<u32>>(|k| {
                match $crate::memmanage::handle_memmanage_fault::<$Config>(k, cfsr, mmfar, faulting_pc) {
                    Some(d) => {
                        $crate::klog!(
                            "[MemManage] partition={} CFSR={:#010x} MMFAR={:#010x} PC={:#010x}",
                            d.partition_id, d.cfsr, d.mmfar, d.faulting_pc);
                        // If the partition was restarted (Ready) or its error
                        // handler was activated (in_error_handler), return the
                        // fresh SP so we can update PSP accordingly.
                        let pid = d.partition_id.as_raw() as usize;
                        if k.pcb(pid).map_or(false, |pcb|
                            pcb.state() == $crate::partition::PartitionState::Ready
                            || pcb.in_error_handler())
                        {
                            return Some(k.partition_sp()[pid]);
                        }
                    },
                    None => $crate::klog!(
                        "[MemManage] no active partition CFSR={:#010x} MMFAR={:#010x} PC={:#010x}",
                        cfsr, mmfar, faulting_pc),
                }
                None
            });

            if matches!(restart_sp, Ok(Some(_))) {
                // Partition was restarted: point PSP past the software context
                // (r4-r11 = 8 words = 32 bytes) to the exception frame so the
                // CPU unstacks the fresh entry point on return.  PendSV will
                // later save r4-r11 into the software context area and
                // partition_sp[pid] will remain correct.
                let sp = restart_sp.unwrap().unwrap();
                // SAFETY: sp is a valid stack address from restart_partition.
                unsafe { cortex_m::register::psp::write(sp + 32); }
            } else {
                // Partition stays faulted: redirect to WFI trampoline.
                // SAFETY: PSP exception frame is valid; fault_trampoline is a valid code address.
                unsafe { $crate::fault::write_stacked_pc(psp, $crate::fault::fault_trampoline as u32); }
            }
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
    [(); C::BP]:,
    [(); C::BZ]:,
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
    let details = crate::fault::FaultDetails::new(
        rtos_traits::ids::PartitionId::new(pid as u32),
        cfsr,
        mmfar,
        faulting_pc,
    );

    #[cfg(feature = "trace")]
    crate::trace::emit_fault_trace(&details);

    // If the partition has a registered error handler and is NOT already
    // inside the error handler (double fault), activate the handler.
    // A double fault (fault while in_error_handler) bypasses the handler
    // to prevent infinite loops — the default fault policy applies instead.
    if kernel
        .pcb(pid as usize)
        .is_some_and(|pcb| pcb.error_handler().is_some() && !pcb.in_error_handler())
    {
        kernel.activate_error_handler(pid as usize, &details);
        return Some(details);
    }

    // Clear in_error_handler so the PCB is clean before applying fault policy.
    if let Some(pcb) = kernel.pcb_mut(pid as usize) {
        pcb.set_in_error_handler(false);
    }

    // No error handler — apply existing fault policy.
    kernel.fault_partition(pid as usize);

    // Check fault policy to decide whether to auto-restart.
    // TODO: fault_count is only incremented on successful restart, so a StayDead partition
    // that faulted once reads as fault_count=0. Consider incrementing the count on fault
    // entry (before policy check) to distinguish "never faulted" from "faulted and terminated".
    let (policy, count) = match kernel.pcb(pid as usize) {
        Some(pcb) => (pcb.fault_policy(), pcb.fault_count()),
        None => return Some(details),
    };

    match policy {
        crate::partition::FaultPolicy::StayDead => {
            crate::klog!("[MemManage] pid={} policy=StayDead, staying faulted", pid);
        }
        crate::partition::FaultPolicy::WarmRestart { max }
        | crate::partition::FaultPolicy::ColdRestart { max } => {
            let warm = matches!(policy, crate::partition::FaultPolicy::WarmRestart { .. });
            let _label = if warm { "warm" } else { "cold" };
            if count < max {
                match kernel.restart_partition(pid as usize, warm) {
                    Ok(()) => {
                        // Clear active_partition: the partition is now Ready,
                        // not Running, so the invariant checker must not see
                        // it as the active Running partition.
                        kernel.set_core_active_partition(None);
                        crate::klog!(
                            "[MemManage] pid={} {} restart ({}/{})",
                            pid,
                            _label,
                            count + 1,
                            max
                        );
                    }
                    Err(_e) => {
                        crate::klog!(
                            "[MemManage] pid={} {} restart FAILED: {:?}",
                            pid,
                            _label,
                            _e
                        );
                    }
                }
            } else {
                crate::klog!(
                    "[MemManage] pid={} {} max restarts reached (count={}/{}), staying faulted",
                    pid,
                    _label,
                    count,
                    max
                );
            }
        }
    }

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
    use rtos_traits::ids::PartitionId;

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
        const BP: usize = 4;
        const BZ: usize = 32;
        const DR: usize = 4;
        kernel_config_types!();
    }

    fn make_test_kernel() -> crate::svc::Kernel<'static, TestConfig> {
        let mut sched = ScheduleTable::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        sched.add(ScheduleEntry::new(1, 10)).unwrap();
        sched.add_system_window(1).unwrap();
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let m = MpuRegion::new(0, 0, 0);
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(
                &mut s0,
                0x0800_1001,
                m,
                PartitionId::new(0),
            )
            .unwrap(),
            ExternalPartitionMemory::from_aligned_stack(
                &mut s1,
                0x0800_2001,
                m,
                PartitionId::new(1),
            )
            .unwrap(),
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
            FaultDetails::new(PartitionId::new(0), cfsr, 0x2000_F000, 0x0800_1234)
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
        assert_eq!(details.partition_id, PartitionId::new(1));
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
        assert_eq!(details.partition_id, PartitionId::new(0));
        assert_eq!(kernel.pcb(0).unwrap().state(), PartitionState::Faulted);
    }

    #[cfg(target_pointer_width = "32")]
    mod fault_policy_tests {
        use super::*;
        use crate::partition::FaultPolicy;

        fn make_policy_kernel(
            s0: &mut AlignedStack256B,
            d0: &mut AlignedStack256B,
            s1: &mut AlignedStack256B,
            policy: FaultPolicy,
        ) -> crate::svc::Kernel<'static, TestConfig> {
            let mut sched = ScheduleTable::new();
            sched.add(ScheduleEntry::new(0, 10)).unwrap();
            sched.add(ScheduleEntry::new(1, 10)).unwrap();
            sched.add_system_window(1).unwrap();
            let m0 = MpuRegion::new(d0.0.as_ptr() as u32, 256, 0);
            let m1 = MpuRegion::new(0, 0, 0);
            let mems = [
                ExternalPartitionMemory::from_aligned_stack(
                    s0,
                    0x0800_1001,
                    m0,
                    PartitionId::new(0),
                )
                .unwrap()
                .with_fault_policy(policy),
                ExternalPartitionMemory::from_aligned_stack(
                    s1,
                    0x0800_2001,
                    m1,
                    PartitionId::new(1),
                )
                .unwrap(),
            ];
            crate::svc::Kernel::<TestConfig>::new(sched, &mems).expect("kernel init")
        }

        fn do_fault(k: &mut crate::svc::Kernel<'_, TestConfig>) {
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(k, CFSR_DACCVIOL, 0x2000_F000, 0x0800_1234);
        }

        #[test]
        fn stay_dead_stays_faulted() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_policy_kernel(&mut s0, &mut d0, &mut s1, FaultPolicy::StayDead);
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 0);
        }

        #[test]
        fn warm_restart_recovers_until_max() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_policy_kernel(
                &mut s0,
                &mut d0,
                &mut s1,
                FaultPolicy::WarmRestart { max: 2 },
            );

            // First fault: should auto-restart (fault_count becomes 1).
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);

            // Second fault: should auto-restart (fault_count becomes 2).
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 2);

            // Third fault: max reached, stays faulted.
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
        }

        #[test]
        fn cold_restart_recovers_until_max() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_policy_kernel(
                &mut s0,
                &mut d0,
                &mut s1,
                FaultPolicy::ColdRestart { max: 1 },
            );

            // First fault: should auto-restart (fault_count becomes 1).
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);

            // Second fault: max reached, stays faulted.
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
        }

        #[test]
        fn warm_restart_after_max_stays_faulted() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_policy_kernel(
                &mut s0,
                &mut d0,
                &mut s1,
                FaultPolicy::WarmRestart { max: 1 },
            );

            // First fault: auto-restart.
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);

            // Second fault: max reached, stays faulted permanently.
            do_fault(&mut k);
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);
        }
    }

    #[cfg(target_pointer_width = "32")]
    mod error_handler_tests {
        use super::*;
        use crate::error_handler::{ErrorStatus, FaultKind};
        use crate::partition::FaultPolicy;
        use crate::partition_core::SP_SENTINEL_FAULT;

        const HANDLER_ADDR: u32 = 0x0800_3001;

        fn make_handler_kernel(
            s0: &mut AlignedStack256B,
            d0: &mut AlignedStack256B,
            s1: &mut AlignedStack256B,
            error_handler: Option<u32>,
        ) -> crate::svc::Kernel<'static, TestConfig> {
            let mut sched = ScheduleTable::new();
            sched.add(ScheduleEntry::new(0, 10)).unwrap();
            sched.add(ScheduleEntry::new(1, 10)).unwrap();
            sched.add_system_window(1).unwrap();
            let m0 = MpuRegion::new(d0.0.as_ptr() as u32, 256, 0);
            let m1 = MpuRegion::new(0, 0, 0);
            let mut epm0 = ExternalPartitionMemory::from_aligned_stack(
                s0,
                0x0800_1001,
                m0,
                PartitionId::new(0),
            )
            .unwrap()
            .with_fault_policy(FaultPolicy::StayDead);
            if let Some(addr) = error_handler {
                epm0 = epm0.with_error_handler(addr);
            }
            let mems = [
                epm0,
                ExternalPartitionMemory::from_aligned_stack(
                    s1,
                    0x0800_2001,
                    m1,
                    PartitionId::new(1),
                )
                .unwrap(),
            ];
            crate::svc::Kernel::<TestConfig>::new(sched, &mems).expect("kernel init")
        }

        #[test]
        fn error_handler_activates_on_fault() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_kernel(&mut s0, &mut d0, &mut s1, Some(HANDLER_ADDR));
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);

            let cfsr = CFSR_DACCVIOL | CFSR_MMARVALID;
            let details =
                handle_memmanage_fault::<TestConfig>(&mut k, cfsr, 0x2000_F000, 0x0800_1234)
                    .expect("should return fault details");

            // Partition must NOT transition to Faulted.
            assert_ne!(k.pcb(0).unwrap().state(), PartitionState::Faulted);

            // in_error_handler flag must be set.
            assert!(k.pcb(0).unwrap().in_error_handler());

            // ErrorStatus must be stored via set_last_error.
            let err = k.get_last_error(0).expect("error status must be stored");
            assert_eq!(err.kind(), FaultKind::MemManage);
            assert_eq!(err.failed_partition(), 0);
            assert_eq!(err.faulting_addr(), 0x2000_F000);
            assert_eq!(err.cfsr(), cfsr);
            assert_eq!(err.faulting_pc(), 0x0800_1234);

            // Stack must be reinitialized (SP is not sentinel).
            let sp = k.partition_sp()[0];
            assert_ne!(sp, SP_SENTINEL_FAULT, "SP must not be sentinel");
            assert_ne!(sp, 0, "SP must be valid");

            // Fault details are still returned correctly.
            assert_eq!(details.partition_id, PartitionId::new(0));
            assert_eq!(details.cfsr, cfsr);
        }

        #[test]
        fn no_error_handler_uses_default_policy() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_kernel(&mut s0, &mut d0, &mut s1, None);
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);

            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_F000,
                0x0800_1234,
            )
            .expect("should return fault details");

            // Without error handler, partition transitions to Faulted (StayDead policy).
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
            assert!(!k.pcb(0).unwrap().in_error_handler());
            assert!(k.get_last_error(0).is_none());
            assert_eq!(k.partition_sp()[0], SP_SENTINEL_FAULT);
        }

        /// Helper: create a handler kernel with a configurable fault policy.
        fn make_handler_policy_kernel(
            s0: &mut AlignedStack256B,
            d0: &mut AlignedStack256B,
            s1: &mut AlignedStack256B,
            error_handler: Option<u32>,
            policy: FaultPolicy,
        ) -> crate::svc::Kernel<'static, TestConfig> {
            let mut sched = ScheduleTable::new();
            sched.add(ScheduleEntry::new(0, 10)).unwrap();
            sched.add(ScheduleEntry::new(1, 10)).unwrap();
            sched.add_system_window(1).unwrap();
            let m0 = MpuRegion::new(d0.0.as_ptr() as u32, 256, 0);
            let m1 = MpuRegion::new(0, 0, 0);
            let mut epm0 = ExternalPartitionMemory::from_aligned_stack(
                s0,
                0x0800_1001,
                m0,
                PartitionId::new(0),
            )
            .unwrap()
            .with_fault_policy(policy);
            if let Some(addr) = error_handler {
                epm0 = epm0.with_error_handler(addr);
            }
            let mems = [
                epm0,
                ExternalPartitionMemory::from_aligned_stack(
                    s1,
                    0x0800_2001,
                    m1,
                    PartitionId::new(1),
                )
                .unwrap(),
            ];
            crate::svc::Kernel::<TestConfig>::new(sched, &mems).expect("kernel init")
        }

        #[test]
        fn double_fault_bypasses_handler_stay_dead() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_kernel(&mut s0, &mut d0, &mut s1, Some(HANDLER_ADDR));

            // First fault: error handler activates.
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_F000,
                0x0800_1234,
            );
            assert!(k.pcb(0).unwrap().in_error_handler());
            assert_ne!(k.pcb(0).unwrap().state(), PartitionState::Faulted);

            // Second fault (double fault): handler must NOT re-activate.
            // Partition has StayDead policy, so it should stay faulted.
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_A000,
                0x0800_3456,
            );

            // in_error_handler must be cleared.
            assert!(!k.pcb(0).unwrap().in_error_handler());
            // Partition must be Faulted (default policy applied).
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
            // SP must be sentinel.
            assert_eq!(k.partition_sp()[0], SP_SENTINEL_FAULT);
        }

        #[test]
        fn double_fault_with_warm_restart_policy() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_policy_kernel(
                &mut s0,
                &mut d0,
                &mut s1,
                Some(HANDLER_ADDR),
                FaultPolicy::WarmRestart { max: 2 },
            );

            // First fault: error handler activates.
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_F000,
                0x0800_1234,
            );
            assert!(k.pcb(0).unwrap().in_error_handler());
            assert_ne!(k.pcb(0).unwrap().state(), PartitionState::Faulted);

            // Double fault while in error handler: warm restart policy applies.
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_A000,
                0x0800_5678,
            );

            // in_error_handler must be cleared.
            assert!(!k.pcb(0).unwrap().in_error_handler());
            // WarmRestart policy should auto-restart (fault_count=1, max=2).
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);
        }

        #[test]
        fn double_fault_with_cold_restart_exhausted() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_policy_kernel(
                &mut s0,
                &mut d0,
                &mut s1,
                Some(HANDLER_ADDR),
                FaultPolicy::ColdRestart { max: 1 },
            );

            // First fault: error handler activates.
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_F000,
                0x0800_1234,
            );
            assert!(k.pcb(0).unwrap().in_error_handler());

            // Double fault: cold restart (fault_count becomes 1).
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(&mut k, CFSR_DACCVIOL, 0x2000_B000, 0x0800_9ABC);
            assert!(!k.pcb(0).unwrap().in_error_handler());
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);

            // Next normal fault: error handler activates again (in_error_handler was cleared).
            k.pcb_mut(0)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(
                &mut k,
                CFSR_DACCVIOL | CFSR_MMARVALID,
                0x2000_C000,
                0x0800_DEF0,
            );
            assert!(k.pcb(0).unwrap().in_error_handler());
            assert_ne!(k.pcb(0).unwrap().state(), PartitionState::Faulted);

            // Another double fault: max restarts exhausted, stays faulted.
            k.active_partition = Some(0);
            handle_memmanage_fault::<TestConfig>(&mut k, CFSR_DACCVIOL, 0x2000_D000, 0x0800_1111);
            assert!(!k.pcb(0).unwrap().in_error_handler());
            assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Faulted);
            assert_eq!(k.pcb(0).unwrap().fault_count(), 1);
        }

        #[test]
        fn activate_error_handler_reinits_stack_with_handler_pc() {
            let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
            let mut d0 = AlignedStack256B::default();
            let mut k = make_handler_kernel(&mut s0, &mut d0, &mut s1, Some(HANDLER_ADDR));

            let details =
                crate::fault::FaultDetails::new(0, CFSR_DACCVIOL, 0x2000_0000, 0x0800_1000);
            let sp = k
                .activate_error_handler(0, &details)
                .expect("should succeed");

            // SP must point within the stack region.
            let base = k.pcb(0).unwrap().stack_base();
            let size = k.pcb(0).unwrap().stack_size();
            assert!(sp >= base && sp < base + size, "SP must be within stack");

            // Verify the exception frame has the handler address as PC.
            // init_stack_frame layout: [r4..r11 | r0, r1, r2, r3, r12, lr, PC, xpsr]
            // PC is at offset 14 words from base index (8 saved + 6 into hw frame).
            let stack_slice =
                unsafe { core::slice::from_raw_parts(base as *const u32, (size / 4) as usize) };
            let sp_idx = ((sp - base) / 4) as usize;
            // PC is at sp_idx + 14 (8 software context + 6 into exception frame).
            let pc = stack_slice[sp_idx + 14];
            assert_eq!(pc, HANDLER_ADDR, "stacked PC must be error handler address");
        }
    }
}
