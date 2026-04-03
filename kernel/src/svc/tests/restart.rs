use super::*;
use crate::partition::StartCondition;

#[test]
fn pcb_faulted_to_ready_transition() {
    let mut p = pcb(0);
    p.transition(PartitionState::Running).unwrap();
    p.transition(PartitionState::Faulted).unwrap();
    assert_eq!(p.state(), PartitionState::Faulted);
    p.transition(PartitionState::Ready).unwrap();
    assert_eq!(p.state(), PartitionState::Ready);
}

#[test]
fn set_start_condition_warm_and_cold() {
    let mut p = pcb(0);
    assert_eq!(p.start_condition(), StartCondition::NormalBoot);
    p.set_start_condition(StartCondition::WarmRestart);
    assert_eq!(p.start_condition(), StartCondition::WarmRestart);
    p.set_start_condition(StartCondition::ColdRestart);
    assert_eq!(p.start_condition(), StartCondition::ColdRestart);
}

#[test]
fn increment_fault_count() {
    let mut p = pcb(0);
    assert_eq!(p.fault_count(), 0);
    p.increment_fault_count();
    assert_eq!(p.fault_count(), 1);
    p.increment_fault_count();
    assert_eq!(p.fault_count(), 2);
}

#[cfg(target_pointer_width = "32")]
mod integration {
    use super::*;
    use crate::partition::RestartError;
    use crate::partition_core::{AlignedStack256B, SP_SENTINEL_FAULT};

    struct Cfg;
    impl KernelConfig for Cfg {
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

    fn mk(
        s0: &mut AlignedStack256B,
        s1: &mut AlignedStack256B,
        d0: &mut AlignedStack256B,
        d1: &mut AlignedStack256B,
    ) -> Kernel<'static, Cfg> {
        let mut sched = ScheduleTable::new();
        sched.add(ScheduleEntry::new(0, 10)).unwrap();
        sched.add(ScheduleEntry::new(1, 10)).unwrap();
        sched.add_system_window(1).unwrap();
        let m0 = MpuRegion::new(d0.0.as_ptr() as u32, 256, 0);
        let m1 = MpuRegion::new(d1.0.as_ptr() as u32, 256, 0);
        let mems = [
            ExternalPartitionMemory::from_aligned_stack(s0, 0x0800_1001, m0, pid(0)).unwrap(),
            ExternalPartitionMemory::from_aligned_stack(s1, 0x0800_2001, m1, pid(1)).unwrap(),
        ];
        Kernel::<Cfg>::new(sched, &mems).expect("kernel init")
    }

    fn do_fault(k: &mut Kernel<'_, Cfg>, pid: usize) {
        k.pcb_mut(pid)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        k.fault_partition(pid);
    }

    #[test]
    fn warm_restart_full() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        k.pcb_mut(0).unwrap().set_sleep_until(9999);
        k.pcb_mut(0).unwrap().increment_starvation();
        do_fault(&mut k, 0);
        k.restart_partition(0, true).unwrap();
        assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
        assert_ne!(k.get_sp(0).unwrap(), SP_SENTINEL_FAULT);
        assert_eq!(
            k.pcb(0).unwrap().start_condition(),
            StartCondition::WarmRestart
        );
        assert_eq!(k.pcb(0).unwrap().fault_count(), 1);
        assert_eq!(k.pcb(0).unwrap().sleep_until(), 0);
        assert_eq!(k.pcb(0).unwrap().starvation_count(), 0);
    }

    #[test]
    fn cold_restart_full() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        do_fault(&mut k, 0);
        k.restart_partition(0, false).unwrap();
        assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
        assert_ne!(k.get_sp(0).unwrap(), SP_SENTINEL_FAULT);
        assert_eq!(
            k.pcb(0).unwrap().start_condition(),
            StartCondition::ColdRestart
        );
    }

    #[test]
    fn cold_restart_zeros_data_region() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        // Fill data region with non-zero pattern before kernel init.
        d0.0.fill(0xDEAD_BEEF);
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        do_fault(&mut k, 0);
        k.restart_partition(0, false).unwrap();
        // Verify the data region was zeroed by cold restart.
        assert!(
            d0.0.iter().all(|&w| w == 0),
            "data region must be zeroed after cold restart"
        );
    }

    #[test]
    fn restart_error_cases() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        assert_eq!(k.restart_partition(0, true), Err(RestartError::NotFaulted));
        assert_eq!(k.restart_partition(99, true), Err(RestartError::InvalidPid));
    }
}
