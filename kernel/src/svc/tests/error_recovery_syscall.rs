use super::*;
use crate::syscall::{SYS_REQUEST_RESTART, SYS_REQUEST_STOP};

#[test]
fn request_restart_denied_when_not_in_error_handler() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    // in_error_handler defaults to false
    assert!(!k.partitions().get(0).unwrap().in_error_handler());
    let mut ef = frame(SYS_REQUEST_RESTART, 1, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::PermissionDenied.to_u32());
    // Partition remains Running (unchanged).
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Running
    );
}

#[test]
fn request_stop_denied_when_not_in_error_handler() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    assert!(!k.partitions().get(0).unwrap().in_error_handler());
    let mut ef = frame(SYS_REQUEST_STOP, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::PermissionDenied.to_u32());
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Running
    );
}

#[test]
fn request_stop_succeeds_in_error_handler() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_in_error_handler(true);
    let mut ef = frame(SYS_REQUEST_STOP, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "RequestStop should return 0 on success");
    // Partition transitions to Faulted permanently.
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Faulted
    );
    // in_error_handler is cleared.
    assert!(!k.partitions().get(0).unwrap().in_error_handler());
}

#[test]
fn request_stop_clears_in_error_handler() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_in_error_handler(true);
    assert!(k.partitions().get(0).unwrap().in_error_handler());
    let mut ef = frame(SYS_REQUEST_STOP, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    assert!(
        !k.partitions().get(0).unwrap().in_error_handler(),
        "in_error_handler must be cleared after RequestStop"
    );
}

#[test]
fn request_stop_does_not_affect_other_partitions() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_in_error_handler(true);
    let mut ef = frame(SYS_REQUEST_STOP, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Partition 1 should remain Running.
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Running
    );
}

/// RequestRestart requires a real stack+MPU setup because restart_partition
/// reinitializes the stack frame. Use the integration-style kernel builder.
#[cfg(target_pointer_width = "32")]
mod restart_integration {
    use super::*;
    use crate::partition::StartCondition;
    use crate::partition_core::AlignedStack256B;

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

    #[test]
    fn request_restart_warm_succeeds() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        // Put partition 0 in Running + in_error_handler state.
        k.current_partition = 0;
        k.pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        k.pcb_mut(0).unwrap().set_in_error_handler(true);

        let mut ef = frame(SYS_REQUEST_RESTART, 1, 0); // r1=1 => warm
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "RequestRestart(warm) should return 0");
        // After restart, partition is Ready.
        assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
        assert_eq!(
            k.pcb(0).unwrap().start_condition(),
            StartCondition::WarmRestart
        );
        assert!(!k.pcb(0).unwrap().in_error_handler());
        assert_eq!(k.pcb(0).unwrap().fault_count(), 1);
    }

    #[test]
    fn request_restart_cold_succeeds() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        d0.0.fill(0xDEAD_BEEF);
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        k.current_partition = 0;
        k.pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        k.pcb_mut(0).unwrap().set_in_error_handler(true);

        let mut ef = frame(SYS_REQUEST_RESTART, 0, 0); // r1=0 => cold
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0, "RequestRestart(cold) should return 0");
        assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Ready);
        assert_eq!(
            k.pcb(0).unwrap().start_condition(),
            StartCondition::ColdRestart
        );
        assert!(!k.pcb(0).unwrap().in_error_handler());
        // Cold restart zeros the data region.
        assert!(
            d0.0.iter().all(|&w| w == 0),
            "data region must be zeroed after cold restart"
        );
    }

    #[test]
    fn request_restart_denied_when_not_in_error_handler() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        k.current_partition = 0;
        k.pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        // in_error_handler is false by default.
        let mut ef = frame(SYS_REQUEST_RESTART, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, SvcError::PermissionDenied.to_u32());
        // Partition remains Running.
        assert_eq!(k.pcb(0).unwrap().state(), PartitionState::Running);
    }

    #[test]
    fn request_restart_clears_last_error() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);
        k.current_partition = 0;
        k.pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        k.pcb_mut(0).unwrap().set_in_error_handler(true);
        // Set a last_error so we can verify it gets cleared.
        use crate::error_handler::{ErrorStatus, FaultKind};
        let es = ErrorStatus::new(FaultKind::MemManage, 0, 0x2000_0000, 0x0001, 0x0800_0000);
        k.set_last_error(0, es);
        assert!(k.get_last_error(0).is_some());

        let mut ef = frame(SYS_REQUEST_RESTART, 1, 0);
        unsafe { k.dispatch(&mut ef) };
        assert_eq!(ef.r0, 0);
        // restart_partition clears last_error.
        assert!(k.get_last_error(0).is_none());
    }
}
