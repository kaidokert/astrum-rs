use super::*;
use crate::error_handler::{ErrorStatus, FaultKind};

#[test]
fn initial_last_error_is_none() {
    let k = Kernel::<TestConfig>::default();
    for pid in 0..TestConfig::N {
        assert_eq!(k.get_last_error(pid), None);
    }
}

#[test]
fn set_and_get_last_error() {
    let mut k = Kernel::<TestConfig>::default();
    let es = ErrorStatus::new(FaultKind::BusFault, 1, 0x4000_0000, 0x0200, 0x0800_1234);
    k.set_last_error(1, es);
    assert_eq!(k.get_last_error(1), Some(es));
    // Other partitions remain None.
    assert_eq!(k.get_last_error(0), None);
    assert_eq!(k.get_last_error(2), None);
}

#[test]
fn set_overwrites_previous_error() {
    let mut k = Kernel::<TestConfig>::default();
    let es1 = ErrorStatus::new(FaultKind::MemManage, 0, 0x2000_0000, 0x01, 0x0800_0000);
    let es2 = ErrorStatus::new(FaultKind::UsageFault, 0, 0, 0x0001_0000, 0x0800_2000);
    k.set_last_error(0, es1);
    assert_eq!(k.get_last_error(0), Some(es1));
    k.set_last_error(0, es2);
    assert_eq!(k.get_last_error(0), Some(es2));
}

#[test]
fn clear_last_error() {
    let mut k = Kernel::<TestConfig>::default();
    let es = ErrorStatus::new(FaultKind::DeadlineMiss, 2, 0, 0, 0);
    k.set_last_error(2, es);
    assert_eq!(k.get_last_error(2), Some(es));
    k.clear_last_error(2);
    assert_eq!(k.get_last_error(2), None);
}

#[test]
fn clear_on_none_is_noop() {
    let mut k = Kernel::<TestConfig>::default();
    assert_eq!(k.get_last_error(0), None);
    k.clear_last_error(0);
    assert_eq!(k.get_last_error(0), None);
}

#[test]
fn out_of_range_pid_ignored() {
    let mut k = Kernel::<TestConfig>::default();
    let es = ErrorStatus::new(FaultKind::StackOverflow, 99, 0, 0, 0);
    // These should not panic.
    k.set_last_error(99, es);
    assert_eq!(k.get_last_error(99), None);
    k.clear_last_error(99);
}

#[cfg(target_pointer_width = "32")]
mod restart_clears {
    use super::*;
    use crate::partition::PartitionState;
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
            ExternalPartitionMemory::from_aligned_stack(s0, 0x0800_1001, m0, 0).unwrap(),
            ExternalPartitionMemory::from_aligned_stack(s1, 0x0800_2001, m1, 1).unwrap(),
        ];
        Kernel::<Cfg>::new(sched, &mems).expect("kernel init")
    }

    #[test]
    fn restart_partition_clears_last_error() {
        let (mut s0, mut s1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let (mut d0, mut d1) = (AlignedStack256B::default(), AlignedStack256B::default());
        let mut k = mk(&mut s0, &mut s1, &mut d0, &mut d1);

        let es = ErrorStatus::new(FaultKind::MemManage, 0, 0x2000_0000, 0x01, 0x0800_0000);
        k.set_last_error(0, es);
        assert_eq!(k.get_last_error(0), Some(es));

        // Fault partition 0 and restart it.
        k.pcb_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        k.fault_partition(0);
        k.restart_partition(0, true).unwrap();

        // last_error must be cleared after restart.
        assert_eq!(k.get_last_error(0), None);
        // Partition 1's error (if any) is unaffected.
        let es1 = ErrorStatus::new(FaultKind::BusFault, 1, 0, 0, 0);
        k.set_last_error(1, es1);
        assert_eq!(k.get_last_error(1), Some(es1));
    }
}
