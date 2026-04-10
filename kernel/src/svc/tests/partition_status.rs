use super::*;
use crate::partition_core::AlignedStack1K;
use crate::syscall::SYS_GET_PARTITION_STATUS;
use rtos_traits::partition::{
    PartitionStatus, OPERATING_MODE_RUNNING, START_CONDITION_COLD_RESTART,
    START_CONDITION_NORMAL_BOOT, START_CONDITION_WARM_RESTART,
};

/// Build a 2-partition kernel with schedule P0(5 ticks), P1(3 ticks), SW(1).
fn kernel_2p() -> Kernel<'static, TestConfig> {
    let mut schedule = ScheduleTable::<4>::new();
    schedule.add(ScheduleEntry::new(0, 5)).unwrap();
    schedule.add(ScheduleEntry::new(1, 3)).unwrap();
    schedule.add_system_window(1).unwrap();
    schedule.start();

    let mut stk0 = AlignedStack1K::default();
    let mut stk1 = AlignedStack1K::default();
    let mems = [
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk0,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            pid(0),
        )
        .unwrap(),
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk1,
            0x0800_1001,
            MpuRegion::new(0x2000_1000, 4096, 0),
            pid(1),
        )
        .unwrap(),
    ];
    let mut k = kernel_from_ext(schedule, &mems);
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Running)
        .unwrap();
    k.active_partition = Some(0);
    k.current_partition = 0;
    k
}

/// Allocate a page at a fixed address matching a partition's MPU data region.
fn mmap_partition_buf(partition_idx: usize) -> *mut u8 {
    extern "C" {
        fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
    }
    let addr = 0x2000_0000_usize + partition_idx * 0x1000;
    let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
    assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
    ptr
}

#[test]
fn correct_fields_p0() {
    let mut k = kernel_2p();
    let buf = mmap_partition_buf(0);

    let mut ef = frame(SYS_GET_PARTITION_STATUS, buf as u32, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "syscall must succeed");

    let status = unsafe { core::ptr::read(buf as *const PartitionStatus) };
    assert_eq!(status.partition_id, 0);
    assert_eq!(status.period_ticks, 9, "major_frame = 5+3+1");
    assert_eq!(status.duration_ticks, 5);
    assert_eq!(status.operating_mode, OPERATING_MODE_RUNNING);
    assert_eq!(status.start_condition, START_CONDITION_NORMAL_BOOT);
}

#[test]
fn correct_fields_p1() {
    let mut k = kernel_2p();
    let buf = mmap_partition_buf(1);

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Ready)
        .unwrap();
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .transition(PartitionState::Running)
        .unwrap();
    k.active_partition = Some(1);
    k.current_partition = 1;

    let mut ef = frame(SYS_GET_PARTITION_STATUS, buf as u32, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);

    let status = unsafe { core::ptr::read(buf as *const PartitionStatus) };
    assert_eq!(status.partition_id, 1);
    assert_eq!(status.period_ticks, 9);
    assert_eq!(status.duration_ticks, 3);
    assert_eq!(status.operating_mode, OPERATING_MODE_RUNNING);
    assert_eq!(status.start_condition, START_CONDITION_NORMAL_BOOT);
}

#[test]
fn misaligned_pointer_returns_error() {
    let mut k = kernel_2p();
    let buf = mmap_partition_buf(0);
    let mut ef = frame(SYS_GET_PARTITION_STATUS, buf as u32 + 1, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
}

#[test]
fn null_pointer_returns_error() {
    let mut k = kernel_2p();
    let mut ef = frame(SYS_GET_PARTITION_STATUS, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidPointer.to_u32());
}

#[test]
fn start_condition_mapping() {
    let mut k = kernel_2p();
    let buf = mmap_partition_buf(0);

    // WarmRestart
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_start_condition(crate::partition::StartCondition::WarmRestart);
    let mut ef = frame(SYS_GET_PARTITION_STATUS, buf as u32, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    let status = unsafe { core::ptr::read(buf as *const PartitionStatus) };
    assert_eq!(status.start_condition, START_CONDITION_WARM_RESTART);

    // ColdRestart
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .set_start_condition(crate::partition::StartCondition::ColdRestart);
    let mut ef = frame(SYS_GET_PARTITION_STATUS, buf as u32, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    let status = unsafe { core::ptr::read(buf as *const PartitionStatus) };
    assert_eq!(status.start_condition, START_CONDITION_COLD_RESTART);
}
