//! Tests verifying that ExternalPartitionMemory fields round-trip correctly
//! through Kernel::new into the resulting PartitionControlBlocks.

use super::*;
use crate::partition::FaultPolicy;

/// Minimum viable stack: 32 bytes = 8 words, aligned to 32.
#[repr(C, align(32))]
struct Align32([u32; 8]);

#[repr(C, align(256))]
struct Align256([u32; 64]);

#[repr(C, align(4096))]
struct Align4K([u32; 1024]);

/// Helper: build a schedule with one partition slot + system window.
fn schedule_1p() -> ScheduleTable<4> {
    let mut s = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 5)).unwrap();
    s.add_system_window(1).unwrap();
    s.start();
    s
}

/// Helper: build a schedule with two partition slots + system window.
fn schedule_2p() -> ScheduleTable<4> {
    let mut s = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 5)).unwrap();
    s.add(ScheduleEntry::new(1, 5)).unwrap();
    s.add_system_window(1).unwrap();
    s.start();
    s
}

/// Helper: build a schedule with three partition slots + system window.
fn schedule_3p() -> ScheduleTable<4> {
    let mut s = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 5)).unwrap();
    s.add(ScheduleEntry::new(1, 5)).unwrap();
    s.add(ScheduleEntry::new(2, 5)).unwrap();
    s.add_system_window(1).unwrap();
    s.start();
    s
}

// ------------------------------------------------------------------
// Test 1: Basic single partition — all fields round-trip
// ------------------------------------------------------------------
#[test]
fn single_partition_fields_roundtrip() {
    let mut stk = Align4K([0u32; 1024]);
    let expected_base = stk.0.as_ptr() as u32;
    let mpu = MpuRegion::new(0x2000_0000, 4096, 0x03);
    let m0 = ExternalPartitionMemory::new(&mut stk.0, 0x0800_0001, mpu, 0).unwrap();

    let k = Kernel::<TestConfig>::new(schedule_1p(), &[m0]).unwrap();
    let pcb = k.partitions().get(0).unwrap();

    assert_eq!(pcb.entry_point(), 0x0800_0001u32);
    assert_eq!(pcb.stack_base(), expected_base);
    assert_eq!(pcb.stack_size(), 4096);
    assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
    assert_eq!(pcb.mpu_region().size(), 4096);
    assert_eq!(pcb.mpu_region().permissions(), 0x03);
    assert!(pcb.peripheral_regions().is_empty());
    assert_eq!(pcb.fault_policy(), FaultPolicy::StayDead);
    assert_eq!(pcb.error_handler(), None);
    assert_eq!(pcb.r0_hint(), 0);
}

// ------------------------------------------------------------------
// Test 2: Minimum stack size (32 bytes) with all optional fields set
// ------------------------------------------------------------------
#[test]
fn minimum_stack_with_all_optional_fields() {
    let mut stk = Align32([0u32; 8]);
    let expected_base = stk.0.as_ptr() as u32;
    let mpu = MpuRegion::new(0x2000_0000, 4096, 0x01);
    let periph = MpuRegion::new(0x4000_C000, 4096, 0x03);

    let m0 = ExternalPartitionMemory::new(&mut stk.0, 0x0800_0001, mpu, 0)
        .unwrap()
        .with_peripheral_regions(&[periph])
        .unwrap()
        .with_fault_policy(FaultPolicy::WarmRestart { max: 3 })
        .with_error_handler(0x0800_2001)
        .with_r0_hint(42);

    let k = Kernel::<TestConfig>::new(schedule_1p(), &[m0]).unwrap();
    let pcb = k.partitions().get(0).unwrap();

    // Stack geometry
    assert_eq!(pcb.stack_base(), expected_base);
    assert_eq!(pcb.stack_size(), 32);

    // Entry point
    assert_eq!(pcb.entry_point(), 0x0800_0001u32);

    // MPU data region
    assert_eq!(pcb.mpu_region().base(), 0x2000_0000);
    assert_eq!(pcb.mpu_region().size(), 4096);
    assert_eq!(pcb.mpu_region().permissions(), 0x01);

    // Peripheral regions
    assert_eq!(pcb.peripheral_regions().len(), 1);
    assert_eq!(pcb.peripheral_regions()[0].base(), 0x4000_C000);
    assert_eq!(pcb.peripheral_regions()[0].size(), 4096);
    assert_eq!(pcb.peripheral_regions()[0].permissions(), 0x03);

    // Fault policy
    assert_eq!(pcb.fault_policy(), FaultPolicy::WarmRestart { max: 3 });

    // Error handler
    assert_eq!(pcb.error_handler(), Some(0x0800_2001));

    // R0 hint
    assert_eq!(pcb.r0_hint(), 42);
}

// ------------------------------------------------------------------
// Test 3: Multiple partitions with distinct configurations
// ------------------------------------------------------------------
#[test]
fn multiple_partitions_distinct_configs() {
    let mut stk0 = Align256([0u32; 64]);
    let mut stk1 = Align256([0u32; 64]);
    let base0 = stk0.0.as_ptr() as u32;
    let base1 = stk1.0.as_ptr() as u32;

    let mpu0 = MpuRegion::new(0x2000_0000, 4096, 0x00);
    let mpu1 = MpuRegion::new(0x2000_1000, 4096, 0x03);

    let m0 = ExternalPartitionMemory::new(&mut stk0.0, 0x0800_0001, mpu0, 0)
        .unwrap()
        .with_fault_policy(FaultPolicy::ColdRestart { max: 5 })
        .with_r0_hint(100);

    let m1 = ExternalPartitionMemory::new(&mut stk1.0, 0x0800_1001, mpu1, 1)
        .unwrap()
        .with_fault_policy(FaultPolicy::StayDead)
        .with_error_handler(0x0800_3001)
        .with_r0_hint(200);

    let k = Kernel::<TestConfig>::new(schedule_2p(), &[m0, m1]).unwrap();

    // Partition 0
    let p0 = k.partitions().get(0).unwrap();
    assert_eq!(p0.entry_point(), 0x0800_0001u32);
    assert_eq!(p0.stack_base(), base0);
    assert_eq!(p0.stack_size(), 256);
    assert_eq!(p0.mpu_region().base(), 0x2000_0000);
    assert_eq!(p0.mpu_region().size(), 4096);
    assert_eq!(p0.mpu_region().permissions(), 0x00);
    assert_eq!(p0.fault_policy(), FaultPolicy::ColdRestart { max: 5 });
    assert_eq!(p0.error_handler(), None);
    assert_eq!(p0.r0_hint(), 100);

    // Partition 1
    let p1 = k.partitions().get(1).unwrap();
    assert_eq!(p1.entry_point(), 0x0800_1001u32);
    assert_eq!(p1.stack_base(), base1);
    assert_eq!(p1.stack_size(), 256);
    assert_eq!(p1.mpu_region().base(), 0x2000_1000);
    assert_eq!(p1.mpu_region().size(), 4096);
    assert_eq!(p1.mpu_region().permissions(), 0x03);
    assert_eq!(p1.fault_policy(), FaultPolicy::StayDead);
    assert_eq!(p1.error_handler(), Some(0x0800_3001));
    assert_eq!(p1.r0_hint(), 200);
}

// ------------------------------------------------------------------
// Test 4: Three partitions with peripheral regions
// ------------------------------------------------------------------
#[test]
fn three_partitions_with_peripheral_regions() {
    let mut stk0 = AlignedStack1K::default();
    let mut stk1 = AlignedStack1K::default();
    let mut stk2 = AlignedStack1K::default();
    let base0 = stk0.0.as_ptr() as u32;
    let base1 = stk1.0.as_ptr() as u32;
    let base2 = stk2.0.as_ptr() as u32;

    let mpu0 = MpuRegion::new(0x2000_0000, 4096, 0x01);
    let mpu1 = MpuRegion::new(0x2000_1000, 4096, 0x01);
    let mpu2 = MpuRegion::new(0x2000_2000, 4096, 0x01);

    // P0: two peripheral regions
    let periph_uart = MpuRegion::new(0x4000_C000, 4096, 0x03);
    let periph_i2c = MpuRegion::new(0x4002_0000, 4096, 0x03);
    let m0 = ExternalPartitionMemory::new(&mut stk0.0, 0x0800_0001, mpu0, 0)
        .unwrap()
        .with_peripheral_regions(&[periph_uart, periph_i2c])
        .unwrap();

    // P1: three peripheral regions (max)
    let periph_spi = MpuRegion::new(0x4000_8000, 4096, 0x03);
    let periph_adc = MpuRegion::new(0x4003_8000, 4096, 0x03);
    let periph_tmr = MpuRegion::new(0x4003_0000, 4096, 0x03);
    let m1 = ExternalPartitionMemory::new(&mut stk1.0, 0x0800_1001, mpu1, 1)
        .unwrap()
        .with_peripheral_regions(&[periph_spi, periph_adc, periph_tmr])
        .unwrap()
        .with_fault_policy(FaultPolicy::WarmRestart { max: 1 });

    // P2: no peripheral regions, default everything
    let m2 = ExternalPartitionMemory::new(&mut stk2.0, 0x0800_2001, mpu2, 2).unwrap();

    let k = Kernel::<TestConfig>::new(schedule_3p(), &[m0, m1, m2]).unwrap();

    // P0: verify peripheral regions
    let p0 = k.partitions().get(0).unwrap();
    assert_eq!(p0.stack_base(), base0);
    assert_eq!(p0.stack_size(), 1024);
    assert_eq!(p0.peripheral_regions().len(), 2);
    assert_eq!(p0.peripheral_regions()[0].base(), 0x4000_C000);
    assert_eq!(p0.peripheral_regions()[0].size(), 4096);
    assert_eq!(p0.peripheral_regions()[1].base(), 0x4002_0000);
    assert_eq!(p0.peripheral_regions()[1].size(), 4096);
    assert_eq!(p0.fault_policy(), FaultPolicy::StayDead);
    assert_eq!(p0.error_handler(), None);

    // P1: verify max peripheral regions + fault policy
    let p1 = k.partitions().get(1).unwrap();
    assert_eq!(p1.stack_base(), base1);
    assert_eq!(p1.stack_size(), 1024);
    assert_eq!(p1.peripheral_regions().len(), 3);
    assert_eq!(p1.peripheral_regions()[0].base(), 0x4000_8000);
    assert_eq!(p1.peripheral_regions()[1].base(), 0x4003_8000);
    assert_eq!(p1.peripheral_regions()[2].base(), 0x4003_0000);
    assert_eq!(p1.fault_policy(), FaultPolicy::WarmRestart { max: 1 });

    // P2: verify defaults
    let p2 = k.partitions().get(2).unwrap();
    assert_eq!(p2.stack_base(), base2);
    assert_eq!(p2.stack_size(), 1024);
    assert!(p2.peripheral_regions().is_empty());
    assert_eq!(p2.fault_policy(), FaultPolicy::StayDead);
    assert_eq!(p2.error_handler(), None);
    assert_eq!(p2.r0_hint(), 0);
    assert_eq!(p2.entry_point(), 0x0800_2001u32);
    assert_eq!(p2.mpu_region().base(), 0x2000_2000);
}

// ------------------------------------------------------------------
// Test 5: Cold restart policy with max=0 edge case
// ------------------------------------------------------------------
#[test]
fn fault_policy_cold_restart_max_zero() {
    let mut stk = AlignedStack1K::default();
    let mpu = MpuRegion::new(0x2000_0000, 4096, 0x01);

    let m0 = ExternalPartitionMemory::new(&mut stk.0, 0x0800_0001, mpu, 0)
        .unwrap()
        .with_fault_policy(FaultPolicy::ColdRestart { max: 0 });

    let k = Kernel::<TestConfig>::new(schedule_1p(), &[m0]).unwrap();
    let pcb = k.partitions().get(0).unwrap();
    assert_eq!(pcb.fault_policy(), FaultPolicy::ColdRestart { max: 0 });
}
