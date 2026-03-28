use super::*;

/// Build a `Kernel` from `ExternalPartitionMemory` slices via `new`.
/// No system-window is added — callers must include one for `dynamic-mpu` builds.
pub fn kernel_from_ext(
    schedule: ScheduleTable<4>,
    mems: &[ExternalPartitionMemory<'_>],
) -> Kernel<'static, TestConfig> {
    // TODO(panic-free): replace .unwrap() with Result return type for consistency
    Kernel::<TestConfig>::new(schedule, mems).unwrap()
}

/// Test kernel configuration with small, fixed pool sizes.
pub struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 4;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 4;
    const MW: usize = 4;
    const QS: usize = 4;
    const QD: usize = 4;
    const QM: usize = 4;
    const QW: usize = 4;
    const SP: usize = 4;
    const BS: usize = 4;
    const BW: usize = 4;
    const BP: usize = 4;

    kernel_config_types!(AlignedStack4K);
}

pub fn frame(r0: u32, r1: u32, r2: u32) -> ExceptionFrame {
    ExceptionFrame {
        r0,
        r1,
        r2,
        r3: LEGACY_TEST_FRAME_R3_SENTINEL,
        r12: 0,
        lr: 0,
        pc: 0,
        xpsr: 0,
    }
}

pub fn frame4(r0: u32, r1: u32, r2: u32, r3: u32) -> ExceptionFrame {
    ExceptionFrame {
        r0,
        r1,
        r2,
        r3,
        r12: 0,
        lr: 0,
        pc: 0,
        xpsr: 0,
    }
}

pub fn pcb(id: u8) -> PartitionControlBlock {
    let o = (id as u32) * 0x1000;
    PartitionControlBlock::new(
        id,
        0x0800_0000 + o,
        0x2000_0000 + o,
        0x2000_0400 + o,
        MpuRegion::new(0x2000_0000 + o, 4096, 0),
    )
}
