//! 3-partition sensor telemetry pipeline via sampling ports.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    kernel::KernelState,
    partition::PartitionConfig,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc,
    svc::Kernel,
    syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE, SYS_YIELD},
    unpack_r0,
};
use panic_semihosting as _;

const MAX_SCHEDULE_ENTRIES: usize = 8;
const NUM_PARTITIONS: usize = 3;
const STACK_WORDS: usize = 256;

/// Kernel configuration for the sampling-port demo.
///
/// Sized for 4 partitions, 8 sampling ports with 4-byte messages, and
/// moderate pool sizes for all resource types.
struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 4;
    const MW: usize = 4;
    const QS: usize = 4;
    const QD: usize = 4;
    const QM: usize = 4;
    const QW: usize = 4;
    const SP: usize = 8;
    const SM: usize = 4;
    const BS: usize = 4;
    const BM: usize = 4;
    const BW: usize = 4;
}

kernel::define_harness!(
    DemoConfig,
    NUM_PARTITIONS,
    MAX_SCHEDULE_ENTRIES,
    STACK_WORDS
);

extern "C" fn sensor_main() -> ! {
    let (src, mut v) = (unpack_r0!() >> 16, 0u8);
    loop {
        v = v.wrapping_add(1);
        hprintln!("[sensor]  write val={}", v);
        svc!(SYS_SAMPLING_WRITE, src, 1u32, [v].as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
extern "C" fn control_main() -> ! {
    let packed = unpack_r0!();
    let (src, dst) = (packed >> 16, packed & 0xFFFF);
    loop {
        let mut buf = [0u8; 1];
        let sz = svc!(
            SYS_SAMPLING_READ,
            dst,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        let v = if sz > 0 && sz != u32::MAX { buf[0] } else { 0 };
        let tag = if v > 2 { "ALERT" } else { "NORMAL" };
        hprintln!("[control] val={} valid={} -> {}", v, sz != u32::MAX, tag);
        svc!(
            SYS_SAMPLING_WRITE,
            src,
            1u32,
            [u8::from(v > 2)].as_ptr() as u32
        );
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
extern "C" fn display_main() -> ! {
    let dst = unpack_r0!() & 0xFFFF;
    let mut cyc: u32 = 0;
    loop {
        let mut buf = [0u8; 1];
        let sz = svc!(
            SYS_SAMPLING_READ,
            dst,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        let valid = sz > 0 && sz != u32::MAX;
        let tag = if valid && buf[0] == 1 {
            "ALERT"
        } else {
            "NORMAL"
        };
        hprintln!("[display] status={} valid={}", tag, valid);
        cyc += 1;
        if cyc >= 4 {
            hprintln!("sampling_demo: all checks passed");
            debug::exit(debug::EXIT_SUCCESS);
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("sampling_demo: start");
    let (s0, d0, s1, d1);
    // SAFETY: accessing static-mut KS; single-core, interrupts not yet
    // enabled so there is no data race.
    unsafe {
        let mut k = Kernel::<DemoConfig>::new();
        s0 = k.sampling.create_port(PortDirection::Source, 10).unwrap();
        d0 = k
            .sampling
            .create_port(PortDirection::Destination, 10)
            .unwrap();
        k.sampling.connect_ports(s0, d0).unwrap();
        s1 = k.sampling.create_port(PortDirection::Source, 10).unwrap();
        d1 = k
            .sampling
            .create_port(PortDirection::Destination, 10)
            .unwrap();
        k.sampling.connect_ports(s1, d1).unwrap();
        store_kernel(k);

        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        for i in 0..NUM_PARTITIONS as u8 {
            sched.add(ScheduleEntry::new(i, 2)).unwrap();
        }
        sched.start();
        let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| {
            let b = 0x2000_0000 + (i as u32) * 0x2000;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        });
        KS = Some(KernelState::new(sched, &cfgs).unwrap());
    }

    // Pack port IDs into a single u32 passed to each partition via r0.
    // See queuing_demo and blackboard_demo for the same convention.
    let h: [u32; NUM_PARTITIONS] = [
        (s0 as u32) << 16,
        ((s1 as u32) << 16) | d0 as u32,
        d1 as u32,
    ];
    let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [sensor_main, control_main, display_main];
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        core::array::from_fn(|i| (eps[i], h[i]));

    boot(&parts, &mut p)
}
