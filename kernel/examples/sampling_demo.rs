//! 3-partition sensor telemetry pipeline via sampling ports.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    partition::{MpuRegion, PartitionConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc,
    svc::Kernel,
    syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE, SYS_YIELD},
    unpack_r0,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 3;
const STACK_WORDS: usize = 256;

/// Kernel configuration for the sampling-port demo.
///
/// Sized for 4 partitions, 8 sampling ports with 4-byte messages, and
/// moderate pool sizes for all resource types.
struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
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
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
}

// Use the unified harness macro: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS);

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

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..NUM_PARTITIONS as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    // Build partition configs using the STACKS addresses.
    // SAFETY: single-core, interrupts disabled — exclusive access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        core::array::from_fn(|i| {
            let b = STACKS[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0, // Not used by Kernel::new
                stack_base: b,
                stack_size: (STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (STACK_WORDS * 4) as u32, 0),
            }
        })
    };

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<DemoConfig>::new(sched, &cfgs).expect("kernel creation");

    // Create and connect sampling ports.
    let s0 = k
        .sampling
        .create_port(PortDirection::Source, 10)
        .expect("s0");
    let d0 = k
        .sampling
        .create_port(PortDirection::Destination, 10)
        .expect("d0");
    k.sampling.connect_ports(s0, d0).expect("connect s0->d0");
    let s1 = k
        .sampling
        .create_port(PortDirection::Source, 10)
        .expect("s1");
    let d1 = k
        .sampling
        .create_port(PortDirection::Destination, 10)
        .expect("d1");
    k.sampling.connect_ports(s1, d1).expect("connect s1->d1");

    store_kernel(k);

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
