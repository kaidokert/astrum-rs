//! 3-partition sensor telemetry pipeline via sampling ports.
//!
//! Demonstrates sampling port communication between three partitions:
//! - P0 (sensor): writes incrementing values to a sampling port
//! - P1 (control): reads sensor values, writes status (alert if > 2)
//! - P2 (display): reads status and tracks cycles
//!
//! Partitions run unprivileged and cannot use semihosting directly.
//! Progress is tracked via atomics; the SysTick handler verifies state
//! and prints results from privileged handler mode.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    boot,
    partition::PartitionConfig,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc,
    svc::Kernel,
    syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE, SYS_YIELD},
};

// Actual partition count (3) differs from DemoConfig::N (4, from Partitions4 capacity).
const NUM_PARTITIONS: usize = 3;

// Atomic state for partition progress tracking (handler mode reads these).
/// Sensor: last value written (increments each cycle)
static SENSOR_VALUE: AtomicU32 = AtomicU32::new(0);
/// Control: last value read from sensor
static CONTROL_READ: AtomicU32 = AtomicU32::new(0);
/// Control: last status written (1 = alert, 0 = normal)
static CONTROL_STATUS: AtomicU32 = AtomicU32::new(0);
/// Display: cycle count (test passes when this reaches 4)
static DISPLAY_CYCLES: AtomicU32 = AtomicU32::new(0);
/// Data integrity errors (display verifies status is 0 or 1)
static DATA_ERRORS: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(DemoConfig<kernel::Partitions4, kernel::SyncStandard, kernel::MsgStandard, kernel::PortsStandard, kernel::DebugEnabled>);

// Use the unified harness macro (no_boot variant) with SysTick hook for progress verification.
// The hook runs in privileged handler mode and can use semihosting.
// We call kernel::boot directly instead of the macro-generated boot().
kernel::define_unified_harness!(no_boot, DemoConfig, |tick, _k| {
    // Check progress every 10 ticks
    if tick.is_multiple_of(10) {
        let sensor = SENSOR_VALUE.load(Ordering::Acquire);
        let ctrl_read = CONTROL_READ.load(Ordering::Acquire);
        let ctrl_status = CONTROL_STATUS.load(Ordering::Acquire);
        let cycles = DISPLAY_CYCLES.load(Ordering::Acquire);

        hprintln!(
            "[tick {}] sensor={} ctrl_read={} status={} cycles={}",
            tick,
            sensor,
            ctrl_read,
            ctrl_status,
            cycles
        );

        // Data integrity: fail on corrupted status values from display
        let data_err = DATA_ERRORS.load(Ordering::Acquire);
        if data_err > 0 {
            hprintln!("sampling_demo: FAIL - {} data integrity errors", data_err);
            debug::exit(debug::EXIT_FAILURE);
        }

        // Test passes when display has completed 4 cycles AND data flowed through pipeline
        if cycles >= 4 {
            if sensor == 0 || ctrl_read == 0 {
                hprintln!(
                    "sampling_demo: FAIL - no data flow (sensor={} ctrl={})",
                    sensor,
                    ctrl_read
                );
                debug::exit(debug::EXIT_FAILURE);
            }
            hprintln!("sampling_demo: all checks passed");
            debug::exit(debug::EXIT_SUCCESS);
        }

        // Timeout after 200 ticks
        if tick > 200 {
            hprintln!("sampling_demo: FAIL - timeout");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
});

extern "C" fn sensor_main_body(r0: u32) -> ! {
    let (src, mut v) = (r0 >> 16, 0u8);
    loop {
        v = v.wrapping_add(1);
        SENSOR_VALUE.store(v as u32, Ordering::Release);
        svc!(SYS_SAMPLING_WRITE, src, 1u32, [v].as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
kernel::partition_trampoline!(sensor_main => sensor_main_body);
extern "C" fn control_main_body(r0: u32) -> ! {
    let packed = r0;
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
        let status = u8::from(v > 2); // 1 = alert, 0 = normal
        CONTROL_READ.store(v as u32, Ordering::Release);
        CONTROL_STATUS.store(status as u32, Ordering::Release);
        svc!(SYS_SAMPLING_WRITE, src, 1u32, [status].as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
kernel::partition_trampoline!(control_main => control_main_body);
extern "C" fn display_main_body(r0: u32) -> ! {
    let dst = r0 & 0xFFFF;
    let mut cyc: u32 = 0;
    loop {
        let mut buf = [0u8; 1];
        let _sz = svc!(
            SYS_SAMPLING_READ,
            dst,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        // Verify data: control writes u8::from(v > 2), so status must be 0 or 1
        if _sz > 0 && _sz != u32::MAX && buf[0] > 1 {
            DATA_ERRORS.fetch_add(1, Ordering::Release);
        }
        // Track cycle count; SysTick handler checks this for completion
        cyc += 1;
        DISPLAY_CYCLES.store(cyc, Ordering::Release);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
kernel::partition_trampoline!(display_main => display_main_body);
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("sampling_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..NUM_PARTITIONS as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| {
        PartitionConfig::sentinel(i as u8, (DemoConfig::STACK_WORDS * 4) as u32)
    });

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<DemoConfig>::new(sched, &cfgs).expect("kernel creation");

    // Create and connect sampling ports.
    let s0 = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("s0");
    let d0 = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 10)
        .expect("d0");
    k.sampling_mut()
        .connect_ports(s0, d0)
        .expect("connect s0->d0");
    let s1 = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("s1");
    let d1 = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 10)
        .expect("d1");
    k.sampling_mut()
        .connect_ports(s1, d1)
        .expect("connect s1->d1");

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

    match boot::boot::<DemoConfig>(&parts, &mut p).expect("sampling_demo: boot failed") {}
}
