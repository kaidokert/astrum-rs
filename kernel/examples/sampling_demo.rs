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
    partition::{ExternalPartitionMemory, MpuRegion},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    PartitionEntry,
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

kernel::kernel_config!(DemoConfig<kernel::Partitions4, kernel::SyncStandard, kernel::MsgStandard, kernel::PortsStandard, kernel::DebugEnabled>);

// Use the unified harness macro (no_boot variant) with SysTick hook for progress verification.
// The hook runs in privileged handler mode and can use semihosting.
// We call kernel::boot directly instead of the macro-generated boot().
kernel::define_kernel!(no_boot, DemoConfig, |tick, _k| {
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

// Port IDs stored by main() before boot, read by partition entry functions.
static SENSOR_SRC: AtomicU32 = AtomicU32::new(0);
static CONTROL_SRC: AtomicU32 = AtomicU32::new(0);
static CONTROL_DST: AtomicU32 = AtomicU32::new(0);
static DISPLAY_DST: AtomicU32 = AtomicU32::new(0);

const _: PartitionEntry = sensor_main;
extern "C" fn sensor_main() -> ! {
    let src = plib::SamplingPortId::new(SENSOR_SRC.load(Ordering::Acquire));
    let mut v = 0u8;
    loop {
        v = v.wrapping_add(1);
        if plib::sys_sampling_write(src, &[v]).is_ok() {
            SENSOR_VALUE.store(v as u32, Ordering::Release);
        }
        let _ = plib::sys_yield();
    }
}
const _: PartitionEntry = control_main;
extern "C" fn control_main() -> ! {
    let src = plib::SamplingPortId::new(CONTROL_SRC.load(Ordering::Acquire));
    let dst = plib::SamplingPortId::new(CONTROL_DST.load(Ordering::Acquire));
    loop {
        let mut buf = [0u8; 1];
        let v = match plib::sys_sampling_read(dst, &mut buf) {
            Ok(sz) if sz > 0 => buf[0],
            _ => 0,
        };
        let status = u8::from(v > 2); // 1 = alert, 0 = normal
        if plib::sys_sampling_write(src, &[status]).is_ok() {
            CONTROL_READ.store(v as u32, Ordering::Release);
            CONTROL_STATUS.store(status as u32, Ordering::Release);
        }
        let _ = plib::sys_yield();
    }
}
const _: PartitionEntry = display_main;
extern "C" fn display_main() -> ! {
    let dst = plib::SamplingPortId::new(DISPLAY_DST.load(Ordering::Acquire));
    let mut cyc: u32 = 0;
    loop {
        let mut buf = [0u8; 1];
        match plib::sys_sampling_read(dst, &mut buf) {
            Ok(sz) if sz > 0 && buf[0] > 1 => {
                DATA_ERRORS.fetch_add(1, Ordering::Release);
            }
            Err(_) => {
                DATA_ERRORS.fetch_add(1, Ordering::Release);
            }
            _ => {}
        }
        cyc += 1;
        let _ = plib::sys_yield();
        DISPLAY_CYCLES.store(cyc, Ordering::Release);
    }
}
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("sampling_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..NUM_PARTITIONS as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    let entry_fns: [PartitionEntry; NUM_PARTITIONS] = [sensor_main, control_main, display_main];
    let k = {
        let stacks = kernel::partition_stacks!(DemoConfig, NUM_PARTITIONS);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NUM_PARTITIONS] = core::array::from_fn(|i| {
            // SAFETY: i < NUM_PARTITIONS, stacks has NUM_PARTITIONS elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(i as u32),
            )
            .expect("mem")
        });
        Kernel::<DemoConfig>::new(sched, &memories).expect("kernel")
    };

    // Create and connect sampling ports.
    let mut k = k;
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

    // Store port IDs in atomics so partition entry functions can read them.
    SENSOR_SRC.store(s0 as u32, Ordering::Release);
    CONTROL_SRC.store(s1 as u32, Ordering::Release);
    CONTROL_DST.store(d0 as u32, Ordering::Release);
    DISPLAY_DST.store(d1 as u32, Ordering::Release);

    // TODO: reviewer flagged as unrelated — required by store_kernel signature change in dff1322
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<DemoConfig>(p) }.expect("boot") {}
}
