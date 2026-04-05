//! QEMU integration test: 3-partition health monitoring.
//!
//! P0 and P1 are simple worker partitions running counter loops.
//! P2 is the health partition using `health_entry` with `SystemHealthConfig`.
//! The SysTick hook reads the health sampling port and verifies Ok status.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,health-partition --example health_partition_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    health::{HealthReport, HealthStatus, SystemHealthConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions3, PortsSmall, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig < Partitions3,
    SyncMinimal,
    MsgMinimal,
    PortsSmall,
    DebugEnabled > {
        sampling_msg_size = 16;
    }
);

/// Counters incremented by worker partitions to prove they are running.
static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Port ID assigned to the health sampling destination port (auto-assigned second).
const HEALTH_DST_PORT: usize = 1;

/// Health config passed to P2 via r0 pointer.
static HEALTH_CFG: SystemHealthConfig = SystemHealthConfig {
    major_frame_deadline_ticks: 100,
    partition_liveness_frames: 5,
    tick_drift_ppm: 100,
    on_violation: kernel::health::HealthAction::Log,
    sampling_port_id: 0,
    num_partitions: 2,
    watchdog_kick: None,
};

// --- Worker partition entry points ---

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        // TODO: reviewer false positive – cortex_m::asm::nop() is a safe function
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        // TODO: reviewer false positive – cortex_m::asm::nop() is a safe function
        asm::nop();
    }
}

// --- Harness (privileged SysTick context) ---

kernel::define_kernel!(TestConfig, |tick, k| {
    // Health monitor needs partition_liveness_frames (5) * major_frame (16) = 80 ticks
    // before it can report Ok. Start polling after that window.
    if tick < 100 {
        return;
    }

    // Read health report from destination port.
    let mut buf = [0u8; 16];
    let sampling = k.sampling_mut();
    match sampling.read_sampling_message(HEALTH_DST_PORT, &mut buf, tick as u64) {
        Ok((len, _validity)) => {
            if len < 2 {
                hprintln!("health_partition_test: FAIL short report len={}", len);
                kernel::kexit!(failure);
            }
            match HealthReport::from_bytes(&buf) {
                Some(report) => {
                    let p0 = P0_COUNTER.load(Ordering::Relaxed);
                    let p1 = P1_COUNTER.load(Ordering::Relaxed);
                    if report.status() == HealthStatus::Ok && p0 > 0 && p1 > 0 {
                        hprintln!(
                            "health_partition_test: PASS (status=Ok, p0={}, p1={})",
                            p0,
                            p1
                        );
                        kernel::kexit!(success);
                    }
                    if tick >= 200 {
                        hprintln!(
                            "health_partition_test: FAIL status={:?} p0={} p1={}",
                            report.status(),
                            p0,
                            p1
                        );
                        kernel::kexit!(failure);
                    }
                }
                None => {
                    if tick >= 200 {
                        hprintln!("health_partition_test: FAIL bad report parse");
                        kernel::kexit!(failure);
                    }
                }
            }
        }
        Err(_) => {
            if tick >= 200 {
                hprintln!("health_partition_test: FAIL no report by tick 200");
                kernel::kexit!(failure);
            }
        }
    }
});

// --- Boot ---

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("health_partition_test: start");

    // Schedule: P0(5), sys(1), P1(5), sys(1), P2(3), sys(1) — 16-tick major frame.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 5)).expect("add P1");
    sched.add_system_window(1).expect("sys1");
    sched.add(ScheduleEntry::new(2, 3)).expect("add P2");
    sched.add_system_window(1).expect("sys2");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
        PartitionSpec::body(
            kernel::health::health_entry,
            &HEALTH_CFG as *const SystemHealthConfig as u32,
        ),
    ];

    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // TODO: reviewer false positive – second arg to create_port is refresh_period, not port ID.
    // Port IDs are auto-assigned: source=0, dest=1 (HEALTH_DST_PORT).
    let src = k
        .sampling_mut()
        .create_port(PortDirection::Source, 0)
        .expect("src port");
    let dst = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 0)
        .expect("dst port");
    k.sampling_mut().connect_ports(src, dst).expect("connect");

    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
