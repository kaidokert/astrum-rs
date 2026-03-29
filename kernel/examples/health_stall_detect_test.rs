//! QEMU integration test: health partition stall detection.
//!
//! P0 is a normal worker partition (runs a counter loop).
//! P1 is a stalled partition — it exists but is NOT scheduled, simulating a hang.
//! P2 is the health partition monitoring P0 and P1 with partition_liveness_frames=2.
//!
//! After enough major frames, the health partition detects P1's stall and reports
//! Degraded status with HealthViolation::PartitionStall(1).
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting,health-partition --example health_stall_detect_test
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
    health::{HealthReport, HealthStatus, HealthViolation, SystemHealthConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions3, PortsSmall, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig < Partitions3,
    SyncMinimal,
    MsgMinimal,
    PortsSmall,
    DebugEnabled > {
        sampling_msg_size = 16;
    }
);

/// Counter incremented by P0 to prove it is running.
static P0_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Port ID assigned to the health sampling destination port (auto-assigned second).
const HEALTH_DST_PORT: usize = 1;

/// Health config: monitor 2 partitions, detect stall after 2 major frames.
static HEALTH_CFG: SystemHealthConfig = SystemHealthConfig {
    major_frame_deadline_ticks: 100,
    partition_liveness_frames: 2,
    tick_drift_ppm: 100,
    on_violation: kernel::health::HealthAction::Log,
    sampling_port_id: 0,
    num_partitions: 2,
    watchdog_kick: None,
};

// --- Partition entry points ---

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
}

// TODO: P1 simulates a stall by not being scheduled (run_count stays 0). A more
// realistic test would use an application-level heartbeat that P1 fails to send,
// but the current health monitor only tracks kernel-level run_count, so scheduling
// P1 in a tight loop would not trigger PartitionStall detection.
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        asm::nop();
    }
}

// --- Harness (privileged SysTick context) ---

kernel::define_unified_harness!(TestConfig, |tick, k| {
    // Health monitor needs partition_liveness_frames (2) * major_frame (10) = 20 ticks
    // plus extra time for the health partition to run and report. Start polling at 40.
    if tick < 40 {
        return;
    }

    // Read health report from destination port.
    let mut buf = [0u8; 16];
    let sampling = k.sampling_mut();
    match sampling.read_sampling_message(HEALTH_DST_PORT, &mut buf, tick as u64) {
        Ok((len, _validity)) => {
            if len < 2 {
                hprintln!("health_stall_detect_test: FAIL short report len={}", len);
                kernel::kexit!(failure);
            }
            match HealthReport::from_bytes(&buf) {
                Some(report) => {
                    let status = report.status();
                    // Check for Degraded or Critical with PartitionStall(1).
                    if status == HealthStatus::Degraded || status == HealthStatus::Critical {
                        // Verify the violation identifies partition 1.
                        let mut found_stall = false;
                        for i in 0..report.violation_count() as usize {
                            if report.violation(i) == Some(HealthViolation::PartitionStall(1)) {
                                found_stall = true;
                            }
                        }
                        if found_stall {
                            let p0 = P0_COUNTER.load(Ordering::Relaxed);
                            hprintln!(
                                "health_stall_detect_test: PASS (status={:?}, stall=P1, p0={})",
                                status,
                                p0
                            );
                            kernel::kexit!(success);
                        }
                    }
                    // Still Ok — keep waiting.
                    if tick >= 200 {
                        hprintln!(
                            "health_stall_detect_test: FAIL status={:?} no stall detected by tick 200",
                            status
                        );
                        kernel::kexit!(failure);
                    }
                }
                None => {
                    if tick >= 200 {
                        hprintln!("health_stall_detect_test: FAIL bad report parse");
                        kernel::kexit!(failure);
                    }
                }
            }
        }
        Err(_) => {
            if tick >= 200 {
                hprintln!("health_stall_detect_test: FAIL no report by tick 200");
                kernel::kexit!(failure);
            }
        }
    }
});

// --- Boot ---

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("health_stall_detect_test: start");

    // Schedule: P0(5), sys(1), P2(3), sys(1) — 10-tick major frame.
    // P1 is deliberately NOT scheduled to simulate a stalled/hung partition.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(2, 3)).expect("add P2");
    sched.add_system_window(1).expect("sys1");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
        PartitionSpec::body(
            kernel::health::health_entry,
            &HEALTH_CFG as *const SystemHealthConfig as u32,
        ),
    ];

    let mut k = init_kernel(sched, &parts).expect("init_kernel");

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
