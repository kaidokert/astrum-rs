//! Health Partition — Hardware Verification (STM32F429ZI)
//!
//! Port of upstream's health_partition_test from QEMU to real hardware.
//! Uses kernel::health::health_entry — the kernel-provided health monitoring
//! entry point that automatically checks partition liveness, schedule timing,
//! and reports health status via sampling port.
//!
//! P0, P1: worker partitions (counter loops)
//! P2: health partition (kernel::health::health_entry with SystemHealthConfig)
//! Tick handler: reads health report from sampling port, verifies Ok status
//!
//! Build: cd f429zi && cargo build --example health_partition_hw \
//!            --features kernel-health --no-default-features --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    health::{HealthReport, HealthStatus, SystemHealthConfig, HealthAction},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    PartitionEntry, PartitionSpec,
    {DebugEnabled, MsgMinimal, Partitions4, PortsSmall, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

kernel::kernel_config!(
    HealthCfg<Partitions4, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled> {
        core_clock_hz = f429zi::CORE_CLOCK_HZ;
        sampling_msg_size = 16;
    }
);

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);
static HEALTH_OK: AtomicU32 = AtomicU32::new(0);
static HEALTH_REPORTS: AtomicU32 = AtomicU32::new(0);

const HEALTH_DST_PORT: usize = 1;

static HEALTH_CFG: SystemHealthConfig = SystemHealthConfig {
    major_frame_deadline_ticks: 100,
    partition_liveness_frames: 5,
    tick_drift_ppm: 100,
    on_violation: HealthAction::Log,
    sampling_port_id: 0,
    num_partitions: 2, // monitor P0 and P1 only
    watchdog_kick: None,
};

// ── Workers ──
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

// ── Tick handler ──
kernel::define_kernel!(HealthCfg, |tick, k| {
    if tick < 100 { return; } // let health partition warm up

    // Read health report from sampling port
    let mut buf = [0u8; 16];
    let sampling = k.sampling_mut();
    if let Ok((len, _)) = sampling.read_sampling_message(HEALTH_DST_PORT, &mut buf, tick as u64) {
        if len >= 2 {
            if let Some(report) = HealthReport::from_bytes(&buf) {
                HEALTH_REPORTS.fetch_add(1, Ordering::Release);
                if report.status() == HealthStatus::Ok {
                    HEALTH_OK.fetch_add(1, Ordering::Release);
                }
                // Log status on first report and every 1000th
                let count = HEALTH_REPORTS.load(Ordering::Relaxed);
                if count == 1 || count % 1000 == 0 {
                    rprintln!("  health status={:?} len={} raw={:02x?}",
                        report.status(), len, &buf[..len.min(8) as usize]);
                }
            }
        }
    }

    if tick % 1000 == 0 && tick > 0 {
        let p0 = P0_COUNTER.load(Ordering::Acquire);
        let p1 = P1_COUNTER.load(Ordering::Acquire);
        let reports = HEALTH_REPORTS.load(Ordering::Acquire);
        let ok = HEALTH_OK.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] P0={} P1={} health_reports={} ok={}",
            tick, p0, p1, reports, ok
        );
        if ok >= 10 && p0 > 0 && p1 > 0 {
            rprintln!("SUCCESS: health partition working on hardware! reports={} ok={}", reports, ok);
        }
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Health Partition — Hardware Verification ===");
    rprintln!("P0,P1=workers  P2=health_entry  sampling port health reports\n");

    let mut sched = ScheduleTable::<{ HealthCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).expect("P0");
    sched.add_system_window(1).expect("SW0");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW1");
    sched.add(ScheduleEntry::new(2, 3)).expect("P2");
    sched.add_system_window(1).expect("SW2");

    let parts: [PartitionSpec; 3] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
        PartitionSpec::body(
            kernel::health::health_entry,
            &HEALTH_CFG as *const SystemHealthConfig as u32,
        ),
    ];

    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // Health sampling port: source=0 (P2 writes), dest=1 (tick handler reads)
    let src = k.sampling_mut().create_port(PortDirection::Source, 0).expect("src");
    let dst = k.sampling_mut().create_port(PortDirection::Destination, 0).expect("dst");
    k.sampling_mut().connect_ports(src, dst).expect("connect");

    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
