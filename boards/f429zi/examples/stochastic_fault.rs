//! Stochastic Fault Injection — ADC noise triggers random divide-by-zero
//!
//! P0 (sensor): reads ADC internal temp at ~10 Hz, masks low 6 bits,
//!   divides by the masked value. When LSBs happen to be zero (~1/64 chance),
//!   divide-by-zero panics. P0 has WarmRestart policy — kernel restarts it.
//! P1 (consumer): reads sampling port, counts sequence gaps from restarts.
//! Tick handler: monitors P0 fault/restart transitions.
//!
//! Proves: partition fault from real application data, automatic restart,
//! consumer handles gaps, health monitoring detects restart events.
//!
//! Build: cd f429zi && cargo build --example stochastic_fault \
//!            --features kernel-mpu-hal-fpu --no-default-features --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, FaultPolicy, MpuRegion, PartitionState},
    partition_core::AlignedStack4K,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use stm32f4xx_hal::{
    adc::{config::AdcConfig, config::SampleTime, Adc, Temperature},
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    signature::{VtempCal30, VtempCal110},
};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const ADC1_BASE: u32 = 0x4001_2000;

// Diagnostics
static SENSOR_READS: AtomicU32 = AtomicU32::new(0);
static SENSOR_DIVIDES: AtomicU32 = AtomicU32::new(0);
static FAULT_COUNT: AtomicU32 = AtomicU32::new(0);
static RESTART_COUNT: AtomicU32 = AtomicU32::new(0);
static CONSUMER_READS: AtomicU32 = AtomicU32::new(0);
static CONSUMER_GAPS: AtomicU32 = AtomicU32::new(0);
static LAST_P0_STATE: AtomicU32 = AtomicU32::new(0); // 0=unknown, 1=running, 2=faulted

// Globals
static mut G_ADC: Option<Adc<pac::ADC1>> = None;
static mut CAL30: f32 = 0.0;
static mut CAL110: f32 = 0.0;

kernel::kernel_config!(FaultCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SP = 2; SM = 8;
    QD = 2; QM = 4; QW = 2;
    BM = 2; BW = 2;
});

kernel::define_kernel!(FaultCfg, |tick, k| {
    // Track P0 state transitions
    let p0_state = k.pcb(0).map(|p| p.state());
    let prev = LAST_P0_STATE.load(Ordering::Relaxed);

    match p0_state {
        Some(PartitionState::Faulted) => {
            if prev != 2 {
                FAULT_COUNT.fetch_add(1, Ordering::Release);
                LAST_P0_STATE.store(2, Ordering::Release);
                let faults = FAULT_COUNT.load(Ordering::Relaxed);
                rprintln!("[{:5}ms] P0 FAULTED (#{}) — div-by-zero from ADC noise", tick, faults);
            }
        }
        Some(PartitionState::Ready | PartitionState::Running) => {
            if prev == 2 {
                RESTART_COUNT.fetch_add(1, Ordering::Release);
                let restarts = RESTART_COUNT.load(Ordering::Relaxed);
                rprintln!("[{:5}ms] P0 RESTARTED (#{}) — warm restart", tick, restarts);
            }
            LAST_P0_STATE.store(1, Ordering::Release);
        }
        _ => {}
    }

    if tick % 2000 == 0 && tick > 0 {
        let reads = SENSOR_READS.load(Ordering::Acquire);
        let divs = SENSOR_DIVIDES.load(Ordering::Acquire);
        let faults = FAULT_COUNT.load(Ordering::Acquire);
        let restarts = RESTART_COUNT.load(Ordering::Acquire);
        let creads = CONSUMER_READS.load(Ordering::Acquire);
        let gaps = CONSUMER_GAPS.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] reads={} divides={} faults={} restarts={} consumer={} gaps={}",
            tick, reads, divs, faults, restarts, creads, gaps
        );
        if faults >= 3 && restarts >= 2 {
            rprintln!("SUCCESS: stochastic fault injection working! faults={} restarts={} gaps={}", faults, restarts, gaps);
        }
    }
});

// ---------------------------------------------------------------------------
// P0: sensor with stochastic fault
// ---------------------------------------------------------------------------

extern "C" fn sensor_body(port_id: u32) -> ! {
    // No error handler — rely on WarmRestart fault policy for auto-restart.
    // Use as_mut() instead of take() — survives warm restart.
    let adc = unsafe { (*addr_of_mut!(G_ADC)).as_mut().unwrap() };
    let cal30 = unsafe { CAL30 };
    let cal110 = unsafe { CAL110 };
    let port = port_id.into();
    let mut seq: u32 = 0;

    loop {
        let raw: u16 = adc.convert(&Temperature, SampleTime::Cycles_480);
        SENSOR_READS.fetch_add(1, Ordering::Release);

        // Temperature calculation (uses FPU)
        let t = (110.0 - 30.0) * (raw as f32 - cal30) / (cal110 - cal30) + 30.0;
        let t_x100 = (t * 100.0) as i32;

        // THE STOCHASTIC FAULT: mask low 2 bits of raw ADC value.
        // When bits[1:0] == 0 (~1/4 chance), write to an ungranted peripheral
        // address → MemManage fault. The kernel catches MemManage, invokes
        // the error handler, which requests warm restart.
        // (UsageFault/DIV_0_TRP won't work — kernel only handles MemManage.)
        let divisor = (raw & 0x03) as u32;
        if divisor == 0 {
            unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD) };
        }
        SENSOR_DIVIDES.fetch_add(1, Ordering::Release);

        // Write reading to sampling port
        seq = seq.wrapping_add(1);
        let record = [
            (t_x100 & 0xFF) as u8,
            ((t_x100 >> 8) & 0xFF) as u8,
            (seq & 0xFF) as u8,
            ((seq >> 8) & 0xFF) as u8,
        ];
        plib::sys_sampling_write(port, &record).ok();

        // Yield between reads — rate determined by schedule slot allocation
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(sensor_main => sensor_body);

// ---------------------------------------------------------------------------
// P1: consumer — reads sampling port, detects sequence gaps
// ---------------------------------------------------------------------------
extern "C" fn consumer_body(port_id: u32) -> ! {
    let port = port_id.into();
    let mut last_seq: u32 = 0;

    loop {
        let mut buf = [0u8; 4];
        if let Ok(sz) = plib::sys_sampling_read(port, &mut buf) {
            if sz as usize >= 4 {
                CONSUMER_READS.fetch_add(1, Ordering::Release);
                let seq = (buf[2] as u32) | ((buf[3] as u32) << 8);
                // Detect gaps: seq should be last_seq + 1 (unless restart reset it)
                if last_seq > 0 && seq != last_seq + 1 && seq != 1 {
                    CONSUMER_GAPS.fetch_add(1, Ordering::Release);
                }
                last_seq = seq;
            }
        }
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(consumer_main => consumer_body);

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();
    kernel::boot::init_fpu().expect("FPU init");
    // kernel-mpu-hal-fpu includes fpu-context — FPU regs saved/restored on context switch.


    let mut rcc = dp.RCC.freeze(RccConfig::hse(8.MHz()).sysclk(168.MHz()));

    // ADC1 internal temp sensor
    let mut adc = Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc);
    adc.enable_temperature_and_vref();
    adc.calibrate();
    unsafe {
        CAL30 = VtempCal30::get().read() as f32;
        CAL110 = VtempCal110::get().read() as f32;
        *addr_of_mut!(G_ADC) = Some(adc);
    }

    rprintln!("\n=== Stochastic Fault Injection — ADC noise div-by-zero ===");
    rprintln!("P0: reads ADC, masks low 2 bits, divides. ~1/4 chance of panic.");
    rprintln!("P0 has WarmRestart(max=10). Expect faults + restarts.\n");

    let mut sched = ScheduleTable::<{ FaultCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 20)).expect("P0"); // 20ms sensor slot
    sched.add_system_window(1).expect("SW1");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW2");

    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;
    let flash = MpuRegion::new(FLASH_BASE, 2 * 1024 * 1024, 0);
    let sram = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(s0, sensor_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(0))
        .expect("mem0").with_code_mpu_region(flash).expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(0x4001_0000, 16384, 0), // APB2 block: covers ADC1
            MpuRegion::new(0x4000_0000, 1024, 0),  // dummy — equalize with P1
        ]).expect("periph0")
        .with_r0_hint(0); // sampling port src

    let mem1 = ExternalPartitionMemory::from_aligned_stack(s1, consumer_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(1))
        .expect("mem1").with_code_mpu_region(flash).expect("code1")
        .with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 1024, 0), // dummy x2 — equalize
            MpuRegion::new(0x4000_1000, 1024, 0),
        ]).expect("periph1")
        .with_r0_hint(1); // sampling port dst

    let mut k = Kernel::<FaultCfg>::new(sched, &[mem0, mem1]).expect("kernel");

    // Set P0 to WarmRestart — automatically restarts after fault
    k.pcb_mut(0).expect("pcb0").set_fault_policy(FaultPolicy::WarmRestart { max: 10 });

    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<FaultCfg, _, _>(|k| {
        let s = k.sampling_mut().create_port(PortDirection::Source, 100).expect("src");
        let d = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("dst");
        k.sampling_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc");

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
