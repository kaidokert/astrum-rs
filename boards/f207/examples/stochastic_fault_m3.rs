//! Stochastic Fault Injection — Cortex-M3 (no FPU) variant
//!
//! Same concept as f429zi's stochastic_fault but without FPU math:
//!   P0 (sensor): reads ADC1 internal temp sensor via raw registers,
//!     masks low 2 bits of the raw value. When bits == 0 (~1/4 chance),
//!     writes to an ungranted peripheral → MemManage fault.
//!     WarmRestart policy → kernel restarts it automatically.
//!   P1 (consumer): reads sampling port, detects sequence gaps from restarts.
//!   Tick handler: monitors P0 fault/restart transitions, reports start_condition.
//!
//! Exercises on Cortex-M3 (non-FPU path):
//!   MPU enforcement, ExternalPartitionMemory, WarmRestart, MemManage handler,
//!   sys_get_start_condition, sampling port IPC, health monitoring.
//!
//! Build: cd f207 && cargo build --example stochastic_fault_m3 \
//!            --features kernel-mpu --release

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
    partition_core::AlignedStack2K,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f207::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

// ADC1 registers (STM32F2xx — same layout as F4)
const RCC_APB2ENR: *mut u32 = 0x4002_3844 as *mut u32;
const ADC1_BASE: u32 = 0x4001_2000;
const ADC1_SR: *mut u32 = ADC1_BASE as *mut u32;
const ADC1_CR1: *mut u32 = (ADC1_BASE + 0x04) as *mut u32;
const ADC1_CR2: *mut u32 = (ADC1_BASE + 0x08) as *mut u32;
const ADC1_SMPR1: *mut u32 = (ADC1_BASE + 0x0C) as *mut u32;
const ADC1_SQR3: *mut u32 = (ADC1_BASE + 0x34) as *mut u32;
const ADC1_DR: *const u32 = (ADC1_BASE + 0x4C) as *const u32;
const ADC_CCR: *mut u32 = (ADC1_BASE + 0x300 + 0x04) as *mut u32;

// Diagnostics
static SENSOR_READS: AtomicU32 = AtomicU32::new(0);
static FAULT_COUNT: AtomicU32 = AtomicU32::new(0);
static RESTART_COUNT: AtomicU32 = AtomicU32::new(0);
static CONSUMER_READS: AtomicU32 = AtomicU32::new(0);
static CONSUMER_GAPS: AtomicU32 = AtomicU32::new(0);
static WARM_STARTS: AtomicU32 = AtomicU32::new(0);
static LAST_P0_STATE: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(FaultCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f207::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SP = 2; SM = 8;
    QD = 2; QM = 4; QW = 2;
    BM = 2; BW = 2;
});

kernel::define_kernel!(FaultCfg, |tick, k| {
    let p0_state = k.pcb(0).map(|p| p.state());
    let prev = LAST_P0_STATE.load(Ordering::Relaxed);

    match p0_state {
        Some(PartitionState::Faulted) => {
            if prev != 2 {
                FAULT_COUNT.fetch_add(1, Ordering::Release);
                LAST_P0_STATE.store(2, Ordering::Release);
            }
        }
        Some(PartitionState::Ready | PartitionState::Running) => {
            if prev == 2 {
                RESTART_COUNT.fetch_add(1, Ordering::Release);
            }
            LAST_P0_STATE.store(1, Ordering::Release);
        }
        _ => {}
    }

    if tick % 2000 == 0 && tick > 0 {
        let reads = SENSOR_READS.load(Ordering::Acquire);
        let faults = FAULT_COUNT.load(Ordering::Acquire);
        let restarts = RESTART_COUNT.load(Ordering::Acquire);
        let creads = CONSUMER_READS.load(Ordering::Acquire);
        let gaps = CONSUMER_GAPS.load(Ordering::Acquire);
        let warm = WARM_STARTS.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] reads={} faults={} restarts={} warm_starts={} consumer={} gaps={}",
            tick, reads, faults, restarts, warm, creads, gaps
        );
        if warm >= 3 && reads > 10 {
            rprintln!(
                "SUCCESS: stochastic fault on Cortex-M3! reads={} warm_starts={} consumer={} gaps={}",
                reads, warm, creads, gaps
            );
        }
    }
});

// ---------------------------------------------------------------------------
// P0: sensor — reads ADC, faults stochastically
// ---------------------------------------------------------------------------
extern "C" fn sensor_body(port_id: u32) -> ! {
    // Check start condition
    match plib::sys_get_start_condition() {
        Ok(plib::StartCondition::WarmRestart) => {
            WARM_STARTS.fetch_add(1, Ordering::Relaxed);
        }
        _ => {}
    }

    let port = port_id.into();
    let mut seq: u32 = 0;

    loop {
        // Read ADC1 (raw register access — partition has ADC1 peripheral grant)
        // Start conversion on channel 18 (internal temp sensor)
        unsafe {
            core::ptr::write_volatile(ADC1_CR2, core::ptr::read_volatile(ADC1_CR2) | (1 << 30)); // SWSTART
        }
        // Wait for conversion (EOC flag, bit 1 of SR)
        while unsafe { core::ptr::read_volatile(ADC1_SR) } & (1 << 1) == 0 {}
        let raw = unsafe { core::ptr::read_volatile(ADC1_DR) } as u16;
        SENSOR_READS.fetch_add(1, Ordering::Release);

        // Integer temperature estimate (no FPU!): raw * 100 / 4096 as percentage
        let temp_pct = (raw as u32 * 100) / 4096;

        // THE STOCHASTIC FAULT: XOR ADC value with sequence counter.
        // seq increments each loop, so even if ADC is constant the XOR
        // produces varying low bits. ~1/8 chance of triggering.
        let noisy = raw as u32 ^ seq;
        if noisy & 0x07 == 0 {
            // Write to ungranted peripheral → MemManage fault
            unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD) };
        }

        // Write to sampling port
        seq = seq.wrapping_add(1);
        let record = [
            (temp_pct & 0xFF) as u8,
            ((temp_pct >> 8) & 0xFF) as u8,
            (seq & 0xFF) as u8,
            ((seq >> 8) & 0xFF) as u8,
        ];
        plib::sys_sampling_write(port, &record).ok();
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
// ADC1 init (privileged, before kernel boot)
// ---------------------------------------------------------------------------
unsafe fn init_adc1() {
    // Enable ADC1 clock (RCC APB2ENR bit 8)
    let apb2 = core::ptr::read_volatile(RCC_APB2ENR);
    core::ptr::write_volatile(RCC_APB2ENR, apb2 | (1 << 8));

    // Enable temp sensor + VREFINT (ADC_CCR bit 23 = TSVREFE)
    let ccr = core::ptr::read_volatile(ADC_CCR);
    core::ptr::write_volatile(ADC_CCR, ccr | (1 << 23));

    // ADC1 CR1: 12-bit resolution (bits 25:24 = 00)
    core::ptr::write_volatile(ADC1_CR1, 0);

    // ADC1 CR2: enable ADC (ADON bit 0)
    core::ptr::write_volatile(ADC1_CR2, 1);

    // Sample time for ch18: 480 cycles (SMPR1 bits [26:24] = 0b111)
    let smpr = core::ptr::read_volatile(ADC1_SMPR1);
    core::ptr::write_volatile(ADC1_SMPR1, smpr | (0b111 << 24));

    // Regular sequence: 1 conversion, channel 18
    core::ptr::write_volatile(ADC1_SQR3, 18);

    // Short delay for ADC stabilization
    cortex_m::asm::delay(1000);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    unsafe { init_adc1(); }

    rprintln!("\n=== Stochastic Fault Injection — Cortex-M3 (no FPU) ===");
    rprintln!("P0: ADC temp sensor, masks low 2 bits, ~1/4 fault chance.");
    rprintln!("P0 has WarmRestart(max=20). Expect faults + restarts.\n");

    let mut sched = ScheduleTable::<{ FaultCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 20)).expect("P0");
    sched.add_system_window(1).expect("SW1");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW2");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;
    let flash = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);
    let sram = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);

    // P0: ADC1 peripheral grant (0x40012000, 4KB covers ADC1 + common)
    let mem0 = ExternalPartitionMemory::from_aligned_stack(s0, sensor_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(0))
        .expect("mem0").with_code_mpu_region(flash).expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(0x4001_2000, 4096, 0), // ADC1 + ADC common
        ]).expect("periph0")
        .with_r0_hint(0); // sampling port src

    let mem1 = ExternalPartitionMemory::from_aligned_stack(s1, consumer_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(1))
        .expect("mem1").with_code_mpu_region(flash).expect("code1")
        .with_r0_hint(1); // sampling port dst

    let mut k = Kernel::<FaultCfg>::new(sched, &[mem0, mem1]).expect("kernel");
    k.pcb_mut(0).expect("pcb0").set_fault_policy(FaultPolicy::WarmRestart { max: 20 });
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<FaultCfg, _, _>(|k| {
        let s = k.sampling_mut().create_port(PortDirection::Source, 100).expect("src");
        let d = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("dst");
        k.sampling_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc");

    rprintln!("[INIT] Booting with MPU...\n");
    match boot(p).expect("boot") {}
}
