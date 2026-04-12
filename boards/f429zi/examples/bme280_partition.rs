//! BME280 Sensor Partition — STM32F429ZI NUCLEO-144
//!
//! MPU pass-through: partition directly drives a BME280 sensor over I2C under
//! real MPU enforcement. Proves the embedded-hal ecosystem works inside
//! an isolated RTOS partition.
//!
//! Architecture:
//!   P0 (sensor, pid=0):
//!     Takes pre-configured I2C1, inits BME280, loops measure() → RTT.
//!     MPU grants: I2C1 (0x40005400/1KB) + GPIOB (0x40020400/1KB).
//!
//!   P1 (idle, pid=1):
//!     Loops with counter, proves scheduling works alongside sensor reads.
//!
//! Hardware:
//!   BME280 breakout on I2C1: PB8(SCL), PB9(SDA), 3.3V, GND.
//!   Internal pull-ups enabled (no external resistors needed).
//!
//! Build:  cd f429zi && cargo build --example bme280_partition \
//!             --features kernel-mpu-hal,bme280 --no-default-features --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack4K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use stm32f4xx_hal::{
    i2c::I2c,
    pac,
    prelude::*,
    rcc::Config as RccConfig,
};
use bme280::i2c::BME280;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE};

// ── Constants ──────────────────────────────────────────────────────────────────

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 1024;

const I2C1_BASE: u32  = 0x4000_5400;
const GPIOB_BASE: u32 = 0x4002_0400;

const MPU_REGION_SIZE: u32 = 512 * 1024;

// ── Statistics ─────────────────────────────────────────────────────────────────

static MEASURE_OK:   AtomicU32 = AtomicU32::new(0);
static MEASURE_ERR:  AtomicU32 = AtomicU32::new(0);
static P1_COUNT:     AtomicU32 = AtomicU32::new(0);

// Sensor readings × 100 (fixed-point for atomics).
static TEMP_X100:    AtomicU32 = AtomicU32::new(0);
static HUM_X100:     AtomicU32 = AtomicU32::new(0);
static PRES_X100:    AtomicU32 = AtomicU32::new(0);

// ── HAL object storage ─────────────────────────────────────────────────────────

type I2c1 = I2c<pac::I2C1>;
static mut G_I2C: Option<I2c1> = None;

// ── Kernel configuration ───────────────────────────────────────────────────────

kernel::kernel_config!(SensorCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = 168_000_000;  // HSE 8MHz + PLL → 168MHz
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

// ── SysTick hook ───────────────────────────────────────────────────────────────

kernel::define_kernel!(SensorCfg, |tick, _k| {
    if tick % 2_000 == 0 && tick > 0 {
        let ok  = MEASURE_OK.load(Ordering::Acquire);
        let err = MEASURE_ERR.load(Ordering::Acquire);
        let p1  = P1_COUNT.load(Ordering::Acquire);
        let t   = TEMP_X100.load(Ordering::Acquire) as i32;
        let h   = HUM_X100.load(Ordering::Acquire) as i32;
        let p   = PRES_X100.load(Ordering::Acquire) as i32;
        rprintln!(
            "[{:5}ms] T={}.{:02}C H={}.{:02}% P={}.{:02}hPa  ok={} err={} P1={}",
            tick,
            t / 100, (t % 100).abs(),
            h / 100, (h % 100).abs(),
            p / 100, (p % 100).abs(),
            ok, err, p1
        );
        if ok >= 5 && err == 0 {
            rprintln!("SUCCESS: BME280 partition reading under MPU enforcement.");
        }
    }
});

// ── P0: BME280 sensor partition ────────────────────────────────────────────────

extern "C" fn sensor_body(_r0: u32) -> ! {
    let i2c = unsafe { (*addr_of_mut!(G_I2C)).take().unwrap() };

    // BME280 measure() needs a Delay. We can't use SYST.delay() directly
    // (SysTick is owned by the kernel). Use cortex_m::asm::delay() via
    // a manual DelayNs implementation.
    let mut delay = AsmDelay { freq_hz: 168_000_000 };

    let mut bme = BME280::new_primary(i2c);
    if bme.init(&mut delay).is_err() {
        MEASURE_ERR.fetch_add(1, Ordering::Release);
        loop { cortex_m::asm::wfi(); }
    }

    // Continuous sensor reads — yield between each to let P1 and tick handler run.
    loop {
        match bme.measure(&mut delay) {
            Ok(m) => {
                let t = (m.temperature * 100.0) as i32;
                let h = (m.humidity * 100.0) as i32;
                let p = (m.pressure / 100.0 * 100.0) as i32;
                TEMP_X100.store(t as u32, Ordering::Release);
                HUM_X100.store(h as u32, Ordering::Release);
                PRES_X100.store(p as u32, Ordering::Release);
                MEASURE_OK.fetch_add(1, Ordering::Release);
            }
            Err(_) => {
                MEASURE_ERR.fetch_add(1, Ordering::Release);
            }
        }
        plib::sys_yield().ok();
    }
}

// ── P1: idle partition ─────────────────────────────────────────────────────────

extern "C" fn idle_body(_r0: u32) -> ! {
    loop {
        P1_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(sensor_main => sensor_body);
kernel::partition_trampoline!(idle_main   => idle_body);

// ── AsmDelay — DelayNs via cortex_m::asm::delay ───────────────────────────────

struct AsmDelay {
    freq_hz: u32,
}

impl stm32f4xx_hal::hal::delay::DelayNs for AsmDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (self.freq_hz as u64 * ns as u64) / 1_000_000_000;
        cortex_m::asm::delay(cycles as u32);
    }
}

// ── main ───────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    // RTT is initialized by the kernel during boot() — no rprintln! before boot().

    let dp = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();

    // Enable FPU context save/restore — BME280 uses floats (temperature * 100.0).
    // Without this, FPU registers are clobbered across context switches, corrupting
    // callee-saved registers (including LR) that the compiler spilled to FPU regs.
    kernel::boot::init_fpu().expect("FPU init");

    let mut rcc = dp.RCC.freeze(
        RccConfig::hse(8.MHz()).sysclk(168.MHz()),
    );

    // ── I2C1 (PB8=SCL, PB9=SDA, 100kHz) ──
    let gpiob = dp.GPIOB.split(&mut rcc);
    let scl = gpiob.pb8.internal_pull_up(true);
    let sda = gpiob.pb9.internal_pull_up(true);
    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);

    unsafe {
        *addr_of_mut!(G_I2C) = Some(i2c);
    }

    let mut sched = ScheduleTable::<{ SensorCfg::SCHED }>::new();
    // P0 gets 90ms — enough for BME280 init + measure without preemption.
    // Must be < 100 ticks (SystemWindowTooInfrequent threshold).
    sched.add(ScheduleEntry::new(0, 90)).expect("sched P0");
    sched.add_system_window(1).expect("sched SW1");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW2");

    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let sentinel_mpu = MpuRegion::new(SRAM_BASE, MPU_REGION_SIZE, 0);

    // P0: I2C1 (1KB) + GPIOB (1KB) — sensor partition.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, sensor_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(I2C1_BASE, 1024, 0),
            MpuRegion::new(GPIOB_BASE, 1024, 0),
        ])
        .expect("periph0");

    // P1: no peripherals, but needs a dummy region for Bug 28-chinchilla
    // (peripheral_reserved is global — both partitions must have regions).
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, idle_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
        .expect("code1")
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 1024, 0)])
        .expect("periph1");
    let mems = [mem0, mem1];

    let mut k = Kernel::<SensorCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    // NOTE: rprintln! is NOT available before boot() — RTT is initialized inside boot().
    // All pre-boot prints would write to uninitialized memory and crash.
    match boot(p).expect("boot") {}
}
