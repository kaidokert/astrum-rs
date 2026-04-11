//! Dual Temperature Logger (2-Partition) — BME280 + ADC → FRAM
//!
//! P0 (sensor): reads both BME280 (I2C1) and internal ADC temp sensor,
//!   writes fused record to sampling port.
//! P1 (storage): reads sampling port, writes to SPI FRAM circular log.
//!
//! Build: cd f429zi && cargo build --example dual_temp_2part \
//!            --features kernel-mpu-hal,bme280 --no-default-features --release

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
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use stm32f4xx_hal::{
    adc::{config::{AdcConfig, SampleTime}, Adc, Temperature},
    gpio::{Output, PushPull, Pin},
    i2c::I2c,
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    signature::{VtempCal30, VtempCal110},
    spi::{Mode, Phase, Polarity, Spi},
};
use bme280::i2c::BME280;
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

// Peripheral bases
const I2C1_BASE: u32 = 0x4000_5400;
const ADC1_BASE: u32 = 0x4001_2000;
const GPIOB_BASE: u32 = 0x4002_0400;
const SPI1_BASE: u32 = 0x4001_3000;
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOD_BASE: u32 = 0x4002_0C00;

// ---------------------------------------------------------------------------
// Fused record: both sensors in one struct
// ---------------------------------------------------------------------------
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct FusedRecord {
    bme_temp_x100: i16,
    bme_hum_x100: u16,
    bme_pres_x10: u16,
    adc_temp_x100: i16,
    seq: u16,
}
const REC_SIZE: usize = core::mem::size_of::<FusedRecord>();

// Diagnostics
static SENSOR_OK: AtomicU32 = AtomicU32::new(0);
static SENSOR_ERR: AtomicU32 = AtomicU32::new(0);
static FRAM_WRITES: AtomicU32 = AtomicU32::new(0);
static LAST_BME_T: AtomicU32 = AtomicU32::new(0);
static LAST_ADC_T: AtomicU32 = AtomicU32::new(0);
static LAST_SEQ: AtomicU32 = AtomicU32::new(0);

// Globals
type I2cType = I2c<pac::I2C1>;
type AdcType = Adc<pac::ADC1>;
type SpiType = Spi<pac::SPI1>;
type CsPin = Pin<'D', 14, Output<PushPull>>;

static mut G_I2C: Option<I2cType> = None;
static mut G_ADC: Option<AdcType> = None;
static mut G_SPI: Option<SpiType> = None;
static mut G_CS: Option<CsPin> = None;
// Factory calibration (read in main, used in partition)
static mut CAL30: f32 = 0.0;
static mut CAL110: f32 = 0.0;

// Kernel config
kernel::kernel_config!(DualCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SP = 2; SM = REC_SIZE;
    QD = 2; QM = 4; QW = 2;
    BM = 2; BW = 2;
});

kernel::define_kernel!(DualCfg, |tick, _k| {
    if tick % 2_000 == 0 && tick > 0 {
        let sok = SENSOR_OK.load(Ordering::Acquire);
        let serr = SENSOR_ERR.load(Ordering::Acquire);
        let fw = FRAM_WRITES.load(Ordering::Acquire);
        let bt = LAST_BME_T.load(Ordering::Acquire) as i32;
        let at = LAST_ADC_T.load(Ordering::Acquire) as i32;
        let seq = LAST_SEQ.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] ok={} err={} fram={} BME={}.{:02}C ADC={}.{:02}C seq={}",
            tick, sok, serr, fw,
            bt / 100, (bt % 100).abs(),
            at / 100, (at % 100).abs(),
            seq
        );
        if fw >= 10 && serr == 0 {
            rprintln!("SUCCESS: dual-temp 2-partition pipeline working!");
        }
    }
});

struct AsmDelay { freq_hz: u32 }
impl stm32f4xx_hal::hal::delay::DelayNs for AsmDelay {
    fn delay_ns(&mut self, ns: u32) {
        cortex_m::asm::delay((self.freq_hz as u64 * ns as u64 / 1_000_000_000) as u32);
    }
}

// ---------------------------------------------------------------------------
// P0: reads BME280 + ADC temp, writes fused record to sampling port
// ---------------------------------------------------------------------------
extern "C" fn sensor_body(port_id: u32) -> ! {
    let i2c = unsafe { (*addr_of_mut!(G_I2C)).take().unwrap() };
    let mut adc = unsafe { (*addr_of_mut!(G_ADC)).take().unwrap() };
    let cal30 = unsafe { CAL30 };
    let cal110 = unsafe { CAL110 };

    let mut delay = AsmDelay { freq_hz: 168_000_000 };
    let mut bme = BME280::new_primary(i2c);
    if bme.init(&mut delay).is_err() {
        SENSOR_ERR.fetch_add(1, Ordering::Release);
        loop { plib::sys_yield().ok(); }
    }

    let port = port_id.into();
    let mut seq: u16 = 0;

    loop {
        match bme.measure(&mut delay) {
            Ok(m) => {
                // ADC internal temp
                let raw: u16 = adc.convert(&Temperature, SampleTime::Cycles_480);
                let adc_t = (110.0 - 30.0) * (raw as f32 - cal30) / (cal110 - cal30) + 30.0;

                let rec = FusedRecord {
                    bme_temp_x100: (m.temperature * 100.0) as i16,
                    bme_hum_x100: (m.humidity * 100.0) as u16,
                    bme_pres_x10: (m.pressure / 10.0) as u16,
                    adc_temp_x100: (adc_t * 100.0) as i16,
                    seq,
                };
                seq = seq.wrapping_add(1);
                SENSOR_OK.fetch_add(1, Ordering::Release);
                LAST_BME_T.store(rec.bme_temp_x100 as u32, Ordering::Release);
                LAST_ADC_T.store(rec.adc_temp_x100 as u32, Ordering::Release);

                let bytes = unsafe {
                    core::slice::from_raw_parts(&rec as *const _ as *const u8, REC_SIZE)
                };
                plib::sys_sampling_write(port, bytes).ok();
            }
            Err(_) => { SENSOR_ERR.fetch_add(1, Ordering::Release); }
        }
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(sensor_main => sensor_body);

// ---------------------------------------------------------------------------
// P1: reads sampling port, writes to SPI FRAM
// ---------------------------------------------------------------------------
const FRAM_SIZE: u16 = 8192;
const WREN: u8 = 0x06;
const WRITE_OP: u8 = 0x02;

extern "C" fn storage_body(port_id: u32) -> ! {
    let mut spi = unsafe { (*addr_of_mut!(G_SPI)).take().unwrap() };
    let mut cs = unsafe { (*addr_of_mut!(G_CS)).take().unwrap() };
    let port = port_id.into();
    let mut addr: u16 = 0;
    let mut last_seq: u16 = 0xFFFF;

    loop {
        let mut buf = [0u8; REC_SIZE];
        if let Ok(sz) = plib::sys_sampling_read(port, &mut buf) {
            if sz as usize >= REC_SIZE {
                let rec = unsafe { *(buf.as_ptr() as *const FusedRecord) };
                if rec.seq != last_seq {
                    last_seq = rec.seq;
                    LAST_SEQ.store(rec.seq as u32, Ordering::Release);

                    cs.set_low();
                    spi.transfer_in_place(&mut [WREN]).ok();
                    cs.set_high();

                    let mut cmd = [0u8; 3 + REC_SIZE];
                    cmd[0] = WRITE_OP;
                    cmd[1] = (addr >> 8) as u8;
                    cmd[2] = addr as u8;
                    cmd[3..3 + REC_SIZE].copy_from_slice(&buf[..REC_SIZE]);
                    cs.set_low();
                    spi.transfer_in_place(&mut cmd).ok();
                    cs.set_high();

                    FRAM_WRITES.fetch_add(1, Ordering::Release);
                    addr = (addr + REC_SIZE as u16) % FRAM_SIZE;
                }
            }
        }
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(storage_main => storage_body);

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();
    kernel::boot::init_fpu().expect("FPU init");

    let mut rcc = dp.RCC.freeze(RccConfig::hse(8.MHz()).sysclk(168.MHz()));

    // I2C1
    let gpiob = dp.GPIOB.split(&mut rcc);
    let i2c = I2c::new(dp.I2C1, (gpiob.pb8.internal_pull_up(true), gpiob.pb9.internal_pull_up(true)), 100.kHz(), &mut rcc);

    // ADC1 + internal temp sensor
    let mut adc = Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc);
    adc.enable_temperature_and_vref();
    adc.calibrate();
    unsafe {
        CAL30 = VtempCal30::get().read() as f32;
        CAL110 = VtempCal110::get().read() as f32;
    }

    // SPI1 + CS
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let mut cs = gpiod.pd14.into_push_pull_output();
    cs.set_high();
    let spi = Spi::new(dp.SPI1, (Some(gpioa.pa5), Some(gpioa.pa6), Some(gpioa.pa7)),
        Mode { polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition },
        1.MHz(), &mut rcc);

    unsafe {
        *addr_of_mut!(G_I2C) = Some(i2c);
        *addr_of_mut!(G_ADC) = Some(adc);
        *addr_of_mut!(G_SPI) = Some(spi);
        *addr_of_mut!(G_CS) = Some(cs);
    }

    // Schedule
    let mut sched = ScheduleTable::<{ DualCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 90)).expect("P0");
    sched.add_system_window(1).expect("SW1");
    sched.add(ScheduleEntry::new(1, 10)).expect("P1");
    sched.add_system_window(1).expect("SW2");

    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;
    let flash = MpuRegion::new(FLASH_BASE, 2 * 1024 * 1024, 0);
    let sram = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);

    // P0: I2C1 + ADC1 (GPIO pins configured in main, not accessed at runtime)
    let mem0 = ExternalPartitionMemory::from_aligned_stack(s0, sensor_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(0))
        .expect("mem0").with_code_mpu_region(flash).expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(I2C1_BASE, 1024, 0),
            MpuRegion::new(ADC1_BASE, 1024, 0),
        ]).expect("periph0")
        .with_r0_hint(0);

    // P1: SPI1 + GPIOD (CS toggle at runtime; GPIOA pins configured in main)
    let mem1 = ExternalPartitionMemory::from_aligned_stack(s1, storage_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(1))
        .expect("mem1").with_code_mpu_region(flash).expect("code1")
        .with_peripheral_regions(&[
            MpuRegion::new(SPI1_BASE, 1024, 0),
            MpuRegion::new(GPIOD_BASE, 1024, 0),
        ]).expect("periph1")
        .with_r0_hint(1);

    let mut k = Kernel::<DualCfg>::new(sched, &[mem0, mem1]).expect("kernel");
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<DualCfg, _, _>(|k| {
        let s = k.sampling_mut().create_port(PortDirection::Source, 100).expect("src");
        let d = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("dst");
        k.sampling_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc");

    match boot(p).expect("boot") {}
}
