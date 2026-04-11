//! Dual Temperature Logger (3-Partition) — Full Isolation
//!
//! P0 (BME280):  reads BME280 over I2C1 → sampling port A
//! P1 (ADC):     reads internal ADC temp → sampling port B
//! P2 (storage): reads both ports, fuses record, writes to SPI FRAM
//!
//! Each partition has access ONLY to its own peripheral — maximum isolation.
//!
//! Build: cd f429zi && cargo build --example dual_temp_3part \
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
    {Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
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

const NUM_PARTITIONS: usize = 3;

const I2C1_BASE: u32 = 0x4000_5400;
const ADC1_BASE: u32 = 0x4001_2000;
const GPIOB_BASE: u32 = 0x4002_0400;
const SPI1_BASE: u32 = 0x4001_3000;
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOD_BASE: u32 = 0x4002_0C00;

// ---------------------------------------------------------------------------
// IPC records
// ---------------------------------------------------------------------------
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct BmeRecord {
    temp_x100: i16,
    hum_x100: u16,
    pres_x10: u16,
    seq: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct AdcRecord {
    temp_x100: i16,
    seq: u16,
}

const BME_SIZE: usize = core::mem::size_of::<BmeRecord>();
const ADC_SIZE: usize = core::mem::size_of::<AdcRecord>();
// Sampling port message size = max of both
const MSG_SIZE: usize = if BME_SIZE > ADC_SIZE { BME_SIZE } else { ADC_SIZE };

// Diagnostics
static BME_OK: AtomicU32 = AtomicU32::new(0);
static BME_ERR: AtomicU32 = AtomicU32::new(0);
static ADC_OK: AtomicU32 = AtomicU32::new(0);
static FRAM_WRITES: AtomicU32 = AtomicU32::new(0);
static LAST_BME_T: AtomicU32 = AtomicU32::new(0);
static LAST_ADC_T: AtomicU32 = AtomicU32::new(0);
static LAST_SEQ: AtomicU32 = AtomicU32::new(0);

// Globals
static mut G_I2C: Option<I2c<pac::I2C1>> = None;
static mut G_ADC: Option<Adc<pac::ADC1>> = None;
static mut G_SPI: Option<Spi<pac::SPI1>> = None;
static mut G_CS: Option<Pin<'D', 14, Output<PushPull>>> = None;
static mut CAL30: f32 = 0.0;
static mut CAL110: f32 = 0.0;

// Kernel config — Partitions4 to hold 3 partitions
kernel::kernel_config!(TriCfg[AlignedStack4K]<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;
    schedule_capacity = 12;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SP = 4; SM = MSG_SIZE; // 4 sampling ports (2 pairs)
    QD = 2; QM = 4; QW = 2;
    BM = 2; BW = 2;
});

kernel::define_kernel!(TriCfg, |tick, _k| {
    if tick % 2_000 == 0 && tick > 0 {
        let bok = BME_OK.load(Ordering::Acquire);
        let berr = BME_ERR.load(Ordering::Acquire);
        let aok = ADC_OK.load(Ordering::Acquire);
        let fw = FRAM_WRITES.load(Ordering::Acquire);
        let bt = LAST_BME_T.load(Ordering::Acquire) as i32;
        let at = LAST_ADC_T.load(Ordering::Acquire) as i32;
        let seq = LAST_SEQ.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] bme={}/{} adc={} fram={} BME={}.{:02}C ADC={}.{:02}C seq={}",
            tick, bok, berr, aok, fw,
            bt / 100, (bt % 100).abs(),
            at / 100, (at % 100).abs(),
            seq
        );
        if fw >= 10 && berr == 0 {
            rprintln!("SUCCESS: dual-temp 3-partition pipeline working!");
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
// P0: BME280 sensor → sampling port A (port 0→1)
// ---------------------------------------------------------------------------
extern "C" fn bme_body(port_id: u32) -> ! {
    let i2c = unsafe { (*addr_of_mut!(G_I2C)).take().unwrap() };
    let mut delay = AsmDelay { freq_hz: 168_000_000 };
    let mut bme = BME280::new_primary(i2c);
    if bme.init(&mut delay).is_err() {
        BME_ERR.fetch_add(1, Ordering::Release);
        loop { plib::sys_yield().ok(); }
    }
    let port = port_id.into();
    let mut seq: u16 = 0;

    loop {
        match bme.measure(&mut delay) {
            Ok(m) => {
                let rec = BmeRecord {
                    temp_x100: (m.temperature * 100.0) as i16,
                    hum_x100: (m.humidity * 100.0) as u16,
                    pres_x10: (m.pressure / 10.0) as u16,
                    seq,
                };
                seq = seq.wrapping_add(1);
                BME_OK.fetch_add(1, Ordering::Release);
                LAST_BME_T.store(rec.temp_x100 as u32, Ordering::Release);
                let bytes = unsafe {
                    core::slice::from_raw_parts(&rec as *const _ as *const u8, BME_SIZE)
                };
                plib::sys_sampling_write(port, bytes).ok();
            }
            Err(_) => { BME_ERR.fetch_add(1, Ordering::Release); }
        }
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(bme_main => bme_body);

// ---------------------------------------------------------------------------
// P1: ADC internal temp → sampling port B (port 2→3)
// ---------------------------------------------------------------------------
extern "C" fn adc_body(port_id: u32) -> ! {
    let mut adc = unsafe { (*addr_of_mut!(G_ADC)).take().unwrap() };
    let cal30 = unsafe { CAL30 };
    let cal110 = unsafe { CAL110 };
    let port = port_id.into();
    let mut seq: u16 = 0;

    loop {
        let raw: u16 = adc.convert(&Temperature, SampleTime::Cycles_480);
        let t = (110.0 - 30.0) * (raw as f32 - cal30) / (cal110 - cal30) + 30.0;
        let rec = AdcRecord {
            temp_x100: (t * 100.0) as i16,
            seq,
        };
        seq = seq.wrapping_add(1);
        ADC_OK.fetch_add(1, Ordering::Release);
        LAST_ADC_T.store(rec.temp_x100 as u32, Ordering::Release);
        let bytes = unsafe {
            core::slice::from_raw_parts(&rec as *const _ as *const u8, ADC_SIZE)
        };
        plib::sys_sampling_write(port, bytes).ok();
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(adc_main => adc_body);

// ---------------------------------------------------------------------------
// P2: reads both sampling ports, fuses, writes to SPI FRAM
// ---------------------------------------------------------------------------
const FRAM_SIZE: u16 = 8192;
const WREN: u8 = 0x06;
const WRITE_OP: u8 = 0x02;

// Packed FRAM record: 12 bytes
#[repr(C)]
struct FramRecord {
    bme_temp_x100: i16,
    bme_hum_x100: u16,
    bme_pres_x10: u16,
    adc_temp_x100: i16,
    seq: u16,
    _pad: u16,
}
const FRAM_REC_SIZE: usize = core::mem::size_of::<FramRecord>();

extern "C" fn storage_body(r0: u32) -> ! {
    let mut spi = unsafe { (*addr_of_mut!(G_SPI)).take().unwrap() };
    let mut cs = unsafe { (*addr_of_mut!(G_CS)).take().unwrap() };
    // r0 encodes both port IDs: bme_dst in high 16, adc_dst in low 16
    let bme_port = (r0 >> 16).into();
    let adc_port = (r0 & 0xFFFF).into();
    let mut addr: u16 = 0;
    let mut seq: u16 = 0;

    let mut last_bme = BmeRecord::default();
    let mut last_adc = AdcRecord::default();

    loop {
        // Read latest from both ports (non-blocking, latest-value-wins)
        let mut buf = [0u8; BME_SIZE];
        if let Ok(sz) = plib::sys_sampling_read(bme_port, &mut buf) {
            if sz as usize >= BME_SIZE {
                last_bme = unsafe { *(buf.as_ptr() as *const BmeRecord) };
            }
        }
        let mut buf2 = [0u8; ADC_SIZE];
        if let Ok(sz) = plib::sys_sampling_read(adc_port, &mut buf2) {
            if sz as usize >= ADC_SIZE {
                last_adc = unsafe { *(buf2.as_ptr() as *const AdcRecord) };
            }
        }

        // Fuse and write to FRAM if we have data from both
        if last_bme.seq > 0 || last_adc.seq > 0 {
            let fused = FramRecord {
                bme_temp_x100: last_bme.temp_x100,
                bme_hum_x100: last_bme.hum_x100,
                bme_pres_x10: last_bme.pres_x10,
                adc_temp_x100: last_adc.temp_x100,
                seq,
                _pad: 0,
            };
            seq = seq.wrapping_add(1);
            LAST_SEQ.store(fused.seq as u32, Ordering::Release);

            let rec_bytes = unsafe {
                core::slice::from_raw_parts(&fused as *const _ as *const u8, FRAM_REC_SIZE)
            };

            cs.set_low();
            spi.transfer_in_place(&mut [WREN]).ok();
            cs.set_high();

            let mut cmd = [0u8; 3 + FRAM_REC_SIZE];
            cmd[0] = WRITE_OP;
            cmd[1] = (addr >> 8) as u8;
            cmd[2] = addr as u8;
            cmd[3..3 + FRAM_REC_SIZE].copy_from_slice(rec_bytes);
            cs.set_low();
            spi.transfer_in_place(&mut cmd).ok();
            cs.set_high();

            FRAM_WRITES.fetch_add(1, Ordering::Release);
            addr = (addr + FRAM_REC_SIZE as u16) % FRAM_SIZE;
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

    // ADC1
    let mut adc = Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc);
    adc.enable_temperature_and_vref();
    adc.calibrate();
    unsafe { CAL30 = VtempCal30::get().read() as f32; CAL110 = VtempCal110::get().read() as f32; }

    // SPI1 + CS
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let mut cs = gpiod.pd14.into_push_pull_output();
    cs.set_high();
    let spi = Spi::new(dp.SPI1, (Some(gpioa.pa5), Some(gpioa.pa6), Some(gpioa.pa7)),
        Mode { polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition }, 1.MHz(), &mut rcc);

    unsafe {
        *addr_of_mut!(G_I2C) = Some(i2c);
        *addr_of_mut!(G_ADC) = Some(adc);
        *addr_of_mut!(G_SPI) = Some(spi);
        *addr_of_mut!(G_CS) = Some(cs);
    }

    // Schedule: P0 (90ms BME280) + P1 (5ms ADC) + P2 (10ms FRAM)
    let mut sched = ScheduleTable::<{ TriCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 90)).expect("P0");
    sched.add_system_window(1).expect("SW1");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW2");
    sched.add(ScheduleEntry::new(2, 10)).expect("P2");
    sched.add_system_window(1).expect("SW3");

    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1, ref mut s2] = *stacks;
    let flash = MpuRegion::new(FLASH_BASE, 2 * 1024 * 1024, 0);
    let sram = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);

    // Each partition gets only the peripherals it accesses at runtime.
    // GPIO pins are configured in main() — no runtime GPIO access needed
    // except P2's CS toggle on GPIOD.
    //
    // All partitions must have the same number of peripheral regions
    // (bug 40-koala: unused slots not cleared on context switch).
    // P0: I2C1 + dummy
    let mem0 = ExternalPartitionMemory::from_aligned_stack(s0, bme_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(0))
        .expect("mem0").with_code_mpu_region(flash).expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(I2C1_BASE, 1024, 0),
            MpuRegion::new(0x4000_0000, 1024, 0), // dummy — bug 40-koala
        ]).expect("periph0")
        .with_r0_hint(0);

    // P1: ADC1 + dummy
    let mem1 = ExternalPartitionMemory::from_aligned_stack(s1, adc_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(1))
        .expect("mem1").with_code_mpu_region(flash).expect("code1")
        .with_peripheral_regions(&[
            MpuRegion::new(ADC1_BASE, 1024, 0),
            MpuRegion::new(0x4000_0000, 1024, 0), // dummy — bug 40-koala
        ]).expect("periph1")
        .with_r0_hint(2);

    // P2: SPI1 + GPIOD (CS toggle at runtime)
    let mem2 = ExternalPartitionMemory::from_aligned_stack(s2, storage_main as kernel::PartitionEntry, sram, kernel::PartitionId::new(2))
        .expect("mem2").with_code_mpu_region(flash).expect("code2")
        .with_peripheral_regions(&[
            MpuRegion::new(SPI1_BASE, 1024, 0),
            MpuRegion::new(GPIOD_BASE, 1024, 0),
        ]).expect("periph2")
        .with_r0_hint((1u32 << 16) | 3u32); // bme_dst=1, adc_dst=3

    let mut k = Kernel::<TriCfg>::new(sched, &[mem0, mem1, mem2]).expect("kernel");
    store_kernel(&mut k);

    // Sampling port pairs: BME280 (0→1), ADC (2→3)
    kernel::state::with_kernel_mut::<TriCfg, _, _>(|k| {
        let bs = k.sampling_mut().create_port(PortDirection::Source, 100).expect("bme src");
        let bd = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("bme dst");
        k.sampling_mut().connect_ports(bs, bd).expect("bme connect");
        let as_ = k.sampling_mut().create_port(PortDirection::Source, 100).expect("adc src");
        let ad = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("adc dst");
        k.sampling_mut().connect_ports(as_, ad).expect("adc connect");
        Ok::<(), ()>(())
    }).expect("ipc");

    match boot(p).expect("boot") {}
}
