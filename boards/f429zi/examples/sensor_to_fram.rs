//! Sensor → FRAM Pipeline — Two-Partition IPC Demo (STM32F429ZI)
//!
//! P0 (sensor): reads BME280 over I2C1, writes temperature/humidity/pressure
//!   to a sampling port every cycle.
//! P1 (storage): reads sampling port, writes each record to MB85RS64V SPI FRAM
//!   as a circular log with incrementing sequence number.
//!
//! Proves: two partitions driving independent peripherals (I2C + SPI) under
//! MPU enforcement, communicating via kernel IPC (sampling port).
//!
//! Hardware:
//!   BME280:    I2C1 PB8(SCL)/PB9(SDA), 100kHz
//!   MB85RS64V: SPI1 PA5(SCK)/PA6(MISO)/PA7(MOSI), CS=PD14, 1MHz
//!
//! Build: cd f429zi && cargo build --example sensor_to_fram \
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
    gpio::{Output, PushPull, Pin},
    i2c::I2c,
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    spi::{Mode, Phase, Polarity, Spi},
};
use bme280::i2c::BME280;
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

// Peripheral base addresses for MPU grants
const I2C1_BASE: u32 = 0x4000_5400;
const GPIOB_BASE: u32 = 0x4002_0400;
const SPI1_BASE: u32 = 0x4001_3000;
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOD_BASE: u32 = 0x4002_0C00;

// ---------------------------------------------------------------------------
// IPC record: 12 bytes packed into sampling port
// ---------------------------------------------------------------------------
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct SensorRecord {
    temp_x100: i16,     // temperature × 100 (e.g., 3106 = 31.06°C)
    hum_x100: u16,      // humidity × 100
    pres_x10: u16,      // pressure × 10 hPa (e.g., 10082 = 1008.2 hPa)
    seq: u16,           // sequence number
}

const RECORD_SIZE: usize = core::mem::size_of::<SensorRecord>();

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static SENSOR_OK: AtomicU32 = AtomicU32::new(0);
static SENSOR_ERR: AtomicU32 = AtomicU32::new(0);
static FRAM_WRITES: AtomicU32 = AtomicU32::new(0);
static FRAM_READS: AtomicU32 = AtomicU32::new(0);
static LAST_TEMP: AtomicU32 = AtomicU32::new(0);
static LAST_SEQ: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Globals: peripherals configured in main(), taken by partitions on first run
// ---------------------------------------------------------------------------
type I2cType = I2c<pac::I2C1>;
type SpiType = Spi<pac::SPI1>;
type CsPin = Pin<'D', 14, Output<PushPull>>;

static mut G_I2C: Option<I2cType> = None;
static mut G_SPI: Option<SpiType> = None;
static mut G_CS: Option<CsPin> = None;

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------
kernel::kernel_config!(PipelineCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SP = 2; SM = RECORD_SIZE; // sampling port sized for SensorRecord
    QD = 2; QM = 4; QW = 2;
    BM = 2; BW = 2;
});

kernel::define_kernel!(PipelineCfg, |tick, _k| {
    if tick % 2_000 == 0 && tick > 0 {
        let sok = SENSOR_OK.load(Ordering::Acquire);
        let serr = SENSOR_ERR.load(Ordering::Acquire);
        let fw = FRAM_WRITES.load(Ordering::Acquire);
        let fr = FRAM_READS.load(Ordering::Acquire);
        let t = LAST_TEMP.load(Ordering::Acquire) as i32;
        let seq = LAST_SEQ.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] sensor ok={} err={} | fram writes={} reads={} | T={}.{:02}C seq={}",
            tick, sok, serr, fw, fr,
            t / 100, (t % 100).abs(), seq
        );
        if fw >= 10 && serr == 0 {
            rprintln!("SUCCESS: sensor→FRAM pipeline working! writes={} seq={}", fw, seq);
        }
    }
});

// ---------------------------------------------------------------------------
// Delay helper (no SysTick access from partition)
// ---------------------------------------------------------------------------
struct AsmDelay { freq_hz: u32 }

impl stm32f4xx_hal::hal::delay::DelayNs for AsmDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (self.freq_hz as u64 * ns as u64) / 1_000_000_000;
        cortex_m::asm::delay(cycles as u32);
    }
}

// ---------------------------------------------------------------------------
// P0: Sensor partition — reads BME280, writes to sampling port
// ---------------------------------------------------------------------------
extern "C" fn sensor_body(port_id: u32) -> ! {
    let i2c = unsafe { (*addr_of_mut!(G_I2C)).take().unwrap() };
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
                let rec = SensorRecord {
                    temp_x100: (m.temperature * 100.0) as i16,
                    hum_x100: (m.humidity * 100.0) as u16,
                    pres_x10: (m.pressure / 10.0) as u16,
                    seq,
                };
                seq = seq.wrapping_add(1);
                SENSOR_OK.fetch_add(1, Ordering::Release);
                LAST_TEMP.store(rec.temp_x100 as u32, Ordering::Release);

                // Write record to sampling port as raw bytes
                let bytes = unsafe {
                    core::slice::from_raw_parts(
                        &rec as *const SensorRecord as *const u8,
                        RECORD_SIZE,
                    )
                };
                plib::sys_sampling_write(port, bytes).ok();
            }
            Err(_) => {
                SENSOR_ERR.fetch_add(1, Ordering::Release);
            }
        }
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(sensor_main => sensor_body);

// ---------------------------------------------------------------------------
// P1: Storage partition — reads sampling port, writes to SPI FRAM
// ---------------------------------------------------------------------------
const FRAM_SIZE: u16 = 8192; // MB85RS64V = 8KB
const WREN: u8 = 0x06;
const WRITE_OP: u8 = 0x02;
const READ_OP: u8 = 0x03;

extern "C" fn storage_body(port_id: u32) -> ! {
    let mut spi = unsafe { (*addr_of_mut!(G_SPI)).take().unwrap() };
    let mut cs = unsafe { (*addr_of_mut!(G_CS)).take().unwrap() };

    let port = port_id.into();
    let mut write_addr: u16 = 0;
    let mut last_seq: u16 = 0xFFFF;

    loop {
        // Read latest sensor record from sampling port
        let mut buf = [0u8; RECORD_SIZE];
        if let Ok(sz) = plib::sys_sampling_read(port, &mut buf) {
            if sz as usize >= RECORD_SIZE {
                FRAM_READS.fetch_add(1, Ordering::Release);

                let rec = unsafe { *(buf.as_ptr() as *const SensorRecord) };

                // Only write if new data (different sequence number)
                if rec.seq != last_seq {
                    last_seq = rec.seq;
                    LAST_SEQ.store(rec.seq as u32, Ordering::Release);

                    // WREN
                    cs.set_low();
                    spi.transfer_in_place(&mut [WREN]).ok();
                    cs.set_high();

                    // WRITE: opcode + 16-bit address + record bytes
                    let mut cmd = [0u8; 3 + RECORD_SIZE];
                    cmd[0] = WRITE_OP;
                    cmd[1] = (write_addr >> 8) as u8;
                    cmd[2] = write_addr as u8;
                    cmd[3..3 + RECORD_SIZE].copy_from_slice(&buf[..RECORD_SIZE]);
                    cs.set_low();
                    spi.transfer_in_place(&mut cmd).ok();
                    cs.set_high();

                    FRAM_WRITES.fetch_add(1, Ordering::Release);

                    // Advance circular write pointer
                    write_addr = (write_addr + RECORD_SIZE as u16) % FRAM_SIZE;
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

    // FPU context save — BME280 uses floats
    kernel::boot::init_fpu().expect("FPU init");

    let mut rcc = dp.RCC.freeze(
        RccConfig::hse(8.MHz()).sysclk(168.MHz()),
    );

    // ── I2C1: BME280 (PB8=SCL, PB9=SDA) ──
    let gpiob = dp.GPIOB.split(&mut rcc);
    let scl = gpiob.pb8.internal_pull_up(true);
    let sda = gpiob.pb9.internal_pull_up(true);
    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);

    // ── SPI1: MB85RS64V FRAM (PA5=SCK, PA6=MISO, PA7=MOSI, PD14=CS) ──
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let mut cs = gpiod.pd14.into_push_pull_output();
    cs.set_high();

    let spi = Spi::new(
        dp.SPI1,
        (Some(gpioa.pa5), Some(gpioa.pa6), Some(gpioa.pa7)),
        Mode { polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition },
        1.MHz(),
        &mut rcc,
    );

    unsafe {
        *addr_of_mut!(G_I2C) = Some(i2c);
        *addr_of_mut!(G_SPI) = Some(spi);
        *addr_of_mut!(G_CS) = Some(cs);
    }

    // ── Kernel setup ──
    let mut sched = ScheduleTable::<{ PipelineCfg::SCHED }>::new();
    // P0 (sensor): 90 ticks — enough for BME280 measure cycle
    sched.add(ScheduleEntry::new(0, 90)).expect("sched P0");
    sched.add_system_window(1).expect("sched SW1");
    // P1 (storage): 10 ticks — SPI FRAM write is fast
    sched.add(ScheduleEntry::new(1, 10)).expect("sched P1");
    sched.add_system_window(1).expect("sched SW2");

    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let flash_mpu = MpuRegion::new(FLASH_BASE, 2 * 1024 * 1024, 0);
    let sram_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);

    // Sampling port IDs: source=0 (P0 writes), dest=1 (P1 reads)
    let src_port: u32 = 0;
    let dst_port: u32 = 1;

    // P0: I2C1 only (GPIO pins configured in main, not accessed at runtime)
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, sensor_main as kernel::PartitionEntry, sram_mpu, kernel::PartitionId::new(0))
        .expect("mem0")
        .with_code_mpu_region(flash_mpu).expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(I2C1_BASE, 1024, 0),
        ]).expect("periph0")
        .with_r0_hint(src_port);

    // P1: SPI1 + GPIOD (CS toggle at runtime; SPI pins configured in main)
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, storage_main as kernel::PartitionEntry, sram_mpu, kernel::PartitionId::new(1))
        .expect("mem1")
        .with_code_mpu_region(flash_mpu).expect("code1")
        .with_peripheral_regions(&[
            MpuRegion::new(SPI1_BASE, 1024, 0),
            MpuRegion::new(GPIOD_BASE, 1024, 0),
        ]).expect("periph1")
        .with_r0_hint(dst_port);

    let mems = [mem0, mem1];
    let mut k = Kernel::<PipelineCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    // Create sampling port pair: P0 writes source, P1 reads destination
    kernel::state::with_kernel_mut::<PipelineCfg, _, _>(|k| {
        let s = k.sampling_mut().create_port(PortDirection::Source, 100).expect("src");
        let d = k.sampling_mut().create_port(PortDirection::Destination, 100).expect("dst");
        k.sampling_mut().connect_ports(s, d).expect("connect");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    match boot(p).expect("boot") {}
}
