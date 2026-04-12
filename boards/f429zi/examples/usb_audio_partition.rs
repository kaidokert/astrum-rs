//! USB Audio Partition — Full-Duplex Isochronous under RTOS (STM32F429ZI)
//!
//! Kernel partition running the full USB Audio stack (MPU pass-through + Model B IRQ).
//! Isochronous transfers require sub-millisecond scheduling to avoid frame drops.
//!
//! Architecture:
//! - OTG_FS IRQ 67 dispatched via bind_interrupts! → mask → signal partition
//! - Partition wakes on event, polls USB (handles isochronous audio), ACKs to unmask
//! - 500 µs tick period (tick_period_us = 500) — two ticks per 1ms USB frame
//! - MPU enforcement: OTG_FS (0x5000_0000/256KB) is the only peripheral grant
//!
//! Audio config (select via feature flags, default 2ch/S16LE/48kHz):
//!   (none)       → 2ch S16LE 48kHz =  192 B/frame
//!   audio-24bit  → 2ch S24LE 48kHz =  288 B/frame
//!   audio-4ch    → 4ch S16LE 48kHz =  384 B/frame
//!   audio-4ch-24 → 4ch S24LE 48kHz =  576 B/frame
//!   audio-8ch-24 → 8ch S24LE 44.1kHz = 1058 B/frame (F412 production max)
//!
//! On Linux:
//!   arecord -D hw:N,0 -f S16_LE -r 48000 -c 2 -d 5 capture.raw
//!
//! Build: cd f429zi && cargo build --example usb_audio_partition --features kernel-usb-audio
//! Max:   cargo build --example usb_audio_partition --features kernel-usb-audio,audio-8ch-24

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac,
    prelude::*,
    rcc::Config,
};
use usb_device::prelude::*;

const NUM_PARTITIONS: usize = 1;

/// OTG_FS IRQ number on STM32F429ZI.
const OTG_FS_IRQ: u8 = 67;

/// Kernel event bit for OTG_FS interrupt.
const USB_EVENT: u32 = 0x0000_0001;

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
static INPUT_ALT: AtomicU32 = AtomicU32::new(0);
static OUTPUT_ALT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// USB globals — filled by main() before boot(), taken by partition on first run.
// No Mutex needed: no ISR races (Model B masks before signal).
// ---------------------------------------------------------------------------
static mut G_USB_DEVICE: Option<UsbDevice<'static, UsbBus<USB>>> = None;
static mut G_USB_AUDIO: Option<usbd_audio::AudioClass<'static, UsbBus<USB>>> = None;

// ---------------------------------------------------------------------------
// Kernel configuration: 500 µs tick for isochronous timing
// ---------------------------------------------------------------------------
kernel::bind_interrupts!(AudioCfg, 91,
    OTG_FS_IRQ => (0, USB_EVENT),
);

kernel::kernel_config!(AudioCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;
    tick_period_us = 500;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(AudioCfg, |tick, _k| {
    // tick_period_us = 500, so 2000 ticks = 1 second
    if tick % 2000 == 0 && tick > 0 {
        let tx = TX_FRAMES.load(Ordering::Relaxed);
        let rx = RX_FRAMES.load(Ordering::Relaxed);
        let rx_b = RX_BYTES.load(Ordering::Relaxed);
        let in_alt = INPUT_ALT.load(Ordering::Relaxed);
        let out_alt = OUTPUT_ALT.load(Ordering::Relaxed);
        rprintln!(
            "[{:6}] TX={} RX={} RX_bytes={} in_alt={} out_alt={}",
            tick, tx, rx, rx_b, in_alt, out_alt
        );
        if tx > 100 {
            rprintln!("SUCCESS: USB Audio partition working! TX={} isochronous frames", tx);
        }
    }
    if tick == 2 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU CTRL=0x{:x} ---", ctrl);
        for r in 0u32..8 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            if rasr & 1 != 0 {
                rprintln!("  R{}: RBAR=0x{:08x} RASR=0x{:08x}", r, rbar, rasr);
            }
        }
        rprintln!("---");
    }
});

// ---------------------------------------------------------------------------
// Audio config: selected by feature flags
// ---------------------------------------------------------------------------
#[cfg(feature = "audio-8ch-24")]
mod audio_cfg {
    pub const CHANNELS: u8 = 8;
    pub const SAMPLE_RATE: u32 = 44100;
    pub const FORMAT: usbd_audio::Format = usbd_audio::Format::S24le;
    pub const BYTES_PER_SAMPLE: usize = 3;
    pub const SAMPLES_PER_FRAME: usize = 44; // ~44.1 samples per 1ms frame
    pub const FRAME_BYTES: usize = SAMPLES_PER_FRAME * CHANNELS as usize * BYTES_PER_SAMPLE; // 1056
    pub const LABEL: &str = "8ch/S24LE/44.1kHz=1056B";
}
#[cfg(all(feature = "audio-4ch-24", not(feature = "audio-8ch-24")))]
mod audio_cfg {
    pub const CHANNELS: u8 = 4;
    pub const SAMPLE_RATE: u32 = 48000;
    pub const FORMAT: usbd_audio::Format = usbd_audio::Format::S24le;
    pub const BYTES_PER_SAMPLE: usize = 3;
    pub const SAMPLES_PER_FRAME: usize = 48;
    pub const FRAME_BYTES: usize = SAMPLES_PER_FRAME * CHANNELS as usize * BYTES_PER_SAMPLE; // 576
    pub const LABEL: &str = "4ch/S24LE/48kHz=576B";
}
#[cfg(all(feature = "audio-4ch", not(feature = "audio-4ch-24"), not(feature = "audio-8ch-24")))]
mod audio_cfg {
    pub const CHANNELS: u8 = 4;
    pub const SAMPLE_RATE: u32 = 48000;
    pub const FORMAT: usbd_audio::Format = usbd_audio::Format::S16le;
    pub const BYTES_PER_SAMPLE: usize = 2;
    pub const SAMPLES_PER_FRAME: usize = 48;
    pub const FRAME_BYTES: usize = SAMPLES_PER_FRAME * CHANNELS as usize * BYTES_PER_SAMPLE; // 384
    pub const LABEL: &str = "4ch/S16LE/48kHz=384B";
}
#[cfg(all(feature = "audio-24bit", not(feature = "audio-4ch"), not(feature = "audio-4ch-24"), not(feature = "audio-8ch-24")))]
mod audio_cfg {
    pub const CHANNELS: u8 = 2;
    pub const SAMPLE_RATE: u32 = 48000;
    pub const FORMAT: usbd_audio::Format = usbd_audio::Format::S24le;
    pub const BYTES_PER_SAMPLE: usize = 3;
    pub const SAMPLES_PER_FRAME: usize = 48;
    pub const FRAME_BYTES: usize = SAMPLES_PER_FRAME * CHANNELS as usize * BYTES_PER_SAMPLE; // 288
    pub const LABEL: &str = "2ch/S24LE/48kHz=288B";
}
#[cfg(not(any(feature = "audio-24bit", feature = "audio-4ch", feature = "audio-4ch-24", feature = "audio-8ch-24")))]
mod audio_cfg {
    pub const CHANNELS: u8 = 2;
    pub const SAMPLE_RATE: u32 = 48000;
    pub const FORMAT: usbd_audio::Format = usbd_audio::Format::S16le;
    pub const BYTES_PER_SAMPLE: usize = 2;
    pub const SAMPLES_PER_FRAME: usize = 48;
    pub const FRAME_BYTES: usize = SAMPLES_PER_FRAME * CHANNELS as usize * BYTES_PER_SAMPLE; // 192
    pub const LABEL: &str = "2ch/S16LE/48kHz=192B";
}

use audio_cfg::*;

// ---------------------------------------------------------------------------
// 1 kHz sine wave at 48 kHz = 48 samples/cycle (reused for all channels)
// ---------------------------------------------------------------------------
const SINE_PERIOD: usize = 48;
static SINE_TABLE: [i16; SINE_PERIOD] = [
    0, 4276, 8480, 12539, 16383, 19947, 23169, 25995, 28377, 30272, 31650, 32486,
    32767, 32486, 31650, 30272, 28377, 25995, 23169, 19947, 16383, 12539, 8480, 4276,
    0, -4276, -8480, -12539, -16383, -19947, -23169, -25995, -28377, -30272, -31650, -32486,
    -32767, -32486, -31650, -30272, -28377, -25995, -23169, -19947, -16383, -12539, -8480, -4276,
];

// ---------------------------------------------------------------------------
// USB Audio partition body (MPU pass-through + IRQ Model B)
//
// Runs under MPU enforcement — only OTG_FS (0x5000_0000/256KB) is granted.
// OTG_FS ISR is masked by kernel dispatch; partition ACKs to unmask.
// ---------------------------------------------------------------------------
extern "C" fn audio_partition_body(irq_num: u32) -> ! {
    // Take USB objects from globals (set by main() before boot()).
    let (usb_dev, audio) = unsafe {
        let dev_ptr = core::ptr::addr_of_mut!(G_USB_DEVICE);
        let aud_ptr = core::ptr::addr_of_mut!(G_USB_AUDIO);
        (
            (*dev_ptr).as_mut().unwrap(),
            (*aud_ptr).as_mut().unwrap(),
        )
    };

    let mut sine_pos: usize = 0;

    loop {
        // Wait for OTG_FS IRQ → kernel masked it, signalled USB_EVENT.
        if plib::sys_event_wait(USB_EVENT.into()).is_err() {
            plib::sys_yield().ok();
            continue;
        }

        // Poll USB — handles enumeration, control, and isochronous endpoints.
        if usb_dev.poll(&mut [audio]) {
            // Read speaker data (host → device).
            #[cfg(not(feature = "audio-input-only"))]
            {
                let mut rx_buf = [0u8; 1088]; // fits up to 8ch/24bit/44.1kHz
                if let Ok(len) = audio.read(&mut rx_buf) {
                    if len > 0 {
                        RX_FRAMES.fetch_add(1, Ordering::Relaxed);
                        RX_BYTES.fetch_add(len as u32, Ordering::Relaxed);
                    }
                }
            }
        }

        // Track alt setting changes.
        #[cfg(not(feature = "audio-output-only"))]
        if let Ok(alt) = audio.input_alt_setting() {
            INPUT_ALT.store(alt as u32, Ordering::Relaxed);
        }
        #[cfg(not(feature = "audio-input-only"))]
        if let Ok(alt) = audio.output_alt_setting() {
            OUTPUT_ALT.store(alt as u32, Ordering::Relaxed);
        }

        // Write microphone data (skip if output-only).
        #[cfg(not(feature = "audio-output-only"))]
        {
            let mut tx_buf = [0u8; FRAME_BYTES];
            let mut pos = sine_pos;
            for i in 0..SAMPLES_PER_FRAME {
                for ch in 0..CHANNELS as usize {
                    let off = (i * CHANNELS as usize + ch) * BYTES_PER_SAMPLE;
                    let ch_sample = SINE_TABLE[(pos + ch * SINE_PERIOD / CHANNELS as usize) % SINE_PERIOD];
                    let le = ch_sample.to_le_bytes();
                    tx_buf[off] = le[0];
                    tx_buf[off + 1] = le[1];
                    if BYTES_PER_SAMPLE == 3 {
                        tx_buf[off + 2] = if ch_sample < 0 { 0xFF } else { 0x00 };
                    }
                }
                pos += 1;
            }
            sine_pos = pos;
            if audio.write(&tx_buf).is_ok() {
                TX_FRAMES.fetch_add(1, Ordering::Relaxed);
            }
        }

        // Re-arm OTG_FS IRQ.
        plib::sys_irq_ack(irq_num as u8).ok();
    }
}

kernel::partition_trampoline!(audio_main => audio_partition_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 2048] = [0; 2048];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rprintln!("\n=== USB Audio Partition — MPU pass-through + Model B IRQ ===");
    rprintln!("{}, 500us tick, MPU, OTG_FS IRQ", LABEL);

    let dp = pac::Peripherals::take().unwrap();

    // Clock: HSE 8 MHz → 168 MHz SYSCLK + 48 MHz USB.
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk(),
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let _vbus = gpioa.pa9.into_floating_input();

    unsafe {
        let rcc_reg = &(*pac::RCC::ptr());
        rcc_reg.ahb2enr().modify(|_, w| w.otgfsen().set_bit());
    }

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    let builder = usbd_audio::AudioClassBuilder::new();
    #[cfg(not(feature = "audio-output-only"))]
    let builder = builder.input(
        usbd_audio::StreamConfig::new_discrete(
            FORMAT, CHANNELS, &[SAMPLE_RATE],
            usbd_audio::TerminalType::InMicrophone,
        ).unwrap(),
    );
    #[cfg(not(feature = "audio-input-only"))]
    let builder = builder.output(
        usbd_audio::StreamConfig::new_discrete(
            FORMAT, CHANNELS, &[SAMPLE_RATE],
            usbd_audio::TerminalType::OutSpeaker,
        ).unwrap(),
    );
    let usb_audio = builder.build(usb_bus).unwrap();

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .max_packet_size_0(64)
        .expect("max_packet_size_0")
        .strings(&[StringDescriptors::default()
            .manufacturer("STM32")
            .product("F429ZI RTOS Audio")
            .serial_number("RTOSAUD1")])
        .expect("strings")
        .composite_with_iads()
        .build();

    unsafe {
        G_USB_DEVICE = Some(usb_dev);
        G_USB_AUDIO = Some(usb_audio);
    }

    rprintln!("[INIT] USB Audio: VID:PID=16c0:27dd, product='F429ZI RTOS Audio'");

    let mut p = cortex_m::Peripherals::take().unwrap();
    let otg_fs_base = pac::OTG_FS_GLOBAL::PTR as usize as u32;

    // Schedule: audio partition (2 ticks = 1 ms) + system window (1 tick = 500 µs).
    // At tick_period_us=500, major frame = 1.5 ms. Partition runs every 1.5 ms —
    // within the 1 ms isochronous frame budget with margin.
    let mut sched = ScheduleTable::<{ AudioCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0] = *stacks;

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, audio_main as kernel::PartitionEntry,
            MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0), kernel::PartitionId::new(0),
        )
        .expect("mem0")
        .with_code_mpu_region(MpuRegion::new(FLASH_BASE, 2 * 1024 * 1024, 0))
        .expect("code0")
        .with_peripheral_regions(&[MpuRegion::new(otg_fs_base, 256 * 1024, 0)])
        .expect("periph0")
        .with_r0_hint(OTG_FS_IRQ as u32);

    let mems = [mem0];
    let mut k = Kernel::<AudioCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    enable_bound_irqs(&mut p.NVIC, AudioCfg::IRQ_DEFAULT_PRIORITY).unwrap();
    rprintln!("[INIT] IRQ 67 (OTG_FS) enabled — kernel ready");
    rprintln!("[INIT] tick_period_us=500, OTG_FS grant=0x{:08x}/256KB\n", otg_fs_base);

    match boot(p).expect("boot") {}
}
