//! USB Audio Class — Full-Duplex Microphone + Speaker (Bare-Metal)
//!
//! Enumerates as a USB Audio device with:
//! - Input (microphone): 48 kHz, mono, 16-bit — sends a 1 kHz sine wave to the host
//! - Output (speaker):   48 kHz, mono, 16-bit — receives audio from the host, counts frames
//!
//! Isochronous transfers: guaranteed bandwidth, periodic (every 1ms frame), no retries.
//! Unlike bulk transfers (USB CDC), isochronous sacrifices reliability for timing —
//! a missed frame is dropped, not retransmitted.
//!
//! On Linux, the device appears as an ALSA capture+playback device:
//!   arecord -D hw:N,0 -f S16_LE -r 48000 -c 1 /dev/null   # capture from "microphone"
//!   aplay -D hw:N,0 -f S16_LE -r 48000 -c 1 test.raw       # play to "speaker"
//!   arecord -D hw:N,0 -f S16_LE -r 48000 -c 1 -d 5 out.raw && xxd out.raw | head
//!
//! Hardware: STM32F429ZI NUCLEO-144, USB OTG_FS on PA11/PA12.
//! Clock: HSE 8 MHz → PLL → SYSCLK 168 MHz, PLL48CLK 48 MHz (USB requirement).
//!
//! Build: cd f429zi && cargo build --example usb_audio --features usb-audio

#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac::{interrupt, Interrupt},
    prelude::*,
    rcc::Config,
};
use usb_device::prelude::*;

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
static INPUT_ALT: AtomicU32 = AtomicU32::new(0);
static OUTPUT_ALT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// USB globals — moved into ISR on first call (same pattern as usb_cdc.rs)
// ---------------------------------------------------------------------------
static G_USB_AUDIO: cortex_m::interrupt::Mutex<
    core::cell::RefCell<Option<usbd_audio::AudioClass<UsbBus<USB>>>>,
> = cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(None));

static G_USB_DEVICE: cortex_m::interrupt::Mutex<
    core::cell::RefCell<Option<UsbDevice<'static, UsbBus<USB>>>>,
> = cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(None));

// ---------------------------------------------------------------------------
// 1 kHz sine wave at 48 kHz sample rate = 48 samples per cycle.
// 16-bit signed, little-endian. One full cycle, looped by the ISR.
// ---------------------------------------------------------------------------
const SINE_SAMPLES: usize = 48;
static SINE_TABLE: [i16; SINE_SAMPLES] = [
    0, 4276, 8480, 12539, 16383, 19947, 23169, 25995, 28377, 30272, 31650, 32486,
    32767, 32486, 31650, 30272, 28377, 25995, 23169, 19947, 16383, 12539, 8480, 4276,
    0, -4276, -8480, -12539, -16383, -19947, -23169, -25995, -28377, -30272, -31650, -32486,
    -32767, -32486, -31650, -30272, -28377, -25995, -23169, -19947, -16383, -12539, -8480, -4276,
];

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rtt_init_print!();
    rprintln!("\n=== USB Audio Class — Full-Duplex Bare-Metal ===");
    rprintln!("Input:  48 kHz mono S16LE (1 kHz sine → host)");
    rprintln!("Output: 48 kHz mono S16LE (host → device, frame count)");

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    // Clock: HSE 8 MHz → 168 MHz SYSCLK + 48 MHz USB clock.
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk(),
    );

    let gpioa = dp.GPIOA.split(&mut rcc);

    // USB GPIO: PA11 = DM, PA12 = DP, PA9 = VBUS sense.
    let _vbus = gpioa.pa9.into_floating_input();

    // Enable OTG_FS clock explicitly (HAL doesn't always do this).
    unsafe {
        let rcc_reg = &(*stm32f4xx_hal::pac::RCC::ptr());
        rcc_reg.ahb2enr().modify(|_, w| w.otgfsen().set_bit());
    }

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    // Build audio class: input (microphone) + output (speaker), both 48 kHz mono 16-bit.
    // 48 kHz mono S16LE = 96 bytes per 1ms frame (48 samples * 2 bytes).
    // OTG_FS can do up to 192 bytes/frame (3 packets * 64), so 96 is well within limits.
    let usb_audio = usbd_audio::AudioClassBuilder::new()
        .input(
            usbd_audio::StreamConfig::new_discrete(
                usbd_audio::Format::S16le,
                1,          // mono
                &[48000],   // 48 kHz
                usbd_audio::TerminalType::InMicrophone,
            )
            .unwrap(),
        )
        .output(
            usbd_audio::StreamConfig::new_discrete(
                usbd_audio::Format::S16le,
                1,          // mono
                &[48000],   // 48 kHz
                usbd_audio::TerminalType::OutSpeaker,
            )
            .unwrap(),
        )
        .build(usb_bus)
        .unwrap();

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .max_packet_size_0(64)
        .expect("max_packet_size_0")
        .strings(&[StringDescriptors::default()
            .manufacturer("STM32")
            .product("F429ZI Audio")
            .serial_number("AUDIO001")])
        .expect("strings")
        .composite_with_iads()
        .build();

    cortex_m::interrupt::free(|cs| {
        *G_USB_AUDIO.borrow(cs).borrow_mut() = Some(usb_audio);
        *G_USB_DEVICE.borrow(cs).borrow_mut() = Some(usb_dev);
    });

    // Enable OTG_FS interrupt — all USB handling happens in the ISR.
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS) };

    rprintln!("[INIT] USB Audio device ready. Waiting for host enumeration...");
    rprintln!("[INIT] On Linux: arecord -D hw:N,0 -f S16_LE -r 48000 -c 1 out.raw");

    // Main loop: report stats via RTT.
    loop {
        cortex_m::asm::delay(168_000_000); // ~1s at 168 MHz
        let tx = TX_FRAMES.load(Ordering::Relaxed);
        let rx = RX_FRAMES.load(Ordering::Relaxed);
        let rx_b = RX_BYTES.load(Ordering::Relaxed);
        let in_alt = INPUT_ALT.load(Ordering::Relaxed);
        let out_alt = OUTPUT_ALT.load(Ordering::Relaxed);
        rprintln!(
            "[AUDIO] TX_frames={} RX_frames={} RX_bytes={} in_alt={} out_alt={}",
            tx, rx, rx_b, in_alt, out_alt
        );
    }
}

// ---------------------------------------------------------------------------
// OTG_FS ISR — polls USB device, handles isochronous audio data.
//
// Isochronous transfer properties:
// - Guaranteed bandwidth: host reserves 96 bytes/frame (48 kHz mono S16LE)
// - Periodic: called every 1ms USB frame (Full Speed)
// - No retries: if device misses a frame, that audio is lost (glitch)
// - No handshake: device sends/receives data, no ACK/NAK protocol
// ---------------------------------------------------------------------------
#[interrupt]
fn OTG_FS() {
    // #[interrupt] body is implicitly unsafe — static mut access is fine here.
    static mut USB_AUDIO: Option<usbd_audio::AudioClass<UsbBus<USB>>> = None;
    static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus<USB>>> = None;
    static mut SINE_POS: usize = 0;

    let audio = USB_AUDIO.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_USB_AUDIO.borrow(cs).replace(None).unwrap())
    });

    let usb_dev = USB_DEVICE.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_USB_DEVICE.borrow(cs).replace(None).unwrap())
    });

    // Poll USB — handles enumeration, control transfers, and data endpoints.
    if usb_dev.poll(&mut [audio]) {
        // Read speaker data (host → device) if available.
        let mut rx_buf = [0u8; 192];
        if let Ok(len) = audio.read(&mut rx_buf) {
            if len > 0 {
                RX_FRAMES.fetch_add(1, Ordering::Relaxed);
                RX_BYTES.fetch_add(len as u32, Ordering::Relaxed);
            }
        }
    }

    // Track alt setting changes (0 = inactive, 1 = streaming).
    if let Ok(alt) = audio.input_alt_setting() {
        INPUT_ALT.store(alt as u32, Ordering::Relaxed);
    }
    if let Ok(alt) = audio.output_alt_setting() {
        OUTPUT_ALT.store(alt as u32, Ordering::Relaxed);
    }

    // Write microphone data (device → host): one frame of sine wave.
    // 48 kHz mono S16LE = 48 samples * 2 bytes = 96 bytes per 1ms frame.
    // SINE_POS is &mut usize (from #[interrupt] implicit unsafe desugar).
    let mut tx_buf = [0u8; 96];
    let mut pos: usize = *SINE_POS;
    for i in 0..48 {
        let sample = SINE_TABLE[pos];
        let le = sample.to_le_bytes();
        tx_buf[i * 2] = le[0];
        tx_buf[i * 2 + 1] = le[1];
        pos = (pos + 1) % SINE_SAMPLES;
    }
    *SINE_POS = pos;
    if audio.write(&tx_buf).is_ok() {
        TX_FRAMES.fetch_add(1, Ordering::Relaxed);
    }
}
