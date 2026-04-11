#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;

use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac::{interrupt, Interrupt},
    prelude::*,
    gpio::{Output, PushPull, Pin},
    rcc::Config,
};

use usb_device::prelude::*;
use usbd_serial::SerialPort;

// Atomic counters
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
static UPTIME_SECONDS: AtomicU32 = AtomicU32::new(0);

// Global USB device and serial port
static G_USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));
static G_USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

// Command buffer for receiving commands
static CMD_BUFFER: Mutex<RefCell<CommandBuffer>> =
    Mutex::new(RefCell::new(CommandBuffer::new()));

// LED state
static LED_STATE: Mutex<RefCell<Option<Pin<'A', 5, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

struct CommandBuffer {
    buf: [u8; 128],
    pos: usize,
}

impl CommandBuffer {
    const fn new() -> Self {
        Self {
            buf: [0u8; 128],
            pos: 0,
        }
    }

    fn push(&mut self, byte: u8) -> bool {
        if byte == b'\n' || byte == b'\r' {
            true // Command complete
        } else if self.pos < self.buf.len() {
            self.buf[self.pos] = byte;
            self.pos += 1;
            false
        } else {
            false // Buffer full
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap_or("")
    }

    fn clear(&mut self) {
        self.pos = 0;
    }
}

// Simple string buffer for formatting responses
struct StrBuf {
    buf: [u8; 256],
    len: usize,
}

impl StrBuf {
    fn new() -> Self {
        Self {
            buf: [0u8; 256],
            len: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl Write for StrBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.len;
        let to_copy = bytes.len().min(remaining);
        self.buf[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
        Ok(())
    }
}

fn process_command(cmd: &str) -> StrBuf {
    let mut response = StrBuf::new();
    let cmd = cmd.trim();

    match cmd {
        "help" => {
            let _ = writeln!(response, "Available commands:");
            let _ = writeln!(response, "  help          - Show this help");
            let _ = writeln!(response, "  led on        - Turn LED on");
            let _ = writeln!(response, "  led off       - Turn LED off");
            let _ = writeln!(response, "  led toggle    - Toggle LED");
            let _ = writeln!(response, "  status        - Show device status");
            let _ = writeln!(response, "  echo <text>   - Echo back text");
        }
        "led on" => {
            cortex_m::interrupt::free(|cs| {
                if let Some(led) = LED_STATE.borrow(cs).borrow_mut().as_mut() {
                    led.set_high();
                    let _ = writeln!(response, "LED: ON");
                }
            });
        }
        "led off" => {
            cortex_m::interrupt::free(|cs| {
                if let Some(led) = LED_STATE.borrow(cs).borrow_mut().as_mut() {
                    led.set_low();
                    let _ = writeln!(response, "LED: OFF");
                }
            });
        }
        "led toggle" => {
            cortex_m::interrupt::free(|cs| {
                if let Some(led) = LED_STATE.borrow(cs).borrow_mut().as_mut() {
                    led.toggle();
                    let state = if led.is_set_high() { "ON" } else { "OFF" };
                    let _ = writeln!(response, "LED: {}", state);
                }
            });
        }
        "status" => {
            let irq = IRQ_COUNT.load(Ordering::Relaxed);
            let uptime = UPTIME_SECONDS.load(Ordering::Relaxed);
            let _ = writeln!(response, "Device: STM32F429ZI");
            let _ = writeln!(response, "Uptime: {}s", uptime);
            let _ = writeln!(response, "IRQs: {}", irq);
        }
        _ if cmd.starts_with("echo ") => {
            let text = &cmd[5..];
            let _ = writeln!(response, "{}", text);
        }
        "" => {
            // Empty command, no response
        }
        _ => {
            let _ = writeln!(response, "Unknown command: '{}'", cmd);
            let _ = writeln!(response, "Type 'help' for available commands");
        }
    }

    response
}

#[entry]
fn main() -> ! {
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

    rtt_init_print!();
    rprintln!("STM32F429ZI USB Command Interface");

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk()
    );

    // Configure GPIO
    let gpioa = dp.GPIOA.split(&mut rcc);
    let led = gpioa.pa5.into_push_pull_output();

    // Store LED in global for command access
    cortex_m::interrupt::free(|cs| {
        *LED_STATE.borrow(cs).borrow_mut() = Some(led);
    });

    // USB GPIO configuration
    let _vbus = gpioa.pa9.into_floating_input();
    let _id = gpioa.pa10.into_alternate::<10>().internal_pull_up(true);

    // Enable USB OTG_FS clock
    unsafe {
        let rcc = &(*stm32f4xx_hal::pac::RCC::ptr());
        rcc.ahb2enr().modify(|_, w| w.otgfsen().set_bit());
    }

    // Initialize USB peripheral
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );

    // Create USB bus allocator
    *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();

    // Create USB device and serial port
    cortex_m::interrupt::free(|cs| {
        *G_USB_SERIAL.borrow(cs).borrow_mut() = Some(SerialPort::new(usb_bus));

        *G_USB_DEVICE.borrow(cs).borrow_mut() = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(usbd_serial::USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("STM32")
                    .product("F429 Command Interface")
                    .serial_number("CMD001")])
                .expect("Failed to set strings")
                .build(),
        );
    });

    // Enable USB interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
    }

    rprintln!("USB initialized. Send 'help' for commands.");

    // Main loop - update uptime counter
    let mut counter = 0u32;

    loop {
        cortex_m::asm::delay(16_800_000); // ~100ms at 168MHz
        counter += 1;
        if counter % 10 == 0 {
            UPTIME_SECONDS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[interrupt]
fn OTG_FS() {
    IRQ_COUNT.fetch_add(1, Ordering::Relaxed);

    static mut USB_SERIAL: Option<SerialPort<UsbBus<USB>>> = None;
    static mut USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;

    // Move USB objects from global to interrupt-local on first call
    let usb_dev = unsafe {
        let ptr = core::ptr::addr_of_mut!(USB_DEVICE);
        (*ptr).get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| G_USB_DEVICE.borrow(cs).replace(None).unwrap())
        })
    };

    let serial = unsafe {
        let ptr = core::ptr::addr_of_mut!(USB_SERIAL);
        (*ptr).get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| G_USB_SERIAL.borrow(cs).replace(None).unwrap())
        })
    };

    // Poll USB and handle data
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        if let Ok(count) = serial.read(&mut buf) {
            // Process received bytes
            for i in 0..count {
                let byte = buf[i];

                let cmd_complete = cortex_m::interrupt::free(|cs| {
                    CMD_BUFFER.borrow(cs).borrow_mut().push(byte)
                });

                if cmd_complete {
                    // Process command
                    let response = cortex_m::interrupt::free(|cs| {
                        let cmd_buf = CMD_BUFFER.borrow(cs);
                        let binding = cmd_buf.borrow();
                        let cmd = binding.as_str();
                        let resp = process_command(cmd);
                        drop(binding);
                        cmd_buf.borrow_mut().clear();
                        resp
                    });

                    // Send response
                    let resp_bytes = response.as_bytes();
                    let mut wr = 0;
                    while wr < resp_bytes.len() {
                        match serial.write(&resp_bytes[wr..]) {
                            Ok(len) => wr += len,
                            Err(_) => break,
                        }
                    }
                }
            }
        }
    }
}
