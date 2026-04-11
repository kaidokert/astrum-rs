#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use atsamd_hal as hal;
use same51_curiosity as _;

use cortex_m::peripheral::NVIC;
use hal::clock::GenericClockController;
use hal::pac::gclk::{genctrl::Srcselect, pchctrl::Genselect};
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );

    let pins = hal::gpio::Pins::new(peripherals.port);
    clocks.configure_gclk_divider_and_source(Genselect::Gclk2, 1, Srcselect::Dfll, false);
    let usb_gclk = clocks.get_gclk(Genselect::Gclk2).unwrap();
    let usb_clock = clocks.usb(&usb_gclk).unwrap();

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
            &usb_clock,
            &mut peripherals.mclk,
            pins.pa24,
            pins.pa25,
            peripherals.usb,
        )));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .strings(&[StringDescriptors::new(LangID::EN)
                    .manufacturer("Kaido")
                    .product("SAME51 CDC IRQ")
                    .serial_number("same51-curiosity-irq")])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB_OTHER, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT0, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT1, 1);
        NVIC::unmask(interrupt::USB_OTHER);
        NVIC::unmask(interrupt::USB_TRCPT0);
        NVIC::unmask(interrupt::USB_TRCPT1);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

fn poll_usb() {
    unsafe {
        if let Some(usb_dev) = USB_BUS.as_mut() {
            if let Some(serial) = USB_SERIAL.as_mut() {
                if !usb_dev.poll(&mut [serial]) {
                    return;
                }

                let mut buf = [0u8; 64];
                if let Ok(count) = serial.read(&mut buf) {
                    if count != 0 {
                        let _ = serial.write(&buf[..count]);
                    }
                }
            }
        }
    }
}

#[interrupt]
fn USB_OTHER() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT0() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT1() {
    poll_usb();
}
