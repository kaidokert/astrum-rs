#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

#[cfg(feature = "pac-rt")]
compile_error!("example `vcom` requires custom-ivt mode and must not be built with `pac-rt`");

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use atsamd_hal as hal;
use atsamd_hal::embedded_io::Write as _;
use cortex_m_rt::{entry, exception};
use critical_section::Mutex;
use embedded_hal_02::serial::{Read as _, Write as EhWrite};
use hal::clock::GenericClockController;
use hal::fugit::RateExtU32;
use hal::pac;
use hal::sercom::{uart, Sercom2};
use kernel::{
    DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal,
    partition::PartitionConfig,
    PartitionSpec,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
};
use plib::{define_partition_debug, dprint};
use same51_curiosity as _;

const UART_RX_IRQ: u8 = 56;
const UART_RX_EVENT: u32 = 0x0000_0001;
const IRQ_COUNT: usize = 136;

type VcomPads = uart::PadsFromIds<Sercom2, hal::gpio::PA13, hal::gpio::PA12>;
type VcomConfig = uart::Config<VcomPads>;
type VcomUart = uart::Uart<VcomConfig, uart::Duplex>;

static RX_BYTES: AtomicU32 = AtomicU32::new(0);
static TX_BYTES: AtomicU32 = AtomicU32::new(0);
static UART: Mutex<RefCell<Option<VcomUart>>> = Mutex::new(RefCell::new(None));

define_partition_debug!(P0_DEBUG, 256);

kernel::kernel_config!(Same51VcomCfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
});

kernel::bind_interrupts!(Same51VcomCfg, IRQ_COUNT,
    UART_RX_IRQ => (0, UART_RX_EVENT),
);

kernel::define_kernel!(Same51VcomCfg, |tick, _k| {
    if tick % 1000 == 0 {
        kernel::klog!(
            "[{:5}ms] same51 astrum vcom: rx={} tx={}",
            tick,
            RX_BYTES.load(Ordering::Acquire),
            TX_BYTES.load(Ordering::Acquire),
        );
    }
});

fn store_uart(uart: VcomUart) {
    critical_section::with(|cs| {
        let replaced = UART.borrow(cs).borrow_mut().replace(uart);
        debug_assert!(replaced.is_none());
    });
}

fn take_uart() -> VcomUart {
    critical_section::with(|cs| UART.borrow(cs).borrow_mut().take()).expect("uart")
}

fn clear_error_state(uart: &mut VcomUart) {
    if uart.read_flags().contains(uart::Flags::ERROR) {
        uart.flush_rx_buffer();
        uart.clear_flags(uart::Flags::ERROR);
    }
}

fn init_vcom_uart(
    port: pac::Port,
    sercom2: pac::Sercom2,
    mclk: &mut pac::Mclk,
    clocks: &mut GenericClockController,
) -> VcomUart {
    let pins = hal::gpio::Pins::new(port);
    let gclk0 = clocks.gclk0();
    let pads = uart::Pads::<Sercom2>::default().rx(pins.pa13).tx(pins.pa12);
    let mut uart = uart::Config::new(
        mclk,
        sercom2,
        pads,
        clocks.sercom2_core(&gclk0).unwrap().freq(),
    )
    .baud(
        115_200.Hz(),
        uart::BaudMode::Fractional(uart::Oversampling::Bits16),
    )
    .enable();

    uart.clear_flags(uart::Flags::RXC | uart::Flags::ERROR);
    uart.clear_status(
        uart::Status::PERR
            | uart::Status::FERR
            | uart::Status::BUFOVF
            | uart::Status::ISF
            | uart::Status::COLL,
    );
    uart.enable_interrupts(uart::Flags::RXC);
    uart
}

extern "C" fn uart_partition_body(uart_irq: u32) -> ! {
    let mut uart = take_uart();
    const BANNER: &[u8] = b"same51 astrum vcom ready\r\n";

    uart.write_all(BANNER).expect("uart tx");
    TX_BYTES.fetch_add(BANNER.len() as u32, Ordering::Release);
    let _ = dprint!(&P0_DEBUG, "vcom partition booted");

    loop {
        if plib::sys_event_wait(UART_RX_EVENT.into()).is_err() {
            plib::sys_yield().ok();
            continue;
        }

        if !uart.read_flags().contains(uart::Flags::RXC) {
            clear_error_state(&mut uart);
            plib::sys_event_clear(UART_RX_EVENT.into()).ok();
            plib::sys_irq_ack(uart_irq as u8).ok();
            plib::sys_yield().ok();
            continue;
        }

        while uart.read_flags().contains(uart::Flags::RXC) {
            let byte = nb::block!(uart.read()).expect("uart rx");
            RX_BYTES.fetch_add(1, Ordering::Release);
            nb::block!(EhWrite::write(&mut uart, byte)).expect("uart tx");
            TX_BYTES.fetch_add(1, Ordering::Release);
        }

        clear_error_state(&mut uart);
        plib::sys_event_clear(UART_RX_EVENT.into()).ok();
        plib::sys_irq_ack(uart_irq as u8).ok();
    }
}

kernel::partition_trampoline!(uart_partition => uart_partition_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    let pac::Peripherals {
        gclk,
        mut mclk,
        mut osc32kctrl,
        mut oscctrl,
        mut nvmctrl,
        port,
        sercom2,
        ..
    } = pac::Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        gclk,
        &mut mclk,
        &mut osc32kctrl,
        &mut oscctrl,
        &mut nvmctrl,
    );

    let uart = init_vcom_uart(port, sercom2, &mut mclk, &mut clocks);
    store_uart(uart);

    let mut sched = ScheduleTable::<{ Same51VcomCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");

    let cfgs: [PartitionConfig; 1] = core::array::from_fn(|i| {
        PartitionConfig::sentinel(i as u8, (Same51VcomCfg::STACK_WORDS * 4) as u32)
    });

    let mut k = Kernel::<Same51VcomCfg>::new(sched, &cfgs).expect("kernel");
    k.partitions_mut()
        .get_mut(0)
        .expect("partition")
        .set_debug_buffer(&P0_DEBUG);
    store_kernel(&mut k);

    enable_bound_irqs(&mut p.NVIC, Same51VcomCfg::IRQ_DEFAULT_PRIORITY).expect("bind irqs");

    let parts: [PartitionSpec; 1] = [PartitionSpec::new(uart_partition as kernel::PartitionEntry, UART_RX_IRQ as u32)];
    match boot(p).expect("boot") {}
}
