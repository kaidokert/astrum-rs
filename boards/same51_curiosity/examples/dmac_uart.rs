#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use hal::clock::GenericClockController;
use hal::dmac::{DmaController, PriorityLevel};
use hal::fugit::RateExtU32;
use hal::pac::Peripherals;
use hal::sercom::{uart, Sercom2};

type VcomPads = uart::Pads<
    Sercom2,
    hal::typelevel::NoneT,
    hal::gpio::Pin<hal::gpio::PA12, hal::gpio::AlternateC>,
>;
type VcomUart = uart::Uart<uart::Config<VcomPads>, uart::Tx>;
type TxChan = hal::dmac::Channel<hal::dmac::Ch0, hal::dmac::Ready>;

const MSG_LEN: usize = 26;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );

    let mut dmac = DmaController::init(peripherals.dmac, &mut peripherals.pm);
    let channels = dmac.split();
    let chan0 = channels.0.init(PriorityLevel::Lvl0);

    let pins = hal::gpio::Pins::new(peripherals.port);
    let gclk0 = clocks.gclk0();
    let pads = uart::Pads::<Sercom2>::default().tx(pins.pa12);
    let mut tx: VcomUart = uart::Config::new(
        &mut peripherals.mclk,
        peripherals.sercom2,
        pads,
        clocks.sercom2_core(&gclk0).unwrap().freq(),
    )
    .baud(
        115_200.Hz(),
        uart::BaudMode::Fractional(uart::Oversampling::Bits16),
    )
    .enable();
    let mut tx_buffer: &'static mut [u8; MSG_LEN] =
        cortex_m::singleton!(: [u8; MSG_LEN] = *b"same51 dmac uart alive\r\n\r\n").unwrap();

    let mut chan: TxChan = chan0;

    loop {
        let transfer = tx.send_with_dma(tx_buffer, chan);
        let (next_chan, next_buffer, next_tx) = transfer.wait();
        chan = next_chan;
        tx_buffer = next_buffer;
        tx = next_tx;
    }
}
