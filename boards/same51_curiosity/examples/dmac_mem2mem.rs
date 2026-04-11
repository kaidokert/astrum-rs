#![no_std]
#![no_main]

use atsamd_hal as hal;
use same51_curiosity as _;

use hal::clock::GenericClockController;
use hal::dmac::{DmaController, PriorityLevel, Transfer, TriggerAction, TriggerSource};
use hal::pac::Peripherals;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let _clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );

    const LENGTH: usize = 64;
    let src: &'static mut [u8; LENGTH] = cortex_m::singleton!(: [u8; LENGTH] = [0; LENGTH]).unwrap();
    let dst: &'static mut [u8; LENGTH] = cortex_m::singleton!(: [u8; LENGTH] = [0; LENGTH]).unwrap();

    for (i, byte) in src.iter_mut().enumerate() {
        *byte = (i as u8) ^ 0x5A;
    }

    defmt::info!("starting DMAC mem2mem copy of {} bytes", LENGTH);

    let mut pm = peripherals.pm;
    let mut dmac = DmaController::init(peripherals.dmac, &mut pm);
    let mut channels = dmac.split();
    let chan0 = channels.0.init(PriorityLevel::Lvl0);

    let xfer = Transfer::new_from_arrays(chan0, src, dst, false)
        .begin(TriggerSource::Disable, TriggerAction::Block);
    let (chan0, src, dst) = xfer.wait();

    defmt::assert_eq!(&src[..], &dst[..]);
    defmt::info!("DMAC mem2mem copy verified");

    channels.0 = chan0.into();
    let _dmac = dmac.free(channels, &mut pm);

    same51_curiosity::exit()
}
