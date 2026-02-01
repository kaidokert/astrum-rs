#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

#[entry]
fn main() -> ! {
    hprintln!("hello from kernel");
    debug::exit(debug::EXIT_SUCCESS);
    loop {
        asm::wfi();
    }
}
