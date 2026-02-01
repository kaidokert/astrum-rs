#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_semihosting as _;

#[entry]
fn main() -> ! {
    panic!("intentional panic test");
}
