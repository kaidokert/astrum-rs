#![no_std]
#![no_main]

use cortex_m_rt::entry;
use kernel as _;

#[entry]
fn main() -> ! {
    panic!("intentional panic test");
}
