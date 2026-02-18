//! Test that `use kernel::kpanic as _;` works uniformly across backends.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
#[allow(unused_imports)]
use kernel::kpanic as _;

#[entry]
fn main() -> ! {
    // If we got here, the panic handler was linked successfully
    debug::exit(debug::EXIT_SUCCESS);
    #[allow(clippy::empty_loop)]
    loop {}
}
