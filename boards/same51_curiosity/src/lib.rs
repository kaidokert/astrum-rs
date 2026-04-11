#![no_main]
#![no_std]

pub mod lcd;

#[cfg(not(feature = "uses-kernel"))]
use defmt_rtt as _; // global logger

use atsamd_hal as _; // memory layout

#[allow(unused_imports)]
#[cfg(feature = "uses-kernel")]
use kernel::kpanic as _;

#[allow(unused_imports)]
#[cfg(not(feature = "uses-kernel"))]
use panic_probe as _;

pub const CORE_CLOCK_HZ: u32 = 120_000_000;

// ATSAME51J20A memory map (from memory.x)
pub const FLASH_BASE: u32 = 0x0000_0000;
pub const FLASH_SIZE: u32 = 0x0010_0000; // 1 MB
pub const SRAM_BASE: u32 = 0x2000_0000;
pub const SRAM_SIZE: u32 = 256 * 1024; // 256 KB

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes a semihosting-capable debug tool exit
/// with status code 0.
pub fn exit() -> ! {
    semihosting::process::exit(0);
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    semihosting::process::exit(1);
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
