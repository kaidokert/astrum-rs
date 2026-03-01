//! Stub example verifying adversarial test infrastructure compiles.

#![no_std]
#![no_main]

#[path = "adversarial/mod.rs"]
mod adversarial;

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;

use adversarial::{AdversarialMpuConfig, FaultInfo, FaultOutcome};

static mut FAULT: FaultInfo = FaultInfo::new();

define_memmanage_handler!(FAULT, {
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
});

#[entry]
fn main() -> ! {
    hprintln!("adversarial_mod_stub: start");

    // Verify FaultOutcome derives (Debug, Clone, Copy, PartialEq)
    let f = FaultOutcome::Faulted {
        mmfsr: 0x82,
        mmfar: 0x2000_F000,
    };
    let _ = Clone::clone(&f); // Clone
    let c = f; // Copy
    assert_eq!(f, c); // PartialEq
    hprintln!("  {:?}", f); // Debug

    // Verify FaultInfo and AdversarialMpuConfig compile
    let _ = FaultInfo::new().to_outcome();
    let _ = AdversarialMpuConfig::new(32 * 1024, 1024, 0x2000_0000, 0x2000_0400);

    hprintln!("adversarial_mod_stub: PASS");
    debug::exit(debug::EXIT_SUCCESS);
    loop {
        asm::wfi();
    }
}
