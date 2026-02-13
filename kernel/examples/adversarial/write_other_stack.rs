//! Adversarial test: partition 0 attempts to write to partition 1's stack region.
//!
//! This test verifies that MPU isolation prevents unprivileged partition code
//! from writing to another partition's data region. With PRIVDEFENA and
//! separate MPU data regions per partition, an attempt by partition 0 to write
//! an address within partition 1's stack must generate a MemManage fault.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example write_other_stack

#![no_std]
#![no_main]

use core::ptr;

use cortex_m_rt::entry;
use panic_semihosting as _;

#[path = "mod.rs"]
mod adversarial;

use adversarial::{AlignedStack, FaultInfo};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Test name for reporting.
const TEST_NAME: &str = "write_other_stack";

// ---------------------------------------------------------------------------
// Partition stacks and fault capture
// ---------------------------------------------------------------------------

/// Partition 0 stack (adversarial partition that attempts the write).
static mut P0_STACK: AlignedStack = AlignedStack::new();

/// Partition 1 stack (victim partition whose memory should be protected).
static mut P1_STACK: AlignedStack = AlignedStack::new();

static mut FAULT: FaultInfo = FaultInfo::new();

define_memmanage_handler!(FAULT, TEST_NAME);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");

    // SAFETY: P0_STACK and P1_STACK are only accessed here before dropping to
    // unprivileged mode, and by the partition code during the test.
    // The write operation deliberately triggers a MemManage fault.
    unsafe {
        adversarial::run_other_stack_test(
            TEST_NAME,
            &mut p,
            &raw const P0_STACK,
            &raw const P1_STACK,
            |target_addr| {
                // Attempt to write to partition 1's stack — must fault (DACCVIOL).
                ptr::write_volatile(target_addr as *mut u32, 0xDEAD_BEEF);
            },
        )
    }
}
