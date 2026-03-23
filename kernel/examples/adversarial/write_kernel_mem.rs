//! Adversarial test: unprivileged partition attempts to write kernel memory.
//!
//! This test verifies that the MPU isolation with PRIVDEFENA correctly
//! prevents unprivileged partition code from writing to kernel-owned RAM.
//! The partition attempts to write to address 0x2000_F000, which is outside
//! any allowed MPU region. Expected outcome: MemManage fault with DACCVIOL.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example write_kernel_mem

#![no_std]
#![no_main]

use core::ptr;

use cortex_m_rt::entry;
#[allow(unused_imports)]
use kernel::kpanic as _;

#[path = "mod.rs"]
mod adversarial;

use adversarial::{AlignedStack, FaultInfo};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Known kernel address to attempt to write.
/// LM3S6965 RAM is 0x2000_0000 – 0x2000_FFFF. This address is well outside
/// the partition's allowed stack region.
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Test name for reporting.
const TEST_NAME: &str = "write_kernel_mem";

// ---------------------------------------------------------------------------
// Partition stack and fault capture
// ---------------------------------------------------------------------------

static mut PARTITION_STACK: AlignedStack = AlignedStack::new();
static mut FAULT: FaultInfo = FaultInfo::new();

define_memmanage_handler!(FAULT, TEST_NAME);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");

    // SAFETY: PARTITION_STACK is only accessed here before dropping to
    // unprivileged mode, and by the partition code during the test.
    // The faulting operation deliberately triggers a MemManage fault.
    unsafe {
        adversarial::run_kernel_access_test(
            TEST_NAME,
            KERNEL_ADDR,
            p,
            &raw const PARTITION_STACK,
            || {
                // Attempt to write to kernel memory — must fault (DACCVIOL).
                ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_BEEF);
            },
        )
    }
}
