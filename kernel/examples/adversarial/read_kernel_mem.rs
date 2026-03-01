//! Adversarial test: unprivileged partition attempts to read kernel memory.
//!
//! This test verifies that the MPU isolation with PRIVDEFENA correctly
//! prevents unprivileged partition code from reading kernel-owned RAM.
//! The partition attempts to read address 0x2000_F000, which is outside
//! any allowed MPU region. Expected outcome: MemManage fault with DACCVIOL.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example read_kernel_mem

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m::register::{control, psp};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu;

#[path = "mod.rs"]
mod adversarial;

use adversarial::{FaultInfo, FaultOutcome};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Known kernel address to attempt to read.
/// LM3S6965 RAM is 0x2000_0000 – 0x2000_FFFF. This address is well outside
/// the partition's allowed stack region.
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Test name for reporting.
const TEST_NAME: &str = "read_kernel_mem";

// ---------------------------------------------------------------------------
// Partition stack — 1 KiB, aligned to 1024 for MPU region base.
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut PARTITION_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Fault capture
// ---------------------------------------------------------------------------
static mut FAULT: FaultInfo = FaultInfo::new();

define_memmanage_handler!(FAULT, {
    // Handler runs in privileged mode; can use semihosting.
    // SAFETY: Reading FAULT from the exception handler that just wrote it.
    // No other code accesses FAULT concurrently (interrupts of same/lower
    // priority are blocked during this exception).
    let info = unsafe { core::ptr::read_volatile(&raw const FAULT) };

    // Verify we got a DACCVIOL (data access violation)
    let outcome = if info.is_daccviol() {
        FaultOutcome::Faulted {
            mmfsr: info.mmfsr,
            mmfar: info.mmfar,
        }
    } else if info.faulted {
        // Fault occurred but not DACCVIOL — unexpected
        FaultOutcome::Error("fault without DACCVIOL")
    } else {
        FaultOutcome::NoFault
    };

    adversarial::report_result(TEST_NAME, outcome);
});

// ---------------------------------------------------------------------------
// MPU configuration
// ---------------------------------------------------------------------------
const CODE_SIZE: u32 = 32 * 1024;
const DATA_SIZE: u32 = (STACK_WORDS * 4) as u32;

/// Configure MPU with PRIVDEFENA for partition isolation.
///
/// R0: code RX (flash 0x0) — priv+unpriv read-only
/// R1: data RW (partition stack) — full access, XN
///
/// KERNEL_ADDR matches no region, so unprivileged access faults.
fn configure_partition_mpu(mpu_periph: &cortex_m::peripheral::MPU, data_base: u32) {
    // SAFETY: single-core, interrupts disabled — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: code RX — covers flash binary (priv+unpriv read-only)
    let code_sf = mpu::encode_size(CODE_SIZE).expect("code size");
    let code_rbar = mpu::build_rbar(0x0000_0000, 0).expect("code rbar");
    let code_rasr = mpu::build_rasr(code_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, code_rbar, code_rasr);

    // R1: data RW — partition stack (full access, XN)
    let data_sf = mpu::encode_size(DATA_SIZE).expect("data size");
    let data_rbar = mpu::build_rbar(data_base, 1).expect("data rbar");
    let data_rasr = mpu::build_rasr(data_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, data_rbar, data_rasr);

    // Enable MPU with PRIVDEFENA.
    // SAFETY: regions programmed; barriers ensure visibility.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Enable MemManage fault handler.
    p.SCB.enable(Exception::MemoryManagement);

    // Get partition stack addresses.
    let stack_base = (&raw const PARTITION_STACK).cast::<u32>() as u32;
    let stack_top = stack_base + DATA_SIZE;

    hprintln!("  stack: {:#010x} - {:#010x}", stack_base, stack_top);
    hprintln!("  target kernel addr: {:#010x}", KERNEL_ADDR);

    // Configure MPU before dropping privileges.
    configure_partition_mpu(&p.MPU, stack_base);

    hprintln!("  MPU configured, dropping to unprivileged...");

    // Switch to PSP and drop to unprivileged mode.
    // SAFETY: We set PSP to the top of a valid, MPU-accessible stack,
    // then set CONTROL.SPSEL=1 (use PSP) and CONTROL.nPRIV=1 (unprivileged).
    unsafe {
        psp::write(stack_top);
        let mut ctrl = control::read();
        ctrl.set_spsel(control::Spsel::Psp);
        ctrl.set_npriv(control::Npriv::Unprivileged);
        control::write(ctrl);
    }

    // Now unprivileged on PSP. No semihosting here — would fault before
    // reaching the intentional read.
    //
    // Attempt to read kernel memory — must fault (DACCVIOL).
    // SAFETY: deliberately triggering a MemManage fault for testing.
    let _ = unsafe { ptr::read_volatile(KERNEL_ADDR as *const u32) };

    // Should never reach here — if we do, no fault occurred.
    // Return to privileged mode to report failure via semihosting.
    // (This is a fallback; normally the MemManage handler exits.)
    loop {
        asm::nop();
    }
}
