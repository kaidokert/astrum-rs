//! Adversarial test: unprivileged partition attempts to write MPU registers.
//!
//! This test verifies that the MPU isolation correctly prevents unprivileged
//! partition code from writing to MPU configuration registers (MPU_CTRL,
//! MPU_RBAR, etc.) in the System Control Space (0xE000_E000 - 0xE000_EFFF).
//!
//! The ARM Cortex-M System Control Space is privileged-only access. Any
//! unprivileged write attempt should trigger a MemManage or BusFault,
//! preventing partitions from reconfiguring their own memory regions.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example write_mpu_regs

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m::register::{control, psp};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use kernel::mpu;
use panic_semihosting as _;

#[path = "mod.rs"]
mod adversarial;

use adversarial::{FaultInfo, FaultOutcome};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// MPU Control Register address in the System Control Space.
/// Privileged-only access — unprivileged writes must fault.
const MPU_CTRL_ADDR: u32 = 0xE000_ED94;

/// MPU Region Base Address Register.
/// Privileged-only access — unprivileged writes must fault.
const MPU_RBAR_ADDR: u32 = 0xE000_ED9C;

/// Test name for reporting.
const TEST_NAME: &str = "write_mpu_regs";

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
/// System Control Space (0xE000_E000 - 0xE000_EFFF) matches no unprivileged
/// region, so unprivileged access to MPU registers will fault.
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
    hprintln!("  target MPU_CTRL: {:#010x}", MPU_CTRL_ADDR);
    hprintln!("  target MPU_RBAR: {:#010x}", MPU_RBAR_ADDR);

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
    // reaching the intentional write.
    //
    // Attempt to write MPU_CTRL — must fault (DACCVIOL).
    // This would disable the MPU if it succeeded, breaking isolation.
    // SAFETY: deliberately triggering a MemManage fault for testing.
    unsafe { ptr::write_volatile(MPU_CTRL_ADDR as *mut u32, 0x0000_0000) };

    // Should never reach here — if we do, no fault occurred.
    // The write above should have faulted. If it didn't, the partition
    // could disable the MPU and access arbitrary memory.
    loop {
        asm::nop();
    }
}
