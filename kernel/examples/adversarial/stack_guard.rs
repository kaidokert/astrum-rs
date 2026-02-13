//! Adversarial test: partition writes downward until stack hits the guard region.
//!
//! This test verifies that the 32-byte no-access guard region at stack_base
//! (MPU Region 3) correctly triggers a MemManage fault before stack overflow
//! can corrupt adjacent memory.
//!
//! The partition entry point uses an assembly loop to write descending addresses
//! from stack_top toward stack_base. When a write reaches the guard region, the
//! MPU should generate a MemManage fault with DACCVIOL.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example stack_guard

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use kernel::mpu;
use panic_semihosting as _;

#[path = "mod.rs"]
mod adversarial;

use adversarial::{FaultInfo, FaultOutcome, STACK_SIZE};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Test name for reporting.
const TEST_NAME: &str = "stack_guard";

/// Stack size in words (256 words = 1 KiB).
const STACK_WORDS: usize = 256;

/// Guard region size in bytes.
const GUARD_SIZE: u32 = 32;

/// Code region size for MPU.
const CODE_SIZE: u32 = 32 * 1024;

// ---------------------------------------------------------------------------
// Partition stack — 1 KiB, aligned to 1024 for MPU region base.
// ---------------------------------------------------------------------------

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut PARTITION_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Fault capture
// ---------------------------------------------------------------------------

static mut FAULT: FaultInfo = FaultInfo::new();

/// Guard region bounds for validation in the handler.
static mut GUARD_BASE: u32 = 0;
static mut GUARD_END: u32 = 0;

define_memmanage_handler!(FAULT, {
    // Handler runs in privileged mode; can use semihosting.
    // SAFETY: Reading FAULT from the exception handler that just wrote it.
    // No other code accesses FAULT concurrently (interrupts of same/lower
    // priority are blocked during this exception).
    let info = unsafe { ptr::read_volatile(&raw const FAULT) };
    // SAFETY: GUARD_BASE was written by main() before dropping privileges.
    // This handler reads it after a fault; no concurrent writers exist.
    let guard_base = unsafe { ptr::read_volatile(&raw const GUARD_BASE) };
    // SAFETY: GUARD_END was written by main() before dropping privileges.
    // This handler reads it after a fault; no concurrent writers exist.
    let guard_end = unsafe { ptr::read_volatile(&raw const GUARD_END) };

    // Verify we got a DACCVIOL (data access violation) and MMFAR is valid.
    let outcome = if info.is_daccviol() && info.is_addr_valid() {
        // Verify the faulting address is within the guard region.
        if info.mmfar >= guard_base && info.mmfar < guard_end {
            hprintln!(
                "  fault in guard region: mmfar={:#010x} guard=[{:#010x}, {:#010x})",
                info.mmfar,
                guard_base,
                guard_end
            );
            FaultOutcome::Faulted {
                mmfsr: info.mmfsr,
                mmfar: info.mmfar,
            }
        } else {
            hprintln!(
                "  fault outside guard: mmfar={:#010x} guard=[{:#010x}, {:#010x})",
                info.mmfar,
                guard_base,
                guard_end
            );
            FaultOutcome::Error("fault address not in guard region")
        }
    } else if info.faulted {
        FaultOutcome::Error("fault without DACCVIOL or no MMFAR")
    } else {
        FaultOutcome::NoFault
    };

    adversarial::report_result(TEST_NAME, outcome);
});

// ---------------------------------------------------------------------------
// MPU configuration with stack guard region
// ---------------------------------------------------------------------------

/// Configure MPU for partition with a 32-byte guard region at stack_base.
///
/// R0: code RX (flash 0x0) — priv+unpriv read-only
/// R1: data RW (partition stack) — full access, XN
/// R2: guard (32 bytes at stack_base) — no access, XN
///
/// Region priority: R2 > R1 > R0, so the guard overlaps and denies access.
fn configure_partition_mpu_with_guard(mpu_periph: &cortex_m::peripheral::MPU, data_base: u32) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: code RX — covers flash binary (priv+unpriv read-only)
    let code_sf = mpu::encode_size(CODE_SIZE).expect("code size");
    let code_rbar = mpu::build_rbar(0x0000_0000, 0).expect("code rbar");
    let code_rasr = mpu::build_rasr(code_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, code_rbar, code_rasr);

    // R1: data RW — partition stack (full access, XN)
    let data_sf = mpu::encode_size(STACK_SIZE).expect("data size");
    let data_rbar = mpu::build_rbar(data_base, 1).expect("data rbar");
    let data_rasr = mpu::build_rasr(data_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, data_rbar, data_rasr);

    // R2: guard region — 32 bytes at stack_base, no access
    let guard_sf = mpu::encode_size(GUARD_SIZE).expect("guard size");
    let guard_rbar = mpu::build_rbar(data_base, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_NO_ACCESS, true, (false, false, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

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
    let stack_top = stack_base + STACK_SIZE;

    // Store guard region bounds for validation in the handler.
    // SAFETY: Single-core, main is the only writer before dropping privileges.
    unsafe {
        ptr::write_volatile(&raw mut GUARD_BASE, stack_base);
        ptr::write_volatile(&raw mut GUARD_END, stack_base + GUARD_SIZE);
    }

    hprintln!("  stack: {:#010x} - {:#010x}", stack_base, stack_top);
    hprintln!(
        "  guard: {:#010x} - {:#010x}",
        stack_base,
        stack_base + GUARD_SIZE
    );

    // Configure MPU with guard region before dropping privileges.
    configure_partition_mpu_with_guard(&p.MPU, stack_base);

    hprintln!("  MPU configured with guard region, dropping to unprivileged...");

    // Switch to PSP and drop to unprivileged mode, then immediately start
    // writing into the guard region.
    //
    // This is all done in a single inline asm block to avoid any function
    // calls or stack accesses after switching to unprivileged PSP.
    //
    // SAFETY: We set PSP to the top of a valid, MPU-accessible stack,
    // then set CONTROL.SPSEL=1 (use PSP) and CONTROL.nPRIV=1 (unprivileged),
    // then write to descending addresses until we hit the guard.
    //
    // We use explicit registers to have full control over the generated code.
    // r2 = stack_top (address to write to)
    // r3 = pattern to write
    // r4 = temp for CONTROL manipulation
    #[allow(asm_sub_register)] // Thumb has no sub-registers; warning not applicable
    unsafe {
        core::arch::asm!(
            // Move stack_top into r2 for use in the loop
            "mov r2, {stack_top}",
            // Write stack_top to PSP
            "msr psp, r2",
            // Read CONTROL into r4
            "mrs r4, control",
            // Set SPSEL bit (bit 1) and nPRIV bit (bit 0)
            "orr r4, r4, #3",
            // Write CONTROL
            "msr control, r4",
            // ISB to ensure CONTROL changes take effect
            "isb",
            // Now unprivileged on PSP. r2 still has stack_top.
            // Subtract 4 to start below stack_top
            "subs r2, r2, #4",
            // Load pattern into r3
            "movs r3, #0xFF",
            // TODO: Use a finite loop with iteration count based on stack size
            // and assert fault occurred, for more defensive test code.
            // Loop: write and decrement until fault
            "1:",
            "str r3, [r2]",
            "subs r2, r2, #4",
            "b 1b",
            stack_top = in(reg) stack_top,
            options(noreturn)
        );
    }
}
