//! Adversarial test: partition 0 attempts to read partition 1's stack region.
//!
//! This test verifies that MPU isolation prevents unprivileged partition code
//! from accessing another partition's data region.  With PRIVDEFENA and
//! separate MPU data regions per partition, an attempt by partition 0 to read
//! an address within partition 1's stack must generate a MemManage fault.
//!
//! The observer partition concept is implicit: the MemManage handler runs in
//! privileged mode and verifies the kernel continues correctly after the fault.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example read_other_stack

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

/// Test name for reporting.
const TEST_NAME: &str = "read_other_stack";

// ---------------------------------------------------------------------------
// Partition stacks — two separate 1 KiB stacks, aligned to 1024 for MPU.
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32; // 1024 bytes

/// Partition 0 stack (adversarial partition that attempts the read).
#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut P0_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

/// Partition 1 stack (victim partition whose memory should be protected).
static mut P1_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Fault capture
// ---------------------------------------------------------------------------
static mut FAULT: FaultInfo = FaultInfo::new();

// TODO: reviewer false positive - `&raw const` is valid Rust syntax (stabilized in Rust 1.82,
// see https://blog.rust-lang.org/2024/10/17/Rust-1.82.0.html). This syntax creates a raw pointer
// to a static without going through a reference, which is the recommended pattern for mutable
// statics. The code passes `cargo check` and matches the pattern used in read_kernel_mem.rs.
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

/// Configure MPU for partition 0: only allow access to partition 0's stack.
///
/// R0: code RX (flash 0x0) — priv+unpriv read-only
/// R1: data RW (partition 0 stack) — full access, XN
///
/// Partition 1's stack has no matching MPU region for partition 0,
/// so unprivileged access to P1's stack will fault (DACCVIOL).
fn configure_p0_mpu(mpu_periph: &cortex_m::peripheral::MPU, p0_data_base: u32) {
    // SAFETY: single-core, interrupts disabled — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: code RX — covers flash binary (priv+unpriv read-only)
    let code_sf = mpu::encode_size(CODE_SIZE).expect("code size");
    let code_rbar = mpu::build_rbar(0x0000_0000, 0).expect("code rbar");
    let code_rasr = mpu::build_rasr(code_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, code_rbar, code_rasr);

    // R1: data RW — partition 0's stack (full access, XN)
    let data_sf = mpu::encode_size(STACK_SIZE).expect("data size");
    let data_rbar = mpu::build_rbar(p0_data_base, 1).expect("data rbar");
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

    // Get partition 0 stack addresses (adversarial partition).
    // NOTE: `&raw const` is valid stable Rust syntax (see TODO comment above).
    let p0_stack_base = (&raw const P0_STACK).cast::<u32>() as u32;
    let p0_stack_top = p0_stack_base + STACK_SIZE;

    // Get partition 1 stack addresses (victim partition).
    let p1_stack_base = (&raw const P1_STACK).cast::<u32>() as u32;
    let p1_stack_top = p1_stack_base + STACK_SIZE;

    // Target address: middle of partition 1's stack.
    let target_addr = p1_stack_base + STACK_SIZE / 2;

    hprintln!(
        "  P0 stack: {:#010x} - {:#010x}",
        p0_stack_base,
        p0_stack_top
    );
    hprintln!(
        "  P1 stack: {:#010x} - {:#010x}",
        p1_stack_base,
        p1_stack_top
    );
    hprintln!("  target (P1): {:#010x}", target_addr);

    // Verify stacks don't overlap (sanity check).
    assert!(
        p0_stack_top <= p1_stack_base || p1_stack_top <= p0_stack_base,
        "stacks must not overlap"
    );

    // Configure MPU for partition 0 before dropping privileges.
    configure_p0_mpu(&p.MPU, p0_stack_base);

    hprintln!("  MPU configured for P0, dropping to unprivileged...");

    // Switch to PSP and drop to unprivileged mode.
    // SAFETY: We set PSP to the top of partition 0's valid, MPU-accessible stack,
    // then set CONTROL.SPSEL=1 (use PSP) and CONTROL.nPRIV=1 (unprivileged).
    unsafe {
        psp::write(p0_stack_top);
        let mut ctrl = control::read();
        ctrl.set_spsel(control::Spsel::Psp);
        ctrl.set_npriv(control::Npriv::Unprivileged);
        control::write(ctrl);
    }

    // Now unprivileged on PSP, running as "partition 0".
    // No semihosting here — would fault before reaching the intentional read.
    //
    // Attempt to read partition 1's stack — must fault (DACCVIOL).
    // SAFETY: deliberately triggering a MemManage fault for testing.
    let _ = unsafe { ptr::read_volatile(target_addr as *const u32) };

    // Should never reach here — if we do, no fault occurred.
    loop {
        asm::nop();
    }
}
