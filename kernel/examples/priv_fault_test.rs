//! QEMU test: verify unprivileged access to kernel memory faults.
//!
//! Configures MPU with PRIVDEFENA and partition-like code/data regions,
//! drops to unprivileged Thread mode (nPRIV=1), then attempts to read a
//! kernel-owned RAM address outside the allowed regions.  With PRIVDEFENA,
//! unprivileged accesses to addresses with no matching region generate a
//! MemManage fault, while privileged code (handler mode) falls through
//! to the default memory map.  The handler verifies the fault status
//! and prints PASS.
//!
//! This is the definitive end-to-end test that the isolation model
//! works: unprivileged partition code cannot reach kernel memory.
//!
//! ## Static mut usage
//!
//! This test intentionally uses `static mut PARTITION_STACK` because:
//!
//! - The MPU requires region base addresses to be aligned to the region size
//!   (1024 bytes for this 1 KiB stack). The `#[repr(C, align(1024))]` attribute
//!   ensures proper alignment, but we need a fixed address known at link time.
//!
//! - The test deliberately triggers a MemManage fault, which prevents using
//!   semihosting or any other abstraction after dropping to unprivileged mode.
//!   The raw address computation via `&raw const` is the only safe way to get
//!   the stack address without creating a reference to the static mut.
//!
//! - This is a fault-injection test that cannot use the unified harness because
//!   the harness expects partitions to run normally, not deliberately fault.
//!   The test must control exact MPU configuration and privilege transitions.

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m::register::{control, psp};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::mpu;
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Partition stack — 1 KiB, aligned to 1024 for MPU region base.
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut PARTITION_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Fault target: a kernel-owned RAM address outside any partition region.
// LM3S6965 RAM is 0x2000_0000 – 0x2000_FFFF.  The partition stack is
// near the start of RAM; 0x2000_F000 is well outside it.
// ---------------------------------------------------------------------------
const KERNEL_ADDR: u32 = 0x2000_F000;

// ---------------------------------------------------------------------------
// MemManage fault handler — runs in privileged handler mode.
// ---------------------------------------------------------------------------
#[exception]
fn MemoryManagement() {
    // SAFETY: accessing SCB from exception handler (privileged mode).
    // We don't have an owned `Peripherals` here, so we use the raw pointer.
    let scb = unsafe { &(*cortex_m::peripheral::SCB::PTR) };
    let cfsr = scb.cfsr.read();
    let mmfsr = cfsr & 0xFF;
    let mmfar = scb.mmfar.read();

    hprintln!("priv_fault_test: MemManage fault!");
    hprintln!("  MMFSR: {:#04x}", mmfsr);

    // DACCVIOL (bit 1): data access violation
    let daccviol = mmfsr & (1 << 1) != 0;
    // MMARVALID (bit 7): MMFAR holds the faulting address
    let mmarvalid = mmfsr & (1 << 7) != 0;

    hprintln!("  DACCVIOL = {} (expect true)", daccviol);
    hprintln!("  MMARVALID = {} (expect true)", mmarvalid);

    if mmarvalid {
        hprintln!("  MMFAR: {:#010x}", mmfar);
        hprintln!("  expected: {:#010x}", KERNEL_ADDR);
    }

    if daccviol {
        hprintln!("priv_fault_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("priv_fault_test: FAIL - not a DACCVIOL");
        debug::exit(debug::EXIT_FAILURE);
    }

    loop {
        asm::wfi();
    }
}

// ---------------------------------------------------------------------------
// MPU setup: partition-style regions WITHOUT a 4 GiB background deny-all.
//
// With PRIVDEFENA enabled and NO explicit background region, addresses
// that don't match any region use the default memory map for privileged
// code but generate a MemManage fault for unprivileged code.
//
// R0: code RX (flash 0x0, CODE_SIZE) — priv+unpriv read-only
// R1: data RW (partition stack, DATA_SIZE) — full access, XN
//
// KERNEL_ADDR (0x2000_F000) matches no region, so:
//   - Privileged handler mode → default map → access succeeds
//   - Unprivileged thread mode → fault (DACCVIOL)
// ---------------------------------------------------------------------------
const CODE_SIZE: u32 = 32 * 1024;
const DATA_SIZE: u32 = (STACK_WORDS * 4) as u32;

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
    // PRIVDEFENA: privileged accesses to unmatched addresses use the
    // default memory map; unprivileged accesses to unmatched addresses
    // generate a MemManage fault.
    // SAFETY: regions programmed; barriers ensure visibility.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("priv_fault_test: start");

    // Enable MemManage fault handler via the SCB peripheral abstraction.
    p.SCB.enable(Exception::MemoryManagement);

    // Get partition stack addresses via raw pointer (no reference to
    // static mut, just an address computation).
    let stack_base = (&raw const PARTITION_STACK).cast::<u32>() as u32;
    let stack_top = stack_base + DATA_SIZE;

    hprintln!("  stack: {:#010x} - {:#010x}", stack_base, stack_top);

    // Configure MPU before dropping privileges.
    configure_partition_mpu(&p.MPU, stack_base);

    hprintln!("  MPU configured, dropping to unprivileged...");

    // Switch to PSP and drop to unprivileged mode.
    // After this sequence the processor uses PSP in unprivileged Thread
    // mode — identical to how a real partition runs after PendSV.
    //
    // SAFETY: We set PSP to the top of a valid, MPU-accessible stack,
    // then set CONTROL.SPSEL=1 (use PSP) and CONTROL.nPRIV=1 (unprivileged).
    // control::write includes the architecturally required ISB.
    unsafe {
        psp::write(stack_top);
        let mut ctrl = control::read();
        ctrl.set_spsel(control::Spsel::Psp);
        ctrl.set_npriv(control::Npriv::Unprivileged);
        control::write(ctrl);
    }

    // Now unprivileged on PSP.  No hprintln! here — semihosting
    // accesses static data outside the partition's MPU data region,
    // which would fault before we reach the intentional read.
    //
    // This read must fault — we are unprivileged and KERNEL_ADDR has
    // no matching MPU region, so PRIVDEFENA denies the access.
    // SAFETY: deliberately triggering a MemManage fault for testing.
    unsafe { ptr::read_volatile(KERNEL_ADDR as *const u32) };

    loop {
        asm::nop();
    }
}
