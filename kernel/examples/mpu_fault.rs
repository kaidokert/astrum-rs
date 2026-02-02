//! Trigger a MemManage fault by reading from a no-access MPU region.
//!
//! Configures MPU region 0 as no-access at 0x2000_F000, enables the
//! MemManage handler, then deliberately reads the protected address.
//! The handler prints fault status and exits with success.

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::mpu;
use panic_semihosting as _;

/// Base address of the no-access region.
const FAULT_ADDR: u32 = 0x2000_F000;

/// SCB CFSR address — MMFSR is the low byte (bits [7:0]).
const SCB_CFSR: u32 = 0xE000_ED28;
/// SCB MMFAR address.
const SCB_MMFAR: u32 = 0xE000_ED34;
/// SCB SHCSR address.
const SCB_SHCSR: u32 = 0xE000_ED24;

#[exception]
fn MemoryManagement() {
    let cfsr = unsafe { ptr::read_volatile(SCB_CFSR as *const u32) };
    let mmfsr = cfsr & 0xFF;
    let mmfar = unsafe { ptr::read_volatile(SCB_MMFAR as *const u32) };

    hprintln!("MemManage fault!");
    hprintln!("  MMFSR: {:#04x}", mmfsr);
    hprintln!("  MMFAR: {:#010x}", mmfar);

    // MMARVALID (bit 7) means MMFAR holds the faulting address
    if mmfsr & (1 << 7) != 0 {
        hprintln!("  fault address valid: {:#010x}", mmfar);
    }

    // Confirm the fault address matches the protected region
    if mmfar == FAULT_ADDR {
        hprintln!("PASS: fault address matches protected region");
    } else {
        hprintln!("FAIL: expected {:#010x}, got {:#010x}", FAULT_ADDR, mmfar);
    }

    debug::exit(debug::EXIT_SUCCESS);

    loop {
        asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    // Enable MemManage fault handler (SHCSR bit 16 = MEMFAULTENA)
    unsafe {
        let shcsr = ptr::read_volatile(SCB_SHCSR as *const u32);
        ptr::write_volatile(SCB_SHCSR as *mut u32, shcsr | (1 << 16));
    }

    // Disable MPU during configuration
    unsafe { p.MPU.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // Region 0: no-access at FAULT_ADDR, 32 bytes (minimum size)
    let size_field = mpu::encode_size(32).unwrap();
    let rbar = mpu::build_rbar(FAULT_ADDR, 0).unwrap();
    let rasr = mpu::build_rasr(size_field, mpu::AP_NO_ACCESS, true, (false, false, false));
    mpu::configure_region(&p.MPU, rbar, rasr);

    // Enable MPU with PRIVDEFENA — privileged default memory map remains active
    // SAFETY: enabling the MPU after region configuration is complete.
    unsafe { p.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();

    hprintln!("MPU configured, reading from {:#010x}...", FAULT_ADDR);

    // Deliberate read from the no-access region — triggers MemManage fault
    unsafe { ptr::read_volatile(FAULT_ADDR as *const u32) };

    // Should never reach here
    hprintln!("ERROR: no fault triggered");
    debug::exit(debug::EXIT_FAILURE);

    loop {
        asm::wfi();
    }
}
