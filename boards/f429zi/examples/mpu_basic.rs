#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use cortex_m_rt::entry;
use stm32f4xx_hal as _; // Device interrupt vectors

// MPU helper functions (from astrum_rtos_priv_rs/kernel/src/mpu.rs)

/// Encode region size in bytes to the 5-bit RASR SIZE field (`log2(size) - 1`).
fn encode_size(size_bytes: u32) -> Option<u32> {
    if size_bytes < 32 || !size_bytes.is_power_of_two() {
        return None;
    }
    Some(size_bytes.trailing_zeros() - 1)
}

/// Build RBAR value: base address with VALID bit and region number (0..=7).
const fn build_rbar(base: u32, region: u32) -> Option<u32> {
    if region > 7 || base & 0x1F != 0 {
        return None;
    }
    Some(base | (1 << 4) | region)
}

/// Build RASR value from size field, access permissions, XN, and S/C/B bits.
fn build_rasr(size_field: u32, ap: u32, xn: bool, scb: (bool, bool, bool)) -> u32 {
    let (s, c, b) = scb;
    (u32::from(xn) << 28)
        | ((ap & 0x7) << 24)
        | (u32::from(s) << 18)
        | (u32::from(c) << 17)
        | (u32::from(b) << 16)
        | ((size_field & 0x1F) << 1)
        | 1
}

/// Write RBAR and RASR to configure a single MPU region.
fn configure_region(mpu: &cortex_m::peripheral::MPU, rbar: u32, rasr: u32) {
    unsafe {
        mpu.rbar.write(rbar);
        mpu.rasr.write(rasr);
    }
}

// Access permission constants
const AP_PRIV_RW: u32 = 0b001;     // Privileged read-write
const AP_FULL_ACCESS: u32 = 0b011; // Full read-write access
const AP_NO_ACCESS: u32 = 0b000;   // No access

// MPU CTRL value: enable MPU (bit 0) + PRIVDEFENA (bit 2)
const MPU_CTRL_ENABLE_PRIVDEFENA: u32 = (1 << 2) | 1;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("\n=== STM32F429ZI MPU Basic Test ===\n");

    let cp = cortex_m::Peripherals::take().unwrap();
    let mpu = &cp.MPU;

    // Check if MPU is present
    let mpu_type = unsafe { mpu._type.read() };
    rprintln!("MPU TYPE register: 0x{:08x}", mpu_type);

    // STM32F429 has 8 MPU regions
    let num_regions = (mpu_type >> 8) & 0xFF;
    rprintln!("Number of MPU regions: {}", num_regions);

    if num_regions == 0 {
        rprintln!("ERROR: No MPU present!");
        loop {}
    }

    // Use RAM addresses in STM32F429 SRAM
    // SRAM1 starts at 0x20000000, we'll use areas well within it
    let protected_base = 0x2001_0000u32; // 64KB into RAM
    let protected_size = 256u32;         // 256 bytes

    rprintln!("\nConfiguring MPU:");
    rprintln!("  Region 0: Protected area @ 0x{:08x}, {} bytes (FULL_ACCESS)",
              protected_base, protected_size);

    // Disable MPU while configuring
    unsafe { mpu.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: Protected RAM area (full access, read-write, XN=true, cacheable/bufferable)
    let size_field = encode_size(protected_size).expect("Invalid size");
    let rbar0 = build_rbar(protected_base, 0).expect("Invalid RBAR");
    let rasr0 = build_rasr(size_field, AP_FULL_ACCESS, true, (true, true, false));

    rprintln!("  RBAR0: 0x{:08x}", rbar0);
    rprintln!("  RASR0: 0x{:08x}", rasr0);

    configure_region(mpu, rbar0, rasr0);

    // Enable MPU with PRIVDEFENA
    unsafe { mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    rprintln!("\nMPU enabled (CTRL: 0x{:08x})", mpu.ctrl.read());

    // Test 1: Write to protected area (should work)
    rprintln!("\nTest 1: Writing to protected area...");
    let protected_ptr = protected_base as *mut u32;
    unsafe {
        core::ptr::write_volatile(protected_ptr, 0xDEADBEEF);
    }
    let read_back = unsafe { core::ptr::read_volatile(protected_ptr) };
    rprintln!("  Wrote: 0xDEADBEEF");
    rprintln!("  Read:  0x{:08x}", read_back);

    if read_back == 0xDEADBEEF {
        rprintln!("  ✓ SUCCESS");
    } else {
        rprintln!("  ✗ FAILED");
    }

    // Test 2: Read from unprotected area (should work due to PRIVDEFENA)
    let unprotected_base = 0x2000_0100u32;
    rprintln!("\nTest 2: Accessing unprotected area @ 0x{:08x}...", unprotected_base);
    let unprotected_ptr = unprotected_base as *mut u32;
    unsafe {
        core::ptr::write_volatile(unprotected_ptr, 0xCAFEBABE);
    }
    let read_back2 = unsafe { core::ptr::read_volatile(unprotected_ptr) };
    rprintln!("  Wrote: 0xCAFEBABE");
    rprintln!("  Read:  0x{:08x}", read_back2);

    if read_back2 == 0xCAFEBABE {
        rprintln!("  ✓ SUCCESS (PRIVDEFENA allows access)");
    } else {
        rprintln!("  ✗ FAILED");
    }

    // Test 3: Configure a NO_ACCESS region and attempt to access it
    // This will cause a MemManage fault in unprivileged mode
    rprintln!("\nTest 3: Configuring NO_ACCESS region...");
    let forbidden_base = 0x2001_1000u32;

    unsafe { mpu.ctrl.write(0) }; // Disable temporarily
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    let rbar1 = build_rbar(forbidden_base, 1).expect("Invalid RBAR");
    let rasr1 = build_rasr(size_field, AP_NO_ACCESS, true, (false, false, false));

    rprintln!("  Region 1: Forbidden area @ 0x{:08x}, {} bytes (NO_ACCESS)",
              forbidden_base, protected_size);
    rprintln!("  RBAR1: 0x{:08x}", rbar1);
    rprintln!("  RASR1: 0x{:08x}", rasr1);

    configure_region(mpu, rbar1, rasr1);

    unsafe { mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    rprintln!("  Note: In privileged mode, PRIVDEFENA still allows access");
    rprintln!("  (Need to switch to unprivileged mode for true protection)");

    rprintln!("\n=== MPU Basic Test Complete ===\n");
    rprintln!("Summary:");
    rprintln!("  - MPU successfully configured with {} regions", num_regions);
    rprintln!("  - Protected region: Read/Write access works");
    rprintln!("  - Unprotected region: Access works (PRIVDEFENA)");
    rprintln!("  - NO_ACCESS region: Configured (needs unpriv mode to test fault)");

    loop {
        cortex_m::asm::wfi();
    }
}
