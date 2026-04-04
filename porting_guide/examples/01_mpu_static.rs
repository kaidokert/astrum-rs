//! Static MPU region configuration and readback verification.
//!
//! Programs 4 MPU regions with distinct base addresses, sizes, and
//! attributes, then reads back RBAR/RASR for each region and compares
//! against the expected values. No context switching, no kernel — just
//! raw MPU register manipulation.
//!
//! # Regions
//!
//! | Slot | Purpose    | Base         | Size  | AP          | XN  |
//! |------|------------|--------------|-------|-------------|-----|
//! | R0   | Code       | 0x0000_0000  | 256K  | RO/RO (6)   | No  |
//! | R1   | Data       | 0x2000_0000  | 64K   | RW/RW (3)   | Yes |
//! | R2   | Stack      | 0x2001_0000  | 4K    | RW/RW (3)   | Yes |
//! | R3   | Peripheral | 0x4000_0000  | 4K    | RW/RW (3)   | Yes |
//!
//! # Success criteria
//!
//! All 4 regions read back with matching base address and RASR value.
//! Prints `01_mpu_static: PASS` and exits with semihosting success.
//! On any mismatch prints `01_mpu_static: FAIL` with the offending
//! region details and exits with semihosting failure.
//!
//! # Run
//!
//! ```text
//! cargo run --target thumbv7m-none-eabi \
//!     --features board-qemu,log-semihosting \
//!     --example 01_mpu_static
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu::{build_rasr, build_rbar, encode_size, AP_FULL_ACCESS, AP_RO_RO};
use porting_guide::klog;

/// Region descriptors: (base, size_bytes, ap, xn).
const REGIONS: [(u32, u32, u32, bool); 4] = [
    // R0: Code — 256 KiB read-only, executable
    (0x0000_0000, 256 * 1024, AP_RO_RO, false),
    // R1: Data — 64 KiB read-write, execute-never
    (0x2000_0000, 64 * 1024, AP_FULL_ACCESS, true),
    // R2: Stack — 4 KiB read-write, execute-never
    (0x2001_0000, 4 * 1024, AP_FULL_ACCESS, true),
    // R3: Peripheral — 4 KiB read-write, execute-never, device memory
    (0x4000_0000, 4 * 1024, AP_FULL_ACCESS, true),
];

/// S/C/B bits for each region: (Shareable, Cacheable, Bufferable).
/// Normal memory for code/data/stack; device memory for peripherals.
const SCB_ATTRS: [(bool, bool, bool); 4] = [
    (false, true, false), // R0 code: normal, cacheable
    (true, true, false),  // R1 data: shareable, cacheable
    (true, true, false),  // R2 stack: shareable, cacheable
    (true, false, true),  // R3 periph: shareable, device (C=0,B=1)
];

/// MPU CTRL: enable + PRIVDEFENA (privileged default map).
const MPU_CTRL_ENABLE_PRIVDEFENA: u32 = (1 << 0) | (1 << 2);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    let mpu = &p.MPU;

    klog!("01_mpu_static: configuring 4 MPU regions");

    // Disable MPU while programming regions.
    // SAFETY: We hold the only reference to MPU and are in privileged mode.
    unsafe {
        mpu.ctrl.write(0);
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Pre-compute and program each region.
    let mut expected: [(u32, u32); 4] = [(0, 0); 4];

    for (i, &(base, size_bytes, ap, xn)) in REGIONS.iter().enumerate() {
        let region = i as u32;
        let rbar = build_rbar(base, region).expect("valid RBAR");
        let size_field = encode_size(size_bytes).expect("valid size");
        let rasr = build_rasr(size_field, ap, xn, SCB_ATTRS[i]);

        // SAFETY: Programming MPU with valid RBAR/RASR while MPU is disabled.
        unsafe {
            mpu.rbar.write(rbar);
            mpu.rasr.write(rasr);
        }

        // Store expected values for readback verification.
        // After write, RBAR readback has base + region number (no VALID bit).
        expected[i] = (base | region, rasr);
    }

    // Re-enable MPU with PRIVDEFENA.
    // SAFETY: All regions are programmed; re-enabling is safe.
    unsafe {
        mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA);
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Verify: select each region via RNR, read back RBAR/RASR.
    let mut pass = true;
    for i in 0..4u32 {
        // SAFETY: Writing RNR to select region for readback is safe.
        unsafe {
            mpu.rnr.write(i);
        }

        let got_rbar = mpu.rbar.read();
        let got_rasr = mpu.rasr.read();

        let (exp_rbar, exp_rasr) = expected[i as usize];

        if got_rbar != exp_rbar || got_rasr != exp_rasr {
            klog!(
                "R{}: FAIL rbar exp={:#010x} got={:#010x} rasr exp={:#010x} got={:#010x}",
                i,
                exp_rbar,
                got_rbar,
                exp_rasr,
                got_rasr
            );
            pass = false;
        } else {
            klog!("R{}: OK base={:#010x} rasr={:#010x}", i, got_rbar, got_rasr);
        }
    }

    if pass {
        klog!("01_mpu_static: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        klog!("01_mpu_static: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }

    loop {
        cortex_m::asm::nop();
    }
}
