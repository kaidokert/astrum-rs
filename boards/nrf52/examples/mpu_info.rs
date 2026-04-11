//! MPU Info — PCA10100 (nRF52833-DK)
//!
//! Reads MPU TYPE and CTRL registers directly, then dumps all region
//! RBAR/RASR values via RTT.  No regions are configured here — this
//! just reports the reset-state of the MPU (should be disabled, 8 regions).
//!
//! Expected: DREGION=8, CTRL=0x0 (disabled on reset)
//!
//! Build:  cargo build --example mpu_info
//! Flash:  cargo run --example mpu_info

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m as _; // critical-section-single-core impl
use nrf52833_pac as _; // force PAC link so __INTERRUPTS from rt feature is included

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("=== MPU Info — nRF52833 ===");

    let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };

    // MPU TYPE register: bits [15:8] = DREGION (number of regions)
    let mpu_type = mpu._type.read();
    let dregion = (mpu_type >> 8) & 0xFF;
    let separate = mpu_type & 1; // 0 = unified I/D

    rprintln!("MPU TYPE  = 0x{:08x}", mpu_type);
    rprintln!("  DREGION = {} (data regions)", dregion);
    rprintln!("  SEPARATE= {} ({})", separate, if separate == 0 { "unified" } else { "separate I/D" });

    let mpu_ctrl = mpu.ctrl.read();
    rprintln!("MPU CTRL  = 0x{:08x}", mpu_ctrl);
    rprintln!("  ENABLE  = {}", mpu_ctrl & 1);
    rprintln!("  HFNMIENA= {}", (mpu_ctrl >> 1) & 1);
    rprintln!("  PRIVDEFA= {}", (mpu_ctrl >> 2) & 1);

    rprintln!("");
    rprintln!("--- Region dump (reset state) ---");
    for r in 0..dregion {
        unsafe { mpu.rnr.write(r) }; // write requires unsafe (side-effectful)
        let rbar = mpu.rbar.read();
        let rasr = mpu.rasr.read();
        rprintln!("  R{}: RBAR=0x{:08x} RASR=0x{:08x} EN={}", r, rbar, rasr, rasr & 1);
    }
    rprintln!("--- end ---");

    if dregion == 8 && mpu_ctrl & 1 == 0 {
        rprintln!("OK: 8 regions, MPU disabled at reset (as expected)");
    }

    loop {
        cortex_m::asm::wfi();
    }
}
