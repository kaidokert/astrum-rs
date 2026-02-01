#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use kernel::mpu;
use panic_semihosting as _;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    // Disable MPU during configuration
    unsafe { p.MPU.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    let size_field = mpu::encode_size(256).unwrap();

    // Region 0: code area at 0x0000_0000, 256 bytes, RX (privileged RO, executable)
    let rbar0 = mpu::build_rbar(0x0000_0000, 0).unwrap();
    let rasr0 = mpu::build_rasr(size_field, mpu::AP_PRIV_RO, false, (false, false, false));
    mpu::configure_region(&p.MPU, rbar0, rasr0);

    // Region 1: data area at 0x2000_0000, 256 bytes, RW/XN (full access, no-execute)
    let rbar1 = mpu::build_rbar(0x2000_0000, 1).unwrap();
    let rasr1 = mpu::build_rasr(size_field, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(&p.MPU, rbar1, rasr1);

    // Enable MPU with PRIVDEFENA
    unsafe { p.MPU.ctrl.write((1 << 2) | 1) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Read back region 0
    unsafe { p.MPU.rnr.write(0) };
    let rbar0_rb = p.MPU.rbar.read();
    let rasr0_rb = p.MPU.rasr.read();

    hprintln!("Region 0 (code RX):");
    hprintln!("  RBAR: write={:#010x} read={:#010x}", rbar0, rbar0_rb);
    hprintln!("  RASR: write={:#010x} read={:#010x}", rasr0, rasr0_rb);

    // Read back region 1
    unsafe { p.MPU.rnr.write(1) };
    let rbar1_rb = p.MPU.rbar.read();
    let rasr1_rb = p.MPU.rasr.read();

    hprintln!("Region 1 (data RW/XN):");
    hprintln!("  RBAR: write={:#010x} read={:#010x}", rbar1, rbar1_rb);
    hprintln!("  RASR: write={:#010x} read={:#010x}", rasr1, rasr1_rb);

    // Verify: RBAR readback has base in bits [31:5], region in [3:0], VALID=0
    let rbar0_ok = rbar0_rb & !0x1F == 0x0000_0000;
    let rasr0_ok = rasr0_rb == rasr0;
    let rbar1_ok = rbar1_rb & !0x1F == 0x2000_0000;
    let rasr1_ok = rasr1_rb == rasr1;

    if rbar0_ok && rasr0_ok && rbar1_ok && rasr1_ok {
        hprintln!("PASS");
    } else {
        hprintln!("FAIL");
        if !rbar0_ok {
            hprintln!("  R0 RBAR base expected {:#010x}", 0u32);
        }
        if !rasr0_ok {
            hprintln!("  R0 RASR expected {:#010x}, got {:#010x}", rasr0, rasr0_rb);
        }
        if !rbar1_ok {
            hprintln!("  R1 RBAR base expected {:#010x}", 0x2000_0000u32);
        }
        if !rasr1_ok {
            hprintln!("  R1 RASR expected {:#010x}, got {:#010x}", rasr1, rasr1_rb);
        }
    }

    debug::exit(debug::EXIT_SUCCESS);
    loop {
        cortex_m::asm::wfi();
    }
}
