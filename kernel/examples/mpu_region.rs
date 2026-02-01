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
    // SAFETY: writing MPU control register to disable before reconfiguration.
    unsafe { p.MPU.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: base 0x2000_0000, 256 bytes, RW, XN
    let size_field = mpu::encode_size(256).unwrap();
    let rbar_val = mpu::build_rbar(0x2000_0000, 0).unwrap();
    let rasr_val = mpu::build_rasr(size_field, 0b011, true, (true, true, false));

    hprintln!("RBAR: write={:#010x}", rbar_val);
    hprintln!("RASR: write={:#010x}", rasr_val);

    mpu::configure_region(&p.MPU, rbar_val, rasr_val);

    // Enable MPU with PRIVDEFENA so privileged code keeps default memory map
    // SAFETY: enabling the MPU with privileged default map.
    unsafe { p.MPU.ctrl.write((1 << 2) | 1) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Read back: select region 0 via RNR first
    // SAFETY: writing RNR to select region for readback.
    unsafe { p.MPU.rnr.write(0) };
    let rbar_rb = p.MPU.rbar.read();
    let rasr_rb = p.MPU.rasr.read();

    hprintln!("RBAR: read ={:#010x}", rbar_rb);
    hprintln!("RASR: read ={:#010x}", rasr_rb);

    // Verify: RBAR readback has VALID=0 and region bits reflect RNR (0),
    // so expect base address only: 0x2000_0000.
    let rbar_ok = rbar_rb == 0x2000_0000;
    let rasr_ok = rasr_rb == rasr_val;

    if rbar_ok && rasr_ok {
        hprintln!("PASS");
    } else {
        hprintln!("FAIL");
        if !rbar_ok {
            hprintln!(
                "  RBAR expected {:#010x}, got {:#010x}",
                0x2000_0000_u32,
                rbar_rb
            );
        }
        if !rasr_ok {
            hprintln!("  RASR expected {:#010x}, got {:#010x}", rasr_val, rasr_rb);
        }
    }

    debug::exit(debug::EXIT_SUCCESS);
    loop {
        cortex_m::asm::wfi();
    }
}
