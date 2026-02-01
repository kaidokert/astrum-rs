#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
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
    //
    // RBAR (Region Base Address Register):
    //   bits [31:5] = base address (must be aligned to region size)
    //   bit  [4]    = VALID (1 = use region number in bits 3:0)
    //   bits [3:0]  = region number
    let region: u32 = 0;
    let rbar_val: u32 = 0x2000_0000 | (1 << 4) | region;

    // RASR (Region Attribute and Size Register):
    //   bit  [28]    = XN (execute never)
    //   bits [26:24] = AP (access permission, 0b011 = full access RW)
    //   bits [21:19] = TEX (type extension, 0b000 for normal memory)
    //   bit  [18]    = S (shareable)
    //   bit  [17]    = C (cacheable)
    //   bit  [16]    = B (bufferable)
    //   bits [15:8]  = SRD (sub-region disable, 0 = all enabled)
    //   bits [5:1]   = SIZE (7 → 2^(7+1) = 256 bytes)
    //   bit  [0]     = ENABLE
    let rasr_val: u32 = (1 << 28)        // XN
        | (0b011 << 24)                  // AP = full access
        | (1 << 18)                      // S
        | (1 << 17)                      // C
        | (7 << 1)                       // SIZE = 7
        | 1; // ENABLE

    hprintln!("RBAR: write={:#010x}", rbar_val);
    hprintln!("RASR: write={:#010x}", rasr_val);

    // SAFETY: writing MPU RBAR and RASR to configure region 0.
    unsafe {
        p.MPU.rbar.write(rbar_val);
        p.MPU.rasr.write(rasr_val);
    }

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
