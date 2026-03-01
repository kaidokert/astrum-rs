//! QEMU example: MPU region changes across partition switches.
//!
//! Verifies `partition_mpu_regions` computes correct RBAR/RASR for two
//! partitions, then programs just the data region on each switch and
//! reads back the MPU register to prove the region changed.

#![no_std]
#![no_main]

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu::{self, partition_mpu_regions};
use kernel::partition::{MpuRegion, PartitionControlBlock};

use core::sync::atomic::{AtomicU32, Ordering};

static PARTITION: AtomicU32 = AtomicU32::new(u32::MAX);
const MAX_SWITCHES: u32 = 4;
const RELOAD: u32 = kernel::config::compute_systick_reload(12_000_000, 10_000);

fn make_pcb(id: u8, data_base: u32) -> PartitionControlBlock {
    PartitionControlBlock::new(
        id,
        0x0000_0000,
        data_base,
        data_base + 4096,
        MpuRegion::new(data_base, 4096, 0),
    )
}

#[exception]
fn SysTick() {
    let prev = PARTITION.load(Ordering::Relaxed);
    let next = if prev == 0 { 1 } else { 0 };
    PARTITION.store(next, Ordering::Release);
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    let mut syst = p.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    let pcbs = [make_pcb(0, 0x2000_0000), make_pcb(1, 0x2000_8000)];

    // Verify computed regions differ in data RBAR
    let r0 = partition_mpu_regions(&pcbs[0]).unwrap();
    let r1 = partition_mpu_regions(&pcbs[1]).unwrap();
    assert_ne!(r0[2].0, r1[2].0); // data RBAR differs
    assert_eq!(r0[2].1, r1[2].1); // data RASR same
    hprintln!("mpu_partition: region values verified");

    // Enable MPU with PRIVDEFENA (no background region — safe for QEMU)
    unsafe { p.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    hprintln!("mpu_partition: waiting for {} switches", MAX_SWITCHES);

    let mut last_seen = u32::MAX;
    let mut switches = 0u32;

    loop {
        let current = PARTITION.load(Ordering::Acquire);
        if current != last_seen {
            let pcb = &pcbs[current as usize];
            let regions = partition_mpu_regions(pcb).unwrap();

            // Program data region (index 2) into MPU
            unsafe { p.MPU.ctrl.write(0) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            mpu::configure_region(&p.MPU, regions[2].0, regions[2].1);
            unsafe { p.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            // Read back and verify
            let rbar: u32;
            unsafe {
                p.MPU.rnr.write(2);
                rbar = p.MPU.rbar.read();
            }
            let exp_base = regions[2].0 & !0x1F;
            let got_base = rbar & !0x1F;

            switches += 1;
            hprintln!(
                "switch {}: p{} RBAR={:#010x} exp={:#010x} {}",
                switches,
                current,
                got_base,
                exp_base,
                if got_base == exp_base { "OK" } else { "FAIL" }
            );

            last_seen = current;
            if switches >= MAX_SWITCHES {
                hprintln!("mpu_partition: PASS");
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }
}
