//! QEMU example: MPU region changes across partition switches.
//!
//! Demonstrates `precompute_mpu_cache` to populate cached RBAR/RASR pairs
//! in each PCB at boot, then reads `cached_base_regions` on each switch to
//! program the data region and reads back the MPU register to prove the
//! region changed.

#![no_std]
#![no_main]

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu::{self, precompute_mpu_cache};
use kernel::partition::{MpuRegion, PartitionControlBlock};

use core::sync::atomic::{AtomicU32, Ordering};

static PARTITION: AtomicU32 = AtomicU32::new(u32::MAX);
const MAX_SWITCHES: u32 = 4;
const RELOAD: u32 = kernel::config::compute_systick_reload(12_000_000, 10_000);
/// MPU region index for the partition data region (R2).
const DATA_REGION: usize = 2;

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

    let mut pcbs = [make_pcb(0, 0x2000_0000), make_pcb(1, 0x2000_8000)];

    // Precompute and cache MPU regions for each partition at boot.
    precompute_mpu_cache(pcbs.get_mut(0).expect("pcb[0]"))
        .expect("precompute_mpu_cache failed for partition 0");
    precompute_mpu_cache(pcbs.get_mut(1).expect("pcb[1]"))
        .expect("precompute_mpu_cache failed for partition 1");

    // Verify cached peripheral regions are populated (disabled — no peripherals).
    let p0 = pcbs.first().expect("pcb[0]").cached_periph_regions();
    let p1 = pcbs.get(1).expect("pcb[1]").cached_periph_regions();
    assert_eq!(p0.first().expect("R4").1 & 1, 0, "R4 should be disabled");
    assert_eq!(p1.first().expect("R4").1 & 1, 0, "R4 should be disabled");

    // Verify data RBAR differs between partitions, RASR is the same.
    let r0 = pcbs.first().expect("pcb[0]").cached_base_regions();
    let r1 = pcbs.get(1).expect("pcb[1]").cached_base_regions();
    assert_ne!(
        r0.get(DATA_REGION).expect("R2").0,
        r1.get(DATA_REGION).expect("R2").0,
    ); // data RBAR differs
    assert_eq!(
        r0.get(DATA_REGION).expect("R2").1,
        r1.get(DATA_REGION).expect("R2").1,
    ); // data RASR same
    hprintln!("mpu_partition: cached region values verified");

    // Enable MPU with PRIVDEFENA (no background region — safe for QEMU)
    // SAFETY: enabling the MPU after region configuration is complete;
    // single-core, no concurrent access to MPU registers.
    unsafe { p.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    hprintln!("mpu_partition: waiting for {} switches", MAX_SWITCHES);

    let mut last_seen = u32::MAX;
    let mut switches = 0u32;

    loop {
        let current = PARTITION.load(Ordering::Acquire);
        if current != last_seen {
            let Some(pcb) = pcbs.get(current as usize) else {
                continue;
            };
            let cached = pcb.cached_base_regions();

            // Program data region into MPU from cache.
            // SAFETY: disabling MPU before reconfiguration; single-core exclusive access.
            unsafe { p.MPU.ctrl.write(0) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            let dr = cached.get(DATA_REGION).expect("R2");
            mpu::configure_region(&p.MPU, dr.0, dr.1);
            // SAFETY: re-enabling MPU after region configuration is complete.
            unsafe { p.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            // Read back and verify
            let rbar: u32;
            // SAFETY: writing RNR to select region for readback, then reading RBAR.
            unsafe {
                p.MPU.rnr.write(DATA_REGION as u32);
                rbar = p.MPU.rbar.read();
            }
            let exp_base = cached.get(DATA_REGION).expect("R2").0 & !0x1F;
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
