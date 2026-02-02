#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::mpu;
use panic_semihosting as _;

/// Partition index toggled by SysTick. Starts at u32::MAX so the first
/// tick (setting it to 0) is detected as a change.
static PARTITION: AtomicU32 = AtomicU32::new(u32::MAX);

/// Number of partition switches before exit.
const MAX_SWITCHES: u32 = 6;

/// SysTick reload: ~10 ms at 12 MHz.
const RELOAD: u32 = 120_000 - 1;

#[exception]
fn SysTick() {
    let prev = PARTITION.load(Ordering::Relaxed);
    // Toggle between 0 and 1 (first tick: MAX wraps to 0 via bitwise AND).
    let next = if prev == 0 { 1 } else { 0 };
    PARTITION.store(next, Ordering::Release);
}

/// Apply MPU regions for the given partition index (0 or 1).
///
/// Partition 0: region 0 = code RX @ 0x0000_0000, region 1 = data RW/XN @ 0x2000_0000
/// Partition 1: region 0 = code RX @ 0x0000_0000, region 1 = data RW/XN @ 0x2000_8000
fn apply_partition(mpu_periph: &cortex_m::peripheral::MPU, partition: u32) {
    let size_field = mpu::encode_size(256).unwrap();

    // Disable MPU while reconfiguring
    unsafe { mpu_periph.ctrl.write(0) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: code area (same for both partitions)
    let rbar0 = mpu::build_rbar(0x0000_0000, 0).unwrap();
    let rasr0 = mpu::build_rasr(size_field, mpu::AP_PRIV_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, rbar0, rasr0);

    // Region 1: data area (differs per partition)
    let data_base = if partition == 0 {
        0x2000_0000
    } else {
        0x2000_8000
    };
    let rbar1 = mpu::build_rbar(data_base, 1).unwrap();
    let rasr1 = mpu::build_rasr(size_field, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, rbar1, rasr1);

    // Re-enable MPU with PRIVDEFENA — privileged default memory map remains active
    // SAFETY: re-enabling the MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    // Configure SysTick
    let mut syst = p.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    hprintln!("partition_switch: waiting for {} switches", MAX_SWITCHES);

    let mut last_seen = u32::MAX;
    let mut switches = 0u32;

    loop {
        let current = PARTITION.load(Ordering::Acquire);
        if current != last_seen {
            apply_partition(&p.MPU, current);
            switches += 1;

            let data_base: u32 = if current == 0 {
                0x2000_0000
            } else {
                0x2000_8000
            };
            hprintln!(
                "switch {}: partition {} active (data @ {:#010x})",
                switches,
                current,
                data_base
            );

            last_seen = current;
            if switches >= MAX_SWITCHES {
                hprintln!("done");
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }
}
