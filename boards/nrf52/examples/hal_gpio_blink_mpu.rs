//! HAL GPIO Blink under MPU Enforcement — MPU pass-through nRF52833
//!
//! Proves that nrf52833-hal GPIO code can run from a kernel partition under
//! real MPU enforcement: the partition only has MPU access to its declared
//! peripheral_regions (P0 GPIO only) and its SRAM data window.  Access to any
//! other peripheral or kernel memory causes a MemManage fault.
//!
//! This is the nRF52833 equivalent of the f429zi `hal_gpio_blink_mpu` example.
//!
//! Key difference from STM32 MPU pass-through:
//!   - nRF52 GPIO is always-on — no clock gating, no RCC equivalent.
//!   - No bit-band alias region — nrf52833-hal uses no bit-band tricks.
//!   - p0::Parts::new(dp.P0) is zero-register-access (just wraps PhantomData).
//!   - No split_unchecked() needed — the standard HAL works cleanly under MPU.
//!   - No privileged setup in main() beyond the kernel boot sequence.
//!
//! MPU region layout after `__boot_mpu_init`:
//!   R0: deny-all  (4 GiB, no-access, XN)           — background
//!   R1: flash RX  (0x0000_0000, 256 KB)             — instruction fetch + .rodata strings
//!   R2: SRAM RW   (0x2000_0000, 256 KB)             — data + stack (actual SRAM is 128 KB)
//!   R3: stack guard (stack_base, 32 B)              — overflow sentinel
//!   R4: P0 GPIO   (0x5000_0000,   4 KB)             — Device, AP=full, XN
//!   R5: partition RAM (dynamic slot)
//!
//! Hardware: PCA10100 (nRF52833-DK)
//!   LED1 = P0.13 (active-low)
//!   LED2 = P0.14 (active-low)
//!   LED3 = P0.15 (active-low)
//!   LED4 = P0.16 (active-low)
//!
//! Build:  cd nrf52 && cargo build --example hal_gpio_blink_mpu --features kernel-mpu-hal
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: LEDs blink at ~1 Hz; RTT shows toggle_count and MPU CTRL=0x5.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use embedded_hal::digital::OutputPin;
use kernel::{PartitionSpec,
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib;
use nrf52833_hal::{
    gpio::{p0, Level},
    pac,
};
use rtt_target::{rprintln, rtt_init_print};
use nrf52::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;
const BLINK_EVENT: u32 = 0x1;
const P0_PART: usize = 0;

// nRF52833 flash starts at 0x0000_0000 (FLASH_BASE from nrf52 lib).
// Code window (R1): FLASH_BASE + SRAM_SIZE (256 KB) — covers all flash including .rodata.
//   Debug builds put panic/.rodata past 128KB; 256KB ensures nothing spills out.
// Data window (R2): SRAM_BASE + SRAM_SIZE (256 KB) — covers 128KB SRAM + kernel structs + RTT.
//   (Actual SRAM is 128KB; granting 256KB is fine — extra grant is beyond physical RAM.)

static TOGGLE_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(HalMpuConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    stack_words = STACK_WORDS;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(HalMpuConfig, |tick, k| {
    // Send BLINK_EVENT to P0 every 500 ms.
    if tick % 500 == 0 {
        kernel::events::event_set(k.partitions_mut(), P0_PART, BLINK_EVENT);
        let tc = TOGGLE_COUNT.load(Ordering::Acquire);
        rprintln!("[{:5}ms] blink event — toggles={}", tick, tc);
        if tc >= 10 {
            rprintln!("✓ SUCCESS: HAL GPIO under MPU enforcement (nRF52833)! toggles={}", tc);
        }
    }

    // Dump MPU regions once on first tick to confirm peripheral wiring.
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU regions (after __boot_mpu_init) --- CTRL=0x{:x}", ctrl);
        for r in 0u32..7 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            rprintln!(
                "  R{}: RBAR=0x{:08x} RASR=0x{:08x} EN={}",
                r, rbar, rasr, rasr & 1
            );
        }
        rprintln!("---");
    }
});

/// P0: steals GPIO P0 via nrf52833-hal, blinks LEDs on BLINK_EVENT.
///
/// Running under MPU enforcement — can only access:
///   - SRAM (0x2000_0000 + 128 KB): own stack, statics, kernel IPC structures
///   - P0 GPIO (0x5000_0000 + 4 KB): LED control
/// Any access outside these ranges → MemManage fault.
///
/// Unlike STM32 MPU pass-through, no split_unchecked() is needed here because:
///   - p0::Parts::new(dp.P0) writes no registers (pure PhantomData wrapper)
///   - nRF52 has no clock gating and no bit-band alias
extern "C" fn blinker_main_body(_r0: u32) -> ! {
    // SAFETY: this partition has exclusive MPU-enforced ownership of P0 GPIO.
    // The kernel grants only P0 to this partition.  No other peripheral is touched.
    let dp = unsafe { pac::Peripherals::steal() };

    // Parts::new() is zero-register-access — no clock enable, no bit-band.
    // This works directly in a partition with only P0 in peripheral_regions.
    let gpio = p0::Parts::new(dp.P0);

    // Configure LED pins as push-pull outputs, starting high (active-low = off).
    // into_push_pull_output() writes only to PIN_CNF[n] within P0 (0x5000_0700+).
    let mut led1 = gpio.p0_13.into_push_pull_output(Level::High); // LED1
    let mut led2 = gpio.p0_14.into_push_pull_output(Level::High); // LED2
    let mut led3 = gpio.p0_15.into_push_pull_output(Level::High); // LED3
    let mut led4 = gpio.p0_16.into_push_pull_output(Level::High); // LED4

    let mut leds_on = false;

    loop {
        // Block until the SysTick hook sends BLINK_EVENT.
        // This is the only syscall in the hot path; all LED toggling is
        // in-partition HAL code — no kernel involvement for GPIO I/O.
        match plib::sys_event_wait(BLINK_EVENT) {
            Err(_) | Ok(0) => continue,
            Ok(_) => {}
        }

        // Toggle all four LEDs — purely HAL register writes within P0 region.
        // Under MPU enforcement this is only allowed because P0 is in peripheral_regions.
        // set_low()/set_high() write to P0.OUTCLR/P0.OUTSET (0x5000_0508/0x5000_050C).
        if leds_on {
            led1.set_high().ok(); // active-low: high = off
            led2.set_high().ok();
            led3.set_high().ok();
            led4.set_high().ok();
        } else {
            led1.set_low().ok(); // active-low: low = on
            led2.set_low().ok();
            led3.set_low().ok();
            led4.set_low().ok();
        }
        leds_on = !leds_on;

        TOGGLE_COUNT.fetch_add(1, Ordering::Release);

        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(blinker_main => blinker_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== HAL GPIO Blink — MPU Enforcement (MPU pass-through nRF52833) ===");
    rprintln!("nRF52833 — nrf52833-hal GPIO from kernel partition under MPU isolation");
    rprintln!("No clock enable needed: nRF52 GPIO is always-on (no RCC equivalent)");
    // GPIO P0: 0x5000_0000, 4 KB (highest reg PIN_CNF[31] at +0x77C, fits in 4 KB ✓)
    let gpio_p0_base = pac::P0::PTR as usize as u32;
    rprintln!("Partition peripheral_regions: P0=0x{:08x}/4KB", gpio_p0_base);

    // Note: no clock enable write needed here — unlike STM32 where we had to
    // write RCC_AHB1ENR from main() before boot() armed the MPU.
    // nRF52833 GPIO P0 is in the always-on power domain.

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ HalMpuConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0"); // 4-tick window
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(blinker_main, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting — nrf52833-hal GPIO via Parts::new() inside partition\n");
    match boot(p).expect("boot") {}
}
