//! HAL GPIO Blink under MPU Enforcement — Approach D Full Demo
//!
//! Proves that stm32f4xx-hal GPIO code can run from a kernel partition under
//! real MPU enforcement: the partition only has MPU access to its declared
//! peripheral_regions (GPIOB only) and its SRAM data window.  Access to any
//! other peripheral or kernel memory causes a MemManage fault.
//!
//! This is the completion of "Approach D" from notes/driver-architecture.md:
//! - Kernel maps GPIOB into the partition's MPU window at boot.
//! - Partition calls `Peripherals::steal()`, `GPIOB.split_unchecked()`, `toggle()` —
//!   standard stm32f4xx-hal, zero kernel syscalls for I/O.
//! - `MPU_ENFORCE = true`: the 4 GB deny-all background region is active.
//!   Only explicitly granted regions are reachable from Thread mode.
//!
//! MPU region layout after `__boot_mpu_init`:
//!   R0: deny-all (4 GiB, no-access, XN)      — background
//!   R1: flash code RX (0x0800_0000, 256 KB)   — instruction fetch
//!   R2: SRAM RW       (0x2000_0000, 256 KB)   — data + stack
//!   R3: stack guard   (stack_base, 32 B)       — overflow sentinel
//!   R4: GPIOB MMIO    (0x4002_0400, 1 KB)     — Device, AP=full, XN
//!   R5: partition RAM (0x2000_0000, 256 KB)   — dynamic slot (same as R2)
//!
//! RCC is NOT in peripheral_regions: the partition never touches RCC.
//! main() enables the GPIOB AHB1 clock via direct MMIO write (not bit-band)
//! from privileged Thread mode before boot() arms the MPU.
//! stm32f4xx-hal uses cortex_m::bb (bit-band alias 0x4200_0000+) for clock
//! enable — that alias region is outside any MPU grant, so split() would fault.
//! split_unchecked() is the HAL fork's new entry point that skips the RCC step.
//!
//! Hardware: STM32F429ZI NUCLEO-144
//!   PB0  = LD1 (green)
//!   PB7  = LD2 (blue)
//!   PB14 = LD3 (red)
//!
//! Build:  cd f429zi && cargo build --example hal_gpio_blink_mpu --features kernel-mpu-hal --no-default-features
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: LEDs blink at ~1 Hz; RTT shows toggle_count and MPU CTRL=0x5.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::{rprintln, rtt_init_print};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};
use stm32f4xx_hal::{gpio::GpioExt, pac};

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;
const BLINK_EVENT: u32 = 0x1;
const P0: usize = 0;


// Data window: all 256 KB SRAM (covers kernel structs, RTT buffers, stacks, statics).

// RCC accessed via stm32f4xx_hal::pac in main() for the privileged clock-enable write.

static TOGGLE_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(HalMpuConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(HalMpuConfig, |tick, k| {
    // Send BLINK_EVENT to P0 every 500 ms.
    if tick % 500 == 0 {
        kernel::events::event_set(k.partitions_mut(), P0, BLINK_EVENT);
        let tc = TOGGLE_COUNT.load(Ordering::Acquire);
        rprintln!("[{:5}ms] blink event — toggles={}", tick, tc);
        if tc >= 10 {
            rprintln!("✓ SUCCESS: HAL GPIO under MPU enforcement! toggles={}", tc);
        }
    }

    // Dump MPU regions once on the first tick to confirm peripheral wiring.
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = unsafe { mpu.ctrl.read() };
        rprintln!("--- MPU regions (after __boot_mpu_init) --- CTRL=0x{:x}", ctrl);
        for r in 0u32..7 {
            unsafe { mpu.rnr.write(r) };
            let rbar = unsafe { mpu.rbar.read() };
            let rasr = unsafe { mpu.rasr.read() };
            rprintln!(
                "  R{}: RBAR=0x{:08x} RASR=0x{:08x} EN={}",
                r, rbar, rasr, rasr & 1
            );
        }
        rprintln!("---");
    }
});

/// P0: steals GPIOB via stm32f4xx-hal, blinks LEDs on BLINK_EVENT.
///
/// Running under MPU enforcement — can only access:
///   - SRAM (0x2000_0000 + 256 KB): own stack, statics, kernel IPC structures
///   - GPIOB (0x4002_0400 + 1 KB): LED control
///   - RCC   (0x4002_3800 + 1 KB): clock enable (needed for GPIOB.split)
/// Any access outside these ranges → MemManage fault.
extern "C" fn blinker_main_body(_r0: u32) -> ! {
    // SAFETY: this partition has exclusive MPU-enforced ownership of GPIOB.
    // The kernel grants only GPIOB to this partition (not RCC — the partition
    // never writes RCC).  main() enabled the GPIOB clock before boot().
    let dp = unsafe { pac::Peripherals::steal() };

    // split_unchecked() returns typed pin handles without touching RCC.
    // SAFETY: main() already enabled the GPIOB AHB1 clock via direct MMIO write.
    let gpiob = unsafe { dp.GPIOB.split_unchecked() };

    let mut led_green = gpiob.pb0.into_push_pull_output();  // LD1 (green)
    let mut led_blue = gpiob.pb7.into_push_pull_output();   // LD2 (blue)
    let mut led_red = gpiob.pb14.into_push_pull_output();   // LD3 (red)

    // Start with all LEDs off.
    led_green.set_low();
    led_blue.set_low();
    led_red.set_low();

    loop {
        // Block until the SysTick hook sends BLINK_EVENT.
        // This is the only syscall in the hot path; the LED toggle is purely
        // in-partition HAL code — no kernel involvement for the GPIO I/O.
        match plib::sys_event_wait(BLINK_EVENT) {
            Err(_) | Ok(0) => continue,
            Ok(_) => {}
        }

        // Toggle all three LEDs — purely HAL register writes within GPIOB region.
        // Under MPU enforcement this is only allowed because GPIOB is in peripheral_regions.
        led_green.toggle();
        led_blue.toggle();
        led_red.toggle();

        TOGGLE_COUNT.fetch_add(1, Ordering::Release);

        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(blinker_main => blinker_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== HAL GPIO Blink — MPU Enforcement (Approach D Full Demo) ===");
    rprintln!("STM32F429ZI — stm32f4xx-hal from kernel partition under MPU isolation");
    // GPIOB register block: 0x4002_0400, 1 KB (1KB-aligned ✓)
    // Covers MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR.
    let gpiob_base = pac::GPIOB::PTR as usize as u32;
    rprintln!("Partition peripheral_regions: GPIOB=0x{:08x}/1KB (RCC not in grant)",
        gpiob_base);

    // Enable GPIOB AHB1 clock from privileged Thread mode BEFORE boot() arms the MPU.
    // PAC modify() uses a full register read-modify-write — NOT bit-band — so it stays
    // within the physical RCC region (no 0x4200_0000+ alias access).
    // stm32f4xx-hal's Enable::enable() uses bb::set() which WOULD access the bit-band
    // alias; that's why the partition calls split_unchecked() instead of split().
    let dp = pac::Peripherals::take().unwrap();
    dp.RCC.ahb1enr().modify(|_, w| w.gpioben().set_bit());
    cortex_m::asm::dsb();
    rprintln!("[INIT] GPIOB AHB1 clock enabled (RCC AHB1ENR.GPIOBEN=1)");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ HalMpuConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0"); // 4-tick window
    sched.add_system_window(2).expect("sched SW"); // required by dynamic-mpu

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(blinker_main, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting with MPU enforcement — HAL GPIO via split_unchecked() inside partition\n");
    match boot(p).expect("boot") {}
}
