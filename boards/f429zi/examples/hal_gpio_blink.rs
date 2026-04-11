//! HAL GPIO Blink — stm32f4xx-hal from a Kernel Partition (Approach D)
//!
//! Validates that stm32f4xx-hal's GPIO API runs correctly from inside a kernel
//! partition using Peripherals::steal().  The partition calls steal() once at
//! init, splits GPIOB using the HAL, then toggles LEDs in response to a
//! SysTick-driven event — zero syscalls for the GPIO I/O path.
//!
//! This is the minimal proof-of-concept for Approach D (MPU peripheral
//! pass-through): the kernel maps nothing here (MPU_ENFORCE = false), but the
//! ownership model is correct — one partition owns one peripheral bank.
//!
//! Hardware: STM32F429ZI NUCLEO-144
//!   PB0  = LD1 (green)
//!   PB7  = LD2 (blue)
//!   PB14 = LD3 (red)
//!
//! Compare with gpio_blinky.rs which does the same thing via raw register writes.
//!
//! Build:  cd f429zi && cargo build --example hal_gpio_blink --features kernel-hal
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: LEDs blink at ~1 Hz; RTT shows toggle_count incrementing.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{gpio::GpioExt, pac};

const NUM_PARTITIONS: usize = 1;
const BLINK_EVENT: u32 = 0x1;
const P0: usize = 0;

static TOGGLE_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(HalBlinkConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 4;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(HalBlinkConfig, |tick, k| {
    if tick % 500 == 0 {
        kernel::events::event_set(k.partitions_mut(), P0, BLINK_EVENT);
        let tc = TOGGLE_COUNT.load(Ordering::Acquire);
        rprintln!("[{:4}ms] blink event — toggle_count={}", tick, tc);
        if tc >= 10 {
            rprintln!("✓ SUCCESS: HAL GPIO blink working! toggles={}", tc);
        }
    }
});

/// Partition P0: steals GPIOB via HAL, blinks LEDs on each BLINK_EVENT.
extern "C" fn blinker_main_body(_r0: u32) -> ! {
    // Steal device peripherals. The kernel holds cortex_m::Peripherals (for SysTick/NVIC);
    // device peripherals (GPIOB, USART, ...) are untouched by the kernel so steal() is safe.
    // SAFETY: this partition has exclusive ownership of GPIOB — no other code touches it.
    let mut dp = unsafe { pac::Peripherals::steal() };

    // split() enables the GPIOB AHB1 clock via RCC internally, then returns
    // typed pin handles.  Takes &mut dp.RCC (PAC type, not HAL Rcc wrapper).
    let gpiob = dp.GPIOB.split(&mut dp.RCC);

    let mut led_green = gpiob.pb0.into_push_pull_output();  // LD1
    let mut led_blue = gpiob.pb7.into_push_pull_output();   // LD2
    let mut led_red = gpiob.pb14.into_push_pull_output();   // LD3

    // Start with all LEDs off.
    led_green.set_low();
    led_blue.set_low();
    led_red.set_low();

    loop {
        // Block until the SysTick hook sends BLINK_EVENT (every 500ms).
        match plib::sys_event_wait(BLINK_EVENT) {
            Err(_) | Ok(0) => continue,
            Ok(_) => {}
        }

        // Toggle all three LEDs — inherent method, returns ().
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
    rprintln!("\n=== HAL GPIO Blink — stm32f4xx-hal from kernel partition ===");
    rprintln!("LEDs: PB0 (green) | PB7 (blue) | PB14 (red)");
    rprintln!("HAL GPIO init happens inside the partition via Peripherals::steal()");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ HalBlinkConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(blinker_main, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting — HAL GPIO init will run inside partition\n");
    match boot(p).expect("boot") {}
}
