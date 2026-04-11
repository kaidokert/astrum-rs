//! Kernel GPIO Blinky — First Kernel-Integrated Hardware Driver
//!
//! Demonstrates GPIO control from a kernel partition using event-based timing:
//! - P0 (blinker): waits for SysTick event (bit 0x1), then toggles all 3 LEDs
//! - Tick hook: every 500 ticks (500ms at 1ms/tick), calls event_set(P0, 0x1)
//!
//! GPIO setup (STM32F429ZI NUCLEO-144):
//! - PB0  = LD1 (Green)
//! - PB7  = LD2 (Blue)
//! - PB14 = LD3 (Red)
//!
//! Since the MPU is not enabled in kernel-example mode, partitions can
//! access peripheral registers directly via volatile reads/writes.
//!
//! This validates the freshly-fixed EventWait blocking path on real hardware:
//! P0 truly blocks in Waiting state between events rather than busy-polling.
//!
//! Build:  cd f429zi && cargo build --example gpio_blinky --features kernel-irq --no-default-features
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: LED visibly blinks at ~1 Hz; RTT shows toggle_count incrementing.

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
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here, it would cause a duplicate panic handler link error.
use stm32f4::stm32f429::Peripherals;
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 1;

/// LED pin bitmask: PB0 | PB7 | PB14
const LED_MASK: u32 = (1 << 0) | (1 << 7) | (1 << 14);

/// Event bit the tick hook sets every 500ms
const BLINK_EVENT: u32 = 0x1;

/// Partition 0 ID (used for event_wait caller and event_set target)
const P0: usize = 0;

// Progress counter readable from GDB and RTT
static TOGGLE_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(BlinkConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 4;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(BlinkConfig, |tick, k| {
    // Set BLINK_EVENT on P0 every 500 ticks (500ms at 1ms/tick).
    // This wakes P0 from its EVT_WAIT blocking state.
    if tick % 500 == 0 {
        kernel::events::event_set(k.partitions_mut(), P0, BLINK_EVENT);
        let tc = TOGGLE_COUNT.load(Ordering::Acquire);
        rprintln!("[{:4}ms] blink event sent — toggle_count={}", tick, tc);
    }
});

/// P0: waits for the 500ms blink event, then toggles the LEDs.
///
/// r0 hint = own partition id (P0 = 0)
extern "C" fn blinker_main_body(r0: u32) -> ! {
    let own_pid = r0; // = 0

    loop {
        // Block until the tick hook sets BLINK_EVENT every 500ms.
        // Ok(0) = was blocked (just woke up); Ok(bits) = immediate path.
        match plib::sys_event_wait(BLINK_EVENT) {
            Err(_) | Ok(0) => continue,
            Ok(_) => {}
        }

        // Event consumed — toggle LEDs.
        // SAFETY: no MPU in kernel-irq mode; steal() is fine for partition access.
        unsafe { Peripherals::steal() }
            .GPIOB.odr.modify(|r, w| unsafe { w.bits(r.bits() ^ LED_MASK) });
        let tc = TOGGLE_COUNT.fetch_add(1, Ordering::Release) + 1;
        let _ = tc;

        // Yield to allow other work (none in this demo, but good practice).
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(blinker_main => blinker_main_body);

/// Configure GPIOB pins PB0, PB7, PB14 as push-pull outputs.
///
/// Must be called from privileged mode (before `boot()`).
fn gpio_init(dp: &Peripherals) {
    // Enable GPIOB clock then wait for it to stabilise.
    dp.RCC.ahb1enr.modify(|_, w| w.gpioben().set_bit());
    for _ in 0..8 { core::hint::spin_loop(); }
    // Set PB0, PB7, PB14 to general-purpose output mode.
    dp.GPIOB.moder.modify(|_, w| {
        w.moder0().output();
        w.moder7().output();
        w.moder14().output()
    });
    // Start with all LEDs off.
    dp.GPIOB.odr.modify(|r, w| unsafe { w.bits(r.bits() & !LED_MASK) });
}

#[entry]
fn main() -> ! {
    rprintln!("\n=== Kernel GPIO Blinky — P0 blinks LEDs via EventWait ===");
    rprintln!("LEDs: PB0 (green) | PB7 (blue) | PB14 (red)");

    let dp = Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    // Configure GPIO before handing off to the kernel.
    gpio_init(&dp);
    rprintln!("[INIT] GPIO configured");

    // Single partition; schedule gives it the full 2-tick window.
    let mut sched = ScheduleTable::<{ BlinkConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(blinker_main), // hint = own_pid = 0
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting blinker partition\n");
    match boot(p).expect("boot") {}
}
