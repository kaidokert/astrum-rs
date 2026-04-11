//! Timer IRQ Demo — STM32F429ZI
//!
//! Demonstrates kernel-mediated hardware interrupt notification:
//! TIM6 fires at 5 Hz; the ISR clears the hardware pending flag and calls
//! `signal_partition_from_isr`, which wakes partition P0 via a kernel event.
//! P0 blocks on `SYS_EVT_WAIT` between interrupts — no polling, no SYS_YIELD
//! spin — true low-latency ISR → partition event delivery.
//!
//! All TIM6 and RCC registers accessed via the stm32f4 PAC (typed field
//! accessors). ISR uses Peripherals::steal() — the standard pattern for
//! ISR register access when the peripheral singleton lives in main().
//!
//! Clock: 16 MHz HSI (default, no PLL).
//! TIM6:  PSC=15999 → 1 kHz tick; ARR=199 → 200 ms period → 5 Hz.
//!
//! Build:  cargo build --example timer_irq_demo --features kernel-irq --no-default-features
//! Flash:  (via GDB + OpenOCD)

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    irq_dispatch::signal_partition_from_isr,
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugEnabled, MsgSmall, Partitions4, PortsTiny, SyncMinimal},
};
// Referencing the PAC Interrupt enum forces the stm32f4 rlib into the link,
// which drags in the 91-entry __INTERRUPTS array and device.x PROVIDE directives.
use stm32f4::stm32f429::Interrupt;
use rtt_target::{rprintln, rtt_init_print};

// All TIM6 and RCC registers accessed via stm32f4 PAC (dp.TIM6.*, dp.RCC.*).

const NUM_PARTITIONS: usize = 1;

/// Event bit used to wake P0 from the TIM6 ISR.
const TIMER_EVENT: u32 = 0x0000_0001;

/// Number of TIM6 interrupts received by P0.
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(TimerConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(TimerConfig, |tick, _k| {
    if tick % 500 == 0 {
        let count = IRQ_COUNT.load(Ordering::Acquire);
        rprintln!("[{:5}ms] IRQ_COUNT={}", tick, count);
        if count > 10 {
            rprintln!("✓ SUCCESS: Timer IRQ working! count={}", count);
        }
    }
});

/// P0: blocks on TIMER_EVENT until TIM6 ISR fires, then counts and loops.
extern "C" fn timer_task_body(_r0: u32) -> ! {
    loop {
        // Block until TIM6 ISR signals TIMER_EVENT on this partition.
        if let Ok(bits) = plib::sys_event_wait(TIMER_EVENT) {
            if bits & TIMER_EVENT != 0 {
                IRQ_COUNT.fetch_add(1, Ordering::Release);
            }
        }
    }
}

kernel::partition_trampoline!(timer_task => timer_task_body);

/// TIM6_DAC interrupt handler.
///
/// Overrides the DefaultHandler from device.x via `#[no_mangle]`.
/// Clears the Update Interrupt Flag in TIM6->SR then signals P0 via
/// the kernel event mechanism.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "C" fn TIM6_DAC() {
    // Clear UIF via PAC. Peripherals::steal() is the standard pattern for
    // ISR register access — the ownership singleton model doesn't work across
    // the ISR boundary, and steal() is explicitly provided for this use case.
    // Safety: TIM6 SR is only ever touched from main() setup (before NVIC
    // unmask) and this ISR — no concurrent access is possible.
    unsafe { stm32f4::stm32f429::Peripherals::steal() }
        .TIM6.sr.write(|w| w.uif().clear());
    // Notify P0.  If P0 is Waiting, this transitions it to Ready and pends
    // PendSV; if P0 is already Running (e.g., due to the single-partition
    // spin-wait path), the event bits are set and P0 consumes them on its
    // next SYS_EVT_WAIT call.
    signal_partition_from_isr::<TimerConfig>(0, TIMER_EVENT);
}

#[entry]
fn main() -> ! {
    rprintln!("\n=== Timer IRQ Demo — TIM6 5 Hz → kernel partition wakeup ===");

    let dp = stm32f4::stm32f429::Peripherals::take().unwrap();

    // Enable TIM6 APB1 clock via PAC.
    dp.RCC.apb1enr.modify(|_, w| w.tim6en().set_bit());

    // Configure TIM6.
    // At 16 MHz HSI, APB1 prescaler = 1 (reset default) → TIM6 input = 16 MHz.
    // PSC=15999: counter tick = 16 MHz / 16000 = 1 kHz.
    // ARR=199:   overflow at 200 counts = 200 ms → 5 Hz update event.
    //
    // Proper init sequence (RM0090 §18.4.7):
    //   1. Write PSC + ARR while CEN=0.
    //   2. Write EGR.UG=1 to immediately load PSC into its shadow register.
    //      (Without UG, the very first period uses PSC=0 → ~12 µs overflow.)
    //   3. Clear SR.UIF — UG generates a spurious update event.
    //   4. Enable UIE, then start the counter (CEN=1).
    //
    // NVIC is NOT unmasked yet — the kernel must be initialised first.
    dp.TIM6.psc.write(|w| w.psc().bits(15999)); // prescaler: divide by 16000
    dp.TIM6.arr.write(|w| w.arr().bits(199)); // auto-reload: 200 ticks
    dp.TIM6.egr.write(|w| w.ug().set_bit()); // UG: force prescaler shadow load
    dp.TIM6.sr.write(|w| w.uif().clear()); // clear UIF generated by UG
    dp.TIM6.dier.write(|w| w.uie().set_bit()); // UIE: enable update interrupt
    dp.TIM6.cr1.write(|w| w.cen().set_bit()); // CEN: start counter

    rprintln!("TIM6 configured: 5 Hz, NVIC masked until kernel ready");

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ TimerConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(timer_task, 0)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Unmask TIM6_DAC only after the kernel is stored — the ISR calls
    // signal_partition_from_isr which accesses kernel state via with_kernel_mut.
    // Unmasking before store_kernel would let the ISR fire on uninitialised storage.
    //
    // Using Interrupt::TIM6_DAC (PAC type) instead of IrqNr(54) because:
    // (1) it is type-safe, and (2) referencing the PAC type is what forces the
    // stm32f4 rlib into the link, pulling in the 91-entry __INTERRUPTS array.
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DAC) };
    rprintln!("NVIC unmasked — TIM6 interrupts live");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
