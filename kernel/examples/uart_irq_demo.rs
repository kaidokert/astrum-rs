//! QEMU end-to-end demo: LM3S6965 UART0 split-ISR pipeline.
//!
//! Custom UART0 RX ISR reads the data register via `read_volatile`, pushes
//! tagged data into `StaticIsrRing`, and signals P0.  The partition bottom-half
//! drains the ring buffer, validates received data, and calls `SYS_IRQ_ACK`.
//! `NVIC::pend()` from the SysTick hook simulates UART RX events.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::split_isr::StaticIsrRing;
use kernel::{
    DebugEnabled, IsrHandler, MsgMinimal, PartitionBody, PartitionEntry, PartitionSpec,
    Partitions1, PortsTiny, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

// ── LM3S6965 UART0 registers ──
const UART0_BASE: u32 = 0x4000_C000;
#[allow(clippy::identity_op)]
const UART0_DR: u32 = UART0_BASE + 0x000; // Data Register
#[allow(dead_code)]
const UART0_FR: u32 = UART0_BASE + 0x018; // Flag Register
#[allow(dead_code)]
const UART0_IMSC: u32 = UART0_BASE + 0x038; // Interrupt Mask Set/Clear
#[allow(dead_code)]
const UART0_ICR: u32 = UART0_BASE + 0x044; // Interrupt Clear Register
#[allow(dead_code)]
const UART0_MIS: u32 = UART0_BASE + 0x040; // Masked Interrupt Status
const UART0_IRQ: u8 = 5;

kernel::kernel_config!(
    Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Ring buffer: 4 slots, 1 byte payload (one UART data byte each).
static RING: StaticIsrRing<4, 1> = StaticIsrRing::new();
static POP_COUNT: AtomicU32 = AtomicU32::new(0);
static PUSH_FAIL: AtomicU32 = AtomicU32::new(0);

const _: IsrHandler = uart0_rx_isr;
/// Vector-table ISR: reads UART0 data register, pushes the byte into the
/// ring buffer, signals P0, and masks the NVIC line.
///
/// # Safety
///
/// Must only be called by hardware via the interrupt vector table. Assumes
/// single-core Cortex-M (sole producer for `RING`). UART0_DR must be a valid
/// memory-mapped register at the expected address.
#[allow(dead_code)]
unsafe extern "C" fn uart0_rx_isr() {
    // SAFETY: UART0_DR (0x4000_C000) is the memory-mapped Data Register for
    // LM3S6965 UART0.  The read is naturally aligned (4-byte address, u32) and
    // has no side-effects beyond dequeueing one byte from the RX FIFO.
    let dr_val = unsafe { core::ptr::read_volatile(UART0_DR as *const u32) };
    let byte = (dr_val & 0xFF) as u8;
    // SAFETY: single-core Cortex-M ISR — sole producer; see StaticIsrRing docs.
    if unsafe { RING.push_from_isr(UART0_IRQ, &[byte]) }.is_err() {
        PUSH_FAIL.fetch_add(1, Ordering::Relaxed);
    }
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<Cfg>(0, 0x01);
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(UART0_IRQ));
}

kernel::bind_interrupts!(Cfg, 70,
    5 => (0, 0x01, handler: uart0_rx_isr),
);

kernel::define_kernel!(Cfg, |tick, _k| {
    // Simulate two UART RX events via NVIC::pend.
    if tick == 2 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(UART0_IRQ));
        hprintln!("uart_demo: pended UART0 IRQ (tick {})", tick);
    }
    if tick == 6 {
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(UART0_IRQ));
        hprintln!("uart_demo: pended UART0 IRQ (tick {})", tick);
    }
    if tick >= 10 {
        if PUSH_FAIL.load(Ordering::Relaxed) > 0 {
            hprintln!("uart_demo: FAIL push_fail");
            debug::exit(debug::EXIT_FAILURE);
        }
        if POP_COUNT.load(Ordering::Acquire) >= 2 {
            hprintln!("uart_demo: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 16 {
        hprintln!(
            "uart_demo: FAIL pop_count={}",
            POP_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionBody = p0_body;
extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        if let Err(e) = plib::sys_event_wait(plib::EventMask::new(0x01)) {
            hprintln!("uart_demo: FAIL evt_wait {:?}", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        // Drain the ring buffer while IRQ is masked.
        loop {
            let mut ok = false;
            // SAFETY: UART0 IRQ is masked — no concurrent access.
            let popped = unsafe {
                RING.pop_with(|tag, data| {
                    // Validate: tag must be UART0 IRQ number.
                    if tag != UART0_IRQ {
                        hprintln!("uart_demo: FAIL tag={} exp={}", tag, UART0_IRQ);
                        debug::exit(debug::EXIT_FAILURE);
                    }
                    // Validate: payload is exactly 1 byte.
                    if data.len() != 1 {
                        hprintln!("uart_demo: FAIL data len={}", data.len());
                        debug::exit(debug::EXIT_FAILURE);
                    }
                    // TODO: data content is not verified because QEMU does not
                    // allow injecting bytes into the UART RX FIFO from software.
                    // The test validates ISR pipeline mechanics (pend → ISR →
                    // ring push → partition pop → IRQ ACK), not payload content.
                    ok = true;
                })
            };
            if !popped || !ok {
                break;
            }
            POP_COUNT.fetch_add(1, Ordering::Release);
        }
        // Unmask UART0 IRQ so the ISR can fire again.
        if let Err(e) = plib::sys_irq_ack(UART0_IRQ) {
            hprintln!("uart_demo: FAIL irq_ack {:?}", e);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("uart_demo: take");
    hprintln!("uart_demo: start");
    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(1, 3).expect("sched");
    let parts: [PartitionSpec; 1] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    init_kernel(sched, &parts).expect("kernel");
    enable_bound_irqs(&mut p.NVIC, Cfg::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("boot") {}
}
