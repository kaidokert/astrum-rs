//! QEMU end-to-end demo: LM3S6965 UART0 split-ISR pipeline.
//!
//! Custom UART0 RX ISR reads the data register via `read_volatile`, pushes
//! tagged data into `IsrRingBuffer`, and signals P0.  The partition bottom-half
//! drains the ring buffer, validates received data, and calls `SYS_IRQ_ACK`.
//! `NVIC::pend()` from the SysTick hook simulates UART RX events.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::split_isr::IsrRingBuffer;
use kernel::svc::{Kernel, SvcError};
use kernel::syscall::{SYS_EVT_WAIT, SYS_IRQ_ACK};
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};

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

kernel::compose_kernel_config!(
    Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// Ring buffer: 4 slots, 1 byte payload (one UART data byte each).
///
/// ## Static mut usage
///
/// `RING` uses `static mut` because `IsrRingBuffer` must reside at a fixed
/// address shared between the ISR (producer, `uart0_rx_isr`) and the partition
/// bottom-half (consumer, `p0_body`).  On single-core Cortex-M, the ISR masks
/// the UART IRQ before returning and the partition unmasks it only after
/// draining, so concurrent access cannot occur.  All accesses use
/// `core::ptr::addr_of_mut!` to avoid creating references to the `static mut`.
static mut RING: IsrRingBuffer<4, 1> = IsrRingBuffer::new();
static POP_COUNT: AtomicU32 = AtomicU32::new(0);

#[allow(dead_code)]
unsafe extern "C" fn uart0_rx_isr() {
    // SAFETY: UART0_DR (0x4000_C000) is the memory-mapped Data Register for
    // LM3S6965 UART0.  The read is naturally aligned (4-byte address, u32) and
    // has no side-effects beyond dequeueing one byte from the RX FIFO.
    let dr_val = unsafe { core::ptr::read_volatile(UART0_DR as *const u32) };
    let byte = (dr_val & 0xFF) as u8;
    // SAFETY: Single-core Cortex-M — this ISR is the sole producer of RING.
    // The consumer (p0_body) only drains while the UART IRQ is masked, so no
    // concurrent access is possible.  `addr_of_mut!` avoids creating a
    // reference to the `static mut`.
    let _ = unsafe { (*core::ptr::addr_of_mut!(RING)).push_from_isr(UART0_IRQ, &[byte]) };
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<Cfg>(0, 0x01);
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(UART0_IRQ));
}

kernel::bind_interrupts!(Cfg, 70,
    5 => (0, 0x01, handler: uart0_rx_isr),
);

kernel::define_unified_harness!(Cfg, |tick, _k| {
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
    if tick >= 10 && POP_COUNT.load(Ordering::Acquire) >= 2 {
        hprintln!("uart_demo: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 16 {
        hprintln!(
            "uart_demo: FAIL pop_count={}",
            POP_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        let rc = kernel::svc!(SYS_EVT_WAIT, 0u32, 0x01u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("uart_demo: FAIL evt_wait 0x{:08X}", rc);
            debug::exit(debug::EXIT_FAILURE);
        }
        // Drain the ring buffer while IRQ is masked.
        loop {
            let mut ok = false;
            // SAFETY: UART0 IRQ is masked by the ISR after pushing, so the
            // ISR cannot preempt this consumer path.  `addr_of_mut!` avoids
            // creating a reference to the `static mut`.
            let popped = unsafe {
                (*core::ptr::addr_of_mut!(RING)).pop_with(|tag, data| {
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
        let rc = kernel::svc!(SYS_IRQ_ACK, UART0_IRQ as u32, 0u32, 0u32);
        if SvcError::is_error(rc) {
            hprintln!("uart_demo: FAIL irq_ack 0x{:08X}", rc);
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
    let cfgs = PartitionConfig::sentinel_array::<1>(Cfg::STACK_WORDS);
    let k = Kernel::<Cfg>::create(sched, &cfgs).expect("kernel");
    store_kernel(k);
    enable_bound_irqs(&mut p.NVIC, Cfg::IRQ_DEFAULT_PRIORITY);
    let parts: [(extern "C" fn() -> !, u32); 1] = [(p0_main, 0)];
    match boot(&parts, &mut p).expect("boot") {}
}
