//! Split-ISR UART Demo — STM32F429ZI
//!
//! Demonstrates the split-ISR interrupt model on real hardware:
//!   Top half (ISR):    reads USART3 DR, pushes byte into StaticIsrRing
//!   Bottom half (P0):  drains ring buffer, validates data, counts bytes
//!
//! Unlike uart_partition (Model B, partition reads DR directly), the
//! partition NEVER touches USART3 registers. All hardware interaction
//! happens in the ISR; the partition only sees ring buffer data.
//!
//! Host sends incrementing bytes over ST-LINK VCP (USART3 PD8/PD9).
//! RTT reports RX_COUNT + ring overflow stats every 500ms.
//!
//! Build:  cd f429zi && cargo build --example split_isr_uart \
//!             --features kernel-irq --no-default-features --release
//! Flash:  probe-rs or GDB+OpenOCD
//! Host:   python3 -c "
//!           import serial, time
//!           s = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
//!           for i in range(1000):
//!               s.write(bytes([i & 0xFF]))
//!               time.sleep(0.001)
//!           s.close()
//!         "

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    split_isr::StaticIsrRing,
    {DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal},
};
use plib::EventMask;
use rtt_target::rprintln;
use f429zi as _;

// ── USART3 registers (PAC-free, raw addresses) ──
const USART3_SR: *const u32 = 0x4000_4800 as *const u32;
const USART3_DR: *const u32 = 0x4000_4804 as *const u32;
const USART3_BRR: *mut u32 = 0x4000_4808 as *mut u32;
const USART3_CR1: *mut u32 = 0x4000_480C as *mut u32;

// ── RCC registers ──
const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
const RCC_APB1ENR: *mut u32 = 0x4002_3840 as *mut u32;

// ── GPIO D registers ──
const GPIOD_MODER: *mut u32 = 0x4002_0C00 as *mut u32;
const GPIOD_AFRH: *mut u32 = 0x4002_0C24 as *mut u32;

const USART3_IRQ: u8 = 39;
const UART_RX_EVENT: EventMask = EventMask::new(0x01);
const HSI_HZ: u32 = 16_000_000;
const BAUD: u32 = 115_200;

// ── Counters ──
static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static PUSH_FAIL: AtomicU32 = AtomicU32::new(0);
static ISR_COUNT: AtomicU32 = AtomicU32::new(0);

// ── Ring buffer: 16 slots, 1 byte payload ──
static RING: StaticIsrRing<16, 1> = StaticIsrRing::new();

// ── Kernel config ──
kernel::kernel_config!(SplitIsrCfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
});

kernel::bind_interrupts!(SplitIsrCfg, 91,
    USART3_IRQ => (0, 0x01, handler: usart3_rx_isr),
);

kernel::define_kernel!(SplitIsrCfg, |tick, _k| {
    if tick % 500 == 0 {
        let rx = RX_COUNT.load(Ordering::Acquire);
        let isr = ISR_COUNT.load(Ordering::Acquire);
        let fail = PUSH_FAIL.load(Ordering::Acquire);
        kernel::klog!("[{:5}ms] RX={} ISR={} RING_OVERFLOW={}", tick, rx, isr, fail);
        if rx >= 100 && fail == 0 {
            kernel::klog!("SUCCESS: split-ISR pipeline verified! RX={}", rx);
        }
    }
});

// ── Top half: ISR reads DR, pushes into ring, signals partition ──
unsafe extern "C" fn usart3_rx_isr() {
    let sr = unsafe { core::ptr::read_volatile(USART3_SR) };
    if sr & (1 << 5) != 0 {
        // RXNE set — read DR (clears RXNE + de-asserts IRQ)
        let byte = unsafe { core::ptr::read_volatile(USART3_DR) } as u8;
        ISR_COUNT.fetch_add(1, Ordering::Relaxed);

        // Push into ring buffer
        if unsafe { RING.push_from_isr(USART3_IRQ, &[byte]) }.is_err() {
            PUSH_FAIL.fetch_add(1, Ordering::Relaxed);
        }
    }
    // ORE (overrun) — read SR then DR to clear
    if sr & (1 << 3) != 0 {
        let _ = unsafe { core::ptr::read_volatile(USART3_DR) };
    }

    // Signal partition + mask IRQ (kernel dispatch does mask, but we
    // signal explicitly since we're using a custom handler)
    kernel::irq_dispatch::signal_partition_from_isr::<SplitIsrCfg>(0, 0x01);
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(USART3_IRQ));
}

// ── Bottom half: partition drains ring buffer ──
extern "C" fn uart_partition_body(_r0: u32) -> ! {
    let mut expected: u8 = 0;
    let mut mismatch: u32 = 0;

    loop {
        // Wait for ISR to signal us
        if plib::sys_event_wait(UART_RX_EVENT).is_err() {
            plib::sys_yield().ok();
            continue;
        }

        // Drain all available bytes from ring
        loop {
            let mut got_byte = false;
            let popped = unsafe {
                RING.pop_with(|tag, data| {
                    if tag == USART3_IRQ && data.len() == 1 {
                        let byte = data[0];
                        if byte != expected {
                            mismatch += 1;
                        }
                        expected = byte.wrapping_add(1);
                        RX_COUNT.fetch_add(1, Ordering::Release);
                        got_byte = true;
                    }
                })
            };
            if !popped || !got_byte {
                break;
            }
        }

        // Clear event + re-arm IRQ
        plib::sys_event_clear(UART_RX_EVENT).ok();
        plib::sys_irq_ack(USART3_IRQ).ok();
    }
}

kernel::partition_trampoline!(uart_partition => uart_partition_body);

// ── USART3 init: PD9 RX only (no TX needed — host sends, we receive) ──
fn init_usart3_rx() {
    unsafe {
        // Enable GPIOD + USART3 clocks
        let ahb1 = core::ptr::read_volatile(RCC_AHB1ENR);
        core::ptr::write_volatile(RCC_AHB1ENR, ahb1 | (1 << 3)); // GPIODEN
        let apb1 = core::ptr::read_volatile(RCC_APB1ENR);
        core::ptr::write_volatile(RCC_APB1ENR, apb1 | (1 << 18)); // USART3EN

        // PD9 → AF7 (USART3_RX)
        let moder = core::ptr::read_volatile(GPIOD_MODER);
        core::ptr::write_volatile(GPIOD_MODER, (moder & !(3 << 18)) | (2 << 18));
        let afrh = core::ptr::read_volatile(GPIOD_AFRH);
        core::ptr::write_volatile(GPIOD_AFRH, (afrh & !(0xF << 4)) | (7 << 4));

        // USART3: 115200 8N1, RX only, RXNEIE enabled
        core::ptr::write_volatile(USART3_BRR, HSI_HZ / BAUD);
        // UE + RE + RXNEIE (no TE — we only receive)
        core::ptr::write_volatile(USART3_CR1, (1 << 13) | (1 << 2) | (1 << 5));
    }
}

#[entry]
fn main() -> ! {
    rprintln!("=== Split-ISR UART Demo — STM32F429ZI ===");
    rprintln!("ISR reads DR → ring buffer → partition drains");
    rprintln!("Host: send bytes on ST-LINK VCP (/dev/ttyACM1) at 115200");

    let mut p = cortex_m::Peripherals::take().unwrap();

    init_usart3_rx();
    rprintln!("[INIT] USART3 RX configured (PD9, 115200, RXNEIE)");

    let mut sched = ScheduleTable::<{ SplitIsrCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    enable_bound_irqs(&mut p.NVIC, SplitIsrCfg::IRQ_DEFAULT_PRIORITY)
        .expect("enable_bound_irqs");
    rprintln!("[INIT] Kernel created, IRQs enabled — booting split-ISR pipeline");

    let parts: [PartitionSpec; 1] = [PartitionSpec::entry(uart_partition)];
    match boot(p).expect("boot") {}
}
