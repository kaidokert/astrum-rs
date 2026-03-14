//! QEMU integration test: split-ISR ring buffer data path.  ISR pushes tagged
//! data into `IsrRingBuffer`; partition pops and verifies FIFO order.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::split_isr::StaticIsrRing;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal};
#[allow(clippy::single_component_path_imports)]
use plib;

kernel::compose_kernel_config!(
    Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

static RING: StaticIsrRing<4, 4> = StaticIsrRing::new();
static PAYLOAD_IDX: AtomicU32 = AtomicU32::new(0);
static POP_COUNT: AtomicU32 = AtomicU32::new(0);

#[allow(dead_code)]
unsafe extern "C" fn ring_buffer_isr() {
    let idx = PAYLOAD_IDX.load(Ordering::Relaxed) as u8;
    let tag = idx.wrapping_add(1);
    // SAFETY: single-core Cortex-M ISR — sole producer; see StaticIsrRing docs.
    let _ = unsafe { RING.push_from_isr(tag, &[0xA0 | idx, idx, idx, idx]) };
    // handler: form bypasses __irq_dispatch, so manual signal + mask are
    // required (not redundant).
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<Cfg>(0, 0x01);
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(0));
}

// 70 is the IVT $count, not IPSR; the IRQ number ($irq) is 0, matching
// the IrqNr(0) NVIC calls below.
kernel::bind_interrupts!(Cfg, 70, 0 => (0, 0x01, handler: ring_buffer_isr));

kernel::define_unified_harness!(Cfg, |tick, _k| {
    if tick == 2 {
        PAYLOAD_IDX.store(0, Ordering::Relaxed);
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("rb_test: pended IRQ 0 (payload 0)");
    }
    if tick == 6 {
        PAYLOAD_IDX.store(1, Ordering::Relaxed);
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("rb_test: pended IRQ 0 (payload 1)");
    }
    if tick >= 10 && POP_COUNT.load(Ordering::Acquire) >= 2 {
        hprintln!("rb_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 16 {
        hprintln!(
            "rb_test: FAIL pop_count={}",
            POP_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});
extern "C" fn p0_body(_r0: u32) -> ! {
    let mut next_tag: u8 = 1;
    loop {
        if let Err(e) = plib::sys_event_wait(plib::EventMask::new(0x01)) {
            hprintln!("rb_test: FAIL evt_wait {:?}", e);
            debug::exit(debug::EXIT_FAILURE);
        }
        // Drain the ring buffer while the IRQ is still masked (PartitionAcks
        // model: the NVIC line stays masked until SYS_IRQ_ACK is called).
        loop {
            let exp_idx = next_tag.wrapping_sub(1);
            let exp = [0xA0 | exp_idx, exp_idx, exp_idx, exp_idx];
            let exp_tag = next_tag;
            let mut ok = false;
            // SAFETY: sole consumer; ISR is masked (IRQ_ACK not yet called).
            let popped = unsafe {
                RING.pop_with(|tag, data| {
                    if tag != exp_tag || data != exp {
                        hprintln!("rb_test: FAIL tag={} exp={}", tag, exp_tag);
                        debug::exit(debug::EXIT_FAILURE);
                    }
                    ok = true;
                })
            };
            if !popped || !ok {
                break;
            }
            next_tag = next_tag.wrapping_add(1);
            POP_COUNT.fetch_add(1, Ordering::Release);
        }
        // Unmask the IRQ after draining so the ISR can fire again.
        if let Err(e) = plib::sys_irq_ack(0) {
            hprintln!("rb_test: FAIL irq_ack {:?}", e);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("rb_test: take");
    hprintln!("rb_test: start");
    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(1, 3).expect("sched");
    let k = Kernel::<Cfg>::create_sentinels(sched).expect("kernel");
    store_kernel(k);
    enable_bound_irqs(&mut p.NVIC, Cfg::IRQ_DEFAULT_PRIORITY).unwrap();
    let parts: [(extern "C" fn() -> !, u32); 1] = [(p0_main, 0)];
    match boot(&parts, p).expect("boot") {}
}
