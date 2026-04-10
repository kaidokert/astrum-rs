//! QEMU integration test: NeverMask IRQ dispatch path.
//!
//! Validates the NeverMask dispatch arm end-to-end:
//!
//! 1. `bind_interrupts!` maps IRQ 0 -> partition 0 with event bit 0x01
//!    using the `never_mask` keyword form (`NeverMask` clear model).
//! 2. Partition 0 loops on `event_wait(0x01)` and increments a counter.
//! 3. The SysTick harness pends IRQ 0 at tick 2 and again at tick 4.
//!    Because NeverMask never disables the IRQ, the second pend fires
//!    without any ack call in between.
//! 4. After both fires are observed, the partition calls `SYS_IRQ_ACK`
//!    and verifies it returns success (no-op for NeverMask).
//! 5. By tick 10 the counter must be >= 2, proving the IRQ dispatched
//!    twice with no masking and that ack is a harmless no-op.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,custom-ivt \
//!           --example irq_never_mask_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionBody, PartitionEntry, PartitionSpec, Partitions1, PortsTiny,
    SyncMinimal,
};

kernel::kernel_config!(
    NeverMaskConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Bind IRQ 0 -> partition 0, event bit 0x01, NeverMask (never_mask keyword).
kernel::bind_interrupts!(NeverMaskConfig, 70,
    0 => (0, 0x01, never_mask),
);

const NUM_PARTITIONS: usize = 1;

/// Incremented by the partition after each successful event_wait.
static WAIT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Set to 1 once the partition verifies SYS_IRQ_ACK returns success.
static ACK_OK: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(NeverMaskConfig, |tick, _k| {
    if tick == 2 || tick == 4 {
        // TODO: reviewer false positive – NVIC::pend is a safe fn in cortex-m 0.7;
        // no unsafe block or SAFETY comment required.
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(0));
        hprintln!("irq_never_mask_test: pended IRQ 0 at tick {}", tick);
    }
    // After the first pend (tick 2), verify exactly one dispatch occurred.
    if tick == 3 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count != 1 {
            hprintln!(
                "irq_never_mask_test: FAIL (expected count==1 at tick 3, got {})",
                count
            );
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    // After the second pend (tick 4), verify exactly two dispatches occurred.
    if tick == 5 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        if count != 2 {
            hprintln!(
                "irq_never_mask_test: FAIL (expected count==2 at tick 5, got {})",
                count
            );
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick >= 8 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        let ack_ok = ACK_OK.load(Ordering::Acquire);
        if count >= 2 && ack_ok == 1 {
            hprintln!(
                "irq_never_mask_test: PASS (wait_count={}, ack_ok={})",
                count,
                ack_ok
            );
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= 12 {
        let count = WAIT_COUNT.load(Ordering::Acquire);
        let ack_ok = ACK_OK.load(Ordering::Acquire);
        hprintln!(
            "irq_never_mask_test: FAIL (wait_count={}, ack_ok={}, expected >=2 and 1)",
            count,
            ack_ok
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionBody = p0_main_body;
extern "C" fn p0_main_body(_r0: u32) -> ! {
    loop {
        // Block until event 0x01 is signalled by the IRQ dispatch handler.
        match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(bits) if bits == plib::EventMask::new(0x01) => {
                let prev = WAIT_COUNT.fetch_add(1, Ordering::Release);

                // After the second fire, verify SYS_IRQ_ACK is a no-op success.
                if prev == 1 {
                    match plib::sys_irq_ack(0) {
                        Ok(0) => {
                            ACK_OK.store(1, Ordering::Release);
                        }
                        Ok(unexpected) => {
                            hprintln!(
                                "irq_never_mask_test: FAIL (irq_ack returned {})",
                                unexpected
                            );
                            debug::exit(debug::EXIT_FAILURE);
                        }
                        Err(e) => {
                            hprintln!("irq_never_mask_test: FAIL (irq_ack err={:?})", e);
                            debug::exit(debug::EXIT_FAILURE);
                        }
                    }
                }
            }
            Ok(bits) => {
                hprintln!(
                    "irq_never_mask_test: FAIL (unexpected event bits: {:?})",
                    bits
                );
                debug::exit(debug::EXIT_FAILURE);
            }
            Err(e) => {
                hprintln!("irq_never_mask_test: FAIL (event_wait err={:?})", e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("irq_never_mask_test: Peripherals::take");
    hprintln!("irq_never_mask_test: start");

    let mut sched = ScheduleTable::<{ NeverMaskConfig::SCHED }>::round_robin(1, 3)
        .expect("irq_never_mask_test: round_robin");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    sched.add_system_window(1).expect("sys window");
    let mut k = init_kernel(sched, &parts).expect("irq_never_mask_test: init_kernel");
    store_kernel(&mut k);

    // Unmask IRQ 0 so the software-triggered pend fires.
    enable_bound_irqs(&mut p.NVIC, NeverMaskConfig::IRQ_DEFAULT_PRIORITY).unwrap();

    match boot(p).expect("irq_never_mask_test: boot") {}
}
