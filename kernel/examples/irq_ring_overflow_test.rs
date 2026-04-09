//! QEMU integration test: ring buffer overflow and recovery.
//!
//! ISR pushes 6 records into a D=4 buffer (4 succeed, 2 overflow).
//! Partition verifies overflow_count > 0, drains survivors in FIFO order,
//! calls reset_overflow_count (expects 2), then verifies a subsequent
//! push succeeds after draining.
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

kernel::kernel_config!(
    Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// D=4 slots, M=4 payload bytes.
static RING: StaticIsrRing<4, 4> = StaticIsrRing::new();
/// ISR phase: 0 = burst (4 pushes into 2 slots), 1 = recovery (1 push).
static ISR_PHASE: AtomicU32 = AtomicU32::new(0);
/// Incremented by partition after each phase passes all checks.
static PASS_FLAG: AtomicU32 = AtomicU32::new(0);
/// Non-zero → partition detected a failure (encoded as check-point id).
static FAIL_LINE: AtomicU32 = AtomicU32::new(0);
/// Debug: observed overflow count, stored by partition for harness to print.
static DBG_OVF: AtomicU32 = AtomicU32::new(0xDEAD);
/// Debug: number of push results that were Err in ISR.
static DBG_ERR_COUNT: AtomicU32 = AtomicU32::new(0);
/// Debug: volatile read of overflow_count from partition.
static DBG_VOL: AtomicU32 = AtomicU32::new(0xBEEF);

const _: IsrHandler = overflow_isr;
/// Vector-table ISR: burst-pushes or recovery-pushes into the ring buffer
/// depending on `ISR_PHASE`, then signals P0 and masks the NVIC line.
///
/// # Safety
///
/// Must only be called by hardware via the interrupt vector table. Assumes
/// single-core Cortex-M (sole producer for `RING`). The corresponding NVIC
/// line must be masked before the partition drains the ring buffer.
#[allow(dead_code)]
unsafe extern "C" fn overflow_isr() {
    let phase = ISR_PHASE.load(Ordering::Acquire);
    if phase == 0 {
        // Burst: push 6 records into 4 slots. Tags 1-4 succeed; 5-6 overflow.
        let mut errs = 0u32;
        if unsafe { RING.push_from_isr(1, &[0xB0, 0, 0, 0]) }.is_err() {
            errs += 1;
        }
        if unsafe { RING.push_from_isr(2, &[0xB1, 1, 1, 1]) }.is_err() {
            errs += 1;
        }
        if unsafe { RING.push_from_isr(3, &[0xB2, 2, 2, 2]) }.is_err() {
            errs += 1;
        }
        if unsafe { RING.push_from_isr(4, &[0xB3, 3, 3, 3]) }.is_err() {
            errs += 1;
        }
        if unsafe { RING.push_from_isr(5, &[0xB4, 4, 4, 4]) }.is_err() {
            errs += 1;
        }
        if unsafe { RING.push_from_isr(6, &[0xB5, 5, 5, 5]) }.is_err() {
            errs += 1;
        }
        DBG_ERR_COUNT.store(errs, Ordering::Relaxed);
        DBG_OVF.store(unsafe { RING.overflow_count() } as u32, Ordering::Relaxed);
    } else {
        // Recovery: single push after drain proves buffer is reusable.
        let _ = unsafe { RING.push_from_isr(0xAA, &[0xCC, 0xDD, 0xEE, 0xFF]) };
    }
    #[cfg(target_arch = "arm")]
    kernel::irq_dispatch::signal_partition_from_isr::<Cfg>(0.into(), 0x01);
    // TODO: abstract direct NVIC calls behind a kernel helper (out of scope).
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::NVIC::mask(kernel::irq_dispatch::IrqNr(60));
}

kernel::bind_interrupts!(Cfg, 70, 60 => (0, 0x01, handler: overflow_isr));

kernel::define_kernel!(Cfg, |tick, _k| {
    if tick == 2 {
        ISR_PHASE.store(0, Ordering::Release);
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("ovf_test: pended IRQ (burst)");
    }
    if tick == 8 {
        ISR_PHASE.store(1, Ordering::Release);
        #[cfg(target_arch = "arm")]
        cortex_m::peripheral::NVIC::pend(kernel::irq_dispatch::IrqNr(60));
        hprintln!("ovf_test: pended IRQ (recovery)");
    }
    let fl = FAIL_LINE.load(Ordering::Acquire);
    if fl != 0 {
        hprintln!(
            "ovf_test: FAIL cp={} ovf={} errs={} vol={}",
            fl,
            DBG_OVF.load(Ordering::Relaxed),
            DBG_ERR_COUNT.load(Ordering::Relaxed),
            DBG_VOL.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick >= 12 && PASS_FLAG.load(Ordering::Acquire) >= 2 {
        hprintln!("ovf_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 20 {
        hprintln!(
            "ovf_test: FAIL pass_flag={}",
            PASS_FLAG.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

fn fail(checkpoint: u32) {
    FAIL_LINE.store(checkpoint, Ordering::Release);
}

const _: PartitionBody = p0_body;
extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        let bits = match plib::sys_event_wait(plib::EventMask::new(0x01)) {
            Ok(v) => v,
            Err(_) => {
                fail(1);
                continue;
            }
        };
        // event_wait returns 0 when entering Waiting state; skip that.
        if bits == plib::EventMask::new(0) {
            continue;
        }
        let phase = ISR_PHASE.load(Ordering::Acquire);
        if phase == 0 {
            p0_phase0();
        } else {
            p0_phase1();
        }
        if plib::sys_irq_ack(60).is_err() {
            fail(2);
        }
    }
}

fn p0_phase0() {
    // Read overflow_count via volatile pointer at known offset 0x38.
    let ring_ptr = &RING as *const _ as *const u8;
    let vol_ovf = unsafe { core::ptr::read_volatile(ring_ptr.add(0x38) as *const u32) };
    DBG_VOL.store(vol_ovf, Ordering::Relaxed);
    let ovf = unsafe { RING.overflow_count() };
    if ovf == 0 {
        fail(10);
        return;
    }
    // Drain and verify FIFO: tags 1, 2 with matching payloads.
    let mut pop_count = 0u32;
    loop {
        let exp_tag = (pop_count as u8) + 1;
        let exp_idx = pop_count as u8;
        let exp = [0xB0 | exp_idx, exp_idx, exp_idx, exp_idx];
        let mut ok = false;
        let popped = unsafe {
            RING.pop_with(|tag, data| {
                if tag == exp_tag && data == exp {
                    ok = true;
                }
            })
        };
        if !popped {
            break;
        }
        if !ok {
            fail(11);
            return;
        }
        pop_count += 1;
    }
    if pop_count != 4 {
        fail(12);
        return;
    }
    // reset_overflow_count must return 2 (tags 5 and 6 were dropped).
    let dropped = unsafe { RING.reset_overflow_count() };
    if dropped != 2 {
        fail(13);
        return;
    }
    if unsafe { RING.overflow_count() } != 0 {
        fail(14);
        return;
    }
    PASS_FLAG.fetch_add(1, Ordering::Release);
}

fn p0_phase1() {
    // Verify recovery push after drain.
    let mut ok = false;
    let popped = unsafe {
        RING.pop_with(|tag, data| {
            if tag == 0xAA && data == [0xCC, 0xDD, 0xEE, 0xFF] {
                ok = true;
            }
        })
    };
    if !popped || !ok {
        fail(20);
        return;
    }
    PASS_FLAG.fetch_add(1, Ordering::Release);
}

kernel::partition_trampoline!(p0_main => p0_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("ovf_test: take");
    hprintln!("ovf_test: start");
    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(1, 3).expect("sched");
    let parts: [PartitionSpec; 1] = [PartitionSpec::new(p0_main as PartitionEntry, 0)];
    sched.add_system_window(1).expect("sys window");
    let mut k = init_kernel(sched, &parts).expect("irq_ring_overflow_test: init_kernel");
    store_kernel(&mut k);
    enable_bound_irqs(&mut p.NVIC, Cfg::IRQ_DEFAULT_PRIORITY).unwrap();
    match boot(p).expect("boot") {}
}
