//! Regression guard for Bug 14-numbat: asserts PRIMASK==0 and BASEPRI==0 in Thread mode.
//! Two partitions read both registers every iteration and abort if non-zero.
//! Fast SysTick drives many PendSV switches; a leaked mask is caught.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions2, PortsTiny, SyncMinimal};

// Fast SysTick: 12 MHz * 83 µs / 1e6 ≈ 996 cycles per tick.
kernel::compose_kernel_config!(
    Config < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 83;
    }
);

const NUM_PARTITIONS: usize = 2;

/// Minimum tick count before declaring success (~10 major frames).
const CHECK_TICK: u32 = 20;
/// Hard timeout tick.
const TIMEOUT_TICK: u32 = 200;
/// Minimum per-partition iterations to accept.
const MIN_COUNT: u32 = 4;

static PARTITION_COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

kernel::define_unified_harness!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    if tick >= CHECK_TICK {
        let counts: [u32; NUM_PARTITIONS] =
            core::array::from_fn(|i| PARTITION_COUNTS[i].load(Ordering::Acquire));
        if counts.iter().all(|&c| c >= MIN_COUNT) {
            hprintln!("pendsv_primask_leak_test: PASS ({:?})", counts);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= TIMEOUT_TICK {
        let counts: [u32; NUM_PARTITIONS] =
            core::array::from_fn(|i| PARTITION_COUNTS[i].load(Ordering::Acquire));
        hprintln!("pendsv_primask_leak_test: FAIL timeout ({:?})", counts);
        debug::exit(debug::EXIT_FAILURE);
    }
});

// TODO: consolidate read_primask to use cortex_m::register::primask instead of
// inline asm, matching the read_basepri pattern, once the legacy unsafe block is
// cleaned up project-wide.

/// Read PRIMASK register.  Returns 0 when interrupts are enabled.
#[inline(always)]
fn read_primask() -> u32 {
    #[cfg(target_arch = "arm")]
    {
        let val: u32;
        // SAFETY: MRS reads a read-only status register with no side effects.
        unsafe {
            core::arch::asm!("mrs {}, PRIMASK", out(reg) val, options(nomem, nostack, preserves_flags))
        };
        val
    }
    #[cfg(not(target_arch = "arm"))]
    {
        0 // host-build stub for clippy / check
    }
}

/// Read BASEPRI register.  Returns 0 when no priority masking is active.
#[inline(always)]
fn read_basepri() -> u32 {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::basepri::read() as u32
    }
    #[cfg(not(target_arch = "arm"))]
    {
        0 // host-build stub for clippy / check
    }
}

/// Report a register leak and terminate the simulation.
fn fail_leak(id: usize, reg: &str, val: u32) -> ! {
    hprintln!("p{}: {} leak detected ({}={})", id, reg, reg, val);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        #[cfg(target_arch = "arm")]
        cortex_m::asm::nop();
        #[cfg(not(target_arch = "arm"))]
        core::hint::spin_loop();
    }
}

fn partition_task(id: usize) -> ! {
    loop {
        let pm = read_primask();
        if pm != 0 {
            fail_leak(id, "PRIMASK", pm);
        }
        let bp = read_basepri();
        if bp != 0 {
            fail_leak(id, "BASEPRI", bp);
        }
        PARTITION_COUNTS[id].fetch_add(1, Ordering::Release);
        #[cfg(target_arch = "arm")]
        cortex_m::asm::nop();
    }
}

extern "C" fn p0_main() -> ! {
    partition_task(0)
}

extern "C" fn p1_main() -> ! {
    partition_task(1)
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("primask_leak: Peripherals::take");
    hprintln!("pendsv_primask_leak_test: start");

    let sched = ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1)
        .expect("primask_leak: sched");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main, 0),
        PartitionSpec::new(p1_main, 0),
    ];
    init_kernel(sched, &parts).expect("primask_leak: Kernel::create");

    match boot(p).expect("primask_leak: boot") {}
}
