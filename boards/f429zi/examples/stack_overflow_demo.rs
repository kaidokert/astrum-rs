//! Stack Overflow Detection Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates the kernel's PendSV stack overflow pre-check on real hardware.
//!
//! Architecture:
//!   P0 (overflower): consumes most of its stack via `sub sp, #N`, then loops.
//!     On the next PendSV context switch, the pre-check detects PSP < stack_limit
//!     and stores sentinel 0xDEAD0001 in partition_sp[0].
//!
//!   P1 (healthy): loops with a counter. Keeps running after P0's overflow is
//!     detected, proving kernel isolation — P0's corruption doesn't affect P1.
//!
//! Verification:
//!   - RTT shows "OVERFLOW DETECTED on P0" when sentinel appears
//!   - P1_COUNT keeps advancing after detection (isolation proof)
//!   - CFSR should be 0 (caught by software pre-check, not MPU fault)
//!
//! Build: cd f429zi && cargo build --example stack_overflow_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;

const NUM_PARTITIONS: usize = 2;

static P0_COUNT: AtomicU32 = AtomicU32::new(0);
static P1_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(OverflowCfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(OverflowCfg, |tick, k| {
    if tick % 500 == 0 {
        let p0_sp = k.partition_sp().get(0).copied().unwrap_or(0);
        let p1_sp = k.partition_sp().get(1).copied().unwrap_or(0);
        let p0_count = P0_COUNT.load(Ordering::Acquire);
        let p1_count = P1_COUNT.load(Ordering::Acquire);

        rprintln!(
            "[{:5}ms] P0_SP={:#010x} P1_SP={:#010x} P0_COUNT={} P1_COUNT={}",
            tick, p0_sp, p1_sp, p0_count, p1_count
        );

        if p0_sp == 0xDEAD0001 {
            rprintln!("OVERFLOW DETECTED on P0! sentinel=0xDEAD0001");
            rprintln!("P1 still running: P1_COUNT={} (isolation proof)", p1_count);
            if p1_count > 0 {
                rprintln!("SUCCESS: stack overflow caught, P1 unaffected.");
            }
        }
    }
});

// P0: consume almost all stack, then loop. PendSV will catch the overflow.
// AlignedStack1K = 1024 bytes = 256 words. Subtract frame overhead (~64 bytes
// for exception frame + callee saves), consuming 996 bytes is enough to trip.
extern "C" fn p0_overflow(_r0: u32) -> ! {
    P0_COUNT.fetch_add(1, Ordering::Release);
    // Consume 996 bytes of stack — pushes SP below stack_limit.
    // The overflow is detected on the NEXT PendSV (context switch out of P0).
    unsafe { core::arch::asm!("sub sp, sp, #996") };
    loop {
        P0_COUNT.fetch_add(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

// P1: healthy partition that keeps running after P0 overflows.
extern "C" fn p1_healthy(_r0: u32) -> ! {
    loop {
        P1_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(p0_main => p0_overflow);
kernel::partition_trampoline!(p1_main => p1_healthy);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Stack Overflow Detection Demo — STM32F429ZI ===");
    rprintln!("P0: consumes 996 bytes of stack (limit ~1024)");
    rprintln!("P1: healthy loop, proves isolation after P0 overflow");
    rprintln!("Expect: P0_SP=0xDEAD0001 sentinel, P1 keeps running\n");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ OverflowCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(p0_main),
        PartitionSpec::entry(p1_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("[INIT] Kernel created. AlignedStack1K (1024 bytes per partition).");
    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
