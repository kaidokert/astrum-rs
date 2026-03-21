//! QEMU test: P0 calls `sys_sleep_ticks(5)` and verifies elapsed ticks >= 5.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::PartitionState;
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = TestConfig::N;
const TIMEOUT_TICKS: u32 = 30;
const SLEEP_TICKS: u16 = 5;
const NOT_YET: u32 = 0xDEAD_C0DE;

static PRE_TIME: AtomicU32 = AtomicU32::new(NOT_YET);
static POST_TIME: AtomicU32 = AtomicU32::new(NOT_YET);
static SLEEP_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static SAW_WAITING: AtomicBool = AtomicBool::new(false);

kernel::define_unified_harness!(TestConfig, |tick, k| {
    let rc = SLEEP_RC.load(Ordering::Acquire);

    if rc == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!("plib_sleep_test: FAIL timeout");
            kernel::kexit!(failure);
        }
        // While P0 is still sleeping, check if it has been descheduled.
        if let Some(p) = k.partitions().get(0) {
            if p.state() == PartitionState::Waiting {
                SAW_WAITING.store(true, Ordering::Release);
            }
        }
        return;
    }

    if rc != 0 {
        hprintln!("plib_sleep_test: FAIL sleep rc={:#x}", rc);
        kernel::kexit!(failure);
    }

    if !SAW_WAITING.load(Ordering::Acquire) {
        hprintln!("plib_sleep_test: FAIL partition was never descheduled (never saw Waiting)");
        kernel::kexit!(failure);
    }

    let pre = PRE_TIME.load(Ordering::Acquire);
    let post = POST_TIME.load(Ordering::Acquire);
    let elapsed = post.wrapping_sub(pre);

    if elapsed >= SLEEP_TICKS as u32 {
        hprintln!(
            "plib_sleep_test: PASS (rc={}, pre={}, post={}, elapsed={})",
            rc,
            pre,
            post,
            elapsed
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_sleep_test: FAIL elapsed {} < {} (pre={}, post={})",
            elapsed,
            SLEEP_TICKS,
            pre,
            post
        );
        kernel::kexit!(failure);
    }
});

/// Store error code and park forever.
fn bail(e: plib::SvcError) -> ! {
    SLEEP_RC.store(e.to_u32(), Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p0_main() -> ! {
    let pre = plib::sys_get_time().unwrap_or_else(|e| bail(e));
    PRE_TIME.store(pre, Ordering::Release);

    let rc = plib::sys_sleep_ticks(SLEEP_TICKS).unwrap_or_else(|e| bail(e));

    let post = plib::sys_get_time().unwrap_or_else(|e| bail(e));

    // All succeeded — publish results. Store SLEEP_RC last as "done" signal.
    POST_TIME.store(post, Ordering::Release);
    SLEEP_RC.store(rc, Ordering::Release);

    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_sleep_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_sleep_test: round_robin");
    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(p0_main, 0),
        PartitionSpec::new(p1_main, 0),
    ];
    init_kernel(sched, &parts).expect("plib_sleep_test: kernel");

    match boot(p).expect("plib_sleep_test: boot") {}
}
