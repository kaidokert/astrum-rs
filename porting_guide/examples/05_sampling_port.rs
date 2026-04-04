//! Two-partition producer/consumer IPC via raw atomics — no kernel IPC.
//!
//! Demonstrates the inter-partition communication primitive that the
//! kernel's sampling port abstraction builds upon. A producer partition
//! writes incrementing sequence numbers to a shared `AtomicU32`, and a
//! consumer partition reads them, tracking how many distinct in-order
//! values it has observed.
//!
//! # Success criteria
//!
//! The test passes when the **consumer has received 10 or more distinct
//! values**, all in strictly increasing order. The SysTick hook monitors
//! progress and exits with PASS once the threshold is met, or FAIL on
//! timeout.
//!
//! # Run
//!
//! ```text
//! cargo run --target thumbv7m-none-eabi \
//!     --features board-qemu,log-semihosting \
//!     --example 05_sampling_port
//! ```

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

const GOAL: u32 = 10;
const TIMEOUT: u32 = 500;

kernel::kernel_config!(
    Cfg < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// Shared "sampling port": producer writes incrementing values here.
/// Value 0 means "no data yet" — producer starts at 1.
static SHARED_PORT: AtomicU32 = AtomicU32::new(0);

/// Consumer's count of distinct in-order values received.
static CONSUMER_COUNT: AtomicU32 = AtomicU32::new(0);
/// The last value the consumer successfully read (for order checking).
static CONSUMER_LAST: AtomicU32 = AtomicU32::new(0);
/// Set to 1 if consumer detects an out-of-order value.
static ORDER_VIOLATION: AtomicU32 = AtomicU32::new(0);

kernel::define_harness!(Cfg, |tick, _k| {
    let count = CONSUMER_COUNT.load(Ordering::Acquire);
    let last = CONSUMER_LAST.load(Ordering::Acquire);
    let violation = ORDER_VIOLATION.load(Ordering::Acquire);

    if violation != 0 {
        klog!("05_sampling_port: FAIL out-of-order value detected");
        kernel::kexit!(failure);
    }

    if count >= GOAL {
        klog!(
            "05_sampling_port: PASS ({} distinct values, last={})",
            count,
            last
        );
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT {
        klog!(
            "05_sampling_port: FAIL timeout (count={}, last={})",
            count,
            last
        );
        kernel::kexit!(failure);
    }

    // Suppress unused-variable warning when klog is a no-op.
    let _ = last;
});

/// Producer: writes incrementing sequence numbers (1, 2, 3, …) to SHARED_PORT.
extern "C" fn producer() -> ! {
    let mut seq: u32 = 0;
    loop {
        seq = seq.wrapping_add(1);
        SHARED_PORT.store(seq, Ordering::Release);
        for _ in 0..50 {
            cortex_m::asm::nop();
        }
    }
}

/// Consumer: reads SHARED_PORT and counts distinct, in-order values.
extern "C" fn consumer() -> ! {
    let mut last_seen: u32 = 0;
    loop {
        let val = SHARED_PORT.load(Ordering::Acquire);
        if val != 0 && val != last_seen {
            if val <= last_seen {
                ORDER_VIOLATION.store(1, Ordering::Release);
            } else {
                last_seen = val;
                CONSUMER_LAST.store(val, Ordering::Release);
                CONSUMER_COUNT.fetch_add(1, Ordering::Release);
            }
        }
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = producer;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("05_sampling_port: producer/consumer via raw atomics");

    let mut sched: ScheduleTable<{ Cfg::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");
    sched.add_system_window(1).expect("syswin 1");

    let entries: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(producer as PartitionEntry, 0),
        PartitionSpec::new(consumer as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &entries).expect("kernel creation");
    store_kernel(&mut k);
    match boot(p).expect("05_sampling_port: boot") {}
}
