//! Semaphore Demo — PCA10100 (nRF52833-DK)
//!
//! Kernel IPC Layer 1: counting semaphore producer/consumer.
//! Identical logic to the f429zi semaphore_demo; the only platform
//! difference is CORE_CLOCK_HZ = 64_000_000 (nRF52833 default).
//!
//! - P0 (producer): waits for slot via SYS_SEM_WAIT, produces item
//! - P1 (consumer): consumes item, signals slot via SYS_SEM_SIGNAL
//!
//! Success: PROD_SUCCESS > 10 && CONS_SUCCESS > 10, printed via RTT.
//!
//! Build:  cargo build --example semaphore_demo --features kernel-example
//! Flash:  cargo run   --example semaphore_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use plib;
// panic-halt is activated via nrf52/src/lib.rs (kernel dep is now optional).
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const BUFFER_SIZE: u32 = 3;

static PROD_SUCCESS: AtomicU32 = AtomicU32::new(0);
static PROD_COUNTER: AtomicU32 = AtomicU32::new(0);
static CONS_SUCCESS: AtomicU32 = AtomicU32::new(0);
static CONS_LAST: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(SemConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    S = 2; SW = 4; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(SemConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let ps = PROD_SUCCESS.load(Ordering::Acquire);
        let pc = PROD_COUNTER.load(Ordering::Acquire);
        let cs = CONS_SUCCESS.load(Ordering::Acquire);
        let cl = CONS_LAST.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] PROD ok={} cnt={} | CONS ok={} last={}",
            tick, ps, pc, cs, cl
        );
        if ps > 10 && cs > 10 {
            rprintln!("✓ SUCCESS: Semaphore working! PROD={} CONS={}", ps, cs);
        }
    }
});

extern "C" fn producer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();
    let mut counter: u8 = 0;
    loop {
        if plib::sys_sem_wait(sem_id).is_ok() {
            counter = counter.wrapping_add(1);
            PROD_SUCCESS.fetch_add(1, Ordering::Release);
            PROD_COUNTER.store(counter as u32, Ordering::Release);
            for _ in 0..10_000 { core::hint::spin_loop(); }
        }
        for _ in 0..15_000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn consumer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();
    loop {
        let value = PROD_COUNTER.load(Ordering::Acquire);
        if value > 0 {
            CONS_SUCCESS.fetch_add(1, Ordering::Release);
            CONS_LAST.store(value, Ordering::Release);
            plib::sys_sem_signal(sem_id).ok();
            for _ in 0..8_000 { core::hint::spin_loop(); }
        }
        for _ in 0..12_000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_main_body);
kernel::partition_trampoline!(consumer_main => consumer_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Semaphore Demo — nRF52833 ===");
    rprintln!("P0=producer  P1=consumer  semaphore buffer_size={}", BUFFER_SIZE);

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ SemConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] =
        [PartitionSpec::entry(producer_main), PartitionSpec::entry(consumer_main)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    kernel::state::with_kernel_mut::<SemConfig, _, _>(|k| {
        k.semaphores_mut()
            .add(Semaphore::new(BUFFER_SIZE, BUFFER_SIZE))
            .expect("semaphore");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
