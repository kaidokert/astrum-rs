//! Semaphore Demo for STM32F429ZI
//!
//! Demonstrates counting semaphores for synchronization:
//! - Producer partition generates items (limited by semaphore count)
//! - Consumer partition consumes items and signals availability
//! - Semaphore prevents producer from overrunning consumer
//!
//! Two partitions:
//! - P0 (Producer): Waits for available slots, produces items
//! - P1 (Consumer): Consumes items, signals slot availability

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
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here.
use plib::SemaphoreId;
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const BUFFER_SIZE: u32 = 3; // Semaphore allows up to 3 pending items


// Progress tracking
static PROD_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
static PROD_SUCCESS: AtomicU32 = AtomicU32::new(0);
static PROD_COUNTER: AtomicU32 = AtomicU32::new(0);
static CONS_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
static CONS_SUCCESS: AtomicU32 = AtomicU32::new(0);
static CONS_LAST: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(SemConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    S = 2; SW = 4; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(SemConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let prod_att = PROD_ATTEMPTS.load(Ordering::Acquire);
        let prod_suc = PROD_SUCCESS.load(Ordering::Acquire);
        let prod_cnt = PROD_COUNTER.load(Ordering::Acquire);
        let cons_att = CONS_ATTEMPTS.load(Ordering::Acquire);
        let cons_suc = CONS_SUCCESS.load(Ordering::Acquire);
        let cons_lst = CONS_LAST.load(Ordering::Acquire);

        rprintln!(
            "[{:5}ms] PROD: att={:4} ok={:4} cnt={:3} | CONS: att={:4} ok={:4} last={:3}",
            tick, prod_att, prod_suc, prod_cnt, cons_att, cons_suc, cons_lst
        );

        if prod_suc > 10 && cons_suc > 10 {
            rprintln!("✓ SUCCESS: Semaphore working! Producer/consumer synchronized.");
        }
    }
});

/// Producer partition - generates items limited by semaphore
extern "C" fn producer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();
    let mut counter: u8 = 0;

    loop {
        PROD_ATTEMPTS.fetch_add(1, Ordering::Release);

        // Wait for available slot in buffer (semaphore down)
        if plib::sys_sem_wait(sem_id).is_ok() {
            // Got semaphore - produce item
            counter = counter.wrapping_add(1);
            PROD_SUCCESS.fetch_add(1, Ordering::Release);
            PROD_COUNTER.store(counter as u32, Ordering::Release);

            // Simulate production work
            for _ in 0..10000 {
                core::hint::spin_loop();
            }
        }

        // Delay between production attempts
        for _ in 0..15000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

/// Consumer partition - consumes items and signals availability
extern "C" fn consumer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();

    loop {
        CONS_ATTEMPTS.fetch_add(1, Ordering::Release);

        // Check current producer counter (simulating consumption)
        let value = PROD_COUNTER.load(Ordering::Acquire);

        if value > 0 {
            // Consume item
            CONS_SUCCESS.fetch_add(1, Ordering::Release);
            CONS_LAST.store(value, Ordering::Release);

            // Signal slot availability (semaphore up)
            plib::sys_sem_signal(sem_id).ok();

            // Simulate consumption work
            for _ in 0..8000 {
                core::hint::spin_loop();
            }
        }

        // Delay between consumption attempts
        for _ in 0..12000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_main_body);
kernel::partition_trampoline!(consumer_main => consumer_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Semaphore Demo ===");
    rprintln!("STM32F429ZI - Producer/Consumer Synchronization");

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: Equal time slices
    let mut sched = ScheduleTable::<{ SemConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");
    rprintln!("Schedule: P0(Producer) P1(Consumer)");

    // Both partitions get the same semaphore ID
    let sem = SemaphoreId::new(0); // First semaphore in pool
    let h: [u32; NUM_PARTITIONS] = [sem.as_raw(), sem.as_raw()];

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(producer_main as kernel::PartitionEntry, h[0]),
        PartitionSpec::new(consumer_main as kernel::PartitionEntry, h[1]),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("Kernel created");

    // Create counting semaphore: initial count = BUFFER_SIZE, max = BUFFER_SIZE
    // This allows up to BUFFER_SIZE pending items
    kernel::state::with_kernel_mut::<SemConfig, _, _>(|k| {
        k.semaphores_mut()
            .add(Semaphore::new(BUFFER_SIZE, BUFFER_SIZE))
            .expect("semaphore");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Semaphore created: {} (buffer size={})", sem.as_raw(), BUFFER_SIZE);

    rprintln!("Booting...\n");

    match boot(p).expect("boot") {}
}
