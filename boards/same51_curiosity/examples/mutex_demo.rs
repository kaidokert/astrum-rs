//! Mutex Demo — SAME51 Curiosity Nano
//!
//! Two worker partitions compete for a shared resource via mutex.
//!
//! - P0 (Worker A): Locks mutex, increments counter, unlocks
//! - P1 (Worker B): Locks mutex, increments counter, unlocks
//!
//! Success: SHARED > 10 && both workers incrementing.
//!
//! Build: cd same51_curiosity && cargo build --example mutex_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

static SHARED_COUNTER: AtomicU32 = AtomicU32::new(0);
static WORKER_A_LOCKS: AtomicU32 = AtomicU32::new(0);
static WORKER_A_INCREMENTS: AtomicU32 = AtomicU32::new(0);
static WORKER_B_LOCKS: AtomicU32 = AtomicU32::new(0);
static WORKER_B_INCREMENTS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(MutexConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 4;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(MutexConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(MutexConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let shared = SHARED_COUNTER.load(Ordering::Acquire);
        let a_locks = WORKER_A_LOCKS.load(Ordering::Acquire);
        let a_inc = WORKER_A_INCREMENTS.load(Ordering::Acquire);
        let b_locks = WORKER_B_LOCKS.load(Ordering::Acquire);
        let b_inc = WORKER_B_INCREMENTS.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] SHARED={:4} | A: locks={:3} inc={:3} | B: locks={:3} inc={:3}",
            tick, shared, a_locks, a_inc, b_locks, b_inc
        );
        if shared > 10 && a_inc > 5 && b_inc > 5 {
            rprintln!("SUCCESS: Mutex working! Both workers incrementing shared counter.");
        }
    }
});

extern "C" fn worker_a_main_body(r0: u32) -> ! {
    let mtx_id = r0.into();
    loop {
        if plib::sys_mtx_lock(mtx_id).is_ok() {
            WORKER_A_LOCKS.fetch_add(1, Ordering::Release);
            let old_val = SHARED_COUNTER.load(Ordering::Acquire);
            for _ in 0..5000 { core::hint::spin_loop(); }
            SHARED_COUNTER.store(old_val + 1, Ordering::Release);
            WORKER_A_INCREMENTS.fetch_add(1, Ordering::Release);
            plib::sys_mtx_unlock(mtx_id).ok();
        }
        for _ in 0..10000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn worker_b_main_body(r0: u32) -> ! {
    let mtx_id = r0.into();
    loop {
        if plib::sys_mtx_lock(mtx_id).is_ok() {
            WORKER_B_LOCKS.fetch_add(1, Ordering::Release);
            let old_val = SHARED_COUNTER.load(Ordering::Acquire);
            for _ in 0..5000 { core::hint::spin_loop(); }
            SHARED_COUNTER.store(old_val + 1, Ordering::Release);
            WORKER_B_INCREMENTS.fetch_add(1, Ordering::Release);
            plib::sys_mtx_unlock(mtx_id).ok();
        }
        for _ in 0..8000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(worker_a_main => worker_a_main_body);
kernel::partition_trampoline!(worker_b_main => worker_b_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Mutex Demo — SAME51 ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ MutexConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    let mtx = 0u32;

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, worker_a_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(mtx as u32))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, worker_b_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(mtx as u32))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
    ];

    let mut k = Kernel::<MutexConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
