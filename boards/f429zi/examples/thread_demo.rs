//! Intra-Partition Thread Demo — two threads in one partition under MPU
//!
//! P0: main thread creates a second thread via SYS_THREAD_CREATE.
//!     Both threads increment separate counters in a round-robin schedule.
//!     Proves intra-partition threading works on real hardware.
//! P1: healthy partition, just yields (proves inter-partition scheduling
//!     still works alongside intra-partition threading).
//!
//! Build: cd f429zi && cargo build --example thread_demo \
//!            --features kernel-mpu --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack4K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtos_traits::thread::SchedulingPolicy;
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

static T0_COUNTER: AtomicU32 = AtomicU32::new(0);
static T1_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_RUNS: AtomicU32 = AtomicU32::new(0);
static THREAD_CREATED: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(ThreadCfg[AlignedStack4K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(ThreadCfg, |tick, _k| {
    if tick % 500 == 0 {
        let t0 = T0_COUNTER.load(Ordering::Acquire);
        let t1 = T1_COUNTER.load(Ordering::Acquire);
        let p1 = P1_RUNS.load(Ordering::Acquire);
        let created = THREAD_CREATED.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] thread_created={} T0={} T1={} P1_runs={}",
            tick, created, t0, t1, p1
        );
        if t0 > 10 && t1 > 10 && p1 > 5 {
            rprintln!(
                "SUCCESS: intra-partition threads on hardware! T0={} T1={} P1={}",
                t0, t1, p1
            );
        }
    }
});

// Thread 1 entry — created by main thread at runtime.
// Spins without yielding — the SysTick intra-partition scheduler
// rotates between threads within P0's time slot.
extern "C" fn thread1_entry() -> ! {
    loop {
        T1_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

// P0 main thread — creates thread1, then loops counting
extern "C" fn p0_body(_r0: u32) -> ! {
    // Create a second thread via plib wrapper.
    match plib::sys_thread_create(thread1_entry as *const (), 1) {
        Ok(tid) => THREAD_CREATED.store(tid + 1, Ordering::Relaxed), // store thread_id + 1
        Err(e) => THREAD_CREATED.store(0x80000000 | (e as u32), Ordering::Relaxed),
    }

    // Main thread also spins — SysTick rotates between T0 and T1
    loop {
        T0_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

// P1: healthy partition, proves inter-partition scheduling works alongside threads
extern "C" fn p1_body(_r0: u32) -> ! {
    loop {
        P1_RUNS.fetch_add(1, Ordering::Relaxed);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(p0_main => p0_body);
kernel::partition_trampoline!(p1_main => p1_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Intra-Partition Thread Demo ===");
    rprintln!("P0: 2 threads (round-robin), P1: healthy. MPU enforced.");
    rprintln!("Thread 1 created at runtime via SYS_THREAD_CREATE.\n");

    let mut sched = ScheduleTable::<{ ThreadCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 10)).expect("P0"); // larger slot for 2 threads
    sched.add_system_window(1).expect("SW");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW");

    static mut STACKS: [AlignedStack4K; 2] = [AlignedStack4K::ZERO; 2];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, p0_main as kernel::PartitionEntry, data, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code)
        .expect("code0")
        .with_scheduling_policy(SchedulingPolicy::RoundRobin);

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, p1_main as kernel::PartitionEntry, data, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code)
        .expect("code1");

    let mut k = Kernel::<ThreadCfg>::new(sched, &[mem0, mem1]).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
