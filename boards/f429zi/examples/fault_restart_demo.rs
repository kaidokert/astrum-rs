//! Fault Restart Demo — STM32F429ZI
//!
//! P0 registers an error handler, then deliberately faults by writing to the
//! kernel guard region. The error handler queries the fault, requests a warm
//! restart. Restarted P0 confirms WarmRestart start condition. P1 monitors.
//!
//! Demonstrates:
//!   - FaultPolicy::WarmRestart
//!   - sys_register_error_handler
//!   - sys_get_error_status (fault classification)
//!   - sys_request_restart (recovery)
//!   - sys_get_start_condition (boot vs warm restart detection)
//!
//! Build: cd f429zi && cargo build --example fault_restart_demo \
//!            --features kernel-mpu --no-default-features --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, FaultPolicy, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    PartitionEntry,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(0);
static P0_RESTARTED: AtomicU32 = AtomicU32::new(0);
static HANDLER_RAN: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(FaultCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(FaultCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let runs = P0_RUN_COUNT.load(Ordering::Acquire);
        let restarted = P0_RESTARTED.load(Ordering::Acquire);
        let handler = HANDLER_RAN.load(Ordering::Acquire);
        let p1 = P1_COUNTER.load(Ordering::Relaxed);
        rprintln!(
            "[{:5}ms] P0_runs={} restarted={} handler={} P1={}",
            tick, runs, restarted, handler, p1
        );
        if restarted > 0 && handler > 0 && p1 > 0 {
            rprintln!("SUCCESS: fault restart demo working! P0 faulted, handler ran, P0 warm-restarted, P1 healthy");
        }
    }
});

// Error handler — kernel activates this when P0 faults.
const _: PartitionEntry = error_handler;
extern "C" fn error_handler() -> ! {
    HANDLER_RAN.fetch_add(1, Ordering::Release);
    // Query the fault info.
    if let Ok(info) = plib::sys_get_error_status() {
        rprintln!("[ERR-HANDLER] fault_kind={} partition={}", info.fault_kind_raw(), info.failed_partition());
    }
    // Request warm restart.
    plib::sys_request_restart(true).ok();
    loop { cortex_m::asm::nop(); }
}

// P0 — faults on first run, detects warm restart on second run.
extern "C" fn p0_body(_r0: u32) -> ! {
    let run = P0_RUN_COUNT.fetch_add(1, Ordering::Release) + 1;

    if run == 1 {
        // First boot — register error handler, then fault.
        let handler_fn: extern "C" fn() =
            unsafe { core::mem::transmute(error_handler as *const ()) };
        plib::sys_register_error_handler(handler_fn).expect("register handler");

        // Burn a few cycles then write to an ungranted peripheral address → MemManage fault.
        // Partition has SRAM + flash only — any peripheral (0x4000_xxxx) is denied.
        for _ in 0..50u32 { cortex_m::asm::nop(); }
        unsafe { core::ptr::write_volatile(0x4000_0000 as *mut u32, 0xDEAD) };
    } else {
        // Warm restart — check condition.
        match plib::sys_get_start_condition() {
            Ok(plib::StartCondition::WarmRestart) => {
                P0_RESTARTED.store(1, Ordering::Release);
            }
            other => {
                rprintln!("[P0] unexpected start condition: {:?}", other);
            }
        }
    }
    loop { plib::sys_yield().ok(); }
}

// P1 — healthy monitor partition.
extern "C" fn p1_body(_r0: u32) -> ! {
    loop {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(p0_main => p0_body);
kernel::partition_trampoline!(p1_main => p1_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Fault Restart Demo — STM32F429ZI ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ FaultCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, p0_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, p1_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<FaultCfg>::new(sched, &mems).expect("kernel");

    // Set fault policy: P0 gets 3 warm restarts before permanent fault.
    k.pcb_mut(0).expect("pcb0").set_fault_policy(FaultPolicy::WarmRestart { max: 3 });

    store_kernel(&mut k);
    rprintln!("[INIT] P0: FaultPolicy::WarmRestart(max=3). Booting with MPU...\n");

    match boot(p).expect("boot") {}
}
