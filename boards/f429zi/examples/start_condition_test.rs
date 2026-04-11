//! Start Condition Test — verify sys_get_start_condition after warm restart
//!
//! P0: checks start condition on each entry. On NormalBoot, faults intentionally.
//!     On WarmRestart, reports SUCCESS. Proves the kernel correctly sets
//!     StartCondition through the fault→restart cycle.
//! P1: healthy partition, just yields.
//!
//! Build: cd f429zi && cargo build --example start_condition_test \
//!            --features kernel-mpu --release

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
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

static NORMAL_BOOTS: AtomicU32 = AtomicU32::new(0);
static WARM_RESTARTS: AtomicU32 = AtomicU32::new(0);
static COLD_RESTARTS: AtomicU32 = AtomicU32::new(0);
static P0_ENTRIES: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(Cfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(Cfg, |tick, _k| {
    if tick % 1000 == 0 && tick > 0 {
        let normal = NORMAL_BOOTS.load(Ordering::Relaxed);
        let warm = WARM_RESTARTS.load(Ordering::Relaxed);
        let cold = COLD_RESTARTS.load(Ordering::Relaxed);
        let entries = P0_ENTRIES.load(Ordering::Relaxed);
        rprintln!(
            "[{:5}ms] entries={} normal={} warm={} cold={}",
            tick, entries, normal, warm, cold
        );
        if warm >= 3 && normal >= 1 {
            rprintln!(
                "SUCCESS: start_condition verified! normal={} warm={} cold={}",
                normal, warm, cold
            );
        }
    }
});

extern "C" fn p0_body(_r0: u32) -> ! {
    let entry = P0_ENTRIES.fetch_add(1, Ordering::Relaxed) + 1;

    match plib::sys_get_start_condition() {
        Ok(plib::StartCondition::NormalBoot) => {
            NORMAL_BOOTS.fetch_add(1, Ordering::Relaxed);
            rprintln!("[P0] entry={} start=NormalBoot — faulting intentionally", entry);
        }
        Ok(plib::StartCondition::WarmRestart) => {
            WARM_RESTARTS.fetch_add(1, Ordering::Relaxed);
            rprintln!("[P0] entry={} start=WarmRestart — correct!", entry);
            // Yield a few times then fault again to test repeated warm restarts
            for _ in 0..5 {
                plib::sys_yield().ok();
            }
            rprintln!("[P0] entry={} faulting again for next restart", entry);
        }
        Ok(plib::StartCondition::ColdRestart) => {
            COLD_RESTARTS.fetch_add(1, Ordering::Relaxed);
            rprintln!("[P0] entry={} start=ColdRestart", entry);
        }
        Err(e) => {
            rprintln!("[P0] entry={} start_condition error: {:?}", entry, e);
        }
    }

    // Fault: write to ungranted peripheral
    unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD) };
    loop { cortex_m::asm::nop(); }
}

extern "C" fn p1_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(p0_main => p0_body);
kernel::partition_trampoline!(p1_main => p1_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Start Condition Test ===");
    rprintln!("P0 checks start_condition, faults, warm-restarts. Expect NormalBoot then WarmRestart.\n");

    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 10)).expect("P0");
    sched.add_system_window(1).expect("SW");
    sched.add(ScheduleEntry::new(1, 5)).expect("P1");
    sched.add_system_window(1).expect("SW");

    static mut STACKS: [AlignedStack2K; 2] = [AlignedStack2K::ZERO; 2];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, p0_main as kernel::PartitionEntry, data, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code)
        .expect("code0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, p1_main as kernel::PartitionEntry, data, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code)
        .expect("code1");

    let mut k = Kernel::<Cfg>::new(sched, &[mem0, mem1]).expect("kernel");
    k.pcb_mut(0).expect("pcb0").set_fault_policy(FaultPolicy::WarmRestart { max: 10 });
    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
