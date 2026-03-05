//! QEMU test: dynamic-mode R0-R3 cached-region readback.
//!
//! Under the `dynamic-mpu` feature, PendSV programs R0-R3 via
//! `write_cached_base_regions()` while R4+ are managed by DynamicStrategy.
//! This test reads back hardware RBAR/RASR for R0-R3 in the SysTick hook
//! and asserts they match the active partition's `cached_base_regions()`.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    boot,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::SYS_YIELD,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const REGION_SZ: u32 = 1024;

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        mpu_enforce = true;
    }
);

macro_rules! partition_entry {
    ($name:ident, $counter:expr) => {
        extern "C" fn $name() -> ! {
            loop {
                $counter.fetch_add(1, Ordering::Release);
                let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
                assert!(rc == 0, "SYS_YIELD returned non-zero");
            }
        }
    };
}
partition_entry!(p0_entry, P0_COUNTER);
partition_entry!(p1_entry, P1_COUNTER);

/// Read the (RBAR, RASR) pair for MPU region `n` from hardware.
///
/// # Safety
///
/// The caller must ensure exclusive access to the MPU peripheral.
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    mpu.rnr.write(n);
    (mpu.rbar.read(), mpu.rasr.read())
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    // After a few ticks, PendSV has run and programmed R0-R3.
    if tick > 2 {
        let pid = k.current_partition as usize;
        if let Some(pcb) = k.partitions().get(pid) {
            let base = pcb.cached_base_regions();
            let p = unsafe { cortex_m::Peripherals::steal() };
            for (r, &(c_rbar, c_rasr)) in base.iter().enumerate() {
                let (hw_rbar, hw_rasr) = unsafe { read_mpu_region(&p.MPU, r as u32) };
                // RBAR VALID bit (bit 4) is write-only; mask for comparison.
                if (c_rbar & !0x10) != hw_rbar || c_rasr != hw_rasr {
                    hprintln!(
                        "FAIL: R{} p{} RBAR exp={:#010x} got={:#010x} RASR exp={:#010x} got={:#010x}",
                        r, pid, c_rbar & !0x10, hw_rbar, c_rasr, hw_rasr
                    );
                    kernel::kexit!(failure);
                }
            }
        }
    }
    let p0 = P0_COUNTER.load(Ordering::Acquire);
    let p1 = P1_COUNTER.load(Ordering::Acquire);
    if tick >= 32 && p0 >= 4 && p1 >= 4 {
        hprintln!("mpu_dynamic_base_test: PASS R0-R3 (p0={}, p1={})", p0, p1);
        kernel::kexit!(success);
    }
    if tick >= 100 {
        hprintln!("FAIL: timeout (p0={}, p1={})", p0, p1);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_dynamic_base_test: start");
    let entry_fns: [extern "C" fn() -> !; NP] = [p0_entry, p1_entry];
    // dynamic-mpu requires system windows in the schedule.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    sched.add_system_window(1).expect("syswin 1");
    let mut cfgs = PartitionConfig::sentinel_array::<NP>(TestConfig::STACK_WORDS);
    for (i, cfg) in cfgs.iter_mut().enumerate() {
        cfg.entry_point = entry_fns[i] as usize as u32 & !(REGION_SZ - 1);
    }
    hprintln!("  schedule built ({} entries)", 4);
    let k = Kernel::<TestConfig>::create(sched, &cfgs).expect("kernel");
    hprintln!("  kernel created");
    store_kernel(k);
    hprintln!("  kernel stored");
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for i in 0..NP {
            let base = k.core_stack_mut(i).expect("stack").as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .expect("partition")
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    });
    hprintln!("  MPU promoted");
    let parts: [(extern "C" fn() -> !, u32); NP] = [(p0_entry, 0), (p1_entry, 0)];
    hprintln!("  calling boot::boot");
    match boot::boot::<TestConfig>(&parts, &mut p).expect("boot") {}
}
