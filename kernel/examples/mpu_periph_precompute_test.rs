//! QEMU test: MPU peripheral regions R4-R5 match cached_periph_regions.
//! Both partitions configure a real peripheral_region (GPIOA / GPIOB).
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    boot,
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const SW: usize = TestConfig::STACK_WORDS;
const REGION_SZ: u32 = 1024;

// TODO: PartitionStacks boilerplate is duplicated across mpu_cached_dynamic_test,
// mpu_dynamic_base_test, and mpu_periph_precompute_test — consolidate into a shared
// macro or common test utilities module.
#[repr(C, align(4096))]
struct PartitionStacks([[u32; SW]; TestConfig::N]);
static mut PARTITION_STACKS: PartitionStacks = PartitionStacks([[0u32; SW]; TestConfig::N]);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        mpu_enforce = true;
    }
);

fn partition_loop() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}

extern "C" fn p0_entry() -> ! {
    partition_loop()
}
extern "C" fn p1_entry() -> ! {
    partition_loop()
}

/// # Safety: caller must have exclusive MPU access.
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    debug_assert!(n < 8, "MPU region index out of range: {}", n);
    mpu.rnr.write(n);
    (mpu.rbar.read(), mpu.rasr.read())
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    static SEEN: [core::sync::atomic::AtomicBool; 2] = [
        core::sync::atomic::AtomicBool::new(false),
        core::sync::atomic::AtomicBool::new(false),
    ];
    if tick > 2 {
        let pid = k.current_partition as usize;
        if pid < NP {
            if let Some(pcb) = k.partitions().get(pid) {
                let periph = pcb.cached_periph_regions();
                // SAFETY: SysTick preempts PendSV → exclusive MPU access.
                let p = unsafe { cortex_m::Peripherals::steal() };
                for (slot, &(c_rbar, c_rasr)) in periph.iter().take(2).enumerate() {
                    let r = (slot + 4) as u32;
                    // SAFETY: exclusive MPU access; see above.
                    let (hw_rbar, hw_rasr) = unsafe { read_mpu_region(&p.MPU, r) };
                    if (c_rbar & !0x10) != hw_rbar || c_rasr != hw_rasr {
                        hprintln!(
                            "FAIL: R{} p{} RBAR exp={:#010x} got={:#010x} RASR exp={:#010x} got={:#010x}",
                            r, pid, c_rbar & !0x10, hw_rbar, c_rasr, hw_rasr
                        );
                        kernel::kexit!(failure);
                    }
                }
                SEEN[pid].store(true, core::sync::atomic::Ordering::Release);
            }
        }
    }
    let both = SEEN[0].load(core::sync::atomic::Ordering::Acquire)
        && SEEN[1].load(core::sync::atomic::Ordering::Acquire);
    if tick >= 32 && both {
        hprintln!("mpu_periph_precompute_test: PASS R4-R5");
        kernel::kexit!(success);
    }
    if tick >= 100 {
        hprintln!("FAIL: timeout (both_seen={})", both);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_periph_precompute_test: start");
    let entry_fns: [extern "C" fn() -> !; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let mut cfgs = PartitionConfig::sentinel_array::<NP>(TestConfig::STACK_WORDS);
    for (i, cfg) in cfgs.iter_mut().enumerate() {
        // entry_point is the MPU code-region base, not the execution start address.
        // boot::boot() receives the actual function pointer via the `parts` array.
        cfg.entry_point = entry_fns[i] as usize as u32 & !(REGION_SZ - 1);
    }
    // P0 → GPIOA (0x4000_4000), P1 → GPIOB (0x4000_5000) on LM3S6965.
    cfgs[0]
        .peripheral_regions
        .push(MpuRegion::new(0x4000_4000, 4096, 0))
        .expect("p0 periph");
    cfgs[1]
        .peripheral_regions
        .push(MpuRegion::new(0x4000_5000, 4096, 0))
        .expect("p1 periph");
    let k = Kernel::<TestConfig>::create(sched, &cfgs).expect("kernel");
    store_kernel(k);
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks: &mut [[u32; SW]; TestConfig::N] =
        unsafe { &mut *(&raw mut PARTITION_STACKS).cast() };
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for (i, stk) in stacks.iter().enumerate() {
            let base = stk.as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .expect("partition")
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    })
    .expect("with_kernel_mut");
    let parts: [(extern "C" fn() -> !, u32); NP] = [(p0_entry, 0), (p1_entry, 0)];
    match boot::boot_external::<TestConfig, SW>(&parts, &mut p, stacks).expect("boot") {}
}
