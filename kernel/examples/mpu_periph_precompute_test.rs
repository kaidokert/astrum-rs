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
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, StackStorage as _,
    SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const REGION_SZ: u32 = 1024;

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        // IMPORTANT: Build with --release. Debug builds spin in PendSV.
        mpu_enforce = true;
    }
);

fn partition_loop() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    partition_loop()
}
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    partition_loop()
}

/// # Safety: caller must have exclusive MPU access.
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    debug_assert!(n < 8, "MPU region index out of range: {}", n);
    mpu.rnr.write(n);
    (mpu.rbar.read(), mpu.rasr.read())
}

kernel::define_kernel!(no_boot, TestConfig, |tick, k| {
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
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("mpu_periph_precompute_test: start");
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    // P0 → GPIOA (0x4000_4000), P1 → GPIOB (0x4000_5000) on LM3S6965.
    let periph_regions: [[MpuRegion; 1]; NP] = [
        [MpuRegion::new(0x4000_4000, 4096, 0)],
        [MpuRegion::new(0x4000_5000, 4096, 0)],
    ];
    let mut k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(base, REGION_SZ, 0),
                kernel::PartitionId::new(i as u32),
            )
            .expect("mem")
            .with_peripheral_regions(&periph_regions[i])
            .expect("periph")
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot") {}
}
