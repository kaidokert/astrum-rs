//! QEMU test: MPU_ENFORCE=true with two partitions.
//!
//! Verifies that the unified harness correctly programs MPU regions
//! on every context switch when `KernelConfig::MPU_ENFORCE` is true.
//! Two partitions each execute a yield-loop under hardware MPU
//! enforcement; the SysTick hook confirms the schedule progresses
//! and reports pass via semihosting.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
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
const SW: usize = 256; // AlignedStack1K = 256 words = 1024 bytes
const REGION_SZ: u32 = 1024;

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        mpu_enforce = true;
    }
);

// Partition entries: yield in a loop. No memory access beyond the
// stack (covered by the MPU data region) and registers (SVC ABI).
extern "C" fn p0_entry() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}
extern "C" fn p1_entry() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, _k| {
    if tick >= 8 {
        hprintln!("mpu_enforce_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
});

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_enforce_test: start");

    // Derive code region base from partition function pointers.
    // Both small yield-loop functions are adjacent in .text and fit
    // within a single 1 KiB MPU region.
    let code_base = (p0_entry as *const () as usize as u32) & !(REGION_SZ - 1);

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched entry 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched entry 1");

    // Sentinel configs: mpu_region size=0 passes the overlap check
    // in Kernel::new. entry_point is set to the computed code base
    // so partition_mpu_regions builds a valid code RX region.
    let cfgs: [PartitionConfig; NP] = [
        PartitionConfig {
            id: 0,
            entry_point: code_base,
            stack_base: 0x2000_0000,
            stack_size: (SW * 4) as u32,
            mpu_region: MpuRegion::new(0, 0, 0),
            peripheral_regions: heapless::Vec::new(),
        },
        PartitionConfig {
            id: 1,
            entry_point: code_base,
            stack_base: 0x2000_8000,
            stack_size: (SW * 4) as u32,
            mpu_region: MpuRegion::new(0, 0, 0),
            peripheral_regions: heapless::Vec::new(),
        },
    ];

    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    store_kernel(k);

    // Promote sentinel MPU regions: read actual stack addresses from
    // PartitionCore (1K-aligned) and give each partition a 1K data
    // region covering its own stack. Non-overlapping because
    // AlignedStack1K guarantees 1K alignment and 1K spacing.
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for i in 0..NP {
            let base = k.core_stack_mut(i).expect("stack").as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .unwrap()
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    })
    .expect("with_kernel_mut");

    let parts: [(extern "C" fn() -> !, u32); NP] = [(p0_entry, 0), (p1_entry, 0)];
    match boot::boot::<TestConfig>(&parts, &mut p).expect("boot failed") {}
}
