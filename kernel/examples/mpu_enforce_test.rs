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

const SW: usize = TestConfig::STACK_WORDS;
const REGION_SZ: u32 = 1024;

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
    let p = cortex_m::Peripherals::take().expect("peripherals");
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
    let cfgs: [PartitionConfig; TestConfig::N] = [
        PartitionConfig::new(0, code_base, MpuRegion::new(0, 0, 0)),
        PartitionConfig::new(1, code_base, MpuRegion::new(0, 0, 0)),
    ];

    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("kernel creation");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("kernel creation");

    store_kernel(k);

    // SAFETY: called once from main before any interrupt handler runs.
    let stacks: &mut [[u32; SW]; TestConfig::N] =
        unsafe { &mut *(&raw mut PARTITION_STACKS).cast() };
    // Promote sentinel MPU regions with actual external stack addresses.
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for (i, stk) in stacks.iter().enumerate() {
            let base = stk.as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .unwrap()
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    })
    .expect("with_kernel_mut");

    let parts: [(extern "C" fn() -> !, u32); TestConfig::N] = [(p0_entry, 0), (p1_entry, 0)];
    match boot::boot_external::<TestConfig, SW>(&parts, p, stacks).expect("boot failed") {}
}
