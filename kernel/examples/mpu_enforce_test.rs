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
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2,
    PortsTiny, StackStorage as _, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const REGION_SZ: u32 = 1024;

static mut STACKS: [AlignedStack1K; TestConfig::N] = [AlignedStack1K::ZERO; TestConfig::N];

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

// Partition entries: yield in a loop. No memory access beyond the
// stack (covered by the MPU data region) and registers (SVC ABI).
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}

kernel::define_kernel!(no_boot, TestConfig, |tick, _k| {
    if tick >= 8 {
        hprintln!("mpu_enforce_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
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
    hprintln!("mpu_enforce_test: start");

    let entry_fns: [PartitionEntry; TestConfig::N] = [p0_entry, p1_entry];

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched entry 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched entry 1");
    sched.add_system_window(1).expect("system window");

    let mut k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = &raw mut STACKS;
        let stacks = unsafe { &mut *ptr };
        let mut stk_iter = stacks.iter_mut();
        // TODO: consider replacing expect() with a non-panicking pattern (e.g. loop+kexit)
        // for panic-free main() policy.
        let memories: [_; TestConfig::N] = core::array::from_fn(|i| {
            let stk = stk_iter.next().expect("stack");
            let base = stk.as_u32_slice().as_ptr() as u32;
            let spec = PartitionSpec::entry(entry_fns[i])
                .with_data_mpu(MpuRegion::new(base, REGION_SZ, 0));
            ExternalPartitionMemory::from_spec(stk, &spec, kernel::PartitionId::new(i as u32))
                .expect("mem")
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel creation")
    };
    store_kernel(&mut k);

    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot failed") {}
}
