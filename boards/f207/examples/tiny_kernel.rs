//! Tiny Kernel — smallest possible RTOS configuration under MPU
//!
//! Single partition, AlignedStack512B, all minimal presets, debug disabled.
//! Proves the kernel fits and runs in the smallest configuration on a
//! RAM-constrained proxy target (Cortex-M3, emulating tight RAM budget).
//!
//! Build: cd f207 && cargo build --example tiny_kernel \
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
    partition_core::AlignedStack512B,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugDisabled},
};
use rtt_target::rprintln;
use f207::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

static P0_RUNS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(TinyCfg[AlignedStack512B]<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugDisabled> {
    core_clock_hz = f207::CORE_CLOCK_HZ;
    mpu_enforce = true;
    SW = 1; MW = 1;
    SM = 1; BM = 1; BW = 1;
});

kernel::define_kernel!(TinyCfg, |tick, _k| {
    if tick % 1000 == 0 && tick > 0 {
        let runs = P0_RUNS.load(Ordering::Acquire);
        rprintln!("[{:5}ms] P0_runs={}", tick, runs);
        if runs > 100 {
            rprintln!("SUCCESS: tiny kernel running under MPU! runs={}", runs);
        }
    }
});

extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        P0_RUNS.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(p0_main => p0_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Tiny Kernel — smallest config, 1 partition, 512B stack, MPU ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ TinyCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("P0");
    sched.add_system_window(1).expect("SW");

    static mut STACK: [AlignedStack512B; 1] = [AlignedStack512B::ZERO; 1];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACK) };
    let [ref mut s0] = *stacks;

    let data = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, p0_main as kernel::PartitionEntry, data, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code)
        .expect("code0");

    let mut k = Kernel::<TinyCfg>::new(sched, &[mem0]).expect("kernel");

    // Report kernel size
    let k_size = core::mem::size_of_val(&k);
    rprintln!("[INIT] Kernel struct size: {} bytes ({} KB)", k_size, k_size / 1024);
    rprintln!("[INIT] Stack: 512 bytes, 1 partition, MPU enforced");

    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
