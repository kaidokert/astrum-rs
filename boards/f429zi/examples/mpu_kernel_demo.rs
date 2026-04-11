//! MPU Kernel Demo — STM32F429ZI
//!
//! Two partitions running under MPU enforcement with code_mpu_region for
//! independent code/data region sizes.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;

static P0_RUNS: AtomicU32 = AtomicU32::new(0);
static P1_RUNS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(MpuConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    mpu_enforce = true;
    SW = 2; MW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(MpuConfig, |tick, _k| {
    if tick % 500 == 0 {
        let p0 = P0_RUNS.load(Ordering::Acquire);
        let p1 = P1_RUNS.load(Ordering::Acquire);
        let cfsr = unsafe { (*cortex_m::peripheral::SCB::PTR).cfsr.read() };
        let ctrl = unsafe { core::ptr::read_volatile(0xE000ED94 as *const u32) };
        rprintln!("[{:5}ms] P0={} P1={} MPU_CTRL=0x{:x} MemManage={}",
            tick, p0, p1, ctrl, cfsr & 0xFF);
        if p0 > 10 && p1 > 10 && (cfsr & 0xFF) == 0 {
            rprintln!("SUCCESS: MPU enforcement working! P0={} P1={} MemManage=0", p0, p1);
        }
    }
});

extern "C" fn p0_body(_r0: u32) -> ! {
    loop { P0_RUNS.fetch_add(1, Ordering::Release); plib::sys_yield().ok(); }
}
extern "C" fn p1_body(_r0: u32) -> ! {
    loop { P1_RUNS.fetch_add(1, Ordering::Release); plib::sys_yield().ok(); }
}
kernel::partition_trampoline!(p0_main => p0_body);
kernel::partition_trampoline!(p1_main => p1_body);

#[entry]
fn main() -> ! {
    rprintln!("=== MPU Kernel Demo — STM32F429ZI ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ MpuConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, p0_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0")
            .with_code_mpu_region(code_mpu)
            .expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, p1_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1))
            .expect("mem1")
            .with_code_mpu_region(code_mpu)
            .expect("code1"),
    ];

    let mut k = Kernel::<MpuConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}
