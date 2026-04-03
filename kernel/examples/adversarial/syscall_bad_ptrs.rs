//! Adversarial test: null pointer (0x0) and wrapping pointer (0xFFFF_FFF0 + 32)
//! validation. Both should return `SvcError::InvalidPointer` without faults.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_bad_ptrs

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::{
    kpanic as _,
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionBody, PartitionEntry, Partitions2,
    PortsSmall, StackStorage as _, SyncMinimal,
};

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

const NULL_PTR: u32 = 0x0;
const WRAP_PTR: u32 = 0xFFFF_FFF0;
const WRAP_LEN: u32 = 32;
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();
const TEST_NAME: &str = "syscall_bad_ptrs";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const NUM_PARTITIONS: usize = 1;

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsSmall,
    DebugEnabled > {
        buffer_pool_regions = 1;
        buffer_zone_size = 32;
        dynamic_regions = 4;
    }
);

// 0 = pending, 1 = pass, 2 = fail (null ptr), 3 = fail (wrap ptr)
static RESULT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == 1 {
        hprintln!("{}: PASS", TEST_NAME);
        debug::exit(debug::EXIT_SUCCESS);
    } else if r >= 2 {
        hprintln!("{}: FAIL (code {})", TEST_NAME, r);
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick > 100 {
        hprintln!("{}: FAIL - timeout", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition 0 entry: test null pointer and wrapping pointer syscalls.
const _: PartitionBody = test_partition_main_body;
extern "C" fn test_partition_main_body(r0: u32) -> ! {
    let port_id = r0;
    let r1 = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, NULL_PTR);
    if r1 != EXPECTED_ERROR {
        RESULT.store(2, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }
    let r2 = kernel::svc!(SYS_SAMPLING_WRITE, port_id, WRAP_LEN, WRAP_PTR);
    if r2 != EXPECTED_ERROR {
        RESULT.store(3, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }
    RESULT.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}
kernel::partition_trampoline!(test_partition_main => test_partition_main_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    let mut stack0 = AlignedStack1K::ZERO;
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mem0 = ExternalPartitionMemory::new(
        &mut stack0.0,
        EntryAddr::from_entry(test_partition_main as PartitionEntry),
        sentinel_mpu,
        kernel::PartitionId::new(0),
    )
    .expect("ext mem");
    let mems: [ExternalPartitionMemory; NUM_PARTITIONS] = [mem0];

    let mut k = Kernel::<TestConfig>::new(sched, &mems).expect("kernel creation");

    let _port_id = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(&mut k);

    hprintln!(
        "  null={:#x} wrap={:#x}+{} expect={:#x}",
        NULL_PTR,
        WRAP_PTR,
        WRAP_LEN,
        EXPECTED_ERROR
    );

    // Build partition array for boot().

    match boot(p).expect("syscall_bad_ptrs: boot failed") {}
}
