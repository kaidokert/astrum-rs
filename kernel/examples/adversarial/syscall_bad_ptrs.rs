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
use kernel::{
    partition::{MpuRegion, PartitionConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
    DebugEnabled, MsgMinimal, Partitions2, PortsSmall, SyncMinimal,
};
use panic_semihosting as _;

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
const STACK_WORDS: usize = 256;

/// Stack size in bytes (must be power of 2 for MPU).
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

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
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    // Build partition configs. Stack bases are derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| PartitionConfig {
        id: i as u8,
        entry_point: 0, // Not used by Kernel::new
        stack_base: 0,  // Ignored: internal stack used
        stack_size: STACK_SIZE,
        mpu_region: MpuRegion::new(0, 0, 0), // Base/size overridden by Kernel::new
        peripheral_regions: heapless::Vec::new(),
    });

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    let port_id = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(k);

    hprintln!(
        "  null={:#x} wrap={:#x}+{} expect={:#x}",
        NULL_PTR,
        WRAP_PTR,
        WRAP_LEN,
        EXPECTED_ERROR
    );

    // Build partition array for boot().
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        [(test_partition_main, port_id as u32)];

    match boot(&parts, &mut p).expect("syscall_bad_ptrs: boot failed") {}
}
