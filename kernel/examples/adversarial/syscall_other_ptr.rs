//! Adversarial test: partition calls syscall with pointer into another partition's region.
//!
//! This test verifies that the kernel's `validate_user_ptr` check in svc.rs
//! correctly rejects syscall data pointers that point to another partition's
//! MPU region. Partition 0 invokes SYS_SAMPLING_WRITE with r3 set to an address
//! within partition 1's data region, which should be rejected with
//! `SvcError::InvalidPointer` (0xFFFF_FFF9) **without** dereferencing the pointer.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_other_ptr

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

/// Expected error code: SvcError::InvalidPointer.
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();

/// Test name for reporting.
const TEST_NAME: &str = "syscall_other_ptr";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 256;

/// Stack size in bytes (must be power of 2 for MPU).
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled>);

static SVC_RESULT: AtomicU32 = AtomicU32::new(0);
static SVC_DONE: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
    if SVC_DONE.load(Ordering::Acquire) == 1 {
        let result = SVC_RESULT.load(Ordering::Acquire);
        if result == EXPECTED_ERROR {
            hprintln!("{}: PASS (error {:#010x})", TEST_NAME, result);
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!(
                "{}: FAIL - expected {:#010x}, got {:#010x}",
                TEST_NAME,
                EXPECTED_ERROR,
                result
            );
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick > 100 {
        hprintln!("{}: FAIL - timeout", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition 0 entry: invoke SYS_SAMPLING_WRITE with pointer into P1's region.
extern "C" fn p0_main_body(r0: u32) -> ! {
    let packed = r0;
    let port_id = packed >> 16;
    let p1_base = (packed & 0xFFFF) << 16;
    let target_in_p1 = p1_base + STACK_SIZE / 2;
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, target_in_p1);
    SVC_RESULT.store(result, Ordering::Release);
    SVC_DONE.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}
kernel::partition_trampoline!(p0_main => p0_main_body);

/// Partition 1 entry: idle loop (never scheduled in this test, exists for
/// separate MPU region).
extern "C" fn p1_main() -> ! {
    // This partition is never scheduled; it exists solely to have a separate
    // MPU region that partition 0 cannot access.
    loop {
        cortex_m::asm::wfi();
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: only partition 0 runs (partition 1 just owns a region).
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

    // Create a source port (partition will "write" to it).
    let port_id = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    // Get actual stack bases from the kernel's internal stacks.
    let p0_base = k.partitions().get(0).unwrap().stack_base();
    let p1_base = k.partitions().get(1).unwrap().stack_base();

    store_kernel(k);

    hprintln!("  port_id: {}", port_id);
    hprintln!(
        "  P0 region: {:#010x} - {:#010x}",
        p0_base,
        p0_base + STACK_SIZE
    );
    hprintln!(
        "  P1 region: {:#010x} - {:#010x}",
        p1_base,
        p1_base + STACK_SIZE
    );
    hprintln!("  target (in P1): {:#010x}", p1_base + STACK_SIZE / 2);
    hprintln!("  expected error: {:#010x}", EXPECTED_ERROR);

    // Pass port_id and P1 base to partition 0 via r0.
    // Pack: port_id in bits [31:16], p1_base upper 16 bits in [15:0].
    // (Stack bases are 64KB aligned, so lower 16 bits are always 0.)
    let p0_arg = ((port_id as u32) << 16) | ((p1_base >> 16) & 0xFFFF);

    // Build partition array for boot().
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [
        (p0_main, p0_arg),
        (p1_main, 0), // P1 never runs, arg unused
    ];

    match boot(&parts, &mut p).expect("syscall_other_ptr: boot failed") {}
}
