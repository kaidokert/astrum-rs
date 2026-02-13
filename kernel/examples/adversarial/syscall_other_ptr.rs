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

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    partition::{MpuRegion, PartitionConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    syscall::SYS_SAMPLING_WRITE,
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

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 2; // Two partitions
    const SCHED: usize = 4;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 4; // Need at least one sampling port
    const SM: usize = 4;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
}

// Use the unified harness macro: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS);

// ---------------------------------------------------------------------------
// Partition entry points
// ---------------------------------------------------------------------------

/// Partition 0 entry: invoke SYS_SAMPLING_WRITE with pointer into P1's region.
extern "C" fn p0_main() -> ! {
    // Retrieve packed argument: upper 16 bits = port_id, lower 16 bits = P1 offset index.
    // Actually, we pack: port_id in bits [31:16], p1_base high bits in [15:0].
    // For simplicity, we pass the full p1_base as a second word via a static.
    let packed = kernel::unpack_r0!();
    let port_id = packed >> 16;
    let p1_base = (packed & 0xFFFF) << 16; // Reconstruct upper half (lower half is 0 for aligned stacks)

    // Target address: middle of partition 1's MPU region.
    let target_in_p1 = p1_base + STACK_SIZE / 2;

    // Issue SYS_SAMPLING_WRITE with:
    //   r1 = port_id (valid sampling port)
    //   r2 = 4 (data length)
    //   r3 = target_in_p1 (invalid: points to partition 1's region)
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, target_in_p1);

    // The kernel should have rejected this with InvalidPointer.
    // If we reach here, no MemManage fault occurred (pointer was validated
    // before dereference).
    if result == EXPECTED_ERROR {
        hprintln!("{}: PASS (error code {:#010x})", TEST_NAME, result);
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

    loop {
        cortex_m::asm::wfi();
    }
}

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

    // Build partition configs with separate MPU regions.
    // SAFETY: single-core, interrupts disabled — exclusive access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        core::array::from_fn(|i| {
            let b = STACKS[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: STACK_SIZE,
                mpu_region: MpuRegion::new(b, STACK_SIZE, 0),
            }
        })
    };

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    // Create a source port (partition will "write" to it).
    let port_id = k
        .sampling
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    store_kernel(k);

    // Compute the P1 region base for logging (same as STACKS[1]).
    let p0_base = unsafe { STACKS[0].0.as_ptr() as u32 };
    let p1_base = unsafe { STACKS[1].0.as_ptr() as u32 };

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

    boot(&parts, &mut p)
}
