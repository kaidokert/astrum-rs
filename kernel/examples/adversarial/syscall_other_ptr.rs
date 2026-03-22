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

/// Expected error code: SvcError::InvalidPointer.
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();

/// Test name for reporting.
const TEST_NAME: &str = "syscall_other_ptr";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const NUM_PARTITIONS: usize = 2;

/// Stack size in bytes (1 KiB, matches AlignedStack1K).
const STACK_SIZE: u32 = AlignedStack1K::SIZE_BYTES as u32;

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled>);

static SVC_RESULT: AtomicU32 = AtomicU32::new(0);
static SVC_DONE: AtomicU32 = AtomicU32::new(0);
/// Packed argument for P0: port_id in bits [31:16], p1_base upper 16 in [15:0].
/// Set by main() before boot, read by p0_main_body.
static P0_ARG: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
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
const _: PartitionBody = p0_main_body;
extern "C" fn p0_main_body(_r0: u32) -> ! {
    let packed = P0_ARG.load(Ordering::Acquire);
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
const _: PartitionEntry = p1_main;
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
    let p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: only partition 0 runs (partition 1 just owns a region).
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("schedule entry must fit");

    let mut stack0 = AlignedStack1K::ZERO;
    let mut stack1 = AlignedStack1K::ZERO;
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mem0 = ExternalPartitionMemory::new(
        &mut stack0.0,
        EntryAddr::from_entry(p0_main as PartitionEntry),
        sentinel_mpu,
        0,
    )
    .expect("ext mem 0");
    let mem1 = ExternalPartitionMemory::new(
        &mut stack1.0,
        EntryAddr::from_entry(p1_main as PartitionEntry),
        sentinel_mpu,
        1,
    )
    .expect("ext mem 1");
    let mems: [ExternalPartitionMemory; NUM_PARTITIONS] = [mem0, mem1];

    let mut k = Kernel::<TestConfig>::new(sched, &mems).expect("kernel creation");

    // Create a source port (partition will "write" to it).
    let port_id = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("create port");

    // Derive stack addresses from the stack variables directly.
    let p0_base = stack0.0.as_ptr() as u32;
    let p1_base = stack1.0.as_ptr() as u32;

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

    // Pass port_id and P1 base to partition 0 via static.
    // Pack: port_id in bits [31:16], p1_base upper 16 bits in [15:0].
    // (Stack bases are 64KB aligned, so lower 16 bits are always 0.)
    P0_ARG.store(
        ((port_id as u32) << 16) | ((p1_base >> 16) & 0xFFFF),
        Ordering::Release,
    );

    match boot(p).expect("syscall_other_ptr: boot failed") {}
}
