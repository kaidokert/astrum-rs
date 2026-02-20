//! Adversarial test: partition calls SYS_SAMPLING_WRITE with kernel pointer.
//!
//! This test verifies that the kernel's `validate_user_ptr` check in svc.rs
//! correctly rejects syscall data pointers that point to kernel memory.
//! The partition invokes SYS_SAMPLING_WRITE with r3 set to 0x2000_F000 (kernel
//! RAM), which should be rejected with `SvcError::InvalidPointer` (0xFFFF_FFF9)
//! **without** dereferencing the pointer or causing a MemManage fault.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example syscall_kernel_ptr

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::{AlignedStack1K, PartitionCore},
    port_pools::PortPools,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{Kernel, SvcError},
    sync_pools::SyncPools,
    syscall::SYS_SAMPLING_WRITE,
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Kernel RAM address to pass as the data pointer.
/// This is outside the partition's MPU data region.
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Expected error code: SvcError::InvalidPointer.
const EXPECTED_ERROR: u32 = SvcError::InvalidPointer.to_u32();

/// Test name for reporting.
const TEST_NAME: &str = "syscall_kernel_ptr";

// ---------------------------------------------------------------------------
// Kernel configuration
// ---------------------------------------------------------------------------

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = NUM_PARTITIONS;
    const SCHED: usize = 4;
    const STACK_WORDS: usize = 256;
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

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

// Use the unified harness: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS);

// ---------------------------------------------------------------------------
// Partition entry point
// ---------------------------------------------------------------------------

/// Partition entry: invoke SYS_SAMPLING_WRITE with kernel address as data ptr.
extern "C" fn test_partition_main() -> ! {
    // Unpack the port ID passed via r0.
    let port_id = kernel::unpack_r0!();

    // Issue SYS_SAMPLING_WRITE with:
    //   r1 = port_id (valid sampling port)
    //   r2 = 4 (data length)
    //   r3 = KERNEL_ADDR (invalid: points to kernel memory)
    let result = kernel::svc!(SYS_SAMPLING_WRITE, port_id, 4u32, KERNEL_ADDR);

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

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: single partition.
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
        stack_size: (STACK_WORDS * 4) as u32,
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

    store_kernel(k);

    hprintln!("  port_id: {}", port_id);
    hprintln!("  kernel_addr (r3): {:#010x}", KERNEL_ADDR);
    hprintln!("  expected error: {:#010x}", EXPECTED_ERROR);

    // Build partition array for boot().
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        [(test_partition_main, port_id as u32)];

    match boot(&parts, &mut p).expect("syscall_kernel_ptr: boot failed") {}
}
