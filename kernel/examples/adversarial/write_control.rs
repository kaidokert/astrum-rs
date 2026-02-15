//! Adversarial test: unprivileged partition attempts to clear CONTROL.nPRIV.
//!
//! On Cortex-M3 in Thread mode with nPRIV=1, the nPRIV bit is sticky —
//! unprivileged code cannot set itself back to privileged mode. Writing
//! CONTROL with nPRIV=0 is silently ignored (no fault, but no effect).
//!
//! This test verifies:
//! 1. Partition boots in unprivileged mode (nPRIV=1)
//! 2. Partition attempts MSR CONTROL, r0 with nPRIV=0
//! 3. No fault occurs
//! 4. CONTROL.nPRIV still reads as 1 after the attempt
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example write_control

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
    partition_core::PartitionCore,
    port_pools::PortPools,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    sync_pools::SyncPools,
};
use panic_semihosting as _;

/// Test name for reporting.
const TEST_NAME: &str = "write_control";

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 1;
    const SCHED: usize = 4;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

// Use the unified harness macro: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS);

/// Partition entry: attempts to escalate privilege by clearing nPRIV.
///
/// Reads CONTROL, verifies nPRIV=1, writes CONTROL with nPRIV=0,
/// reads again and verifies nPRIV is still 1 (write was ignored).
extern "C" fn partition_main() -> ! {
    // Step 1: Read initial CONTROL register
    let control_before: u32;
    #[cfg(target_arch = "arm")]
    // SAFETY: MRS reads the CONTROL register with no side effects.
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control_before);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control_before = 3; // Simulate nPRIV=1, SPSEL=1 for host tests
    }

    let npriv_before = control_before & 1;
    hprintln!("{}: CONTROL before = {:#06x}", TEST_NAME, control_before);
    hprintln!("  nPRIV = {} (expect 1)", npriv_before);

    // Verify we start unprivileged
    if npriv_before != 1 {
        hprintln!("{}: FAIL - partition not unprivileged at start", TEST_NAME);
        debug::exit(debug::EXIT_FAILURE);
        loop {
            cortex_m::asm::nop();
        }
    }

    // Step 2: Attempt to clear nPRIV (escalate to privileged)
    // Keep SPSEL=1 (bit 1), clear nPRIV (bit 0)
    let control_write = control_before & !1u32; // Clear nPRIV bit
    hprintln!(
        "{}: attempting MSR CONTROL, {:#06x} (nPRIV=0)",
        TEST_NAME,
        control_write
    );

    #[cfg(target_arch = "arm")]
    // SAFETY: MSR CONTROL writes to the CONTROL register. In unprivileged
    // Thread mode, writes to nPRIV are ignored (sticky bit). ISB ensures
    // the pipeline sees any potential state change.
    unsafe {
        core::arch::asm!(
            "msr CONTROL, {0}",
            "isb",
            in(reg) control_write,
        );
    }

    // Step 3: Read CONTROL again after the write attempt
    let control_after: u32;
    #[cfg(target_arch = "arm")]
    // SAFETY: MRS reads the CONTROL register with no side effects.
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control_after);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control_after = 3; // Simulate unchanged for host tests
    }

    let npriv_after = control_after & 1;
    hprintln!("{}: CONTROL after = {:#06x}", TEST_NAME, control_after);
    hprintln!("  nPRIV = {} (expect 1)", npriv_after);

    // Step 4: Verify nPRIV is still 1 (write was ignored)
    if npriv_after == 1 {
        hprintln!("{}: nPRIV unchanged (write was ignored)", TEST_NAME);
        hprintln!("{}: PASS", TEST_NAME);
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!(
            "{}: FAIL - nPRIV changed to {} (privilege escalation!)",
            TEST_NAME,
            npriv_after
        );
        debug::exit(debug::EXIT_FAILURE);
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals already taken");
    hprintln!("{}: start", TEST_NAME);

    // Build schedule: partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("static schedule entry must fit");

    // Build partition config using the STACKS addresses.
    // SAFETY: single-core, interrupts disabled — exclusive access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        core::array::from_fn(|i| {
            let b = STACKS[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: (STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (STACK_WORDS * 4) as u32, 0),
            }
        })
    };

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    boot(&parts, &mut p)
}
