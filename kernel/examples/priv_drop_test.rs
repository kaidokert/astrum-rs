//! QEMU test: verify partitions run unprivileged via CONTROL register.
//!
//! Boots a single partition whose entry function reads the CONTROL
//! register and checks:
//!
//! - **Bit 0 (nPRIV) = 1** — partition executes in unprivileged mode.
//! - **Bit 1 (SPSEL) = 1** — partition uses the Process Stack Pointer.
//!
//! On QEMU the semihosting BKPT trap works regardless of privilege
//! level, so the partition can report results and exit directly.
//!
//! This validates the end-to-end privilege drop from the PendSV handler
//! through partition execution.

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

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

// TODO: dynamic-mpu constants (BP, BZ, DR) and the Kernel::new() branch
// below are not relevant to this test but are required to satisfy the
// KernelConfig trait when CI compiles all examples with --features
// qemu,dynamic-mpu.  Remove once examples can opt out of feature
// combinations at the Cargo level.
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

/// Partition entry: reads CONTROL, asserts nPRIV and SPSEL, then exits.
extern "C" fn partition_main() -> ! {
    let control: u32;

    #[cfg(target_arch = "arm")]
    // SAFETY: MRS reads the CONTROL register with no side effects.
    // The partition is running in Thread mode so CONTROL reflects the
    // current privilege level and stack pointer selection.
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control);
    }

    #[cfg(not(target_arch = "arm"))]
    {
        control = 0;
    }

    let npriv = control & 1;
    let spsel = (control >> 1) & 1;

    hprintln!("priv_drop_test: CONTROL = {:#06x}", control);
    hprintln!("  nPRIV (bit 0) = {} (expect 1)", npriv);
    hprintln!("  SPSEL (bit 1) = {} (expect 1)", spsel);

    if npriv == 1 && spsel == 1 {
        hprintln!("priv_drop_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("priv_drop_test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }

    // TODO: reviewer false positive — debug::exit() returns `()`, not `!`.
    // The debugger may allow the program to continue, so this loop is
    // required to satisfy the `-> !` return type.
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals already taken");
    hprintln!("priv_drop_test: start");

    // Build schedule: single partition runs for 2 ticks.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 2))
        .expect("static schedule entry must fit");

    // Build partition configs using the STACKS addresses.
    // SAFETY: single-core, interrupts disabled — exclusive access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        [{
            let b = STACKS[0].0.as_ptr() as u32;
            PartitionConfig {
                id: 0,
                entry_point: 0,
                stack_base: b,
                stack_size: (STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (STACK_WORDS * 4) as u32, 0),
            }
        }]
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
