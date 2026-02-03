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
    kernel::KernelState,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 1;
const MAX_SCHEDULE_ENTRIES: usize = 4;
const STACK_WORDS: usize = 256;

// TODO: dynamic-mpu constants (BP, BZ, DR) and the Kernel::new() branch
// below are not relevant to this test but are required to satisfy the
// KernelConfig trait when CI compiles all examples with --features
// qemu,dynamic-mpu.  Remove once examples can opt out of feature
// combinations at the Cargo level.
struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 1;
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
}

kernel::define_harness!(
    TestConfig,
    NUM_PARTITIONS,
    MAX_SCHEDULE_ENTRIES,
    STACK_WORDS
);

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

    // SAFETY: single-core, interrupts not yet enabled — exclusive access
    // to all statics.
    unsafe {
        #[cfg(feature = "dynamic-mpu")]
        let k = Kernel::<TestConfig>::new(kernel::virtual_device::DeviceRegistry::new());
        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new();
        store_kernel(k);

        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        sched
            .add(ScheduleEntry::new(0, 2))
            .expect("static schedule entry must fit");
        sched.start();

        // TODO: MpuRegion is structurally required by PartitionConfig
        // (validation enforces size >= 32).  This test only cares about
        // the CONTROL register, not MPU behaviour; the region below is
        // the minimum valid configuration.
        let cfgs: [PartitionConfig; NUM_PARTITIONS] = [{
            let b = 0x2000_0000u32;
            PartitionConfig {
                id: 0,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        }];
        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    boot(&parts, &mut p)
}
