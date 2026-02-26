//! QEMU test: verify partitions run unprivileged via CONTROL register.
//!
//! Boots a single partition whose entry function reads the CONTROL
//! register and checks:
//!
//! - **Bit 0 (nPRIV) = 1** — partition executes in unprivileged mode.
//! - **Bit 1 (SPSEL) = 1** — partition uses the Process Stack Pointer.
//!
//! The partition stores its CONTROL reading in an atomic; the SysTick
//! hook (privileged Handler mode) reads it and reports via semihosting.
//!
//! This validates the end-to-end privilege drop from the PendSV handler
//! through partition execution.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::{AlignedStack1K, PartitionCore},
    port_pools::PortPools,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    sync_pools::SyncPools,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 1;
    const SM: usize = 1;
    const BM: usize = 1;

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

/// Partition stores CONTROL reading here; 0 = not yet read.
static CONTROL_VAL: AtomicU32 = AtomicU32::new(0);
/// Set to 1 once the partition has stored its reading.
static DONE: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
    if DONE.load(Ordering::Acquire) == 1 {
        let control = CONTROL_VAL.load(Ordering::Acquire);
        let npriv = control & 1;
        let spsel = (control >> 1) & 1;
        hprintln!("priv_drop_test: CONTROL = {:#06x}", control);
        if npriv == 1 && spsel == 1 {
            hprintln!("priv_drop_test: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("priv_drop_test: FAIL");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    if tick > 100 {
        hprintln!("priv_drop_test: FAIL - timeout");
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Partition entry: reads CONTROL and stores it in an atomic.
extern "C" fn partition_main() -> ! {
    let control: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("mrs {0}, CONTROL", out(reg) control);
    }
    #[cfg(not(target_arch = "arm"))]
    {
        control = 0;
    }
    CONTROL_VAL.store(control, Ordering::Release);
    DONE.store(1, Ordering::Release);
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

    // Build partition configs. Stack bases are derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = [PartitionConfig {
        id: 0,
        entry_point: 0, // Not used by Kernel::new
        stack_base: 0,  // Ignored: internal stack used
        stack_size: (STACK_WORDS * 4) as u32,
        mpu_region: MpuRegion::new(0, 0, 0), // Base/size overridden by Kernel::new
        peripheral_regions: heapless::Vec::new(),
    }];

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel creation");

    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    match boot(&parts, &mut p).expect("priv_drop_test: boot failed") {}
}
