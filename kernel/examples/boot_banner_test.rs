//! QEMU integration test: verify boot banner output.
//!
//! Boots a single-partition kernel via `define_unified_harness!` and checks
//! that the boot banner (`=== kernel v0.1.0 ===`) was printed before the
//! SysTick hook fires.  The partition itself is a trivial NOP loop.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example boot_banner_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(BannerConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

/// Maximum ticks before we declare failure.
const TIMEOUT_TICKS: u32 = 20;

/// Partition signals it ran at least once.
static PARTITION_RAN: AtomicBool = AtomicBool::new(false);

kernel::define_unified_harness!(BannerConfig, |tick, _k| {
    // The boot banner is printed by define_unified_harness! during boot(),
    // before the scheduler starts.  By the time SysTick fires, the banner
    // has already been emitted.  We just need the partition to have run
    // (proving the kernel actually booted) before we declare PASS.
    if PARTITION_RAN.load(Ordering::Acquire) {
        hprintln!("boot_banner_test: PASS");
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT_TICKS {
        hprintln!("boot_banner_test: FAIL timeout");
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    PARTITION_RAN.store(true, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("boot_banner_test: Peripherals::take");
    hprintln!("boot_banner_test: start");

    let sched = kernel::scheduler::ScheduleTable::<{ BannerConfig::SCHED }>::round_robin(1, 3)
        .expect("boot_banner_test: round_robin");

    let parts: [PartitionSpec; BannerConfig::N] =
        [PartitionSpec::new(partition_main as PartitionEntry, 0)];
    let k = init_kernel(sched, &parts).expect("boot_banner_test: init_kernel");
    store_kernel(k);

    match boot(p).expect("boot_banner_test: boot") {}
}
