//! QEMU test: asymmetric peripheral region counts (2 vs 3).
//!
//! Boots two partitions with different peripheral region counts:
//! - P0: 2 peripheral regions (UART0 + TIMER0)
//! - P1: 3 peripheral regions (UART0 + TIMER0 + I2C0)
//!
//! Each partition performs volatile write+read to its granted peripherals.
//! The tick handler verifies that after context-switching to P0, the third
//! peripheral MPU slot (R6) is disabled — proving P1's extra peripheral
//! (I2C0) is not leaked to P0.
//!
//! Run:
//!   cargo run --example asymmetric_periph_test --features qemu,log-semihosting \
//!     --target thumbv7m-none-eabi --release
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const STACK_WORDS: usize = 256;

// Peripheral base addresses on LM3S6965.
const UART0_BASE: u32 = 0x4000_C000;
const TIMER0_BASE: u32 = 0x4003_0000;
const I2C0_BASE: u32 = 0x4002_0000;
const PERIPH_SIZE: u32 = 4096;

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// State flags (one per partition):
///   0 = not started, 1 = volatile accesses OK.
static P0_RESULT: AtomicU32 = AtomicU32::new(0);
static P1_RESULT: AtomicU32 = AtomicU32::new(0);

const TIMEOUT_TICK: u32 = 100;

/// Read a single MPU region's RBAR/RASR.
///
/// # Safety
/// Caller must have exclusive access to the MPU peripheral (e.g. from
/// SysTick preempting PendSV).
unsafe fn read_mpu_rasr(mpu: &cortex_m::peripheral::MPU, region: u32) -> u32 {
    mpu.rnr.write(region);
    mpu.rasr.read()
}

kernel::define_unified_harness!(TestConfig, |tick, k| {
    let p0_ok = P0_RESULT.load(Ordering::Acquire) == 1;
    let p1_ok = P1_RESULT.load(Ordering::Acquire) == 1;

    // Once both partitions have confirmed peripheral access, verify
    // MPU isolation: when P0 is the current partition, the third
    // peripheral slot (R6) must be disabled.
    if p0_ok && p1_ok && tick >= 8 {
        let pid = k.current_partition;
        if pid == 0 {
            // SAFETY: SysTick preempts PendSV → exclusive MPU access.
            let p = unsafe { cortex_m::Peripherals::steal() };
            // TODO: R6 == MPU region 6 assumes peripheral slots start at 4.
            // The kernel should expose a FIRST_PERIPHERAL_SLOT constant so tests
            // can compute the correct slot instead of hard-coding magic numbers.
            // R6 is MPU region 6, the third peripheral-reserved slot.
            // For P0 (which has only 2 peripherals), R6 must be disabled.
            // SAFETY: SysTick preempts PendSV, so we have exclusive MPU access.
            // `read_mpu_rasr` only reads RNR/RASR which is safe under that guarantee.
            let rasr6 = unsafe { read_mpu_rasr(&p.MPU, 6) };
            let enabled = rasr6 & 1;
            if enabled != 0 {
                hprintln!(
                    "asymmetric_periph_test: FAIL R6 not disabled for P0 (RASR={:#010x})",
                    rasr6
                );
                kernel::kexit!(failure);
            }
            hprintln!("asymmetric_periph_test: P0 2-periph access OK");
            hprintln!("asymmetric_periph_test: P1 3-periph access OK");
            hprintln!("asymmetric_periph_test: R6 disabled for P0 (isolation OK)");
            hprintln!("asymmetric_periph_test: PASS");
            kernel::kexit!(success);
        }
    }

    if tick >= TIMEOUT_TICK {
        hprintln!(
            "asymmetric_periph_test: FAIL timeout (P0={}, P1={})",
            P0_RESULT.load(Ordering::Acquire),
            P1_RESULT.load(Ordering::Acquire)
        );
        kernel::kexit!(failure);
    }
});

/// P0: access UART0 + TIMER0 (2 peripherals).
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    let uart0_dr = UART0_BASE as *mut u32;
    let timer0_cfg = TIMER0_BASE as *mut u32;

    // SAFETY: UART0 DR and TIMER0 CFG are valid MMIO registers on lm3s6965evb.
    // We have MPU-granted access to both regions.
    unsafe {
        core::ptr::write_volatile(uart0_dr, 0x41);
        let _u = core::ptr::read_volatile(uart0_dr);

        core::ptr::write_volatile(timer0_cfg, 0x0);
        let _t = core::ptr::read_volatile(timer0_cfg);
    }

    P0_RESULT.store(1, Ordering::Release);

    loop {
        plib::sys_yield().expect("p0 yield");
    }
}

/// P1: access UART0 + TIMER0 + I2C0 (3 peripherals).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    let uart0_dr = UART0_BASE as *mut u32;
    let timer0_cfg = TIMER0_BASE as *mut u32;
    let i2c0_msa = I2C0_BASE as *mut u32;

    // SAFETY: UART0 DR, TIMER0 CFG, and I2C0 MSA are valid MMIO registers
    // on lm3s6965evb.  We have MPU-granted access to all three regions.
    unsafe {
        core::ptr::write_volatile(uart0_dr, 0x42);
        let _u = core::ptr::read_volatile(uart0_dr);

        core::ptr::write_volatile(timer0_cfg, 0x0);
        let _t = core::ptr::read_volatile(timer0_cfg);

        core::ptr::write_volatile(i2c0_msa, 0x0);
        let _i = core::ptr::read_volatile(i2c0_msa);
    }

    P1_RESULT.store(1, Ordering::Release);

    loop {
        plib::sys_yield().expect("p1 yield");
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("asymmetric_periph_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");

    let p0_periphs = [
        MpuRegion::new(UART0_BASE, PERIPH_SIZE, 0),
        MpuRegion::new(TIMER0_BASE, PERIPH_SIZE, 0),
    ];
    let p1_periphs = [
        MpuRegion::new(UART0_BASE, PERIPH_SIZE, 0),
        MpuRegion::new(TIMER0_BASE, PERIPH_SIZE, 0),
        MpuRegion::new(I2C0_BASE, PERIPH_SIZE, 0),
    ];

    // TODO: reviewer false positive — `__PARTITION_STACKS` is defined by
    // `define_unified_harness!` via `@impl_compat`; `store_kernel` and `boot`
    // are likewise generated by the macro expansion.
    let mut k = {
        let ptr = (&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>();
        // SAFETY: `__PARTITION_STACKS` is a macro-generated static with layout
        // `[[u32; STACK_WORDS]; NP]`.  We have `&mut` exclusivity here (called
        // once from `main`) and the cast preserves the original type.
        let stacks = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(
                s0,
                EntryAddr::from_entry(p0_entry as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(0),
            )
            .expect("mem 0")
            .with_peripheral_regions(&p0_periphs)
            .expect("periph 0"),
            ExternalPartitionMemory::new(
                s1,
                EntryAddr::from_entry(p1_entry as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(1),
            )
            .expect("mem 1")
            .with_peripheral_regions(&p1_periphs)
            .expect("periph 1"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
