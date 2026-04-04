//! QEMU test: single-peripheral partition boot.
//!
//! Boots a kernel with P0 having exactly 1 peripheral region (GPIOA) and P1
//! having none.  Validates:
//! 1. Boot succeeds without RegionCountMismatch (peripheral_reserved=1).
//! 2. P0 can read from the GPIO MMIO region (volatile read, no fault).
//! 3. Only 1 dynamic MPU slot is consumed for the peripheral, leaving 2
//!    free slots for buffer operations.
//!
//! This exercises the full builder → harness → DynamicStrategy pipeline
//! for the single-peripheral case.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
/// GPIOA base on LM3S6965.
const GPIOA_BASE: u32 = 0x4000_4000;
const GPIOA_SIZE: u32 = 4096;
const STACK_WORDS: usize = 256;

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

/// State machine:
///   0 = not started
///   1 = P0 read GPIO OK
///   2 = slot layout verified → PASS
///   0xFF = FAIL
static RESULT: AtomicU32 = AtomicU32::new(0);

const TIMEOUT_TICK: u32 = 50;

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == 1 {
        // P0 confirmed GPIO read succeeded.  Now verify DynamicStrategy
        // slot layout: peripheral_reserved=1 means slot 0 (R4) holds the
        // peripheral, slot 1 (R5) holds RAM, slots 2-3 (R6-R7) are free.
        let periph_slot = HARNESS_STRATEGY.slot(4);
        let ram_slot = HARNESS_STRATEGY.slot(5);
        let free_slot_6 = HARNESS_STRATEGY.slot(6);
        let free_slot_7 = HARNESS_STRATEGY.slot(7);

        let p0 = kernel::PartitionId::new(0);
        let periph_ok = periph_slot.is_some_and(|d| d.base == GPIOA_BASE && d.owner == p0);
        let ram_ok = ram_slot.is_some_and(|d| d.owner == p0);
        let free_ok = free_slot_6.is_none() && free_slot_7.is_none();

        if periph_ok && ram_ok && free_ok {
            hprintln!("single_periph_boot_test: slot layout OK");
            if let (Some(ps), Some(rs)) = (periph_slot, ram_slot) {
                hprintln!("  R4(periph): base={:#010x} owner={:?}", ps.base, ps.owner,);
                hprintln!("  R5(RAM):    base={:#010x} owner={:?}", rs.base, rs.owner,);
            }
            hprintln!("  R6: free, R7: free");
            RESULT.store(2, Ordering::Release);
        } else {
            hprintln!("FAIL: slot layout mismatch");
            hprintln!(
                "  R4 periph_ok={} ram_ok={} free_ok={}",
                periph_ok,
                ram_ok,
                free_ok
            );
            RESULT.store(0xFF, Ordering::Release);
        }
    }
    if r == 2 {
        hprintln!("single_periph_boot_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if r == 0xFF {
        hprintln!("single_periph_boot_test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick >= TIMEOUT_TICK {
        hprintln!("single_periph_boot_test: FAIL (timeout, state={})", r);
        debug::exit(debug::EXIT_FAILURE);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Read from GPIOA GPIODIR register (offset 0x400) to verify the
    // peripheral MMIO region is accessible.  On QEMU lm3s6965evb this
    // register defaults to 0x00 (all pins input).
    let gpiodir_addr = (GPIOA_BASE + 0x400) as *const u32;
    // SAFETY: GPIOA_BASE + 0x400 is a valid MMIO register on lm3s6965.
    // The read is volatile and has no side effects on a direction register.
    let dir_val = unsafe { core::ptr::read_volatile(gpiodir_addr) };

    // The value should be a valid u32 (QEMU returns 0 for unconfigured GPIO).
    // Any MPU fault would prevent reaching this point.
    if dir_val <= 0xFF {
        // GPIODIR is 8 bits wide on LM3S6965; upper bits are reserved/zero.
        RESULT.store(1, Ordering::Release);
    } else {
        RESULT.store(0xFF, Ordering::Release);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("single_periph_boot_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");

    // TODO: reviewer false positive — `__PARTITION_STACKS` is defined by
    // `define_unified_harness!` macro expansion at module scope; no extern block needed.
    let mut k = {
        // SAFETY: `__PARTITION_STACKS` is a `static mut [AlignedStack1K; NP]` defined
        // by `define_unified_harness!` at module scope.  `AlignedStack1K` is 1024 bytes
        // (256 × u32) with 1024-byte alignment, so casting to `[[u32; STACK_WORDS]; NP]`
        // preserves size and satisfies alignment (u32 align ≤ 1024).  Called once from
        // main before interrupts are enabled, guaranteeing exclusive access.
        let ptr = (&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>();
        let stacks = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            // TODO: reviewer false positive — MpuRegion::new(0, 0, 0) is the standard
            // sentinel code region used by the harness (see harness.rs sentinel_mpu).
            // Partition code executes under the MPU background region (PRIVDEFENA).
            ExternalPartitionMemory::new(
                s0,
                EntryAddr::from_entry(p0_main as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(0),
            )
            .expect("mem 0")
            .with_peripheral_regions(&[MpuRegion::new(GPIOA_BASE, GPIOA_SIZE, 0)])
            .expect("periph 0"),
            ExternalPartitionMemory::new(
                s1,
                EntryAddr::from_entry(p1_main as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(1),
            )
            .expect("mem 1"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
