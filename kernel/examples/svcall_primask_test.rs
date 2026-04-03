//! Defense-in-depth: verifies PRIMASK=0 and BASEPRI=0 in Thread mode after
//! sys_get_time returns.  Validates pendstclr-basepri-ordering.md §Defense-in-Depth.
//!
//! Caveat: partitions run unprivileged, so MRS to PRIMASK/BASEPRI reads-as-zero
//! (RAZ) on ARMv7-M (ARM B5.2.3).  This test therefore validates the *Thread-mode
//! observable* state (which should always be zero) as a sanity backstop.  For
//! privileged Handler-mode verification that avoids RAZ, see
//! `svcall_primask_clear_test.rs`.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::{EntryAddr, ExternalPartitionMemory, MpuRegion};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal};

const STACK_WORDS: usize = 256;

kernel::compose_kernel_config!(
    Config<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = 2;
const CHECK_TICK: u32 = 5;
const TIMEOUT_TICK: u32 = 12;
const MIN_COUNT: u32 = 2;

static COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

kernel::define_unified_harness!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    let c: [u32; 2] =
        core::array::from_fn(|i| COUNTS.get(i).map_or(0, |c| c.load(Ordering::Acquire)));
    if tick >= CHECK_TICK && c.iter().all(|&v| v >= MIN_COUNT) {
        hprintln!("svcall_primask_test: PASS ({:?})", c);
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= TIMEOUT_TICK {
        hprintln!("svcall_primask_test: FAIL timeout ({:?})", c);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Read PRIMASK register.  Returns `true` when interrupts are enabled (PRIMASK=0).
///
/// Note: On ARMv7-M, unprivileged MRS to PRIMASK reads-as-zero (RAZ) per
/// ARMv7-M ARM B5.2.3, so this check is vacuously true for unprivileged
/// partitions.  It serves as a defense-in-depth sanity check — if the kernel
/// ever misconfigures partition privilege, or if the test is run in a
/// privileged configuration, a real non-zero PRIMASK will be caught.
/// See svcall_primask_clear_test.rs for the Handler-mode privileged read
/// variant that avoids the RAZ caveat.
#[inline(always)]
fn read_primask_active() -> bool {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::primask::read().is_active()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        true
    }
}

/// Read BASEPRI register.  Returns 0 when no priority masking is active.
///
/// Same RAZ caveat as `read_primask_active` applies for unprivileged partitions.
#[inline(always)]
fn read_basepri() -> u32 {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::register::basepri::read() as u32
    }
    #[cfg(not(target_arch = "arm"))]
    {
        0
    }
}

fn fail_halt(msg: &str) -> ! {
    hprintln!("{}", msg);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        cortex_m::asm::nop();
    }
}

fn partition_task(id: usize) -> ! {
    let counter = match COUNTS.get(id) {
        Some(c) => c,
        None => fail_halt("partition id out of range"),
    };
    loop {
        if plib::sys_yield().is_err() {
            fail_halt("sys_yield failed");
        }
        let t = match plib::sys_get_time() {
            Ok(v) => v,
            Err(_) => fail_halt("sys_get_time failed"),
        };
        if !read_primask_active() {
            fail_halt("PRIMASK non-zero after sys_get_time");
        }
        if read_basepri() != 0 {
            fail_halt("BASEPRI non-zero after sys_get_time");
        }
        if counter.load(Ordering::Relaxed) > 0 && t == 0 {
            fail_halt("sys_get_time stuck at zero");
        }
        counter.fetch_add(1, Ordering::Release);
    }
}

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    partition_task(0)
}
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    partition_task(1)
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("svcall_primask: peripherals");
    hprintln!("svcall_primask_test: start");
    let mut sched = ScheduleTable::<{ Config::SCHED }>::new();
    sched
        .add(kernel::scheduler::ScheduleEntry::new(0, 3))
        .expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched
        .add(kernel::scheduler::ScheduleEntry::new(1, 3))
        .expect("add P1");
    sched.add_system_window(1).expect("sys1");

    // SAFETY: called once from main before any interrupt handler runs.
    let stacks = unsafe {
        &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NUM_PARTITIONS]>()
    };
    let [ref mut s0, ref mut s1] = *stacks;
    let mpu = MpuRegion::new(0, 0, 0);
    let e = EntryAddr::from_entry;
    let memories = [
        ExternalPartitionMemory::new(
            s0,
            e(p0_main as PartitionEntry),
            mpu,
            kernel::PartitionId::new(0),
        )
        .expect("mem 0"),
        ExternalPartitionMemory::new(
            s1,
            e(p1_main as PartitionEntry),
            mpu,
            kernel::PartitionId::new(1),
        )
        .expect("mem 1"),
    ];
    let mut k = Kernel::<Config>::new(sched, &memories).expect("svcall_primask: kernel");
    store_kernel(&mut k);
    match boot(p).expect("svcall_primask: boot") {}
}
