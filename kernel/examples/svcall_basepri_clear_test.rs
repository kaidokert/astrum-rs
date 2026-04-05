//! Defense-in-depth: verifies SVCall entry clears BASEPRI.
//!
//! A verifying dispatch hook reads BASEPRI from Handler mode (privileged,
//! not RAZ per ARMv7-M ARM B5.2.3) and flags a violation if non-zero.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::{EntryAddr, ExternalPartitionMemory, MpuRegion};
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny,
    StackStorage as _, SyncMinimal,
};

kernel::kernel_config!(
    Config < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        tick_period_us = 83;
    }
);

const NUM_PARTITIONS: usize = 2;
const CHECK_TICK: u32 = 20;
const TIMEOUT_TICK: u32 = 200;
const MIN_COUNT: u32 = 4;

static COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Set by the dispatch hook if BASEPRI is non-zero on SVCall entry
/// (meaning the assembly clear failed).
static BASEPRI_VIOLATION: AtomicBool = AtomicBool::new(false);

kernel::define_kernel!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    if tick >= CHECK_TICK {
        let c: [u32; 2] =
            core::array::from_fn(|i| COUNTS.get(i).map_or(0, |c| c.load(Ordering::Acquire)));
        if c.iter().all(|&v| v >= MIN_COUNT) {
            if BASEPRI_VIOLATION.load(Ordering::Acquire) {
                hprintln!("svcall_basepri_clear_test: FAIL basepri leak ({:?})", c);
                debug::exit(debug::EXIT_FAILURE);
            }
            hprintln!("svcall_basepri_clear_test: PASS ({:?})", c);
            debug::exit(debug::EXIT_SUCCESS);
        }
        if tick >= TIMEOUT_TICK {
            hprintln!("svcall_basepri_clear_test: FAIL timeout ({:?})", c);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
});

// TODO: verifying_dispatch_hook duplicates the yield-handling state machine from the
// macro-generated dispatch_hook (yield_requested / yield_current_slot / set_next_partition /
// drain_debug_auto).  The same pattern exists in svcall_primask_clear_test.  A stable
// kernel API (e.g. `Kernel::dispatch_and_yield`) would eliminate this DRY violation and
// the associated panic-in-Handler risk.  Deferred: requires kernel API change.
/// Verifying SVC dispatch hook: reads BASEPRI from Handler mode (privileged,
/// real value — not RAZ per ARMv7-M ARM B5.2.3) to check the assembly
/// defense-in-depth clear actually worked, then delegates to the kernel dispatch.
///
/// # Safety
///
/// Must be called from SVCall exception context with a valid `ExceptionFrame`.
/// The caller must ensure `f` points to the hardware-stacked exception frame on
/// the process stack, and that kernel state has been initialised via `store_kernel`.
// TODO: reviewer false positive — verifying_dispatch_hook is an SVC dispatch hook
// (signature: `fn(&mut ExceptionFrame)`), not an ISR handler (`fn()`). IsrHandler
// type assertion does not apply here.
unsafe extern "C" fn verifying_dispatch_hook(f: &mut kernel::context::ExceptionFrame) {
    #[cfg(target_arch = "arm")]
    {
        if cortex_m::register::basepri::read() != 0 {
            BASEPRI_VIOLATION.store(true, Ordering::Release);
        }
    }

    // Delegate to the real kernel dispatch, replicating the yield handling
    // from the macro-generated dispatch_hook.
    let _ = kernel::state::with_kernel_mut::<Config, _, _>(|k| {
        // SAFETY: `f` is the hardware-stacked SVCall exception frame from the
        // process stack, and we are in SVCall Handler mode — same invariants as
        // the macro-generated dispatch_hook.
        unsafe { k.dispatch(f) }
        if k.yield_requested {
            k.yield_requested = false;
            use kernel::svc::YieldResult;
            let result = k.yield_current_slot();
            if let Some(pid) = result.partition_id() {
                k.set_next_partition(pid);
            }
            k.drain_debug_auto();
        }
    });
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
        // Check handler-side verification (privileged read, not RAZ).
        if BASEPRI_VIOLATION.load(Ordering::Acquire) {
            fail_halt("BASEPRI non-zero in SVCall handler");
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
    let p = cortex_m::Peripherals::take().expect("basepri: peripherals");
    hprintln!("svcall_basepri_clear_test: start");
    let sched =
        ScheduleTable::<{ Config::SCHED }>::round_robin(NUM_PARTITIONS, 1).expect("basepri: sched");
    let mut stk0 = AlignedStack1K::ZERO;
    let mut stk1 = AlignedStack1K::ZERO;
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mem0 = ExternalPartitionMemory::new(
        &mut stk0.0,
        EntryAddr::from_entry(p0_main as PartitionEntry),
        sentinel_mpu,
        kernel::PartitionId::new(0),
    )
    .expect("ext mem");
    let mem1 = ExternalPartitionMemory::new(
        &mut stk1.0,
        EntryAddr::from_entry(p1_main as PartitionEntry),
        sentinel_mpu,
        kernel::PartitionId::new(1),
    )
    .expect("ext mem");
    let mut k = Kernel::<Config>::new(sched, &[mem0, mem1]).expect("basepri: kernel");
    store_kernel(&mut k);
    // Override dispatch with our verifying wrapper that reads BASEPRI
    // from Handler mode before delegating.
    kernel::svc::set_dispatch_hook(verifying_dispatch_hook);
    match boot(p).expect("basepri: boot") {}
}
