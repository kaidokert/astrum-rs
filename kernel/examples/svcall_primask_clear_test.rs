//! Defense-in-depth test: two partitions call sys_yield() and sys_get_time()
//! in a loop under fast SysTick+PendSV stress.
//!
//! Verification strategy: a custom SVC dispatch hook reads PRIMASK and BASEPRI
//! from Handler mode (privileged) inside the SVCall exception — after the
//! assembly defense-in-depth clears but before returning to Thread mode.
//! This avoids the ARMv7-M Read-As-Zero (RAZ) caveat that makes unprivileged
//! MRS to PRIMASK/BASEPRI vacuously return 0 (ARMv7-M ARM B5.2.3).
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
use kernel::{DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal};

const STACK_WORDS: usize = 256;

kernel::kernel_config!(
    Config<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = 2;
const CHECK_TICK: u32 = 5;
const TIMEOUT_TICK: u32 = 12;
const MIN_COUNT: u32 = 2;

static PARTITION_COUNTS: [AtomicU32; NUM_PARTITIONS] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Set by the verifying dispatch hook if PRIMASK or BASEPRI is non-zero
/// inside the SVCall handler (Handler mode, privileged read — not RAZ).
static HANDLER_MASK_VIOLATION: AtomicBool = AtomicBool::new(false);

kernel::define_kernel!(Config, |tick, _k| {
    #[cfg(target_arch = "arm")]
    cortex_m::peripheral::SCB::set_pendsv();

    // Check for mask violations on every tick — fail fast if PRIMASK/BASEPRI
    // leaked, rather than hanging until timeout.
    if HANDLER_MASK_VIOLATION.load(Ordering::Acquire) {
        let c: [u32; 2] = core::array::from_fn(|i| {
            PARTITION_COUNTS
                .get(i)
                .map_or(0, |c| c.load(Ordering::Acquire))
        });
        hprintln!(
            "svcall_primask_clear_test: FAIL mask leak at tick {} ({:?})",
            tick,
            c
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    if tick >= CHECK_TICK {
        let c: [u32; 2] = core::array::from_fn(|i| {
            PARTITION_COUNTS
                .get(i)
                .map_or(0, |c| c.load(Ordering::Acquire))
        });
        if c.iter().all(|&v| v >= MIN_COUNT) {
            hprintln!("svcall_primask_clear_test: PASS ({:?})", c);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    if tick >= TIMEOUT_TICK {
        let c: [u32; 2] = core::array::from_fn(|i| {
            PARTITION_COUNTS
                .get(i)
                .map_or(0, |c| c.load(Ordering::Acquire))
        });
        hprintln!("svcall_primask_clear_test: FAIL timeout ({:?})", c);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Verifying SVC dispatch hook: reads PRIMASK and BASEPRI from Handler mode
/// (privileged — real values, not RAZ) to check the assembly defense-in-depth
/// clears actually worked, then delegates to the kernel dispatch.
///
/// # Safety
///
/// Must be called from SVCall exception context with a valid `ExceptionFrame`.
// TODO: reviewer false positive — verifying_dispatch_hook is an SVC dispatch hook
// (signature: `fn(&mut ExceptionFrame)`), not an ISR handler (`fn()`). IsrHandler
// type assertion does not apply here.
unsafe extern "C" fn verifying_dispatch_hook(f: &mut kernel::context::ExceptionFrame) {
    // Read PRIMASK and BASEPRI from Handler mode (privileged, real values).
    // cortex_m 0.7 Primask::is_inactive() returns true when PRIMASK bit == 1
    // (i.e. configurable exceptions inactive / interrupts masked).
    // Reference: cortex-m-0.7.7/src/register/primask.rs — read() maps bit=1 → Inactive.
    #[cfg(target_arch = "arm")]
    {
        let primask_set = cortex_m::register::primask::read().is_inactive();
        let basepri_nonzero = cortex_m::register::basepri::read() != 0;
        if primask_set || basepri_nonzero {
            HANDLER_MASK_VIOLATION.store(true, Ordering::Release);
        }
    }
    // Delegate to the macro-generated dispatch_hook which handles dispatch
    // and post-dispatch yield logic, avoiding manual duplication.
    // SAFETY: called from SVCall exception context with a valid ExceptionFrame.
    unsafe { dispatch_hook(f) }
}

fn fail_halt(msg: &str) -> ! {
    hprintln!("{}", msg);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        cortex_m::asm::nop();
    }
}

fn partition_task(id: usize) -> ! {
    let counter = match PARTITION_COUNTS.get(id) {
        Some(c) => c,
        None => fail_halt("partition id out of range"),
    };
    loop {
        if plib::sys_yield().is_err() {
            fail_halt("sys_yield failed");
        }
        // Check handler-side verification (privileged read, not RAZ).
        if HANDLER_MASK_VIOLATION.load(Ordering::Acquire) {
            fail_halt("PRIMASK/BASEPRI non-zero in SVCall handler");
        }
        // Also exercise sys_get_time to cover a second syscall path.
        if plib::sys_get_time().is_err() {
            fail_halt("sys_get_time failed");
        }
        if HANDLER_MASK_VIOLATION.load(Ordering::Acquire) {
            fail_halt("PRIMASK/BASEPRI non-zero in SVCall handler");
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
    let p = cortex_m::Peripherals::take().expect("svcall: peripherals");
    hprintln!("svcall_primask_clear_test: start");
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
    let mut k = Kernel::<Config>::new(sched, &memories).expect("svcall: kernel");
    // boot() and store_kernel() are generated by define_kernel!().
    store_kernel(&mut k);
    // Override the macro-generated dispatch hook with our verifying wrapper
    // that reads PRIMASK/BASEPRI from Handler mode before delegating.
    kernel::svc::set_dispatch_hook(verifying_dispatch_hook);
    match boot(p).expect("svcall: boot") {}
}
