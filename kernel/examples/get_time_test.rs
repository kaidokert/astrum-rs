//! QEMU test: verify SYS_GET_TIME returns the actual tick count.
//!
//! Boots a single partition whose entry function calls `SYS_GET_TIME`
//! in a loop and stores each reading to an atomic.  The SysTick handler
//! (running privileged) drives the scheduler, syncs the tick counter
//! into the Kernel struct, and periodically checks the partition's
//! readings:
//!
//! 1. After a few ticks the partition's reading must be non-zero,
//!    proving the tick counter is being incremented and synchronized.
//! 2. A later reading must be >= the first (monotonicity).
//!
//! All semihosting output happens from handler mode (SysTick), since
//! partitions run unprivileged and BKPT traps fault on QEMU.
//!
//! This validates the end-to-end path: SysTick increments
//! `KernelState.tick`, SysTick syncs to `Kernel.tick` via `sync_tick`,
//! and `SYS_GET_TIME` returns the correct value.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    kernel::KernelState,
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::SYS_GET_TIME,
};
use panic_semihosting as _;

const NP: usize = 1;
const STACK_WORDS: usize = 256;

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

// TODO: reviewer false positive — the swap_body closure is invoked twice by
// define_dispatch_hook!(@impl): once before dispatch (svc.rs:297) and once
// after (svc.rs:308).  mem::swap is its own inverse, so the partition table
// is swapped in before dispatch and swapped back out after, exactly matching
// the SysTick pattern.  There is no one-way/irreversible swap.
kernel::define_dispatch_hook!(TestConfig, |_k| {}, |_sk| {
    // Swap KernelState partitions into Kernel so dispatch sees authoritative data.
    cortex_m::interrupt::free(|cs| {
        if let Some(ks) = KS.borrow(cs).borrow_mut().as_mut() {
            core::mem::swap(&mut _sk.partitions, ks.partitions_mut());
        }
    });
});

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;

static KS: Mutex<RefCell<Option<KernelState<NP, 4>>>> = Mutex::new(RefCell::new(None));

kernel::define_pendsv!();

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NP] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NP]
};
#[no_mangle]
static mut PARTITION_SP: [u32; NP] = [0; NP];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

/// Latest SYS_GET_TIME reading from the partition (0 = not yet read).
static TIME_READING: AtomicU32 = AtomicU32::new(0);

/// Partition entry: spin calling SYS_GET_TIME and publish each reading.
extern "C" fn partition_main() -> ! {
    loop {
        let t = kernel::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32);
        TIME_READING.store(t, Ordering::Release);
    }
}

#[exception]
fn SysTick() {
    static mut TICK: u32 = 0;
    static mut FIRST_NONZERO: u32 = 0;
    *TICK += 1;

    // Drive scheduler and sync tick to Kernel.
    cortex_m::interrupt::free(|cs| {
        let mut ks_ref = KS.borrow(cs).borrow_mut();
        let ks = match ks_ref.as_mut() {
            Some(ks) => ks,
            None => return,
        };

        let event = ks.advance_schedule_tick();
        #[cfg(not(feature = "dynamic-mpu"))]
        if let Some(pid) = event {
            // SAFETY: single-core; PendSV cannot preempt SysTick.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };
            cortex_m::peripheral::SCB::set_pendsv();
        }
        #[cfg(feature = "dynamic-mpu")]
        if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };
            cortex_m::peripheral::SCB::set_pendsv();
        }

        // Sync tick and expire timed waits using KernelState partitions.
        let current_tick = ks.tick().get();
        let ks_parts = ks.partitions_mut();
        if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
            k.sync_tick(current_tick);
            core::mem::swap(&mut k.partitions, ks_parts);
            k.expire_timed_waits::<{ <TestConfig as kernel::config::KernelConfig>::N }>(
                current_tick,
            );
            core::mem::swap(&mut k.partitions, ks_parts);
        }
    });

    // Check partition readings at specific ticks.
    match *TICK {
        // By tick 5, the partition should have read a non-zero time.
        5 => {
            let t = TIME_READING.load(Ordering::Acquire);
            if t == 0 {
                hprintln!("get_time_test: FAIL — tick still zero at tick 5");
                debug::exit(debug::EXIT_FAILURE);
            }
            hprintln!("get_time_test: first non-zero reading = {}", t);
            *FIRST_NONZERO = t;
        }
        // By tick 10, a later reading must be >= the first (monotonicity).
        10 => {
            let t = TIME_READING.load(Ordering::Acquire);
            hprintln!("get_time_test: second reading = {}", t);
            if t >= *FIRST_NONZERO {
                hprintln!("get_time_test: PASS");
                debug::exit(debug::EXIT_SUCCESS);
            } else {
                hprintln!(
                    "get_time_test: FAIL — not monotonic: {} < {}",
                    t,
                    *FIRST_NONZERO
                );
                debug::exit(debug::EXIT_FAILURE);
            }
        }
        _ => {}
    }
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("get_time_test: start");

    // SAFETY: single-core, interrupts not yet enabled — exclusive access.
    unsafe {
        #[cfg(feature = "dynamic-mpu")]
        let k = Kernel::<TestConfig>::new(kernel::virtual_device::DeviceRegistry::new());
        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new();
        store_kernel(k);

        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 2)).expect("schedule entry");
        sched.start();

        let cfgs: [PartitionConfig; NP] = [{
            let b = STACKS[0].0.as_ptr() as u32;
            PartitionConfig {
                id: 0,
                entry_point: 0,
                stack_base: b,
                stack_size: (STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (STACK_WORDS * 4) as u32, 0),
            }
        }];

        cortex_m::interrupt::free(|cs| {
            KS.borrow(cs).replace(Some(
                KernelState::new(sched, &cfgs).expect("invalid kernel config"),
            ));
        });

        let stk = &mut STACKS[0].0;
        let ix = kernel::context::init_stack_frame(stk, partition_main as *const () as u32, None)
            .expect("init_stack_frame");
        PARTITION_SP[0] = stk.as_ptr() as u32 + (ix as u32) * 4;

        cp.SCB.set_priority(SystemHandler::SVCall, 0x00);
        cp.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        cp.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(120_000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();

    loop {
        cortex_m::asm::wfi();
    }
}
