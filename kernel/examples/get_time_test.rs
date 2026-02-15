// Migrated to unified Kernel API.

//! QEMU test: verify SYS_GET_TIME returns the actual tick count.
//!
//! Boots a single partition whose entry function calls `SYS_GET_TIME`
//! in a loop and stores each reading to an atomic.  The SysTick handler
//! (running privileged) drives the scheduler and periodically checks
//! the partition's readings:
//!
//! 1. After a few ticks the partition's reading must be non-zero,
//!    proving the tick counter is being incremented.
//! 2. A later reading must be >= the first (monotonicity).
//!
//! All semihosting output happens from handler mode (SysTick), since
//! partitions run unprivileged and BKPT traps fault on QEMU.
//!
//! This validates the end-to-end path: SysTick increments the kernel
//! tick via `advance_schedule_tick`, and `SYS_GET_TIME` returns the
//! correct value via the unified Kernel struct.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
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
    syscall::SYS_GET_TIME,
};
use panic_semihosting as _;

const NP: usize = 1;
const STACK_WORDS: usize = 256;

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

kernel::define_unified_kernel!(TestConfig, |_k| {});

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;

kernel::define_pendsv!();

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NP] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NP]
};

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

    // Drive scheduler via unified Kernel.
    cortex_m::interrupt::free(|cs| {
        let mut k_ref = KERNEL.borrow(cs).borrow_mut();
        let k = match k_ref.as_mut() {
            Some(k) => k,
            None => return,
        };

        let event = k.advance_schedule_tick();
        #[cfg(not(feature = "dynamic-mpu"))]
        if let Some(pid) = event {
            k.set_next_partition(pid);
            cortex_m::peripheral::SCB::set_pendsv();
        }
        #[cfg(feature = "dynamic-mpu")]
        if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
            k.set_next_partition(pid);
            cortex_m::peripheral::SCB::set_pendsv();
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
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 2)).expect("schedule entry");

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

        #[cfg(not(feature = "dynamic-mpu"))]
        let k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel config");
        #[cfg(feature = "dynamic-mpu")]
        let k =
            Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
                .expect("kernel config");

        store_kernel(k);

        // Start the schedule after storing the kernel.
        cortex_m::interrupt::free(|cs| {
            if let Some(k) = KERNEL.borrow(cs).borrow_mut().as_mut() {
                k.start_schedule();
            }
        });

        let stk = &mut STACKS[0].0;
        let ix = kernel::context::init_stack_frame(stk, partition_main as *const () as u32, None)
            .expect("init_stack_frame");
        let sp = stk.as_ptr() as u32 + (ix as u32) * 4;
        set_partition_sp(0, sp);

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
