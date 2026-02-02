//! 3-partition shared-config demo: blackboard + semaphore + events.
//!
//! Demonstrates: blackboard display/read with wake-all semantics,
//! semaphore-guarded critical sections, and event-based completion
//! signalling between a config partition and two worker partitions.
//!
//! R0 packing scheme (passed to each partition at entry):
//!   bits [31:24] = partition ID (so workers can self-identify)
//!   bits [23:16] = semaphore ID
//!   bits [15:0]  = blackboard ID
// TODO: The per-example boilerplate (statics, svc macro, unpack_r0, SysTick,
// partition/scheduler setup) is duplicated across all QEMU examples. Extract
// a shared `qemu_harness` module or proc-macro crate to eliminate this when
// the kernel's example infrastructure matures.
#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, SCB};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    context::init_stack_frame,
    kernel::KernelState,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    svc::Kernel,
    syscall::{
        SYS_BB_DISPLAY, SYS_BB_READ, SYS_EVT_SET, SYS_EVT_WAIT, SYS_SEM_SIGNAL, SYS_SEM_WAIT,
        SYS_YIELD,
    },
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config (tuned to this example's resource needs)
// ---------------------------------------------------------------------------
const MAX_SCHEDULE_ENTRIES: usize = 4;
const NUM_PARTITIONS: usize = 3;

/// Kernel configuration for the blackboard demo.
///
/// Sized for 3 partitions, 1 semaphore, 1 blackboard (4-byte messages,
/// 3-deep wait queue), and minimal allocations for unused resource pools.
struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 3;
    const S: usize = 1;
    const SW: usize = 3;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 4;
    const BW: usize = 3;
}

// ---------------------------------------------------------------------------
// R0 packing helpers
// ---------------------------------------------------------------------------

/// Pack partition ID, semaphore ID, and blackboard ID into a single u32.
const fn pack_r0(partition_id: u32, sem: u32, bb: u32) -> u32 {
    (partition_id << 24) | (sem << 16) | bb
}

// ---------------------------------------------------------------------------
// Boilerplate: kernel statics, SVC dispatch hook, asm macros
// ---------------------------------------------------------------------------
static mut STACKS: [[u32; 256]; NUM_PARTITIONS] = [[0; 256]; NUM_PARTITIONS];
#[no_mangle]
static mut PARTITION_SP: [u32; NUM_PARTITIONS] = [0; NUM_PARTITIONS];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;
static mut KERN: Option<Kernel<DemoConfig>> = None;
static mut KS: Option<KernelState<{ DemoConfig::N }, MAX_SCHEDULE_ENTRIES>> = None;
#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;
kernel::define_pendsv!();

unsafe extern "C" fn hook(f: &mut kernel::context::ExceptionFrame) {
    let p = &raw mut KERN;
    if let Some(k) = unsafe { (*p).as_mut() } {
        unsafe { k.dispatch(f) }
    }
}

macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r: u32;
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!(
                "svc #0",
                inout("r0") $id => r,
                in("r1") $a,
                in("r2") $b,
                in("r3") $c,
                out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        {
            let _ = ($id, $a, $b, $c);
            r = 0;
        }
        r
    }};
}

/// Read the value the kernel placed in r0 before entering this partition.
macro_rules! unpack_r0 {
    () => {{
        let p: u32;
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!("", out("r0") p)
        }
        #[cfg(not(target_arch = "arm"))]
        {
            p = 0;
        }
        p
    }};
}

// ---------------------------------------------------------------------------
// Config partition: displays config on blackboard, waits for worker acks
// ---------------------------------------------------------------------------
extern "C" fn config_main() -> ! {
    let packed = unpack_r0!();
    let bb = packed & 0xFFFF;

    for round in 0..2u8 {
        let cfg = [round + 1, 10 + round];
        hprintln!("[config] display v={} thresh={}", cfg[0], cfg[1]);
        svc!(SYS_BB_DISPLAY, bb, 2u32, cfg.as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);

        let mask = 0x03u32;
        let mut got = svc!(SYS_EVT_WAIT, 0u32, mask, 0u32);
        while got & mask != mask {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            got = svc!(SYS_EVT_WAIT, 0u32, mask, 0u32);
        }
        hprintln!("[config] round {} done", round);
    }

    hprintln!("blackboard_demo: all checks passed");
    debug::exit(debug::EXIT_SUCCESS);
    loop {}
}

// ---------------------------------------------------------------------------
// Worker: reads config from blackboard, acquires semaphore, signals event
// ---------------------------------------------------------------------------
fn worker(tag: &str, bb: u32, sem: u32, partition_id: u32, evt: u32) -> ! {
    loop {
        let mut buf = [0u8; 4];
        let sz = svc!(SYS_BB_READ, bb, 0u32, buf.as_mut_ptr() as u32);
        if sz > 0 && sz != u32::MAX {
            hprintln!("[{}] cfg v={} thresh={}", tag, buf[0], buf[1]);
            svc!(SYS_SEM_WAIT, sem, partition_id, 0u32);
            hprintln!("[{}] sem acquire+release", tag);
            svc!(SYS_SEM_SIGNAL, sem, 0u32, 0u32);
            svc!(SYS_EVT_SET, 0u32, evt, 0u32);
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

extern "C" fn worker_a() -> ! {
    let p = unpack_r0!();
    worker("wrkA", p & 0xFFFF, (p >> 16) & 0xFF, p >> 24, 0x01)
}

extern "C" fn worker_b() -> ! {
    let p = unpack_r0!();
    worker("wrkB", p & 0xFFFF, (p >> 16) & 0xFF, p >> 24, 0x02)
}

// ---------------------------------------------------------------------------
// SysTick handler: drives the round-robin scheduler
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    let p = &raw mut KS;
    if let Some(ks) = unsafe { (*p).as_mut() } {
        if let Some(pid) = ks.advance_schedule_tick() {
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
            SCB::set_pendsv();
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point: create resources, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("blackboard_demo: start");

    unsafe {
        let mut k = Kernel::<DemoConfig>::new();

        let bb: u32 = k.blackboards.create().unwrap() as u32;
        k.semaphores.add(Semaphore::new(1, 1)).unwrap();
        let sem: u32 = 0; // first (and only) semaphore in the pool

        KERN = Some(k);
        kernel::svc::set_dispatch_hook(hook);

        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        for i in 0..NUM_PARTITIONS as u8 {
            sched.add(ScheduleEntry::new(i, 2)).unwrap();
        }
        sched.start();

        let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| {
            let b = 0x2000_0000 + (i as u32) * 0x2000;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        });
        KS = Some(KernelState::new(sched, &cfgs).unwrap());

        // Pack per-partition R0 values:
        //   config_main (partition 0): only needs blackboard ID
        //   worker_a    (partition 1): needs partition_id=1, sem, bb
        //   worker_b    (partition 2): needs partition_id=2, sem, bb
        let hints: [u32; NUM_PARTITIONS] = [
            pack_r0(0, sem, bb),
            pack_r0(1, sem, bb),
            pack_r0(2, sem, bb),
        ];
        let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [config_main, worker_a, worker_b];
        let (stk, sp) = (&raw mut STACKS, &raw mut PARTITION_SP);
        for (i, (ep, &hv)) in eps.iter().zip(hints.iter()).enumerate() {
            let ix = init_stack_frame(&mut (*stk)[i], *ep as *const () as u32, Some(hv)).unwrap();
            (*sp)[i] = (*stk)[i].as_ptr() as u32 + (ix as u32) * 4;
        }

        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(120_000 - 1);
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi()
    }
}
