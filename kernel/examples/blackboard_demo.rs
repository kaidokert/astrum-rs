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
#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    kernel::KernelState,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    semaphore::Semaphore,
    svc,
    svc::Kernel,
    syscall::{
        SYS_BB_DISPLAY, SYS_BB_READ, SYS_EVT_SET, SYS_EVT_WAIT, SYS_SEM_SIGNAL, SYS_SEM_WAIT,
        SYS_YIELD,
    },
    unpack_r0,
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config (tuned to this example's resource needs)
// ---------------------------------------------------------------------------
const MAX_SCHEDULE_ENTRIES: usize = 4;
const NUM_PARTITIONS: usize = 3;
const STACK_WORDS: usize = 256;

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
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
}

// ---------------------------------------------------------------------------
// R0 packing helpers
// ---------------------------------------------------------------------------

/// Pack partition ID, semaphore ID, and blackboard ID into a single u32.
const fn pack_r0(partition_id: u32, sem: u32, bb: u32) -> u32 {
    (partition_id << 24) | (sem << 16) | bb
}

kernel::define_harness!(
    DemoConfig,
    NUM_PARTITIONS,
    MAX_SCHEDULE_ENTRIES,
    STACK_WORDS
);

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
// Entry point: create resources, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("blackboard_demo: start");

    let (bb, sem);
    // SAFETY: accessing static-mut KS; single-core, interrupts not yet
    // enabled so there is no data race.
    unsafe {
        // TODO: register devices in the registry at init time (backlog item 195).
        #[cfg(feature = "dynamic-mpu")]
        let mut k = Kernel::<DemoConfig>::new(kernel::virtual_device::DeviceRegistry::new());
        #[cfg(not(feature = "dynamic-mpu"))]
        let mut k = Kernel::<DemoConfig>::new();

        bb = k.blackboards.create().unwrap() as u32;
        k.semaphores.add(Semaphore::new(1, 1)).unwrap();
        sem = 0u32; // first (and only) semaphore in the pool

        store_kernel(k);

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
        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

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
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        core::array::from_fn(|i| (eps[i], hints[i]));

    boot(&parts, &mut p)
}
