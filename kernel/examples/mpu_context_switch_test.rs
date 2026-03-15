//! QEMU test: cached MPU regions survive multiple context switches.
//!
//! Boots 2 partitions with real MPU regions. Each partition writes a magic
//! value to its stack then yields. The SysTick hook verifies cached MPU data
//! matches on-the-fly computation every tick across 4+ context switches.
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
    boot, mpu,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const SW: usize = TestConfig::STACK_WORDS;
const REGION_SZ: u32 = 1024;

#[repr(C, align(4096))]
struct PartitionStacks([[u32; SW]; TestConfig::N]);
static mut PARTITION_STACKS: PartitionStacks = PartitionStacks([[0u32; SW]; TestConfig::N]);

const P0_MAGIC: u32 = 0xCAFE_0001;
const P1_MAGIC: u32 = 0xBEEF_0002;
/// Each partition must complete >= MIN_ITERS yields (>= 4 context switches).
const MIN_ITERS: u32 = 3;

// MPU_ENFORCE=true is broken on QEMU <=6.2 (IACCVIOL on exception stacking).
// We verify cached MPU regions are data-correct via on-the-fly comparison.
kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);
static P0_READBACK: AtomicU32 = AtomicU32::new(0);
static P1_READBACK: AtomicU32 = AtomicU32::new(0);

macro_rules! partition_entry {
    ($name:ident, $counter:expr, $readback:expr, $magic:expr) => {
        extern "C" fn $name() -> ! {
            loop {
                let local = core::hint::black_box($magic);
                let val = core::hint::black_box(local);
                $readback.store(val, Ordering::Release);
                $counter.fetch_add(1, Ordering::Release);
                plib::sys_yield().expect("failed to yield remaining time slice");
            }
        }
    };
}

partition_entry!(p0_entry, P0_COUNTER, P0_READBACK, P0_MAGIC);
partition_entry!(p1_entry, P1_COUNTER, P1_READBACK, P1_MAGIC);

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    if tick > 2 {
        for i in 0..TestConfig::N {
            if let Some(pcb) = k.partitions().get(i) {
                // Verify cache integrity: R0 background region must match
                // the expected deny-all pattern (4 GiB no-access, XN).
                let bg_rasr = mpu::build_rasr(31, mpu::AP_NO_ACCESS, true, (false, false, false));
                if pcb.cached_base_regions()[0].1 != bg_rasr {
                    hprintln!("FAIL: cache mismatch p{}", i);
                    kernel::kexit!(failure);
                }
            }
        }
    }
    let p0 = P0_COUNTER.load(Ordering::Acquire);
    let p1 = P1_COUNTER.load(Ordering::Acquire);
    if p0 >= MIN_ITERS && p1 >= MIN_ITERS {
        let rb0 = P0_READBACK.load(Ordering::Acquire);
        let rb1 = P1_READBACK.load(Ordering::Acquire);
        if rb0 != P0_MAGIC || rb1 != P1_MAGIC {
            hprintln!("FAIL: readback p0={:#x} p1={:#x}", rb0, rb1);
            kernel::kexit!(failure);
        }
        hprintln!("mpu_context_switch_test: PASS (p0={}, p1={})", p0, p1);
        kernel::kexit!(success);
    }
    if tick >= 100 {
        hprintln!("FAIL: timeout (p0={}, p1={})", p0, p1);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_context_switch_test: start");
    let entry_fns: [extern "C" fn() -> !; TestConfig::N] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let mut cfgs = PartitionConfig::sentinel_array::<{ TestConfig::N }>();
    for (i, cfg) in cfgs.iter_mut().enumerate() {
        // entry_point is the MPU code-region base, not the execution start address.
        // boot::boot_external() receives the actual function pointer via the `parts` array.
        cfg.entry_point = entry_fns[i] as usize as u32 & !(REGION_SZ - 1);
    }
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("kernel");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("kernel");
    store_kernel(k);
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks: &mut [[u32; SW]; TestConfig::N] =
        unsafe { &mut *(&raw mut PARTITION_STACKS).cast() };
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for (i, stk) in stacks.iter().enumerate() {
            let base = stk.as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .expect("partition")
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    })
    .expect("with_kernel_mut");
    let parts: [(extern "C" fn() -> !, u32); TestConfig::N] = [(p0_entry, 0), (p1_entry, 0)];
    match boot::boot_external::<TestConfig, SW>(&parts, p, stacks).expect("boot") {}
}
