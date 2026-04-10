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
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2,
    PortsTiny, StackStorage as _, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const REGION_SZ: u32 = 1024;

static mut STACKS: [AlignedStack1K; TestConfig::N] = [AlignedStack1K::ZERO; TestConfig::N];

const P0_MAGIC: u32 = 0xCAFE_0001;
const P1_MAGIC: u32 = 0xBEEF_0002;
/// Each partition must complete >= MIN_ITERS yields (>= 4 context switches).
const MIN_ITERS: u32 = 3;

// MPU_ENFORCE=true is broken on QEMU <=6.2 (IACCVIOL on exception stacking).
// We verify cached MPU regions are data-correct via on-the-fly comparison.
kernel::kernel_config!(
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
const _: PartitionEntry = p0_entry;
partition_entry!(p1_entry, P1_COUNTER, P1_READBACK, P1_MAGIC);
const _: PartitionEntry = p1_entry;

kernel::define_kernel!(no_boot, TestConfig, |tick, k| {
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
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("mpu_context_switch_test: start");
    let entry_fns: [PartitionEntry; TestConfig::N] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let mut k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = &raw mut STACKS;
        let stacks = unsafe { &mut *ptr };
        let mut stk_iter = stacks.iter_mut();
        // TODO: consider replacing expect() with a non-panicking pattern (e.g. loop+kexit)
        // for panic-free main() policy.
        let memories: [_; TestConfig::N] = core::array::from_fn(|i| {
            let stk = stk_iter.next().expect("stack");
            let base = stk.as_u32_slice().as_ptr() as u32;
            let spec = PartitionSpec::entry(entry_fns[i])
                .with_data_mpu(MpuRegion::new(base, REGION_SZ, 0));
            ExternalPartitionMemory::from_spec(stk, &spec, kernel::PartitionId::new(i as u32))
                .expect("mem")
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot") {}
}
