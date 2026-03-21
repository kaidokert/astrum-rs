//! QEMU test: MPU registers match precomputed values after context switch.
//!
//! With MPU_ENFORCE=true, PendSV programs RBAR/RASR from each partition's
//! cached_base_regions() and cached_periph_regions() on every context switch.
//! The SysTick hook reads back all 6 (RBAR, RASR) pairs (R0-R5) from hardware
//! and asserts they match the currently-active partition's precomputed cache.
//! R4-R5 are peripheral regions; these test partitions don't configure
//! peripherals, so R4-R5 should match the disabled-region fallback.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    boot,
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny,
    StackStorage as _, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const REGION_SZ: u32 = 1024;

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        mpu_enforce = true;
    }
);

static mut STACKS: [AlignedStack1K; NP] = [AlignedStack1K::ZERO; NP];

// Shared partition body: yield in a loop, asserting success each time.
// No memory access beyond the stack (covered by the MPU data region)
// and registers (SVC ABI).
fn partition_loop() -> ! {
    loop {
        plib::sys_yield().expect("yield failed");
    }
}

extern "C" fn p0_entry() -> ! {
    partition_loop()
}
extern "C" fn p1_entry() -> ! {
    partition_loop()
}

/// Read the (RBAR, RASR) pair for MPU region `n` from hardware.
///
/// # Safety
///
/// The caller must ensure exclusive access to the MPU peripheral
/// (e.g. by running in an exception handler that cannot be preempted
/// by another MPU writer).
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    // SAFETY: Caller guarantees exclusive MPU access.  Writing RNR to
    // select a region then reading RBAR/RASR is a standard MPU query
    // sequence (ARMv7-M ARM B3.5.5).
    mpu.rnr.write(n);
    (mpu.rbar.read(), mpu.rasr.read())
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    // Track which partitions have been observed running (privileged context).
    static SEEN: [core::sync::atomic::AtomicBool; 2] = [
        core::sync::atomic::AtomicBool::new(false),
        core::sync::atomic::AtomicBool::new(false),
    ];

    // Verify that PendSV programmed the correct MPU registers for the
    // currently-active partition.  SysTick preempts PendSV, so the MPU
    // hardware state here reflects the last completed context switch.
    if tick > 2 {
        let pid = k.current_partition as usize;
        if pid < 2 {
            SEEN[pid].store(true, core::sync::atomic::Ordering::Release);
        }
        if let Some(pcb) = k.partitions().get(pid) {
            let base = pcb.cached_base_regions();
            let periph = pcb.cached_periph_regions();
            // SAFETY: SysTick is the highest-priority exception in use;
            // PendSV (lower priority) cannot preempt, so we have exclusive
            // access to the MPU peripheral for readback.
            let p = unsafe { cortex_m::Peripherals::steal() };
            for (r, &(c_rbar, c_rasr)) in base.iter().chain(periph.iter()).enumerate() {
                // SAFETY: Exclusive MPU access guaranteed; see above.
                let (hw_rbar, hw_rasr) = unsafe { read_mpu_region(&p.MPU, r as u32) };
                // RBAR VALID bit (bit 4) is write-only; mask for comparison.
                if (c_rbar & !0x10) != hw_rbar || c_rasr != hw_rasr {
                    hprintln!(
                        "FAIL: R{} p{} RBAR exp={:#010x} got={:#010x} RASR exp={:#010x} got={:#010x}",
                        r, pid, c_rbar & !0x10, hw_rbar, c_rasr, hw_rasr
                    );
                    kernel::kexit!(failure);
                }
            }
        }
    }
    let both = SEEN[0].load(core::sync::atomic::Ordering::Acquire)
        && SEEN[1].load(core::sync::atomic::Ordering::Acquire);
    if tick >= 32 && both {
        hprintln!("mpu_precompute_test: PASS R0-R5");
        kernel::kexit!(success);
    }
    if tick >= 100 {
        hprintln!("FAIL: timeout (both_seen={})", both);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_precompute_test: start");
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = &raw mut STACKS;
        let stacks = unsafe { &mut *ptr };
        let mut stk_iter = stacks.iter_mut();
        // TODO: consider replacing expect() with a non-panicking pattern (e.g. loop+kexit)
        // for panic-free main() policy.
        let memories: [_; NP] = core::array::from_fn(|i| {
            let stk = stk_iter.next().expect("stack");
            let base = stk.as_u32_slice().as_ptr() as u32;
            ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(base, REGION_SZ, 0),
                i as u8,
            )
            .expect("mem")
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot") {}
}
