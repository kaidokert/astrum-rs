//! QEMU test: cached MPU in dynamic-mode context switch.
//!
//! Two partitions with distinct MPU data regions context-switch via the
//! dynamic-mode PendSV handler. The SysTick hook reads back MPU RBAR/RASR
//! for R0-R3 and asserts they match the incoming partition's cached values.
//! Verifies R2 (data region) bases differ between partitions.
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
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, StackStorage as _,
    SyncMinimal,
};

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

// Nop-loop partitions: no memory access beyond the stack, safe under MPU.
extern "C" fn p0_entry() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
extern "C" fn p1_entry() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

/// Read (RBAR, RASR) for MPU region `n` from hardware.
///
/// # Safety
/// Caller must have exclusive access to the MPU peripheral.  The RNR write
/// selects the region whose RBAR/RASR are then read; if another context
/// could change RNR between the write and reads, the wrong region would be
/// returned.  Exclusive access (e.g. running inside `interrupt::free` or a
/// single-owner reference) prevents this.
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    mpu.rnr.write(n);
    (mpu.rbar.read(), mpu.rasr.read())
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    if tick > 2 {
        let pid = k.current_partition as usize;
        if let Some(pcb) = k.partitions().get(pid) {
            let cached = pcb.cached_base_regions();
            // SAFETY: SysTick hook runs inside `interrupt::free`, so we have
            // exclusive access to all peripherals; `steal()` is the only way
            // to obtain a peripheral handle in handler mode.
            let p = unsafe { cortex_m::Peripherals::steal() };
            for (r, &(c_rbar, c_rasr)) in cached.iter().enumerate() {
                // SAFETY: interrupts are masked (`interrupt::free`), so we
                // have exclusive access to the MPU peripheral — no other
                // context can change RNR between the write and reads.
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
    if tick >= 32 {
        // Verify distinct R2 (data region) bases between partitions.
        // Use if-let chains instead of unwrap()/panicking index — this runs
        // in handler mode where panics are unrecoverable.
        if let (Some(p0), Some(p1)) = (k.partitions().get(0), k.partitions().get(1)) {
            if let (Some(&(r2_0_raw, _)), Some(&(r2_1_raw, _))) = (
                p0.cached_base_regions().get(2),
                p1.cached_base_regions().get(2),
            ) {
                let r2_0 = r2_0_raw & !0x1F;
                let r2_1 = r2_1_raw & !0x1F;
                if r2_0 == r2_1 {
                    hprintln!("FAIL: R2 base identical {:#010x}", r2_0);
                    kernel::kexit!(failure);
                }
                hprintln!(
                    "mpu_cached_dynamic_test: PASS (R2 {:#010x} != {:#010x})",
                    r2_0,
                    r2_1
                );
                kernel::kexit!(success);
            }
        }
        hprintln!("FAIL: partition or region lookup failed");
        kernel::kexit!(failure);
    }
    if tick >= 100 {
        hprintln!("FAIL: timeout");
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_cached_dynamic_test: start");
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    sched.add_system_window(1).expect("syswin 1");
    let k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i] as usize as u32,
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
