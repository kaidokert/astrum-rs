//! QEMU test: verify cached MPU regions match on-the-fly computation across context switches.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu::AP_FULL_ACCESS;
use kernel::{
    boot, mpu,
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, StackStorage as _, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

kernel::define_unified_harness!(no_boot, TestConfig, |tick, k| {
    if tick > 2 {
        let pid = k.current_partition() as usize;
        if let Some(pcb) = k.partitions().get(pid) {
            // Verify base cache integrity: R0 background region must match
            // the expected deny-all pattern (4 GiB no-access, XN).
            let bg_rasr = mpu::build_rasr(31, mpu::AP_NO_ACCESS, true, (false, false, false));
            if pcb.cached_base_regions()[0].1 != bg_rasr {
                hprintln!("FAIL: base cache mismatch p{}", pid);
                kernel::kexit!(failure);
            }
            // Verify periph cache integrity: R4/R5 must target slots 4 and 5.
            let cp = pcb.cached_periph_regions();
            if cp[0].0 & 0xF != 4 || cp[1].0 & 0xF != 5 {
                hprintln!("FAIL: periph cache mismatch p{}", pid);
                kernel::kexit!(failure);
            }
        }
    }
    let p0 = P0_COUNTER.load(Ordering::Acquire);
    let p1 = P1_COUNTER.load(Ordering::Acquire);
    if tick >= 32 && p0 >= 8 && p1 >= 8 {
        hprintln!(
            "mpu_cached_test: PASS (p0={}, p1={}, ticks={})",
            p0,
            p1,
            tick
        );
        kernel::kexit!(success);
    }
    if tick >= 100 {
        hprintln!("mpu_cached_test: FAIL timeout (p0={}, p1={})", p0, p1);
        kernel::kexit!(failure);
    }
});

/// Generate a partition entry function that:
/// 1. Reads a byte from its own code region (verifying MPU code-read access),
/// 2. Performs a data computation using `black_box` to prevent elision,
/// 3. Yields via SVC and checks the return code.
macro_rules! partition_entry {
    ($name:ident, $counter:expr, $magic:expr) => {
        extern "C" fn $name() -> ! {
            // Verify MPU allows data reads from our own code region.
            // SAFETY: `$name` resides in .text which is mapped as RX by the MPU;
            // its address is valid, aligned, and readable. read_volatile prevents
            // the compiler from eliding this data-read from code space.
            let code_byte = unsafe { core::ptr::read_volatile($name as *const () as *const u8) };
            core::hint::black_box(code_byte);

            let mut data: u32 = 0;
            loop {
                data = core::hint::black_box(data.wrapping_add($magic));
                $counter.fetch_add(1, Ordering::Release);
                plib::sys_yield().expect("yield failed");
            }
        }
    };
}

partition_entry!(p0_entry, P0_COUNTER, 0xCAFE_0000u32);
partition_entry!(p1_entry, P1_COUNTER, 0xBEEF_0000u32);

const REGION_SZ: u32 = 1024;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_cached_test: start");
    let entry_fns: [extern "C" fn() -> !; TestConfig::N] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let k = {
        let stacks = kernel::partition_stacks!(TestConfig, TestConfig::N);
        let [ref mut s0, ref mut s1] = *stacks;
        let memories =
            [
                {
                    // UART0 peripheral region for p0: exercises the non-trivial peripheral cache path.
                    let base = s0.as_u32_slice().as_ptr() as u32;
                    ExternalPartitionMemory::from_aligned_stack(
                        s0,
                        entry_fns[0] as usize as u32,
                        MpuRegion::new(base, REGION_SZ, 0),
                        0,
                    )
                    .expect("mem 0")
                    .with_peripheral_regions(&[MpuRegion::new(0x4000_C000, 4096, AP_FULL_ACCESS)])
                },
                {
                    let base = s1.as_u32_slice().as_ptr() as u32;
                    ExternalPartitionMemory::from_aligned_stack(
                        s1,
                        entry_fns[1] as usize as u32,
                        MpuRegion::new(base, REGION_SZ, 0),
                        1,
                    )
                    .expect("mem 1")
                },
            ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot") {}
}
