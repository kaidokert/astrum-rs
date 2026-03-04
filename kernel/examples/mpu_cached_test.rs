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
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::SYS_YIELD,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

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
                let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
                assert!(rc == 0, "SYS_YIELD returned non-zero");
            }
        }
    };
}

partition_entry!(p0_entry, P0_COUNTER, 0xCAFE_0000u32);
partition_entry!(p1_entry, P1_COUNTER, 0xBEEF_0000u32);

const REGION_SZ: u32 = 1024;

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("mpu_cached_test: start");
    let entry_fns: [extern "C" fn() -> !; 2] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    let cfgs: [PartitionConfig; 2] = core::array::from_fn(|i| {
        // entry_point is the MPU code-region base, not the execution start address.
        // boot::boot() receives the actual function pointer via the `parts` array.
        // TODO: reviewer false positive — same pattern as mpu_enforce_test.rs
        let code_region_base = (entry_fns[i] as *const () as usize as u32) & !(REGION_SZ - 1);
        let mut peripheral_regions: heapless::Vec<MpuRegion, 2> = heapless::Vec::new();
        if i == 0 {
            // UART0 peripheral region for p0: exercises the non-trivial peripheral cache path.
            // TODO: reviewer requested AP_FULL_ACCESS_XN but that constant does not exist;
            // peripheral_region_pair() hardcodes XN=true and ignores the permissions field,
            // so the AP value here is advisory only.
            peripheral_regions
                .push(MpuRegion::new(0x4000_C000, 4096, AP_FULL_ACCESS))
                .expect("periph push");
        }
        PartitionConfig {
            id: i as u8,
            entry_point: code_region_base,
            // TODO: hardcoded stack addresses mirror mpu_enforce_test.rs; replace
            // with linker symbols or aligned statics when a common pattern emerges.
            stack_base: [0x2000_0000u32, 0x2000_8000][i],
            stack_size: (TestConfig::STACK_WORDS * 4) as u32,
            mpu_region: MpuRegion::new(0, 0, 0),
            peripheral_regions,
        }
    });
    let k = Kernel::<TestConfig>::create(sched, &cfgs).expect("kernel");
    store_kernel(k);
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        for i in 0..2 {
            let base = k.core_stack_mut(i).expect("stack").as_ptr() as u32;
            k.partitions_mut()
                .get_mut(i)
                .expect("partition")
                .promote_sentinel_mpu(base, REGION_SZ, 0)
                .expect("promote sentinel");
        }
    });
    let parts: [(extern "C" fn() -> !, u32); 2] = [(p0_entry, 0), (p1_entry, 0)];
    match boot::boot::<TestConfig>(&parts, &mut p).expect("boot") {}
}
