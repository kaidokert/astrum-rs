//! QEMU test: read-only buffer lend and revoke via SVC syscalls.
//!
//! Two-partition test: P0 allocates, writes known data, lends read-only
//! to P1.  P1 reads via read_volatile through the AP_RO_RO MPU window
//! and verifies the data matches.  P0 revokes and releases.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    buf_syscall,
    mpu_strategy::MpuStrategy,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    virtual_device::DeviceRegistry,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const P1: u8 = 1;
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Mirror dynamic strategy window to HARNESS_STRATEGY so PendSV
// programs the correct AP_RO_RO MPU region for P1.
kernel::define_unified_harness!(TestConfig, |_tick, k| {
    if MIRROR.load(Ordering::Acquire) == 1 {
        let rid = RID.load(Ordering::Relaxed) as u8;
        if let Some(d) = k.dynamic_strategy.slot(rid) {
            BUF_ADDR.store(d.base, Ordering::Release);
            let _ = HARNESS_STRATEGY.add_window(d.base, d.size, d.permissions, d.owner);
        }
        MIRROR.store(2, Ordering::Release);
    }
});

static RID: AtomicU32 = AtomicU32::new(0);
static BUF_ADDR: AtomicU32 = AtomicU32::new(0);
static MIRROR: AtomicU32 = AtomicU32::new(0);
// P1 stores 1 on mismatch, 2 on success.
static P1_RESULT: AtomicU32 = AtomicU32::new(0);

extern "C" fn p0_main() -> ! {
    let slot = buf_syscall::buf_alloc(true, 0).expect("alloc");
    buf_syscall::buf_write(slot, &MAGIC).expect("write");
    // Lend read-only to partition 1 (writable=false).
    let rid = buf_syscall::buf_lend(slot, P1, false).expect("lend");
    RID.store(rid as u32, Ordering::Release);
    MIRROR.store(1, Ordering::Release);
    while P1_RESULT.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    buf_syscall::buf_revoke(slot, P1).expect("revoke");
    buf_syscall::buf_release(slot).expect("release");
    if P1_RESULT.load(Ordering::Acquire) == 2 {
        hprintln!("buf_lend_ro_test: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("FAIL: P1 read mismatch");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

extern "C" fn p1_main() -> ! {
    loop {
        let addr = BUF_ADDR.load(Ordering::Acquire);
        if addr != 0 && MIRROR.load(Ordering::Acquire) == 2 {
            let mut buf = [0u8; 4];
            for (i, b) in buf.iter_mut().enumerate() {
                // SAFETY: addr is within the AP_RO_RO MPU window granted
                // by the SysTick mirror hook; offset i < 4 stays in range.
                *b = unsafe { core::ptr::read_volatile((addr as *const u8).add(i)) };
            }
            if buf == MAGIC {
                P1_RESULT.store(2, Ordering::Release);
            } else {
                P1_RESULT.store(1, Ordering::Release);
            }
            break;
        }
        cortex_m::asm::nop();
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals already taken");
    hprintln!("buf_lend_ro_test: start");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");
    let cfgs: [PartitionConfig; NP] = core::array::from_fn(|i| {
        PartitionConfig::sentinel(i as u8, (TestConfig::STACK_WORDS * 4) as u32)
    });
    let k = Kernel::<TestConfig>::new(sched, &cfgs, DeviceRegistry::new()).expect("kernel");
    store_kernel(k);
    let parts: [(extern "C" fn() -> !, u32); NP] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, p).expect("boot") {}
}
