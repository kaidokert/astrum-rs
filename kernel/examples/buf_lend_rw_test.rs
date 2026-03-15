//! QEMU test: writable buffer lend and revoke via SVC syscalls.
//!
//! Two-partition test exercising SYS_BUF_ALLOC → SYS_BUF_WRITE →
//! SYS_BUF_LEND (writable) → write_volatile by P1 → SYS_BUF_REVOKE →
//! SYS_BUF_READ.  Verifies that P1's modifications persist after revoke.

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
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const P1: u8 = 1; // Target partition index.
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const MODIFIED: [u8; 4] = [0xCA, 0xFE, 0xBA, 0xBE];

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// The SVC handler stores MPU windows in `kernel.dynamic_strategy`, but PendSV
// reads from the harness-generated HARNESS_STRATEGY.  The SysTick hook below
// mirrors the window so PendSV programs the correct MPU region for P1.
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
static P1_DONE: AtomicU32 = AtomicU32::new(0);

extern "C" fn p0_main() -> ! {
    // 1. Allocate a writable buffer slot.
    let slot = buf_syscall::buf_alloc(true, 0).expect("alloc");
    // 2. Write magic data into the slot.
    buf_syscall::buf_write(slot, &MAGIC).expect("write");
    // 3. Lend writable to partition 1.
    let rid = buf_syscall::buf_lend(slot, P1, true).expect("lend");
    RID.store(rid as u32, Ordering::Release);
    MIRROR.store(1, Ordering::Release);
    // 4. Wait for P1 to modify the buffer.
    while P1_DONE.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    // 5. Revoke the lending.
    buf_syscall::buf_revoke(slot, P1).expect("revoke");
    // 6. Read buffer and verify P1's modifications persisted.
    let mut dst = [0u8; 4];
    let n = buf_syscall::buf_read(slot, &mut dst).expect("read");
    if n == 4 && dst == MODIFIED {
        hprintln!("buf_lend_rw_test: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("FAIL: expected {:?} got {:?}", MODIFIED, &dst[..n]);
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
            // Write modified data through the AP_FULL_ACCESS MPU window.
            for (i, &b) in MODIFIED.iter().enumerate() {
                // SAFETY: `addr` is the base of a kernel buffer slot whose
                // MPU region was granted to this partition with AP_FULL_ACCESS
                // by the SysTick mirror hook.  The offset `i` stays within the
                // 4-byte MODIFIED array so the pointer is within the granted
                // region.  No other partition writes to this buffer while P1
                // holds the writable grant.
                unsafe { core::ptr::write_volatile((addr as *mut u8).add(i), b) };
            }
            P1_DONE.store(1, Ordering::Release);
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
    hprintln!("buf_lend_rw_test: start");
    // Schedule: P0(3) → sys(1) → P1(3) → sys(1) → repeat.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 3))
        .expect("add P0 schedule entry");
    sched.add_system_window(1).expect("sys0");
    sched
        .add(ScheduleEntry::new(1, 3))
        .expect("add P1 schedule entry");
    sched.add_system_window(1).expect("sys1");
    let k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let stacks = unsafe {
            &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; TestConfig::STACK_WORDS]; NP]>()
        };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(s0, 0, MpuRegion::new(0, 0, 0), 0).expect("mem 0"),
            ExternalPartitionMemory::new(s1, 0, MpuRegion::new(0, 0, 0), 1).expect("mem 1"),
        ];
        Kernel::<TestConfig>::new_external(sched, &memories).expect("kernel")
    };
    store_kernel(k);
    let parts: [(extern "C" fn() -> !, u32); NP] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, p).expect("boot") {}
}
