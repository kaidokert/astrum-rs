//! QEMU test: buffer ownership transfer and SYS_BUF_READ.
//!
//! Two-partition test: P0 allocates a buffer (BorrowedWrite), writes magic
//! data, then transfers ownership to P1 via SYS_BUF_TRANSFER.  After
//! transfer P0 no longer owns the slot.  P1 uses SYS_BUF_READ to copy
//! buffer contents into partition-local memory (stack buffer), verifies the
//! magic pattern matches, then releases via SYS_BUF_RELEASE.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    buf_syscall,
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const P1: u8 = 1;
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

kernel::define_unified_harness!(TestConfig, |_tick, _k| {});

static SLOT: AtomicU32 = AtomicU32::new(u32::MAX);
static TRANSFERRED: AtomicU32 = AtomicU32::new(0);
static P1_RESULT: AtomicU32 = AtomicU32::new(0); // 0=pending, 1=pass, 2=fail

fn fail_p0(msg: &str) -> ! {
    hprintln!("FAIL(P0): {}", msg);
    debug::exit(debug::EXIT_FAILURE);
    loop {
        cortex_m::asm::wfi();
    }
}

fn fail_p1() -> ! {
    P1_RESULT.store(2, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let slot = buf_syscall::buf_alloc(true, 0).unwrap_or_else(|_| fail_p0("alloc"));
    let n = buf_syscall::buf_write(slot, &MAGIC).unwrap_or_else(|_| fail_p0("write"));
    if n != MAGIC.len() {
        fail_p0("write len mismatch");
    }
    buf_syscall::buf_transfer(slot, P1).unwrap_or_else(|_| fail_p0("transfer"));
    // Verify sender (P0) no longer owns the slot after transfer.
    let mut tmp = [0u8; 4];
    if buf_syscall::buf_read(slot, &mut tmp).is_ok() {
        fail_p0("post-transfer read should fail");
    }
    SLOT.store(slot as u32, Ordering::Release);
    TRANSFERRED.store(1, Ordering::Release);
    while P1_RESULT.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    if P1_RESULT.load(Ordering::Acquire) == 1 {
        hprintln!("buf_transfer_test: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("FAIL: P1 read/release verification failed");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    while TRANSFERRED.load(Ordering::Acquire) != 1 {
        cortex_m::asm::nop();
    }
    let slot = SLOT.load(Ordering::Relaxed) as u8;
    // Read buffer contents via SYS_BUF_READ into stack-local memory.
    let mut dst = [0u8; 4];
    let n = buf_syscall::buf_read(slot, &mut dst).unwrap_or_else(|_| fail_p1());
    if n != MAGIC.len() || dst != MAGIC {
        fail_p1();
    }
    // Release the buffer slot.
    buf_syscall::buf_release(slot).unwrap_or_else(|_| fail_p1());
    // Verify the slot is invalid after release.
    if buf_syscall::buf_read(slot, &mut dst).is_ok() {
        fail_p1();
    }
    P1_RESULT.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("buf_transfer_test: start");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("P1");
    sched.add_system_window(1).expect("sys1");
    let k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = (&raw mut __PARTITION_STACKS).cast::<[[u32; TestConfig::STACK_WORDS]; NP]>();
        let stacks = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(
                s0,
                EntryAddr::from_fn(p0_main),
                MpuRegion::new(0, 0, 0),
                0,
            )
            .expect("mem 0"),
            ExternalPartitionMemory::new(
                s1,
                EntryAddr::from_fn(p1_main),
                MpuRegion::new(0, 0, 0),
                1,
            )
            .expect("mem 1"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(k);
    match boot(p).expect("boot") {}
}
