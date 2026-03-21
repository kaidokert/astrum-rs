//! QEMU test: deadline auto-revoke via SysTick.
//!
//! P0 allocates a writable buffer, writes data, lends to P1 with a short
//! deadline (2 ticks).  After enough SysTick ticks, `revoke_expired`
//! auto-revokes the lend.  P0 verifies by reading its data back and
//! releasing the buffer (release would fail if still lent).

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use kernel::{
    buf_syscall,
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const DEADLINE: u32 = 2;
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

const PASS: u32 = 1;
const FAIL: u32 = 2;
static RESULT: AtomicU32 = AtomicU32::new(0);
static TICKS: AtomicU32 = AtomicU32::new(0);

fn fail() -> ! {
    RESULT.store(FAIL, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    TICKS.store(tick, Ordering::Release);
    let r = RESULT.load(Ordering::Acquire);
    if r == PASS {
        hprintln!("buf_deadline_test: PASS");
        kernel::kexit!(success);
    } else if r == FAIL {
        hprintln!("buf_deadline_test: FAIL");
        kernel::kexit!(failure);
    } else if tick >= 60 {
        hprintln!("buf_deadline_test: timeout");
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // 1. Allocate a writable buffer slot (second arg is max_ticks, not size; 0 = no deadline).
    let slot = match buf_syscall::buf_alloc(true, 0) {
        Ok(s) => s,
        Err(_) => fail(),
    };
    // 2. Write magic data.
    if buf_syscall::buf_write(slot, &MAGIC).is_err() {
        fail();
    }
    // 3. Lend to P1 with short deadline.
    if buf_syscall::buf_lend_with_deadline(slot, 1, true, DEADLINE).is_err() {
        fail();
    }
    let t0 = TICKS.load(Ordering::Acquire);
    // 4. Spin until enough ticks for deadline expiry + system windows.
    while TICKS.load(Ordering::Acquire) < t0 + DEADLINE + 16 {
        cortex_m::asm::wfi();
    }
    // 5. Read buffer — data should be intact (P1 never modified it).
    let mut dst = [0u8; 4];
    match buf_syscall::buf_read(slot, &mut dst) {
        Ok(n) if n == MAGIC.len() && dst == MAGIC => {}
        _ => fail(),
    }
    // 6. Release — succeeds only if auto-revoke cleared lent_to.
    if buf_syscall::buf_release(slot).is_err() {
        fail();
    }
    RESULT.store(PASS, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("buf_deadline_test: start");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).ok();
    sched.add_system_window(1).ok();
    sched.add(ScheduleEntry::new(1, 3)).ok();
    sched.add_system_window(1).ok();
    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_main, 0),
        PartitionSpec::new(p1_main, 0),
    ];
    init_kernel(sched, &parts).expect("kernel");
    match boot(p).expect("boot") {}
}
