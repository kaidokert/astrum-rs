//! QEMU test: buffer syscall error-path validation.
//!
//! Single-partition driver (P0) exercises four error scenarios:
//! 1. `buf_release` while lent → error (cannot release while lent).
//! 2. `buf_lend` same slot again → error (already lent).
//! 3. `buf_revoke` then `buf_release` → success.
//! 4. `buf_read` on released slot → error (invalid resource).
//!
//! P1 idles — it exists only so the lend target is valid.

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
    svc::SvcError,
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const PASS: u32 = 1;
const FAIL: u32 = 2;
static RESULT: AtomicU32 = AtomicU32::new(0);

fn fail(msg: &str) -> ! {
    hprintln!("FAIL: {}", msg);
    RESULT.store(FAIL, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

kernel::define_kernel!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == PASS {
        hprintln!("buf_error_test: PASS");
        kernel::kexit!(success);
    } else if r == FAIL {
        hprintln!("buf_error_test: FAIL");
        kernel::kexit!(failure);
    } else if tick >= 60 {
        hprintln!("buf_error_test: timeout");
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Setup: allocate and lend to P1.
    let slot = match buf_syscall::buf_alloc(true, 0) {
        Ok(s) => s,
        Err(_) => fail("alloc"),
    };
    if buf_syscall::buf_lend(slot, 1, true).is_err() {
        fail("initial lend");
    }

    // 1. Release while lent → must fail with OperationFailed.
    match buf_syscall::buf_release(slot) {
        Err(SvcError::OperationFailed) => {}
        _ => fail("release-while-lent expected OperationFailed"),
    }

    // 2. Double lend → must fail with OperationFailed.
    match buf_syscall::buf_lend(slot, 1, true) {
        Err(SvcError::OperationFailed) => {}
        _ => fail("double-lend expected OperationFailed"),
    }

    // 3. Revoke then release → must succeed.
    if buf_syscall::buf_revoke(slot, 1).is_err() {
        fail("revoke after lend");
    }
    if buf_syscall::buf_release(slot).is_err() {
        fail("release after revoke");
    }

    // 4. Read after release → must fail with InvalidResource.
    let mut tmp = [0u8; 4];
    match buf_syscall::buf_read(slot, &mut tmp) {
        Err(SvcError::InvalidResource) => {}
        _ => fail("read-after-release expected InvalidResource"),
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
    hprintln!("buf_error_test: start");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).ok();
    sched.add_system_window(1).ok();
    sched.add(ScheduleEntry::new(1, 3)).ok();
    sched.add_system_window(1).ok();
    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("kernel");
    match boot(p).expect("boot") {}
}
