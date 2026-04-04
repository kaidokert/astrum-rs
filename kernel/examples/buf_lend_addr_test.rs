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

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const PASS: u32 = 1;
const FAIL: u32 = 2;
static ADDR: AtomicU32 = AtomicU32::new(0);
static P1_OK: AtomicU32 = AtomicU32::new(0);
static RESULT: AtomicU32 = AtomicU32::new(0);
fn fail() -> ! {
    RESULT.store(FAIL, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}
kernel::define_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == PASS {
        hprintln!("buf_lend_addr_test: PASS");
        kernel::kexit!(success);
    } else if r == FAIL {
        hprintln!("buf_lend_addr_test: FAIL");
        kernel::kexit!(failure);
    } else if tick >= 30 {
        hprintln!("buf_lend_addr_test: timeout");
        kernel::kexit!(failure);
    }
});
const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let slot = match buf_syscall::buf_alloc(true, 0) {
        Ok(s) => s,
        Err(_) => fail(),
    };
    let data = [0xAAu8; 4];
    if buf_syscall::buf_write(slot, &data).is_err() {
        fail();
    }
    let slot_id = plib::BufferSlotId::new(slot as u8);
    let (base_ptr, _region_id) = match plib::sys_buf_lend(slot_id, 1, true) {
        Ok(pair) => pair,
        Err(_) => fail(),
    };
    let base_addr = base_ptr as u32;
    // Verify the returned address is in SRAM (>= 0x2000_0000).
    // P1's subsequent read from this address (checking for 0xAA) serves as
    // the definitive verification that base_addr matches the actual slot base.
    if base_addr < 0x2000_0000 {
        fail();
    }
    ADDR.store(base_addr, Ordering::Release);
    while P1_OK.load(Ordering::Acquire) != PASS {
        cortex_m::asm::nop();
    }
    RESULT.store(PASS, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    while ADDR.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    let a = ADDR.load(Ordering::Acquire);
    // SAFETY: `a` is the base address returned by SYS_BUF_LEND, which
    // dynamically granted P1 read access to the buffer via an MPU region
    // update.  The kernel guarantees the address is valid and aligned.
    let v = unsafe { core::ptr::read_volatile(a as *const u8) };
    if v == 0xAA {
        P1_OK.store(PASS, Ordering::Release);
    } else {
        P1_OK.store(FAIL, Ordering::Release);
        RESULT.store(FAIL, Ordering::Release);
    }
    loop {
        cortex_m::asm::wfi();
    }
}
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("buf_lend_addr_test: start");
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
