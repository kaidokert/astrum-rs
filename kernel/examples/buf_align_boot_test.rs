//! QEMU boot test: BufferPool alignment verification.
//!
//! Boots a kernel with a 2-slot BufferPool (SIZE=32) and verifies alignment
//! by allocating a buffer and checking its base address is 32-byte aligned.
//! The kernel's `init_kernel_struct` also calls `buffers.assert_aligned()` at
//! boot; if that false-positives on QEMU's naturally-aligned allocations, the
//! kernel panics before reaching the SysTick hook.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::{
    buf_syscall, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny,
    SyncMinimal,
};

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        buffer_pool_regions = 2;
        buffer_zone_size = 32;
    }
);

const PASS: u32 = 1;
const FAIL: u32 = 2;
static RESULT: AtomicU32 = AtomicU32::new(0);

fn fail() -> ! {
    RESULT.store(FAIL, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

/// Hard timeout — should never be reached.
const TIMEOUT_TICK: u32 = 30;

kernel::define_harness!(TestConfig, |tick, _k| {
    let r = RESULT.load(Ordering::Acquire);
    if r == PASS {
        hprintln!("buf_align_boot_test: PASS");
        kernel::kexit!(success);
    } else if r == FAIL {
        hprintln!("buf_align_boot_test: FAIL");
        kernel::kexit!(failure);
    } else if tick >= TIMEOUT_TICK {
        hprintln!("buf_align_boot_test: FAIL (timeout)");
        kernel::kexit!(failure);
    }
});

/// P0 allocates a buffer, retrieves its address via lend, and verifies
/// 32-byte alignment.
const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Allocate a writable buffer slot.
    let slot = match buf_syscall::buf_alloc(true, 0) {
        Ok(s) => s,
        Err(_) => fail(),
    };

    // Write test data so the buffer is exercised, not just allocated.
    let data = [0xABu8; 4];
    if buf_syscall::buf_write(slot, &data).is_err() {
        fail();
    }

    // Lend to P1 to obtain the base address via buf_lend_with_addr.
    let slot_id = plib::BufferSlotId::new(slot as u8);
    let (base_ptr, _region_id) = match plib::sys_buf_lend(slot_id, 1, true) {
        Ok(pair) => pair,
        Err(_) => fail(),
    };
    let base_addr = base_ptr as usize;

    // Verify the address is in SRAM.
    if base_addr < 0x2000_0000 {
        fail();
    }

    // Verify 32-byte alignment (SIZE=32).
    if !base_addr.is_multiple_of(32) {
        fail();
    }

    RESULT.store(PASS, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    // P1 is the lend target; no active work needed.
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("buf_align_boot_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).ok();
    sched.add_system_window(1).ok();
    sched.add(ScheduleEntry::new(1, 2)).ok();
    sched.add_system_window(1).ok();

    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_main as PartitionEntry, 0),
        PartitionSpec::new(p1_main as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("kernel init");
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
