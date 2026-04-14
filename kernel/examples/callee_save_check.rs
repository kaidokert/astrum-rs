//! QEMU regression: callee-saved register (r4-r11) preservation across PendSV.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::ScheduleTable, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2,
    PortsTiny, SyncMinimal,
};

kernel::kernel_config!(Cfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);
static CHECKS: AtomicU32 = AtomicU32::new(0);
static ERRORS: AtomicU32 = AtomicU32::new(0);
kernel::define_kernel!(Cfg, |tick, _k| {
    let chk = CHECKS.load(Ordering::Acquire);
    let err = ERRORS.load(Ordering::Acquire);
    if err > 0 {
        hprintln!("callee_save_check: FAIL - {} errors/{} checks", err, chk);
        debug::exit(debug::EXIT_FAILURE);
    }
    if chk >= 10 {
        hprintln!("callee_save_check: PASS ({} checks, 0 errors)", chk);
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 200 {
        hprintln!("callee_save_check: FAIL - timeout ({} checks)", chk);
        debug::exit(debug::EXIT_FAILURE);
    }
});
core::arch::global_asm!(
    ".syntax unified",
    ".global callee_save_stress",
    ".thumb_func",
    "callee_save_stress:",
    "push {{r4-r11, lr}}",
    "ldr r4,  =0xAAAA0004",
    "ldr r5,  =0xAAAA0005",
    "ldr r6,  =0xAAAA0006",
    "ldr r7,  =0xAAAA0007",
    "ldr r8,  =0xAAAA0008",
    "ldr r9,  =0xAAAA0009",
    "ldr r10, =0xAAAA000A",
    "ldr r11, =0xAAAA000B",
    "movs r0, #0",
    "ldr r1, =2000",
    "1: adds r0, #1",
    "cmp r0, r1",
    "blt 1b",
    "ldr r1, =0xAAAA0004",
    "cmp r4, r1",
    "bne 2f",
    "ldr r1, =0xAAAA0005",
    "cmp r5, r1",
    "bne 2f",
    "ldr r1, =0xAAAA0006",
    "cmp r6, r1",
    "bne 2f",
    "ldr r1, =0xAAAA0007",
    "cmp r7, r1",
    "bne 2f",
    "ldr r1, =0xAAAA0008",
    "cmp r8, r1",
    "bne 2f",
    "ldr r1, =0xAAAA0009",
    "cmp r9, r1",
    "bne 2f",
    "ldr r1, =0xAAAA000A",
    "cmp r10, r1",
    "bne 2f",
    "ldr r1, =0xAAAA000B",
    "cmp r11, r1",
    "bne 2f",
    "movs r0, #0",
    "pop {{r4-r11, pc}}",
    "2: movs r0, #1",
    "pop {{r4-r11, pc}}",
);

extern "C" {
    fn callee_save_stress() -> u32;
}

const _: PartitionEntry = partition_0_entry;
extern "C" fn partition_0_entry() -> ! {
    loop {
        // SAFETY: callee_save_stress is a self-contained assembly function
        // that only reads/writes callee-saved registers and the stack. No
        // shared mutable state or memory-safety invariants are involved.
        let err = unsafe { callee_save_stress() };
        if err != 0 {
            ERRORS.fetch_add(1, Ordering::Release);
        }
        CHECKS.fetch_add(1, Ordering::Release);
    }
}

const _: PartitionEntry = partition_1_entry;
extern "C" fn partition_1_entry() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    // TODO: panicking ops (.unwrap/.expect) are standard in QEMU examples
    // (consistent with all other examples); consider replacing with `?` if
    // main's return type is changed to Result in the future.
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("callee_save_check: start");
    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(2, 2).expect("sched");
    sched.add_system_window(1).expect("system window");
    let parts: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(partition_0_entry as PartitionEntry, 0),
        PartitionSpec::new(partition_1_entry as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("kernel");
    match boot(p).expect("boot") {}
}
