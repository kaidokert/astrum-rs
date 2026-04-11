//! Bad Syscall Pointer Test — verify kernel rejects malicious buffers
//!
//! P0 (survivor): simple counter, proves kernel stays alive.
//! P1 (attacker): passes bad pointers to syscalls:
//!   - Flash pointer (0x0800_xxxx) — string literal, outside partition RAM
//!   - Null pointer (0x0000_0000)
//!   - Kernel guard region (0x2000_8000) — privileged-only
//!   - Out-of-SRAM address (0x6000_0000) — unmapped
//!
//! Expected: all bad pointer syscalls return Err (InvalidPointer or similar),
//! P1 continues running (not faulted), P0 counter advances, kernel alive.
//! If the kernel crashes or P1 faults, the pointer validation is broken.
//!
//! Build: cd f429zi && cargo build --example bad_pointer_test --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    partition::PartitionState,
    scheduler::{ScheduleEntry, ScheduleTable},
    message::MessageQueue,
    PartitionSpec,
    {DebugEnabled, MsgSmall, Partitions4, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

kernel::kernel_config!(
    PtrCfg<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
        SW = 2; MS = 2; MW = 2;
        SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
    }
);

static P0_COUNT: AtomicU32 = AtomicU32::new(0);

// Attack results from P1 (0 = not attempted, 1 = got Err as expected, 2 = unexpected Ok, 3 = faulted)
static FLASH_PTR_RESULT: AtomicU32 = AtomicU32::new(0);
static NULL_PTR_RESULT: AtomicU32 = AtomicU32::new(0);
static GUARD_PTR_RESULT: AtomicU32 = AtomicU32::new(0);
static UNMAPPED_PTR_RESULT: AtomicU32 = AtomicU32::new(0);
static ATTACKS_DONE: AtomicU32 = AtomicU32::new(0);

kernel::define_kernel!(PtrCfg, |tick, k| {
    if tick % 500 == 0 && tick > 0 {
        let p0 = P0_COUNT.load(Ordering::Acquire);
        let done = ATTACKS_DONE.load(Ordering::Acquire);
        let p1_state = k.pcb(1).map(|p| p.state());

        let flash = FLASH_PTR_RESULT.load(Ordering::Acquire);
        let null = NULL_PTR_RESULT.load(Ordering::Acquire);
        let guard = GUARD_PTR_RESULT.load(Ordering::Acquire);
        let unmapped = UNMAPPED_PTR_RESULT.load(Ordering::Acquire);

        rprintln!(
            "[{:5}ms] P0={} P1={:?} attacks={} flash={} null={} guard={} unmapped={}",
            tick, p0, p1_state, done, flash, null, guard, unmapped
        );

        // Success: all 4 attacks returned Err (result=1), P1 still running, P0 alive
        if done >= 4 && flash == 1 && null == 1 && guard == 1 && unmapped == 1
            && p1_state != Some(PartitionState::Faulted)
            && p0 > 0
        {
            rprintln!("SUCCESS: all bad pointer syscalls rejected, no faults, kernel alive");
        }

        // Partial: some attacks caused faults (result stayed 0 or P1 faulted)
        if done >= 4 && p1_state == Some(PartitionState::Faulted) {
            rprintln!("FAIL: P1 faulted — kernel did not validate pointer before access");
        }
    }
});

// ── P0: survivor ──
extern "C" fn survivor_body(_r0: u32) -> ! {
    loop {
        P0_COUNT.fetch_add(1, Ordering::Release);
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(survivor_main => survivor_body);

// ── P1: attacker — tries bad pointers via sys_msg_send ──
extern "C" fn attacker_body(_r0: u32) -> ! {
    let target = 0u32.into(); // msg queue 0

    // Attack 1: flash pointer (string literal lives at 0x0800xxxx)
    // Bounce through a raw pointer to construct a slice in flash.
    {
        let flash_addr: *const u8 = 0x0800_1000 as *const u8;
        let flash_slice = unsafe { core::slice::from_raw_parts(flash_addr, 4) };
        let result = plib::sys_msg_send(target, flash_slice);
        FLASH_PTR_RESULT.store(if result.is_err() { 1 } else { 2 }, Ordering::Release);
        ATTACKS_DONE.fetch_add(1, Ordering::Release);
    }

    // Attack 2: near-null pointer (0x00000004 — vector table, not partition RAM)
    {
        let null_addr: *const u8 = 0x0000_0004 as *const u8;
        let null_slice = unsafe { core::slice::from_raw_parts(null_addr, 4) };
        let result = plib::sys_msg_send(target, null_slice);
        NULL_PTR_RESULT.store(if result.is_err() { 1 } else { 2 }, Ordering::Release);
        ATTACKS_DONE.fetch_add(1, Ordering::Release);
    }

    // Attack 3: kernel guard region (privileged-only, 0x2000_8000)
    {
        let guard_addr: *const u8 = 0x2000_8000 as *const u8;
        let guard_slice = unsafe { core::slice::from_raw_parts(guard_addr, 4) };
        let result = plib::sys_msg_send(target, guard_slice);
        GUARD_PTR_RESULT.store(if result.is_err() { 1 } else { 2 }, Ordering::Release);
        ATTACKS_DONE.fetch_add(1, Ordering::Release);
    }

    // Attack 4: unmapped address (0x6000_0000 — no peripheral or RAM here)
    {
        let bad_addr: *const u8 = 0x6000_0000 as *const u8;
        let bad_slice = unsafe { core::slice::from_raw_parts(bad_addr, 4) };
        let result = plib::sys_msg_send(target, bad_slice);
        UNMAPPED_PTR_RESULT.store(if result.is_err() { 1 } else { 2 }, Ordering::Release);
        ATTACKS_DONE.fetch_add(1, Ordering::Release);
    }

    // All attacks done — keep running to prove P1 survived
    loop {
        plib::sys_yield().ok();
    }
}
kernel::partition_trampoline!(attacker_main => attacker_body);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Bad Syscall Pointer Test ===");
    rprintln!("P1 passes flash/null/guard/unmapped pointers to sys_msg_send");
    rprintln!("Expected: all rejected with Err, P1 survives, kernel alive\n");

    let mut sched = ScheduleTable::<{ PtrCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("P0");
    sched.add(ScheduleEntry::new(1, 4)).expect("P1");
    sched.add_system_window(1).expect("SW");

    let parts: [PartitionSpec; 2] = [
        PartitionSpec::entry(survivor_main),
        PartitionSpec::entry(attacker_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Create a message queue for P1 to target
    kernel::state::with_kernel_mut::<PtrCfg, _, _>(|k| {
        k.messages_mut().add(MessageQueue::new()).expect("queue");
        Ok::<(), ()>(())
    }).expect("ipc");

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
