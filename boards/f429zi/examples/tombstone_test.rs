//! Tombstone Post-Mortem Test
//!
//! P0 panics after a few iterations. After reset, the tombstone at .noinit
//! should contain the panic file/line/message, readable via GDB or probe-rs.
//!
//! Protocol:
//!   1. Flash and run → P0 panics → tombstone written → halt
//!   2. Soft reset (no flash) → tombstone survives in .noinit RAM
//!   3. Read tombstone at PANIC_TOMBSTONE address → verify magic 0xDEADC0DE
//!
//! Build: cd f429zi && cargo build --example tombstone_test --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, PartitionEntry,
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use f429zi as _;

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(
    TombCfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

kernel::define_kernel!(TombCfg, |tick, _k| {
    if tick % 100 == 0 {
        let c = P0_COUNTER.load(Ordering::Acquire);
        rprintln!("[{:5}ms] P0={}", tick, c);
    }
});

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    for i in 0..500u32 {
        P0_COUNTER.store(i, Ordering::Release);
        cortex_m::asm::nop();
    }
    panic!("tombstone test: deliberate panic at iteration 500");
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop { cortex_m::asm::nop(); }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();

    // Init RTT early so tombstone readout is visible
    kernel::boot::init_rtt();

    // Check tombstone from previous run
    // Address must match the linker placement of kernel::tombstone::PANIC_TOMBSTONE
    // (in .noinit). Verify with: `nm firmware.elf | grep PANIC_TOMBSTONE`
    let tomb_addr = 0x2000_9000 as *const u32;
    let magic = unsafe { core::ptr::read_volatile(tomb_addr) };
    if magic == 0xDEAD_C0DE {
        rprintln!("\n!!! TOMBSTONE FROM PREVIOUS PANIC !!!");
        let line = unsafe { core::ptr::read_volatile(tomb_addr.add(1)) };
        let file_ptr = (tomb_addr as u32 + 8) as *const u8;
        let msg_ptr = (tomb_addr as u32 + 56) as *const u8;
        // Read file as string
        let mut file_buf = [0u8; 48];
        for i in 0..48 {
            file_buf[i] = unsafe { core::ptr::read_volatile(file_ptr.add(i)) };
        }
        let file_len = file_buf.iter().position(|&b| b == 0).unwrap_or(48);
        let file = core::str::from_utf8(&file_buf[..file_len]).unwrap_or("???");
        // Read message
        let mut msg_buf = [0u8; 72];
        for i in 0..72 {
            msg_buf[i] = unsafe { core::ptr::read_volatile(msg_ptr.add(i)) };
        }
        let msg_len = msg_buf.iter().position(|&b| b == 0).unwrap_or(72);
        let msg = core::str::from_utf8(&msg_buf[..msg_len]).unwrap_or("???");
        rprintln!("  File: {}:{}", file, line);
        rprintln!("  Message: {}", msg);
        rprintln!("SUCCESS: tombstone survived reset!\n");
    } else {
        rprintln!("\n=== Tombstone Test — no previous tombstone (magic={:#010x}) ===", magic);
        rprintln!("P0 will panic after 500 iterations. Then soft-reset to read tombstone.\n");
    }

    let sched = ScheduleTable::<{ TombCfg::SCHED }>::round_robin(2, 2).expect("sched");
    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
