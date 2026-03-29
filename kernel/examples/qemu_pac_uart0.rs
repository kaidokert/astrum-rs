//! QEMU proof-of-concept: partition writes to UART0 via PAC typed register API.
//!
//! Privileged main() enables the UART0 clock via SYSCTL RCGC1 and configures
//! UART0 for TX.  The kernel's init_kernel + boot path handles MPU setup
//! automatically.  Partition P0 receives UART0 peripheral access via
//! PartitionSpec::with_peripherals() and transmits bytes using PAC typed
//! register accessors.  A SysTick hook checks an AtomicU32 state flag set
//! by P0 after successful TX and reports PASS via semihosting.
//!
//! Run:
//!   cargo run --example qemu_pac_uart0 --features qemu,log-semihosting,qemu-peripherals \
//!     --target thumbv7m-none-eabi --release
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    partition::MpuRegion,
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const UART0_BASE: u32 = 0x4000_C000;
const UART0_SIZE: u32 = 4096;

/// Static UART0 peripheral region for PartitionSpec builder.
static UART0_REGIONS: [MpuRegion; 1] = [MpuRegion::new(UART0_BASE, UART0_SIZE, 0)];

/// State flag: 0 = not started, 1 = TX complete.
static TX_STATE: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: write bytes to UART0 DR via PAC typed API, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: We have MPU-granted access to UART0 region; no other code
    // accesses UART0 concurrently.
    let uart0 = unsafe { &*tudelft_lm3s6965_pac::UART0::ptr() };

    let msg: &[u8] = b"UART0:TX_OK\n";
    for &byte in msg {
        // Busy-wait while TX FIFO is full before writing next byte.
        while uart0.fr.read().uart_fr_txff().bit() {
            cortex_m::asm::nop();
        }
        // SAFETY: Writing valid 8-bit data to the DR data field.
        uart0.dr.write(|w| unsafe { w.uart_dr_data().bits(byte) });
    }

    // Signal completion AFTER all bytes successfully written.
    TX_STATE.store(1, Ordering::Release);
    loop {
        let _ = plib::sys_yield();
    }
}

/// P1: idle partition (two-partition minimum for round-robin).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        let _ = plib::sys_yield();
    }
}

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let state = TX_STATE.load(Ordering::Acquire);
    if state == 1 {
        hprintln!("qemu_pac_uart0: PASS partition TX complete");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_uart0: FAIL timeout (state={})", state);
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("qemu_pac_uart0: start");

    // --- Privileged UART0 setup via PAC typed API ---
    // SAFETY: We are in privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };

    // Enable UART0 clock in SYSCTL RCGC1.
    pac.SYSCTL
        .rcgc1
        .modify(|_, w| w.sysctl_rcgc1_uart0().set_bit());

    // Brief delay for clock to stabilize (a few NOPs suffice on QEMU).
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    // Configure UART0: 8-bit word length, FIFOs enabled.
    pac.UART0.lcrh.write(|w| {
        w.uart_lcrh_wlen()
            .uart_lcrh_wlen_8()
            .uart_lcrh_fen()
            .set_bit()
    });
    // Enable UART and TX.
    pac.UART0
        .ctl
        .write(|w| w.uart_ctl_uarten().set_bit().uart_ctl_txe().set_bit());

    // --- Kernel setup via init_kernel + boot ---
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched 0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    sched.add_system_window(1).expect("sys1");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::entry(p0_entry).with_peripherals(&UART0_REGIONS),
        PartitionSpec::entry(p1_entry),
    ];

    let k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(k);
    match boot(p).expect("boot") {}
}
