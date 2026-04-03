//! QEMU test: virtual UART end-to-end with two partitions.
//!
//! 2-partition demo using paired virtual UARTs (device 0 = UART-A,
//! device 1 = UART-B) with a system window for kernel bottom-half
//! data transfer.
//!
//! **Round-trip flow:**
//! 1. P1 opens UART-A and writes a message ("Hi").
//! 2. System window: kernel transfers UART-A TX → UART-B RX.
//! 3. P2 opens UART-B, reads the message, writes a response ("Ok").
//! 4. System window: kernel transfers UART-B TX → UART-A RX.
//! 5. P1 reads the response from UART-A and verifies it.
//!
//! Exits with `EXIT_SUCCESS` after verifying round-trip integrity.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    virtual_device::VirtualDevice,
    AlignedStack1K, DebugEnabled, MsgMinimal, PartitionEntry, Partitions4, PortsTiny,
    StackStorage as _, SyncMinimal,
};

const NUM_PARTITIONS: usize = 2;

/// UART-A device ID (used by P1).
const UART_A: plib::DeviceId = plib::DeviceId::new(0);
/// UART-B device ID (used by P2).
const UART_B: plib::DeviceId = plib::DeviceId::new(1);

/// Timeout in ticks for blocking device reads.
const READ_TIMEOUT: u16 = 50;

/// Message P1 sends to P2 via UART-A TX → UART-B RX.
const MSG_HELLO: &[u8] = b"Hi";
/// Response P2 sends back via UART-B TX → UART-A RX.
const MSG_REPLY: &[u8] = b"Ok";

kernel::compose_kernel_config!(DemoConfig<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Use the unified harness: single KERNEL global, no separate KS/KERN.
kernel::define_unified_harness!(DemoConfig);

// ---------------------------------------------------------------------------
// P1: opens UART-A, writes message, yields, later reads response
// ---------------------------------------------------------------------------
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    hprintln!("[P1] opening UART-A (dev {:?})", UART_A);
    assert_or_fail(
        plib::sys_dev_open(UART_A).is_ok(),
        "P1: DEV_OPEN UART-A failed",
    );

    // Copy static data to the stack so the pointer falls within the
    // partition's MPU data region (required by validate_user_ptr).
    let hello_buf: [u8; 2] = [MSG_HELLO[0], MSG_HELLO[1]];
    hprintln!("[P1] writing {:?} to UART-A", &hello_buf);
    let rc = plib::sys_dev_write(UART_A, &hello_buf).unwrap_or(0);
    assert_or_fail(rc == MSG_HELLO.len() as u32, "P1: DEV_WRITE short");
    hprintln!("[P1] wrote {} bytes to UART-A TX", rc);

    let mut buf = [0u8; 8];
    let mut received = 0usize;
    while received < MSG_REPLY.len() {
        if let Ok(n) = plib::sys_dev_read_timed(UART_A, &mut buf[received..], READ_TIMEOUT) {
            received += n as usize;
        }
    }
    hprintln!("[P1] read {} bytes from UART-A RX", received);
    let ok = received == MSG_REPLY.len() && buf[..received] == *MSG_REPLY;
    if ok {
        hprintln!("[P1] round-trip verified: response matches");
        hprintln!("virtual_uart_demo: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!(
            "[P1] MISMATCH: expected {:?}, got {:?}",
            MSG_REPLY,
            &buf[..received]
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

// ---------------------------------------------------------------------------
// P2: opens UART-B, reads message from P1, writes response
// ---------------------------------------------------------------------------
const _: PartitionEntry = p2_main;
extern "C" fn p2_main() -> ! {
    hprintln!("[P2] opening UART-B (dev {:?})", UART_B);
    assert_or_fail(
        plib::sys_dev_open(UART_B).is_ok(),
        "P2: DEV_OPEN UART-B failed",
    );

    let mut buf = [0u8; 8];
    let mut received = 0usize;
    while received < MSG_HELLO.len() {
        if let Ok(n) = plib::sys_dev_read_timed(UART_B, &mut buf[received..], READ_TIMEOUT) {
            received += n as usize;
        }
    }
    hprintln!(
        "[P2] read {} bytes from UART-B RX: {:?}",
        received,
        &buf[..received]
    );

    let ok = received == MSG_HELLO.len() && buf[..received] == *MSG_HELLO;
    assert_or_fail(ok, "P2: received unexpected data");

    // Send the response back on UART-B TX.
    // Copy static data to the stack for pointer validation.
    let reply_buf: [u8; 2] = [MSG_REPLY[0], MSG_REPLY[1]];
    hprintln!("[P2] writing {:?} to UART-B", &reply_buf);
    let rc = plib::sys_dev_write(UART_B, &reply_buf).unwrap_or(0);
    assert_or_fail(rc == MSG_REPLY.len() as u32, "P2: DEV_WRITE short");
    hprintln!("[P2] wrote {} bytes to UART-B TX", rc);

    loop {
        let _ = plib::sys_yield();
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
fn assert_or_fail(cond: bool, msg: &str) {
    if !cond {
        hprintln!("virtual_uart_demo: FAIL — {}", msg);
        debug::exit(debug::EXIT_FAILURE);
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("virtual_uart_demo: start");

    // Schedule: P1(2) → system window(1) → P2(2) → system window(1)
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).unwrap();
    sched.add_system_window(1).unwrap();
    sched.add(ScheduleEntry::new(1, 2)).unwrap();
    sched.add_system_window(1).unwrap();

    // Partition stacks must be `static` so they persist for the lifetime of
    // the system — local variables would be clobbered once the kernel starts
    // using MSP for exception frames.
    static mut STACKS: [AlignedStack1K; NUM_PARTITIONS] = [AlignedStack1K::ZERO; NUM_PARTITIONS];
    let sentinel_mpu = MpuRegion::new(0, 0, 0);
    let mems: [ExternalPartitionMemory; NUM_PARTITIONS] = {
        // SAFETY: called once from main before the scheduler starts (interrupts
        // disabled, single-core). No concurrent access to STACKS.
        let ptr = &raw mut STACKS;
        let stacks: &mut [AlignedStack1K; NUM_PARTITIONS] = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        [
            ExternalPartitionMemory::new(
                &mut s0.0,
                EntryAddr::from_entry(p1_main as PartitionEntry),
                sentinel_mpu,
                kernel::PartitionId::new(0),
            )
            .expect("ext mem"),
            ExternalPartitionMemory::new(
                &mut s1.0,
                EntryAddr::from_entry(p2_main as PartitionEntry),
                sentinel_mpu,
                kernel::PartitionId::new(1),
            )
            .expect("ext mem"),
        ]
    };

    // Create the unified kernel with schedule and partitions.
    let mut k = Kernel::<DemoConfig>::new(sched, &mems).expect("kernel creation");

    // Register the uart_pair backends in the device registry so that
    // dev_dispatch can route SYS_DEV_* syscalls to UART-A and UART-B.
    // SAFETY: k is a local variable that will be stored into the static
    // KERNEL before boot() is called. The backends live inside uart_pair
    // which lives inside k. Single-core, interrupts disabled guarantees
    // exclusive access. The 'static lifetime is valid because KERNEL is
    // 'static once stored.
    unsafe {
        let a: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.a as *mut _);
        let b: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.b as *mut _);
        k.registry.add(a).expect("register UART-A");
        k.registry.add(b).expect("register UART-B");
    }

    store_kernel(&mut k);

    match boot(p).expect("virtual_uart_demo: boot failed") {}
}
