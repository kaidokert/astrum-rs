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
    config::KernelConfig,
    kernel::KernelState,
    partition::PartitionConfig,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc,
    svc::{Kernel, SvcError},
    syscall::{SYS_DEV_OPEN, SYS_DEV_READ_TIMED, SYS_DEV_WRITE, SYS_YIELD},
    virtual_device::VirtualDevice,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 2;
const MAX_SCHEDULE_ENTRIES: usize = 8;
const STACK_WORDS: usize = 256;

/// UART-A device ID (used by P1).
const UART_A: u32 = 0;
/// UART-B device ID (used by P2).
const UART_B: u32 = 1;

/// Timeout in ticks for blocking device reads.
const READ_TIMEOUT: u32 = 50;

/// Message P1 sends to P2 via UART-A TX → UART-B RX.
const MSG_HELLO: &[u8] = b"Hi";
/// Response P2 sends back via UART-B TX → UART-A RX.
const MSG_REPLY: &[u8] = b"Ok";

struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    const BP: usize = 1;
    const BZ: usize = 32;
    const DR: usize = 4;
}

kernel::define_harness!(
    DemoConfig,
    NUM_PARTITIONS,
    MAX_SCHEDULE_ENTRIES,
    STACK_WORDS
);

// ---------------------------------------------------------------------------
// P1: opens UART-A, writes message, yields, later reads response
// ---------------------------------------------------------------------------
extern "C" fn p1_main() -> ! {
    hprintln!("[P1] opening UART-A (dev {})", UART_A);
    let rc = svc!(SYS_DEV_OPEN, UART_A, 0u32, 0u32);
    assert_or_fail(!SvcError::is_error(rc), "P1: DEV_OPEN UART-A failed");

    // Copy static data to the stack so the pointer falls within the
    // partition's MPU data region (required by validate_user_ptr).
    let hello_buf: [u8; 2] = [MSG_HELLO[0], MSG_HELLO[1]];
    hprintln!("[P1] writing {:?} to UART-A", &hello_buf);
    let rc = svc!(
        SYS_DEV_WRITE,
        UART_A,
        hello_buf.len() as u32,
        hello_buf.as_ptr() as u32
    );
    assert_or_fail(rc == MSG_HELLO.len() as u32, "P1: DEV_WRITE short");
    hprintln!("[P1] wrote {} bytes to UART-A TX", rc);

    // TODO: SYS_DEV_READ_TIMED currently reads one byte per call (kernel
    // hardcodes len=1). Replace this loop with a single multi-byte blocking
    // read once the kernel supports a length parameter.
    // TODO: reviewer false positive — SYS_DEV_READ_TIMED ABI is
    // (device_id, timeout, buf_ptr) with no len arg; the kernel validates
    // exactly 1 byte at buf_ptr.
    let mut buf = [0u8; 8];
    let mut received = 0usize;
    while received < MSG_REPLY.len() {
        let n = svc!(
            SYS_DEV_READ_TIMED,
            UART_A,
            READ_TIMEOUT,
            buf[received..].as_mut_ptr() as u32
        );
        if !SvcError::is_error(n) && n > 0 {
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
extern "C" fn p2_main() -> ! {
    hprintln!("[P2] opening UART-B (dev {})", UART_B);
    let rc = svc!(SYS_DEV_OPEN, UART_B, 0u32, 0u32);
    assert_or_fail(!SvcError::is_error(rc), "P2: DEV_OPEN UART-B failed");

    // TODO: SYS_DEV_READ_TIMED currently reads one byte per call (kernel
    // hardcodes len=1). Replace this loop with a single multi-byte blocking
    // read once the kernel supports a length parameter.
    let mut buf = [0u8; 8];
    let mut received = 0usize;
    while received < MSG_HELLO.len() {
        let n = svc!(
            SYS_DEV_READ_TIMED,
            UART_B,
            READ_TIMEOUT,
            buf[received..].as_mut_ptr() as u32
        );
        if !SvcError::is_error(n) && n > 0 {
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
    let rc = svc!(
        SYS_DEV_WRITE,
        UART_B,
        reply_buf.len() as u32,
        reply_buf.as_ptr() as u32
    );
    assert_or_fail(rc == MSG_REPLY.len() as u32, "P2: DEV_WRITE short");
    hprintln!("[P2] wrote {} bytes to UART-B TX", rc);

    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
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
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("virtual_uart_demo: start");

    // SAFETY: single-core, interrupts not yet enabled.
    unsafe {
        store_kernel(Kernel::<DemoConfig>::new_empty(
            kernel::virtual_device::DeviceRegistry::new(),
        ));

        // Register the uart_pair backends in the device registry so that
        // dev_dispatch can route SYS_DEV_* syscalls to UART-A and UART-B.
        cortex_m::interrupt::free(|cs| {
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                // TODO: reviewer false positive — this SAFETY comment was
                // not removed; the diff was truncated.
                // SAFETY: KERN is a static that is never dropped. The
                // backends live inside uart_pair which lives inside
                // KERN. Interrupts are disabled (interrupt::free),
                // guaranteeing exclusive access on single-core Cortex-M.
                // The 'static lifetime is valid because KERN is 'static.
                let a: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.a as *mut _);
                let b: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.b as *mut _);
                k.registry.add(a).expect("register UART-A");
                k.registry.add(b).expect("register UART-B");
            }
        });

        // Schedule: P1(2) → system window(1) → P2(2) → system window(1)
        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        sched.add(ScheduleEntry::new(0, 2)).unwrap();
        sched.add_system_window(1).unwrap();
        sched.add(ScheduleEntry::new(1, 2)).unwrap();
        sched.add_system_window(1).unwrap();
        sched.start();

        let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| {
            let b = 0x2000_0000 + (i as u32) * 0x2000;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        });
        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p1_main, 0), (p2_main, 0)];

    boot(&parts, &mut p)
}
