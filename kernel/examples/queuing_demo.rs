//! 2-partition command/response pipeline via paired queuing ports.
//!
//! Demonstrates: FIFO message delivery, command→response round-trips,
//! **queue-full detection** (the commander sends more messages than
//! the queue depth and verifies that the overflow is reported), and
//! **timed receive** (`SYS_QUEUING_RECV_TIMED` with a non-zero timeout
//! to block until a message arrives).
//!
//! Partitions run unprivileged and cannot use semihosting directly.
//! Progress is tracked via atomics; the SysTick handler verifies state
//! and prints results from privileged handler mode.
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
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    PartitionBody, PartitionEntry,
};
use plib::SvcError;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config
// ---------------------------------------------------------------------------
const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
// Kernel configuration for the queuing-port demo.
//
// Sized for 2 partitions, depth-4 queuing ports with 4-byte messages,
// and moderate pool sizes for all resource types.
kernel::kernel_config!(DemoConfig<kernel::Partitions2, kernel::SyncStandard, kernel::MsgStandard, kernel::PortsSmall, kernel::DebugEnabled>);

// ---------------------------------------------------------------------------
// Command / response protocol constants
// ---------------------------------------------------------------------------
const CMD_START: u8 = 1;
const CMD_MEASURE: u8 = 2;
const CMD_STOP: u8 = 3;
const CMD_EXTRA_1: u8 = 4; // overflow probe: exceeds QUEUE_DEPTH
const CMD_EXTRA_2: u8 = 5; // overflow probe: exceeds QUEUE_DEPTH

const CMD_TIMED: u8 = 6; // Phase 3: timed receive test

const RSP_START_ACK: u8 = 0x10;
const RSP_MEASURE_ACK: u8 = 0x20;
const RSP_STOP_ACK: u8 = 0x30;
const RSP_EXTRA_1_ACK: u8 = 0x40;
const RSP_TIMED_ACK: u8 = 0x60; // Phase 3: timed receive response
const RSP_UNKNOWN: u8 = 0xFF;

/// Total commands we attempt to send, including those that should overflow.
const TOTAL_CMDS: usize = 5;

/// Commands that fit within QUEUE_DEPTH and will be delivered.
const CMDS: [u8; TOTAL_CMDS] = [CMD_START, CMD_MEASURE, CMD_STOP, CMD_EXTRA_1, CMD_EXTRA_2];

/// Expected responses for the commands that were successfully delivered (first QUEUE_DEPTH).
const EXPECTED_RSPS: [u8; QUEUE_DEPTH] = [
    RSP_START_ACK,
    RSP_MEASURE_ACK,
    RSP_STOP_ACK,
    RSP_EXTRA_1_ACK,
];

// ---------------------------------------------------------------------------
// Atomic state for partition progress tracking (handler mode reads these)
// ---------------------------------------------------------------------------

/// Commander: number of commands successfully delivered
static CMD_DELIVERED: AtomicU32 = AtomicU32::new(0);
/// Commander: set to 1 when queue-full was detected
static QUEUE_FULL_SEEN: AtomicU32 = AtomicU32::new(0);
/// Commander: number of responses received
static RSP_RECEIVED: AtomicU32 = AtomicU32::new(0);
/// Commander: set to non-zero if a response mismatch was detected (stores 0x100 + expected)
static RSP_MISMATCH: AtomicU32 = AtomicU32::new(0);
/// Commander: set to 1 when timed response was received and matched
static TIMED_RSP_OK: AtomicU32 = AtomicU32::new(0);
/// Commander: set to 1 when timed command was sent
static TIMED_CMD_SENT: AtomicU32 = AtomicU32::new(0);

/// Worker: number of commands processed
static WORKER_PROCESSED: AtomicU32 = AtomicU32::new(0);
/// Worker: set to 1 when timed recv started
static WORKER_TIMED_RECV: AtomicU32 = AtomicU32::new(0);
/// Worker: set to 1 when timed command was received
static WORKER_TIMED_OK: AtomicU32 = AtomicU32::new(0);

// Use the unified harness macro with SysTick hook for progress verification.
// The hook runs in privileged handler mode and can use semihosting.
kernel::define_kernel!(DemoConfig, |tick, _k| {
    // Check progress every 10 ticks
    if tick.is_multiple_of(10) {
        let delivered = CMD_DELIVERED.load(Ordering::Acquire);
        let qf_seen = QUEUE_FULL_SEEN.load(Ordering::Acquire);
        let rsp_recv = RSP_RECEIVED.load(Ordering::Acquire);
        let mismatch = RSP_MISMATCH.load(Ordering::Acquire);
        let timed_sent = TIMED_CMD_SENT.load(Ordering::Acquire);
        let timed_ok = TIMED_RSP_OK.load(Ordering::Acquire);
        let worker_proc = WORKER_PROCESSED.load(Ordering::Acquire);
        let worker_timed = WORKER_TIMED_OK.load(Ordering::Acquire);

        hprintln!(
            "[tick {}] del={} qf={} rsp={} w_proc={} timed_sent={} timed_ok={}",
            tick,
            delivered,
            qf_seen,
            rsp_recv,
            worker_proc,
            timed_sent,
            timed_ok
        );

        // Check for failures
        if mismatch != 0 {
            hprintln!("queuing_demo: FAIL - response mismatch (code={})", mismatch);
            debug::exit(debug::EXIT_FAILURE);
        }

        // Test passes when:
        // 1. Queue full was detected (qf_seen == 1)
        // 2. All expected responses received (rsp_recv >= delivered)
        // 3. Timed response received and matched (timed_ok == 1)
        // 4. Worker processed all commands and timed command
        if rsp_recv >= QUEUE_DEPTH as u32
            && delivered >= QUEUE_DEPTH as u32
            && timed_ok == 1
            && worker_timed == 1
        {
            hprintln!("queuing_demo: all checks passed");
            debug::exit(debug::EXIT_SUCCESS);
        }

        // Timeout after 200 ticks
        if tick > 200 {
            hprintln!("queuing_demo: FAIL - timeout");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
});

// ---------------------------------------------------------------------------
// Commander partition: sends commands, detects queue-full, receives responses
// ---------------------------------------------------------------------------
const _: PartitionBody = commander_main_body;
extern "C" fn commander_main_body(r0: u32) -> ! {
    // Unpack port IDs: upper 16 bits = command Source port, lower 16 = response Destination port.
    let packed = r0;
    let (cmd_port, rsp_port) = (
        plib::QueuingPortId::new(packed >> 16),
        plib::QueuingPortId::new(packed & 0xFFFF),
    );

    // Phase 1: Flood the command queue to demonstrate queue-full detection.
    // QUEUE_DEPTH is 4, so the 5th send must fail with a kernel error.
    let mut delivered: u32 = 0;
    for &cmd in &CMDS {
        match plib::sys_queuing_send(cmd_port, &[cmd]) {
            Ok(_) => {
                delivered += 1;
                CMD_DELIVERED.store(delivered, Ordering::Release);
            }
            Err(SvcError::OperationFailed) => {
                QUEUE_FULL_SEEN.store(1, Ordering::Release);
            }
            Err(e) => {
                RSP_MISMATCH.store(0x300 | e.to_u32(), Ordering::Release);
            }
        }
    }

    // Yield to let the worker drain and respond.
    plib::sys_yield().expect("yield after send phase");

    // Phase 2: Collect responses for the commands that were successfully delivered.
    let mut n: usize = 0;
    while n < delivered as usize && n < EXPECTED_RSPS.len() {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        match plib::sys_queuing_recv(rsp_port, &mut buf) {
            Ok(0) | Err(SvcError::OperationFailed) => {
                plib::sys_yield().expect("yield on empty recv");
                continue;
            }
            Err(e) => {
                RSP_MISMATCH.store(0x400 | e.to_u32(), Ordering::Release);
                break;
            }
            Ok(_) => {}
        }
        let (got, exp) = (buf[0], EXPECTED_RSPS[n]);
        if got != exp {
            RSP_MISMATCH.store(0x100 | exp as u32, Ordering::Release);
        }
        n += 1;
        RSP_RECEIVED.store(n as u32, Ordering::Release);
        plib::sys_yield().expect("yield after recv");
    }

    // Phase 3: Timed receive test.
    // Yield first so the worker enters the timed-recv wait.
    plib::sys_yield().expect("yield before timed phase");
    plib::sys_yield().expect("yield before timed phase 2");

    // Send a late command – this wakes the blocked worker.
    let timed_buf = [CMD_TIMED];
    match plib::sys_queuing_send(cmd_port, &timed_buf) {
        Ok(_) => {
            TIMED_CMD_SENT.store(1, Ordering::Release);
        }
        Err(e) => {
            RSP_MISMATCH.store(0x500 | e.to_u32(), Ordering::Release);
        }
    }

    // Yield to let the worker process and respond.
    plib::sys_yield().expect("yield after timed send");

    // Collect the timed response.
    loop {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        match plib::sys_queuing_recv(rsp_port, &mut buf) {
            Ok(0) | Err(SvcError::OperationFailed) => {
                plib::sys_yield().expect("yield on empty timed recv");
                continue;
            }
            Err(e) => {
                RSP_MISMATCH.store(0x600 | e.to_u32(), Ordering::Release);
                break;
            }
            Ok(_) => {}
        }
        if buf[0] == RSP_TIMED_ACK {
            TIMED_RSP_OK.store(1, Ordering::Release);
        } else {
            RSP_MISMATCH.store(0x200 | RSP_TIMED_ACK as u32, Ordering::Release);
        }
        break;
    }

    // Done – keep yielding forever.
    #[allow(clippy::empty_loop)]
    loop {
        plib::sys_yield().expect("yield in idle loop");
    }
}
kernel::partition_trampoline!(commander_main => commander_main_body);

// ---------------------------------------------------------------------------
// Worker partition: receives commands, maps to responses, sends back
// ---------------------------------------------------------------------------
const _: PartitionBody = worker_main_body;
extern "C" fn worker_main_body(r0: u32) -> ! {
    // Unpack port IDs: upper 16 bits = response Source port, lower 16 = command Destination port.
    let packed = r0;
    let (rsp_port, cmd_port) = (
        plib::QueuingPortId::new(packed >> 16),
        plib::QueuingPortId::new(packed & 0xFFFF),
    );

    // Phase 1-2: Process the initial batch of commands using non-blocking recv.
    let mut processed: u32 = 0;
    let mut empty_streak: u32 = 0;
    while empty_streak < 3 || processed < QUEUE_DEPTH as u32 {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        match plib::sys_queuing_recv(cmd_port, &mut buf) {
            Ok(0) | Err(SvcError::OperationFailed) => {
                if processed >= QUEUE_DEPTH as u32 {
                    empty_streak += 1;
                }
                plib::sys_yield().expect("yield on empty worker recv");
                continue;
            }
            Err(_) => {
                plib::sys_yield().expect("yield on worker recv error");
                continue;
            }
            Ok(_) => {
                empty_streak = 0;
            }
        }
        let rsp = match buf[0] {
            CMD_START => RSP_START_ACK,
            CMD_MEASURE => RSP_MEASURE_ACK,
            CMD_STOP => RSP_STOP_ACK,
            CMD_EXTRA_1 => RSP_EXTRA_1_ACK,
            _ => RSP_UNKNOWN,
        };
        match plib::sys_queuing_send(rsp_port, &[rsp]) {
            Ok(_) => {}
            Err(e) => {
                // Response send failed — report but continue processing.
                RSP_MISMATCH.store(0x700 | e.to_u32(), Ordering::Release);
            }
        }
        processed += 1;
        WORKER_PROCESSED.store(processed, Ordering::Release);
        plib::sys_yield().expect("yield after worker send");
    }

    // Phase 3: Use timed receive to block waiting for the next command.
    WORKER_TIMED_RECV.store(1, Ordering::Release);
    let mut buf = [0u8; QUEUE_MSG_SIZE];

    // Step 1: register in the receiver wait queue with a timeout.
    // Step 2: if blocked (sz == 0, i.e. Ok(0)), yield to let the scheduler
    // switch to the commander, then poll until the message arrives.
    match plib::sys_queuing_recv_timed(cmd_port, &mut buf, 100) {
        Ok(0) => {
            // Blocked — no immediate message. Yield and poll.
            loop {
                plib::sys_yield().expect("yield in timed recv poll");
                match plib::sys_queuing_recv(cmd_port, &mut buf) {
                    Ok(n) if n > 0 => break,
                    Ok(_) | Err(SvcError::OperationFailed) => continue,
                    Err(_) => continue,
                }
            }
        }
        Ok(_sz) => {
            // Message received immediately — proceed.
        }
        Err(_) => {
            // Unexpected error from timed recv — fall through to response.
        }
    }

    let rsp = match buf[0] {
        CMD_TIMED => RSP_TIMED_ACK,
        _ => RSP_UNKNOWN,
    };
    match plib::sys_queuing_send(rsp_port, &[rsp]) {
        Ok(_) => {}
        Err(e) => {
            RSP_MISMATCH.store(0x800 | e.to_u32(), Ordering::Release);
        }
    }
    if buf[0] == CMD_TIMED {
        WORKER_TIMED_OK.store(1, Ordering::Release);
    }

    // Done – keep yielding forever.
    #[allow(clippy::empty_loop)]
    loop {
        plib::sys_yield().expect("yield in worker idle loop");
    }
}
kernel::partition_trampoline!(worker_main => worker_main_body);

// ---------------------------------------------------------------------------
// Entry point: create ports, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("queuing_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..DemoConfig::N as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    let eps: [PartitionEntry; DemoConfig::N] = [commander_main, worker_main];
    let mut k = {
        use kernel::partition::{ExternalPartitionMemory, MpuRegion};
        let stacks = kernel::partition_stacks!(DemoConfig, DemoConfig::N);
        let sp = stacks.as_mut_ptr();
        let mems: [_; DemoConfig::N] = core::array::from_fn(|i| {
            // SAFETY: `sp` points to the first of `DemoConfig::N` contiguous
            // `StackStorage` elements returned by `partition_stacks!`, and `i`
            // is in 0..N, so `sp.add(i)` is in-bounds. Each index is visited
            // exactly once by `from_fn`, so no aliasing occurs.
            let s = unsafe { &mut *sp.add(i) };
            ExternalPartitionMemory::from_aligned_stack(
                s,
                eps[i],
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(i as u32),
            )
            .expect("mem")
        });
        sched.add_system_window(1).expect("sys window");
        Kernel::<DemoConfig>::new(sched, &mems).expect("kernel creation")
    };

    // Command channel: commander (Source cs) -> worker (Destination cd)
    let cs = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("cs port");
    let cd = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("cd port");
    k.queuing_mut()
        .connect_ports(cs, cd)
        .expect("connect cs->cd");

    // Response channel: worker (Source rs) -> commander (Destination rd)
    let rs = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("rs port");
    let rd = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("rd port");
    k.queuing_mut()
        .connect_ports(rs, rd)
        .expect("connect rs->rd");

    // Set r0 hints: bits [31:16] = outgoing port, bits [15:0] = incoming port.
    let hints = [(cs as u32) << 16 | rd as u32, (rs as u32) << 16 | cd as u32];
    for (i, &h) in hints.iter().enumerate() {
        k.partitions_mut().get_mut(i).expect("pcb").set_r0_hint(h);
    }
    store_kernel(&mut k);
    // SAFETY: PCBs populated by Kernel::new() with valid stacks.
    match unsafe { kernel::boot::boot_preconfigured::<DemoConfig>(p) }.expect("queuing_demo: boot") {}
}
