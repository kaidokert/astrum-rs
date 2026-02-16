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
use kernel::{
    config::KernelConfig,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::PartitionCore,
    port_pools::PortPools,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc,
    svc::Kernel,
    sync_pools::SyncPools,
    syscall::{SYS_QUEUING_RECV, SYS_QUEUING_RECV_TIMED, SYS_QUEUING_SEND, SYS_YIELD},
    unpack_r0,
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config
// ---------------------------------------------------------------------------
const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 256;

/// Kernel configuration for the queuing-port demo.
///
/// Sized for 4 partitions, depth-4 queuing ports with 4-byte messages,
/// and moderate pool sizes for all resource types.
struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const STACK_WORDS: usize = 256;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 4;
    const MW: usize = 4;
    const QS: usize = 4;
    const QD: usize = QUEUE_DEPTH;
    const QM: usize = QUEUE_MSG_SIZE;
    const QW: usize = 4;
    const SP: usize = 4;
    const SM: usize = 4;
    const BS: usize = 4;
    const BM: usize = 4;
    const BW: usize = 4;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

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

/// The syscall returns an error code with the high bit set on failure
/// (queue empty, queue full, invalid port).  All `SvcError` variants live
/// in the range 0xFFFF_FFFA ..= 0xFFFF_FFFF, so testing the MSB is the
/// portable way to detect any kernel error.
const SVC_ERROR_BIT: u32 = 0x8000_0000;

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
kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
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
        if qf_seen == 1
            && rsp_recv >= delivered
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
extern "C" fn commander_main() -> ! {
    // Unpack port IDs: upper 16 bits = command Source port, lower 16 = response Destination port.
    let packed = unpack_r0!();
    let (cmd_port, rsp_port) = (packed >> 16, packed & 0xFFFF);

    // Phase 1: Flood the command queue to demonstrate queue-full detection.
    // QUEUE_DEPTH is 4, so the 5th send must fail with a kernel error.
    let mut delivered: u32 = 0;
    for &cmd in &CMDS {
        let rc = svc!(SYS_QUEUING_SEND, cmd_port, 1u32, [cmd].as_ptr() as u32);
        if rc & SVC_ERROR_BIT != 0 {
            QUEUE_FULL_SEEN.store(1, Ordering::Release);
        } else {
            delivered += 1;
            CMD_DELIVERED.store(delivered, Ordering::Release);
        }
    }

    // Yield to let the worker drain and respond.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // Phase 2: Collect responses for the commands that were successfully delivered.
    let mut n: usize = 0;
    while n < delivered as usize && n < EXPECTED_RSPS.len() {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        let sz = svc!(
            SYS_QUEUING_RECV,
            rsp_port,
            QUEUE_MSG_SIZE as u32,
            buf.as_mut_ptr() as u32
        );
        if sz & SVC_ERROR_BIT != 0 {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            continue;
        }
        let (got, exp) = (buf[0], EXPECTED_RSPS[n]);
        if got != exp {
            RSP_MISMATCH.store(0x100 | exp as u32, Ordering::Release);
        }
        n += 1;
        RSP_RECEIVED.store(n as u32, Ordering::Release);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }

    // Phase 3: Timed receive test.
    // Yield first so the worker enters the timed-recv wait.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // Send a late command – this wakes the blocked worker.
    let cmd = [CMD_TIMED];
    let rc = svc!(SYS_QUEUING_SEND, cmd_port, 1u32, cmd.as_ptr() as u32);
    if rc & SVC_ERROR_BIT == 0 {
        TIMED_CMD_SENT.store(1, Ordering::Release);
    }

    // Yield to let the worker process and respond.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // Collect the timed response.
    loop {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        let sz = svc!(
            SYS_QUEUING_RECV,
            rsp_port,
            QUEUE_MSG_SIZE as u32,
            buf.as_mut_ptr() as u32
        );
        if sz & SVC_ERROR_BIT != 0 {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            continue;
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
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

// ---------------------------------------------------------------------------
// Worker partition: receives commands, maps to responses, sends back
// ---------------------------------------------------------------------------
extern "C" fn worker_main() -> ! {
    // Unpack port IDs: upper 16 bits = response Source port, lower 16 = command Destination port.
    let packed = unpack_r0!();
    let (rsp_port, cmd_port) = (packed >> 16, packed & 0xFFFF);

    // Phase 1-2: Process the initial batch of commands using non-blocking recv.
    let mut processed: u32 = 0;
    while processed < QUEUE_DEPTH as u32 {
        let mut buf = [0u8; QUEUE_MSG_SIZE];
        let sz = svc!(
            SYS_QUEUING_RECV,
            cmd_port,
            QUEUE_MSG_SIZE as u32,
            buf.as_mut_ptr() as u32
        );
        if sz & SVC_ERROR_BIT != 0 {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            continue;
        }
        let rsp = match buf[0] {
            CMD_START => RSP_START_ACK,
            CMD_MEASURE => RSP_MEASURE_ACK,
            CMD_STOP => RSP_STOP_ACK,
            CMD_EXTRA_1 => RSP_EXTRA_1_ACK,
            _ => RSP_UNKNOWN,
        };
        svc!(SYS_QUEUING_SEND, rsp_port, 1u32, [rsp].as_ptr() as u32);
        processed += 1;
        WORKER_PROCESSED.store(processed, Ordering::Release);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }

    // Phase 3: Use timed receive to block waiting for the next command.
    WORKER_TIMED_RECV.store(1, Ordering::Release);
    let mut buf = [0u8; QUEUE_MSG_SIZE];

    // Step 1: register in the receiver wait queue with a timeout.
    let sz = svc!(
        SYS_QUEUING_RECV_TIMED,
        cmd_port,
        100u32, // timeout: 100 ticks
        buf.as_mut_ptr() as u32
    );

    // Step 2: if blocked (sz == 0), yield to let the scheduler switch to
    // the commander, then poll until the message arrives.
    let sz = if sz == 0 {
        loop {
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
            let rc = svc!(
                SYS_QUEUING_RECV,
                cmd_port,
                QUEUE_MSG_SIZE as u32,
                buf.as_mut_ptr() as u32
            );
            if rc & SVC_ERROR_BIT == 0 {
                break rc;
            }
        }
    } else {
        sz
    };

    if sz & SVC_ERROR_BIT == 0 {
        let rsp = match buf[0] {
            CMD_TIMED => RSP_TIMED_ACK,
            _ => RSP_UNKNOWN,
        };
        svc!(SYS_QUEUING_SEND, rsp_port, 1u32, [rsp].as_ptr() as u32);
        if buf[0] == CMD_TIMED {
            WORKER_TIMED_OK.store(1, Ordering::Release);
        }
    }

    // Done – keep yielding forever.
    #[allow(clippy::empty_loop)]
    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

// ---------------------------------------------------------------------------
// Entry point: create ports, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("queuing_demo: start");

    // Build schedule: each partition runs for 2 ticks per slot.
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    for i in 0..NUM_PARTITIONS as u8 {
        sched.add(ScheduleEntry::new(i, 2)).expect("sched entry");
    }

    // Build partition configs. Stack bases are derived from internal
    // PartitionCore stacks by Kernel::new(), so we use dummy values here.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| PartitionConfig {
        id: i as u8,
        entry_point: 0, // Not used by Kernel::new
        stack_base: 0,  // Ignored: internal stack used
        stack_size: (STACK_WORDS * 4) as u32,
        mpu_region: MpuRegion::new(0, 0, 0), // Base/size overridden by Kernel::new
        peripheral_regions: heapless::Vec::new(),
    });

    // Create the unified kernel with schedule and partitions.
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<DemoConfig>::new(sched, &cfgs).expect("kernel creation");

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

    store_kernel(k);

    // Pack two port IDs into a single u32 passed to each partition via r0:
    //   bits [31:16] = outgoing (Source) port ID
    //   bits [15:0]  = incoming (Destination) port ID
    // Commander: sends on cs, receives on rd.
    // Worker:    sends on rs, receives on cd.
    let hints = [(cs as u32) << 16 | rd as u32, (rs as u32) << 16 | cd as u32];
    let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [commander_main, worker_main];
    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] =
        core::array::from_fn(|i| (eps[i], hints[i]));

    match boot(&parts, &mut p).expect("queuing_demo: boot failed") {}
}
