//! 2-partition command/response pipeline via paired queuing ports.
//!
//! Demonstrates: FIFO message delivery, command→response round-trips,
//! and **queue-full detection** (the commander sends more messages than
//! the queue depth and verifies that the overflow is reported).
// TODO: The per-example boilerplate (statics, svc macro, unpack_r0, SysTick,
// partition/scheduler setup) is duplicated across all QEMU examples. Extract
// a shared `qemu_harness` module or proc-macro crate to eliminate this when
// the kernel's example infrastructure matures.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, SCB};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    context::init_stack_frame,
    kernel::KernelState,
    partition::PartitionConfig,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::{SYS_QUEUING_RECV, SYS_QUEUING_SEND, SYS_YIELD},
};
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Kernel sizing constants and config
// ---------------------------------------------------------------------------
const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
const MAX_SCHEDULE_ENTRIES: usize = 8;
const NUM_PARTITIONS: usize = 2;

/// Kernel configuration for the queuing-port demo.
///
/// Sized for 4 partitions, depth-4 queuing ports with 4-byte messages,
/// and moderate pool sizes for all resource types.
struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
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
}

// ---------------------------------------------------------------------------
// Command / response protocol constants
// ---------------------------------------------------------------------------
const CMD_START: u8 = 1;
const CMD_MEASURE: u8 = 2;
const CMD_STOP: u8 = 3;
const CMD_EXTRA_1: u8 = 4; // overflow probe: exceeds QUEUE_DEPTH
const CMD_EXTRA_2: u8 = 5; // overflow probe: exceeds QUEUE_DEPTH

const RSP_START_ACK: u8 = 0x10;
const RSP_MEASURE_ACK: u8 = 0x20;
const RSP_STOP_ACK: u8 = 0x30;
const RSP_EXTRA_1_ACK: u8 = 0x40;
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
// Boilerplate: kernel statics, SVC dispatch hook, asm macros
// ---------------------------------------------------------------------------
static mut STACKS: [[u32; 256]; NUM_PARTITIONS] = [[0; 256]; NUM_PARTITIONS];
#[no_mangle]
static mut PARTITION_SP: [u32; NUM_PARTITIONS] = [0; NUM_PARTITIONS];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;
static mut KERN: Option<Kernel<DemoConfig>> = None;
static mut KS: Option<KernelState<{ DemoConfig::N }, MAX_SCHEDULE_ENTRIES>> = None;
#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;
kernel::define_pendsv!();
unsafe extern "C" fn hook(f: &mut kernel::context::ExceptionFrame) {
    let p = &raw mut KERN;
    if let Some(k) = unsafe { (*p).as_mut() } {
        unsafe { k.dispatch(f) }
    }
}
macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{ let r: u32;
        #[cfg(target_arch = "arm")]
        unsafe { core::arch::asm!("svc #0", inout("r0") $id => r,
            in("r1") $a, in("r2") $b, in("r3") $c, out("r12") _) }
        #[cfg(not(target_arch = "arm"))] { let _ = ($id, $a, $b, $c); r = 0; } r }};
}

/// Read the value the kernel placed in r0 before entering this partition.
///
/// Port IDs are packed into a single u32 passed via r0 at partition entry:
///   bits [31:16] = "outgoing" port (the Source port this partition writes to)
///   bits [15:0]  = "incoming" port (the Destination port this partition reads from)
///
/// This avoids needing shared memory or an extra syscall for port discovery.
macro_rules! unpack_r0 {
    () => {{ let p: u32; #[cfg(target_arch = "arm")] unsafe { core::arch::asm!("", out("r0") p) }
        #[cfg(not(target_arch = "arm"))] { p = 0; } p }};
}

// ---------------------------------------------------------------------------
// Commander partition: sends commands, detects queue-full, receives responses
// ---------------------------------------------------------------------------
extern "C" fn commander_main() -> ! {
    // Unpack port IDs: upper 16 bits = command Source port, lower 16 = response Destination port.
    let packed = unpack_r0!();
    let (cmd_port, rsp_port) = (packed >> 16, packed & 0xFFFF);

    // Phase 1: Flood the command queue to demonstrate queue-full detection.
    // QUEUE_DEPTH is 4, so the 5th send must fail with a kernel error.
    let mut queue_full_seen = false;
    let mut delivered: usize = 0;
    for &cmd in &CMDS {
        let rc = svc!(SYS_QUEUING_SEND, cmd_port, 1u32, [cmd].as_ptr() as u32);
        if rc & SVC_ERROR_BIT != 0 {
            hprintln!("[commander] send cmd={} -> QUEUE FULL (expected)", cmd);
            queue_full_seen = true;
        } else {
            delivered += 1;
            hprintln!("[commander] sent cmd={} depth={}", cmd, delivered);
        }
    }
    if !queue_full_seen {
        hprintln!("queuing_demo: FAIL – queue-full was never detected");
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!(
        "[commander] queue-full correctly detected, {} delivered",
        delivered
    );

    // Yield to let the worker drain and respond.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // Phase 2: Collect responses for the commands that were successfully delivered.
    let mut n: usize = 0;
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
        let (got, exp) = (buf[0], EXPECTED_RSPS[n]);
        hprintln!(
            "[commander] recv rsp=0x{:02X} (expected 0x{:02X})",
            got,
            exp
        );
        if got != exp {
            hprintln!("queuing_demo: FAIL at response {}", n);
            debug::exit(debug::EXIT_FAILURE);
        }
        n += 1;
        if n >= delivered {
            break;
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
    hprintln!("queuing_demo: all checks passed");
    debug::exit(debug::EXIT_SUCCESS);
    #[allow(clippy::empty_loop)]
    loop {}
}

// ---------------------------------------------------------------------------
// Worker partition: receives commands, maps to responses, sends back
// ---------------------------------------------------------------------------
extern "C" fn worker_main() -> ! {
    // Unpack port IDs: upper 16 bits = response Source port, lower 16 = command Destination port.
    let packed = unpack_r0!();
    let (rsp_port, cmd_port) = (packed >> 16, packed & 0xFFFF);
    loop {
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
        hprintln!("[worker]    cmd={} -> rsp=0x{:02X}", buf[0], rsp);
        svc!(SYS_QUEUING_SEND, rsp_port, 1u32, [rsp].as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

// ---------------------------------------------------------------------------
// SysTick handler: drives the round-robin scheduler
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    let p = &raw mut KS;
    if let Some(pid) = unsafe { (*p).as_mut() }
        .expect("KS")
        .advance_schedule_tick()
    {
        unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
        SCB::set_pendsv();
    }
}

// ---------------------------------------------------------------------------
// Entry point: create ports, configure partitions and scheduler, start OS
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("queuing_demo: start");
    unsafe {
        let mut k = Kernel::<DemoConfig>::new();

        // Command channel: commander (Source cs) -> worker (Destination cd)
        let cs = k.queuing.create_port(PortDirection::Source).unwrap();
        let cd = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing.connect_ports(cs, cd).unwrap();

        // Response channel: worker (Source rs) -> commander (Destination rd)
        let rs = k.queuing.create_port(PortDirection::Source).unwrap();
        let rd = k.queuing.create_port(PortDirection::Destination).unwrap();
        k.queuing.connect_ports(rs, rd).unwrap();

        KERN = Some(k);
        kernel::svc::SVC_DISPATCH_HOOK = Some(hook);

        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        for i in 0..NUM_PARTITIONS as u8 {
            sched.add(ScheduleEntry::new(i, 2)).unwrap();
        }
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
        KS = Some(KernelState::new(sched, &cfgs).unwrap());

        // Pack two port IDs into a single u32 passed to each partition via r0:
        //   bits [31:16] = outgoing (Source) port ID
        //   bits [15:0]  = incoming (Destination) port ID
        // Commander: sends on cs, receives on rd.
        // Worker:    sends on rs, receives on cd.
        let hints = [(cs as u32) << 16 | rd as u32, (rs as u32) << 16 | cd as u32];
        let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [commander_main, worker_main];
        let (stk, sp) = (&raw mut STACKS, &raw mut PARTITION_SP);
        for (i, (ep, &hv)) in eps.iter().zip(hints.iter()).enumerate() {
            let ix = init_stack_frame(&mut (*stk)[i], *ep as *const () as u32, Some(hv)).unwrap();
            (*sp)[i] = (*stk)[i].as_ptr() as u32 + (ix as u32) * 4;
        }

        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(120_000 - 1);
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi()
    }
}
