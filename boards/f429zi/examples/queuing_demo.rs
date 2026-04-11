//! Queuing Port Demo for STM32F429ZI
//!
//! Demonstrates FIFO message queues between partitions:
//! - Commander partition sends commands via queuing port
//! - Worker partition receives commands, sends responses
//! - Shows queue full/empty handling
//! - Demonstrates command/response pattern
//!
//! Two partitions:
//! - P0 (Commander): Sends commands, receives responses
//! - P1 (Worker): Receives commands, processes, sends responses

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
// panic-halt is brought in unconditionally by the kernel crate — do NOT also
// use panic_rtt_target here, it would cause a duplicate panic handler link error.
use plib::QueuingPortId;
use rtt_target::{rprintln, rtt_init_print};

const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
const NUM_PARTITIONS: usize = 2;

// Command protocol
const CMD_START: u8 = 1;
const CMD_PROCESS: u8 = 2;
const CMD_STOP: u8 = 3;

const RSP_START_ACK: u8 = 0x10;
const RSP_PROCESS_ACK: u8 = 0x20;
const RSP_STOP_ACK: u8 = 0x30;


// Progress tracking
static CMD_SENT: AtomicU32 = AtomicU32::new(0);
static CMD_RECV: AtomicU32 = AtomicU32::new(0);
static RSP_SENT: AtomicU32 = AtomicU32::new(0);
static RSP_RECV: AtomicU32 = AtomicU32::new(0);
static LAST_CMD: AtomicU32 = AtomicU32::new(0);
static LAST_RSP: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(QueuingConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 2;
    QS = 4; QD = QUEUE_DEPTH; QM = QUEUE_MSG_SIZE; QW = 4;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(QueuingConfig, |tick, _k| {
    if tick % 500 == 0 {
        let cmd_s = CMD_SENT.load(Ordering::Acquire);
        let cmd_r = CMD_RECV.load(Ordering::Acquire);
        let rsp_s = RSP_SENT.load(Ordering::Acquire);
        let rsp_r = RSP_RECV.load(Ordering::Acquire);
        let last_cmd = LAST_CMD.load(Ordering::Acquire);
        let last_rsp = LAST_RSP.load(Ordering::Acquire);

        rprintln!(
            "[{:4}ms] CMD: sent={:3} recv={:3} (last={:02x}) | RSP: sent={:3} recv={:3} (last={:02x})",
            tick, cmd_s, cmd_r, last_cmd, rsp_s, rsp_r, last_rsp
        );

        if cmd_r > 10 && rsp_r > 10 {
            rprintln!("✓ SUCCESS: Queuing ports working! Commands and responses flowing.");
        }
    }
});

/// Commander partition - sends commands, receives responses
extern "C" fn commander_main_body(r0: u32) -> ! {
    let ports = r0;
    let cmd_port = QueuingPortId::new((ports >> 16) as u32);  // Send commands
    let rsp_port = QueuingPortId::new((ports & 0xFFFF) as u32); // Receive responses

    let commands = [CMD_START, CMD_PROCESS, CMD_STOP];
    let mut cmd_idx = 0;

    loop {
        // Send next command
        let cmd = commands[cmd_idx];
        let msg = [cmd, 0, 0, 0];

        if plib::sys_queuing_send(cmd_port, &msg).is_ok() {
            CMD_SENT.fetch_add(1, Ordering::Release);
            LAST_CMD.store(cmd as u32, Ordering::Release);
            cmd_idx = (cmd_idx + 1) % commands.len();
        }

        // Try to receive response
        let mut rsp = [0u8; 4];
        if plib::sys_queuing_recv(rsp_port, &mut rsp).is_ok() {
            RSP_RECV.fetch_add(1, Ordering::Release);
            LAST_RSP.store(rsp[0] as u32, Ordering::Release);
        }

        // Delay
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

/// Worker partition - receives commands, sends responses
extern "C" fn worker_main_body(r0: u32) -> ! {
    let ports = r0;
    let cmd_port = QueuingPortId::new((ports >> 16) as u32);  // Receive commands
    let rsp_port = QueuingPortId::new((ports & 0xFFFF) as u32); // Send responses

    loop {
        // Receive command
        let mut cmd = [0u8; 4];
        if plib::sys_queuing_recv(cmd_port, &mut cmd).is_ok() {
            CMD_RECV.fetch_add(1, Ordering::Release);

            // Process command and generate response
            let response = match cmd[0] {
                CMD_START => RSP_START_ACK,
                CMD_PROCESS => RSP_PROCESS_ACK,
                CMD_STOP => RSP_STOP_ACK,
                _ => 0xFF,
            };

            // Send response
            let rsp_msg = [response, 0, 0, 0];
            if plib::sys_queuing_send(rsp_port, &rsp_msg).is_ok() {
                RSP_SENT.fetch_add(1, Ordering::Release);
            }
        }

        // Small delay
        for _ in 0..5000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(commander_main => commander_main_body);
kernel::partition_trampoline!(worker_main => worker_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Queuing Port Demo ===");
    rprintln!("STM32F429ZI - Command/Response Pattern");

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: Equal time slices
    let mut sched = ScheduleTable::<{ QueuingConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");
    rprintln!("Schedule: P0(Commander) P1(Worker)");

    // Port IDs are deterministic: created in order → cmd_src=0, cmd_dst=1, rsp_src=2, rsp_dst=3
    // Pack port IDs for each partition
    let cmd_src: u32 = 0;
    let cmd_dst: u32 = 1;
    let rsp_src: u32 = 2;
    let rsp_dst: u32 = 3;

    let h: [u32; NUM_PARTITIONS] = [
        (cmd_src << 16) | rsp_dst,  // P0: send cmd, recv rsp
        (cmd_dst << 16) | rsp_src,  // P1: recv cmd, send rsp
    ];

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(commander_main as kernel::PartitionEntry, h[0]),
        PartitionSpec::new(worker_main as kernel::PartitionEntry, h[1]),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("Kernel created");

    // Create queuing ports
    kernel::state::with_kernel_mut::<QueuingConfig, _, _>(|k| {
        // Command queue: P0 -> P1
        let cs = k.queuing_mut().create_port(PortDirection::Source).expect("cmd src");
        let cd = k.queuing_mut().create_port(PortDirection::Destination).expect("cmd dst");
        k.queuing_mut().connect_ports(cs, cd).expect("connect cmd");

        // Response queue: P1 -> P0
        let rs = k.queuing_mut().create_port(PortDirection::Source).expect("rsp src");
        let rd = k.queuing_mut().create_port(PortDirection::Destination).expect("rsp dst");
        k.queuing_mut().connect_ports(rs, rd).expect("connect rsp");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Queuing ports created:");
    rprintln!("  CMD: {} -> {}", cmd_src, cmd_dst);
    rprintln!("  RSP: {} -> {}", rsp_src, rsp_dst);

    rprintln!("Booting...\n");

    match boot(p).expect("boot") {}
}
