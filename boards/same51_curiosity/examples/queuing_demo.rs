//! Queuing Port Demo — SAME51 Curiosity Nano
//!
//! Command/response pattern via queuing ports.
//!
//! - P0 (Commander): sends commands, receives responses
//! - P1 (Worker): receives commands, sends responses
//!
//! Success: CMD_RECV > 10 && RSP_RECV > 10.
//!
//! Build: cd same51_curiosity && cargo build --example queuing_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

const CMD_START: u8 = 1;
const CMD_PROCESS: u8 = 2;
const CMD_STOP: u8 = 3;
const RSP_START_ACK: u8 = 0x10;
const RSP_PROCESS_ACK: u8 = 0x20;
const RSP_STOP_ACK: u8 = 0x30;

static CMD_SENT: AtomicU32 = AtomicU32::new(0);
static CMD_RECV: AtomicU32 = AtomicU32::new(0);
static RSP_SENT: AtomicU32 = AtomicU32::new(0);
static RSP_RECV: AtomicU32 = AtomicU32::new(0);
static LAST_CMD: AtomicU32 = AtomicU32::new(0);
static LAST_RSP: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(QueuingConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 2;
    QS = 4; QD = QUEUE_DEPTH; QM = QUEUE_MSG_SIZE; QW = 4;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(QueuingConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

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
            rprintln!("SUCCESS: Queuing ports working!");
        }
    }
});

extern "C" fn commander_main_body(r0: u32) -> ! {
    let ports = r0;
    let cmd_port = (ports >> 16).into();
    let rsp_port = (ports & 0xFFFF).into();
    let commands = [CMD_START, CMD_PROCESS, CMD_STOP];
    let mut cmd_idx = 0;
    loop {
        let cmd = commands[cmd_idx];
        let msg = [cmd, 0, 0, 0];
        if plib::sys_queuing_send(cmd_port, &msg).is_ok() {
            CMD_SENT.fetch_add(1, Ordering::Release);
            LAST_CMD.store(cmd as u32, Ordering::Release);
            cmd_idx = (cmd_idx + 1) % commands.len();
        }
        let mut rsp = [0u8; 4];
        if plib::sys_queuing_recv(rsp_port, &mut rsp).is_ok() {
            RSP_RECV.fetch_add(1, Ordering::Release);
            LAST_RSP.store(rsp[0] as u32, Ordering::Release);
        }
        for _ in 0..10000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn worker_main_body(r0: u32) -> ! {
    let ports = r0;
    let cmd_port = (ports >> 16).into();
    let rsp_port = (ports & 0xFFFF).into();
    loop {
        let mut cmd = [0u8; 4];
        if plib::sys_queuing_recv(cmd_port, &mut cmd).is_ok() {
            CMD_RECV.fetch_add(1, Ordering::Release);
            let response = match cmd[0] {
                CMD_START => RSP_START_ACK,
                CMD_PROCESS => RSP_PROCESS_ACK,
                CMD_STOP => RSP_STOP_ACK,
                _ => 0xFF,
            };
            let rsp_msg = [response, 0, 0, 0];
            if plib::sys_queuing_send(rsp_port, &rsp_msg).is_ok() {
                RSP_SENT.fetch_add(1, Ordering::Release);
            }
        }
        for _ in 0..5000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(commander_main => commander_main_body);
kernel::partition_trampoline!(worker_main => worker_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Queuing Port Demo — SAME51 ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ QueuingConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    let cmd_src: u32 = 0;
    let cmd_dst: u32 = 1;
    let rsp_src: u32 = 2;
    let rsp_dst: u32 = 3;

    let h: [u32; NUM_PARTITIONS] = [
        (cmd_src << 16) | rsp_dst,
        (cmd_dst << 16) | rsp_src,
    ];

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, commander_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(h[0] as u32))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, worker_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(h[1] as u32))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
    ];

    let mut k = Kernel::<QueuingConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<QueuingConfig, _, _>(|k| {
        let cs = k.queuing_mut().create_port(PortDirection::Source).expect("cmd src");
        let cd = k.queuing_mut().create_port(PortDirection::Destination).expect("cmd dst");
        k.queuing_mut().connect_ports(cs, cd).expect("connect cmd");
        let rs = k.queuing_mut().create_port(PortDirection::Source).expect("rsp src");
        let rd = k.queuing_mut().create_port(PortDirection::Destination).expect("rsp dst");
        k.queuing_mut().connect_ports(rs, rd).expect("connect rsp");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
