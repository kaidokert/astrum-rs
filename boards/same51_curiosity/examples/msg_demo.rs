//! Message Queue Demo — SAME51 Curiosity Nano
//!
//! - P0 (sender): sends increasing counter via SYS_MSG_SEND
//! - P1 (receiver): receives messages, tracks count
//!
//! Success: SENT > 10 and RECV > 10.
//!
//! Build: cd same51_curiosity && cargo build --example msg_demo --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    message::MessageQueue,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const MSG_SIZE: usize = 4;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

static SENT: AtomicU32 = AtomicU32::new(0);
static RECV: AtomicU32 = AtomicU32::new(0);
static LAST_SENT: AtomicU32 = AtomicU32::new(0);
static LAST_RECV: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(MsgConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(MsgConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(MsgConfig, |tick, _k| {
    if tick % 500 == 0 {
        let s = SENT.load(Ordering::Acquire);
        let r = RECV.load(Ordering::Acquire);
        let ls = LAST_SENT.load(Ordering::Acquire);
        let lr = LAST_RECV.load(Ordering::Acquire);
        rprintln!(
            "[{:4}ms] SENT={} (last_val={}) RECV={} (last_val={})",
            tick, s, ls, r, lr
        );
        if r > 10 && s > 10 {
            rprintln!("SUCCESS: Message queues working! SENT={} RECV={}", s, r);
        }
    }
});

extern "C" fn sender_main() -> ! {
    let target_pid = 0u32.into();
    let mut counter: u32 = 1;
    loop {
        let msg = counter.to_le_bytes();
        if plib::sys_msg_send(target_pid, &msg).is_ok() {
            SENT.fetch_add(1, Ordering::Release);
            LAST_SENT.store(counter, Ordering::Release);
            counter = counter.wrapping_add(1);
        }
        for _ in 0..5000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn receiver_main() -> ! {
    let mut buf = [0u8; MSG_SIZE];
    loop {
        if plib::sys_msg_recv(&mut buf).is_ok() {
            let val = u32::from_le_bytes(buf);
            if val > 0 {
                RECV.fetch_add(1, Ordering::Release);
                LAST_RECV.store(val, Ordering::Release);
                buf = [0u8; MSG_SIZE];
            }
        }
        for _ in 0..5000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

#[entry]
fn main() -> ! {
    rprintln!("\n=== Message Queue Demo — SAME51 ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ MsgConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, sender_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, receiver_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
    ];

    let mut k = Kernel::<MsgConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<MsgConfig, _, _>(|k| {
        k.messages_mut().add(MessageQueue::new()).expect("add queue");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("[INIT] Booting: P0=sender P1=receiver\n");
    match boot(p).expect("boot") {}
}
