//! Blackboard Demo — SAME51 Curiosity Nano
//!
//! Shared memory communication via blackboards (last-value semantics).
//!
//! - P0 (Publisher): writes status values to blackboard
//! - P1 (Subscriber A): reads status, tracks changes
//! - P2 (Subscriber B): reads status independently
//!
//! Success: PUB_WRITES > 10 && both subscribers reading.
//!
//! Build: cd same51_curiosity && cargo build --example blackboard_demo --features kernel-example

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
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 3;
const BB_MSG_SIZE: usize = 8;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

const STATE_IDLE: u32 = 0;
const STATE_ACTIVE: u32 = 1;
const STATE_BUSY: u32 = 2;

static PUB_WRITES: AtomicU32 = AtomicU32::new(0);
static PUB_COUNTER: AtomicU32 = AtomicU32::new(0);
static SUB_A_READS: AtomicU32 = AtomicU32::new(0);
static SUB_A_LAST: AtomicU32 = AtomicU32::new(0);
static SUB_B_READS: AtomicU32 = AtomicU32::new(0);
static SUB_B_LAST: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(BlackboardConfig[AlignedStack2K]<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    S = 2; SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = BB_MSG_SIZE; BW = 4;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(BlackboardConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(BlackboardConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let pub_w = PUB_WRITES.load(Ordering::Acquire);
        let pub_c = PUB_COUNTER.load(Ordering::Acquire);
        let sub_a_r = SUB_A_READS.load(Ordering::Acquire);
        let sub_a_l = SUB_A_LAST.load(Ordering::Acquire);
        let sub_b_r = SUB_B_READS.load(Ordering::Acquire);
        let sub_b_l = SUB_B_LAST.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] PUB: writes={:4} cnt={:3} | SUB_A: reads={:4} last={:3} | SUB_B: reads={:4} last={:3}",
            tick, pub_w, pub_c, sub_a_r, sub_a_l, sub_b_r, sub_b_l
        );
        if pub_w > 10 && sub_a_r > 5 && sub_b_r > 5 {
            rprintln!("SUCCESS: Blackboard working!");
        }
    }
});

extern "C" fn publisher_main_body(r0: u32) -> ! {
    let bb_id = r0.into();
    let mut counter: u8 = 0;
    let states = [STATE_IDLE, STATE_ACTIVE, STATE_BUSY];
    let mut state_idx = 0;
    loop {
        counter = counter.wrapping_add(1);
        let state = states[state_idx];
        state_idx = (state_idx + 1) % states.len();
        let buf = [counter, 0, 0, 0, state as u8, 0, 0, 0];
        if plib::sys_bb_display(bb_id, &buf).is_ok() {
            PUB_WRITES.fetch_add(1, Ordering::Release);
            PUB_COUNTER.store(counter as u32, Ordering::Release);
        }
        for _ in 0..20000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn subscriber_a_main_body(r0: u32) -> ! {
    let bb_id = r0.into();
    loop {
        let mut buf = [0u8; BB_MSG_SIZE];
        if let Ok(sz) = plib::sys_bb_read(bb_id, &mut buf) {
            if sz > 0 {
                SUB_A_READS.fetch_add(1, Ordering::Release);
                SUB_A_LAST.store(buf[0] as u32, Ordering::Release);
            }
        }
        for _ in 0..15000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn subscriber_b_main_body(r0: u32) -> ! {
    let bb_id = r0.into();
    loop {
        let mut buf = [0u8; BB_MSG_SIZE];
        if let Ok(sz) = plib::sys_bb_read(bb_id, &mut buf) {
            if sz > 0 {
                SUB_B_READS.fetch_add(1, Ordering::Release);
                SUB_B_LAST.store(buf[0] as u32, Ordering::Release);
            }
        }
        for _ in 0..12000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(publisher_main => publisher_main_body);
kernel::partition_trampoline!(subscriber_a_main => subscriber_a_main_body);
kernel::partition_trampoline!(subscriber_b_main => subscriber_b_main_body);

#[entry]
fn main() -> ! {
    rprintln!("=== Blackboard Demo — SAME51 ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ BlackboardConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched");
    sched.add(ScheduleEntry::new(2, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    let bb: u32 = 0;

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1, ref mut s2] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, publisher_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(bb as u32))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, subscriber_a_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(bb as u32))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
        ExternalPartitionMemory::from_aligned_stack(s2, subscriber_b_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(bb as u32))
            .expect("mem2").with_code_mpu_region(code_mpu).expect("code2"),
    ];

    let mut k = Kernel::<BlackboardConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<BlackboardConfig, _, _>(|k| {
        k.blackboards_mut().create().expect("blackboard");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
