//! Events Demo — SAME51 Curiosity Nano
//!
//! Inter-partition event signaling: ping-pong pattern.
//!
//! - P0 (setter): sets event bit 0x1 on P1, then yields
//! - P1 (waiter): waits for event bit 0x1, verifies, yields
//!
//! Success: RECV > 20.
//!
//! Build: cd same51_curiosity && cargo build --example events_demo --features kernel-example

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
    {Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use plib::{self, EventMask, PartitionId};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

static RECV_COUNT: AtomicU32 = AtomicU32::new(0);

const P1_PID: PartitionId = PartitionId::new(1);

kernel::kernel_config!(EventConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(EventConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(EventConfig, |tick, k| {
    if tick % 500 == 0 {
        let p0_state = k.partitions().get(0).map(|p| p.state());
        let p1_state = k.partitions().get(1).map(|p| p.state());
        let p1_flags = k.partitions().get(1).map(|p| p.event_flags());
        let recvs = RECV_COUNT.load(Ordering::Acquire);
        rprintln!(
            "[KERNEL] tick={} P0={:?} P1={:?} P1.flags={:#010b} RECV={}",
            tick, p0_state, p1_state, p1_flags.unwrap_or(0), recvs
        );
        if recvs > 20 {
            rprintln!("SUCCESS: Events working! RECV={}", recvs);
        }
    }
});

extern "C" fn setter_main_body(r0: u32) -> ! {
    let target_pid = r0.into();
    let event_bit = EventMask::new(0x1);
    loop {
        plib::sys_event_set(target_pid, event_bit).ok();
        for _ in 0..3000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn waiter_main_body(_r0: u32) -> ! {
    let event_bit = EventMask::new(0x1);
    loop {
        match plib::sys_event_wait(event_bit) {
            Err(_) => continue,
            Ok(bits) if bits.as_raw() == 0 => continue,
            Ok(bits) => {
                if (bits & event_bit).as_raw() != 0 {
                    RECV_COUNT.fetch_add(1, Ordering::Release);
                }
            }
        }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(setter_main => setter_main_body);
kernel::partition_trampoline!(waiter_main => waiter_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Events Demo — SAME51 ===");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ EventConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, setter_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, waiter_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
    ];

    let mut k = Kernel::<EventConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting: P0=setter P1=waiter event_bit=0x1\n");
    match boot(p).expect("boot") {}
}
