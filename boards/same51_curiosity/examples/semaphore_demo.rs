//! Semaphore Demo — SAME51 Curiosity Nano
//!
//! Kernel IPC: counting semaphore producer/consumer.
//! Identical logic to the nRF52/f429zi semaphore_demo.
//!
//! - P0 (producer): waits for slot via SYS_SEM_WAIT, produces item
//! - P1 (consumer): consumes item, signals slot via SYS_SEM_SIGNAL
//!
//! Success: PROD_SUCCESS > 10 && CONS_SUCCESS > 10, printed via RTT.
//!
//! Build: cd same51_curiosity && cargo build --example semaphore_demo --features kernel-example

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
    semaphore::Semaphore,
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const BUFFER_SIZE: u32 = 3;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

static PROD_SUCCESS: AtomicU32 = AtomicU32::new(0);
static PROD_COUNTER: AtomicU32 = AtomicU32::new(0);
static CONS_SUCCESS: AtomicU32 = AtomicU32::new(0);
static CONS_LAST: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(SemConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    S = 2; SW = 4; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(SemConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(SemConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let ps = PROD_SUCCESS.load(Ordering::Acquire);
        let pc = PROD_COUNTER.load(Ordering::Acquire);
        let cs = CONS_SUCCESS.load(Ordering::Acquire);
        let cl = CONS_LAST.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] PROD ok={} cnt={} | CONS ok={} last={}",
            tick, ps, pc, cs, cl
        );
        if ps > 10 && cs > 10 {
            rprintln!("SUCCESS: Semaphore working! PROD={} CONS={}", ps, cs);
        }
    }
});

extern "C" fn producer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();
    let mut counter: u8 = 0;
    loop {
        if plib::sys_sem_wait(sem_id).is_ok() {
            counter = counter.wrapping_add(1);
            PROD_SUCCESS.fetch_add(1, Ordering::Release);
            PROD_COUNTER.store(counter as u32, Ordering::Release);
            for _ in 0..10_000 { core::hint::spin_loop(); }
        }
        for _ in 0..15_000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

extern "C" fn consumer_main_body(r0: u32) -> ! {
    let sem_id = r0.into();
    loop {
        let value = PROD_COUNTER.load(Ordering::Acquire);
        if value > 0 {
            CONS_SUCCESS.fetch_add(1, Ordering::Release);
            CONS_LAST.store(value, Ordering::Release);
            plib::sys_sem_signal(sem_id).ok();
            for _ in 0..8_000 { core::hint::spin_loop(); }
        }
        for _ in 0..12_000 { core::hint::spin_loop(); }
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_main_body);
kernel::partition_trampoline!(consumer_main => consumer_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Semaphore Demo — SAME51 ===");
    rprintln!("P0=producer  P1=consumer  semaphore buffer_size={}", BUFFER_SIZE);

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ SemConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, producer_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, consumer_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
    ];

    let mut k = Kernel::<SemConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    kernel::state::with_kernel_mut::<SemConfig, _, _>(|k| {
        k.semaphores_mut()
            .add(Semaphore::new(BUFFER_SIZE, BUFFER_SIZE))
            .expect("semaphore");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("Booting...\n");
    match boot(p).expect("boot") {}
}
