//! QEMU test: P0 sends a 4-byte payload via queuing timed send, P1 receives
//! via queuing timed recv. SysTick hook verifies send-rc==0, recv-rc==4, and
//! payload match.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::{EntryAddr, ExternalPartitionMemory, MpuRegion};
use kernel::sampling::PortDirection;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgSmall, PartitionEntry, Partitions2, PortsTiny, SyncMinimal};

const NP: usize = 2;
const STACK_WORDS: usize = 256;

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;

const PAYLOAD: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);
const NOT_YET: u32 = 0xDEAD_C0DE;

static SEND_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static RECV_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static RECV_DATA: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let s = SEND_RC.load(Ordering::Acquire);
    let r = RECV_RC.load(Ordering::Acquire);

    if s == NOT_YET || r == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_queuing_timed_test: FAIL timeout (s={:#x}, r={:#x})",
                s,
                r
            );
            kernel::kexit!(failure);
        }
        return;
    }

    let data = RECV_DATA.load(Ordering::Acquire);
    if s == 0 && r == 4 && data == EXPECTED_DATA {
        hprintln!(
            "plib_queuing_timed_test: PASS (s={}, r={}, data={:#010x})",
            s,
            r,
            data
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_queuing_timed_test: FAIL (s={:#x}, r={:#x}, data={:#010x} exp {:#010x})",
            s,
            r,
            data,
            EXPECTED_DATA
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let payload = PAYLOAD;
    match plib::sys_queuing_send_timed(plib::QueuingPortId::new(0), &payload, 10) {
        Ok(rc) => SEND_RC.store(rc, Ordering::Release),
        Err(_) => SEND_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    // Yield once so P0 sends before we receive.
    let _ = plib::sys_yield();
    let mut buf = [0u8; 4];
    match plib::sys_queuing_recv_timed(plib::QueuingPortId::new(1), &mut buf, 10) {
        Ok(rc) => {
            RECV_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            RECV_RC.store(rc, Ordering::Release);
        }
        Err(_) => RECV_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_queuing_timed_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks = unsafe { &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>() };
    let [ref mut s0, ref mut s1] = *stacks;
    let mpu = MpuRegion::new(0, 0, 0);
    let e = EntryAddr::from_entry;
    let memories = [
        ExternalPartitionMemory::new(s0, e(p0_main as PartitionEntry), mpu, 0).expect("mem 0"),
        ExternalPartitionMemory::new(s1, e(p1_main as PartitionEntry), mpu, 1).expect("mem 1"),
    ];
    let mut k =
        Kernel::<TestConfig>::new(sched, &memories).expect("plib_queuing_timed_test: kernel");

    let src = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("src port");
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("dst port");
    k.queuing_mut().connect_ports(src, dst).expect("connect");
    store_kernel(&mut k);

    match boot(p).expect("plib_queuing_timed_test: boot") {}
}
