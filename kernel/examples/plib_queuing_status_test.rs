//! QEMU test: send one message to queuing port 0, query status, verify Ok(1).
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
};
use kernel::{
    svc::Kernel, DebugEnabled, MsgSmall, PartitionEntry, Partitions1, PortsSmall, SyncMinimal,
};
kernel::kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgSmall, PortsSmall, DebugEnabled>
);

const NP: usize = 1;
const STACK_WORDS: usize = 256;
const NOT_YET: u32 = 0xDEAD_C0DE;
static SEND_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static STATUS_RC: AtomicU32 = AtomicU32::new(NOT_YET);
kernel::define_kernel!(TestConfig, |tick, _k| {
    let s = SEND_RC.load(Ordering::Acquire);
    let st = STATUS_RC.load(Ordering::Acquire);
    if s == NOT_YET || st == NOT_YET {
        if tick >= 50 {
            kernel::kexit!(failure);
        }
        return;
    }
    if s == 0 && st == 1 {
        hprintln!("plib_queuing_status_test: PASS");
        kernel::kexit!(success);
    }
    hprintln!("plib_queuing_status_test: FAIL ({:#x},{:#x})", s, st);
    kernel::kexit!(failure);
});
const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Copy to stack so pointer is in MPU-accessible partition memory.
    let payload = [0xDE_u8, 0xAD, 0xBE, 0xEF];
    // Send on Source port 0; message is routed to Destination port 1.
    match plib::sys_queuing_send(plib::QueuingPortId::new(0), &payload) {
        Ok(rc) => SEND_RC.store(rc, Ordering::Release),
        Err(_) => SEND_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    // Query status of Destination port 1 to see nb_messages == 1.
    match plib::sys_queuing_status(plib::QueuingPortId::new(1)) {
        Ok(status) => STATUS_RC.store(status.nb_messages, Ordering::Release),
        Err(_) => STATUS_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks = unsafe { &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>() };
    let [ref mut s0] = *stacks;
    let mpu = MpuRegion::new(0, 0, 0);
    let e = EntryAddr::from_entry;
    let memories = [ExternalPartitionMemory::new(
        s0,
        e(p0_main as PartitionEntry),
        mpu,
        kernel::PartitionId::new(0),
    )
    .expect("mem 0")];
    let mut k = Kernel::<TestConfig>::new(sched, &memories).expect("kernel");
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
    match boot(p).expect("boot") {}
}
