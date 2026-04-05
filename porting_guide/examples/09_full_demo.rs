//! Three-partition IPC demo: scheduling + sampling + queuing + semaphores.
#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::sampling::PortDirection;
use kernel::semaphore::Semaphore;
use kernel::{
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgRich, PartitionEntry, PartitionSpec, Partitions3, PortsStandard, SyncStandard,
};
use porting_guide::klog;

kernel::kernel_config!(
    Cfg < Partitions3,
    SyncStandard,
    MsgRich,
    PortsStandard,
    DebugEnabled > {}
);

const GOAL: u32 = 10;
const TIMEOUT: u32 = 2000;
static P1_RECV: AtomicU32 = AtomicU32::new(0);
static P2_STATUS: AtomicU32 = AtomicU32::new(0);
static ERROR: AtomicU32 = AtomicU32::new(0);
fn halt_with(code: u32) -> ! {
    ERROR.store(code, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

kernel::define_harness!(Cfg, |tick, _k| {
    let err = ERROR.load(Ordering::Acquire);
    if err != 0 {
        klog!("09_full_demo: FAIL error ({:#x})", err);
        kernel::kexit!(failure);
    }
    let r = P1_RECV.load(Ordering::Acquire);
    let s = P2_STATUS.load(Ordering::Acquire);
    if r >= GOAL && s >= GOAL {
        klog!("09_full_demo: PASS (r={}, s={})", r, s);
        kernel::kexit!(success);
    }
    if tick >= TIMEOUT {
        klog!("09_full_demo: FAIL timeout");
        kernel::kexit!(failure);
    }
    let _ = (r, s);
});

const _: PartitionEntry = producer;
const _: PartitionEntry = processor;
const _: PartitionEntry = monitor;

extern "C" fn producer() -> ! {
    let sem = plib::SemaphoreId::new(0);
    let qport = plib::QueuingPortId::new(0);
    let mut seq: u32 = 1;
    loop {
        if plib::sys_queuing_send(qport, &seq.to_le_bytes()).is_err() {
            halt_with(0xA001_0001);
        }
        if plib::sys_sem_signal(sem).is_err() {
            halt_with(0xA002_0001);
        }
        seq += 1;
        if plib::sys_yield().is_err() {
            halt_with(0xA0FF_0001);
        }
    }
}

extern "C" fn processor() -> ! {
    let sem = plib::SemaphoreId::new(0);
    let qport = plib::QueuingPortId::new(1);
    let samp = plib::SamplingPortId::new(0);
    let mut expected: u32 = 1;
    let mut buf = [0u8; 64];
    loop {
        if plib::sys_sem_wait(sem).is_err() {
            halt_with(0xB000_0001);
        }
        let len = match plib::sys_queuing_recv(qport, &mut buf) {
            Ok(l) => l,
            Err(_) => halt_with(0xB001_0001),
        };
        if len < 4 {
            halt_with(0xB001_0003);
        }
        let seq = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if seq != expected {
            halt_with(0xB001_0002);
        }
        expected += 1;
        P1_RECV.store(seq, Ordering::Release);
        let status = [0x01u8];
        if plib::sys_sampling_write(samp, &status).is_err() {
            halt_with(0xB002_0001);
        }
        if plib::sys_yield().is_err() {
            halt_with(0xB0FF_0001);
        }
    }
}

extern "C" fn monitor() -> ! {
    let samp = plib::SamplingPortId::new(1);
    let mut n: u32 = 0;
    loop {
        let mut buf = [0u8; 4];
        match plib::sys_sampling_read(samp, &mut buf) {
            Ok(sz) => {
                if sz > 0 && buf[0] == 0x01 {
                    n += 1;
                    P2_STATUS.store(n, Ordering::Release);
                }
            }
            Err(_) => halt_with(0xC001_0001),
        }
        if plib::sys_yield().is_err() {
            halt_with(0xC0FF_0001);
        }
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("09_full_demo: three-partition IPC demo");
    let mut sched: ScheduleTable<{ Cfg::SCHED }> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 4)).expect("sched P1");
    sched.add_system_window(1).expect("syswin 1");
    sched.add(ScheduleEntry::new(2, 4)).expect("sched P2");
    sched.add_system_window(1).expect("syswin 2");
    let entries: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(producer as PartitionEntry, 0),
        PartitionSpec::new(processor as PartitionEntry, 0),
        PartitionSpec::new(monitor as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &entries).expect("init_kernel");
    k.semaphores_mut().add(Semaphore::new(0, 0)).expect("sem");
    let qs = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("qs");
    let qd = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("qd");
    k.queuing_mut().connect_ports(qs, qd).expect("qc");
    let ss = k
        .sampling_mut()
        .create_port(PortDirection::Source, 10)
        .expect("ss");
    let sd = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 10)
        .expect("sd");
    k.sampling_mut().connect_ports(ss, sd).expect("sc");
    store_kernel(&mut k);
    match boot(p).expect("09_full_demo: boot") {}
}
