#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(target_arch = "arm", no_main)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#[cfg(not(target_arch = "arm"))]
fn main() {}

#[cfg(target_arch = "arm")]
#[rustfmt::skip]
mod hw {
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::exception;
use kernel::{
    boot,
    kexit, klog,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgSmall, Partitions2, PortsTiny, SyncMinimal,
};
#[allow(unused_imports)]
use kernel::kpanic as _;
kernel::compose_kernel_config!(Cfg<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>);
const SW: usize = Cfg::STACK_WORDS;
#[repr(C, align(4096))]
struct PartitionStacks([[u32; SW]; Cfg::N]);
static mut PARTITION_STACKS: PartitionStacks = PartitionStacks([[0u32; SW]; Cfg::N]);
static P0_SENT: AtomicU32 = AtomicU32::new(0);
static P1_RECV_OK: AtomicU32 = AtomicU32::new(0);
static PARTS_RAN: AtomicU32 = AtomicU32::new(0);
kernel::define_unified_harness!(no_boot, Cfg, |tick, k| {
    let addr = k as *const _ as usize;
    if (addr & (kernel::state::KERNEL_ALIGNMENT - 1)) != 0 { kexit!(failure); }
    if tick.is_multiple_of(5) {
        let s = P0_SENT.load(Ordering::Acquire);
        let r = P1_RECV_OK.load(Ordering::Acquire);
        let p = PARTS_RAN.load(Ordering::Acquire);
        klog!("[t{}] s={} r={} p={:02b}", tick, s, r, p);
        if s == 1 && r == 1 && p == 0b11 { klog!("PASS"); kexit!(success); }
        if tick > 100 { klog!("FAIL: timeout"); kexit!(failure); }
    }
});
extern "C" fn p0_main_body(r0: u32) -> ! {
    PARTS_RAN.fetch_or(1, Ordering::Release);
    let (port, msg) = (plib::QueuingPortId::new(r0 >> 16), [0xDE_u8, 0xAD, 0xBE, 0xEF]);
    if plib::sys_queuing_send(port, &msg).is_ok() { P0_SENT.store(1, Ordering::Release); }
    loop { plib::sys_yield().expect("sys_yield"); }
}
kernel::partition_trampoline!(p0_main => p0_main_body);
extern "C" fn p1_main_body(r0: u32) -> ! {
    PARTS_RAN.fetch_or(2, Ordering::Release);
    let port = plib::QueuingPortId::new(r0 & 0xFFFF);
    loop {
        let mut buf = [0u8; 4];
        if plib::sys_queuing_recv(port, &mut buf).is_ok() && buf == [0xDE, 0xAD, 0xBE, 0xEF] {
            P1_RECV_OK.store(1, Ordering::Release); break;
        }
        plib::sys_yield().expect("sys_yield");
    }
    loop { plib::sys_yield().expect("sys_yield"); }
}
kernel::partition_trampoline!(p1_main => p1_main_body);
#[cortex_m_rt::entry]
#[allow(clippy::never_loop)] // kexit! diverges via inner loop on catch-all backend
fn main() -> ! {
    let mut p = match cortex_m::Peripherals::take() {
        Some(p) => p,
        None => loop { kexit!(failure); },
    };
    klog!("hw_integration: start");
    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 2)).is_err() { loop { kexit!(failure); } }
    #[cfg(feature = "dynamic-mpu")]
    if sched.add_system_window(1).is_err() { loop { kexit!(failure); } }
    if sched.add(ScheduleEntry::new(1, 2)).is_err() { loop { kexit!(failure); } }
    #[cfg(feature = "dynamic-mpu")]
    if sched.add_system_window(1).is_err() { loop { kexit!(failure); } }
    let mut k = match Kernel::<Cfg>::create_sentinels(sched) {
        Ok(k) => k,
        Err(_) => loop { kexit!(failure); },
    };
    let src = match k.queuing_mut().create_port(PortDirection::Source) {
        Ok(id) => id,
        Err(_) => loop { kexit!(failure); },
    };
    let dst = match k.queuing_mut().create_port(PortDirection::Destination) {
        Ok(id) => id,
        Err(_) => loop { kexit!(failure); },
    };
    if k.queuing_mut().connect_ports(src, dst).is_err() { loop { kexit!(failure); } }
    store_kernel(k);
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks: &mut [[u32; SW]; Cfg::N] =
        unsafe { &mut *(&raw mut PARTITION_STACKS).cast() };
    let parts: [(extern "C" fn() -> !, u32); Cfg::N] =
        [(p0_main, (src as u32) << 16), (p1_main, dst as u32)];
    match boot::boot_external::<Cfg, SW>(&parts, &mut p, stacks) {
        Ok(never) => match never {},
        Err(_) => loop { kexit!(failure); },
    }
}
}
