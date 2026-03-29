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
    partition::{ExternalPartitionMemory, MpuRegion},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    AlignedStack1K, DebugEnabled, MsgSmall, PartitionEntry, Partitions2, PortsTiny,
    StackStorage as _, SyncMinimal,
};
#[allow(unused_imports)]
use kernel::kpanic as _;
kernel::compose_kernel_config!(Cfg<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>);
static mut STACKS: [AlignedStack1K; Cfg::N] = [AlignedStack1K::ZERO; Cfg::N];
static P0_SENT: AtomicU32 = AtomicU32::new(0);
static P1_RECV_OK: AtomicU32 = AtomicU32::new(0);
static PARTS_RAN: AtomicU32 = AtomicU32::new(0);
static SRC_PORT: AtomicU32 = AtomicU32::new(0);
static DST_PORT: AtomicU32 = AtomicU32::new(0);
kernel::define_unified_harness!(no_boot, Cfg, |tick, k| {
    let addr = k as *const _ as usize;
    // TODO: align_of_val is a tautology (references are always naturally aligned).
    // Restore an MPU-aware alignment check once the kernel exposes its required
    // MPU region alignment as a const (KERNEL_ALIGNMENT was removed).
    if (addr & (core::mem::align_of_val(k) - 1)) != 0 { kexit!(failure); }
    if tick.is_multiple_of(5) {
        let s = P0_SENT.load(Ordering::Acquire);
        let r = P1_RECV_OK.load(Ordering::Acquire);
        let p = PARTS_RAN.load(Ordering::Acquire);
        klog!("[t{}] s={} r={} p={:02b}", tick, s, r, p);
        if s == 1 && r == 1 && p == 0b11 { klog!("PASS"); kexit!(success); }
        if tick > 100 { klog!("FAIL: timeout"); kexit!(failure); }
    }
});
const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    PARTS_RAN.fetch_or(1, Ordering::Release);
    let port = plib::QueuingPortId::new(SRC_PORT.load(Ordering::Acquire));
    let msg = [0xDE_u8, 0xAD, 0xBE, 0xEF];
    if plib::sys_queuing_send(port, &msg).is_ok() { P0_SENT.store(1, Ordering::Release); }
    loop { plib::sys_yield().expect("sys_yield"); }
}
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    PARTS_RAN.fetch_or(2, Ordering::Release);
    let port = plib::QueuingPortId::new(DST_PORT.load(Ordering::Acquire));
    loop {
        let mut buf = [0u8; 4];
        if plib::sys_queuing_recv(port, &mut buf).is_ok() && buf == [0xDE, 0xAD, 0xBE, 0xEF] {
            P1_RECV_OK.store(1, Ordering::Release); break;
        }
        plib::sys_yield().expect("sys_yield");
    }
    loop { plib::sys_yield().expect("sys_yield"); }
}
#[cortex_m_rt::entry]
#[allow(clippy::never_loop)] // kexit! diverges via inner loop on catch-all backend
fn main() -> ! {
    let p = match cortex_m::Peripherals::take() {
        Some(p) => p,
        None => loop { kexit!(failure); },
    };
    klog!("hw_integration: start");
    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 2)).is_err() { loop { kexit!(failure); } }
    if sched.add_system_window(1).is_err() { loop { kexit!(failure); } }
    if sched.add(ScheduleEntry::new(1, 2)).is_err() { loop { kexit!(failure); } }
    if sched.add_system_window(1).is_err() { loop { kexit!(failure); } }
    let entry_fns: [PartitionEntry; Cfg::N] = [p0_main, p1_main];
    let mut k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = &raw mut STACKS;
        let stacks = unsafe { &mut *ptr };
        let mut stk_iter = stacks.iter_mut();
        let memories: [_; Cfg::N] = core::array::from_fn(|i| {
            match ExternalPartitionMemory::from_aligned_stack(
                stk_iter.next().expect("stack"),
                entry_fns[i],
                MpuRegion::new(0, 0, 0),
                i as u8,
            ) {
                Ok(m) => m, Err(_) => loop { kexit!(failure); },
            }
        });
        match Kernel::<Cfg>::new(sched, &memories) {
            Ok(k) => k, Err(_) => loop { kexit!(failure); },
        }
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
    SRC_PORT.store(src as u32, Ordering::Release);
    DST_PORT.store(dst as u32, Ordering::Release);
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<Cfg>(p) } {
        Ok(never) => match never {},
        Err(_) => loop { kexit!(failure); },
    }
}
}
