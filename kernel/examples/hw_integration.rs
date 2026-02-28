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
    partition::{MpuRegion, PartitionConfig},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::{SYS_QUEUING_RECV, SYS_QUEUING_SEND, SYS_YIELD},
    DebugEnabled, MsgSmall, Partitions2, PortsTiny, SyncMinimal,
};
#[cfg(panic_backend = "halt")]
use panic_halt as _;
#[cfg(panic_backend = "rtt")]
use panic_rtt_target as _;
#[cfg(panic_backend = "semihosting")]
use panic_semihosting as _;
// TODO: replace ERR bitmask with a typed Result abstraction once syscall API supports it
const ERR: u32 = 0x8000_0000;
kernel::compose_kernel_config!(Cfg<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>);
static P0_SENT: AtomicU32 = AtomicU32::new(0);
static P1_RECV_OK: AtomicU32 = AtomicU32::new(0);
static PARTS_RAN: AtomicU32 = AtomicU32::new(0);
kernel::define_unified_harness!(no_boot, Cfg, { Cfg::N }, { Cfg::STACK_WORDS }, |tick, k| {
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
    let (port, msg) = (r0 >> 16, [0xDE_u8, 0xAD, 0xBE, 0xEF]);
    let rc = kernel::svc!(SYS_QUEUING_SEND, port, 4u32, msg.as_ptr() as u32);
    if rc & ERR == 0 { P0_SENT.store(1, Ordering::Release); }
    loop { kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32); }
}
kernel::partition_trampoline!(p0_main => p0_main_body);
extern "C" fn p1_main_body(r0: u32) -> ! {
    PARTS_RAN.fetch_or(2, Ordering::Release);
    let port = r0 & 0xFFFF;
    loop {
        let mut buf = [0u8; 4];
        let sz = kernel::svc!(SYS_QUEUING_RECV, port, 4u32, buf.as_mut_ptr() as u32);
        if sz & ERR == 0 && buf == [0xDE, 0xAD, 0xBE, 0xEF] {
            P1_RECV_OK.store(1, Ordering::Release); break;
        }
        kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
    loop { kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32); }
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
    let cfgs: [PartitionConfig; Cfg::N] = core::array::from_fn(|i| PartitionConfig {
        id: i as u8, entry_point: 0, stack_base: 0, stack_size: (Cfg::STACK_WORDS * 4) as u32,
        mpu_region: MpuRegion::new(0, 0, 0), peripheral_regions: heapless::Vec::new(),
    });
    #[cfg(feature = "dynamic-mpu")]
    let mut k = match Kernel::<Cfg>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new()) {
        Ok(k) => k,
        Err(_) => loop { kexit!(failure); },
    };
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = match Kernel::<Cfg>::new(sched, &cfgs) {
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
    let parts: [(extern "C" fn() -> !, u32); Cfg::N] =
        [(p0_main, (src as u32) << 16), (p1_main, dst as u32)];
    match boot::boot::<Cfg>(&parts, &mut p) {
        Ok(never) => match never {},
        Err(_) => loop { kexit!(failure); },
    }
}
}
