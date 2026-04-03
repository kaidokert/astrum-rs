//! QEMU test: sys_dev_read_timed packed-r2 ABI (timeout=0 and timeout>0).

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
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::virtual_device::VirtualDevice;
use kernel::{DebugEnabled, MsgMinimal, PartitionEntry, Partitions1, PortsTiny, SyncMinimal};

const NP: usize = 1;
const STACK_WORDS: usize = 256;

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;
const NOT_YET: u32 = 0xDEAD_C0DE;
const PAYLOAD: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);

static NONBLOCK_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static TIMED_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static TIMED_DATA: AtomicU32 = AtomicU32::new(NOT_YET);
static PHASE1_DONE: AtomicU32 = AtomicU32::new(0);
static RX_READY: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, k| {
    // After non-blocking read completes, push data into UART-A RX.
    if PHASE1_DONE.load(Ordering::Acquire) == 1 && RX_READY.load(Ordering::Relaxed) == 0 {
        k.uart_pair.a.push_rx(&PAYLOAD);
        RX_READY.store(1, Ordering::Release);
        return;
    }

    let nb = NONBLOCK_RC.load(Ordering::Acquire);
    let tr = TIMED_RC.load(Ordering::Acquire);

    if nb == NOT_YET || tr == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!("plib_dev_read_timed_test: FAIL timeout");
            kernel::kexit!(failure);
        }
        return;
    }

    let d = TIMED_DATA.load(Ordering::Acquire);
    let pass = nb == 0 && tr == 4 && d == EXPECTED_DATA;
    #[rustfmt::skip]
    hprintln!("plib_dev_read_timed_test: {} nb={} tr={} d={:#x}", if pass {"PASS"} else {"FAIL"}, nb, tr, d);
    if pass {
        kernel::kexit!(success)
    } else {
        kernel::kexit!(failure)
    }
});

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    let dev = plib::DeviceId::new(0);
    if plib::sys_dev_open(dev).is_err() {
        NONBLOCK_RC.store(u32::MAX, Ordering::Release);
        loop {
            cortex_m::asm::nop();
        }
    }

    let mut buf = [0u8; 4]; // Phase 1: non-blocking (timeout=0), empty RX → Ok(0)
    match plib::sys_dev_read_timed(dev, &mut buf, 0) {
        Ok(rc) => NONBLOCK_RC.store(rc, Ordering::Release),
        Err(_) => NONBLOCK_RC.store(u32::MAX, Ordering::Release),
    }
    PHASE1_DONE.store(1, Ordering::Release);

    while RX_READY.load(Ordering::Acquire) == 0 {
        let _ = plib::sys_yield();
    }

    let mut buf = [0u8; 4]; // Phase 2: timed (timeout=50), data in RX → Ok(4)
    match plib::sys_dev_read_timed(dev, &mut buf, 50) {
        Ok(rc) => {
            TIMED_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            TIMED_RC.store(rc, Ordering::Release);
        }
        Err(_) => TIMED_RC.store(u32::MAX, Ordering::Release),
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("plib_dev_read_timed_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    // SAFETY: called once from main before any interrupt handler runs.
    // TODO: reviewer false positive — `__PARTITION_STACKS` is defined by
    // `define_unified_harness!(@impl_compat)` as a module-level `static mut`.
    let stacks = unsafe { &mut *(&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>() };
    let [ref mut s0] = *stacks;
    let mpu = MpuRegion::new(0, 0, 0);
    let e = EntryAddr::from_entry;
    let memories = [ExternalPartitionMemory::new(
        s0,
        e(partition_main as PartitionEntry),
        mpu,
        kernel::PartitionId::new(0),
    )
    .expect("mem 0")];
    let mut k = Kernel::<TestConfig>::new(sched, &memories).expect("kernel");
    store_kernel(&mut k);

    with_kernel_mut(|k| {
        // SAFETY: Kernel state lives on the caller's stack in a -> !
        // function, so it is effectively 'static. The UART-A backend lives
        // inside the kernel struct. with_kernel_mut runs inside
        // interrupt::free, guaranteeing exclusive access on single-core
        // Cortex-M.
        unsafe {
            let a: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.a as *mut _);
            k.registry.add(a).expect("register UART-A");
        }
    });

    match boot(p).expect("boot") {}
}
