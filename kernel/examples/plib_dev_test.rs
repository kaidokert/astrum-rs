//! QEMU test: dev_open / dev_write / dev_read / dev_close round-trip via plib.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::virtual_device::VirtualDevice;
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions1, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;
const NOT_YET: u32 = 0xDEAD_C0DE;
const PAYLOAD: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);

static OPEN_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static WRITE_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_DATA: AtomicU32 = AtomicU32::new(NOT_YET);
static CLOSE_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static WRITE_DONE: AtomicU32 = AtomicU32::new(0);
static RX_READY: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, k| {
    // Inject payload into UART-A RX once write is done.
    if WRITE_DONE.load(Ordering::Acquire) == 1 && RX_READY.load(Ordering::Relaxed) == 0 {
        k.uart_pair.a.push_rx(&PAYLOAD);
        RX_READY.store(1, Ordering::Release);
        return;
    }

    let o = OPEN_RC.load(Ordering::Acquire);
    let w = WRITE_RC.load(Ordering::Acquire);
    let r = READ_RC.load(Ordering::Acquire);
    let c = CLOSE_RC.load(Ordering::Acquire);

    if o == NOT_YET || w == NOT_YET || r == NOT_YET || c == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!("plib_dev_test: FAIL timeout");
            kernel::kexit!(failure);
        }
        return;
    }

    let d = READ_DATA.load(Ordering::Acquire);
    let pass = o == 0 && w == 4 && r == 4 && d == EXPECTED_DATA && c == 0;
    #[rustfmt::skip]
    hprintln!("plib_dev_test: {} o={} w={} r={} d={:#x} c={}", if pass {"PASS"} else {"FAIL"}, o, w, r, d, c);
    if pass {
        kernel::kexit!(success)
    } else {
        kernel::kexit!(failure)
    }
});

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    let dev = plib::DeviceId::new(0);
    match plib::sys_dev_open(dev) {
        Ok(rc) => OPEN_RC.store(rc, Ordering::Release),
        Err(_) => OPEN_RC.store(u32::MAX, Ordering::Release),
    }
    let payload = PAYLOAD;
    match plib::sys_dev_write(dev, &payload) {
        Ok(rc) => WRITE_RC.store(rc, Ordering::Release),
        Err(_) => WRITE_RC.store(u32::MAX, Ordering::Release),
    }
    WRITE_DONE.store(1, Ordering::Release);
    while RX_READY.load(Ordering::Acquire) == 0 {
        let _ = plib::sys_yield();
    }
    let mut buf = [0u8; 4];
    match plib::sys_dev_read(dev, &mut buf) {
        Ok(rc) => {
            READ_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            READ_RC.store(rc, Ordering::Release);
        }
        Err(_) => READ_RC.store(u32::MAX, Ordering::Release),
    }
    match plib::sys_dev_close(dev) {
        Ok(rc) => CLOSE_RC.store(rc, Ordering::Release),
        Err(_) => CLOSE_RC.store(u32::MAX, Ordering::Release),
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("plib_dev_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 3))
        .expect("plib_dev_test: add P0");
    sched.add_system_window(1).expect("plib_dev_test: sys0");
    init_kernel(
        sched,
        &[PartitionSpec::new(partition_main as PartitionEntry, 0)],
    )
    .expect("plib_dev_test: kernel");

    // Register UART-A (device 0) so SYS_DEV_* syscalls can reach it.
    // TODO: the 'static lifetime cast is architecturally suspect; replace with
    //       a safe kernel device-registration API when one exists.
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

    match boot(p).expect("plib_dev_test: boot") {}
}
