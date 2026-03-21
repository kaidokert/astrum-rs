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
use kernel::partition::{entry_point_addr, PartitionConfig};
use kernel::sampling::PortDirection;
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgSmall, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled>
);

const NUM_PARTITIONS: usize = TestConfig::N;
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

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_queuing_timed_test: round_robin");
    let mut cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>();
    cfgs[0].entry_point = entry_point_addr(p0_main);
    cfgs[1].entry_point = entry_point_addr(p1_main);
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[])
        .expect("plib_queuing_timed_test: kernel");
    #[cfg(feature = "dynamic-mpu")]
    let mut k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("plib_queuing_timed_test: kernel");

    let src = k
        .queuing_mut()
        .create_port(PortDirection::Source)
        .expect("src port");
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .expect("dst port");
    k.queuing_mut().connect_ports(src, dst).expect("connect");
    store_kernel(k);

    match boot(p).expect("plib_queuing_timed_test: boot") {}
}
