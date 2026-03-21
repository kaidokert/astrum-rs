//! QEMU test: P0 sends a 4-byte pattern via queuing port, P1 receives and verifies.
//! SysTick hook checks send-rc==0, recv-rc==4 (byte count), and data match.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::partition::{EntryAddr, PartitionConfig};
use kernel::sampling::PortDirection;
use kernel::scheduler::ScheduleTable;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgSmall, PartitionEntry, Partitions2, PortsTiny, SyncMinimal};

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
    let sr = SEND_RC.load(Ordering::Acquire);
    let rr = RECV_RC.load(Ordering::Acquire);

    if sr == NOT_YET || rr == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_queuing_test: FAIL timeout (sr={:#x}, rr={:#x})",
                sr,
                rr
            );
            kernel::kexit!(failure);
        }
        return;
    }

    let data = RECV_DATA.load(Ordering::Acquire);
    if sr == 0 && rr == 4 && data == EXPECTED_DATA {
        hprintln!(
            "plib_queuing_test: PASS (sr={}, rr={}, data={:#010x})",
            sr,
            rr,
            data
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_queuing_test: FAIL (sr={:#x}, rr={:#x}, data={:#010x} exp {:#010x})",
            sr,
            rr,
            data,
            EXPECTED_DATA
        );
        kernel::kexit!(failure);
    }
});

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    // Copy const to stack so pointer is in MPU-accessible partition memory.
    let payload = PAYLOAD;
    match plib::sys_queuing_send(plib::QueuingPortId::new(0), &payload) {
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
    match plib::sys_queuing_recv(plib::QueuingPortId::new(1), &mut buf) {
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
    hprintln!("plib_queuing_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_queuing_test: round_robin");
    let mut cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>();
    cfgs[0].entry_point = EntryAddr::from_fn(p0_main);
    cfgs[1].entry_point = EntryAddr::from_fn(p1_main);
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k =
        Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("plib_queuing_test: kernel");
    #[cfg(feature = "dynamic-mpu")]
    let mut k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("plib_queuing_test: kernel");

    // Create source port (id=0) on P0 and destination port (id=1) on P1, then connect.
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

    match boot(p).expect("plib_queuing_test: boot") {}
}
