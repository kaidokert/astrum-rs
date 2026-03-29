//! QEMU test: P0 writes a 4-byte pattern via sampling port, P1 reads and verifies.
//! SysTick hook checks write-rc==0, read-rc==4 (byte count), and data match.
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
use kernel::{DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsSmall, SyncMinimal};

const NP: usize = 2;
const STACK_WORDS: usize = 256;

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;

const PAYLOAD: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);
const NOT_YET: u32 = 0xDEAD_C0DE;

static WRITE_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_DATA: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let wr = WRITE_RC.load(Ordering::Acquire);
    let rd = READ_RC.load(Ordering::Acquire);

    if wr == NOT_YET || rd == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_sampling_test: FAIL timeout (wr={:#x}, rd={:#x})",
                wr,
                rd
            );
            kernel::kexit!(failure);
        }
        return;
    }

    let data = READ_DATA.load(Ordering::Acquire);
    if wr == 0 && rd == 4 && data == EXPECTED_DATA {
        hprintln!(
            "plib_sampling_test: PASS (wr={}, rd={}, data={:#010x})",
            wr,
            rd,
            data
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_sampling_test: FAIL (wr={:#x}, rd={:#x}, data={:#010x} exp {:#010x})",
            wr,
            rd,
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
    match plib::sys_sampling_write(plib::SamplingPortId::new(0), &payload) {
        Ok(rc) => WRITE_RC.store(rc, Ordering::Release),
        Err(_) => WRITE_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    // Yield once so P0 writes before we read.
    let _ = plib::sys_yield();
    let mut buf = [0u8; 4];
    match plib::sys_sampling_read(plib::SamplingPortId::new(1), &mut buf) {
        Ok(rc) => {
            READ_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            READ_RC.store(rc, Ordering::Release);
        }
        Err(_) => READ_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_sampling_test: start");

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
    let mut k = Kernel::<TestConfig>::new(sched, &memories).expect("plib_sampling_test: kernel");

    // Create source port (id=0) and destination port (id=1), then connect.
    let src = k
        .sampling_mut()
        .create_port(PortDirection::Source, 0)
        .expect("src port");
    let dst = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 0)
        .expect("dst port");
    k.sampling_mut().connect_ports(src, dst).expect("connect");
    store_kernel(&mut k);

    match boot(p).expect("plib_sampling_test: boot") {}
}
