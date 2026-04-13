//! QEMU test: peripheral_regions + buf_lend coexistence.
//!
//! Two-partition test that exercises the scenario where peripheral wiring
//! would exhaust dynamic MPU slots if the desc_idx guard were missing.
//! P0 has a UART0 peripheral region AND lends a buffer to P1.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    buf_syscall,
    mpu_strategy::MpuStrategy,
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const P1: u8 = 1;
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const UART0_BASE: u32 = 0x4000_C000;
const UART0_SIZE: u32 = 4096;
const STACK_WORDS: usize = 256;

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

kernel::define_kernel!(TestConfig, |_tick, k| {
    if MIRROR.load(Ordering::Acquire) == 1 {
        let rid = RID.load(Ordering::Relaxed) as u8;
        if let Some(d) = k.dynamic_strategy.slot(rid) {
            BUF_ADDR.store(d.base, Ordering::Release);
            assert!(
                HARNESS_STRATEGY
                    .add_window(d.base, d.size, d.permissions, d.owner)
                    .is_ok(),
                "harness add_window failed for lent buffer"
            );
            MIRROR.store(2, Ordering::Release);
        }
    }
});

static RID: AtomicU32 = AtomicU32::new(0);
static BUF_ADDR: AtomicU32 = AtomicU32::new(0);
static MIRROR: AtomicU32 = AtomicU32::new(0);
static P1_RESULT: AtomicU32 = AtomicU32::new(0);

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    let slot = buf_syscall::buf_alloc(true, 0);
    assert!(slot.is_ok(), "buf_alloc failed: {:?}", slot);
    let slot = slot.unwrap();
    let w = buf_syscall::buf_write(slot, &MAGIC);
    assert!(w.is_ok(), "buf_write failed: {:?}", w);
    let rid = buf_syscall::buf_lend(slot, P1, false);
    assert!(rid.is_ok(), "buf_lend failed: {:?}", rid);
    let rid = rid.unwrap();
    RID.store(rid as u32, Ordering::Release);
    MIRROR.store(1, Ordering::Release);
    while P1_RESULT.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    let rev = buf_syscall::buf_revoke(slot, P1);
    assert!(rev.is_ok(), "buf_revoke failed: {:?}", rev);
    let rel = buf_syscall::buf_release(slot);
    assert!(rel.is_ok(), "buf_release failed: {:?}", rel);
    if P1_RESULT.load(Ordering::Acquire) == 2 {
        hprintln!("peripheral_buf_lend_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("FAIL: P1 read mismatch");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        let addr = BUF_ADDR.load(Ordering::Acquire);
        if addr != 0 && MIRROR.load(Ordering::Acquire) == 2 {
            let mut buf = [0u8; 4];
            for (i, b) in buf.iter_mut().enumerate() {
                // SAFETY: `addr` is the base of a lent buffer whose MPU window
                // was mapped by the harness (MIRROR==2 guarantees add_window
                // succeeded). The read is within the 4-byte buffer bounds.
                *b = unsafe { core::ptr::read_volatile((addr as *const u8).add(i)) };
            }
            if buf == MAGIC {
                P1_RESULT.store(2, Ordering::Release);
            } else {
                P1_RESULT.store(1, Ordering::Release);
            }
            break;
        }
        cortex_m::asm::nop();
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals already taken");
    hprintln!("peripheral_buf_lend_test: start");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("sched P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("sched P1");
    sched.add_system_window(1).expect("sys1");

    // Build partition descriptors with P0 having a UART0 peripheral region.
    let mut k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = (&raw mut __PARTITION_STACKS).cast::<[[u32; STACK_WORDS]; NP]>();
        let stacks = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(
                s0,
                EntryAddr::from_entry(p0_main as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(0),
            )
            .expect("mem 0")
            .with_code_mpu_region(MpuRegion::new(0, 0x4_0000, 0))
            .expect("code mpu 0")
            .with_peripheral_regions(&[MpuRegion::new(UART0_BASE, UART0_SIZE, 0)])
            .expect("periph 0"),
            ExternalPartitionMemory::new(
                s1,
                EntryAddr::from_entry(p1_main as PartitionEntry),
                MpuRegion::new(0, 0, 0),
                kernel::PartitionId::new(1),
            )
            .expect("mem 1")
            .with_code_mpu_region(MpuRegion::new(0, 0x4_0000, 0))
            .expect("code mpu 1"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
