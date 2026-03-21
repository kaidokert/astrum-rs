//! QEMU test: buf lend/revoke between two partitions via plib.
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
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const PAYLOAD: [u8; 4] = [0xCA, 0xFE, 0xBA, 0xBE];
/// Sentinel stored by `unwrap_or` on syscall failure.
const SYSCALL_FAILED: u32 = u32::MAX;
/// Maximum valid slot index returned by `sys_buf_alloc`.
const MAX_SLOT: u32 = TestConfig::BP as u32;
/// Maximum valid MPU region ID returned by `sys_buf_lend` (ARMv7-M: 8 regions).
const MAX_MPU_REGION: u32 = 8;

static ALLOC_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static WRITE_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static LEND_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static REVOKE_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static P0_DONE: AtomicU32 = AtomicU32::new(0);
static P1_READ_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static P1_WRITE_RC: AtomicU32 = AtomicU32::new(SYSCALL_FAILED);
static P1_DATA: AtomicU32 = AtomicU32::new(0);
static P1_DONE: AtomicU32 = AtomicU32::new(0);
static LENT: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if P0_DONE.load(Ordering::Acquire) == 0 || P1_DONE.load(Ordering::Acquire) == 0 {
        if tick >= 50 {
            hprintln!("FAIL timeout");
            kernel::kexit!(failure);
        }
        return;
    }
    if ALLOC_RC.load(Ordering::Relaxed) < MAX_SLOT
        && WRITE_RC.load(Ordering::Relaxed) == 4
        && LEND_RC.load(Ordering::Relaxed) < MAX_MPU_REGION
        && P1_READ_RC.load(Ordering::Relaxed) == 4
        && P1_WRITE_RC.load(Ordering::Relaxed) == SYSCALL_FAILED
        && P1_DATA.load(Ordering::Relaxed) == u32::from_le_bytes(PAYLOAD)
        && REVOKE_RC.load(Ordering::Relaxed) == 0
    {
        hprintln!("plib_buf_lend_test: PASS");
        kernel::kexit!(success);
    }
    hprintln!("FAIL");
    kernel::kexit!(failure);
});

extern "C" fn p0_main() -> ! {
    let slot = match plib::sys_buf_alloc(true, 0) {
        Ok(id) => {
            ALLOC_RC.store(id.as_raw() as u32, Ordering::Release);
            id
        }
        Err(_) => {
            ALLOC_RC.store(SYSCALL_FAILED, Ordering::Release);
            loop {
                cortex_m::asm::nop();
            }
        }
    };
    let payload = PAYLOAD;
    let wr = plib::sys_buf_write(slot, &payload).unwrap_or(SYSCALL_FAILED);
    WRITE_RC.store(wr, Ordering::Release);
    let (_base, lr) =
        plib::sys_buf_lend(slot, 1, false).unwrap_or((core::ptr::null_mut(), SYSCALL_FAILED));
    LEND_RC.store(lr, Ordering::Release);
    if lr < MAX_MPU_REGION {
        LENT.store(1, Ordering::Release);
    }
    while P1_DONE.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    let vr = plib::sys_buf_revoke(slot, 1).unwrap_or(SYSCALL_FAILED);
    REVOKE_RC.store(vr, Ordering::Release);
    P0_DONE.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    while LENT.load(Ordering::Acquire) == 0 {
        cortex_m::asm::nop();
    }
    let slot = plib::BufferSlotId::new(ALLOC_RC.load(Ordering::Relaxed) as u8);
    // Negative test: P1 must NOT be able to write when writable=false.
    let wr = plib::sys_buf_write(slot, &[0xFF; 4]).unwrap_or(SYSCALL_FAILED);
    P1_WRITE_RC.store(wr, Ordering::Release);
    // Positive test: P1 should be able to read the lent buffer.
    let mut buf = [0u8; 4];
    let rr = plib::sys_buf_read(slot, &mut buf).unwrap_or(SYSCALL_FAILED);
    P1_READ_RC.store(rr, Ordering::Release);
    P1_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
    P1_DONE.store(1, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 3)).expect("add P1");
    sched.add_system_window(1).expect("sys1");
    let mut cfgs = PartitionConfig::sentinel_array::<2>();
    cfgs[0].entry_point = entry_point_addr(p0_main);
    cfgs[1].entry_point = entry_point_addr(p1_main);
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("kernel");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("kernel");
    store_kernel(k);
    match boot(p).expect("boot") {}
}
