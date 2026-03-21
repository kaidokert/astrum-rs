//! QEMU test: buffer alloc / write / read / release round-trip via plib.
//!
//! Single partition: allocate a writable buffer slot, write 4 bytes, read back,
//! verify data matches, then release.  SysTick hook checks every return code.

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
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;
const NOT_YET: u32 = 0xDEAD_C0DE;
const PAYLOAD: [u8; 4] = [0xB0, 0xFF, 0xE4, 0x01];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);

/// Return code from sys_buf_alloc (slot index on success).
static ALLOC_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Return code from sys_buf_write (bytes written).
static WRITE_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Return code from sys_buf_read (bytes read).
static READ_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Data read back from the buffer (as little-endian u32).
static READ_DATA: AtomicU32 = AtomicU32::new(NOT_YET);
/// Return code from sys_buf_release.
static RELEASE_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let alloc = ALLOC_RC.load(Ordering::Acquire);
    let write = WRITE_RC.load(Ordering::Acquire);
    let read = READ_RC.load(Ordering::Acquire);
    let release = RELEASE_RC.load(Ordering::Acquire);

    if alloc == NOT_YET || write == NOT_YET || read == NOT_YET || release == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_buf_test: FAIL timeout ({:#x},{:#x},{:#x},{:#x})",
                alloc,
                write,
                read,
                release
            );
            kernel::kexit!(failure);
        }
        return;
    }

    let data = READ_DATA.load(Ordering::Acquire);
    // alloc returns slot index (valid: 0..BP), write/read return 4 (byte count),
    // data must match payload, release returns 0 on success.
    if alloc < 16 && write == 4 && read == 4 && data == EXPECTED_DATA && release == 0 {
        hprintln!(
            "plib_buf_test: PASS (slot={}, w={}, r={}, data={:#010x}, rel={})",
            alloc,
            write,
            read,
            data,
            release
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_buf_test: FAIL ({},{},{},{:#010x} exp {:#010x},{})",
            alloc,
            write,
            read,
            data,
            EXPECTED_DATA,
            release
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn p0_main() -> ! {
    hprintln!("p0: entering");
    // 1. Allocate a writable buffer slot (no timeout).
    let slot = match plib::sys_buf_alloc(true, 0) {
        Ok(id) => {
            ALLOC_RC.store(id.as_raw() as u32, Ordering::Release);
            id
        }
        Err(_) => {
            ALLOC_RC.store(0xFFFF_FFFF, Ordering::Release);
            loop {
                cortex_m::asm::nop();
            }
        }
    };

    // 2. Write payload into the buffer.
    let payload = PAYLOAD;
    match plib::sys_buf_write(slot, &payload) {
        Ok(rc) => WRITE_RC.store(rc, Ordering::Release),
        Err(_) => WRITE_RC.store(0xFFFF_FFFF, Ordering::Release),
    }

    // 3. Read back from the buffer.
    let mut buf = [0u8; 4];
    match plib::sys_buf_read(slot, &mut buf) {
        Ok(rc) => {
            READ_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            READ_RC.store(rc, Ordering::Release);
        }
        Err(_) => READ_RC.store(0xFFFF_FFFF, Ordering::Release),
    }

    // 4. Release the buffer slot.
    match plib::sys_buf_release(slot) {
        Ok(rc) => RELEASE_RC.store(rc, Ordering::Release),
        Err(_) => RELEASE_RC.store(0xFFFF_FFFF, Ordering::Release),
    }

    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_idle() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("plib_buf_test: start");

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(ScheduleEntry::new(0, 3))
        .expect("plib_buf_test: add P0");
    sched.add_system_window(1).expect("plib_buf_test: sys0");
    sched
        .add(ScheduleEntry::new(1, 3))
        .expect("plib_buf_test: add P1");
    sched.add_system_window(1).expect("plib_buf_test: sys1");
    let mut cfgs = PartitionConfig::sentinel_array::<2>();
    cfgs[0].entry_point = EntryAddr::from_fn(p0_main);
    cfgs[1].entry_point = EntryAddr::from_fn(p1_idle);
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::with_config(sched, &cfgs, &[]).expect("plib_buf_test: kernel");
    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::with_config(
        sched,
        &cfgs,
        kernel::virtual_device::DeviceRegistry::new(),
        &[],
    )
    .expect("plib_buf_test: kernel");
    store_kernel(k);

    match boot(p).expect("plib_buf_test: boot") {}
}
