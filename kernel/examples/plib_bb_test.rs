//! QEMU test: blackboard display / read / clear lifecycle.
//!
//! P0 displays 4 bytes to blackboard 0, P1 reads and verifies, then clears.
//! SysTick hook checks display-rc==0, read-rc==4, data-match, clear-rc==0.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::scheduler::ScheduleTable;
use kernel::{DebugEnabled, MsgMinimal, PartitionSpec, Partitions2, PortsSmall, SyncMinimal};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled>
);

const TIMEOUT_TICKS: u32 = 50;

const PAYLOAD: [u8; 4] = [0xBB, 0x0A, 0xBD, 0x01];
const EXPECTED_DATA: u32 = u32::from_le_bytes(PAYLOAD);
const NOT_YET: u32 = 0xDEAD_C0DE;

static DISPLAY_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_RC: AtomicU32 = AtomicU32::new(NOT_YET);
static READ_DATA: AtomicU32 = AtomicU32::new(NOT_YET);
static CLEAR_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    let display = DISPLAY_RC.load(Ordering::Acquire);
    let read = READ_RC.load(Ordering::Acquire);
    let clear = CLEAR_RC.load(Ordering::Acquire);

    if display == NOT_YET || read == NOT_YET || clear == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "plib_bb_test: FAIL timeout ({:#x},{:#x},{:#x})",
                display,
                read,
                clear
            );
            kernel::kexit!(failure);
        }
        return;
    }

    let data = READ_DATA.load(Ordering::Acquire);
    if display == 0 && read == 4 && data == EXPECTED_DATA && clear == 0 {
        hprintln!(
            "plib_bb_test: PASS (display={}, read={}, data={:#010x}, clear={})",
            display,
            read,
            data,
            clear
        );
        kernel::kexit!(success);
    } else {
        hprintln!(
            "plib_bb_test: FAIL ({:#x},{:#x},{:#010x} exp {:#010x},{:#x})",
            display,
            read,
            data,
            EXPECTED_DATA,
            clear
        );
        kernel::kexit!(failure);
    }
});

extern "C" fn p0_main() -> ! {
    let payload = PAYLOAD;
    match plib::sys_bb_display(plib::BlackboardId::new(0), &payload) {
        Ok(rc) => DISPLAY_RC.store(rc, Ordering::Release),
        Err(_) => DISPLAY_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    // Yield so P0 runs first and displays the payload.
    let _ = plib::sys_yield();
    let mut buf = [0u8; 4];
    match plib::sys_bb_read(plib::BlackboardId::new(0), &mut buf) {
        Ok(rc) => {
            READ_DATA.store(u32::from_le_bytes(buf), Ordering::Release);
            READ_RC.store(rc, Ordering::Release);
        }
        Err(_) => READ_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    match plib::sys_bb_clear(plib::BlackboardId::new(0)) {
        Ok(rc) => CLEAR_RC.store(rc, Ordering::Release),
        Err(_) => CLEAR_RC.store(0xFFFF_FFFF, Ordering::Release),
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");
    hprintln!("plib_bb_test: start");

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 3)
        .expect("plib_bb_test: round_robin");

    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_main, 0),
        PartitionSpec::new(p1_main, 0),
    ];
    init_kernel(sched, &parts).expect("plib_bb_test: kernel");
    with_kernel_mut(|k| {
        k.blackboards_mut()
            .create()
            .expect("plib_bb_test: create blackboard");
    });

    match boot(p).expect("plib_bb_test: boot") {}
}
