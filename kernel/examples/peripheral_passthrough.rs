//! Peripheral passthrough: first example with non-empty `peripheral_regions`.
//!
//! Configures a partition with an MPU peripheral window covering UART0
//! (0x4000_C000, 4 KiB). The partition entry performs a volatile read of
//! the UART Flag Register (UARTFR at 0x4000_C018). If the MPU window is
//! correctly programmed the read succeeds; otherwise a DACCVIOL fault fires.
//!
//! Exercises: PartitionConfig → Kernel::new → PCB → wire_boot_peripherals
//!            → PendSV → hardware MPU pipeline end-to-end.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    partition::MpuRegion,
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(PassthroughConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

const NUM_PARTITIONS: usize = 1;

/// UART0 register block on lm3s6965evb.
const UART0_BASE: u32 = 0x4000_C000;
const UART0_SIZE: u32 = 4096;
/// UART Flag Register offset from base.
const UARTFR_OFFSET: u32 = 0x18;
/// UARTFR reset value on PL011 (TXFE | RXFE = bits 7 and 4).
const UARTFR_RESET: u32 = 0x90;

kernel::define_unified_harness!(PassthroughConfig);

/// Partition stores the UARTFR value here after a successful volatile read.
static UARTFR_VALUE: AtomicU32 = AtomicU32::new(0);

extern "C" fn partition_main() -> ! {
    // Volatile read of UART0 Flag Register.
    // If the MPU peripheral window is not programmed this faults (DACCVIOL).
    let uartfr_addr = (UART0_BASE + UARTFR_OFFSET) as *const u32;
    // SAFETY: `uartfr_addr` (0x4000_C018) lies within the UART0 peripheral
    // region (0x4000_C000..+4 KiB) that was granted to this partition via
    // `peripheral_regions`. The MPU window makes this address accessible from
    // unprivileged mode. The read is naturally aligned (4-byte address, u32).
    let val = unsafe { core::ptr::read_volatile(uartfr_addr) };
    UARTFR_VALUE.store(val, Ordering::Release);

    hprintln!(
        "peripheral_passthrough: UARTFR = {:#06x} (expect {:#06x})",
        val,
        UARTFR_RESET
    );
    if val == UARTFR_RESET {
        hprintln!("peripheral_passthrough: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("peripheral_passthrough: FAIL — unexpected UARTFR value");
        debug::exit(debug::EXIT_FAILURE);
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    // TODO: .unwrap()/.expect() are acceptable in QEMU examples but should
    // be replaced with proper error handling if this logic moves into the kernel.
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("peripheral_passthrough: start");

    let mut sched = ScheduleTable::<{ PassthroughConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(partition_main, 0)];
    init_kernel(sched, &parts).expect("kernel");
    with_kernel_mut(|k| {
        k.partitions_mut()
            .get_mut(0)
            .expect("partition 0")
            .set_peripheral_regions(&[MpuRegion::new(UART0_BASE, UART0_SIZE, 0)]);
    });
    hprintln!("peripheral_passthrough: booting");

    match boot(p).expect("boot failed") {}
}
