//! QEMU test: verify scheduler tick drives partition switches via Kernel.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::config::KernelConfig;
use kernel::partition::{MpuRegion, PartitionConfig};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::tick::configure_systick;
use panic_semihosting as _;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
}

static KERNEL: Mutex<RefCell<Option<Kernel<TestConfig>>>> = Mutex::new(RefCell::new(None));
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);
const RELOAD: u32 = 120_000 - 1; // ~10 ms at 12 MHz
const MAX_SWITCHES: u32 = 6;

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut k) = *KERNEL.borrow(cs).borrow_mut() {
            let event = k.advance_schedule_tick();
            #[cfg(not(feature = "dynamic-mpu"))]
            if let Some(pid) = event {
                hprintln!("switch -> partition {}", pid);
                SWITCH_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            #[cfg(feature = "dynamic-mpu")]
            if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
                hprintln!("switch -> partition {}", pid);
                SWITCH_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }
    });
}

#[entry]
fn main() -> ! {
    let mut sched: ScheduleTable<8> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 3)).unwrap();
    sched.add(ScheduleEntry::new(1, 2)).unwrap();

    let configs = [
        PartitionConfig {
            id: 0,
            entry_point: 0x0800_0000,
            stack_base: 0x2000_2000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_2000, 1024, 0),
        },
        PartitionConfig {
            id: 1,
            entry_point: 0x0800_1000,
            stack_base: 0x2000_3000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_3000, 1024, 0),
        },
    ];

    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(
        sched,
        &configs,
        kernel::virtual_device::DeviceRegistry::new(),
    )
    .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &configs).expect("kernel creation");

    cortex_m::interrupt::free(|cs| {
        KERNEL.borrow(cs).replace(Some(k));
    });

    let p = cortex_m::Peripherals::take().unwrap();
    configure_systick(&mut { p.SYST }, RELOAD);
    hprintln!("scheduler_tick: started, reload={}", RELOAD);

    loop {
        if SWITCH_COUNT.load(Ordering::Relaxed) >= MAX_SWITCHES {
            hprintln!("done: {} switches", MAX_SWITCHES);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
}
