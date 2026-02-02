#![no_std]
#![no_main]

use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::kernel::KernelState;
use kernel::partition::{MpuRegion, PartitionConfig};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::tick::{configure_systick, on_systick};
use panic_semihosting as _;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

static STATE: Mutex<RefCell<Option<KernelState<4, 8>>>> = Mutex::new(RefCell::new(None));
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);

/// SysTick reload for ~10 ms at 12 MHz: 120_000 - 1.
const RELOAD: u32 = 120_000 - 1;
const MAX_SWITCHES: u32 = 6;

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut ks) = *STATE.borrow(cs).borrow_mut() {
            #[cfg(not(feature = "dynamic-mpu"))]
            if let Some(pid) = on_systick(ks) {
                hprintln!("switch -> partition {}", pid);
                SWITCH_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            #[cfg(feature = "dynamic-mpu")]
            if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = on_systick(ks) {
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
    sched.start();

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

    let ks = KernelState::new(sched, &configs).expect("invalid kernel config");
    cortex_m::interrupt::free(|cs| {
        STATE.borrow(cs).replace(Some(ks));
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
