#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

#[cfg(feature = "pac-rt")]
compile_error!("example `astrum_blinky` requires custom-ivt mode and must not be built with `pac-rt`");

use core::cell::RefCell;

use atsamd_hal as hal;
use cortex_m_rt::{entry, exception};
use critical_section::Mutex;
use embedded_hal_02::digital::v2::OutputPin as _;
use hal::clock::GenericClockController;
use hal::pac;
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{define_partition_debug, dprint};
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

type UserLed = hal::gpio::Pin<hal::gpio::PB02, hal::gpio::PushPullOutput>;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

static LED: Mutex<RefCell<Option<UserLed>>> = Mutex::new(RefCell::new(None));

define_partition_debug!(P0_DEBUG, 256);

kernel::kernel_config!(Same51AstrumBlinkyCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
});

unsafe extern "C" fn dummy_irq() {}

kernel::bind_interrupts!(Same51AstrumBlinkyCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(Same51AstrumBlinkyCfg, |tick, _k| {
    if tick % 1000 == 0 {
        kernel::klog!("[{:5}ms] same51 astrum_blinky tick", tick);
        if tick >= 3000 {
            kernel::klog!("SUCCESS: SAME51 astrum_blinky running");
        }
    }
});

fn store_led(led: UserLed) {
    critical_section::with(|cs| {
        let replaced = LED.borrow(cs).borrow_mut().replace(led);
        debug_assert!(replaced.is_none());
    });
}

fn take_led() -> UserLed {
    critical_section::with(|cs| LED.borrow(cs).borrow_mut().take()).expect("led")
}

extern "C" fn blinky_partition_body(_arg: u32) -> ! {
    let mut led = take_led();
    let _ = led.set_high();
    let _ = dprint!(&P0_DEBUG, "astrum blinky partition booted");

    loop {
        let _ = led.set_low();
        let _ = plib::sys_sleep_ticks(200);
        let _ = led.set_high();
        let _ = plib::sys_sleep_ticks(200);
    }
}

kernel::partition_trampoline!(blinky_partition => blinky_partition_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let pac::Peripherals {
        gclk,
        mut mclk,
        mut osc32kctrl,
        mut oscctrl,
        mut nvmctrl,
        port,
        ..
    } = pac::Peripherals::take().unwrap();

    let _clocks = GenericClockController::with_internal_32kosc(
        gclk,
        &mut mclk,
        &mut osc32kctrl,
        &mut oscctrl,
        &mut nvmctrl,
    );

    let pins = hal::gpio::Pins::new(port);
    let mut led = pins.pb02.into_push_pull_output();
    let _ = led.set_high();
    store_led(led);

    let mut sched = ScheduleTable::<{ Same51AstrumBlinkyCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; 1] = [AlignedStack2K::ZERO; 1];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, blinky_partition as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0");

    let mems = [mem0];
    let mut k = Kernel::<Same51AstrumBlinkyCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    let _ = kernel::state::with_kernel_mut::<Same51AstrumBlinkyCfg, _, _>(|k| {
        k.partitions_mut()
            .get_mut(0)
            .expect("partition")
            .set_debug_buffer(&P0_DEBUG);
        Ok::<(), ()>(())
    });

    match boot(p).expect("boot") {}
}
