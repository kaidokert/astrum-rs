#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};
kernel::compose_kernel_config!(Config<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);
kernel::define_unified_harness!(Config, |tick, k| {
    if tick >= 10 && k.partition_sp().first() == Some(&0xDEAD0001) {
        hprintln!("pendsv_stack_overflow_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 200 {
        hprintln!("pendsv_stack_overflow_test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
});
extern "C" fn p0_overflow() -> ! {
    unsafe { core::arch::asm!("sub sp, sp, #996") };
    loop {
        cortex_m::asm::nop();
    }
}
extern "C" fn p1_healthy() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("pendsv_stack_overflow_test: start");
    let sched =
        kernel::scheduler::ScheduleTable::<{ Config::SCHED }>::round_robin(2, 1).expect("sched");
    let k = kernel::svc::Kernel::<Config>::create_sentinels(sched).expect("kernel");
    store_kernel(k);
    match boot(&[(p0_overflow, 0), (p1_healthy, 0)], &mut p).expect("boot") {}
}
