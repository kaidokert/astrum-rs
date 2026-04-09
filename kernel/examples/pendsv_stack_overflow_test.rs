#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
kernel::kernel_config!(Config<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);
kernel::define_kernel!(Config, |tick, k| {
    if tick >= 10 && k.partition_sp().first() == Some(&kernel::partition_core::SP_SENTINEL_FAULT) {
        hprintln!("pendsv_stack_overflow_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= 200 {
        hprintln!("pendsv_stack_overflow_test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
});
const _: PartitionEntry = p0_overflow;
extern "C" fn p0_overflow() -> ! {
    unsafe { core::arch::asm!("sub sp, sp, #996") };
    loop {
        cortex_m::asm::nop();
    }
}
const _: PartitionEntry = p1_healthy;
extern "C" fn p1_healthy() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("pendsv_stack_overflow_test: start");
    let sched =
        kernel::scheduler::ScheduleTable::<{ Config::SCHED }>::round_robin(2, 1).expect("sched");
    let parts: [PartitionSpec; 2] = [
        PartitionSpec::new(p0_overflow as PartitionEntry, 0),
        PartitionSpec::new(p1_healthy as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("kernel");
    match boot(p).expect("boot") {}
}
