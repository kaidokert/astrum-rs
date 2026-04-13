//! QEMU integration test: error handler full flow.
//!
//! P0 registers an error handler, faults, handler queries status and requests
//! warm restart, restarted P0 confirms WarmRestart condition.  P1 is a monitor.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example error_handler_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    mpu,
    partition::{FaultPolicy, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// Shared atomics — partition→harness signalling (no hprintln from unpriv code).
/// 0=pending, 1=PASS, 2=FAIL.
static RESULT: AtomicU32 = AtomicU32::new(0);
static FAIL_CODE: AtomicU32 = AtomicU32::new(0);
static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(0);
/// 0=not run, 1=ok, 2+=error.
static HANDLER_STATUS: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);

// TODO: derive from MPU guard region symbol/constant instead of hardcoding;
// currently matches the guard region configured in configure_static_mpu().
const KERNEL_ADDR: u32 = 0x2000_F000;

fn fail(code: u32) -> ! {
    RESULT.store(2, Ordering::Release);
    FAIL_CODE.store(code, Ordering::Release);
    loop {
        asm::nop();
    }
}

// --- Error handler (kernel activates this on fault) ---

const _: PartitionEntry = error_handler;
extern "C" fn error_handler() -> ! {
    match plib::sys_get_error_status() {
        Ok(info) => {
            // FaultKind::MemManage == 0, partition must be 0
            if info.fault_kind_raw() != 0 || info.failed_partition() != 0 {
                fail(10);
            }
            HANDLER_STATUS.store(1, Ordering::Release);
        }
        Err(_) => fail(11),
    }
    if plib::sys_request_restart(true).is_err() {
        fail(12);
    }
    loop {
        asm::nop();
    }
}

// --- Partition entry points ---

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    let run = P0_RUN_COUNT.fetch_add(1, Ordering::Release) + 1;
    if run == 1 {
        if !matches!(
            plib::sys_get_start_condition(),
            Ok(plib::StartCondition::NormalBoot)
        ) {
            fail(20);
        }
        // SAFETY: error_handler has the correct `extern "C" fn() -> !` signature
        // expected by the kernel; transmuting the fn-item pointer to a fn-pointer
        // of the same ABI is sound.
        let handler_fn: extern "C" fn() =
            unsafe { core::mem::transmute(error_handler as *const ()) };
        if plib::sys_register_error_handler(handler_fn).is_err() {
            fail(21);
        }
        for _ in 0..50u32 {
            asm::nop();
        }
        // SAFETY: intentional invalid write to kernel guard region.
        unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0001) };
    } else {
        match plib::sys_get_start_condition() {
            Ok(plib::StartCondition::WarmRestart) => RESULT.store(1, Ordering::Release),
            _ => fail(30),
        }
    }
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
}

// --- Harness (privileged SysTick context) ---

kernel::define_kernel!(TestConfig, |tick, _k| {
    let result = RESULT.load(Ordering::Acquire);
    if result == 1 {
        let hs = HANDLER_STATUS.load(Ordering::Acquire);
        let p1 = P1_COUNTER.load(Ordering::Relaxed);
        if hs != 1 || p1 == 0 {
            hprintln!("error_handler_test: FAIL hs={} p1={}", hs, p1);
            kernel::kexit!(failure);
        }
        hprintln!("error_handler_test: PASS (p1={}, hs={})", p1, hs);
        kernel::kexit!(success);
    }
    if result == 2 {
        hprintln!(
            "error_handler_test: FAIL code={} hs={} runs={}",
            FAIL_CODE.load(Ordering::Relaxed),
            HANDLER_STATUS.load(Ordering::Relaxed),
            P0_RUN_COUNT.load(Ordering::Relaxed),
        );
        kernel::kexit!(failure);
    }
    if tick >= 300 {
        hprintln!(
            "error_handler_test: FAIL timeout runs={} hs={}",
            P0_RUN_COUNT.load(Ordering::Relaxed),
            HANDLER_STATUS.load(Ordering::Relaxed),
        );
        kernel::kexit!(failure);
    }
});

// --- MPU setup ---

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();
    let flash_sf = mpu::encode_size(256 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);
    let ram_sf = mpu::encode_size(64 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);
    // SAFETY: enabling MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// --- Boot ---

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("error_handler_test: start");
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    configure_static_mpu(&p.MPU);
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("add P0");
    sched.add_system_window(1).expect("sys0");
    sched.add(ScheduleEntry::new(1, 2)).expect("add P1");
    sched.add_system_window(1).expect("sys1");
    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
        PartitionSpec::new(p1_entry as PartitionEntry, 0)
            .with_code_mpu(MpuRegion::new(0, 0x4_0000, 0)),
    ];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_fault_policy(FaultPolicy::WarmRestart { max: 3 });
    store_kernel(&mut k);
    match boot(p).expect("boot") {}
}
