//! QEMU integration test: restart hook fires on warm restart.
//!
//! P0 is configured with `FaultPolicy::WarmRestart { max: 1 }` and a restart
//! hook that sets a global flag.  On first run P0 faults (writes to kernel
//! guard region).  The MemManage handler auto-restarts P0, invoking the hook.
//! On the second run P0 reads the flag and stores the result in an atomic.
//! The SysTick hook observes the outcome and prints PASS/FAIL.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example restart_hook_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    mpu,
    partition::{FaultPolicy, RestartHook},
    scheduler::ScheduleTable,
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// ---------------------------------------------------------------------------
// Shared atomics
// ---------------------------------------------------------------------------

/// Set to `true` by the restart hook — proves the hook fired.
static HOOK_FIRED: AtomicBool = AtomicBool::new(false);

/// Set to `true` by the restart hook when `warm == true`.
static HOOK_WARM: AtomicBool = AtomicBool::new(false);

/// Incremented by P0 at entry — distinguishes first run from restart.
static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(0);

/// Set to `true` by P0 on the second run if HOOK_FIRED was true.
static P0_VERIFIED: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Restart hook (runs in privileged/kernel context)
// ---------------------------------------------------------------------------

fn my_restart_hook(_pid: usize, warm: bool) {
    HOOK_FIRED.store(true, Ordering::SeqCst);
    HOOK_WARM.store(warm, Ordering::SeqCst);
}

// ---------------------------------------------------------------------------
// Partition entry points
// ---------------------------------------------------------------------------

const KERNEL_ADDR: u32 = 0x2000_F000;

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    let run = P0_RUN_COUNT.fetch_add(1, Ordering::SeqCst);
    if run == 0 {
        // First run: trigger a MemManage fault (DACCVIOL).
        // SAFETY: Deliberately writing to the kernel guard region.
        unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0000) };
    } else {
        // Warm restart: check that the hook set the flag.
        if HOOK_FIRED.load(Ordering::SeqCst) && HOOK_WARM.load(Ordering::SeqCst) {
            P0_VERIFIED.store(true, Ordering::SeqCst);
        }
    }
    loop {
        asm::nop();
    }
}

/// P1 is a simple monitor partition — keeps the scheduler busy.
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        asm::nop();
    }
}

// ---------------------------------------------------------------------------
// SysTick hook
// ---------------------------------------------------------------------------

const TIMEOUT_TICKS: u32 = 300;

kernel::define_unified_harness!(TestConfig, |tick, _k| {
    if P0_VERIFIED.load(Ordering::SeqCst) {
        hprintln!("restart_hook_test: PASS");
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT_TICKS {
        let run = P0_RUN_COUNT.load(Ordering::SeqCst);
        let fired = HOOK_FIRED.load(Ordering::SeqCst);
        let warm = HOOK_WARM.load(Ordering::SeqCst);
        hprintln!(
            "restart_hook_test: FAIL timeout (run={}, fired={}, warm={})",
            run,
            fired,
            warm
        );
        kernel::kexit!(failure);
    }
});

// ---------------------------------------------------------------------------
// MPU setup
// ---------------------------------------------------------------------------

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: Single-core, called before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash (code) — read-only for all.
    let flash_sf = mpu::encode_size(256 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full access for partitions.
    let ram_sf = mpu::encode_size(64 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard region — privileged-only (overrides R1).
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    // SAFETY: Enabling the MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("restart_hook_test: start");

    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // SAFETY: Privileged context before boot; setting MemManage to lowest
    // priority so SysTick can preempt and observe partition state.
    unsafe {
        p.SCB.set_priority(
            cortex_m::peripheral::scb::SystemHandler::MemoryManagement,
            0xFF,
        );
    }

    configure_static_mpu(&p.MPU);

    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched
        .add(kernel::scheduler::ScheduleEntry::new(0, 2))
        .expect("P0");
    sched.add_system_window(1).expect("sys0");
    sched
        .add(kernel::scheduler::ScheduleEntry::new(1, 2))
        .expect("P1");
    sched.add_system_window(1).expect("sys1");

    // TODO: reviewer false positive — cast is required to coerce fn item to fn pointer
    // type so EntryPointFn trait bound is satisfied.
    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    // TODO: reviewer false positive — init_kernel, store_kernel, boot are generated
    // by define_unified_harness! macro, not imported.
    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // Configure P0 with WarmRestart policy and restart hook.
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_fault_policy(FaultPolicy::WarmRestart { max: 1 });
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_on_restart(Some(my_restart_hook as RestartHook));

    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
