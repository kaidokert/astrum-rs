//! QEMU integration test: restart hook fires on cold restart.
//!
//! P0 is configured with `FaultPolicy::ColdRestart { max: 1 }` and a restart
//! hook.  On first run P0 faults (writes to kernel guard region).  The
//! MemManage handler auto-restarts P0, invoking the hook with `warm=false`.
//! Since cold restart zeros the partition's data region, the hook stores its
//! result in global statics (outside partition RAM).  On the second run P0
//! reads the flag and marks verification complete.  The SysTick hook observes
//! the outcome and prints PASS/FAIL.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example restart_hook_cold_test
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
// Shared atomics (global statics — survive cold restart)
// ---------------------------------------------------------------------------

/// Set to `true` by the restart hook — proves the hook fired.
static HOOK_FIRED: AtomicBool = AtomicBool::new(false);

/// Set to `true` by the restart hook when `warm == false` (cold restart).
static HOOK_COLD: AtomicBool = AtomicBool::new(false);

/// Incremented by P0 at entry — distinguishes first run from restart.
static P0_RUN_COUNT: AtomicU32 = AtomicU32::new(0);

/// Set to `true` by P0 on the second run if all checks pass.
static P0_VERIFIED: AtomicBool = AtomicBool::new(false);

/// Address of the signature written in run 0 (inside partition stack).
static SIGNATURE_ADDR: AtomicU32 = AtomicU32::new(0);

/// Set to `true` when the signature location was verified as zeroed after cold restart.
static DATA_ZEROED: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Restart hook (runs in privileged/kernel context)
// ---------------------------------------------------------------------------

fn my_restart_hook(_pid: usize, warm: bool) {
    HOOK_FIRED.store(true, Ordering::SeqCst);
    HOOK_COLD.store(!warm, Ordering::SeqCst);
}

// ---------------------------------------------------------------------------
// Partition entry points
// ---------------------------------------------------------------------------

const KERNEL_ADDR: u32 = 0x2000_F000;

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    let run = P0_RUN_COUNT.fetch_add(1, Ordering::SeqCst);
    if run == 0 {
        // Write a signature deep in the partition stack so we can verify
        // cold restart zeroed it.  The pad array pushes the address well
        // below run-1's shallow stack frame.
        let mut pad = [0u32; 32];
        // SAFETY: Volatile write prevents the compiler from optimising away the store.
        unsafe { core::ptr::write_volatile(&mut pad[0] as *mut u32, 0xCAFE_BABE) };
        SIGNATURE_ADDR.store(&pad[0] as *const u32 as u32, Ordering::SeqCst);

        // First run: trigger a MemManage fault (DACCVIOL).
        // SAFETY: Deliberately writing to the kernel guard region.
        unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0001) };
    } else {
        // Verify cold restart zeroed partition RAM at the signature address.
        let addr = SIGNATURE_ADDR.load(Ordering::SeqCst);
        if addr != 0 {
            // SAFETY: addr points into the partition stack region which is
            // valid memory; we only read one u32.
            let val = unsafe { core::ptr::read_volatile(addr as *const u32) };
            DATA_ZEROED.store(val == 0, Ordering::SeqCst);
        }

        // Cold restart: check hook flags AND data integrity.
        if HOOK_FIRED.load(Ordering::SeqCst)
            && HOOK_COLD.load(Ordering::SeqCst)
            && DATA_ZEROED.load(Ordering::SeqCst)
        {
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
        hprintln!("restart_hook_cold_test: PASS");
        kernel::kexit!(success);
    }

    if tick >= TIMEOUT_TICKS {
        let run = P0_RUN_COUNT.load(Ordering::SeqCst);
        let fired = HOOK_FIRED.load(Ordering::SeqCst);
        let cold = HOOK_COLD.load(Ordering::SeqCst);
        let zeroed = DATA_ZEROED.load(Ordering::SeqCst);
        hprintln!(
            "restart_hook_cold_test: FAIL timeout (run={}, fired={}, cold={}, zeroed={})",
            run,
            fired,
            cold,
            zeroed
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
    let flash_sf = mpu::encode_size(256 * 1024).expect("flash size"); // TODO(panic-free)
    let flash_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar"); // TODO(panic-free)
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full access for partitions.
    let ram_sf = mpu::encode_size(64 * 1024).expect("ram size"); // TODO(panic-free)
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar"); // TODO(panic-free)
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard region — privileged-only (overrides R1).
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size"); // TODO(panic-free)
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar"); // TODO(panic-free)
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
    hprintln!("restart_hook_cold_test: start");

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

    // Cast coerces fn item → fn pointer so EntryPointFn trait bound is satisfied.
    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    // init_kernel, store_kernel, boot are generated by define_unified_harness! macro.
    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    // Configure P0 with ColdRestart policy and restart hook.
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_fault_policy(FaultPolicy::ColdRestart { max: 1 });
    k.pcb_mut(0)
        .expect("pcb_mut(0)")
        .set_on_restart(Some(my_restart_hook as RestartHook));

    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
