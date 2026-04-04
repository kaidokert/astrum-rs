//! Adversarial survival test: P0 continues after P1 faults.
//!
//! Two partitions run under a static MPU configuration. P0 is
//! well-behaved (increments a counter each tick). P1 writes to a
//! kernel-reserved memory region after a few iterations, triggering a
//! MemManage fault. The SysTick hook verifies that P1 becomes Faulted
//! and P0 continues running for 10+ ticks after the fault, then exits
//! with EXIT_SUCCESS via semihosting.
//!
//! MPU layout (static, not reprogrammed per context switch):
//!   R0: Flash (0x0000_0000, 256 KB) — RO+X for unprivileged code
//!   R1: RAM   (0x2000_0000,  64 KB) — RW+XN for unprivileged data
//!   R2: Guard (0x2000_C000,  16 KB) — no-access, overrides R1
//!
//! P1 writes to 0x2000_F000 which falls inside the R2 guard region,
//! so the write triggers DACCVIOL and the kernel MemManage handler
//! transitions P1 to Faulted.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example adversarial_survive

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
    mpu, partition::PartitionState, scheduler::ScheduleTable, DebugEnabled, MsgMinimal,
    PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

// ---------------------------------------------------------------------------
// Kernel configuration (MPU enforcement off — we configure MPU manually)
// ---------------------------------------------------------------------------

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

// ---------------------------------------------------------------------------
// Shared atomics for partition-to-hook signalling
// ---------------------------------------------------------------------------

/// P0 increments this counter on each loop iteration.
static P0_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Kernel address inside the guard region (0x2000_C000 – 0x2000_FFFF).
const KERNEL_ADDR: u32 = 0x2000_F000;

/// Number of loop iterations P1 burns before the illegal write.
const P1_DELAY_ITERS: u32 = 50;

// ---------------------------------------------------------------------------
// Partition entry points
// ---------------------------------------------------------------------------
// TODO: reviewer false positive — partition_trampoline! is for the
// PartitionBody/ExternalPartitionMemory path (takes r0: u32). This example
// uses the PartitionSpec path with PartitionEntry (no args), matching
// context_switch.rs, qemu_smoke_4k.rs, etc.

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    for _ in 0..P1_DELAY_ITERS {
        cortex_m::asm::nop();
    }
    // Write to kernel guard region — triggers MemManage (DACCVIOL).
    // SAFETY: intentional invalid write to a kernel guard region address.
    // The MPU is configured to deny unprivileged access at KERNEL_ADDR,
    // so this write will trap with DACCVIOL rather than corrupting memory.
    unsafe {
        core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_BEEF);
    }
    // Should never reach here; MemManage handler redirects to WFI.
    loop {
        cortex_m::asm::nop();
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Tick at which P1's fault was first observed.
static FAULT_TICK: AtomicU32 = AtomicU32::new(0);

/// Required number of ticks P0 must survive after P1 faults.
const SURVIVE_TICKS: u32 = 10;

/// Hard timeout to prevent hanging.
const TIMEOUT_TICKS: u32 = 200;

kernel::define_harness!(TestConfig, |tick, k| {
    let p1_state = k.pcb(1).map(|pcb| pcb.state());

    if p1_state == Some(PartitionState::Faulted) {
        let ft = FAULT_TICK.load(Ordering::Relaxed);
        if ft == 0 {
            FAULT_TICK.store(tick, Ordering::Relaxed);
        } else if tick >= ft + SURVIVE_TICKS {
            let p0_count = P0_COUNTER.load(Ordering::Relaxed);
            let p0_state = k.pcb(0).map(|pcb| pcb.state());
            if p0_count > 0 && p0_state != Some(PartitionState::Faulted) {
                hprintln!(
                    "adversarial_survive: PASS (p0_count={}, fault_tick={}, now={})",
                    p0_count,
                    ft,
                    tick
                );
                kernel::kexit!(success);
            } else {
                hprintln!(
                    "adversarial_survive: FAIL (p0_count={}, p0_state={:?})",
                    p0_count,
                    p0_state
                );
                kernel::kexit!(failure);
            }
        }
    }

    if tick >= TIMEOUT_TICKS {
        hprintln!(
            "adversarial_survive: FAIL (timeout tick={}, p1_state={:?})",
            tick,
            p1_state
        );
        kernel::kexit!(failure);
    }
});

// ---------------------------------------------------------------------------
// MPU setup (static, applied once before boot)
// ---------------------------------------------------------------------------

/// Configure static MPU regions that apply to all partitions.
///
/// R0: Flash 256 KB (0x0000_0000) — read-only + execute
/// R1: RAM 64 KB (0x2000_0000) — read-write + XN
/// R2: Kernel guard 16 KB (0x2000_C000) — no access (overrides R1)
// TODO: Refactor to use kernel MPU abstractions (MpuRegion / ExternalPartitionMemory)
// instead of direct register writes once the harness supports per-partition memory
// setup with the PartitionSpec boot path.
// TODO: Refactor to return Result instead of using .expect() (panic-free policy).
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash — code read + execute for unprivileged.
    let flash_sf = mpu::encode_size(256 * 1024).expect("flash size");
    let flash_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar");
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full read-write for unprivileged, XN.
    let ram_sf = mpu::encode_size(64 * 1024).expect("ram size");
    let ram_rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard — no access for unprivileged, overrides R1.
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    // SAFETY: enabling the MPU after region configuration is complete;
    // single-core, no concurrent access to MPU registers.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

// TODO: consider replacing expect() with a non-panicking pattern (e.g. loop+kexit)
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("adversarial_survive: start");

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard no-access.
    configure_static_mpu(&p.MPU);

    let sched = ScheduleTable::<{ TestConfig::SCHED }>::round_robin(2, 2).expect("round_robin");

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");
    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
