//! QEMU test: N-1 faulted then last partition faults → safe idle.
//!
//! P0 faults immediately (writes to kernel guard region).
//! P1 runs a delay loop then faults.
//! SysTick hook verifies `all_runnable_faulted()` is false while P1 runs,
//! then `enter_safe_idle()` confirms the all-faulted transition after P1 faults.
//!
//! Run: cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example last_partition_faults_test
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
    mpu, partition::PartitionState, scheduler::ScheduleTable, DebugEnabled, MsgMinimal,
    PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};

kernel::kernel_config!(
    TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
);

const KERNEL_ADDR: u32 = 0x2000_F000;
const P1_DELAY_ITERS: u32 = 50_000;

// Shared atomics for partition-to-hook signalling
static P0_FAULTED_CHECKED: AtomicBool = AtomicBool::new(false);
static PHASE1_TICK: AtomicU32 = AtomicU32::new(0);
static P1_COUNTER: AtomicU32 = AtomicU32::new(0);
static PHASE2_DONE: AtomicBool = AtomicBool::new(false);
const STABLE_TICKS: u32 = 3;

// Partition entry points

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: Deliberately writing to the kernel guard region to trigger a
    // MemManage fault (DACCVIOL). The address is within the MPU-protected
    // kernel region configured by configure_static_mpu.
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0000) };
    loop {
        asm::nop();
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    for _ in 0..P1_DELAY_ITERS {
        P1_COUNTER.fetch_add(1, Ordering::Relaxed);
        asm::nop();
    }
    // SAFETY: Deliberately writing to the kernel guard region to trigger a
    // MemManage fault (DACCVIOL) after the delay loop completes.
    unsafe { core::ptr::write_volatile(KERNEL_ADDR as *mut u32, 0xDEAD_0001) };
    loop {
        asm::nop();
    }
}

// SysTick hook

const TIMEOUT_TICKS: u32 = 300;

kernel::define_kernel!(TestConfig, |tick, k| {
    let p0_state = k.pcb(0).map(|pcb| pcb.state());
    let p1_state = k.pcb(1).map(|pcb| pcb.state());
    let all_faulted = k.all_runnable_faulted();

    // Phase 1: P0 faulted, P1 still running → all_runnable_faulted must be false
    if !P0_FAULTED_CHECKED.load(Ordering::Relaxed) {
        if p0_state == Some(PartitionState::Faulted) {
            if p1_state == Some(PartitionState::Faulted) {
                hprintln!("last_partition_faults_test: FAIL P1 faulted too early");
                kernel::kexit!(failure);
            }
            if all_faulted {
                hprintln!(
                    "last_partition_faults_test: FAIL all_runnable_faulted true with P1 running"
                );
                kernel::kexit!(failure);
            }
            hprintln!("all_runnable_faulted=false OK");
            P0_FAULTED_CHECKED.store(true, Ordering::Relaxed);
            PHASE1_TICK.store(tick, Ordering::Relaxed);
        }
    } else if !PHASE2_DONE.load(Ordering::Relaxed) {
        // Phase 2: Stability window — confirm P1 is still alive
        let phase1_at = PHASE1_TICK.load(Ordering::Relaxed);
        if tick >= phase1_at + STABLE_TICKS {
            let p1_count = P1_COUNTER.load(Ordering::Relaxed);
            if p1_count == 0 {
                hprintln!("last_partition_faults_test: FAIL P1 never ran");
                kernel::kexit!(failure);
            }
            if p1_state == Some(PartitionState::Faulted) {
                hprintln!("last_partition_faults_test: FAIL P1 faulted before stable check");
                kernel::kexit!(failure);
            }
            if all_faulted {
                hprintln!("last_partition_faults_test: FAIL all_runnable_faulted true (stable)");
                kernel::kexit!(failure);
            }
            hprintln!("intermediate OK");
            PHASE2_DONE.store(true, Ordering::Relaxed);
        }
    } else {
        // Phase 3: Wait for P1 to fault → verify all_runnable_faulted and exit.
        if p1_state == Some(PartitionState::Faulted) {
            if !all_faulted {
                hprintln!(
                    "last_partition_faults_test: FAIL both faulted but all_runnable_faulted=false"
                );
                kernel::kexit!(failure);
            }
            hprintln!("last_partition_faults_test: PASS");
            kernel::kexit!(success);
        }
    }

    if tick >= TIMEOUT_TICKS {
        hprintln!("last_partition_faults_test: FAIL timeout");
        kernel::kexit!(failure);
    }
});

// MPU setup

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: Single-core, called before scheduler starts — exclusive MPU access.
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

    // SAFETY: Enabling the MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// Boot

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!("last_partition_faults_test: start");

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

    let parts: [PartitionSpec; TestConfig::N] = [
        PartitionSpec::new(p0_entry as PartitionEntry, 0),
        PartitionSpec::new(p1_entry as PartitionEntry, 0),
    ];
    let mut k = init_kernel(sched, &parts).expect("init_kernel");

    store_kernel(&mut k);

    match boot(p).expect("boot") {}
}
