//! Adversarial Multi-Attack Test — SAME51
//!
//! P0: well-behaved counter (the survivor)
//! P1: writes to guard region → DACCVIOL (MemManage)
//! P2: reads from guard region → DACCVIOL (MemManage)
//! P3: reads from guard region (different offset) → DACCVIOL (MemManage)
//!
//! Static MPU layout:
//!   R0: Flash 1 MB (0x0000_0000) — RO+X unprivileged
//!   R1: RAM 256 KB (0x2000_0000) — RW+XN unprivileged
//!   R2: Guard 32 KB (0x2003_8000) — priv-only (top of SRAM)
//!
//! Success: P0 count > 200 while P1, P2, P3 are all Faulted.
//!
//! Build: cd same51_curiosity && cargo build --example adversarial_test --features kernel-example

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    mpu, partition::PartitionState,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    PartitionEntry,
    {DebugEnabled, MsgMinimal, Partitions4, PortsTiny, SyncMinimal},
};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 4;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

kernel::kernel_config!(
    AdvCfg[AlignedStack2K]<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
        core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    }
);

unsafe extern "C" fn dummy_irq() {}
kernel::bind_interrupts!(AdvCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

static P0_COUNTER: AtomicU32 = AtomicU32::new(0);
static FAULT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Guard region in mid-SRAM (avoid top — MSP stack + kernel live there).
const GUARD_BASE: u32 = 0x2001_0000;
const GUARD_SIZE: u32 = 32 * 1024;
const P1_TARGET: u32 = 0x2001_0800;
const P2_TARGET: u32 = 0x2001_0400;
const P3_TARGET: u32 = 0x2001_0C00;

const ATTACK_DELAY: u32 = 100;
const TIMEOUT_TICKS: u32 = 500;

kernel::define_kernel!(AdvCfg, |tick, k| {
    let mut faulted = 0u32;
    for pid in 1..4u32 {
        if k.pcb(pid as usize).map(|p| p.state()) == Some(PartitionState::Faulted) {
            faulted += 1;
        }
    }
    let prev = FAULT_COUNT.load(Ordering::Relaxed);
    if faulted > prev {
        FAULT_COUNT.store(faulted, Ordering::Relaxed);
        let p0_state = k.pcb(0).map(|p| p.state());
        rprintln!("[{:5}ms] New fault! total={}/3 P0={:?}", tick, faulted, p0_state);
    }

    if tick % 200 == 0 {
        let p0 = P0_COUNTER.load(Ordering::Acquire);
        let states: [Option<PartitionState>; 4] = core::array::from_fn(|i| {
            k.pcb(i).map(|p| p.state())
        });
        rprintln!(
            "[{:5}ms] P0={} faulted={}/3 states=[{:?}, {:?}, {:?}, {:?}]",
            tick, p0, faulted, states[0], states[1], states[2], states[3]
        );
    }

    if faulted == 3 {
        let p0 = P0_COUNTER.load(Ordering::Acquire);
        let p0_ok = k.pcb(0).map(|p| p.state()) != Some(PartitionState::Faulted);
        if p0 > 200 && p0_ok {
            rprintln!(
                "SUCCESS: adversarial test passed! P0 survived with count={}, all 3 attackers Faulted",
                p0
            );
        }
    }

    if tick >= TIMEOUT_TICKS {
        rprintln!("FAIL: timeout at tick={}, faulted={}/3", tick, faulted);
    }
});

// ── P0: well-behaved survivor ──
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    loop {
        P0_COUNTER.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

// ── P1: write to guard region ──
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    for _ in 0..ATTACK_DELAY { cortex_m::asm::nop(); }
    unsafe { core::ptr::write_volatile(P1_TARGET as *mut u32, 0xDEAD_0001); }
    loop { cortex_m::asm::wfi(); }
}

// ── P2: read from guard region ──
const _: PartitionEntry = p2_entry;
extern "C" fn p2_entry() -> ! {
    for _ in 0..(ATTACK_DELAY * 2) { cortex_m::asm::nop(); }
    let _val = unsafe { core::ptr::read_volatile(P2_TARGET as *const u32) };
    loop { cortex_m::asm::wfi(); }
}

// ── P3: read from guard region (different offset) ──
const _: PartitionEntry = p3_entry;
extern "C" fn p3_entry() -> ! {
    for _ in 0..(ATTACK_DELAY * 3) { cortex_m::asm::nop(); }
    let _val = unsafe { core::ptr::read_volatile(P3_TARGET as *const u32) };
    loop { cortex_m::asm::wfi(); }
}

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb(); asm::isb();

    // R0: Flash 1 MB (0x0000_0000) — RO+X unprivileged
    let sf = mpu::encode_size(1024 * 1024).expect("flash");
    let rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    // R1: RAM 256 KB (0x2000_0000) — RW+XN unprivileged
    let sf = mpu::encode_size(256 * 1024).expect("ram");
    let rbar = mpu::build_rbar(0x2000_0000, 1).expect("ram rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    // R2: Guard 32 KB (top of SRAM) — priv-only
    let sf = mpu::encode_size(GUARD_SIZE).expect("guard");
    let rbar = mpu::build_rbar(GUARD_BASE, 2).expect("guard rbar");
    let rasr = mpu::build_rasr(sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, rbar, rasr);

    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb(); asm::isb();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Adversarial Multi-Attack Test — SAME51 ===");
    rprintln!("P0: survivor | P1: write guard | P2: read guard | P3: read guard");
    rprintln!("Guard region: 0x{:08X} ({} KB)\n", GUARD_BASE, GUARD_SIZE / 1024);

    p.SCB.enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    configure_static_mpu(&p.MPU);

    let mut sched = ScheduleTable::<{ AdvCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add(ScheduleEntry::new(2, 2)).expect("sched P2");
    sched.add(ScheduleEntry::new(3, 2)).expect("sched P3");
    sched.add_system_window(1).expect("sys_window");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1, ref mut s2, ref mut s3] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, p0_entry as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0").with_code_mpu_region(code_mpu).expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, p1_entry as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem1").with_code_mpu_region(code_mpu).expect("code1"),
        ExternalPartitionMemory::from_aligned_stack(s2, p2_entry as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem2").with_code_mpu_region(code_mpu).expect("code2"),
        ExternalPartitionMemory::from_aligned_stack(s3, p3_entry as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem3").with_code_mpu_region(code_mpu).expect("code3"),
    ];

    let mut k = Kernel::<AdvCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] 4 partitions, MPU + MemManage enabled. Booting...\n");
    match boot(p).expect("boot") {}
}
