//! QEMU proof-of-concept: TIMER0 16-bit edge-time capture mode via PAC typed API.
//!
//! Privileged main() enables TIMER0 clock, MPU grants P0 access to TIMER0 (0x4003_0000).
//! P0 configures 16-bit split (CFG=4), capture + edge-time (TAMR), positive-edge event
//! (CTL), capture interrupt (IMR). Verifies register readback.
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
    boot, mpu,
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, StackStorage as _,
    SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const REGION_SZ: u32 = 1024;
const TIMER0_BASE: u32 = 0x4003_0000;
const TIMER0_SIZE: u32 = 4096;
const MPU_REGION_FLASH: u32 = 0;
const MPU_REGION_RAM: u32 = 1;
const MPU_REGION_GUARD: u32 = 2;
const MPU_REGION_TIMER0: u32 = 3;

static TIMER_STATE: AtomicU32 = AtomicU32::new(0);
static TIMER_CTL: AtomicU32 = AtomicU32::new(0);
static TIMER_CFG: AtomicU32 = AtomicU32::new(0);
static TIMER_TAMR: AtomicU32 = AtomicU32::new(0);
static TIMER_IMR: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: configure TIMER0 in 16-bit edge-time capture mode, read registers, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: MPU-granted access to TIMER0; no concurrent access.
    let t = unsafe { &*tudelft_lm3s6965_pac::TIMER0::ptr() };

    // Disable Timer A before configuration.
    t.ctl.write(|w| w.timer_ctl_taen().clear_bit());

    // 16-bit split mode.
    t.cfg.write(|w| w.timer_cfg().timer_cfg_16_bit());

    // Timer A: capture mode (TAMR=0x3) with edge-time (TACMR=1).
    t.tamr.write(|w| {
        w.timer_tamr_tamr()
            .timer_tamr_tamr_cap()
            .timer_tamr_tacmr()
            .set_bit()
    });

    // Enable capture event interrupt in IMR.
    t.imr.write(|w| w.timer_imr_caeim().set_bit());

    // Enable Timer A with positive-edge event selection (TAEVENT=0 is positive edge).
    t.ctl.write(|w| {
        w.timer_ctl_taen()
            .set_bit()
            .timer_ctl_taevent()
            .timer_ctl_taevent_pos()
    });

    for _ in 0..100u32 {
        cortex_m::asm::nop();
    }

    // Read back all configured registers.
    TIMER_CTL.store(t.ctl.read().bits(), Ordering::Release);
    TIMER_CFG.store(t.cfg.read().bits(), Ordering::Release);
    TIMER_TAMR.store(t.tamr.read().bits(), Ordering::Release);
    TIMER_IMR.store(t.imr.read().bits(), Ordering::Release);
    TIMER_STATE.store(1, Ordering::Release);
    loop {
        // TODO: replace assert! with non-panicking alternative (kexit or loop) for panic-free partition goal.
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p0");
    }
}

/// P1: idle partition (two-partition minimum for round-robin).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        // TODO: replace assert! with non-panicking alternative (kexit or loop) for panic-free partition goal.
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p1");
    }
}

kernel::define_harness!(no_boot, TestConfig, |tick, _k| {
    if TIMER_STATE.load(Ordering::Acquire) == 1 {
        let ctl = TIMER_CTL.load(Ordering::Acquire);
        let cfg = TIMER_CFG.load(Ordering::Acquire);
        let tamr = TIMER_TAMR.load(Ordering::Acquire);
        let imr = TIMER_IMR.load(Ordering::Acquire);
        hprintln!(
            "qemu_pac_timer0_capture: CTL={:#06x} CFG={} TAMR={:#06x} IMR={:#06x}",
            ctl,
            cfg,
            tamr,
            imr
        );

        // CFG=4 (16-bit split), TAMR low 2 bits=0x3 (capture), TAMR bit 2 (TACMR) set (edge-time).
        if cfg != 0x4 {
            hprintln!("FAIL CFG={} expected 4", cfg);
            kernel::kexit!(failure);
        }
        if tamr & 0x03 != 0x03 {
            hprintln!("FAIL TAMR={:#06x} expected capture (0x03)", tamr);
            kernel::kexit!(failure);
        }
        if tamr & 0x04 == 0 {
            hprintln!("FAIL TAMR TACMR not set for edge-time");
            kernel::kexit!(failure);
        }
        // CTL.TAEN (bit 0) set, TAEVENT (bits 3:2)=0 for positive edge.
        if ctl & 0x01 == 0 {
            hprintln!("FAIL CTL.TAEN not set");
            kernel::kexit!(failure);
        }
        if ctl & 0x0C != 0x00 {
            hprintln!("FAIL CTL.TAEVENT expected 0 (pos edge)");
            kernel::kexit!(failure);
        }
        // IMR.CAEIM (bit 2) set for capture event interrupt.
        if imr & 0x04 == 0 {
            hprintln!("FAIL IMR.CAEIM not set");
            kernel::kexit!(failure);
        }

        hprintln!("qemu_pac_timer0_capture: PASS TIMER0 16-bit edge-time capture mode OK");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_timer0_capture: FAIL timeout");
        kernel::kexit!(failure);
    }
});

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) -> Option<()> {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    let flash_sf = mpu::encode_size(256 * 1024)?;
    let flash_rbar = mpu::build_rbar(0x0000_0000, MPU_REGION_FLASH)?;
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    let ram_sf = mpu::encode_size(64 * 1024)?;
    let ram_rbar = mpu::build_rbar(0x2000_0000, MPU_REGION_RAM)?;
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    let guard_sf = mpu::encode_size(16 * 1024)?;
    let guard_rbar = mpu::build_rbar(0x2000_C000, MPU_REGION_GUARD)?;
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    let timer_sf = mpu::encode_size(TIMER0_SIZE)?;
    let timer_rbar = mpu::build_rbar(TIMER0_BASE, MPU_REGION_TIMER0)?;
    let timer_rasr = mpu::build_rasr(timer_sf, mpu::AP_FULL_ACCESS, true, (true, false, true));
    mpu::configure_region(mpu_periph, timer_rbar, timer_rasr);

    // SAFETY: enabling the MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
    Some(())
}

fn fatal_halt() -> ! {
    loop {
        asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let Some(mut p) = cortex_m::Peripherals::take() else {
        fatal_halt();
    };
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("qemu_pac_timer0_capture: start");

    // SAFETY: privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };
    pac.SYSCTL
        .rcgc1
        .modify(|_, w| w.sysctl_rcgc1_timer0().set_bit());
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("qemu_pac_timer0_capture: FATAL MPU config failed");
        fatal_halt();
    }

    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        fatal_halt();
    }

    let timer0_region = [MpuRegion::new(TIMER0_BASE, TIMER0_SIZE, 0)];
    let mut k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            let mem = match ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(base, REGION_SZ, 0),
                kernel::PartitionId::new(i as u32),
            ) {
                Ok(m) => m,
                Err(_) => fatal_halt(),
            };
            if i == 0 {
                match mem.with_peripheral_regions(&timer0_region) {
                    Ok(m) => m,
                    Err(_) => fatal_halt(),
                }
            } else {
                mem
            }
        });
        match Kernel::<TestConfig>::new(sched, &memories) {
            Ok(k) => k,
            Err(_) => fatal_halt(),
        }
    };
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) } {
        Ok(n) => match n {},
        Err(_) => fatal_halt(),
    }
}
