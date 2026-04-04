//! QEMU proof-of-concept: partition exercises PWM0 generator 0 via PAC typed API.
//!
//! Privileged main() enables PWM0 clock via SYSCTL RCGC0 bit 20, grants P0 MPU access
//! to PWM0 (0x4002_8000, 4 KiB). P0 configures generator 0 (_0_LOAD, _0_CMPA, _0_GENA,
//! _0_CTL, ENABLE). QEMU doesn't model PWM0 — test verifies access without fault.
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
const PWM0_BASE: u32 = 0x4002_8000;
const PWM0_SIZE: u32 = 4096;
const MPU_REGION_FLASH: u32 = 0;
const MPU_REGION_RAM: u32 = 1;
const MPU_REGION_GUARD: u32 = 2;
const MPU_REGION_PWM0: u32 = 3;

const PWM_LOAD_VAL: u16 = 0x00FF;
const PWM_CMPA_VAL: u16 = 0x007F;
/// GENA: drive high on LOAD (bits[3:2]=3), drive low on comparator A up (bits[5:4]=2).
const PWM_GENA_VAL: u32 = (3 << 2) | (2 << 4);

/// Readback: [0]=LOAD [1]=CMPA [2]=CTL [3]=GENA [4]=ENABLE [5]=state.
static RB: [AtomicU32; 6] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: configure PWM generator 0, read back registers, set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: MPU-granted access to PWM0; no concurrent access.
    let pwm = unsafe { &*tudelft_lm3s6965_pac::PWM0::ptr() };

    // SAFETY: Valid 16-bit values for LOAD/CMPA registers.
    pwm._0_load
        .write(|w| unsafe { w.pwm_x_load().bits(PWM_LOAD_VAL) });
    pwm._0_cmpa
        .write(|w| unsafe { w.pwm_x_cmpa().bits(PWM_CMPA_VAL) });
    // GENA: drive high on load, drive low on compare A up.
    pwm._0_gena.write(|w| {
        w.pwm_x_gena_actload()
            .pwm_x_gena_actload_one()
            .pwm_x_gena_actcmpau()
            .pwm_x_gena_actcmpau_zero()
    });
    pwm._0_ctl.write(|w| w.pwm_x_ctl_enable().set_bit());
    pwm.enable.write(|w| w.pwm_enable_pwm0en().set_bit());

    for _ in 0..100u32 {
        cortex_m::asm::nop();
    }

    // Read back all configured registers.
    RB[0].store(pwm._0_load.read().bits(), Ordering::Release);
    RB[1].store(pwm._0_cmpa.read().bits(), Ordering::Release);
    RB[2].store(pwm._0_ctl.read().bits(), Ordering::Release);
    RB[3].store(pwm._0_gena.read().bits(), Ordering::Release);
    RB[4].store(pwm.enable.read().bits(), Ordering::Release);
    RB[5].store(1, Ordering::Release);
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
    if RB[5].load(Ordering::Acquire) == 1 {
        let load = RB[0].load(Ordering::Acquire);
        let cmpa = RB[1].load(Ordering::Acquire);
        let ctl = RB[2].load(Ordering::Acquire);
        let gena = RB[3].load(Ordering::Acquire);
        let enable = RB[4].load(Ordering::Acquire);
        hprintln!(
            "qemu_pac_pwm0: LOAD={:#06x} CMPA={:#06x} CTL={:#06x} GENA={:#06x} EN={:#06x}",
            load,
            cmpa,
            ctl,
            gena,
            enable
        );
        // QEMU does not model PWM0 hardware, so readback may be zero.
        // Primary check: P0 accessed PWM0 address space without MemManage fault.
        // When readback is non-zero (real HW), validate configured values.
        if load != 0 && (load & 0xFFFF) as u16 != PWM_LOAD_VAL {
            hprintln!("qemu_pac_pwm0: FAIL LOAD mismatch");
            kernel::kexit!(failure);
        }
        if cmpa != 0 && (cmpa & 0xFFFF) as u16 != PWM_CMPA_VAL {
            hprintln!("qemu_pac_pwm0: FAIL CMPA mismatch");
            kernel::kexit!(failure);
        }
        if ctl != 0 && ctl & 0x01 == 0 {
            hprintln!("qemu_pac_pwm0: FAIL CTL.ENABLE not set");
            kernel::kexit!(failure);
        }
        if gena != 0 && gena != PWM_GENA_VAL {
            hprintln!("qemu_pac_pwm0: FAIL GENA mismatch");
            kernel::kexit!(failure);
        }
        if enable != 0 && enable & 0x01 == 0 {
            hprintln!("qemu_pac_pwm0: FAIL ENABLE.PWM0EN not set");
            kernel::kexit!(failure);
        }
        hprintln!("qemu_pac_pwm0: PASS PWM0 generator 0 output OK");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_pwm0: FAIL timeout");
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

    let pwm_sf = mpu::encode_size(PWM0_SIZE)?;
    let pwm_rbar = mpu::build_rbar(PWM0_BASE, MPU_REGION_PWM0)?;
    let pwm_rasr = mpu::build_rasr(pwm_sf, mpu::AP_FULL_ACCESS, true, (true, false, true));
    mpu::configure_region(mpu_periph, pwm_rbar, pwm_rasr);

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
    hprintln!("qemu_pac_pwm0: start");

    // SAFETY: privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };
    // Enable PWM0 clock via SYSCTL RCGC0 bit 20. The PAC does not expose a
    // typed field for this bit, so we set it via raw modify.
    pac.SYSCTL
        .rcgc0
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) });
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("qemu_pac_pwm0: FATAL MPU config failed");
        fatal_halt();
    }

    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        fatal_halt();
    }

    let pwm0_region = [MpuRegion::new(PWM0_BASE, PWM0_SIZE, 0)];
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
                match mem.with_peripheral_regions(&pwm0_region) {
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
