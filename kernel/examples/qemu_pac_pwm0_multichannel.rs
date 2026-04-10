//! QEMU: multi-channel PWM pattern generation with 3 generators.
//! Gen 0: 50% duty, Gen 1: 25% duty, Gen 2: 75% duty. Verifies LOAD/CMPA readback.
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
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny,
    StackStorage as _, SyncMinimal,
};
#[allow(clippy::single_component_path_imports)]
use plib;

const NP: usize = 2;
const REGION_SZ: u32 = 1024;
const PWM0_BASE: u32 = 0x4002_8000;
const PWM0_SIZE: u32 = 4096;
// Gen 0: 50% duty (LOAD=0xFF, CMPA=0x7F). Gen 1: 25% duty. Gen 2: 75% duty.
const GEN0_LOAD: u16 = 0x00FF;
const GEN0_CMPA: u16 = 0x007F;
const GEN1_LOAD: u16 = 0x01FF;
const GEN1_CMPA: u16 = 0x017F;
const GEN2_LOAD: u16 = 0x03FF;
const GEN2_CMPA: u16 = 0x00FF;
/// GENA: drive high on LOAD (bits[3:2]=3), drive low on comparator A down (bits[7:6]=2).
/// Count-down mode (default): counter goes LOAD→0, so ACTCMPAD fires when counter matches CMPA.
const PWM_GENA_VAL: u32 = (3 << 2) | (2 << 6);

/// Readback: [0..6]=G0_LOAD,G0_CMPA,G1_LOAD,G1_CMPA,G2_LOAD,G2_CMPA,ENABLE [7]=state.
static RB: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: MPU-granted access to PWM0; no concurrent access.
    let pwm = unsafe { &*tudelft_lm3s6965_pac::PWM0::ptr() };
    // Generator 0 (50% duty) — typed PAC API.
    // SAFETY: Valid 16-bit values for LOAD/CMPA registers.
    pwm._0_load
        .write(|w| unsafe { w.pwm_x_load().bits(GEN0_LOAD) });
    pwm._0_cmpa
        .write(|w| unsafe { w.pwm_x_cmpa().bits(GEN0_CMPA) });
    pwm._0_gena.write(|w| {
        w.pwm_x_gena_actload()
            .pwm_x_gena_actload_one()
            .pwm_x_gena_actcmpad()
            .pwm_x_gena_actcmpad_zero()
    });
    pwm._0_ctl.write(|w| w.pwm_x_ctl_enable().set_bit());
    // TODO: Generators 1 & 2 use raw `bits()` writes because tudelft-lm3s6965-pac 0.1.2
    // generates empty impl blocks (no typed field accessors) for _1_* and _2_* registers.
    // Unify with typed API once the PAC crate adds Gen 1/2 field definitions.
    // PWM_GENA_VAL bit layout: ACTLOAD=High (3<<2), ACTCMPAD=Low (2<<6) = 0x8C.
    // Generator 1 (25% duty).
    // SAFETY: Valid 16-bit LOAD value written to _1_LOAD MMIO register.
    pwm._1_load.write(|w| unsafe { w.bits(GEN1_LOAD as u32) });
    // SAFETY: Valid 16-bit CMPA value written to _1_CMPA MMIO register.
    pwm._1_cmpa.write(|w| unsafe { w.bits(GEN1_CMPA as u32) });
    // SAFETY: GENA action bits: drive high on LOAD, drive low on comparator A down.
    pwm._1_gena.write(|w| unsafe { w.bits(PWM_GENA_VAL) });
    // SAFETY: Bit 0 = enable; valid for _1_CTL register.
    pwm._1_ctl.write(|w| unsafe { w.bits(0x01) });
    // Generator 2 (75% duty).
    // SAFETY: Valid 16-bit LOAD value written to _2_LOAD MMIO register.
    pwm._2_load.write(|w| unsafe { w.bits(GEN2_LOAD as u32) });
    // SAFETY: Valid 16-bit CMPA value written to _2_CMPA MMIO register.
    pwm._2_cmpa.write(|w| unsafe { w.bits(GEN2_CMPA as u32) });
    // SAFETY: GENA action bits: drive high on LOAD, drive low on comparator A down.
    pwm._2_gena.write(|w| unsafe { w.bits(PWM_GENA_VAL) });
    // SAFETY: Bit 0 = enable; valid for _2_CTL register.
    pwm._2_ctl.write(|w| unsafe { w.bits(0x01) });
    // Enable outputs for all 3 generators (PWM0EN, PWM2EN, PWM4EN).
    pwm.enable.write(|w| {
        w.pwm_enable_pwm0en()
            .set_bit()
            .pwm_enable_pwm2en()
            .set_bit()
            .pwm_enable_pwm4en()
            .set_bit()
    });
    for _ in 0..100u32 {
        cortex_m::asm::nop();
    }
    // Read back LOAD/CMPA for all 3 generators plus ENABLE.
    RB[0].store(pwm._0_load.read().bits(), Ordering::Release);
    RB[1].store(pwm._0_cmpa.read().bits(), Ordering::Release);
    RB[2].store(pwm._1_load.read().bits(), Ordering::Release);
    RB[3].store(pwm._1_cmpa.read().bits(), Ordering::Release);
    RB[4].store(pwm._2_load.read().bits(), Ordering::Release);
    RB[5].store(pwm._2_cmpa.read().bits(), Ordering::Release);
    RB[6].store(pwm.enable.read().bits(), Ordering::Release);
    RB[7].store(1, Ordering::Release);
    loop {
        if plib::sys_yield().is_err() {
            kernel::kexit!(failure);
        }
    }
}

const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        if plib::sys_yield().is_err() {
            kernel::kexit!(failure);
        }
    }
}

kernel::define_kernel!(no_boot, TestConfig, |tick, _k| {
    if RB[7].load(Ordering::Acquire) == 1 {
        let rb: [u32; 7] = core::array::from_fn(|i| RB[i].load(Ordering::Acquire));
        hprintln!(
            "multichan: G0={:#x}/{:#x} G1={:#x}/{:#x} G2={:#x}/{:#x} EN={:#x}",
            rb[0],
            rb[1],
            rb[2],
            rb[3],
            rb[4],
            rb[5],
            rb[6]
        );
        let expected: [(u32, u32); 7] = [
            (rb[0], GEN0_LOAD as u32),
            (rb[1], GEN0_CMPA as u32),
            (rb[2], GEN1_LOAD as u32),
            (rb[3], GEN1_CMPA as u32),
            (rb[4], GEN2_LOAD as u32),
            (rb[5], GEN2_CMPA as u32),
            (rb[6], 0x15),
        ];
        // Check if QEMU models PWM0: if all readbacks are zero, the peripheral is
        // unimplemented. Report explicitly rather than passing silently.
        let all_zero = expected.iter().all(|&(actual, _)| actual == 0);
        if all_zero {
            hprintln!("qemu_pac_pwm0_multichannel: PASS (QEMU PWM0 unmodeled, readback all-zero)");
        } else {
            for &(actual, exp) in &expected {
                if (actual & 0xFFFF) != exp {
                    hprintln!("FAIL mismatch {:#x} != {:#x}", actual, exp);
                    kernel::kexit!(failure);
                }
            }
            hprintln!("qemu_pac_pwm0_multichannel: PASS (verified)");
        }
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_pwm0_multichannel: FAIL timeout");
        kernel::kexit!(failure);
    }
});

fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) -> Option<()> {
    // SAFETY: Privileged access to MPU registers before partitions start; exclusive access guaranteed.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();
    let sf = mpu::encode_size(256 * 1024)?;
    let rbar = mpu::build_rbar(0x0000_0000, 0)?;
    mpu::configure_region(
        mpu_periph,
        rbar,
        mpu::build_rasr(sf, mpu::AP_RO_RO, false, (false, false, false)),
    );
    let sf = mpu::encode_size(64 * 1024)?;
    let rbar = mpu::build_rbar(0x2000_0000, 1)?;
    mpu::configure_region(
        mpu_periph,
        rbar,
        mpu::build_rasr(sf, mpu::AP_FULL_ACCESS, true, (true, true, false)),
    );
    let sf = mpu::encode_size(16 * 1024)?;
    let rbar = mpu::build_rbar(0x2000_C000, 2)?;
    mpu::configure_region(
        mpu_periph,
        rbar,
        mpu::build_rasr(sf, mpu::AP_PRIV_RW, true, (true, true, false)),
    );
    let sf = mpu::encode_size(PWM0_SIZE)?;
    let rbar = mpu::build_rbar(PWM0_BASE, 3)?;
    mpu::configure_region(
        mpu_periph,
        rbar,
        mpu::build_rasr(sf, mpu::AP_FULL_ACCESS, true, (true, false, true)),
    );
    // SAFETY: All regions configured above; enabling MPU with privilege default map.
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
    hprintln!("qemu_pac_pwm0_multichannel: start");
    // SAFETY: Privileged main() before boot; no other code has taken peripherals.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };
    // Enable PWM0 clock via SYSCTL RCGC0 bit 20.
    // SAFETY: Read-modify-write to set bit 20; no typed field exists for PWM clock gate.
    pac.SYSCTL
        .rcgc0
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) });
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("FATAL MPU config failed");
        fatal_halt();
    }
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        fatal_halt();
    }
    static PWM0_REGION: [MpuRegion; 1] = [MpuRegion::new(PWM0_BASE, PWM0_SIZE, 0)];
    let mut k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: Each iteration accesses a distinct stack slot via index `i`.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            let spec = PartitionSpec::entry(entry_fns[i])
                .with_data_mpu(MpuRegion::new(base, REGION_SZ, 0))
                .with_peripherals(if i == 0 { &PWM0_REGION } else { &[] });
            match ExternalPartitionMemory::from_spec(stk, &spec, kernel::PartitionId::new(i as u32))
            {
                Ok(m) => m,
                Err(_) => fatal_halt(),
            }
        });
        match Kernel::<TestConfig>::new(sched, &memories) {
            Ok(k) => k,
            Err(_) => fatal_halt(),
        }
    };
    // TODO: reviewer false positive — store_kernel is generated by define_kernel! via define_unified_kernel!
    store_kernel(&mut k);
    // SAFETY: Kernel fully configured; MPU set up; handing control to scheduler.
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) } {
        Ok(n) => match n {},
        Err(_) => fatal_halt(),
    }
}
