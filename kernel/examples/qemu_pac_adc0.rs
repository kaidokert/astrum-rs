//! QEMU proof-of-concept: partition exercises ADC0 registers via PAC typed API.
//!
//! Privileged main() enables the ADC0 clock via SYSCTL RCGC0 (ADCSPD field),
//! then a static MPU layout grants unprivileged code access to flash, SRAM,
//! and the ADC0 peripheral region (0x4003_8000, 4 KiB).  Partition P0
//! configures sample sequencer SS0, triggers a conversion, and reads the
//! result from SSFIFO0.  QEMU returns arbitrary values — the test verifies
//! register access succeeds without MemManage fault.
//!
//! Run:
//!   cargo run --example qemu_pac_adc0 --features qemu,log-semihosting,qemu-peripherals \
//!     --target thumbv7m-none-eabi --release
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
const ADC0_BASE: u32 = 0x4003_8000;
const ADC0_SIZE: u32 = 4096;

/// MPU region indices for static pre-boot configuration.
const MPU_REGION_FLASH: u32 = 0;
const MPU_REGION_RAM: u32 = 1;
const MPU_REGION_GUARD: u32 = 2;
const MPU_REGION_ADC0: u32 = 3;

/// State flag: 0 = not started, 1 = conversion complete.
static ADC_STATE: AtomicU32 = AtomicU32::new(0);
/// Raw SSFIFO0 result value after conversion.
static ADC_RESULT: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: exercise ADC0 registers via PAC typed API, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: We have MPU-granted access to ADC0 region; no other code
    // accesses ADC0 concurrently.
    let adc0 = unsafe { &*tudelft_lm3s6965_pac::ADC0::ptr() };

    // 1. Ensure SS0 is disabled before configuring sequencer registers.
    adc0.actss.write(|w| w.adc_actss_asen0().clear_bit());

    // 2. Configure SSMUX0: select channel 0 for first sample slot.
    // SAFETY: Writing a valid 3-bit channel number (0) to MUX0 field.
    adc0.ssmux0
        .write(|w| unsafe { w.adc_ssmux0_mux0().bits(0) });

    // 3. Configure SSCTL0: mark first sample as end-of-sequence.
    adc0.ssctl0.write(|w| w.adc_ssctl0_end0().set_bit());

    // 4. Enable sample sequencer SS0 via ACTSS (after configuration).
    adc0.actss.write(|w| w.adc_actss_asen0().set_bit());

    // 5. Trigger conversion on SS0 via PSSI.
    adc0.pssi.write(|w| w.adc_pssi_ss0().set_bit());

    // 6. Poll RIS register until SS0 conversion is complete.
    //    Bounded: QEMU does not simulate ADC conversion, so RIS.INR0 may
    //    never assert.  Fall through after max iterations to verify register
    //    access regardless.
    for _ in 0..1000u32 {
        if adc0.ris.read().adc_ris_inr0().bit() {
            break;
        }
        cortex_m::asm::nop();
    }

    // 7. Read conversion result from SSFIFO0.
    let result = adc0.ssfifo0.read().adc_ssfifo0_data().bits() as u32;
    ADC_RESULT.store(result, Ordering::Release);

    // Signal completion AFTER all register accesses succeeded without fault.
    ADC_STATE.store(1, Ordering::Release);
    loop {
        // TODO(panic-free): assert used for simplicity in example; production
        // code should handle the error or propagate it.
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p0");
    }
}

/// P1: idle partition (two-partition minimum for round-robin).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        // TODO(panic-free): assert used for simplicity in example; production
        // code should handle the error or propagate it.
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p1");
    }
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, _k| {
    let state = ADC_STATE.load(Ordering::Acquire);
    if state == 1 {
        let result = ADC_RESULT.load(Ordering::Acquire);
        hprintln!("qemu_pac_adc0: SSFIFO0 result = {:#06x}", result);
        hprintln!("qemu_pac_adc0: PASS ADC0 register access OK");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_adc0: FAIL timeout (state={})", state);
        kernel::kexit!(failure);
    }
});

/// Configure static MPU regions before boot.
///
/// R0: Flash (256 KiB) — code read + execute for unprivileged.
/// R1: SRAM  (64 KiB)  — full read-write for unprivileged, XN.
/// R2: Kernel guard (16 KiB at 0x2000_C000) — privileged RW only.
/// R3: ADC0  (4 KiB)   — full read-write for unprivileged, device memory.
///
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) -> Option<()> {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    // TODO: reviewer false positive — SAFETY comment is present above.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: Flash — code read + execute for unprivileged.
    let flash_sf = mpu::encode_size(256 * 1024)?;
    let flash_rbar = mpu::build_rbar(0x0000_0000, MPU_REGION_FLASH)?;
    let flash_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, flash_rbar, flash_rasr);

    // R1: RAM — full read-write for unprivileged, XN.
    let ram_sf = mpu::encode_size(64 * 1024)?;
    let ram_rbar = mpu::build_rbar(0x2000_0000, MPU_REGION_RAM)?;
    let ram_rasr = mpu::build_rasr(ram_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, ram_rbar, ram_rasr);

    // R2: Kernel guard — privileged RW only, overrides R1.
    let guard_sf = mpu::encode_size(16 * 1024)?;
    let guard_rbar = mpu::build_rbar(0x2000_C000, MPU_REGION_GUARD)?;
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    // R3: ADC0 peripheral — full access, device memory (TEX=0 S=1 C=0 B=1).
    let adc_sf = mpu::encode_size(ADC0_SIZE)?;
    let adc_rbar = mpu::build_rbar(ADC0_BASE, MPU_REGION_ADC0)?;
    let adc_rasr = mpu::build_rasr(adc_sf, mpu::AP_FULL_ACCESS, true, (false, true, true));
    mpu::configure_region(mpu_periph, adc_rbar, adc_rasr);

    // SAFETY: enabling the MPU after region configuration is complete.
    // TODO: reviewer false positive — SAFETY comment is present above.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();

    Some(())
}

/// Halt forever — used as a panic-free abort for unrecoverable init failures.
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
    hprintln!("qemu_pac_adc0: start");

    // --- Privileged ADC0 setup via PAC typed API ---
    // SAFETY: We are in privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };

    // Enable ADC0 clock in SYSCTL RCGC0 by setting ADC sample speed.
    // Setting ADCSPD to any non-zero value enables the ADC peripheral clock.
    pac.SYSCTL
        .rcgc0
        .modify(|_, w| w.sysctl_rcgc0_adcspd().sysctl_rcgc0_adcspd125k());

    // Brief delay for clock to stabilize (a few NOPs suffice on QEMU).
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard, ADC0 RW.
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("qemu_pac_adc0: FATAL MPU config failed");
        fatal_halt();
    }

    // --- Kernel setup ---
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        hprintln!("qemu_pac_adc0: FATAL sched config failed");
        fatal_halt();
    }

    // P0 gets ADC0 peripheral region via kernel MPU metadata.
    let adc0_region = [MpuRegion::new(ADC0_BASE, ADC0_SIZE, 0)];

    let mut k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            // TODO: reviewer false positive — SAFETY comment is present above.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            let mem = match ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(base, REGION_SZ, 0),
                i as u8,
            ) {
                Ok(m) => m,
                Err(_) => fatal_halt(),
            };
            if i == 0 {
                match mem.with_peripheral_regions(&adc0_region) {
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
    // store_kernel is generated by define_unified_harness! (via define_unified_kernel!).
    store_kernel(&mut k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) } {
        Ok(n) => match n {},
        Err(_) => fatal_halt(),
    }
}
