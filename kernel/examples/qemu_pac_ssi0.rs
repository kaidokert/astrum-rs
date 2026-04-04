//! QEMU proof-of-concept: partition exercises SSI0 (SPI) registers via PAC typed API.
//!
//! Privileged main() enables SSI0 clock via SYSCTL RCGC1, static MPU grants P0
//! access to SSI0 (0x4000_8000, 4 KiB). P0 configures SSI0 as SPI master
//! (Freescale SPI, 8-bit, prescale 2), writes 0xA5 to DR, reads SR.
//! QEMU SSD0323 OLED on SSI0 accepts the write without fault.
//!
//! Run: cargo run --example qemu_pac_ssi0 --features qemu,log-semihosting,qemu-peripherals \
//!   --target thumbv7m-none-eabi --release
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
const SSI0_BASE: u32 = 0x4000_8000;
const SSI0_SIZE: u32 = 4096;

/// MPU region indices for static pre-boot configuration.
const MPU_REGION_FLASH: u32 = 0;
const MPU_REGION_RAM: u32 = 1;
const MPU_REGION_GUARD: u32 = 2;
const MPU_REGION_SSI0: u32 = 3;

static SPI_STATE: AtomicU32 = AtomicU32::new(0); // 0=pending, 1=done
static SPI_SR: AtomicU32 = AtomicU32::new(0); // raw SR after write

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: exercise SSI0 registers via PAC typed API, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: We have MPU-granted access to SSI0 region; no other code
    // accesses SSI0 concurrently.
    let ssi0 = unsafe { &*tudelft_lm3s6965_pac::SSI0::ptr() };

    // 1. Disable SSI before configuring (CR1.SSE = 0).
    ssi0.cr1.write(|w| w.ssi_cr1_sse().clear_bit());

    // 2. Configure CR0: Freescale SPI frame format, 8-bit data, SCR=0.
    ssi0.cr0.write(|w| {
        w.ssi_cr0_dss()
            .ssi_cr0_dss_8()
            .ssi_cr0_frf()
            .ssi_cr0_frf_moto()
    });

    // 3. Set clock prescale divisor to 2 (minimum valid even value).
    // SAFETY: Writing a valid 8-bit prescale value.
    ssi0.cpsr.write(|w| unsafe { w.ssi_cpsr_cpsdvsr().bits(2) });

    // 4. Enable SSI as master (MS=0 means master, SSE=1 enables port).
    ssi0.cr1
        .write(|w| w.ssi_cr1_ms().clear_bit().ssi_cr1_sse().set_bit());

    // 5. Write a test byte (0xA5) to the data register.
    // SAFETY: Writing a valid 16-bit data value.
    ssi0.dr.write(|w| unsafe { w.ssi_dr_data().bits(0xA5) });

    // 6. Brief delay then read SR for TX FIFO status.
    for _ in 0..100u32 {
        cortex_m::asm::nop();
    }
    let sr_val = ssi0.sr.read().bits();
    SPI_SR.store(sr_val, Ordering::Release);

    // Signal completion AFTER all register accesses succeeded without fault.
    SPI_STATE.store(1, Ordering::Release);
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
    let state = SPI_STATE.load(Ordering::Acquire);
    if state == 1 {
        let sr = SPI_SR.load(Ordering::Acquire);
        hprintln!("qemu_pac_ssi0: SR = {:#06x}", sr);
        // Verify TX FIFO status: TFE (bit 0) should be set (FIFO empty after
        // transmit completes) and BSY (bit 4) should be clear.
        let tfe = sr & (1 << 0) != 0;
        let bsy = sr & (1 << 4) != 0;
        if !tfe || bsy {
            hprintln!("qemu_pac_ssi0: FAIL SR check: TFE={} BSY={}", tfe, bsy);
            kernel::kexit!(failure);
        }
        hprintln!("qemu_pac_ssi0: PASS SSI0 SPI register access OK");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_ssi0: FAIL timeout (state={})", state);
        kernel::kexit!(failure);
    }
});

/// Static MPU: R0=Flash RX, R1=RAM RW, R2=kernel guard, R3=SSI0 device RW.
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) -> Option<()> {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
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

    // R3: SSI0 peripheral — full access, device memory (TEX=0 S=1 C=0 B=1).
    let ssi_sf = mpu::encode_size(SSI0_SIZE)?;
    let ssi_rbar = mpu::build_rbar(SSI0_BASE, MPU_REGION_SSI0)?;
    let ssi_rasr = mpu::build_rasr(ssi_sf, mpu::AP_FULL_ACCESS, true, (true, false, true));
    mpu::configure_region(mpu_periph, ssi_rbar, ssi_rasr);

    // SAFETY: enabling the MPU after region configuration is complete.
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
    hprintln!("qemu_pac_ssi0: start");

    // SAFETY: privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };

    // Enable SSI0 clock in SYSCTL RCGC1 (bit 4).
    pac.SYSCTL
        .rcgc1
        .modify(|_, w| w.sysctl_rcgc1_ssi0().set_bit());

    cortex_m::asm::nop();
    cortex_m::asm::nop();

    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard, SSI0 RW.
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("qemu_pac_ssi0: FATAL MPU config failed");
        fatal_halt();
    }

    // --- Kernel setup ---
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        hprintln!("qemu_pac_ssi0: FATAL sched config failed");
        fatal_halt();
    }

    // P0 gets SSI0 peripheral region via kernel MPU metadata.
    let ssi0_region = [MpuRegion::new(SSI0_BASE, SSI0_SIZE, 0)];

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
                match mem.with_peripheral_regions(&ssi0_region) {
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
