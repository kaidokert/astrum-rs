//! QEMU proof-of-concept: partition exercises I2C0 master registers via PAC typed API.
//!
//! Privileged main() enables the I2C0 clock via SYSCTL RCGC1 and configures I2C0
//! for master mode.  A static MPU layout grants unprivileged code access to flash,
//! SRAM, and the I2C0 peripheral region (0x4002_0000, 4 KiB).  Partition P0
//! sets a slave address, writes data, and initiates a START+RUN transaction.
//! Since QEMU has no I2C slave attached, the partition verifies register access
//! succeeds without MemManage fault and observes error/no-ACK status.
//!
//! Run:
//!   cargo run --example qemu_pac_i2c0 --features qemu,log-semihosting,qemu-peripherals \
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
const I2C0_BASE: u32 = 0x4002_0000;
const I2C0_SIZE: u32 = 4096;

/// MPU region indices for static pre-boot configuration.
/// These are overwritten by `boot_preconfigured()` → `precompute_mpu_cache()` before
/// the scheduler starts, so they only govern privileged main() setup.
const MPU_REGION_FLASH: u32 = 0;
const MPU_REGION_RAM: u32 = 1;
const MPU_REGION_GUARD: u32 = 2;
const MPU_REGION_I2C0: u32 = 3;

/// State flag: 0 = not started, 1 = transaction complete.
static I2C_STATE: AtomicU32 = AtomicU32::new(0);
/// Raw MCS status register value after transaction attempt.
static I2C_MCS_RAW: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: exercise I2C0 master registers via PAC typed API, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: We have MPU-granted access to I2C0 region; no other code
    // accesses I2C0 concurrently.
    // PAC register accessors (read/write) use volatile internally, so no
    // additional volatile or memory barriers are needed beyond what the PAC
    // provides. The MPU and I2C hardware are configured and barrier-fenced
    // in privileged main() before partition entry.
    let i2c0 = unsafe { &*tudelft_lm3s6965_pac::I2C0::ptr() };

    // 1. Enable master mode via MCR (bit 4 = MFE).
    i2c0.mcr.write(|w| w.i2c_mcr_mfe().set_bit());

    // 1b. Configure I2C clock frequency via MTPR.
    //     TPR = (SysClk / (2 * 10 * SCL_freq)) - 1.
    //     For 12.5 MHz system clock and 100 kHz standard mode: TPR = 5.
    //     QEMU does not enforce clock timing, but setting MTPR exercises the
    //     register and makes this example architecturally complete.
    // SAFETY: Writing a valid 7-bit TPR value (5) to the MTPR field.
    i2c0.mtpr.write(|w| unsafe { w.i2c_mtpr_tpr().bits(5) });

    // 2. Set slave address 0x50, direction = write (RS = 0).
    i2c0.msa.write(|w| {
        // SAFETY: Writing a valid 7-bit slave address (0x50) to the SA field.
        unsafe { w.i2c_msa_sa().bits(0x50) }
            .i2c_msa_rs()
            .clear_bit()
    });

    // 3. Write data byte to transmit.
    // SAFETY: Writing a valid 8-bit data value to the MDR data field.
    i2c0.mdr.write(|w| unsafe { w.i2c_mdr_data().bits(0xAB) });

    // 4. Initiate transaction: write START + RUN to MCS command register.
    //    MCS command bits on write: bit 0 = RUN, bit 1 = START.
    //    (PAC field names reflect read-side status names; raw bits avoids confusion.)
    // SAFETY: Writing valid command bits (START | RUN = 0x03) to MCS.
    i2c0.mcs.write(|w| unsafe { w.bits(0x03) });

    // 5. Poll for BUSY to clear (bounded).
    //    QEMU completes I2C transactions synchronously so this is near-instant.
    for _ in 0..100u32 {
        if !i2c0.mcs.read().i2c_mcs_busy().bit() {
            break;
        }
        cortex_m::asm::nop();
    }

    // 6. Read final MCS status and store for kernel tick handler to inspect.
    let status_raw = i2c0.mcs.read().bits();
    I2C_MCS_RAW.store(status_raw, Ordering::Release);

    // Signal completion AFTER all register accesses succeeded without fault.
    I2C_STATE.store(1, Ordering::Release);
    loop {
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p0");
    }
}

/// P1: idle partition (two-partition minimum for round-robin).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        assert!(plib::sys_yield().is_ok(), "sys_yield failed in p1");
    }
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, _k| {
    let state = I2C_STATE.load(Ordering::Acquire);
    if state == 1 {
        let mcs = I2C_MCS_RAW.load(Ordering::Acquire);
        let error = (mcs & 0x02) != 0;
        let busy = (mcs & 0x01) != 0;
        hprintln!(
            "qemu_pac_i2c0: MCS status = {:#04x} (error={}, busy={})",
            mcs,
            error,
            busy
        );
        if !busy {
            hprintln!("qemu_pac_i2c0: PASS I2C0 master register access OK");
            kernel::kexit!(success);
        } else {
            hprintln!("qemu_pac_i2c0: FAIL unexpected busy after poll");
            kernel::kexit!(failure);
        }
    }
    if tick >= 200 {
        hprintln!("qemu_pac_i2c0: FAIL timeout (state={})", state);
        kernel::kexit!(failure);
    }
});

/// Configure static MPU regions before boot.
///
/// R0: Flash (256 KiB) — code read + execute for unprivileged.
/// R1: SRAM  (64 KiB)  — full read-write for unprivileged, XN.
/// R2: Kernel guard (16 KiB at 0x2000_C000) — privileged RW only.
/// R3: I2C0  (4 KiB)   — full read-write for unprivileged, device memory.
///
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

    // R3: I2C0 peripheral — full access, device memory (TEX=0 S=1 C=0 B=1).
    let i2c_sf = mpu::encode_size(I2C0_SIZE)?;
    let i2c_rbar = mpu::build_rbar(I2C0_BASE, MPU_REGION_I2C0)?;
    let i2c_rasr = mpu::build_rasr(i2c_sf, mpu::AP_FULL_ACCESS, true, (false, true, true));
    mpu::configure_region(mpu_periph, i2c_rbar, i2c_rasr);

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
    hprintln!("qemu_pac_i2c0: start");

    // --- Privileged I2C0 setup via PAC typed API ---
    // SAFETY: We are in privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };

    // Enable I2C0 clock in SYSCTL RCGC1.
    pac.SYSCTL
        .rcgc1
        .modify(|_, w| w.sysctl_rcgc1_i2c0().set_bit());

    // Brief delay for clock to stabilize (a few NOPs suffice on QEMU).
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard, I2C0 RW.
    if configure_static_mpu(&p.MPU).is_none() {
        hprintln!("qemu_pac_i2c0: FATAL MPU config failed");
        fatal_halt();
    }

    // --- Kernel setup ---
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    if sched.add(ScheduleEntry::new(0, 4)).is_err() || sched.add(ScheduleEntry::new(1, 2)).is_err()
    {
        hprintln!("qemu_pac_i2c0: FATAL sched config failed");
        fatal_halt();
    }

    // P0 gets I2C0 peripheral region via kernel MPU metadata.
    let i2c0_region = [MpuRegion::new(I2C0_BASE, I2C0_SIZE, 0)];

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
                match mem.with_peripheral_regions(&i2c0_region) {
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
