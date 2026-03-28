//! QEMU proof-of-concept: partition writes to UART0 via PAC typed register API.
//!
//! Privileged main() enables the UART0 clock via SYSCTL RCGC1 and configures
//! UART0 for TX.  A static MPU layout grants unprivileged code access to flash,
//! SRAM, and the UART0 peripheral region (0x4000_C000, 4 KiB).  Partition P0
//! transmits bytes using the PAC's typed register accessors.  A SysTick hook
//! checks an AtomicU32 state flag set by P0 after successful TX and reports
//! PASS via semihosting.
//!
//! Run:
//!   cargo run --example qemu_pac_uart0 --features qemu,log-semihosting,qemu-peripherals \
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
const UART0_BASE: u32 = 0x4000_C000;
const UART0_SIZE: u32 = 4096;

/// State flag: 0 = not started, 1 = TX complete.
static TX_STATE: AtomicU32 = AtomicU32::new(0);

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {}
);

/// P0: write bytes to UART0 DR via PAC typed API, then set state flag.
const _: PartitionEntry = p0_entry;
extern "C" fn p0_entry() -> ! {
    // SAFETY: We have MPU-granted access to UART0 region; no other code
    // accesses UART0 concurrently.
    // TODO: reviewer false positive — PAC register accessors (read/write) use volatile
    // internally, so no additional volatile or memory barriers are needed beyond what
    // the PAC provides. The MPU and UART hardware are configured and barrier-fenced
    // in privileged main() before partition entry.
    let uart0 = unsafe { &*tudelft_lm3s6965_pac::UART0::ptr() };

    let msg: &[u8] = b"UART0:TX_OK\n";
    for &byte in msg {
        // Busy-wait while TX FIFO is full before writing next byte.
        while uart0.fr.read().uart_fr_txff().bit() {
            cortex_m::asm::nop();
        }
        // SAFETY: Writing valid 8-bit data to the DR data field.
        uart0.dr.write(|w| unsafe { w.uart_dr_data().bits(byte) });
    }

    // Signal completion AFTER all bytes successfully written.
    TX_STATE.store(1, Ordering::Release);
    loop {
        let _ = plib::sys_yield();
    }
}

/// P1: idle partition (two-partition minimum for round-robin).
const _: PartitionEntry = p1_entry;
extern "C" fn p1_entry() -> ! {
    loop {
        let _ = plib::sys_yield();
    }
}

kernel::define_unified_harness!(no_boot, TestConfig, |tick, _k| {
    let state = TX_STATE.load(Ordering::Acquire);
    if state == 1 {
        hprintln!("qemu_pac_uart0: PASS partition TX complete");
        kernel::kexit!(success);
    }
    if tick >= 200 {
        hprintln!("qemu_pac_uart0: FAIL timeout (state={})", state);
        kernel::kexit!(failure);
    }
});

/// Configure static MPU regions before boot.
///
/// R0: Flash (256 KiB) — code read + execute for unprivileged.
/// R1: SRAM  (64 KiB)  — full read-write for unprivileged, XN.
/// R2: Kernel guard (16 KiB at 0x2000_C000) — privileged RW only.
/// R3: UART0 (4 KiB)   — full read-write for unprivileged, device memory.
///
// TODO: Hardcoded region indices 0-3 overlap with kernel's partition_mpu_regions() layout.
// These static regions are only active during privileged main() before boot; boot_preconfigured()
// overwrites R0-R3 via precompute_mpu_cache(). If the kernel's region strategy changes, this
// static setup should be revisited. Consider using higher indices (R4-R7) if available.
fn configure_static_mpu(mpu_periph: &cortex_m::peripheral::MPU) {
    // SAFETY: single-core, before scheduler starts — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // TODO: .expect() calls below are used for init-time MPU configuration with constant
    // arguments that cannot fail at runtime. Replace with a fallback/abort pattern if this
    // example is promoted to production code.

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

    // R2: Kernel guard — privileged RW only, overrides R1.
    let guard_sf = mpu::encode_size(16 * 1024).expect("guard size");
    let guard_rbar = mpu::build_rbar(0x2000_C000, 2).expect("guard rbar");
    let guard_rasr = mpu::build_rasr(guard_sf, mpu::AP_PRIV_RW, true, (true, true, false));
    mpu::configure_region(mpu_periph, guard_rbar, guard_rasr);

    // R3: UART0 peripheral — full access, device memory (TEX=0 S=1 C=0 B=1).
    let uart_sf = mpu::encode_size(UART0_SIZE).expect("uart size");
    let uart_rbar = mpu::build_rbar(UART0_BASE, 3).expect("uart rbar");
    let uart_rasr = mpu::build_rasr(uart_sf, mpu::AP_FULL_ACCESS, true, (false, true, true));
    mpu::configure_region(mpu_periph, uart_rbar, uart_rasr);

    // SAFETY: enabling the MPU after region configuration is complete.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("peripherals");
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("qemu_pac_uart0: start");

    // --- Privileged UART0 setup via PAC typed API ---
    // SAFETY: We are in privileged main() before boot; exclusive access.
    let pac = unsafe { tudelft_lm3s6965_pac::Peripherals::steal() };

    // Enable UART0 clock in SYSCTL RCGC1.
    pac.SYSCTL
        .rcgc1
        .modify(|_, w| w.sysctl_rcgc1_uart0().set_bit());

    // Brief delay for clock to stabilize (a few NOPs suffice on QEMU).
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    // Configure UART0: 8-bit word length, FIFOs enabled.
    pac.UART0.lcrh.write(|w| {
        w.uart_lcrh_wlen()
            .uart_lcrh_wlen_8()
            .uart_lcrh_fen()
            .set_bit()
    });
    // Enable UART and TX.
    pac.UART0
        .ctl
        .write(|w| w.uart_ctl_uarten().set_bit().uart_ctl_txe().set_bit());

    // Enable MemManage exception before configuring MPU.
    p.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Set up static MPU: flash RX, RAM RW, kernel guard, UART0 RW.
    configure_static_mpu(&p.MPU);

    // TODO: .expect() calls in kernel setup below are fatal init errors (cannot proceed
    // without a valid scheduler/kernel). Replace with a structured error path if this
    // example is promoted to production code.

    // --- Kernel setup ---
    let entry_fns: [PartitionEntry; NP] = [p0_entry, p1_entry];
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");

    // P0 gets UART0 peripheral region via kernel MPU metadata.
    let uart0_region = [MpuRegion::new(UART0_BASE, UART0_SIZE, 0)];

    let k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let base = stk.as_u32_slice().as_ptr() as u32;
            let mem = ExternalPartitionMemory::from_aligned_stack(
                stk,
                entry_fns[i],
                MpuRegion::new(base, REGION_SZ, 0),
                i as u8,
            )
            .expect("mem");
            if i == 0 {
                mem.with_peripheral_regions(&uart0_region).expect("periph")
            } else {
                mem
            }
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    // store_kernel is generated by define_unified_harness! (via define_unified_kernel!).
    store_kernel(k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<TestConfig>(p) }.expect("boot") {}
}
