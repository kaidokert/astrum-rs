//! Adversarial test: unprivileged partition attempts to disable interrupts.
//!
//! On Cortex-M3, unprivileged code cannot modify interrupt masking registers
//! (PRIMASK via CPSID/CPSIE, or BASEPRI). When executed from nPRIV=1 Thread
//! mode, these instructions are silently ignored (no fault occurs).
//!
//! This test verifies:
//! 1. Partition attempts CPSID i (disable interrupts)
//! 2. No fault occurs (instruction is silently ignored for nPRIV)
//! 3. SysTick interrupt still fires afterward (ticks advance)
//!
//! The test waits for a few SysTick interrupts to verify the timer is working,
//! then drops to unprivileged mode and runs CPSID i in a tight loop. If the
//! instruction had any effect, SysTick would stop firing. The SysTick handler
//! counts ticks and reports PASS once enough ticks have elapsed.
//!
//! Run with: cargo run --target thumbv7m-none-eabi --features qemu --example disable_interrupts

#![no_std]
#![no_main]

use core::ptr;

use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::syst::SystClkSource;
// Note: We use inline asm for privilege drop instead of cortex_m::register
// to have a single atomic privilege-drop-and-loop sequence.
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::mpu;

#[path = "mod.rs"]
mod adversarial;

use adversarial::FaultInfo;

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

/// Test name for reporting.
const TEST_NAME: &str = "disable_interrupts";

/// Stack size in words (256 words = 1 KiB).
const STACK_WORDS: usize = 256;

/// Stack size in bytes.
const DATA_SIZE: u32 = (STACK_WORDS * 4) as u32;

/// Code region size for MPU.
const CODE_SIZE: u32 = 32 * 1024;

/// Number of ticks to wait before dropping to unprivileged mode.
const WARMUP_TICKS: u32 = 3;

/// Tick at which the partition drops to unprivileged and starts CPSID loop.
/// Set by main() before dropping privileges, read by SysTick handler.
static mut DROP_TICK: u32 = 0;

/// Total number of ticks after DROP_TICK to wait before declaring success.
/// If ticks keep advancing after CPSID i, the instruction had no effect.
const VERIFY_TICKS: u32 = 5;

// ---------------------------------------------------------------------------
// Partition stack
// ---------------------------------------------------------------------------

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut PARTITION_STACK: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Tick counter (written by SysTick handler in privileged mode)
// ---------------------------------------------------------------------------

/// Global tick counter. Only accessed from privileged mode (main before drop,
/// SysTick handler after).
static mut TICK_COUNT: u32 = 0;

// ---------------------------------------------------------------------------
// Fault capture (for unexpected faults)
// ---------------------------------------------------------------------------

static mut FAULT: FaultInfo = FaultInfo::new();

define_memmanage_handler!(FAULT, {
    // SAFETY: Reading FAULT from the exception handler that just wrote it.
    let info = unsafe { ptr::read_volatile(&raw const FAULT) };

    // Report unexpected fault and fail.
    hprintln!(
        "{}: FAIL - unexpected MemManage fault at {:#010x} (mmfsr={:#04x})",
        TEST_NAME,
        info.mmfar,
        info.mmfsr
    );
    debug::exit(debug::EXIT_FAILURE);

    loop {
        asm::wfi();
    }
});

// ---------------------------------------------------------------------------
// SysTick handler
// ---------------------------------------------------------------------------

#[exception]
fn SysTick() {
    // SAFETY: Single-core, this handler is privileged and preempts all
    // non-exception code. TICK_COUNT and DROP_TICK are only written from
    // main() before the drop or from this handler. We use a single unsafe
    // block for the tightly-coupled read-modify-write and volatile read.
    // All accesses use volatile to ensure consistent semantics.
    // TODO: Consider using critical_section::Mutex for shared state (idiomatic).
    let (tick, drop_tick) = unsafe {
        let t = ptr::read_volatile(&raw const TICK_COUNT) + 1;
        ptr::write_volatile(&raw mut TICK_COUNT, t);
        (t, ptr::read_volatile(&raw const DROP_TICK))
    };

    // If we haven't dropped to unprivileged yet, drop_tick is 0.
    if drop_tick == 0 {
        return;
    }

    // After dropping, wait for VERIFY_TICKS more interrupts.
    // If we reach this point, SysTick is still firing despite CPSID i.
    if tick >= drop_tick + VERIFY_TICKS {
        hprintln!("{}: drop_tick = {}", TEST_NAME, drop_tick);
        hprintln!("{}: final_tick = {}", TEST_NAME, tick);
        hprintln!(
            "{}: SysTick advanced by {} ticks after CPSID i",
            TEST_NAME,
            tick - drop_tick
        );
        hprintln!(
            "{}: CPSID i was silently ignored in unprivileged mode (expected)",
            TEST_NAME
        );
        hprintln!("{}: PASS", TEST_NAME);
        debug::exit(debug::EXIT_SUCCESS);

        // Never reached, but required for control flow.
        loop {
            asm::wfi();
        }
    }
}

// ---------------------------------------------------------------------------
// MPU configuration
// ---------------------------------------------------------------------------

/// Configure MPU for partition: code RX, data RW.
// TODO: Refactor to return Result instead of using .expect() (panic-free policy).
fn configure_partition_mpu(mpu_periph: &cortex_m::peripheral::MPU, data_base: u32) {
    // SAFETY: single-core, interrupts disabled — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: code RX — covers flash binary (priv+unpriv read-only)
    let code_sf = mpu::encode_size(CODE_SIZE).expect("code size");
    let code_rbar = mpu::build_rbar(0x0000_0000, 0).expect("code rbar");
    let code_rasr = mpu::build_rasr(code_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, code_rbar, code_rasr);

    // R1: data RW — partition stack (full access, XN)
    let data_sf = mpu::encode_size(DATA_SIZE).expect("data size");
    let data_rbar = mpu::build_rbar(data_base, 1).expect("data rbar");
    let data_rasr = mpu::build_rasr(data_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, data_rbar, data_rasr);

    // Enable MPU with PRIVDEFENA.
    // SAFETY: regions programmed; barriers ensure visibility.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("cortex-m peripherals");
    hprintln!("{}: start", TEST_NAME);

    // Enable fault handlers (for unexpected faults).
    p.SCB.enable(Exception::MemoryManagement);
    p.SCB.enable(Exception::UsageFault);

    // Get partition stack addresses.
    let stack_base = (&raw const PARTITION_STACK).cast::<u32>() as u32;
    let stack_top = stack_base + DATA_SIZE;

    hprintln!("  stack: {:#010x} - {:#010x}", stack_base, stack_top);

    // Configure MPU before dropping privileges.
    configure_partition_mpu(&p.MPU, stack_base);

    // Configure SysTick to fire periodically.
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();

    hprintln!("  MPU and SysTick configured, waiting for warm-up ticks...");

    // Wait for initial ticks to verify SysTick is working.
    // SAFETY: We're still privileged, so we can read TICK_COUNT.
    while unsafe { ptr::read_volatile(&raw const TICK_COUNT) } < WARMUP_TICKS {
        asm::nop();
    }

    // Record the tick at which we'll drop to unprivileged.
    // SAFETY: Single-core, SysTick may preempt us but only reads DROP_TICK.
    let current_tick = unsafe { ptr::read_volatile(&raw const TICK_COUNT) };
    unsafe { ptr::write_volatile(&raw mut DROP_TICK, current_tick) };

    hprintln!("  warm-up complete, tick_count = {}", current_tick);
    hprintln!("  dropping to unprivileged and executing CPSID i...");

    // SAFETY: We set PSP to the top of a valid, MPU-accessible stack,
    // then set CONTROL.SPSEL=1 (use PSP) and CONTROL.nPRIV=1 (unprivileged),
    // then loop executing CPSID i and BASEPRI writes. The instructions have
    // no effect in unprivileged mode, so SysTick continues firing and
    // eventually reports success. All registers are declared via symbolic
    // operands so the compiler can allocate them safely.
    #[allow(asm_sub_register)] // Thumb mode has no sub-registers; warning not applicable
    unsafe {
        core::arch::asm!(
            // Write stack_top to PSP
            "msr psp, {stack_top}",
            // Read CONTROL into ctrl
            "mrs {ctrl}, control",
            // Set SPSEL bit (bit 1) and nPRIV bit (bit 0)
            "orr {ctrl}, {ctrl}, #3",
            // Write CONTROL
            "msr control, {ctrl}",
            // ISB to ensure CONTROL changes take effect
            "isb",
            // Now unprivileged on PSP.
            // Loop forever executing CPSID i and BASEPRI writes.
            // These are silently ignored in unprivileged mode.
            "1:",
            "cpsid i",              // Attempt to disable interrupts (ignored)
            "movs {basepri_val}, #0xFF",
            "msr basepri, {basepri_val}", // Attempt to mask all interrupts (ignored)
            "nop",
            "b 1b",
            stack_top = in(reg) stack_top,
            ctrl = out(reg) _,
            basepri_val = out(reg) _,
        );
    }
    // SAFETY: The asm block above loops forever and never returns.
    unsafe { core::hint::unreachable_unchecked() }
}
