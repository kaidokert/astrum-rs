// TODO: Not migrated to define_harness! — this example intentionally
// tests the raw PendSV context-switch mechanism without KernelState,
// SVC dispatch, or the round-robin scheduler. Its custom SysTick
// handler (switch counting, semihosting exit) and separate per-
// partition stacks don't fit the shared harness pattern.

//! QEMU integration example: two-partition context switch via PendSV.
//!
//! Sets up two partitions, each with its own stack. Initialises their
//! stack frames with `init_stack_frame`, then triggers context switches
//! through PendSV. Each partition entry function stores its ID to a
//! shared atomic; the SysTick handler observes which partition ran and
//! exits after enough interleaved switches.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::context::init_stack_frame;
use panic_semihosting as _;

// ---------------------------------------------------------------------------
// Partition stacks — placed in .bss (RAM). Each partition gets 256 words
// (1 KB). These are u32 arrays so `init_stack_frame` can write to them.
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;
static mut STACK_P0: [u32; STACK_WORDS] = [0; STACK_WORDS];
static mut STACK_P1: [u32; STACK_WORDS] = [0; STACK_WORDS];

// ---------------------------------------------------------------------------
// Per-partition saved stack pointers. PendSV loads/stores PSP from these.
// ---------------------------------------------------------------------------
#[no_mangle]
static mut PARTITION_SP: [u32; 2] = [0; 2];

/// Which partition is currently running (0 or 1). `u32::MAX` means none yet.
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;

/// Which partition to switch to next. Set by SysTick before pending PendSV.
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

/// Partition entry functions write their ID here so SysTick can observe.
#[no_mangle]
static PARTITION_RUNNING: AtomicU32 = AtomicU32::new(u32::MAX);

/// Number of context switches to observe before declaring success.
const TARGET_SWITCHES: u32 = 6;

/// SysTick reload: ~10 ms at 12 MHz (QEMU lm3s6965evb).
const RELOAD: u32 = 120_000 - 1;

// ---------------------------------------------------------------------------
// Partition entry points
//
// These are `extern "C"` so their addresses can be placed in the initial
// exception frame's PC field. They never return.
// ---------------------------------------------------------------------------
extern "C" fn partition_0_entry() -> ! {
    loop {
        PARTITION_RUNNING.store(0, Ordering::Release);
        cortex_m::asm::nop();
    }
}

extern "C" fn partition_1_entry() -> ! {
    loop {
        PARTITION_RUNNING.store(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

// ---------------------------------------------------------------------------
// PendSV handler — performs the actual context switch.
//
// Uses `global_asm!` per architecture decision D6 (stable Rust, no #[naked]).
//
// Algorithm:
//   1. If CURRENT_PARTITION == 0xFFFFFFFF, skip save (first switch).
//   2. Save r4-r11 onto the current partition's stack (stmdb).
//   3. Store updated PSP into PARTITION_SP[current].
//   4. Load NEXT_PARTITION index; set CURRENT_PARTITION = NEXT_PARTITION.
//   5. Load PSP from PARTITION_SP[next]; restore r4-r11 (ldmia).
//   6. Set PSP and return with EXC_RETURN = 0xFFFFFFFD (Thread/PSP).
// ---------------------------------------------------------------------------
core::arch::global_asm!(
    r#"
    .syntax unified
    .thumb

    .global PendSV
    .type PendSV, %function

PendSV:
    /* --- Check if this is the first switch (no partition to save) --- */
    ldr     r0, =CURRENT_PARTITION
    ldr     r1, [r0]
    ldr     r2, =0xFFFFFFFF
    cmp     r1, r2
    beq     .Lskip_save

    /* --- Save current partition context --- */
    mrs     r3, psp
    stmdb   r3!, {{r4-r11}}

    /* Store updated SP into PARTITION_SP[current] */
    ldr     r2, =PARTITION_SP
    lsl     r0, r1, #2            /* r0 = current * 4 */
    str     r3, [r2, r0]

.Lskip_save:
    /* --- Load next partition --- */
    ldr     r0, =NEXT_PARTITION
    ldr     r1, [r0]              /* r1 = next partition index */

    /* Update CURRENT_PARTITION = next */
    ldr     r0, =CURRENT_PARTITION
    str     r1, [r0]

    /* Load SP from PARTITION_SP[next] */
    ldr     r2, =PARTITION_SP
    lsl     r0, r1, #2
    ldr     r3, [r2, r0]

    /* Restore r4-r11 from next partition's stack */
    ldmia   r3!, {{r4-r11}}

    /* Set PSP */
    msr     psp, r3

    /* Return to Thread mode using PSP (EXC_RETURN = 0xFFFFFFFD) */
    ldr     lr, =0xFFFFFFFD
    bx      lr

    .size PendSV, . - PendSV
"#
);

// ---------------------------------------------------------------------------
// SysTick handler — runs at higher priority than PendSV. Alternates
// NEXT_PARTITION, observes which partition ran, prints and counts
// switches. After TARGET_SWITCHES, exits via semihosting.
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    static mut SWITCH_COUNT: u32 = 0;

    let who = PARTITION_RUNNING.load(Ordering::Acquire);

    unsafe {
        let current = core::ptr::read_volatile(core::ptr::addr_of!(CURRENT_PARTITION));
        // Only count/print after the first partition has actually started.
        if current != u32::MAX {
            *SWITCH_COUNT += 1;
            let count = *SWITCH_COUNT;
            hprintln!("switch {}: partition {} was running", count, who);

            if count >= TARGET_SWITCHES {
                hprintln!(
                    "context_switch: {} switches observed -- PASS",
                    TARGET_SWITCHES
                );
                debug::exit(debug::EXIT_SUCCESS);
            }
        }

        // Alternate to the other partition
        let next = if current == 0 { 1 } else { 0 };
        core::ptr::write_volatile(core::ptr::addr_of_mut!(NEXT_PARTITION), next);
    }

    // Pend PendSV (fires after SysTick returns, since PendSV is lower priority)
    cortex_m::peripheral::SCB::set_pendsv();
}

// ---------------------------------------------------------------------------
// Main — initialise stacks, configure exceptions, kick off scheduling,
// then enter the first partition via PendSV.
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    let mut core = cortex_m::Peripherals::take().unwrap();

    hprintln!("context_switch: setting up two partitions");

    // --- Initialise partition stacks ---
    // Safety: we are in privileged mode before any partition runs; exclusive
    // access to these statics is guaranteed at this point.
    unsafe {
        let p0_ptr = &raw mut STACK_P0;
        let p1_ptr = &raw mut STACK_P1;
        let sp0_idx = init_stack_frame(&mut *p0_ptr, partition_0_entry as *const () as u32, None)
            .expect("stack P0 too small");
        let sp1_idx = init_stack_frame(&mut *p1_ptr, partition_1_entry as *const () as u32, None)
            .expect("stack P1 too small");

        // Convert stack-array indices to actual RAM addresses (PSP values).
        // PSP = base address of stack array + sp_idx * 4
        let p0_base = (*p0_ptr).as_ptr() as u32;
        let p1_base = (*p1_ptr).as_ptr() as u32;
        let p0_sp = p0_base + (sp0_idx as u32) * 4;
        let p1_sp = p1_base + (sp1_idx as u32) * 4;

        hprintln!("  P0 stack @ {:#010x}, SP = {:#010x}", p0_base, p0_sp);
        hprintln!("  P1 stack @ {:#010x}, SP = {:#010x}", p1_base, p1_sp);

        PARTITION_SP[0] = p0_sp;
        PARTITION_SP[1] = p1_sp;
    }

    // --- Configure exception priorities ---
    // PendSV = lowest priority so it fires after SysTick completes.
    unsafe {
        core.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        core.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    // --- Configure SysTick ---
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(RELOAD);
    core.SYST.clear_current();
    core.SYST.enable_counter();
    core.SYST.enable_interrupt();

    hprintln!("context_switch: triggering first PendSV");

    // Trigger the first context switch — PendSV will boot partition 0.
    // After this, the CPU runs partition code in Thread mode (PSP).
    // Main never regains control; SysTick/PendSV drive everything.
    cortex_m::peripheral::SCB::set_pendsv();

    // PendSV is lowest priority and fires immediately since we are in
    // Thread mode. This loop is unreachable once PendSV executes, but
    // structurally required by the `-> !` return type.
    loop {
        cortex_m::asm::wfi();
    }
}
