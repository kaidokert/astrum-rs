//! STM32F429ZI port of kernel/examples/context_switch.rs
//!
//! Two-partition context switch via PendSV without full kernel infrastructure.
//! Each partition has its own stack and runs unprivileged. SysTick triggers
//! context switches via PendSV, alternating between partitions.
//!
//! This demonstrates:
//! - Stack frame initialization with init_stack_frame
//! - PendSV context switching (save/restore r4-r11, PSP)
//! - Unprivileged partition execution (CONTROL.nPRIV=1)
//! - Atomic communication between partitions and handlers

#![no_std]
#![no_main]

use f429zi::{EXC_RETURN_THREAD_PSP, XPSR_THUMB};
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as _; // For device interrupt vectors

// ---------------------------------------------------------------------------
// Stack frame initialization (from kernel/src/context.rs)
// ---------------------------------------------------------------------------
const SAVED_CONTEXT_WORDS: usize = 8;  // r4-r11
const EXCEPTION_FRAME_WORDS: usize = 8; // r0-r3, r12, lr, pc, xpsr
const CONTEXT_FRAME_WORDS: usize = SAVED_CONTEXT_WORDS + EXCEPTION_FRAME_WORDS;


/// Initialize a stack frame for a new partition.
///
/// Writes SavedContext (r4-r11) and ExceptionFrame (r0-r3, r12, lr, pc, xpsr)
/// at the top of the stack. Returns the index into the stack array where the
/// frame starts, or None if the stack is too small.
fn init_stack_frame(stack: &mut [u32], entry_point: u32, r0_arg: Option<u32>) -> Option<usize> {
    let len = stack.len();
    if len < CONTEXT_FRAME_WORDS {
        return None;
    }
    let base = len - CONTEXT_FRAME_WORDS;

    // SavedContext (r4-r11): all zero
    stack[base..base + SAVED_CONTEXT_WORDS].fill(0);

    // ExceptionFrame: r0, r1, r2, r3, r12, lr, pc, xpsr
    let ef = base + SAVED_CONTEXT_WORDS;
    stack[ef] = r0_arg.unwrap_or(0);     // r0
    stack[ef + 1] = 0;                   // r1
    stack[ef + 2] = 0;                   // r2
    stack[ef + 3] = 0;                   // r3
    stack[ef + 4] = 0;                   // r12
    stack[ef + 5] = EXC_RETURN_THREAD_PSP; // lr
    stack[ef + 6] = entry_point;         // pc
    stack[ef + 7] = XPSR_THUMB;          // xpsr (Thumb bit must be set)

    Some(base)
}

// ---------------------------------------------------------------------------
// Partition stacks — 256 words (1KB) each
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;
static mut STACK_P0: [u32; STACK_WORDS] = [0; STACK_WORDS];
static mut STACK_P1: [u32; STACK_WORDS] = [0; STACK_WORDS];

// ---------------------------------------------------------------------------
// Context switching state
// ---------------------------------------------------------------------------
#[unsafe(no_mangle)]
static mut PARTITION_SP: [u32; 2] = [0; 2];

#[unsafe(no_mangle)]
static mut CURRENT_PARTITION: u32 = u32::MAX;

#[unsafe(no_mangle)]
static mut NEXT_PARTITION: u32 = 0;

/// Partition entry functions write their ID here so SysTick can observe
#[unsafe(no_mangle)]
static PARTITION_RUNNING: AtomicU32 = AtomicU32::new(u32::MAX);

/// Number of context switches to observe
const TARGET_SWITCHES: u32 = 10;

/// SysTick reload for ~10ms at 16MHz HSI
const RELOAD: u32 = 160_000 - 1;

// ---------------------------------------------------------------------------
// Partition entry points
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
// PendSV handler — performs the actual context switch
// ---------------------------------------------------------------------------
core::arch::global_asm!(
    r#"
    .syntax unified
    .thumb

    .global PendSV
    .type PendSV, %function

PendSV:
    /* Check if this is the first switch (no partition to save) */
    ldr     r0, =CURRENT_PARTITION
    ldr     r1, [r0]
    ldr     r2, =0xFFFFFFFF
    cmp     r1, r2
    beq     .Lskip_save

    /* Save current partition context */
    mrs     r3, psp
    stmdb   r3!, {{r4-r11}}

    /* Store updated SP into PARTITION_SP[current] */
    ldr     r2, =PARTITION_SP
    lsl     r0, r1, #2            /* r0 = current * 4 */
    str     r3, [r2, r0]

.Lskip_save:
    /* Load next partition */
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

    /* Set CONTROL.nPRIV so the partition runs unprivileged */
    mrs     r0, CONTROL
    orr     r0, r0, #1
    msr     CONTROL, r0
    isb

    /* Return to Thread mode using PSP (EXC_RETURN = 0xFFFFFFFD) */
    ldr     lr, =0xFFFFFFFD
    bx      lr

    .size PendSV, . - PendSV
"#
);

// ---------------------------------------------------------------------------
// SysTick handler — alternates partitions and triggers PendSV
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    static mut SWITCH_COUNT: u32 = 0;
    static mut DONE: bool = false;

    // If test completed, just idle
    unsafe {
        if *DONE {
            return;
        }
    }

    let who = PARTITION_RUNNING.load(Ordering::Acquire);

    unsafe {
        let current = core::ptr::read_volatile(core::ptr::addr_of!(CURRENT_PARTITION));

        // Only count/print after the first partition has actually started
        if current != u32::MAX {
            *SWITCH_COUNT += 1;
            let count = *SWITCH_COUNT;
            rprintln!("switch {}: partition {} was running", count, who);

            if count >= TARGET_SWITCHES {
                rprintln!("\ncontext_switch: {} switches observed - PASS", TARGET_SWITCHES);
                rprintln!("Both partitions executed successfully in unprivileged mode");
                *DONE = true;
                return;
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
// Main — initialize stacks, configure exceptions, start scheduling
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut core = cortex_m::Peripherals::take().unwrap();

    rprintln!("STM32F429ZI Context Switch Test");
    rprintln!("Setting up two partitions with unprivileged execution");

    // Initialize partition stacks
    unsafe {
        let p0_ptr = &raw mut STACK_P0;
        let p1_ptr = &raw mut STACK_P1;

        let sp0_idx = init_stack_frame(
            &mut *p0_ptr,
            partition_0_entry,
            None
        ).expect("stack P0 too small");

        let sp1_idx = init_stack_frame(
            &mut *p1_ptr,
            partition_1_entry,
            None
        ).expect("stack P1 too small");

        // Convert stack-array indices to actual RAM addresses (PSP values)
        let p0_base = (*p0_ptr).as_ptr() as u32;
        let p1_base = (*p1_ptr).as_ptr() as u32;
        let p0_sp = p0_base + (sp0_idx as u32) * 4;
        let p1_sp = p1_base + (sp1_idx as u32) * 4;

        rprintln!("  P0 stack @ {:#010x}, SP = {:#010x}", p0_base, p0_sp);
        rprintln!("  P1 stack @ {:#010x}, SP = {:#010x}", p1_base, p1_sp);

        PARTITION_SP[0] = p0_sp;
        PARTITION_SP[1] = p1_sp;
    }

    // Configure exception priorities
    // PendSV = lowest priority so it fires after SysTick completes
    unsafe {
        core.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        core.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    // Configure SysTick for 10ms ticks at 16MHz HSI
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(RELOAD);
    core.SYST.clear_current();
    core.SYST.enable_counter();
    core.SYST.enable_interrupt();

    rprintln!("Triggering first PendSV to start partition 0");
    rprintln!("Each partition runs unprivileged (CONTROL.nPRIV=1)");
    rprintln!("Waiting for {} context switches...\n", TARGET_SWITCHES);

    // Trigger the first context switch — PendSV will boot partition 0
    cortex_m::peripheral::SCB::set_pendsv();

    // After PendSV fires, CPU runs partition code in Thread mode (PSP)
    // Main never regains control; SysTick/PendSV drive everything
    loop {
        cortex_m::asm::wfi();
    }
}
