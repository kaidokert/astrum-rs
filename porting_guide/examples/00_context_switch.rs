//! Bare-metal two-partition context switch — no kernel dependency.
//!
//! Demonstrates the minimal PendSV-based context switch mechanism that
//! the RTOS kernel builds upon. Two "partitions" (simple loops that
//! increment atomic counters) are preempted by SysTick, which triggers
//! PendSV to save/restore r4-r11 and swap the Process Stack Pointer.
//!
//! # Success criteria
//!
//! The test passes when **10 or more context switches** are observed
//! with partitions alternating (0 → 1 → 0 → …). Each SysTick tick
//! checks which partition last ran; if it differs from the previous
//! tick, a switch is counted. After 10 switches the test prints PASS
//! and exits via semihosting.
//!
//! # Run
//!
//! ```text
//! cargo run --target thumbv7m-none-eabi \
//!     --features board-qemu,log-semihosting \
//!     --example 00_context_switch
//! ```

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::debug;
#[allow(unused_imports)]
use kernel::kpanic as _;
use porting_guide::klog;

// Partition stacks (1 KiB each, 8-byte aligned per AAPCS).
const STACK_SIZE_WORDS: usize = 256;

#[repr(align(8))]
#[allow(dead_code)]
struct AlignedStack([u32; STACK_SIZE_WORDS]);

// SAFETY: Each stack is only accessed during init (before scheduling starts)
// and then exclusively by its owning partition via PSP, so no data races occur.
static mut STACK_0: AlignedStack = AlignedStack([0; STACK_SIZE_WORDS]);
static mut STACK_1: AlignedStack = AlignedStack([0; STACK_SIZE_WORDS]);

// Context-switch state (accessed from PendSV naked asm via symbol name).
// SAFETY: These are only mutated inside PendSV, which runs at the lowest
// exception priority and cannot preempt itself, so accesses are inherently
// sequential. Initialization in main() completes before PendSV is first pended.
#[no_mangle]
static mut PARTITION_SP: [u32; 2] = [0; 2];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = 0;

// Observation atomics (written by partitions, read by SysTick).
static RUNNING_ID: AtomicU32 = AtomicU32::new(u32::MAX);
static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);
static PREV_ID: AtomicU32 = AtomicU32::new(u32::MAX);
static TICK: AtomicU32 = AtomicU32::new(0);

const TARGET_SWITCHES: u32 = 10;
const TIMEOUT_TICKS: u32 = 200;

extern "C" fn partition_0() -> ! {
    loop {
        RUNNING_ID.store(0, Ordering::Release);
        cortex_m::asm::nop();
    }
}

extern "C" fn partition_1() -> ! {
    loop {
        RUNNING_ID.store(1, Ordering::Release);
        cortex_m::asm::nop();
    }
}

// Stack init: build a fake 16-word frame (8 sw r4-r11 + 8 hw exception)
// so PendSV exception return lands at the entry point.
const XPSR_THUMB: u32 = 0x0100_0000;

/// # Safety
/// - `stack` must point to a valid, exclusively owned `[u32; STACK_SIZE_WORDS]`
///   that is 8-byte aligned and remains valid for the partition's entire lifetime.
/// - `STACK_SIZE_WORDS` must be >= 16 to hold the full synthetic frame.
unsafe fn init_stack(stack: *mut u32, entry: extern "C" fn() -> !) -> u32 {
    // SAFETY: caller guarantees `stack` is valid for STACK_SIZE_WORDS words.
    let top = stack.add(STACK_SIZE_WORDS);

    // Build a 16-word synthetic frame: 8 software-saved (r4-r11) zeroed,
    // plus 8 hardware exception frame slots with targeted writes.
    // Software-saved r4-r11 (slots 16..9 from top): zero.
    for i in 9..=16 {
        top.sub(i).write_volatile(0);
    }
    // Hardware exception frame: R0-R3, R12 default to zero.
    for i in 4..=8 {
        top.sub(i).write_volatile(0);
    }
    // Set the three meaningful hardware frame fields.
    top.sub(1).write_volatile(XPSR_THUMB); // xPSR
    top.sub(2).write_volatile(entry as usize as u32); // PC
    top.sub(3).write_volatile(0xFFFF_FFFF); // LR (unused return address)

    // Return PSP pointing at bottom of frame (r4 position).
    top.sub(16) as u32
}

// PendSV: save r4-r11 + PSP for current partition, toggle, restore next.
// SAFETY: This naked function is the PendSV handler. It only accesses
// PARTITION_SP and CURRENT_PARTITION, which are never concurrently mutated
// (PendSV is the lowest priority exception and cannot preempt itself).
// The inline assembly correctly saves/restores the callee-saved registers
// and PSP, maintaining ABI invariants across context switches.
#[unsafe(naked)]
#[export_name = "PendSV"]
extern "C" fn pend_sv_handler() {
    core::arch::naked_asm!(
        // --- Save current context ---
        "mrs     r0, psp",
        "stmdb   r0!, {{r4-r11}}",
        // PARTITION_SP[CURRENT_PARTITION] = updated PSP
        "ldr     r1, =CURRENT_PARTITION",
        "ldr     r2, [r1]",
        "ldr     r3, =PARTITION_SP",
        "str     r0, [r3, r2, lsl #2]",
        // --- Toggle partition index ---
        "eor     r2, r2, #1",
        "str     r2, [r1]",
        // --- Restore next context ---
        "ldr     r0, [r3, r2, lsl #2]",
        "ldmia   r0!, {{r4-r11}}",
        "msr     psp, r0",
        // Return to Thread mode, PSP.
        "ldr     lr, =0xFFFFFFFD",
        "bx      lr",
    );
}

// SysTick: observe which partition ran, count alternations, trigger PendSV.
#[exception]
fn SysTick() {
    let tick = TICK.fetch_add(1, Ordering::Relaxed) + 1;
    let who = RUNNING_ID.load(Ordering::Acquire);
    let prev = PREV_ID.swap(who, Ordering::Relaxed);

    // Count a switch when the running partition actually alternated.
    if who != prev && prev != u32::MAX && who != u32::MAX {
        let count = SWITCH_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
        klog!("switch {}: partition {} -> {}", count, prev, who);

        if count >= TARGET_SWITCHES {
            klog!("00_context_switch: PASS ({} switches)", count);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }

    if tick >= TIMEOUT_TICKS {
        klog!(
            "00_context_switch: FAIL timeout ({} switches)",
            SWITCH_COUNT.load(Ordering::Relaxed)
        );
        debug::exit(debug::EXIT_FAILURE);
    }

    // Pend PendSV — fires after SysTick returns (lowest priority).
    cortex_m::peripheral::SCB::set_pendsv();
}

// Main: init stacks, configure SysTick + PendSV priority, enter P0 directly.
// P0's context is saved by the first PendSV; P1 enters via its fake frame.
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("Peripherals::take");

    klog!("00_context_switch: bare-metal two-partition demo");

    // Initialise partition 1's stack with a fake exception frame.
    // Partition 0 doesn't need one — we enter it directly.
    // SAFETY: Called before scheduling starts; STACK_1, PARTITION_SP, and
    // CURRENT_PARTITION are not yet accessed by any exception handler.
    unsafe {
        let s1 = core::ptr::addr_of_mut!(STACK_1) as *mut u32;
        PARTITION_SP[1] = init_stack(s1, partition_1);
        CURRENT_PARTITION = 0;
    }

    // Set PendSV to lowest priority (0xFF) so it never preempts SysTick.
    // SAFETY: We own `p` (the peripherals singleton) and no exception
    // handler accesses SCB priority registers.
    unsafe {
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
    }

    // Configure SysTick: fire every 10 000 cycles for rapid preemption.
    let mut syst = p.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(10_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    // Point PSP at the top of stack_0 (empty — room for hw to push frames).
    // SAFETY: STACK_0 is valid, exclusively owned, and not yet in use.
    let stack_0_top =
        unsafe { (core::ptr::addr_of_mut!(STACK_0) as *mut u32).add(STACK_SIZE_WORDS) as u32 };
    // SAFETY: Called before switching to PSP; the stack address is valid.
    unsafe {
        cortex_m::register::psp::write(stack_0_top);
    }

    // Switch Thread mode to use PSP (CONTROL.SPSEL = 1).
    // SAFETY: PSP has been set to a valid stack above. After this point,
    // Thread mode uses PSP and the original MSP is reserved for handlers.
    unsafe {
        core::arch::asm!(
            "mrs {tmp}, CONTROL",
            "orr {tmp}, {tmp}, #0x2",
            "msr CONTROL, {tmp}",
            "isb",
            tmp = out(reg) _,
        );
    }

    // Now running on PSP / stack_0. Enter partition 0 directly.
    // The first SysTick will trigger PendSV, which saves P0's context
    // and restores P1 from its pre-built fake frame.
    klog!("00_context_switch: entering partition 0");
    partition_0()
}
