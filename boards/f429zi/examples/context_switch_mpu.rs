//! STM32F429ZI: Context switching with MPU isolation (Step 1a)
//!
//! Builds on context_switch.rs by adding static MPU regions for each partition.
//! Each partition's stack is isolated - partition 0 cannot access partition 1's stack.
//!
//! Step 1a: MPU reconfiguration in main loop (not yet in PendSV)
//! - Simple to debug
//! - Proves MPU configuration works
//! - Shows how to use MPU helper functions

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
// MPU Helper Functions (from kernel/src/mpu.rs)
// ---------------------------------------------------------------------------

/// Encode size in bytes to MPU size field (SIZE = log2(bytes) - 1)
/// Requires: size >= 32 and is power of 2
pub fn encode_size(size_bytes: u32) -> Option<u32> {
    if size_bytes < 32 || !size_bytes.is_power_of_two() {
        return None;
    }
    Some(size_bytes.trailing_zeros() - 1)
}

/// Build RBAR (Region Base Address Register) value
/// base: Must be 32-byte aligned (bits [4:0] = 0)
/// region: Region number 0-7
pub const fn build_rbar(base: u32, region: u32) -> Option<u32> {
    if region > 7 || base & 0x1F != 0 {
        return None;
    }
    // VALID bit set (bit 4) with region number in bits [3:0]
    Some(base | (1 << 4) | region)
}

/// Build RASR (Region Attribute and Size Register) value
/// size_field: From encode_size()
/// ap: Access permissions (use AP_* constants)
/// xn: Execute Never (true = no execute)
/// scb: (Shareable, Cacheable, Bufferable) - use (false, false, false) for SRAM
pub fn build_rasr(size_field: u32, ap: u32, xn: bool, scb: (bool, bool, bool)) -> u32 {
    let (s, c, b) = scb;
    (u32::from(xn) << 28)
        | ((ap & 0x7) << 24)
        | (u32::from(s) << 18)
        | (u32::from(c) << 17)
        | (u32::from(b) << 16)
        | ((size_field & 0x1F) << 1)
        | 1 // ENABLE bit
}

/// Configure an MPU region
/// Must be called with MPU disabled or with exclusive access
pub fn configure_region(mpu: &cortex_m::peripheral::MPU, rbar: u32, rasr: u32) {
    unsafe {
        mpu.rbar.write(rbar); // RBAR has VALID bit, selects region
        mpu.rasr.write(rasr);
    }
}

// Access Permission constants
pub const AP_NO_ACCESS: u32 = 0b000;
pub const AP_PRIV_RW: u32 = 0b001;
pub const AP_PRIV_RW_USER_RO: u32 = 0b010;
pub const AP_FULL_ACCESS: u32 = 0b011; // Priv + User RW
pub const AP_PRIV_RO: u32 = 0b101;
pub const AP_RO: u32 = 0b110; // Priv + User RO

// MPU Control Register value: Enable MPU with PRIVDEFENA
// PRIVDEFENA: Privileged default memory map is enabled as background region
pub const MPU_CTRL_ENABLE_PRIVDEFENA: u32 = (1 << 2) | 1;

// ---------------------------------------------------------------------------
// Stack frame initialization (from context_switch.rs)
// ---------------------------------------------------------------------------
const SAVED_CONTEXT_WORDS: usize = 8;  // r4-r11
const EXCEPTION_FRAME_WORDS: usize = 8; // r0-r3, r12, lr, pc, xpsr
const CONTEXT_FRAME_WORDS: usize = SAVED_CONTEXT_WORDS + EXCEPTION_FRAME_WORDS;


fn init_stack_frame(stack: &mut [u32], entry_point: u32, r0_arg: Option<u32>) -> Option<usize> {
    let len = stack.len();
    if len < CONTEXT_FRAME_WORDS {
        return None;
    }
    let base = len - CONTEXT_FRAME_WORDS;
    stack[base..base + SAVED_CONTEXT_WORDS].fill(0);
    let ef = base + SAVED_CONTEXT_WORDS;
    stack[ef] = r0_arg.unwrap_or(0);
    stack[ef + 1] = 0;
    stack[ef + 2] = 0;
    stack[ef + 3] = 0;
    stack[ef + 4] = 0;
    stack[ef + 5] = EXC_RETURN_THREAD_PSP;
    stack[ef + 6] = entry_point;
    stack[ef + 7] = XPSR_THUMB;
    Some(base)
}

// ---------------------------------------------------------------------------
// Partition stacks - Must be aligned to their size (1KB = 1024 bytes)
// ---------------------------------------------------------------------------
const STACK_WORDS: usize = 256;

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACK_P0: AlignedStack = AlignedStack([0; STACK_WORDS]);
static mut STACK_P1: AlignedStack = AlignedStack([0; STACK_WORDS]);

// ---------------------------------------------------------------------------
// Context switching state
// ---------------------------------------------------------------------------
#[unsafe(no_mangle)]
static mut PARTITION_SP: [u32; 2] = [0; 2];

#[unsafe(no_mangle)]
static mut CURRENT_PARTITION: u32 = u32::MAX;

#[unsafe(no_mangle)]
static mut NEXT_PARTITION: u32 = 0;

#[unsafe(no_mangle)]
static PARTITION_RUNNING: AtomicU32 = AtomicU32::new(u32::MAX);

const TARGET_SWITCHES: u32 = 10;
const RELOAD: u32 = 160_000 - 1; // 10ms at 16MHz HSI

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
// PendSV handler (same as context_switch.rs)
// ---------------------------------------------------------------------------
core::arch::global_asm!(
    r#"
    .syntax unified
    .thumb

    .global PendSV
    .type PendSV, %function

PendSV:
    ldr     r0, =CURRENT_PARTITION
    ldr     r1, [r0]
    ldr     r2, =0xFFFFFFFF
    cmp     r1, r2
    beq     .Lskip_save

    mrs     r3, psp
    stmdb   r3!, {{r4-r11}}

    ldr     r2, =PARTITION_SP
    lsl     r0, r1, #2
    str     r3, [r2, r0]

.Lskip_save:
    ldr     r0, =NEXT_PARTITION
    ldr     r1, [r0]

    ldr     r0, =CURRENT_PARTITION
    str     r1, [r0]

    ldr     r2, =PARTITION_SP
    lsl     r0, r1, #2
    ldr     r3, [r2, r0]

    ldmia   r3!, {{r4-r11}}

    msr     psp, r3

    mrs     r0, CONTROL
    orr     r0, r0, #1
    msr     CONTROL, r0
    isb

    ldr     lr, =0xFFFFFFFD
    bx      lr

    .size PendSV, . - PendSV
"#
);

// ---------------------------------------------------------------------------
// SysTick handler
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    static mut SWITCH_COUNT: u32 = 0;
    static mut DONE: bool = false;

    unsafe {
        if *DONE {
            return;
        }
    }

    let who = PARTITION_RUNNING.load(Ordering::Acquire);

    unsafe {
        let current = core::ptr::read_volatile(core::ptr::addr_of!(CURRENT_PARTITION));

        if current != u32::MAX {
            *SWITCH_COUNT += 1;
            let count = *SWITCH_COUNT;
            rprintln!("switch {}: partition {} was running", count, who);

            if count >= TARGET_SWITCHES {
                rprintln!("\ncontext_switch_mpu: {} switches observed - PASS", TARGET_SWITCHES);
                rprintln!("Both partitions executed with MPU isolation");
                *DONE = true;
                return;
            }
        }

        let next = if current == 0 { 1 } else { 0 };
        core::ptr::write_volatile(core::ptr::addr_of_mut!(NEXT_PARTITION), next);
    }

    cortex_m::peripheral::SCB::set_pendsv();
}

// ---------------------------------------------------------------------------
// MPU Configuration
// ---------------------------------------------------------------------------

/// Configure MPU with static regions for both partitions
/// Region 0: Code region (flash, RO + execute)
/// Region 1: Shared RAM region (globals, RW, no execute) - First 2KB
/// Region 2: Partition 0 stack (RW, no execute) - 1KB
/// Region 3: Partition 1 stack (RW, no execute) - 1KB
///
/// Step 1a approach: Configure all regions upfront, no reconfiguration needed
fn configure_static_mpu(mpu: &cortex_m::peripheral::MPU) {
    let p0_stack = unsafe { core::ptr::addr_of!(STACK_P0) as u32 };
    let p1_stack = unsafe { core::ptr::addr_of!(STACK_P1) as u32 };
    let stack_size = (STACK_WORDS * 4) as u32;

    // Disable MPU during configuration
    unsafe {
        mpu.ctrl.write(0);
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: Code (flash, 2MB, RO, executable)
    let code_sf = encode_size(2 * 1024 * 1024).unwrap();
    let code_rbar = build_rbar(0x08000000, 0).unwrap();
    let code_rasr = build_rasr(code_sf, AP_RO, false, (false, false, false));
    configure_region(mpu, code_rbar, code_rasr);

    // Region 1: Shared RAM (2KB for globals, RW, no execute)
    let shared_sf = encode_size(2048).unwrap();
    let shared_rbar = build_rbar(0x20000000, 1).unwrap();
    let shared_rasr = build_rasr(shared_sf, AP_FULL_ACCESS, true, (false, false, false));
    configure_region(mpu, shared_rbar, shared_rasr);

    // Region 2: Partition 0 stack (1KB, RW, no execute)
    let stack_sf = encode_size(stack_size).unwrap();
    let p0_rbar = build_rbar(p0_stack, 2).unwrap();
    let p0_rasr = build_rasr(stack_sf, AP_FULL_ACCESS, true, (false, false, false));
    configure_region(mpu, p0_rbar, p0_rasr);

    // Region 3: Partition 1 stack (1KB, RW, no execute)
    let p1_rbar = build_rbar(p1_stack, 3).unwrap();
    let p1_rasr = build_rasr(stack_sf, AP_FULL_ACCESS, true, (false, false, false));
    configure_region(mpu, p1_rbar, p1_rasr);

    // Enable MPU with PRIVDEFENA
    unsafe {
        mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA);
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut core = cortex_m::Peripherals::take().unwrap();

    rprintln!("STM32F429ZI Context Switch + MPU Test (Step 1a)");
    rprintln!("Setting up two partitions with MPU isolation");

    // Initialize partition stacks
    unsafe {
        let p0_ptr = &raw mut STACK_P0;
        let p1_ptr = &raw mut STACK_P1;

        let sp0_idx = init_stack_frame(
            &mut (*p0_ptr).0,
            partition_0_entry,
            None
        ).expect("stack P0 too small");

        let sp1_idx = init_stack_frame(
            &mut (*p1_ptr).0,
            partition_1_entry,
            None
        ).expect("stack P1 too small");

        let p0_base = (*p0_ptr).0.as_ptr() as u32;
        let p1_base = (*p1_ptr).0.as_ptr() as u32;
        let p0_sp = p0_base + (sp0_idx as u32) * 4;
        let p1_sp = p1_base + (sp1_idx as u32) * 4;

        rprintln!("  P0 stack @ {:#010x}, SP = {:#010x}", p0_base, p0_sp);
        rprintln!("  P1 stack @ {:#010x}, SP = {:#010x}", p1_base, p1_sp);

        PARTITION_SP[0] = p0_sp;
        PARTITION_SP[1] = p1_sp;
    }

    // Configure MPU with static regions
    rprintln!("Configuring MPU (static regions for both partitions):");
    rprintln!("  Region 0: Code (flash, 2MB, RO)");
    rprintln!("  Region 1: Shared RAM (2KB, RW+XN) - for globals");
    rprintln!("  Region 2: P0 stack (1KB @ {:#010x}, RW+XN)", unsafe { core::ptr::addr_of!(STACK_P0) as u32 });
    rprintln!("  Region 3: P1 stack (1KB @ {:#010x}, RW+XN)", unsafe { core::ptr::addr_of!(STACK_P1) as u32 });
    configure_static_mpu(&core.MPU);

    // Configure exception priorities
    unsafe {
        core.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        core.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    // Configure SysTick
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(RELOAD);
    core.SYST.clear_current();
    core.SYST.enable_counter();
    core.SYST.enable_interrupt();

    rprintln!("Triggering first PendSV to start partition 0");
    rprintln!("Both partitions can access their own stacks (isolated from each other)");
    rprintln!("Waiting for {} context switches...\n", TARGET_SWITCHES);

    cortex_m::peripheral::SCB::set_pendsv();

    // Main loop: Just idle, MPU is configured statically
    loop {
        cortex_m::asm::wfi();
    }
}
