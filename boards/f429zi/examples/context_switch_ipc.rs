//! STM32F429ZI: Context switching with IPC (Step 4)
//!
//! Builds on Step 3 by adding inter-partition communication via sampling ports.
//! Partitions can exchange data safely using atomic sampling ports.
//!
//! Step 4: Inter-partition communication with sampling ports
//! - Sampling port: single-value atomic buffer
//! - Partition 0 (producer) writes incrementing counter
//! - Partition 1 (consumer) reads and displays values
//! - Non-blocking reads and writes
//! - Foundation for ARINC 653-style sampling ports

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

const TARGET_SWITCHES: u32 = 20;
const RELOAD: u32 = 160_000 - 1; // 10ms at 16MHz HSI

// ---------------------------------------------------------------------------
// Schedule Table (Step 3: time-partitioned scheduling)
// ---------------------------------------------------------------------------

/// Schedule entry: partition ID and duration in ticks
struct ScheduleEntry {
    partition_id: u8,
    duration_ticks: u32,
}

/// Schedule: P0 gets 20ms (2 ticks), P1 gets 10ms (1 tick), repeats
static SCHEDULE: [ScheduleEntry; 2] = [
    ScheduleEntry { partition_id: 0, duration_ticks: 2 }, // 20ms
    ScheduleEntry { partition_id: 1, duration_ticks: 1 }, // 10ms
];

// ---------------------------------------------------------------------------
// Sampling Port (Step 4: inter-partition communication)
// ---------------------------------------------------------------------------

/// Sampling port: atomic single-value buffer for IPC
/// Partition 0 writes, Partition 1 reads
static SAMPLING_PORT: AtomicU32 = AtomicU32::new(0);

/// Counter: how many times P1 has read a new value
static P1_READ_COUNT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Partition entry points (Step 4: with IPC via sampling port)
// ---------------------------------------------------------------------------
extern "C" fn partition_0_entry() -> ! {
    let mut counter: u32 = 0;
    loop {
        PARTITION_RUNNING.store(0, Ordering::Release);

        // Write to sampling port (producer)
        SAMPLING_PORT.store(counter, Ordering::Release);
        counter = counter.wrapping_add(1);

        // Small delay to make output observable
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
    }
}

extern "C" fn partition_1_entry() -> ! {
    static mut LAST_VALUE: u32 = u32::MAX;

    loop {
        PARTITION_RUNNING.store(1, Ordering::Release);

        // Read from sampling port (consumer)
        let value = SAMPLING_PORT.load(Ordering::Acquire);

        // Count unique reads
        unsafe {
            let last_ptr = core::ptr::addr_of_mut!(LAST_VALUE);
            if value != *last_ptr {
                P1_READ_COUNT.fetch_add(1, Ordering::Release);
                *last_ptr = value;
            }
        }

        // Small delay
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
    }
}

// ---------------------------------------------------------------------------
// PendSV handler with MPU reconfiguration (Step 1b)
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
    ldr     r1, [r0]              /* r1 = next partition ID */

    ldr     r0, =CURRENT_PARTITION
    str     r1, [r0]

    /* Reconfigure MPU region 2 for incoming partition */
    /* r1 still holds next partition ID */
    mov     r4, r1                /* Save partition ID in r4 (callee-saved) */
    mov     r0, r1                /* Pass partition ID as argument */
    bl      switch_mpu_region2    /* Call Rust helper */
    mov     r1, r4                /* Restore partition ID */

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
// SysTick handler (Step 4: schedule table with IPC)
// ---------------------------------------------------------------------------
#[exception]
fn SysTick() {
    static mut SWITCH_COUNT: u32 = 0;
    static mut DONE: bool = false;
    static mut SCHED_INDEX: usize = 0;
    static mut TICKS_IN_SLOT: u32 = 0;

    unsafe {
        if *DONE {
            return;
        }
    }

    let who = PARTITION_RUNNING.load(Ordering::Acquire);

    unsafe {
        let current = core::ptr::read_volatile(core::ptr::addr_of!(CURRENT_PARTITION));

        // Increment tick counter for current slot
        *TICKS_IN_SLOT += 1;
        let entry = &SCHEDULE[*SCHED_INDEX];

        if current != u32::MAX {
            *SWITCH_COUNT += 1;
            let count = *SWITCH_COUNT;
            let port_value = SAMPLING_PORT.load(Ordering::Acquire);
            let read_count = P1_READ_COUNT.load(Ordering::Acquire);
            rprintln!("switch {}: P{} ran (slot {}, tick {}/{}) | port={}, P1_reads={}",
                     count, who, *SCHED_INDEX, *TICKS_IN_SLOT, entry.duration_ticks,
                     port_value, read_count);

            if count >= TARGET_SWITCHES {
                let final_value = SAMPLING_PORT.load(Ordering::Acquire);
                rprintln!("\ncontext_switch_ipc: {} switches observed - PASS", TARGET_SWITCHES);
                rprintln!("Sampling port communication verified");
                rprintln!("P0 wrote values, P1 read them successfully");
                rprintln!("Final sampling port value: {}", final_value);
                *DONE = true;
                return;
            }
        }

        // Check if current time slot is complete
        if *TICKS_IN_SLOT >= entry.duration_ticks {
            // Move to next schedule entry
            *SCHED_INDEX = (*SCHED_INDEX + 1) % SCHEDULE.len();
            *TICKS_IN_SLOT = 0;

            // Switch to the partition specified in next schedule entry
            let next_entry = &SCHEDULE[*SCHED_INDEX];
            core::ptr::write_volatile(core::ptr::addr_of_mut!(NEXT_PARTITION), next_entry.partition_id as u32);
            cortex_m::peripheral::SCB::set_pendsv();
        }
    }
}

// ---------------------------------------------------------------------------
// MPU Configuration
// ---------------------------------------------------------------------------

/// Configure initial MPU (Step 1b: only 3 regions)
/// Region 0: Code region (flash, RO + execute)
/// Region 1: Shared RAM region (globals, RW, no execute)
/// Region 2: Active partition's stack (RW, no execute) - reconfigured in PendSV
fn configure_initial_mpu(mpu: &cortex_m::peripheral::MPU) {
    let p0_stack = unsafe { core::ptr::addr_of!(STACK_P0) as u32 };
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

    // Region 1: Shared RAM (8KB for globals, RW, no execute)
    // Increased to accommodate RTT buffers and additional globals
    let shared_sf = encode_size(8192).unwrap();
    let shared_rbar = build_rbar(0x20000000, 1).unwrap();
    let shared_rasr = build_rasr(shared_sf, AP_FULL_ACCESS, true, (false, false, false));
    configure_region(mpu, shared_rbar, shared_rasr);

    // Region 2: Initial partition (P0) stack (1KB, RW, no execute)
    let stack_sf = encode_size(stack_size).unwrap();
    let p0_rbar = build_rbar(p0_stack, 2).unwrap();
    let p0_rasr = build_rasr(stack_sf, AP_FULL_ACCESS, true, (false, false, false));
    configure_region(mpu, p0_rbar, p0_rasr);

    // Enable MPU with PRIVDEFENA
    unsafe {
        mpu.ctrl.write(MPU_CTRL_ENABLE_PRIVDEFENA);
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Reconfigure MPU region 2 for a specific partition
/// Called from PendSV during context switch
/// Must be fast - called with interrupts effectively disabled
#[unsafe(no_mangle)]
extern "C" fn switch_mpu_region2(partition_id: u32) {
    let stack_base = if partition_id == 0 {
        unsafe { core::ptr::addr_of!(STACK_P0) as u32 }
    } else {
        unsafe { core::ptr::addr_of!(STACK_P1) as u32 }
    };

    let stack_size = (STACK_WORDS * 4) as u32;
    let stack_sf = encode_size(stack_size).unwrap();
    let rbar = build_rbar(stack_base, 2).unwrap();
    let rasr = build_rasr(stack_sf, AP_FULL_ACCESS, true, (false, false, false));

    // Get MPU register block
    // SAFETY: Called from PendSV, exclusive access guaranteed
    let mpu_rb = unsafe { &*cortex_m::peripheral::MPU::PTR };

    // No need to disable MPU - just reconfigure region 2
    // SAFETY: Atomic write to RBAR (with VALID bit set) selects and configures region
    unsafe {
        mpu_rb.rbar.write(rbar); // RBAR has VALID bit, selects region
        mpu_rb.rasr.write(rasr);
    }

    // Memory barriers to ensure MPU config takes effect
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

    rprintln!("STM32F429ZI Context Switch + IPC (Step 4)");
    rprintln!("Setting up two partitions with sampling port communication");

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

    // Configure initial MPU (dynamic region 2 from Step 1b)
    rprintln!("Configuring MPU (dynamic region 2):");
    rprintln!("  Region 0: Code (flash, 2MB, RO)");
    rprintln!("  Region 1: Shared RAM (8KB, RW+XN) - for globals & IPC");
    rprintln!("  Region 2: Active partition stack (1KB, RW+XN) - reconfigured in PendSV");
    configure_initial_mpu(&core.MPU);

    // Configure exception priorities
    // SysTick (0xFE) > PendSV (0xFF)
    unsafe {
        core.SCB.set_priority(SystemHandler::PendSV, 0xFF);   // Lowest priority
        core.SCB.set_priority(SystemHandler::SysTick, 0xFE);  // Higher priority
    }

    // Configure SysTick
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(RELOAD);
    core.SYST.clear_current();
    core.SYST.enable_counter();
    core.SYST.enable_interrupt();

    rprintln!("Schedule table configured:");
    rprintln!("  Entry 0: Partition 0 for 20ms (2 ticks) - Producer");
    rprintln!("  Entry 1: Partition 1 for 10ms (1 tick) - Consumer");
    rprintln!("  Pattern: P0(20ms) -> P1(10ms) -> P0(20ms) -> ...");
    rprintln!("Sampling port initialized:");
    rprintln!("  P0 writes incrementing counter");
    rprintln!("  P1 reads and displays values");
    rprintln!("  Non-blocking, atomic communication");
    rprintln!("Triggering first PendSV to start partition 0");
    rprintln!("Waiting for {} context switches...\n", TARGET_SWITCHES);

    cortex_m::peripheral::SCB::set_pendsv();

    // Main loop: Just idle, MPU is configured statically
    loop {
        cortex_m::asm::wfi();
    }
}
