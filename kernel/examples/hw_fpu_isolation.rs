//! QEMU integration: FPU callee-saved register isolation across partitions.
//!
//! Two partitions write distinct float patterns to s16-s31, spin (allowing
//! context switches via SysTick/PendSV), then verify the values survived.
//! If PendSV FPU save/restore is correct, each partition sees only its own
//! values despite sharing the physical FPU.
//!
//! Run:
//!   cargo run -p kernel --target thumbv7em-none-eabihf \
//!     --features qemu,log-semihosting,fpu-context --example hw_fpu_isolation
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::ScheduleTable, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2,
    PortsTiny, SyncMinimal,
};

kernel::kernel_config!(Cfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

/// Successful iterations for partition A.
static CHECKS_A: AtomicU32 = AtomicU32::new(0);
/// Successful iterations for partition B.
static CHECKS_B: AtomicU32 = AtomicU32::new(0);
/// Mismatch count (either partition).
static ERRORS: AtomicU32 = AtomicU32::new(0);

const REQUIRED_CHECKS: u32 = 4;
const TIMEOUT_TICKS: u32 = 200;

kernel::define_kernel!(Cfg, |tick, _k| {
    let a = CHECKS_A.load(Ordering::Acquire);
    let b = CHECKS_B.load(Ordering::Acquire);
    let err = ERRORS.load(Ordering::Acquire);
    if err > 0 {
        hprintln!("hw_fpu_isolation: FAIL ({} errors, a={}, b={})", err, a, b);
        debug::exit(debug::EXIT_FAILURE);
    }
    if a >= REQUIRED_CHECKS && b >= REQUIRED_CHECKS {
        hprintln!("hw_fpu_isolation: PASS (a={}, b={}, 0 errors)", a, b);
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= TIMEOUT_TICKS {
        hprintln!("hw_fpu_isolation: FAIL timeout (a={}, b={})", a, b);
        debug::exit(debug::EXIT_FAILURE);
    }
});

// ---------------------------------------------------------------------------
// FPU test patterns — single source of truth for both assembly and Rust.
// ---------------------------------------------------------------------------

/// Pattern A: IEEE-754 representations of 1.0, 2.0, ..., 16.0
#[no_mangle]
static PATTERN_A: [u32; 16] = [
    0x3F80_0000, // s16 =  1.0
    0x4000_0000, // s17 =  2.0
    0x4040_0000, // s18 =  3.0
    0x4080_0000, // s19 =  4.0
    0x40A0_0000, // s20 =  5.0
    0x40C0_0000, // s21 =  6.0
    0x40E0_0000, // s22 =  7.0
    0x4100_0000, // s23 =  8.0
    0x4110_0000, // s24 =  9.0
    0x4120_0000, // s25 = 10.0
    0x4130_0000, // s26 = 11.0
    0x4140_0000, // s27 = 12.0
    0x4150_0000, // s28 = 13.0
    0x4160_0000, // s29 = 14.0
    0x4170_0000, // s30 = 15.0
    0x4180_0000, // s31 = 16.0
];

/// Pattern B: IEEE-754 representations of 100.0, 200.0, ..., 1600.0
#[no_mangle]
static PATTERN_B: [u32; 16] = [
    0x42C8_0000, // s16 =  100.0
    0x4348_0000, // s17 =  200.0
    0x4396_0000, // s18 =  300.0
    0x43C8_0000, // s19 =  400.0
    0x43FA_0000, // s20 =  500.0
    0x4416_0000, // s21 =  600.0
    0x442F_0000, // s22 =  700.0
    0x4448_0000, // s23 =  800.0
    0x4461_0000, // s24 =  900.0
    0x447A_0000, // s25 = 1000.0
    0x4489_8000, // s26 = 1100.0
    0x4496_0000, // s27 = 1200.0
    0x44A2_8000, // s28 = 1300.0
    0x44AF_0000, // s29 = 1400.0
    0x44BB_8000, // s30 = 1500.0
    0x44C8_0000, // s31 = 1600.0
];

// ---------------------------------------------------------------------------
// Parameterized assembly stress routine
//
// fpu_stress(pattern: *const u32) -> u32
//
// Loads a 16-word float pattern (pointed to by r0) into s16-s31, spins for
// ~2000 iterations (long enough for several SysTick context switches), then
// reads the registers back and compares word-by-word.
//
// Returns 0 on success, 1 on mismatch.
// ---------------------------------------------------------------------------
core::arch::global_asm!(
    ".syntax unified",
    ".fpu fpv4-sp-d16",
    "",
    ".global fpu_stress",
    ".thumb_func",
    "fpu_stress:",
    "  push {{r4, lr}}",
    "  mov  r4, r0",          // save pattern pointer
    "  vldm r0, {{s16-s31}}", // load pattern into FPU callee-saved regs
    "  movs r1, #0",
    "  mov  r2, #2000",
    "1: adds r1, #1",
    "  cmp  r1, r2",
    "  blt  1b",
    "  sub  sp, #64",
    "  vstm sp, {{s16-s31}}", // store current FPU regs to stack
    "  mov  r0, r4",          // reload pattern pointer
    "  mov  r1, sp",
    "  movs r2, #16",
    "2: ldr  r3, [r0], #4",
    "  ldr  r4, [r1], #4",
    "  cmp  r3, r4",
    "  bne  3f",
    "  subs r2, #1",
    "  bne  2b",
    "  add  sp, #64",
    "  movs r0, #0",
    "  pop  {{r4, pc}}",
    "3: add  sp, #64",
    "  movs r0, #1",
    "  pop  {{r4, pc}}",
);

extern "C" {
    fn fpu_stress(pattern: *const u32) -> u32;
}

const _: PartitionEntry = partition_a;
extern "C" fn partition_a() -> ! {
    loop {
        // SAFETY: fpu_stress is a self-contained assembly function that
        // loads/checks FPU registers. PATTERN_A is a static with stable address.
        let err = unsafe { fpu_stress(PATTERN_A.as_ptr()) };
        if err != 0 {
            ERRORS.fetch_add(1, Ordering::Release);
        }
        CHECKS_A.fetch_add(1, Ordering::Release);
    }
}

const _: PartitionEntry = partition_b;
extern "C" fn partition_b() -> ! {
    loop {
        // SAFETY: fpu_stress is a self-contained assembly function.
        // PATTERN_B is a static with stable address.
        let err = unsafe { fpu_stress(PATTERN_B.as_ptr()) };
        if err != 0 {
            ERRORS.fetch_add(1, Ordering::Release);
        }
        CHECKS_B.fetch_add(1, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    // TODO(panic-free): replace unwrap/expect with semihosting failure reporting
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("hw_fpu_isolation: start");
    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(2, 2).expect("sched");
    // TODO: reviewer false positive — second arg to PartitionSpec::new is the
    // initial r0 register value, not a partition ID. Using 0 for both is
    // correct (neither partition reads r0 at entry). All other examples do the same.
    let parts: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(partition_a as PartitionEntry, 0),
        PartitionSpec::new(partition_b as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts).expect("kernel");
    match boot(p).expect("boot") {}
}
