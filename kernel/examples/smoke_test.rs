//! QEMU smoke integration test: boot → yield → semaphore IPC → exit.
//!
//! Regression gate exercising the full kernel lifecycle:
//!
//! 1. Kernel boots with two partitions via `define_unified_harness!`.
//! 2. Round-robin scheduler gives each partition 3 ticks.
//! 3. Partition 0 calls SYS_YIELD (return code 0 = success), then
//!    signals semaphore 0 (count 0→1).
//! 4. Partition 1 waits on semaphore 0 (count 1→0, rc = SEM_ACQUIRED).
//! 5. SysTick callback verifies all three return codes, exits via semihosting.
//!
//! Run:  cargo run --target thumbv7m-none-eabi --features qemu,log-semihosting --example smoke_test
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::partition::PartitionConfig;
use kernel::scheduler::ScheduleTable;
use kernel::semaphore::Semaphore;
use kernel::svc::Kernel;
use kernel::syscall::{SYS_SEM_SIGNAL, SYS_SEM_WAIT, SYS_YIELD};
use kernel::{DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal};
use panic_semihosting as _;

kernel::compose_kernel_config!(SmokeConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

const NUM_PARTITIONS: usize = SmokeConfig::N;
const STACK_WORDS: usize = SmokeConfig::STACK_WORDS;
const TIMEOUT_TICKS: u32 = 50;

/// SYS_YIELD success return code.
const YIELD_OK: u32 = 0;
/// SYS_SEM_SIGNAL success return code.
const SEM_SIGNAL_OK: u32 = 0;
/// SYS_SEM_WAIT returns 1 when the semaphore was acquired immediately
/// (count was > 0 and decremented without blocking).
const SEM_ACQUIRED: u32 = 1;

/// Sentinel: partition has not yet executed its syscall.
///
/// Chosen to avoid overlap with both success codes (small integers) and
/// `SvcError` codes (`0xFFFF_FFF6..=0xFFFF_FFFF`).
const NOT_YET: u32 = 0xDEAD_C0DE;

/// Partition 0 stores SYS_YIELD return code here.
static YIELD_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Partition 0 stores SYS_SEM_SIGNAL return code here.
static SIG_RC: AtomicU32 = AtomicU32::new(NOT_YET);
/// Partition 1 stores SYS_SEM_WAIT return code here.
static WAIT_RC: AtomicU32 = AtomicU32::new(NOT_YET);

kernel::define_unified_harness!(SmokeConfig, NUM_PARTITIONS, STACK_WORDS, |tick, _k| {
    let y = YIELD_RC.load(Ordering::Acquire);
    let sig = SIG_RC.load(Ordering::Acquire);
    let wait = WAIT_RC.load(Ordering::Acquire);

    if y == NOT_YET || sig == NOT_YET || wait == NOT_YET {
        if tick >= TIMEOUT_TICKS {
            hprintln!(
                "smoke_test: FAIL: timeout after {} ticks (yield={:#x}, sig={:#x}, wait={:#x})",
                tick,
                y,
                sig,
                wait
            );
            debug::exit(debug::EXIT_FAILURE);
        }
        return;
    }

    if y == YIELD_OK && sig == SEM_SIGNAL_OK && wait == SEM_ACQUIRED {
        hprintln!("smoke_test: PASS (yield={}, sig={}, wait={})", y, sig, wait);
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!(
            "smoke_test: FAIL: incorrect return code (yield={:#x} expected {:#x}, sig={:#x} expected {:#x}, wait={:#x} expected {:#x})",
            y, YIELD_OK, sig, SEM_SIGNAL_OK, wait, SEM_ACQUIRED
        );
        debug::exit(debug::EXIT_FAILURE);
    }
});

extern "C" fn p0_main() -> ! {
    // Signal first so the semaphore count is >0 before P1 runs.
    let rc = kernel::svc!(SYS_SEM_SIGNAL, 0u32, 0u32, 0u32);
    SIG_RC.store(rc, Ordering::Release);
    // Yield voluntarily — exercises SYS_YIELD and triggers context switch.
    let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    YIELD_RC.store(rc, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

extern "C" fn p1_main() -> ! {
    let rc = kernel::svc!(SYS_SEM_WAIT, 0u32, 0u32, 0u32);
    WAIT_RC.store(rc, Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

/// Print an error message via semihosting and exit with failure.
///
/// Used instead of `panic!`/`.expect()` so headless CI sees a clean
/// diagnostic rather than a panic handler backtrace.
macro_rules! fail_init {
    ($($arg:tt)*) => {{
        hprintln!($($arg)*);
        debug::exit(debug::EXIT_FAILURE);
        loop {} // unreachable — satisfies `-> !`
    }};
}

#[entry]
fn main() -> ! {
    let mut p = match cortex_m::Peripherals::take() {
        Some(p) => p,
        None => fail_init!("smoke_test: FAIL: Peripherals::take returned None"),
    };
    hprintln!("smoke_test: start");

    let sched = match ScheduleTable::<{ SmokeConfig::SCHED }>::round_robin(2, 3) {
        Ok(s) => s,
        Err(e) => fail_init!("smoke_test: FAIL: round_robin: {}", e),
    };

    let cfgs = PartitionConfig::sentinel_array::<NUM_PARTITIONS>(STACK_WORDS);

    let mut k = match Kernel::<SmokeConfig>::create(sched, &cfgs) {
        Ok(k) => k,
        Err(_e) => fail_init!("smoke_test: FAIL: Kernel::create failed"),
    };

    // Semaphore 0: initial count=0, max=1.
    // P0 signals (0→1), then P1 waits and acquires (1→0).
    if k.semaphores_mut().add(Semaphore::new(0, 1)).is_err() {
        fail_init!("smoke_test: FAIL: add semaphore failed");
    }

    store_kernel(k);

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p0_main, 0), (p1_main, 0)];
    match boot(&parts, &mut p) {
        Ok(never) => match never {},
        Err(_e) => fail_init!("smoke_test: FAIL: boot failed"),
    }
}
