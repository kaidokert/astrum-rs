//! Debug test for the unified harness boot sequence.
//!
//! # Purpose
//!
//! This example isolates and validates the boot sequence used by the unified
//! kernel harness. It manually constructs and initializes all boot components
//! (stacks, kernel, schedule, partition configs) without relying on the
//! `define_unified_harness!` macro. This allows debugging boot issues by
//! providing detailed semihosting output at each step.
//!
//! # Architecture
//!
//! This example uses `define_unified_kernel!` to provide the kernel storage
//! and PendSV offset constants, but manually manages stack initialization
//! to enable detailed debug output.
//!
//! - **STACKS**: Raw stack storage must be mutable for `init_stack_frame` to
//!   write the initial context. Using `Mutex<RefCell<>>` would add overhead
//!   and complicate the low-level stack pointer arithmetic.
//!
//! - **Kernel state**: Managed by `define_unified_kernel!` which provides
//!   `store_kernel()` and `set_partition_sp()` for accessing kernel internals.
//!
//! # Test Behavior
//!
//! 1. Creates a minimal kernel with one partition
//! 2. Initializes the partition stack with an entry point
//! 3. Triggers PendSV to switch to the partition
//! 4. The partition sets an atomic flag when it runs
//! 5. SysTick checks the flag and reports PASS/FAIL
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    context::init_stack_frame,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

// Manual stacks (not using define_unified_harness!)
#[repr(C, align(1024))]
struct AlignedStack([u32; TestConfig::STACK_WORDS]);

static mut STACKS: [AlignedStack; TestConfig::N] = {
    const ZERO: AlignedStack = AlignedStack([0; TestConfig::STACK_WORDS]);
    [ZERO; TestConfig::N]
};

// Use define_unified_kernel! to provide kernel storage and offset symbols for PendSV
kernel::define_unified_kernel!(TestConfig, |_k| {});

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;

// Simple PendSV handler
kernel::define_pendsv!();

// Atomic flag to indicate partition ran
static PARTITION_RAN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

extern "C" fn partition_main() -> ! {
    // Set flag instead of using semihosting (which requires privileged mode)
    PARTITION_RAN.store(true, core::sync::atomic::Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
fn SysTick() {
    static mut TICK: u32 = 0;
    // Handler-local static: cortex-m-rt guarantees exclusive access.
    *TICK += 1;
    let tick = *TICK;
    if tick == 1 {
        hprintln!("SysTick #1");
        if PARTITION_RAN.load(core::sync::atomic::Ordering::Acquire) {
            hprintln!("debug_boot: PASS - partition ran!");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("debug_boot: partition not yet run");
        }
    } else if tick == 2 {
        hprintln!("SysTick #2");
        if PARTITION_RAN.load(core::sync::atomic::Ordering::Acquire) {
            hprintln!("debug_boot: PASS - partition ran!");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("debug_boot: FAIL - partition never ran");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("HardFault!");
    hprintln!("  r0  = {:#010x}", ef.r0());
    hprintln!("  r1  = {:#010x}", ef.r1());
    hprintln!("  r2  = {:#010x}", ef.r2());
    hprintln!("  r3  = {:#010x}", ef.r3());
    hprintln!("  r12 = {:#010x}", ef.r12());
    hprintln!("  lr  = {:#010x}", ef.lr());
    hprintln!("  pc  = {:#010x}", ef.pc());
    hprintln!("  xpsr = {:#010x}", ef.xpsr());
    debug::exit(debug::EXIT_FAILURE);
    #[allow(clippy::empty_loop)]
    loop {}
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("debug_boot: start");

    // Build schedule
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");

    // Create kernel
    let mut k = Kernel::<TestConfig>::create_sentinels(sched).expect("kernel");
    hprintln!("debug_boot: kernel created");

    // Start schedule and get first partition (BEFORE storing kernel)
    hprintln!("debug_boot: starting schedule");
    let first_pid = kernel::svc::scheduler::start_schedule(&mut k);
    hprintln!("debug_boot: first_pid = {:?}", first_pid);

    // Set next_partition before storing kernel
    let pid = first_pid.expect("no partition");
    k.set_next_partition(pid);
    hprintln!("debug_boot: next_partition = {}", pid);

    // Store kernel via store_kernel() from define_unified_kernel!
    store_kernel(k);
    hprintln!("debug_boot: kernel stored");

    // Initialize stacks AFTER storing kernel (so set_partition_sp can access it)
    // SAFETY: single-core, interrupts disabled — exclusive access to statics.
    #[allow(clippy::deref_addrof)]
    unsafe {
        let stacks = &mut *(&raw mut STACKS);

        let ep = partition_main as *const () as u32;
        hprintln!("debug_boot: entry point = {:#010x}", ep);

        let stk = &mut stacks[0].0;
        let ix = init_stack_frame(stk, ep, Some(0)).expect("init_stack_frame");
        let sp = stk.as_ptr() as u32 + (ix as u32) * 4;
        set_partition_sp(0, sp);

        hprintln!(
            "debug_boot: stack @ {:#010x}, SP = {:#010x}",
            stk.as_ptr() as u32,
            sp
        );

        // Debug: print the initialized stack frame
        hprintln!(
            "debug_boot: stack[{}..{}] (SavedContext r4-r11):",
            ix,
            ix + 8
        );
        for j in 0..8 {
            hprintln!("  stack[{}] = {:#010x}", ix + j, stk[ix + j]);
        }
        hprintln!(
            "debug_boot: stack[{}..{}] (ExceptionFrame):",
            ix + 8,
            ix + 16
        );
        for j in 8..16 {
            hprintln!("  stack[{}] = {:#010x}", ix + j, stk[ix + j]);
        }
    }

    // Set exception priorities per the three-tier model:
    //   Tier 1 (highest): SVCall  – synchronous syscall entry
    //   Tier 2 (middle):  SysTick – time-slice preemption
    //   Tier 3 (lowest):  PendSV  – deferred context switch
    //
    // SAFETY: set_priority is unsafe because changing priority levels can
    // break priority-based critical sections. Interrupts are not yet enabled
    // so no preemption or race conditions can occur during configuration.
    unsafe {
        p.SCB
            .set_priority(SystemHandler::SVCall, TestConfig::SVCALL_PRIORITY);
        p.SCB
            .set_priority(SystemHandler::PendSV, TestConfig::PENDSV_PRIORITY);
        p.SCB
            .set_priority(SystemHandler::SysTick, TestConfig::SYSTICK_PRIORITY);
    }
    hprintln!("debug_boot: priorities set");

    // Configure SysTick
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    hprintln!("debug_boot: SysTick configured");

    // Trigger PendSV
    hprintln!("debug_boot: triggering PendSV");
    cortex_m::peripheral::SCB::set_pendsv();

    hprintln!("debug_boot: entering idle loop");
    loop {
        cortex_m::asm::wfi();
    }
}
