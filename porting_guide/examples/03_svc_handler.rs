//! SVC voluntary yield coexisting with SysTick preemption.
//! PASS when **>= 4 SVC yields** and **>= 4 SysTick preemptions** observed,
//! AND both partitions have made forward progress (partition-side counters).

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::debug;
use kernel::context::ExceptionFrame;
#[allow(unused_imports)]
use kernel::kpanic as _;
use porting_guide::klog;

#[used]
static _SVC_REF: kernel::svc::SvcDispatchFn = kernel::svc::SVC_HANDLER;

const STACK_WORDS: usize = 256;
#[repr(align(8))]
#[allow(dead_code)]
struct Stk([u32; STACK_WORDS]);

// SAFETY: Each stack is only accessed during init (before scheduling starts)
// and then exclusively by its owning partition via PSP, so no data races occur.
static mut STK0: Stk = Stk([0; STACK_WORDS]);
static mut STK1: Stk = Stk([0; STACK_WORDS]);

// Context-switch state (accessed from PendSV naked asm via symbol name).
// SAFETY: These are only mutated inside PendSV, which runs at the lowest
// exception priority and cannot preempt itself, so accesses are inherently
// sequential. Initialization in main() completes before PendSV is first pended.
#[no_mangle]
static mut PARTITION_SP: [u32; 2] = [0; 2];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = 0;

static SVC_COUNT: AtomicU32 = AtomicU32::new(0);
static TICK_COUNT: AtomicU32 = AtomicU32::new(0);
static TICK: AtomicU32 = AtomicU32::new(0);
// Partition-side progress counters — verify context switches result in real execution.
static P0_PROGRESS: AtomicU32 = AtomicU32::new(0);
static P1_PROGRESS: AtomicU32 = AtomicU32::new(0);
const GOAL: u32 = 4;
const TIMEOUT: u32 = 500;

// SAFETY: Called only from SVC exception context; `frame` is a valid
// exception frame provided by the hardware. Writing `frame.r0` is safe
// because we own the frame for the duration of this handler.
unsafe extern "C" fn svc_hook(frame: &mut ExceptionFrame) {
    SVC_COUNT.fetch_add(1, Ordering::Relaxed);
    cortex_m::peripheral::SCB::set_pendsv();
    frame.r0 = 0;
}

extern "C" fn partition_0() -> ! {
    loop {
        P0_PROGRESS.fetch_add(1, Ordering::Relaxed);
        #[cfg(target_arch = "arm")]
        // SAFETY: The SVC instruction triggers a supervisor call. Register clobbers
        // are declared so the compiler knows r0-r3 are modified by the call.
        unsafe {
            core::arch::asm!(
                "mov r0, #0", "svc #0",
                out("r0") _, out("r1") _, out("r2") _, out("r3") _,
            );
        }
        for _ in 0..200 {
            cortex_m::asm::nop();
        }
    }
}

extern "C" fn partition_1() -> ! {
    loop {
        P1_PROGRESS.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::nop();
    }
}

/// # Safety
/// - `stack` must point to a valid, exclusively owned `[u32; STACK_WORDS]`
///   that is 8-byte aligned and remains valid for the partition's entire lifetime.
/// - `STACK_WORDS` must be >= 16 to hold the full synthetic frame.
unsafe fn init_stack(stack: *mut u32, entry: extern "C" fn() -> !) -> u32 {
    // SAFETY: caller guarantees `stack` is valid for STACK_WORDS words.
    // `top` points one past the last element (standard stack-top convention).
    let top = stack.add(STACK_WORDS);

    // Build a 16-word synthetic frame: 8 software-saved (r4-r11) zeroed,
    // plus 8 hardware exception frame slots with targeted writes.
    for i in 4..=16 {
        top.sub(i).write_volatile(0);
    }
    top.sub(1).write_volatile(0x0100_0000); // xPSR (Thumb)
    top.sub(2).write_volatile(entry as usize as u32); // PC
    top.sub(3).write_volatile(0xFFFF_FFFF); // LR

    // Return PSP pointing at bottom of frame (r4 position).
    top.sub(16) as u32
}

// PendSV: save r4-r11 + PSP for current partition, toggle, restore next.
// SAFETY: This naked function is the PendSV handler. It only accesses
// PARTITION_SP and CURRENT_PARTITION, which are never concurrently mutated
// (PendSV is the lowest priority exception and cannot preempt itself).
// The inline assembly correctly saves/restores the callee-saved registers
// and PSP, maintaining ABI invariants across context switches.
#[cfg(target_arch = "arm")]
#[unsafe(naked)]
#[export_name = "PendSV"]
extern "C" fn pend_sv_handler() {
    core::arch::naked_asm!(
        "cpsid   i",
        "mrs     r0, psp",
        "stmdb   r0!, {{r4-r11}}",
        "ldr     r1, =CURRENT_PARTITION",
        "ldr     r2, [r1]",
        "ldr     r3, =PARTITION_SP",
        "str     r0, [r3, r2, lsl #2]",
        "eor     r2, r2, #1",
        "str     r2, [r1]",
        "ldr     r0, [r3, r2, lsl #2]",
        "ldmia   r0!, {{r4-r11}}",
        "msr     psp, r0",
        // Clear only PENDSVCLR — leave PENDSTCLR to hardware/kernel defaults.
        "ldr     r0, =0xE000ED04",
        "ldr     r1, =0x08000000", // PENDSVCLR only
        "str     r1, [r0]",
        "dsb",
        "cpsie   i",
        "ldr     lr, =0xFFFFFFFD",
        "bx      lr",
    );
}

#[exception]
fn SysTick() {
    let tick = TICK.fetch_add(1, Ordering::Relaxed) + 1;
    TICK_COUNT.fetch_add(1, Ordering::Relaxed);
    let svc = SVC_COUNT.load(Ordering::Relaxed);
    let ticks = TICK_COUNT.load(Ordering::Relaxed);
    let p0 = P0_PROGRESS.load(Ordering::Relaxed);
    let p1 = P1_PROGRESS.load(Ordering::Relaxed);
    if svc >= GOAL && ticks >= GOAL && p0 > 0 && p1 > 0 {
        klog!(
            "03_svc_handler: PASS (svc_yields={}, tick_preemptions={}, p0={}, p1={})",
            svc,
            ticks,
            p0,
            p1
        );
        debug::exit(debug::EXIT_SUCCESS);
    }
    if tick >= TIMEOUT {
        klog!(
            "03_svc_handler: FAIL timeout (svc={}, ticks={}, p0={}, p1={})",
            svc,
            ticks,
            p0,
            p1
        );
        debug::exit(debug::EXIT_FAILURE);
    }
    cortex_m::peripheral::SCB::set_pendsv();
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().expect("Peripherals::take");
    klog!("03_svc_handler: SVC + SysTick coexistence demo");
    kernel::svc::set_dispatch_hook(svc_hook);

    // Initialise partition 1's stack with a fake exception frame.
    // Partition 0 doesn't need one — we enter it directly.
    // SAFETY: Called before scheduling starts; STK1, PARTITION_SP, and
    // CURRENT_PARTITION are not yet accessed by any exception handler.
    unsafe {
        let s1 = core::ptr::addr_of_mut!(STK1) as *mut u32;
        PARTITION_SP[1] = init_stack(s1, partition_1);
        CURRENT_PARTITION = 0;
    }

    // Set PendSV to lowest priority, SVCall to highest.
    // SAFETY: We own `p` (the peripherals singleton) and no exception
    // handler accesses SCB priority registers.
    unsafe {
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
    }

    let mut syst = p.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(10_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    // Point PSP at the top of STK0 (empty — room for hw to push frames).
    // SAFETY: STK0 is valid, exclusively owned, and not yet in use.
    let top = unsafe { (core::ptr::addr_of_mut!(STK0) as *mut u32).add(STACK_WORDS) as u32 };
    // SAFETY: Called before switching to PSP; the stack address is valid.
    unsafe { cortex_m::register::psp::write(top) };

    // Switch Thread mode to use PSP (CONTROL.SPSEL = 1).
    // SAFETY: PSP has been set to a valid stack above. After this point,
    // Thread mode uses PSP and the original MSP is reserved for handlers.
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!(
            "mrs {t}, CONTROL", "orr {t}, {t}, #0x2", "msr CONTROL, {t}", "isb",
            t = out(reg) _,
        );
    }
    klog!("03_svc_handler: entering partition 0");
    partition_0()
}
