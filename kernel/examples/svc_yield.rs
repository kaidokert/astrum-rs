//! QEMU test: verify SVC yield syscall triggers PendSV.
//!
//! This test exercises the SVC dispatcher's yield path (SYS_YIELD = 0).
//! It configures SVCall and PendSV priorities, switches to PSP (Thread mode),
//! and issues `svc #0`. The kernel's SVC handler should set the PendSV pending
//! bit. A minimal PendSV stub records that it fired, and the test verifies
//! both the SVC return value (r0 = 0 for success) and that PendSV executed.
//!
//! ## Static mut usage
//!
//! This test intentionally uses `static mut` for two items:
//!
//! - `PSTACK`: The process stack must be at a stable address so we can set PSP
//!   to point to it. The test runs outside the unified harness because it needs
//!   a minimal PendSV stub (not a full context switch) to verify yield triggers
//!   PendSV without side effects.
//!
//! - `PENDSV_FIRED`: A flag written by the inline-assembly PendSV handler and
//!   read by main. The PendSV handler runs asynchronously after the SVC returns,
//!   so the flag must be in a fixed location accessible from global_asm. Using
//!   a Mutex<RefCell> would require the PendSV assembly to call Rust functions,
//!   defeating the purpose of testing the raw mechanism.
//!
//! These items cannot use the unified harness infrastructure because this test
//! specifically validates the low-level SVC-to-PendSV triggering path before
//! any higher-level kernel abstractions.

#![no_std]
#![no_main]

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;

// Force the linker to pull in the SVCall assembly trampoline + dispatch_svc
// from the kernel library.
#[used]
static _SVC_HANDLER: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) =
    kernel::svc::SVC_HANDLER;

#[allow(dead_code)]
static mut PSTACK: [u32; 256] = [0; 256];
#[no_mangle]
static mut PENDSV_FIRED: u32 = 0;

// Test-only PendSV stub: just records that PendSV fired.
// This is co-located with the test because it exists purely for
// verification and is not a real context-switch handler.
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".global PendSV",
    ".type PendSV, %function",
    "PendSV:",
    "ldr r0, =PENDSV_FIRED",
    "movs r1, #1",
    "str r1, [r0]",
    "bx lr",
    ".size PendSV, . - PendSV",
);

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("svc_yield: start");
    unsafe {
        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
    }
    let ret: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        let psp = (&raw const PSTACK) as *const u32 as u32 + 256 * 4;
        core::arch::asm!(
            "msr psp, {0}", "mrs {1}, control", "orr {1}, {1}, #2",
            "msr control, {1}", "isb",
            in(reg) psp, out(reg) _,
        );
        core::arch::asm!(
            "mov r0, #0", "svc #0", out("r0") ret,
            out("r1") _, out("r2") _, out("r3") _, out("r12") _,
        );
    }
    #[cfg(not(target_arch = "arm"))]
    {
        ret = 0;
    }
    let pf = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(PENDSV_FIRED)) };
    if ret == 0 && pf == 1 {
        hprintln!("svc_yield: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("svc_yield: FAIL r0={} pf={}", ret, pf);
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}
