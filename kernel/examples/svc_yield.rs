#![no_std]
#![no_main]

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

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
