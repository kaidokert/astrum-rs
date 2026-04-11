//! FPU Context Switch Test — verify FPU registers survive partition switches
//!
//! Two partitions each load distinct FPU register patterns and verify them
//! after every context switch.  Any corruption from the other partition's
//! FPU state indicates a save/restore bug.
//!
//! P0: loads s0-s31 with pattern  0x_AA00_00nn (nn = register index)
//! P1: loads s0-s31 with pattern  0x_BB00_00nn
//!
//! After each sys_yield, both partitions re-check their FPU registers.
//! If any register doesn't match, it reports FAIL with the register index
//! and the corrupted value.
//!
//! Success criterion: PASS count > 100 for both partitions with 0 FAILs.
//!
//! Build: cd nrf52 && cargo build --example fpu_context_test --features kernel-fpu

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;

const NUM_PARTITIONS: usize = 2;

static P0_PASS: AtomicU32 = AtomicU32::new(0);
static P0_FAIL: AtomicU32 = AtomicU32::new(0);
static P1_PASS: AtomicU32 = AtomicU32::new(0);
static P1_FAIL: AtomicU32 = AtomicU32::new(0);
// Store the first corruption details
static FAIL_REG: AtomicU32 = AtomicU32::new(0xFFFF);
static FAIL_EXPECTED: AtomicU32 = AtomicU32::new(0);
static FAIL_GOT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(FpuTestCfg<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = nrf52::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 4; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(FpuTestCfg, |tick, _k| {
    if tick % 500 == 0 {
        let p0_pass = P0_PASS.load(Ordering::Acquire);
        let p0_fail = P0_FAIL.load(Ordering::Acquire);
        let p1_pass = P1_PASS.load(Ordering::Acquire);
        let p1_fail = P1_FAIL.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] P0: pass={} fail={} | P1: pass={} fail={}",
            tick, p0_pass, p0_fail, p1_pass, p1_fail
        );
        if p0_fail > 0 || p1_fail > 0 {
            let reg = FAIL_REG.load(Ordering::Acquire);
            let exp = FAIL_EXPECTED.load(Ordering::Acquire);
            let got = FAIL_GOT.load(Ordering::Acquire);
            rprintln!("FAIL: FPU corruption! reg=s{} expected=0x{:08x} got=0x{:08x}", reg, exp, got);
        }
        if p0_pass > 100 && p1_pass > 100 && p0_fail == 0 && p1_fail == 0 {
            rprintln!("SUCCESS: FPU context switch working! P0={} P1={} cycles, 0 corruptions", p0_pass, p1_pass);
        }
    }
});

/// Load all 32 FPU registers with a known pattern, yield, then verify.
/// Pattern: base | register_index (e.g., 0xAA000000 | 5 = 0xAA000005 for s5).
#[inline(never)]
fn fpu_load_and_verify(base: u32, pass_counter: &AtomicU32, fail_counter: &AtomicU32) {
    unsafe {
        core::arch::asm!(
            "vmov s0, {base}",
            "orr {tmp}, {base}, #1",
            "vmov s1, {tmp}",
            "orr {tmp}, {base}, #2",
            "vmov s2, {tmp}",
            "orr {tmp}, {base}, #3",
            "vmov s3, {tmp}",
            "orr {tmp}, {base}, #4",
            "vmov s4, {tmp}",
            "orr {tmp}, {base}, #5",
            "vmov s5, {tmp}",
            "orr {tmp}, {base}, #6",
            "vmov s6, {tmp}",
            "orr {tmp}, {base}, #7",
            "vmov s7, {tmp}",
            "orr {tmp}, {base}, #8",
            "vmov s8, {tmp}",
            "orr {tmp}, {base}, #9",
            "vmov s9, {tmp}",
            "orr {tmp}, {base}, #10",
            "vmov s10, {tmp}",
            "orr {tmp}, {base}, #11",
            "vmov s11, {tmp}",
            "orr {tmp}, {base}, #12",
            "vmov s12, {tmp}",
            "orr {tmp}, {base}, #13",
            "vmov s13, {tmp}",
            "orr {tmp}, {base}, #14",
            "vmov s14, {tmp}",
            "orr {tmp}, {base}, #15",
            "vmov s15, {tmp}",
            "orr {tmp}, {base}, #16",
            "vmov s16, {tmp}",
            "orr {tmp}, {base}, #17",
            "vmov s17, {tmp}",
            "orr {tmp}, {base}, #18",
            "vmov s18, {tmp}",
            "orr {tmp}, {base}, #19",
            "vmov s19, {tmp}",
            "orr {tmp}, {base}, #20",
            "vmov s20, {tmp}",
            "orr {tmp}, {base}, #21",
            "vmov s21, {tmp}",
            "orr {tmp}, {base}, #22",
            "vmov s22, {tmp}",
            "orr {tmp}, {base}, #23",
            "vmov s23, {tmp}",
            "orr {tmp}, {base}, #24",
            "vmov s24, {tmp}",
            "orr {tmp}, {base}, #25",
            "vmov s25, {tmp}",
            "orr {tmp}, {base}, #26",
            "vmov s26, {tmp}",
            "orr {tmp}, {base}, #27",
            "vmov s27, {tmp}",
            "orr {tmp}, {base}, #28",
            "vmov s28, {tmp}",
            "orr {tmp}, {base}, #29",
            "vmov s29, {tmp}",
            "orr {tmp}, {base}, #30",
            "vmov s30, {tmp}",
            "orr {tmp}, {base}, #31",
            "vmov s31, {tmp}",
            base = in(reg) base,
            tmp = out(reg) _,
        );
    }

    // Yield to let the other partition run (and potentially corrupt our FPU state)
    plib::sys_yield().ok();

    // Read back and verify all 32 FPU registers
    let mut regs = [0u32; 32];
    unsafe {
        core::arch::asm!(
            "vmov {0}, s0",  "str {0}, [{arr}]",
            "vmov {0}, s1",  "str {0}, [{arr}, #4]",
            "vmov {0}, s2",  "str {0}, [{arr}, #8]",
            "vmov {0}, s3",  "str {0}, [{arr}, #12]",
            "vmov {0}, s4",  "str {0}, [{arr}, #16]",
            "vmov {0}, s5",  "str {0}, [{arr}, #20]",
            "vmov {0}, s6",  "str {0}, [{arr}, #24]",
            "vmov {0}, s7",  "str {0}, [{arr}, #28]",
            "vmov {0}, s8",  "str {0}, [{arr}, #32]",
            "vmov {0}, s9",  "str {0}, [{arr}, #36]",
            "vmov {0}, s10", "str {0}, [{arr}, #40]",
            "vmov {0}, s11", "str {0}, [{arr}, #44]",
            "vmov {0}, s12", "str {0}, [{arr}, #48]",
            "vmov {0}, s13", "str {0}, [{arr}, #52]",
            "vmov {0}, s14", "str {0}, [{arr}, #56]",
            "vmov {0}, s15", "str {0}, [{arr}, #60]",
            "vmov {0}, s16", "str {0}, [{arr}, #64]",
            "vmov {0}, s17", "str {0}, [{arr}, #68]",
            "vmov {0}, s18", "str {0}, [{arr}, #72]",
            "vmov {0}, s19", "str {0}, [{arr}, #76]",
            "vmov {0}, s20", "str {0}, [{arr}, #80]",
            "vmov {0}, s21", "str {0}, [{arr}, #84]",
            "vmov {0}, s22", "str {0}, [{arr}, #88]",
            "vmov {0}, s23", "str {0}, [{arr}, #92]",
            "vmov {0}, s24", "str {0}, [{arr}, #96]",
            "vmov {0}, s25", "str {0}, [{arr}, #100]",
            "vmov {0}, s26", "str {0}, [{arr}, #104]",
            "vmov {0}, s27", "str {0}, [{arr}, #108]",
            "vmov {0}, s28", "str {0}, [{arr}, #112]",
            "vmov {0}, s29", "str {0}, [{arr}, #116]",
            "vmov {0}, s30", "str {0}, [{arr}, #120]",
            "vmov {0}, s31", "str {0}, [{arr}, #124]",
            out(reg) _,
            arr = in(reg) regs.as_mut_ptr(),
        );
    }

    let mut ok = true;
    for i in 0..32u32 {
        let expected = base | i;
        if regs[i as usize] != expected {
            ok = false;
            // Record first failure
            if FAIL_REG.load(Ordering::Relaxed) == 0xFFFF {
                FAIL_REG.store(i, Ordering::Release);
                FAIL_EXPECTED.store(expected, Ordering::Release);
                FAIL_GOT.store(regs[i as usize], Ordering::Release);
            }
            break;
        }
    }

    if ok {
        pass_counter.fetch_add(1, Ordering::Release);
    } else {
        fail_counter.fetch_add(1, Ordering::Release);
    }
}

extern "C" fn p0_body(_r0: u32) -> ! {
    loop {
        fpu_load_and_verify(0xAA00_0000, &P0_PASS, &P0_FAIL);
    }
}
kernel::partition_trampoline!(p0_main => p0_body);

extern "C" fn p1_body(_r0: u32) -> ! {
    loop {
        fpu_load_and_verify(0xBB00_0000, &P1_PASS, &P1_FAIL);
    }
}
kernel::partition_trampoline!(p1_main => p1_body);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    // Enable FPU before kernel init — must happen before init_stack_frame
    // which allocates the FPU context frame (50 words vs 16 without FPU).
    kernel::boot::init_fpu().expect("FPU init");

    let mut sched = ScheduleTable::<{ FpuTestCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::entry(p0_main),
        PartitionSpec::entry(p1_main),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    rprintln!("\n=== FPU Context Switch Test — nRF52833 (Cortex-M4F) ===");
    rprintln!("P0: pattern 0xAA0000nn | P1: pattern 0xBB0000nn");
    rprintln!("Verifying s0-s31 survive context switches...");
    rprintln!("[INIT] FPU enabled, lazy stacking verified. Booting...\n");
    match boot(p).expect("boot") {}
}
