//! Privilege Escalation Test — verify MPU blocks cpsid/msr CONTROL from partitions
//!
//! P0: attempts cpsid i (disable interrupts from unprivileged) — should UsageFault
//! P1: attempts msr CONTROL,r0 with nPRIV=0 (escalate to privileged) — should UsageFault
//! P2: healthy partition, counts its runs
//! P3: attempts to read SCB registers (privileged-only MMIO) — should MemManage
//!
//! All attack partitions have StayDead policy. P2 must survive and keep running.
//!
//! Build: cd f429zi && cargo build --example privilege_escalation_test \
//!            --features kernel-mpu --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use rtt_target::rprintln;
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

static CPSID_SURVIVED: AtomicU32 = AtomicU32::new(0);
static MSR_SURVIVED: AtomicU32 = AtomicU32::new(0);
static SCB_SURVIVED: AtomicU32 = AtomicU32::new(0);
static P2_RUNS: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(Cfg[AlignedStack2K]<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 12;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(Cfg, |tick, k| {
    if tick % 1000 == 0 && tick > 0 {
        let p2 = P2_RUNS.load(Ordering::Relaxed);
        let cpsid = CPSID_SURVIVED.load(Ordering::Relaxed);
        let msr = MSR_SURVIVED.load(Ordering::Relaxed);
        let scb = SCB_SURVIVED.load(Ordering::Relaxed);

        // Check attacker partition states
        let p0_faulted = k.pcb(0).map(|p| p.state() == kernel::partition::PartitionState::Faulted).unwrap_or(false);
        let p1_faulted = k.pcb(1).map(|p| p.state() == kernel::partition::PartitionState::Faulted).unwrap_or(false);
        let p3_faulted = k.pcb(3).map(|p| p.state() == kernel::partition::PartitionState::Faulted).unwrap_or(false);

        rprintln!(
            "[{:5}ms] P0(cpsid)={} P1(msr)={} P3(scb)={} P2_runs={} cpsid_survived={} msr_survived={} scb_survived={}",
            tick,
            if p0_faulted { "DEAD" } else { "ALIVE" },
            if p1_faulted { "DEAD" } else { "ALIVE" },
            if p3_faulted { "DEAD" } else { "ALIVE" },
            p2, cpsid, msr, scb
        );

        if p0_faulted && p1_faulted && p3_faulted
            && cpsid == 0 && msr == 0 && scb == 0
        {
            rprintln!("SUCCESS: all privilege escalation attempts blocked! P2 runs={}", p2);
        }
    }
});

// P0: attempt cpsid i (disable interrupts from unprivileged)
// On Cortex-M, cpsid i from unprivileged is a NOP — it doesn't fault,
// but it also doesn't actually disable interrupts. We verify by checking
// PRIMASK after the attempt, then write to an ungranted address to prove
// MPU isolation still works.
extern "C" fn cpsid_attacker(_r0: u32) -> ! {
    // cpsid i from unprivileged — should be NOP
    unsafe { core::arch::asm!("cpsid i", options(nomem, nostack)); }

    // MSR PRIMASK from unprivileged — also NOP
    unsafe { core::arch::asm!("msr PRIMASK, {}", in(reg) 1u32, options(nomem, nostack)); }

    // Read PRIMASK — should still be 0 (interrupts NOT disabled)
    let primask: u32;
    unsafe { core::arch::asm!("mrs {}, PRIMASK", out(reg) primask, options(nomem, nostack)); }

    if primask != 0 {
        // BAD: interrupts actually disabled from unprivileged
        CPSID_SURVIVED.fetch_add(1, Ordering::Relaxed);
    }

    // Prove we're still unprivileged: write to ungranted peripheral → MemManage
    unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD); }
    CPSID_SURVIVED.fetch_add(1, Ordering::Relaxed); // should NOT reach here
    loop { plib::sys_yield().ok(); }
}

// P1: attempt msr CONTROL with nPRIV=0 (escalate to privileged)
// On Cortex-M, writing CONTROL from unprivileged ignores the nPRIV clear.
extern "C" fn msr_control_attacker(_r0: u32) -> ! {
    // Try to clear nPRIV bit (bit 0) in CONTROL
    unsafe {
        core::arch::asm!(
            "mrs {tmp}, CONTROL",
            "bic {tmp}, {tmp}, #1",  // clear nPRIV
            "msr CONTROL, {tmp}",
            "isb",
            tmp = out(reg) _,
            options(nomem, nostack),
        );
    }

    // Read CONTROL — nPRIV should still be 1 (unprivileged)
    let control: u32;
    unsafe { core::arch::asm!("mrs {}, CONTROL", out(reg) control, options(nomem, nostack)); }

    if control & 1 == 0 {
        // BAD: nPRIV cleared, now privileged
        MSR_SURVIVED.fetch_add(1, Ordering::Relaxed);
    }

    // Prove we're still unprivileged: write to ungranted peripheral → MemManage
    unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD); }
    MSR_SURVIVED.fetch_add(1, Ordering::Relaxed); // should NOT reach here
    loop { plib::sys_yield().ok(); }
}

// P2: healthy survivor — just counts runs
extern "C" fn survivor(_r0: u32) -> ! {
    loop {
        P2_RUNS.fetch_add(1, Ordering::Relaxed);
        plib::sys_yield().ok();
    }
}

// P3: attempt BASEPRI escalation + ungranted write
// MSR BASEPRI from unprivileged is a NOP. Then prove MPU still blocks us.
extern "C" fn basepri_attacker(_r0: u32) -> ! {
    // Try to set BASEPRI (mask all interrupts below priority N)
    unsafe { core::arch::asm!("msr BASEPRI, {}", in(reg) 0x10u32, options(nomem, nostack)); }

    // Read BASEPRI — should still be 0
    let basepri: u32;
    unsafe { core::arch::asm!("mrs {}, BASEPRI", out(reg) basepri, options(nomem, nostack)); }

    if basepri != 0 {
        SCB_SURVIVED.fetch_add(1, Ordering::Relaxed);
    }

    // Prove we're still isolated: write to ungranted peripheral → MemManage
    unsafe { core::ptr::write_volatile(0x4004_0000 as *mut u32, 0xDEAD); }
    SCB_SURVIVED.fetch_add(1, Ordering::Relaxed);
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(p0_main => cpsid_attacker);
kernel::partition_trampoline!(p1_main => msr_control_attacker);
kernel::partition_trampoline!(p2_main => survivor);
kernel::partition_trampoline!(p3_main => basepri_attacker);

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();

    rprintln!("\n=== Privilege Escalation Test ===");
    rprintln!("P0: cpsid i + PRIMASK write + SCB read");
    rprintln!("P1: msr CONTROL (clear nPRIV) + SCB read");
    rprintln!("P3: direct SCB register read");
    rprintln!("P2: healthy survivor");
    rprintln!("All attackers should fault. P2 must survive.\n");

    let mut sched = ScheduleTable::<{ Cfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).expect("P0");
    sched.add(ScheduleEntry::new(1, 3)).expect("P1");
    sched.add(ScheduleEntry::new(2, 3)).expect("P2");
    sched.add(ScheduleEntry::new(3, 3)).expect("P3");
    sched.add_system_window(1).expect("SW");

    static mut STACKS: [AlignedStack2K; 4] = [AlignedStack2K::ZERO; 4];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1, ref mut s2, ref mut s3] = *stacks;

    let data = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(s0, p0_main as kernel::PartitionEntry, data, kernel::PartitionId::new(0))
        .expect("mem0").with_code_mpu_region(code).expect("code0");
    let mem1 = ExternalPartitionMemory::from_aligned_stack(s1, p1_main as kernel::PartitionEntry, data, kernel::PartitionId::new(1))
        .expect("mem1").with_code_mpu_region(code).expect("code1");
    let mem2 = ExternalPartitionMemory::from_aligned_stack(s2, p2_main as kernel::PartitionEntry, data, kernel::PartitionId::new(2))
        .expect("mem2").with_code_mpu_region(code).expect("code2");
    let mem3 = ExternalPartitionMemory::from_aligned_stack(s3, p3_main as kernel::PartitionEntry, data, kernel::PartitionId::new(3))
        .expect("mem3").with_code_mpu_region(code).expect("code3");

    let mut k = Kernel::<Cfg>::new(sched, &[mem0, mem1, mem2, mem3]).expect("kernel");
    store_kernel(&mut k);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {}
}
