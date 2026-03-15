// Not migrated to define_unified_harness! — this test requires reading MPU R4
// RBAR and RASR registers directly in SysTick to verify partition-specific
// values were programmed by PendSV. It uses distinct data region sizes per
// partition (4 KiB vs 8 KiB) to confirm correct RASR encoding. The standard
// harness encapsulates MPU state and doesn't expose raw register access for
// hardware-level verification.

//! QEMU test: dynamic MPU region verification across partition switches.
//!
//! Two partitions with distinct data-region bases and sizes. On each
//! context switch the SysTick handler reads back MPU R4 RBAR and RASR
//! registers via the memory-mapped MPU interface and verifies they match
//! the expected partition-specific values.  Runs for several major-frame
//! cycles, printing PASS/FAIL for every check.  Exits with
//! EXIT_SUCCESS when all checks pass.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    mpu::{self, build_rasr, encode_size, AP_FULL_ACCESS, RBAR_ADDR_MASK},
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

const NP: usize = TestConfig::N;
const STACK_WORDS: usize = TestConfig::STACK_WORDS;
/// Partition 0: 4 KiB data region at 0x2000_0000.
/// Partition 1: 8 KiB data region at 0x2000_8000.
const DATA_BASES: [u32; NP] = [0x2000_0000, 0x2000_8000];
const DATA_SIZES: [u32; NP] = [4096, 8192];
/// Run for 3 full major frames (each frame = 2 partitions × 2 ticks = 4 ticks).
/// That yields 6 partition switches total.
const TARGET_SWITCHES: u32 = 6;

/// Aligned wrapper so each partition stack satisfies the MPU region
/// alignment requirement (stack base must be 32-byte aligned for the
/// stack guard and naturally aligned for the data region).
#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NP] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NP]
};

// These #[no_mangle] statics are required by the PendSV assembly handler
// (define_pendsv_dynamic!) which references them by symbol name. They cannot
// be encapsulated behind a Mutex because the assembly performs raw loads/stores.
#[no_mangle]
static mut PARTITION_SP: [u32; NP] = [0; NP];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

// Use define_unified_kernel! with empty yield handler (this test doesn't use SVC yield).
kernel::define_unified_kernel!(TestConfig, |_k| {});

static STRATEGY: DynamicStrategy = DynamicStrategy::new();

kernel::define_pendsv_dynamic!(STRATEGY, TestConfig);

/// Compute the expected RASR value for partition `pid`.
///
/// This mirrors the kernel's `partition_mpu_regions` data-region formula:
/// AP_FULL_ACCESS, XN=true, S=true, C=true, B=false.
fn expected_rasr(pid: usize) -> u32 {
    let sf = encode_size(DATA_SIZES[pid]).expect("valid size");
    build_rasr(sf, AP_FULL_ACCESS, true, (true, true, false))
}

extern "C" fn partition_main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
fn SysTick() {
    static mut SW: u32 = 0;
    static mut LAST_PID: Option<u8> = None;
    static mut FAIL: bool = false;
    // SAFETY: sole MPU accessor at this priority; steal() yields a
    // reference to the memory-mapped MPU registers.
    let p = unsafe { cortex_m::Peripherals::steal() };

    // After a switch, PendSV has executed and programmed R4. Verify.
    if let Some(active_pid) = *LAST_PID {
        // SAFETY: RNR write selects region; RBAR/RASR reads have no
        // side effects beyond returning the register value.
        let (got_rbar, got_rasr) = unsafe {
            p.MPU.rnr.write(4);
            (p.MPU.rbar.read(), p.MPU.rasr.read())
        };

        let got_base = got_rbar & RBAR_ADDR_MASK;
        let exp_base = DATA_BASES[active_pid as usize];
        let exp_rasr = expected_rasr(active_pid as usize);

        let base_ok = got_base == exp_base;
        let rasr_ok = got_rasr == exp_rasr;
        let pass = base_ok && rasr_ok;

        if !pass {
            *FAIL = true;
        }

        hprintln!(
            "sw{}: p{} RBAR={:#010x} exp={:#010x} {} | RASR={:#010x} exp={:#010x} {} => {}",
            *SW,
            active_pid,
            got_base,
            exp_base,
            if base_ok { "OK" } else { "FAIL" },
            got_rasr,
            exp_rasr,
            if rasr_ok { "OK" } else { "FAIL" },
            if pass { "PASS" } else { "FAIL" },
        );

        if *SW >= TARGET_SWITCHES {
            // Final descriptor check via DynamicStrategy software state.
            let desc = STRATEGY.slot(4).expect("R4 occupied");
            let desc_ok = desc.base == exp_base && desc.owner == active_pid;
            hprintln!(
                "desc: base={:#010x} owner={} => {}",
                desc.base,
                desc.owner,
                if desc_ok { "PASS" } else { "FAIL" },
            );
            if !desc_ok {
                *FAIL = true;
            }

            if *FAIL {
                hprintln!("mpu_dynamic_test: FAIL");
                debug::exit(debug::EXIT_FAILURE);
            } else {
                hprintln!("mpu_dynamic_test: all checks passed — PASS");
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
    }

    with_kernel_mut(|k| {
        // TODO: svc_scheduler→svc::scheduler rename is an out-of-scope refactor;
        // should be split into a separate PR if not required for this migration.
        let event = kernel::svc::scheduler::advance_schedule_tick(k);
        if let ScheduleEvent::PartitionSwitch(pid) = event {
            if let Some(pcb) = k.partitions().get(pid as usize) {
                let dyn_region = pcb.cached_dynamic_region();
                STRATEGY
                    .configure_partition(pid, &[dyn_region], 0)
                    .expect("configure_partition");
            }
            // SAFETY: single-core exclusive write; PendSV reads this after
            // SysTick returns but cannot preempt us (lower priority).
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };

            *SW += 1;
            *LAST_PID = Some(pid);
        }
    });
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "mpu_dynamic_test: start ({} switches over 3 major frames)",
        TARGET_SWITCHES
    );

    // Build schedule table: P0(2) → system window(1) → P1(2) → system window(1)
    // TODO: system windows are unconditionally added because this example is
    // inherently dynamic-mpu-only (uses DynamicStrategy, define_pendsv_dynamic!).
    // Consider adding a top-level #[cfg(feature = "dynamic-mpu")] to the whole file.
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    sched.add(ScheduleEntry::new(1, 2)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    // Build partition memories and create kernel
    // stack_base/stack_size point at the actual partition stacks (STACKS
    // array), not the data regions. mpu_region carries the data-region
    // descriptor that DynamicStrategy programs into MPU R4.
    #[allow(clippy::deref_addrof)]
    let k = {
        // SAFETY: before interrupts; single-core exclusive.
        let stacks: &mut [AlignedStack; NP] = unsafe { &mut *(&raw mut STACKS) };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(
                &mut s0.0,
                0,
                MpuRegion::new(DATA_BASES[0], DATA_SIZES[0], 0),
                0,
            )
            .expect("ext mem"),
            ExternalPartitionMemory::new(
                &mut s1.0,
                0,
                MpuRegion::new(DATA_BASES[1], DATA_SIZES[1], 0),
                1,
            )
            .expect("ext mem"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel creation")
    };
    store_kernel(k);

    // Seal the MPU cache so cached_dynamic_region() returns valid data.
    with_kernel_mut(|k| {
        for pid in 0..NP {
            let pcb = k.partitions_mut().get_mut(pid).expect("partition");
            mpu::precompute_mpu_cache(pcb).expect("precompute_mpu_cache");
        }
    });

    // Initialize partition stacks
    #[allow(clippy::deref_addrof)]
    // SAFETY: before interrupts; single-core exclusive.
    unsafe {
        for i in 0..NP {
            let stk = &mut STACKS[i].0;
            let ix = kernel::context::init_stack_frame(
                stk,
                partition_main as *const () as u32,
                Some(i as u32),
            )
            .expect("init_stack_frame");
            PARTITION_SP[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
        }
        // Set exception priorities per the three-tier model:
        //   Tier 1 (highest): SVCall  – synchronous syscall entry
        //   Tier 2 (middle):  SysTick – time-slice preemption
        //   Tier 3 (lowest):  PendSV  – deferred context switch
        //
        // SAFETY: set_priority is unsafe because changing priority levels can
        // break priority-based critical sections. Interrupts are not yet enabled
        // so no preemption or race conditions can occur during configuration.
        cp.SCB
            .set_priority(SystemHandler::SVCall, TestConfig::SVCALL_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::PendSV, TestConfig::PENDSV_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::SysTick, TestConfig::SYSTICK_PRIORITY);
    }

    // SAFETY: Interrupts not yet enabled. PRIVDEFENA ensures privileged
    // code retains a default memory map.
    unsafe { cp.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}
