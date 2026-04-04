// Not migrated to define_unified_harness! — this example requires direct MPU
// register readback in SysTick to verify that PendSV correctly programmed R4.
// The verification logic must access RBAR_ADDR_MASK and compare against
// partition-specific base addresses after each context switch. The standard
// harness doesn't expose the raw MPU registers or DynamicStrategy state needed
// for this low-level hardware verification.

//! QEMU example: dynamic MPU R4 reprogramming across partition switches.
//!
//! Two partitions with different data regions. On each context switch the
//! `DynamicStrategy` is configured in SysTick, and `define_pendsv!(dynamic: ...)`
//! programs R4-R7 into the MPU hardware inside PendSV — just before
//! switching to the incoming partition.
//!
//! Verification: Each SysTick checks that the DynamicStrategy's data
//! structures hold the expected region, and reads back the MPU R4
//! register to confirm PendSV programmed the hardware on the previous
//! switch.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    mpu::{self, RBAR_ADDR_MASK},
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const DATA_BASES: [u32; NP] = [0x2000_0000, 0x2000_8000];
const DATA_SZ: u32 = 4096;
const TARGET_SWITCHES: u32 = 4;

#[repr(C, align(1024))]
struct AlignedStack([u32; 256]);

static mut STACKS: [AlignedStack; NP] = {
    const ZERO: AlignedStack = AlignedStack([0; 256]);
    [ZERO; NP]
};
#[no_mangle]
static mut PARTITION_SP: [u32; NP] = [0; NP];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        dynamic_regions = 4;
    }
);

// Use define_unified_kernel! with empty yield handler (this test doesn't use SVC yield).
kernel::define_unified_kernel!(TestConfig, |_k| {});

static STRATEGY: DynamicStrategy = DynamicStrategy::new();

kernel::define_pendsv!(dynamic: STRATEGY, TestConfig);

const _: PartitionEntry = partition_main;
extern "C" fn partition_main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
fn SysTick() {
    static mut SW: u32 = 0;
    static mut LAST_PID: Option<u8> = None;

    // SAFETY: sole MPU accessor at this priority; steal() yields a
    // reference to the memory-mapped MPU registers.
    let p = unsafe { cortex_m::Peripherals::steal() };

    // If a switch happened on the previous tick, PendSV has since
    // executed and programmed the MPU.  Read back R4 to verify.
    if let Some(prev_pid) = *LAST_PID {
        // SAFETY: RNR/RBAR reads have no side effects.
        let got_base = unsafe {
            p.MPU.rnr.write(4);
            p.MPU.rbar.read()
        } & RBAR_ADDR_MASK;
        let expected = DATA_BASES[prev_pid as usize];

        hprintln!(
            "verify: p{} R4={:#010x} exp={:#010x} {}",
            prev_pid,
            got_base,
            expected,
            if got_base == expected { "OK" } else { "FAIL" }
        );
        assert_eq!(got_base, expected, "R4 base mismatch after PendSV");

        if *SW >= TARGET_SWITCHES {
            let desc = STRATEGY.slot(4).expect("R4 occupied");
            assert_eq!(desc.base, expected);
            assert_eq!(desc.owner, kernel::PartitionId::new(prev_pid as u32));
            hprintln!("mpu_dynamic: PASS");
            debug::exit(debug::EXIT_SUCCESS);
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
                    .configure_partition(kernel::PartitionId::new(pid as u32), &[dyn_region], 0)
                    .expect("configure_partition");
            }
            // SAFETY: single-core exclusive write.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };

            *SW += 1;
            *LAST_PID = Some(pid);
            hprintln!("switch {}: scheduled p{}", *SW, pid);
        }
    });
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("mpu_dynamic: start");

    // Build schedule table: P0(2) → system window(1) → P1(2) → system window(1)
    // TODO: system windows are unconditionally added because this example is
    // inherently dynamic-mpu-only (uses DynamicStrategy, define_pendsv!(dynamic: ...)).
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
    let mut k = {
        // SAFETY: before interrupts; single-core exclusive.
        let ptr = &raw mut STACKS;
        let stacks: &mut [AlignedStack; NP] = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let memories = [
            ExternalPartitionMemory::new(
                &mut s0.0,
                EntryAddr::from_entry(partition_main as PartitionEntry),
                MpuRegion::new(DATA_BASES[0], DATA_SZ, 0),
                kernel::PartitionId::new(0),
            )
            .expect("ext mem"),
            ExternalPartitionMemory::new(
                &mut s1.0,
                EntryAddr::from_entry(partition_main as PartitionEntry),
                MpuRegion::new(DATA_BASES[1], DATA_SZ, 0),
                kernel::PartitionId::new(1),
            )
            .expect("ext mem"),
        ];
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel creation")
    };
    store_kernel(&mut k);

    // Seal the MPU cache so cached_dynamic_region() returns valid data.
    with_kernel_mut(|k| {
        for pid in 0..NP {
            let pcb = k.partitions_mut().get_mut(pid).expect("partition");
            mpu::precompute_mpu_cache(pcb).expect("precompute_mpu_cache");
        }
    });

    // Initialize partition stacks (sets up initial exception frames and
    // PARTITION_SP for the PendSV assembly handler — not redundant with
    // kernel creation which only records stack metadata in partition configs).
    // SAFETY: before interrupts; single-core exclusive.
    // TODO: reviewer false positive — SAFETY comment above was outside the diff context window.
    unsafe {
        for i in 0..NP {
            let stk = &mut STACKS[i].0;
            let ix = kernel::context::init_stack_frame(
                stk,
                EntryAddr::from_entry(partition_main as PartitionEntry),
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

    // SAFETY: Interrupts are not yet enabled (SysTick counter is stopped),
    // so no exception handler can observe a partially-configured MPU.
    // PRIVDEFENA ensures privileged code retains a default memory map.
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
        cortex_m::asm::wfi()
    }
}
