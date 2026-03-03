// Not migrated to define_unified_harness! — this example requires direct MPU
// register readback in SysTick to verify that PendSV correctly programmed R4.
// The verification logic must access RBAR_ADDR_MASK and compare against
// partition-specific base addresses after each context switch. The standard
// harness doesn't expose the raw MPU registers or DynamicStrategy state needed
// for this low-level hardware verification.

//! QEMU example: dynamic MPU R4 reprogramming across partition switches.
//!
//! Two partitions with different data regions. On each context switch the
//! `DynamicStrategy` is configured in SysTick, and `define_pendsv_dynamic!`
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
    mpu::{self, partition_dynamic_regions, RBAR_ADDR_MASK},
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
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

kernel::compose_kernel_config!(
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

kernel::define_pendsv_dynamic!(STRATEGY);

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
            assert_eq!(desc.owner, prev_pid);
            hprintln!("mpu_dynamic: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        }
    }

    with_kernel_mut(|k| {
        let event = k.advance_schedule_tick();
        if let ScheduleEvent::PartitionSwitch(pid) = event {
            if let Some(pcb) = k.partitions().get(pid as usize) {
                if let Some(regions) = partition_dynamic_regions(pcb) {
                    let _ = STRATEGY.configure_partition(pid, &regions, 0);
                }
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
    hprintln!("mpu_dynamic: start");

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

    // Build partition configs
    // SAFETY: before interrupts; single-core exclusive.
    let cfgs: [PartitionConfig; NP] = unsafe {
        core::array::from_fn(|i| PartitionConfig {
            id: i as u8,
            entry_point: 0,
            stack_base: STACKS[i].0.as_ptr() as u32,
            stack_size: DATA_SZ,
            mpu_region: MpuRegion::new(DATA_BASES[i], DATA_SZ, 0),
            peripheral_regions: heapless::Vec::new(),
        })
    };

    // Create unified kernel
    let k = Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    store_kernel(k);

    // Initialize partition stacks
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
