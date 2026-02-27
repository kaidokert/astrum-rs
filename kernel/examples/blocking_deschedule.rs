//! QEMU test: verify blocking recv triggers immediate deschedule.
//!
//! A single partition calls SYS_QUEUING_RECV_TIMED on an empty queue. The
//! SysTick handler verifies the partition transitions to Waiting state and
//! is descheduled within a few ticks (not the full slot duration).
//!
// TODO: This test uses a single partition for simplicity, though a two-partition
// scenario (sender/receiver) would better demonstrate the full IPC blocking path.
// The single-partition approach still validates the core deschedule mechanism.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    partition::{MpuRegion, PartitionConfig, PartitionState},
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::{try_transition, Kernel},
    syscall::{SYS_GET_TIME, SYS_QUEUING_RECV_TIMED, SYS_YIELD},
};
use panic_semihosting as _;

const NP: usize = 1;

kernel::kernel_config! { Cfg {
    partitions = 1;
    queues = 2;
    queue_depth = 4;
    max_msg_size = 4;
    queue_waitq = 4;
    sampling_msg_size = 1;
    blackboard_msg_size = 1;
}}

// Use define_unified_kernel! which generates KERNEL static, dispatch_hook, and store_kernel
kernel::define_unified_kernel!(Cfg, |k| {
    // Yield handler: advance schedule when yield_requested is set
    use kernel::svc::YieldResult;
    let result = k.yield_current_slot();
    if let Some(pid) = result.partition_id() {
        k.set_next_partition(pid);
        cortex_m::peripheral::SCB::set_pendsv();
    }
});

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;
kernel::define_pendsv!();

#[repr(C, align(1024))]
struct Stack([u32; Cfg::STACK_WORDS]);
static mut STACKS: [Stack; NP] = {
    const Z: Stack = Stack([0; Cfg::STACK_WORDS]);
    [Z; NP]
};

/// Tick when partition called blocking recv
static BLOCK_TICK: AtomicU32 = AtomicU32::new(0);
/// Set to 1 when partition has called blocking recv
static BLOCKED: AtomicU32 = AtomicU32::new(0);

/// Partition: waits a bit, then calls blocking recv on empty queue.
extern "C" fn partition_main_body(r0: u32) -> ! {
    let port = r0;
    // Wait for a few ticks
    while kernel::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32) < 5 {
        kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
    // Record tick and signal we're about to block
    BLOCK_TICK.store(
        kernel::svc!(SYS_GET_TIME, 0u32, 0u32, 0u32),
        Ordering::Release,
    );
    BLOCKED.store(1, Ordering::Release);
    // Block on empty queue
    let mut buf = [0u8; 4];
    kernel::svc!(
        SYS_QUEUING_RECV_TIMED,
        port,
        1000u32,
        buf.as_mut_ptr() as u32
    );
    // Should not return quickly - keep yielding
    loop {
        kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
kernel::partition_trampoline!(partition_main => partition_main_body);

#[exception]
fn SysTick() {
    // Use function-local atomic to track tick count within the handler.
    // This avoids `static mut` access for the tick counter itself.
    static TICK: AtomicU32 = AtomicU32::new(0);
    let tick = TICK.fetch_add(1, Ordering::Relaxed) + 1;

    // TODO: with_kernel_mut refactoring is a DRY cleanup, technically out of
    // scope for the schedule audit. Consider splitting into a separate commit.
    with_kernel_mut(|k| {
        let event = k.advance_schedule_tick();
        if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
            // Transition incoming partition to Running so syscalls can block it
            let _ = try_transition(k.partitions_mut(), pid, PartitionState::Running);
            k.set_next_partition(pid);
            cortex_m::peripheral::SCB::set_pendsv();
        }

        // Sync tick and expire timed waits
        let current_tick = k.tick().get();
        k.sync_tick(current_tick);
        k.expire_timed_waits::<{ Cfg::N }>(current_tick);
    });

    // Check if partition has blocked and verify state transition
    if BLOCKED.load(Ordering::Acquire) == 1 {
        let block_tick = BLOCK_TICK.load(Ordering::Acquire);
        let delta = tick.saturating_sub(block_tick);
        // Check partition state
        with_kernel(|k| {
            if let Some(p) = k.partitions().get(0) {
                let state = p.state();
                // Partition should transition to Waiting within a few ticks
                if state == PartitionState::Waiting && delta <= 4 {
                    hprintln!("blocking_deschedule: state=Waiting, delta={}", delta);
                    hprintln!("blocking_deschedule: PASS");
                    debug::exit(debug::EXIT_SUCCESS);
                } else if delta > 10 {
                    hprintln!(
                        "blocking_deschedule: FAIL - state={:?}, delta={}",
                        state,
                        delta
                    );
                    debug::exit(debug::EXIT_FAILURE);
                }
            }
        });
    }
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    hprintln!("blocking_deschedule: start");

    // Build schedule
    let mut sched = ScheduleTable::<4>::new();
    sched.add(ScheduleEntry::new(0, 20)).unwrap();
    #[cfg(feature = "dynamic-mpu")]
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    // Build partition config
    // SAFETY: single-core Cortex-M, interrupts not yet enabled — exclusive
    // access to all static-mut variables (STACKS, PARTITION_SP). Exception
    // priorities are configured before SysTick is enabled.
    let cfgs: [PartitionConfig; NP] = unsafe {
        [{
            let b = STACKS[0].0.as_ptr() as u32;
            PartitionConfig {
                id: 0,
                entry_point: 0,
                stack_base: b,
                stack_size: (Cfg::STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (Cfg::STACK_WORDS * 4) as u32, 0),
                peripheral_regions: heapless::Vec::new(),
            }
        }]
    };

    // Create unified kernel with schedule and partitions
    #[cfg(feature = "dynamic-mpu")]
    let mut k = Kernel::<Cfg>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
        .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<Cfg>::new(sched, &cfgs).expect("kernel creation");

    // Create queuing port for the test
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .unwrap();

    // Transition partition 0 to Running before the first context switch
    // so that blocking syscalls can transition it to Waiting.
    let _ = try_transition(k.partitions_mut(), 0, PartitionState::Running);

    // Start schedule and set next partition before storing kernel
    let first_pid = k.start_schedule().expect("schedule start failed");
    k.set_next_partition(first_pid);
    hprintln!("blocking_deschedule: first_pid={}", first_pid);

    store_kernel(k);
    hprintln!("blocking_deschedule: kernel stored");

    // Initialize partition stack
    // SAFETY: single-core, interrupts disabled — exclusive access
    unsafe {
        let stk = &mut STACKS[0].0;
        let ix = kernel::context::init_stack_frame(
            stk,
            partition_main as *const () as u32,
            Some(dst as u32),
        )
        .unwrap();
        let sp = stk.as_ptr() as u32 + (ix as u32) * 4;
        set_partition_sp(0, sp);

        cp.SCB.set_priority(SystemHandler::SVCall, 0x00);
        cp.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        cp.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }

    hprintln!("blocking_deschedule: stack initialized");

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    hprintln!("blocking_deschedule: SysTick enabled, triggering PendSV");
    cortex_m::peripheral::SCB::set_pendsv();

    loop {
        cortex_m::asm::wfi();
    }
}
