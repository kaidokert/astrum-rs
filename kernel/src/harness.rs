//! Shared runtime harness for QEMU demo examples.
//!
//! The [`define_unified_harness!`] macro emits the boilerplate statics, SVC
//! linkage, PendSV context-switch handler, SysTick exception handler,
//! and a safe `boot()` function that every multi-partition example
//! would otherwise duplicate.
//!
//! # Usage
//!
//! ```ignore
//! kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS);
//! ```
//!
//! This generates:
//!
//! | Item | Description |
//! |------|-------------|
//! | `static mut STACKS` | Per-partition stack arrays (`$SW` words each) |
//! | `static mut PARTITION_SP` | Saved stack pointers for PendSV |
//! | `static mut CURRENT_PARTITION` | Index of currently running partition |
//! | `static mut NEXT_PARTITION` | Index of next partition to switch to |
//! | `KERNEL` | `Mutex<RefCell<Option<Kernel<…>>>>` for SVC dispatch and SysTick |
//! | `static _SVC` | Forces linker to include the SVC assembly trampoline |
//! | `SysTick` exception handler | Drives the round-robin scheduler |
//! | `PendSV` handler | Via [`define_pendsv!`] |
//! | `boot()` | Safe function: inits stacks, configures exceptions, starts OS |
//!
//! After invoking the macro, call the generated `boot()` from `main()`
//! to perform the common startup sequence (partition stack init,
//! exception priorities, SysTick configuration, and first PendSV
//! trigger).

#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick {
    ($kernel:expr, $next:ident) => {{
        let event = $kernel.advance_schedule_tick();
        if let Some(pid) = event {
            // SAFETY: single-core Cortex-M — SysTick has exclusive access to
            // NEXT_PARTITION; PendSV (lower priority) cannot preempt us.
            unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
            cortex_m::peripheral::SCB::set_pendsv();
        }
    }};
}

#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick {
    ($kernel:expr, $next:ident, $tick:expr, $strategy:expr) => {{
        let event = $kernel.advance_schedule_tick();
        $crate::_unified_handle_tick_event!($kernel, event, $next, $tick, $strategy);
    }};
}

/// Shared helper: run bottom-half processing for system window and wake any
/// blocked device readers. Used by both `_unified_handle_tick_event!` and
/// `_unified_handle_yield!` to avoid code duplication.
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_run_system_window {
    ($kernel:expr, $tick:expr, $strategy:expr) => {{
        let bh = $crate::tick::run_bottom_half(
            &mut $kernel.uart_pair,
            &mut $kernel.isr_ring,
            &mut $kernel.buffers,
            &mut $kernel.hw_uart,
            $tick,
            $strategy,
        );
        if bh.has_rx_data {
            if let Some(woken) = $kernel.dev_wait_queue.wake_one_reader() {
                $crate::svc::try_transition(
                    $kernel.partitions_mut(),
                    woken,
                    $crate::partition::PartitionState::Ready,
                );
            }
        }
    }};
}

#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick_event {
    ($kernel:expr, $event:expr, $next:ident, $tick:expr, $strategy:expr) => {{
        match $event {
            $crate::scheduler::ScheduleEvent::PartitionSwitch(pid) => {
                // SAFETY: single-core Cortex-M — SysTick has exclusive access to
                // NEXT_PARTITION; PendSV (lower priority) cannot preempt us.
                unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
                cortex_m::peripheral::SCB::set_pendsv();
            }
            $crate::scheduler::ScheduleEvent::SystemWindow => {
                $crate::_unified_run_system_window!($kernel, $tick, $strategy);
            }
            $crate::scheduler::ScheduleEvent::None => {}
        }
    }};
}

#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_yield {
    ($kernel:expr, $next:ident) => {{
        use $crate::svc::YieldResult;
        let result = $kernel.yield_current_slot();
        if let Some(pid) = result.partition_id() {
            // SAFETY: single-core Cortex-M — SVC (priority 0x00) has
            // exclusive access; PendSV (priority 0xFF) cannot preempt.
            unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
        }
    }};
}

#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_yield {
    ($kernel:expr, $next:ident, $tick:expr, $strategy:expr) => {{
        use $crate::svc::YieldResult;
        loop {
            let result = $kernel.yield_current_slot();
            if let Some(pid) = result.partition_id() {
                // SAFETY: single-core Cortex-M — SVC (priority 0x00) has
                // exclusive access; PendSV (priority 0xFF) cannot preempt.
                unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
                break;
            }
            if result.is_system_window() {
                $crate::_unified_run_system_window!($kernel, $tick, $strategy);
                continue;
            }
            break;
        }
    }};
}

/// Unified harness: single `KERNEL` global for SVC dispatch and SysTick scheduling.
/// Generates STACKS, PARTITION_SP, CURRENT/NEXT_PARTITION, KERNEL, PendSV, SysTick, boot().
///
/// # MPU Alignment Constraint
///
/// The stack alignment is hardcoded to 1024 bytes, which requires `$SW == 256`
/// (256 words × 4 bytes = 1024 bytes). This is enforced by a compile-time
/// assertion. Supporting variable stack sizes would require either:
/// - A procedural macro that can compute alignment from the size parameter
/// - Multiple macro variants for different power-of-two sizes
/// - A build.rs script to generate the appropriate alignment
#[macro_export]
macro_rules! define_unified_harness {
    ($Config:ty, $NP:expr, $SW:expr) => {
        // Compile-time check: MPU requires stack alignment == stack size.
        // Since #[repr(align(...))] requires a literal, we hardcode 1024-byte
        // alignment, which mandates $SW == 256 words (256 * 4 = 1024 bytes).
        const _: () = assert!(
            $SW == 256,
            "define_unified_harness! requires $SW == 256 (1024-byte stacks) for correct MPU alignment"
        );

        /// Wrapper that aligns each partition stack to 1024 bytes so
        /// that the address is valid as an MPU region base.
        /// Note: alignment is hardcoded; see MPU Alignment Constraint above.
        #[repr(C, align(1024))]
        struct AlignedStack([u32; $SW]);

        static mut STACKS: [AlignedStack; $NP] = {
            const ZERO: AlignedStack = AlignedStack([0; $SW]);
            [ZERO; $NP]
        };

        #[no_mangle]
        static mut PARTITION_SP: [u32; $NP] = [0; $NP];

        #[no_mangle]
        static mut CURRENT_PARTITION: u32 = u32::MAX;

        #[no_mangle]
        static mut NEXT_PARTITION: u32 = 0;

        #[cfg(feature = "dynamic-mpu")]
        static HARNESS_STRATEGY: $crate::mpu_strategy::DynamicStrategy =
            $crate::mpu_strategy::DynamicStrategy::new();

        $crate::define_unified_kernel!($Config, |k| {
            #[cfg(not(feature = "dynamic-mpu"))]
            $crate::_unified_handle_yield!(k, NEXT_PARTITION);
            #[cfg(feature = "dynamic-mpu")]
            {
                let tick_val = k.tick().get();
                $crate::_unified_handle_yield!(k, NEXT_PARTITION, tick_val, &HARNESS_STRATEGY);
            }
        });

        #[used]
        static _SVC: unsafe extern "C" fn(&mut $crate::context::ExceptionFrame) =
            $crate::svc::SVC_HANDLER;

        $crate::define_pendsv!();

        #[exception]
        fn SysTick() {
            #[cfg(feature = "qemu")]
            {
                static mut TICK_COUNT: u32 = 0;
                // SAFETY: SysTick is not reentrant; exclusive access to TICK_COUNT.
                let tc = unsafe { &mut *(&raw mut TICK_COUNT) };
                *tc += 1;
                ::cortex_m_semihosting::hprintln!("[SysTick] #{}", *tc);
            }
            ::cortex_m::interrupt::free(|cs| {
                let mut guard = KERNEL.borrow(cs).borrow_mut();
                let k = match guard.as_mut() {
                    Some(k) => k,
                    None => return,
                };
                #[cfg(not(feature = "dynamic-mpu"))]
                $crate::_unified_handle_tick!(k, NEXT_PARTITION);
                #[cfg(feature = "dynamic-mpu")]
                {
                    let tick_val = k.tick().get();
                    $crate::_unified_handle_tick!(k, NEXT_PARTITION, tick_val, &HARNESS_STRATEGY);
                }
                let current_tick = k.tick().get();
                k.expire_timed_waits::<{ <$Config as $crate::config::KernelConfig>::N }>(
                    current_tick,
                );
            });
        }

        /// Initialize stacks, priorities, start schedule, enable SysTick, enter idle loop.
        fn boot(
            partitions: &[(extern "C" fn() -> !, u32)],
            peripherals: &mut cortex_m::Peripherals,
        ) -> ! {
            use cortex_m::peripheral::scb::SystemHandler;
            use cortex_m::peripheral::syst::SystClkSource;
            use cortex_m::peripheral::SCB;
            #[cfg(feature = "qemu")]
            use ::cortex_m_semihosting::hprintln;

            #[cfg(feature = "qemu")]
            hprintln!("[boot] entered");

            // SAFETY: called exactly once from main() with interrupts
            // disabled (before the scheduler has started). Single-core
            // Cortex-M guarantees exclusive access to these statics and
            // to the exception priority registers.
            unsafe {
                let stacks = &mut *(&raw mut STACKS);
                let partition_sp = &mut *(&raw mut PARTITION_SP);

                #[cfg(feature = "qemu")]
                hprintln!("[boot] init stacks for {} partitions", partitions.len());

                for (i, &(ep, hint)) in partitions.iter().enumerate() {
                    let stk = &mut stacks[i].0;
                    // TODO(panic-free): expect() can panic if stack is too small for
                    // the exception frame. At boot time there is no recovery path, but
                    // a panic-free design would propagate the error to the caller.
                    let ix =
                        $crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                            .expect("init_stack_frame");
                    partition_sp[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
                    #[cfg(feature = "qemu")]
                    hprintln!("[boot] partition {} sp={:#010x}", i, partition_sp[i]);
                }

                const { $crate::config::assert_priority_order::<$Config>() }

                peripherals.SCB.set_priority(
                    SystemHandler::SVCall,
                    <$Config as $crate::config::KernelConfig>::SVCALL_PRIORITY,
                );
                peripherals.SCB.set_priority(
                    SystemHandler::PendSV,
                    <$Config as $crate::config::KernelConfig>::PENDSV_PRIORITY,
                );
                peripherals.SCB.set_priority(
                    SystemHandler::SysTick,
                    <$Config as $crate::config::KernelConfig>::SYSTICK_PRIORITY,
                );
            }

            #[cfg(feature = "qemu")]
            hprintln!("[boot] priorities set");

            let first_partition = ::cortex_m::interrupt::free(|cs| {
                KERNEL
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .and_then(|k| k.start_schedule())
            });

            #[cfg(feature = "qemu")]
            hprintln!("[boot] first_partition={:?}", first_partition);

            // SAFETY: single-core Cortex-M — boot() is called exactly once
            // before SysTick/PendSV are enabled, so exclusive access is
            // guaranteed.
            // TODO(panic-free): expect() can panic if no partition is ready.
            // At boot time there is no recovery path, but a panic-free design
            // would return an error from boot() instead of diverging.
            unsafe {
                core::ptr::write_volatile(
                    &raw mut NEXT_PARTITION,
                    first_partition.expect("no partition ready at boot") as u32,
                );
            }

            #[cfg(feature = "qemu")]
            hprintln!("[boot] triggering PendSV");

            peripherals.SYST.set_clock_source(SystClkSource::Core);
            peripherals.SYST.set_reload(120_000 - 1);
            peripherals.SYST.clear_current();
            peripherals.SYST.enable_counter();
            peripherals.SYST.enable_interrupt();
            SCB::set_pendsv();

            #[cfg(feature = "qemu")]
            hprintln!("[boot] entering idle loop");

            loop {
                cortex_m::asm::wfi();
            }
        }
    };
}

// Unit tests: the define_unified_harness! macro emits global_asm (via
// define_pendsv!) and an #[exception] handler, which are only
// meaningful on ARM targets. Correctness is verified by the QEMU
// integration tests (sampling_demo, queuing_demo, blackboard_demo).
