//! Shared runtime harness for QEMU demo examples.
//!
//! The [`define_harness!`] macro emits the boilerplate statics, SVC
//! linkage, PendSV context-switch handler, SysTick exception handler,
//! and a safe `boot()` function that every multi-partition example
//! would otherwise duplicate.
//!
//! # Usage
//!
//! ```ignore
//! kernel::define_harness!(DemoConfig, NUM_PARTITIONS, MAX_SCHEDULE_ENTRIES, STACK_WORDS);
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
//! | `static mut KS` | `Option<KernelState<…>>` for the SysTick handler |
//! | `static _SVC` | Forces linker to include the SVC assembly trampoline |
//! | `SysTick` exception handler | Drives the round-robin scheduler |
//! | `KERN` / `dispatch_hook` / `store_kernel` | Via [`define_dispatch_hook!`] |
//! | `PendSV` handler | Via [`define_pendsv!`] |
//! | `boot()` | Safe function: inits stacks, configures exceptions, starts OS |
//!
//! After invoking the macro, call the generated `boot()` from `main()`
//! to perform the common startup sequence (partition stack init,
//! exception priorities, SysTick configuration, and first PendSV
//! trigger).

/// Declare all runtime statics, the SysTick handler, SVC linkage,
/// PendSV handler, and a safe `boot()` function for a QEMU demo
/// example.
///
/// # Parameters
///
/// - `$Config`: a type implementing [`KernelConfig`](crate::config::KernelConfig)
/// - `$NP`: number of partitions (e.g. `3`)
/// - `$MS`: maximum schedule entries (e.g. `8`)
/// - `$SW`: per-partition stack size in `u32` words (e.g. `256`)
///
/// # Generated `boot()` function
///
/// ```ignore
/// fn boot(
///     partitions: &[(extern "C" fn() -> !, u32)],
///     peripherals: &mut cortex_m::Peripherals,
/// ) -> !
/// ```
///
/// Initialises partition stacks, configures exception priorities,
/// starts SysTick, triggers the first PendSV, and enters the idle
/// loop.  Must be called exactly once from `main()` before the
/// scheduler has started (interrupts disabled).
///
/// # Example
///
/// ```ignore
/// const NUM_PARTITIONS: usize = 3;
/// const MAX_SCHEDULE_ENTRIES: usize = 8;
/// const STACK_WORDS: usize = 256;
/// struct DemoConfig;
/// impl KernelConfig for DemoConfig { /* … */ }
///
/// kernel::define_harness!(DemoConfig, NUM_PARTITIONS, MAX_SCHEDULE_ENTRIES, STACK_WORDS);
///
/// fn main() -> ! {
///     let mut p = cortex_m::Peripherals::take().unwrap();
///     // … create kernel, schedule, KS …
///     boot(&parts, &mut p)
/// }
/// ```
/// Internal helper: force-advance the schedule on yield and update
/// `NEXT_PARTITION`.  Uses the [`YieldResult`] trait so a single
/// implementation works for both feature gates.
#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _harness_handle_yield {
    ($ks:expr, $next:ident) => {{
        use $crate::kernel::YieldResult;
        let result = $ks.yield_current_slot();
        if let Some(pid) = result.partition_id() {
            // SAFETY: single-core Cortex-M — SVC (priority 0x00) has
            // exclusive access; PendSV (priority 0xFF) cannot preempt.
            unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
        }
    }};
}

/// Dynamic-mpu variant: if yield lands on a system window, run the
/// bottom-half using the already-borrowed kernel reference and keep
/// advancing until a partition slot is reached. When the bottom-half
/// detects RX data, the oldest blocked device reader is woken using
/// KernelState partitions.
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _harness_handle_yield {
    ($ks:expr, $next:ident, $kern:expr, $tick:expr, $strategy:expr) => {{
        use $crate::kernel::YieldResult;
        loop {
            let result = $ks.yield_current_slot();
            if let Some(pid) = result.partition_id() {
                // SAFETY: single-core Cortex-M — SVC (priority 0x00) has
                // exclusive access; PendSV (priority 0xFF) cannot preempt.
                unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
                break;
            }
            if result.is_system_window() {
                let bh = $crate::tick::run_bottom_half(
                    &mut $kern.uart_pair,
                    &mut $kern.isr_ring,
                    &mut $kern.buffers,
                    &mut $kern.hw_uart,
                    $tick,
                    $strategy,
                );
                if bh.has_rx_data {
                    if let Some(woken) = $kern.dev_wait_queue.wake_one_reader() {
                        $crate::svc::try_transition(
                            $ks.partitions_mut(),
                            woken,
                            $crate::partition::PartitionState::Ready,
                        );
                    }
                }
                continue;
            }
            // ScheduleEvent::None — schedule not started or empty.
            break;
        }
    }};
}

/// Internal helper: dispatch the result of `advance_schedule_tick` inside
/// the SysTick handler.  Two implementations are provided so the harness
/// compiles under both feature configurations.
#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _harness_handle_tick {
    ($event:expr, $next:ident) => {
        if let Some(pid) = $event {
            unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
            cortex_m::peripheral::SCB::set_pendsv();
        }
    };
}

/// Internal helper (dynamic-mpu variant).
///
/// On `PartitionSwitch` this behaves identically to the non-dynamic
/// variant.  On `SystemWindow` it borrows `KERN` and calls
/// [`run_bottom_half`](crate::tick::run_bottom_half) to perform
/// deferred kernel work (UART transfer, ISR drain, buffer revocation,
/// and device reader wake-up).
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _harness_handle_tick {
    ($event:expr, $next:ident, $tick:expr, $strategy:expr, $partitions:expr) => {
        match $event {
            $crate::scheduler::ScheduleEvent::PartitionSwitch(pid) => {
                // SAFETY: single-core Cortex-M — SysTick has exclusive
                // access to NEXT_PARTITION; PendSV (lower priority)
                // cannot preempt us.
                unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
                cortex_m::peripheral::SCB::set_pendsv();
            }
            $crate::scheduler::ScheduleEvent::SystemWindow => {
                ::cortex_m::interrupt::free(|cs| {
                    if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                        let bh = $crate::tick::run_bottom_half(
                            &mut k.uart_pair,
                            &mut k.isr_ring,
                            &mut k.buffers,
                            &mut k.hw_uart,
                            $tick,
                            $strategy,
                        );
                        if bh.has_rx_data {
                            if let Some(woken) = k.dev_wait_queue.wake_one_reader() {
                                $crate::svc::try_transition(
                                    $partitions,
                                    woken,
                                    $crate::partition::PartitionState::Ready,
                                );
                            }
                        }
                    }
                });
            }
            $crate::scheduler::ScheduleEvent::None => {}
        }
    };
}

#[macro_export]
macro_rules! define_harness {
    ($Config:ty, $NP:expr, $MS:expr, $SW:expr) => {
        /// Wrapper that aligns each partition stack to its byte-size so
        /// that the address is valid as an MPU region base.
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

        $crate::define_dispatch_hook!(
            $Config,
            |_k| {
                // SAFETY: single-core Cortex-M — SVC (priority 0x00) has
                // exclusive access; PendSV cannot preempt us.  KS and
                // NEXT_PARTITION are defined by define_harness!.
                if let Some(ks) = unsafe { KS.as_mut() } {
                    #[cfg(not(feature = "dynamic-mpu"))]
                    $crate::_harness_handle_yield!(ks, NEXT_PARTITION);
                    #[cfg(feature = "dynamic-mpu")]
                    $crate::_harness_handle_yield!(
                        ks,
                        NEXT_PARTITION,
                        _k,
                        ks.tick().get(),
                        &HARNESS_STRATEGY
                    );
                }
            },
            |_cs| {
                // SAFETY: KS is initialised before SysTick/SVC fire.
                // Single-core Cortex-M: interrupts are masked by the
                // surrounding interrupt::free, so exclusive access is
                // guaranteed.
                unsafe { KS.as_mut().map(|ks| ks.partitions_mut()) }
            }
        );

        // NOTE: KernelState::new returns Result<KernelState, ConfigError>.
        // Call sites (in each example's main()) must use .expect() or
        // .unwrap() before wrapping in Some(...) when storing into KS.
        static mut KS: Option<
            $crate::kernel::KernelState<{ <$Config as $crate::config::KernelConfig>::N }, $MS>,
        > = None;

        #[used]
        static _SVC: unsafe extern "C" fn(&mut $crate::context::ExceptionFrame) =
            $crate::svc::SVC_HANDLER;

        $crate::define_pendsv!();

        #[exception]
        fn SysTick() {
            // SAFETY: single-core Cortex-M — the SysTick handler has exclusive
            // access to KS because higher-priority interrupts do not touch it,
            // and PendSV (lower priority) cannot preempt us.  KS is a
            // `static mut Option<KernelState>` defined by this macro; only
            // boot() writes `Some(…)` into it (once, with interrupts
            // disabled) before enabling SysTick, and nothing ever writes
            // `None` back, so the pointer returned by `as_mut()` is valid
            // and unique for the lifetime of this handler invocation.
            let ks = unsafe { KS.as_mut() };
            let ks = if let Some(ks) = ks {
                ks
            } else {
                // KS must be initialised before SysTick fires; if it is
                // not, the kernel is in an unrecoverable state.
                loop {}
            };

            let event = ks.advance_schedule_tick();
            let current_tick = ks.tick().get();
            let ks_parts = ks.partitions_mut();

            #[cfg(not(feature = "dynamic-mpu"))]
            $crate::_harness_handle_tick!(event, NEXT_PARTITION);
            #[cfg(feature = "dynamic-mpu")]
            $crate::_harness_handle_tick!(
                event,
                NEXT_PARTITION,
                current_tick,
                &HARNESS_STRATEGY,
                ks_parts
            );

            // Synchronize Kernel.tick with the authoritative KernelState.tick
            // and expire timed waits (queuing ports, blackboards) each tick.
            ::cortex_m::interrupt::free(|cs| {
                if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                    k.sync_tick(current_tick);
                    k.expire_timed_waits::<{ <$Config as $crate::config::KernelConfig>::N }>(
                        current_tick,
                        ks_parts,
                    );
                }
            });
        }

        /// Initialise partition stacks, configure exception priorities,
        /// start SysTick, trigger the first PendSV, and enter the idle
        /// loop.
        ///
        /// Must be called exactly once from `main()` before the scheduler
        /// has started (interrupts disabled, single-core).
        fn boot(
            partitions: &[(extern "C" fn() -> !, u32)],
            peripherals: &mut cortex_m::Peripherals,
        ) -> ! {
            use cortex_m::peripheral::scb::SystemHandler;
            use cortex_m::peripheral::syst::SystClkSource;
            use cortex_m::peripheral::SCB;

            // SAFETY: called exactly once from main() with interrupts
            // disabled (before the scheduler has started). Single-core
            // Cortex-M guarantees exclusive access to these statics and
            // to the exception priority registers.
            unsafe {
                let stacks = &mut *(&raw mut STACKS);
                let partition_sp = &mut *(&raw mut PARTITION_SP);

                for (i, &(ep, hint)) in partitions.iter().enumerate() {
                    let stk = &mut stacks[i].0;
                    let ix =
                        $crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                            .expect("init_stack_frame");
                    partition_sp[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
                }

                // Compile-time check: PendSV must be lower priority
                // (larger number) than SysTick.
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

            peripherals.SYST.set_clock_source(SystClkSource::Core);
            peripherals.SYST.set_reload(120_000 - 1);
            peripherals.SYST.clear_current();
            peripherals.SYST.enable_counter();
            peripherals.SYST.enable_interrupt();
            SCB::set_pendsv();

            loop {
                cortex_m::asm::wfi();
            }
        }
    };
}

// Unit tests: the define_harness! macro emits global_asm (via
// define_pendsv!) and an #[exception] handler, which are only
// meaningful on ARM targets.  Correctness is verified by the QEMU
// integration tests (sampling_demo, queuing_demo, blackboard_demo).
