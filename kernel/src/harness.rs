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
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _harness_handle_tick {
    ($event:expr, $next:ident) => {
        if let $crate::scheduler::ScheduleEvent::PartitionSwitch(pid) = $event {
            unsafe { core::ptr::write_volatile(&raw mut $next, pid as u32) }
            cortex_m::peripheral::SCB::set_pendsv();
        }
    };
}

#[macro_export]
macro_rules! define_harness {
    ($Config:ty, $NP:expr, $MS:expr, $SW:expr) => {
        static mut STACKS: [[u32; $SW]; $NP] = [[0; $SW]; $NP];

        #[no_mangle]
        static mut PARTITION_SP: [u32; $NP] = [0; $NP];

        #[no_mangle]
        static mut CURRENT_PARTITION: u32 = u32::MAX;

        #[no_mangle]
        static mut NEXT_PARTITION: u32 = 0;

        $crate::define_dispatch_hook!($Config);

        static mut KS: Option<
            $crate::kernel::KernelState<{ <$Config as $crate::config::KernelConfig>::N }, $MS>,
        > = None;

        #[used]
        static _SVC: unsafe extern "C" fn(&mut $crate::context::ExceptionFrame) =
            $crate::svc::SVC_HANDLER;

        $crate::define_pendsv!();

        #[exception]
        fn SysTick() {
            let p = &raw mut KS;
            // SAFETY: single-core Cortex-M — the SysTick handler has exclusive
            // access to KS because higher-priority interrupts do not touch it,
            // and PendSV (lower priority) cannot preempt us.
            let event = unsafe { (*p).as_mut() }
                .expect("KS")
                .advance_schedule_tick();
            $crate::_harness_handle_tick!(event, NEXT_PARTITION);
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
                    let stk = &mut stacks[i];
                    let ix =
                        $crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                            .expect("init_stack_frame");
                    partition_sp[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
                }

                peripherals.SCB.set_priority(SystemHandler::SVCall, 0x00);
                peripherals.SCB.set_priority(SystemHandler::PendSV, 0xFF);
                peripherals.SCB.set_priority(SystemHandler::SysTick, 0xFE);
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
