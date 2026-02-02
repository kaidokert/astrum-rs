//! Shared runtime harness for QEMU demo examples.
//!
//! The [`define_harness!`] macro emits the boilerplate statics, SVC
//! linkage, PendSV context-switch handler, and SysTick exception
//! handler that every multi-partition example duplicates.
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
//!
//! After invoking the macro, call [`boot`] from `main()` to perform
//! the common startup sequence (partition stack init, exception
//! priorities, SysTick configuration, and first PendSV trigger).

/// Initialise partition stacks, configure exception priorities, start
/// SysTick, trigger the first PendSV, and enter the idle loop.
///
/// # Type parameters
///
/// - `STACK_WORDS`: number of `u32` words per partition stack (e.g. `256`).
///
/// # Safety
///
/// Must be called exactly once from `main()` with interrupts disabled
/// (before the scheduler has started). The caller must ensure:
/// - `stacks` points to a valid `static mut [[u32; STACK_WORDS]; N]` array
///   with at least `partitions.len()` entries.
/// - `partition_sp` points to a valid `static mut [u32; N]` array
///   with at least `partitions.len()` entries.
/// - `partitions` contains valid `(entry_point, r0_hint)` pairs.
#[cfg(not(test))]
pub unsafe fn boot<const STACK_WORDS: usize>(
    stacks: *mut [[u32; STACK_WORDS]],
    partition_sp: *mut [u32],
    partitions: &[(extern "C" fn() -> !, u32)],
    peripherals: &mut cortex_m::Peripherals,
) -> ! {
    use cortex_m::peripheral::scb::SystemHandler;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SCB;

    // SAFETY: caller guarantees the pointers are valid and we have
    // exclusive access (interrupts disabled, single-core).
    let stacks = &mut *stacks;
    let partition_sp = &mut *partition_sp;

    for (i, &(ep, hint)) in partitions.iter().enumerate() {
        let stk = &mut stacks[i];
        let ix = crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
            .expect("init_stack_frame");
        partition_sp[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
    }

    peripherals.SCB.set_priority(SystemHandler::SVCall, 0x00);
    peripherals.SCB.set_priority(SystemHandler::PendSV, 0xFF);
    peripherals.SCB.set_priority(SystemHandler::SysTick, 0xFE);

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

/// Declare all runtime statics, the SysTick handler, SVC linkage, and
/// PendSV handler for a QEMU demo example.
///
/// # Parameters
///
/// - `$Config`: a type implementing [`KernelConfig`](crate::config::KernelConfig)
/// - `$NP`: number of partitions (e.g. `3`)
/// - `$MS`: maximum schedule entries (e.g. `8`)
/// - `$SW`: per-partition stack size in `u32` words (e.g. `256`)
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
/// ```
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
            if let Some(pid) = unsafe { (*p).as_mut() }
                .expect("KS")
                .advance_schedule_tick()
            {
                // SAFETY: same single-core exclusivity as above.
                unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
                cortex_m::peripheral::SCB::set_pendsv();
            }
        }
    };
}

// Unit tests: the define_harness! macro emits global_asm (via
// define_pendsv!) and an #[exception] handler, which are only
// meaningful on ARM targets.  Correctness is verified by the QEMU
// integration tests (sampling_demo, queuing_demo, blackboard_demo).
