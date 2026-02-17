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
//! | Kernel state | Via `state::init_kernel_state()` in linker-controlled storage |
//! | `static _SVC` | Forces linker to include the SVC assembly trampoline |
//! | `SysTick` exception handler | Drives the round-robin scheduler |
//! | `PendSV` handler | Via [`define_pendsv!`] |
//! | `boot()` | Safe function: inits stacks, configures exceptions, starts OS |
//!
//! Note: Per-partition stacks are stored in [`PartitionCore`] within the `Kernel`
//! struct, not as a separate static array.
//!
//! After invoking the macro, call the generated `boot()` from `main()`
//! to perform the common startup sequence (partition stack init,
//! exception priorities, SysTick configuration, and first PendSV
//! trigger). Returns `Result<Never, BootError>` for panic-free init.

// Re-export BootError and Never from boot module for backwards compatibility.
// The canonical definitions live in boot.rs.
pub use crate::boot::{BootError, Never};

#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick {
    ($kernel:expr) => {{
        let event = $kernel.advance_schedule_tick();
        if let Some(pid) = event {
            $kernel.set_next_partition(pid);
            cortex_m::peripheral::SCB::set_pendsv();
        }
    }};
}

#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick {
    ($kernel:expr, $tick:expr, $strategy:expr) => {{
        let event = $kernel.advance_schedule_tick();
        $crate::_unified_handle_tick_event!($kernel, event, $tick, $strategy);
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
    ($kernel:expr, $event:expr, $tick:expr, $strategy:expr) => {{
        match $event {
            $crate::scheduler::ScheduleEvent::PartitionSwitch(pid) => {
                $kernel.set_next_partition(pid);
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
    ($kernel:expr) => {{
        use $crate::svc::YieldResult;
        let result = $kernel.yield_current_slot();
        if let Some(pid) = result.partition_id() {
            $kernel.set_next_partition(pid);
        }
    }};
}

#[cfg(feature = "dynamic-mpu")]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_yield {
    ($kernel:expr, $tick:expr, $strategy:expr) => {{
        use $crate::svc::YieldResult;
        loop {
            let result = $kernel.yield_current_slot();
            if let Some(pid) = result.partition_id() {
                $kernel.set_next_partition(pid);
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

/// Unified harness: uses linker-controlled kernel state for SVC dispatch and SysTick scheduling.
/// Generates PendSV, SysTick, boot(). Stacks are in `PartitionCore`.
///
/// # Usage
///
/// Basic form (standard scheduler):
/// ```ignore
/// kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS);
/// ```
///
/// Extended form with SysTick hook for test verification:
/// ```ignore
/// kernel::define_unified_harness!(DemoConfig, NUM_PARTITIONS, STACK_WORDS, |tick, k| {
///     // tick: current tick count (u32)
///     // k: &mut Kernel<Config>
///     if tick == 10 { /* verify something */ }
/// });
/// ```
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
    // Basic form: no SysTick hook
    ($Config:ty, $NP:expr, $SW:expr) => {
        $crate::define_unified_harness!(@impl $Config, $NP, $SW, |_tick, _k| {});
    };
    // Extended form: with SysTick hook
    ($Config:ty, $NP:expr, $SW:expr, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@impl $Config, $NP, $SW, |$tick, $k| $hook);
    };
    // no_boot form: handlers only, caller uses kernel::boot directly
    (no_boot, $Config:ty, $NP:expr, $SW:expr) => {
        $crate::define_unified_harness!(@handlers $Config, $NP, $SW, |_tick, _k| {});
    };
    // no_boot form with SysTick hook
    (no_boot, $Config:ty, $NP:expr, $SW:expr, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, $NP, $SW, |$tick, $k| $hook);
    };
    // Internal: handlers only (SysTick, PendSV, SVC linkage, kernel state)
    (@handlers $Config:ty, $NP:expr, $SW:expr, |$tick:ident, $k:ident| $hook:block) => {
        // NOTE: Per-partition stacks are stored in PartitionCore within the Kernel
        // struct, not as a separate static array. The $SW parameter is kept for
        // compatibility with KernelConfig::STACK_WORDS validation.

        // NOTE: CURRENT_PARTITION, NEXT_PARTITION, and PARTITION_SP statics are
        // no longer needed. PendSV reads/writes these values via Rust shims:
        // get_current_partition(), set_current_partition(), get_next_partition(),
        // get_partition_sp(), set_partition_sp(). The partition_sp array now
        // lives inside PartitionCore within the Kernel struct.

        #[cfg(feature = "dynamic-mpu")]
        static HARNESS_STRATEGY: $crate::mpu_strategy::DynamicStrategy =
            $crate::mpu_strategy::DynamicStrategy::new();

        $crate::define_unified_kernel!($Config, |k| {
            #[cfg(not(feature = "dynamic-mpu"))]
            $crate::_unified_handle_yield!(k);
            #[cfg(feature = "dynamic-mpu")]
            {
                let tick_val = k.tick().get();
                $crate::_unified_handle_yield!(k, tick_val, &HARNESS_STRATEGY);
            }
        });

        #[used]
        static _SVC: unsafe extern "C" fn(&mut $crate::context::ExceptionFrame) =
            $crate::svc::SVC_HANDLER;

        #[cfg(not(feature = "dynamic-mpu"))]
        $crate::define_pendsv!();
        #[cfg(feature = "dynamic-mpu")]
        $crate::define_pendsv!(dynamic: HARNESS_STRATEGY);

        #[exception]
        fn SysTick() {
            static _HARNESS_TICK_COUNT: ::core::sync::atomic::AtomicU32 =
                ::core::sync::atomic::AtomicU32::new(0);
            let _systick_tick: u32 = _HARNESS_TICK_COUNT
                .fetch_add(1, ::core::sync::atomic::Ordering::Relaxed)
                + 1;
            #[cfg(feature = "qemu")]
            ::cortex_m_semihosting::hprintln!("[SysTick] #{}", _systick_tick);

            // Single critical section for both systick_handler and user hook to preserve atomicity
            $crate::state::with_kernel_mut::<$Config, _, _>(|_systick_kernel| {
                // Delegate to standalone systick_handler
                $crate::tick::systick_handler::<$Config>(_systick_kernel);

                // Call user-provided SysTick hook
                let $tick = _systick_tick;
                let $k = _systick_kernel;
                $hook
            });
        }
    };
    // Internal: full implementation (handlers + boot function)
    (@impl $Config:ty, $NP:expr, $SW:expr, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, $NP, $SW, |$tick, $k| $hook);

        /// Initialize stacks, priorities, start schedule, enable SysTick, enter idle loop.
        /// Returns `Err(BootError)` on stack init or schedule failure.
        ///
        /// This is a simple forwarding wrapper to the canonical [`boot::boot`] function.
        fn boot(
            partitions: &[(extern "C" fn() -> !, u32)],
            peripherals: &mut cortex_m::Peripherals,
        ) -> Result<$crate::harness::Never, $crate::harness::BootError> {
            $crate::boot::boot::<$Config>(partitions, peripherals)
        }
    };
}

// Unit tests: the define_unified_harness! macro emits global_asm (via
// define_pendsv!) and an #[exception] handler, which are only
// meaningful on ARM targets. Correctness is verified by the QEMU
// integration tests (sampling_demo, queuing_demo, blackboard_demo).

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;

    /// Fixed-size buffer for formatting in no_std tests.
    struct FmtBuf {
        buf: [u8; 64],
        len: usize,
    }

    impl FmtBuf {
        const fn new() -> Self {
            Self {
                buf: [0u8; 64],
                len: 0,
            }
        }

        fn as_str(&self) -> &str {
            core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
        }
    }

    impl Write for FmtBuf {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let remaining = self.buf.len() - self.len;
            let to_copy = bytes.len().min(remaining);
            self.buf[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
            self.len += to_copy;
            Ok(())
        }
    }

    #[test]
    fn boot_error_construction_and_traits() {
        let err = BootError::StackInitFailed { partition_index: 2 };
        assert_eq!(err, BootError::StackInitFailed { partition_index: 2 });
        assert_ne!(err, BootError::NoReadyPartition);
        assert_eq!(BootError::NoReadyPartition, BootError::NoReadyPartition);

        // Test Display impl using core::fmt::Write (no std dependency)
        let mut buf = FmtBuf::new();
        write!(&mut buf, "{}", err).unwrap();
        assert!(buf.as_str().contains("2"));
    }
}
