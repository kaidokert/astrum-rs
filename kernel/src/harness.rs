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
        // Defense-in-depth: verify kernel storage alignment at every tick
        // (debug builds only; see TODO(panic-free) in invariants.rs).
        #[cfg(debug_assertions)]
        {
            let addr = $kernel as *const _ as usize;
            $crate::invariants::assert_storage_alignment(addr, $crate::state::KERNEL_ALIGNMENT);
        }
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
        // Defense-in-depth: verify kernel storage alignment at every tick
        // (debug builds only; see TODO(panic-free) in invariants.rs).
        #[cfg(debug_assertions)]
        {
            let addr = $kernel as *const _ as usize;
            $crate::invariants::assert_storage_alignment(addr, $crate::state::KERNEL_ALIGNMENT);
        }
        let event = $kernel.advance_schedule_tick();
        $crate::_unified_handle_tick_event!($kernel, event, $tick, $strategy);
        #[cfg(feature = "dynamic-mpu")]
        $kernel.fallback_revoke_expired_buffers();
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
        let bh = $crate::run_bottom_half!($kernel, $tick, $strategy);
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
/// # Supported Stack Sizes
///
/// The `KernelConfig::Core` type parameter selects the stack tier via one of
/// the pre-defined `AlignedStack*` types (256B, 512B, 1K, 2K, 4K). Each
/// type has alignment equal to its size, as required by the Cortex-M MPU.
/// A compile-time assertion in `PartitionCore::new()` verifies this invariant.
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

        /// Boot-time MPU initialisation hook called from `boot::boot()`
        /// before `SCB::set_pendsv()` triggers the first context switch.
        /// Programs static regions R0-R3 for the first scheduled partition.
        #[cfg(not(feature = "dynamic-mpu"))]
        #[no_mangle]
        fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) {
            if !<$Config as $crate::config::KernelConfig>::MPU_ENFORCE { return; }
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                let pcb = k.partitions().get(pid as usize)
                    .expect("boot: next_partition PID missing from partition table");
                $crate::mpu::apply_partition_mpu(mpu, pcb);
            });
        }

        /// Boot-time MPU initialisation hook called from `boot::boot()`
        /// before `SCB::set_pendsv()` triggers the first context switch.
        /// Programs static regions R0-R3 and dynamic slot 0 (R4) for the
        /// first scheduled partition.  Also populates dynamic slots 1-3
        /// (R5-R7) with peripheral regions from all partitions, so that
        /// `program_regions()` hardware-programs them on every PendSV
        /// context switch.
        #[cfg(feature = "dynamic-mpu")]
        #[no_mangle]
        fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) {
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                // TODO(panic-free): convert to Result once __boot_mpu_init
                // can propagate errors back to boot().
                let pcb = k.partitions().get(pid as usize)
                    .expect("boot: next_partition PID missing from partition table");
                $crate::mpu::apply_partition_mpu(mpu, pcb);
                if let Some(regions) = $crate::mpu::partition_dynamic_regions(pcb) {
                    let periph_reserved = if pcb.peripheral_regions().is_empty() { 0 } else { 2 };
                    $crate::mpu_strategy::MpuStrategy::configure_partition(
                        &HARNESS_STRATEGY, pid, &regions, periph_reserved,
                    )
                    .expect("failed to configure boot MPU");
                }

                // Populate dynamic slots 1-3 (R5-R7) with deduplicated
                // peripheral regions from all partitions.
                HARNESS_STRATEGY.wire_boot_peripherals(
                    k.partitions().as_slice(),
                );
            });
        }

        $crate::define_unified_kernel!($Config, |k| {
            #[cfg(not(feature = "dynamic-mpu"))]
            $crate::_unified_handle_yield!(k);
            #[cfg(feature = "dynamic-mpu")]
            {
                let tick_val = k.tick().get();
                $crate::_unified_handle_yield!(k, tick_val, &HARNESS_STRATEGY);
            }
            // Flush partition debug buffers at yield boundaries.
            // Zero-cost when partition-debug is disabled or budget is 0.
            k.drain_debug_auto();
        });

        #[used]
        static _SVC: unsafe extern "C" fn(&mut $crate::context::ExceptionFrame) =
            $crate::svc::SVC_HANDLER;

        /// PendSV MPU programming shim: reprogram MPU regions for the
        /// incoming partition on every context switch.
        ///
        /// - Static mode: R0-R3 (base) + R4-R5 (peripheral) via
        ///   `apply_partition_mpu` (self-contained disable/enable cycle).
        /// - Dynamic mode: single disable/enable cycle writing R0-R3
        ///   (base partition) then R4-R7 (dynamic strategy), avoiding
        ///   redundant MPU state transitions and overlapping writes.
        #[export_name = "__pendsv_program_mpu"]
        extern "C" fn __pendsv_program_mpu() {
            // SAFETY: PendSV is the lowest-priority exception and CAN be
            // preempted by higher-priority ISRs (SysTick, SVC).  `steal()`
            // is sound for MPU access because no other exception handler in
            // this system writes to MPU registers — SysTick only advances
            // the schedule and SVC dispatch does not reprogram the MPU.
            let p = unsafe { cortex_m::Peripherals::steal() };

            // Dynamic mode: disable MPU up front so base-region and
            // dynamic-strategy writes share a single disable/enable cycle.
            #[cfg(feature = "dynamic-mpu")]
            $crate::mpu::mpu_disable(&p.MPU);

            #[cfg_attr(not(feature = "dynamic-mpu"), allow(unused_variables))]
            let pid = $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                let pcb = match k.partitions().get(pid as usize) {
                    Some(pcb) => pcb,
                    None => {
                        // [KPANIC:pendsv-bad-pid] Fatal: apply deny-all
                        // and reset — panics are unrecoverable in PendSV.
                        $crate::klog!("[KPANIC:pendsv-bad-pid] pid={}", pid);
                        $crate::mpu::apply_deny_all_mpu(&p.MPU);
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                };

                // Static mode: apply_partition_mpu handles R0-R5 with
                // its own disable/enable cycle.
                #[cfg(not(feature = "dynamic-mpu"))]
                if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                    $crate::mpu::apply_partition_mpu(&p.MPU, pcb);
                }

                // Dynamic mode: write only R0-R3 base partition regions
                // (MPU already disabled above; R4-R5 overridden below).
                #[cfg(feature = "dynamic-mpu")]
                {
                    let regions = $crate::mpu::partition_mpu_regions_or_deny_all(pcb);
                    for &(rbar, rasr) in &regions {
                        $crate::mpu::configure_region(&p.MPU, rbar, rasr);
                    }

                    // Update dynamic strategy: reconfigure the partition's
                    // private-RAM slot and peripheral reservation count.
                    let periph_reserved = if pcb.peripheral_regions().is_empty() { 0 } else { 2 };
                    if let Some(dyn_regions) = $crate::mpu::partition_dynamic_regions(pcb) {
                        let _ = $crate::mpu_strategy::MpuStrategy::configure_partition(
                            &HARNESS_STRATEGY, pid, &dyn_regions, periph_reserved,
                        );
                    }
                }

                // Return pid for dynamic-mode peripheral cache lookup.
                // In static mode this is unused.
                pid
            });

            // Dynamic mode: write R4-R7 strategy regions, then
            // override R4-R5 with cached peripheral regions and re-enable.
            #[cfg(feature = "dynamic-mpu")]
            {
                let values = HARNESS_STRATEGY.compute_region_values();
                for &(rbar, rasr) in &values {
                    $crate::mpu::configure_region(&p.MPU, rbar, rasr);
                }

                // Overwrite R4-R5 with per-partition peripheral regions
                // from the DynamicStrategy's boot-time cache.
                let periph = HARNESS_STRATEGY.cached_peripheral_regions(pid);
                for &(rbar, rasr) in &periph {
                    $crate::mpu::configure_region(&p.MPU, rbar, rasr);
                }

                $crate::mpu::mpu_enable(&p.MPU);
            }
        }
        $crate::define_pendsv!(@impl "bl __pendsv_program_mpu");

        #[exception]
        fn SysTick() {
            static _HARNESS_TICK_COUNT: ::core::sync::atomic::AtomicU32 =
                ::core::sync::atomic::AtomicU32::new(0);
            let _systick_tick: u32 = _HARNESS_TICK_COUNT
                .fetch_add(1, ::core::sync::atomic::Ordering::Relaxed)
                + 1;
            #[cfg(feature = "qemu")]
            $crate::klog!("[SysTick] #{}", _systick_tick);

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
    use crate::config::KernelConfig;
    use crate::kernel_config_types;
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

    // ============ MPU_ENFORCE gating tests ============
    //
    // The define_unified_harness! macro emits __boot_mpu_init and
    // __pendsv_program_mpu with MPU_ENFORCE-based gating.  These
    // functions require ARM peripherals, so we test the gating
    // pattern itself using helper functions that mirror the exact
    // const-generic branching used in the macro.

    /// Config with default MPU_ENFORCE (false).
    struct GateTestDefault;
    impl KernelConfig for GateTestDefault {
        const N: usize = 2;
        kernel_config_types!();
    }

    /// Config with MPU_ENFORCE = true.
    struct GateTestEnforced;
    impl KernelConfig for GateTestEnforced {
        const N: usize = 2;
        const MPU_ENFORCE: bool = true;
        kernel_config_types!();
    }

    /// Mirrors the __boot_mpu_init gating pattern:
    /// `if !<Config>::MPU_ENFORCE { return; }` — returns false for
    /// early-return (no-op), true when MPU would be programmed.
    fn boot_mpu_init_would_program<C: KernelConfig>() -> bool {
        if !C::MPU_ENFORCE {
            return false;
        }
        true
    }

    /// Mirrors the PendSV gating pattern:
    /// `if <Config>::MPU_ENFORCE { apply... }` — returns true when
    /// apply_partition_mpu would be called, false when skipped.
    fn pendsv_would_apply_mpu<C: KernelConfig>() -> bool {
        if C::MPU_ENFORCE {
            return true;
        }
        false
    }

    #[test]
    fn boot_mpu_init_skips_when_mpu_enforce_false() {
        assert!(!boot_mpu_init_would_program::<GateTestDefault>());
    }

    #[test]
    fn boot_mpu_init_programs_when_mpu_enforce_true() {
        assert!(boot_mpu_init_would_program::<GateTestEnforced>());
    }

    #[test]
    fn pendsv_apply_skips_when_mpu_enforce_false() {
        assert!(!pendsv_would_apply_mpu::<GateTestDefault>());
    }

    #[test]
    fn pendsv_apply_runs_when_mpu_enforce_true() {
        assert!(pendsv_would_apply_mpu::<GateTestEnforced>());
    }

    // Compile-time const assertions: verify that the gating expressions
    // used in the macro resolve correctly at const-eval time.
    const _: () = assert!(!GateTestDefault::MPU_ENFORCE);
    const _: () = assert!(GateTestEnforced::MPU_ENFORCE);
}
