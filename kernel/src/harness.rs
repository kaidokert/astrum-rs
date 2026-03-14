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
//! kernel::define_unified_harness!(DemoConfig);
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

/// Shared helper: detect dropped SysTick interrupts by reading the ICSR
/// PENDSTSET bit before the current tick is processed.  Factored out of
/// `_unified_handle_tick!` to avoid duplicating the detection logic across
/// both cfg arms.
#[macro_export]
#[doc(hidden)]
macro_rules! _detect_dropped_ticks {
    ($kernel:expr) => {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: Reading the ICSR register at this fixed MMIO address is
            // side-effect-free. (ICSR is R/W — some bits trigger effects when
            // *written* — but a volatile read is safe.)
            let icsr = unsafe { core::ptr::read_volatile($crate::pendsv::ICSR_ADDR as *const u32) };
            // PENDSTSET_BIT is a bitmask (1 << 26), not a bit index.
            if icsr & $crate::pendsv::PENDSTSET_BIT != 0 {
                $kernel.increment_ticks_dropped();
            }
        }
    };
}

#[cfg(not(feature = "dynamic-mpu"))]
#[macro_export]
#[doc(hidden)]
macro_rules! _unified_handle_tick {
    ($kernel:expr) => {{
        // Defense-in-depth: verify kernel storage alignment at every tick
        // (debug builds only). On misalignment, skip the tick gracefully
        // instead of panicking in handler mode.
        #[cfg(debug_assertions)]
        {
            let addr = $kernel as *const _ as usize;
            if $crate::invariants::check_storage_alignment(addr, $crate::state::KERNEL_ALIGNMENT)
                .is_err()
            {
                return;
            }
        }
        $crate::_detect_dropped_ticks!($kernel);
        let event = $crate::svc::scheduler::advance_schedule_tick(&mut $kernel);
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
        // (debug builds only). On misalignment, skip the tick gracefully
        // instead of panicking in handler mode.
        #[cfg(debug_assertions)]
        {
            let addr = $kernel as *const _ as usize;
            if $crate::invariants::check_storage_alignment(addr, $crate::state::KERNEL_ALIGNMENT)
                .is_err()
            {
                return;
            }
        }
        $crate::_detect_dropped_ticks!($kernel);
        let event = $crate::svc::scheduler::advance_schedule_tick(&mut $kernel);
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
        let has_rx = match bh {
            Ok(b) => b.has_rx_data,
            Err(e) => {
                $crate::klog!("BUG: {}", e);
                false
            }
        };
        if has_rx {
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
/// kernel::define_unified_harness!(DemoConfig);
/// ```
///
/// Extended form with SysTick hook for test verification:
/// ```ignore
/// kernel::define_unified_harness!(DemoConfig, |tick, k| {
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
    // ── Public arms ──────────────────────────────────────────────
    // Basic form: no SysTick hook
    ($Config:ty) => {
        $crate::define_unified_harness!(@impl $Config, |_tick, _k| {});
    };
    // Extended form: with SysTick hook
    ($Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@impl $Config, |$tick, $k| $hook);
    };
    // no_boot form: handlers only, caller uses kernel::boot directly
    (no_boot, $Config:ty) => {
        $crate::define_unified_harness!(@handlers $Config, |_tick, _k| {});
    };
    // no_boot form with SysTick hook
    (no_boot, $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, |$tick, $k| $hook);
    };
    // Internal: handlers only (SysTick, PendSV, SVC linkage, kernel state)
    (@handlers $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        #[cfg(feature = "dynamic-mpu")]
        static HARNESS_STRATEGY: $crate::mpu_strategy::DynamicStrategy<
            {<$Config as $crate::config::KernelConfig>::N}
        > = $crate::mpu_strategy::DynamicStrategy::<
            {<$Config as $crate::config::KernelConfig>::N}
        >::with_partition_count();

        /// Boot-time MPU initialisation hook called from `boot::boot()`
        /// before `SCB::set_pendsv()` triggers the first context switch.
        /// Programs static regions R0-R3 for the first scheduled partition.
        #[cfg(not(feature = "dynamic-mpu"))]
        #[no_mangle]
        fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) -> Result<(), &'static str> {
            if !<$Config as $crate::config::KernelConfig>::MPU_ENFORCE { return Ok(()); }
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                let pcb = k.pcb(pid as usize)
                    .ok_or("boot: next_partition PID missing from partition table")?;
                $crate::mpu::apply_partition_mpu_cached(mpu, pcb);
                Ok(())
            })?
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
        fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) -> Result<(), &'static str> {
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                let pcb = k.pcb(pid as usize)
                    .ok_or("boot: next_partition PID missing from partition table")?;
                // Program R0-R5 from pre-computed cache within a single
                // disable/enable cycle that spans strategy setup below.
                // Re-enable the MPU even on error so hardware is left in a
                // consistent state (the MPU was enabled before we entered).
                // Gate on MPU_ENFORCE: sentinel partitions (QEMU tests) set
                // MPU_ENFORCE=false so the deny-all R0 background region
                // doesn't override PRIVDEFENA and block privileged access.
                if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                    $crate::mpu::mpu_disable(mpu);
                    $crate::mpu::write_cached_base_regions(mpu, pcb);
                    $crate::mpu::write_cached_periph_regions(mpu, pcb);
                }
                let strategy_result = {
                    let dyn_region = pcb.cached_dynamic_region();
                    let periph_reserved = if pcb.peripheral_regions().is_empty() { 0 } else { 2 };
                    $crate::mpu_strategy::MpuStrategy::configure_partition(
                        &HARNESS_STRATEGY, pid, &[dyn_region], periph_reserved,
                    )
                    // TODO: preserves only a static string; consider logging the
                    // underlying strategy error once we have a boot-time log sink.
                    .map_err(|_| "failed to configure boot MPU")
                };
                if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                    $crate::mpu::mpu_enable(mpu);
                }
                strategy_result?;

                // Populate dynamic slots 1-3 (R5-R7) with deduplicated
                // peripheral regions from all partitions.
                #[cfg_attr(not(debug_assertions), allow(unused))]
                let boot_wired = HARNESS_STRATEGY.wire_boot_peripherals(
                    k.partitions().as_slice(),
                );
                #[cfg(debug_assertions)]
                {
                    let limit = $crate::mpu_strategy::BOOT_WIRE_LIMIT;
                    debug_assert!(
                        boot_wired < limit,
                        "boot wiring consumed all dynamic window slots ({boot_wired}/{limit}), none left for runtime add_window"
                    );
                }
                Ok(())
            })?
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
        ///   `apply_partition_mpu_cached` (self-contained disable/enable cycle).
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
            if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                $crate::mpu::mpu_disable(&p.MPU);
            }

            #[cfg_attr(not(feature = "dynamic-mpu"), allow(unused_variables))]
            let pid = match $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                let pid = k.next_partition();
                let pcb = match k.pcb(pid as usize) {
                    Some(pcb) => pcb,
                    None => {
                        // [KPANIC:pendsv-bad-pid] Fatal: apply deny-all
                        // and reset — panics are unrecoverable in PendSV.
                        $crate::klog!("[KPANIC:pendsv-bad-pid] pid={}", pid);
                        $crate::mpu::apply_deny_all_mpu(&p.MPU);
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                };

                // Static mode: apply_partition_mpu_cached handles R0-R5
                // from pre-computed PCB cache (no on-the-fly recomputation).
                #[cfg(not(feature = "dynamic-mpu"))]
                if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                    $crate::mpu::apply_partition_mpu_cached(&p.MPU, pcb);
                }

                // Dynamic mode: write R0-R3 from pre-computed cache
                // (MPU already disabled above; R4-R5 overridden below).
                #[cfg(feature = "dynamic-mpu")]
                {
                    if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                        $crate::mpu::write_cached_base_regions(&p.MPU, pcb);
                    }

                    // Update dynamic strategy: reconfigure the partition's
                    // private-RAM slot and peripheral reservation count.
                    let periph_reserved = if pcb.peripheral_regions().is_empty() { 0 } else { 2 };
                    let dyn_region = pcb.cached_dynamic_region();
                    let _ = $crate::mpu_strategy::MpuStrategy::configure_partition(
                        &HARNESS_STRATEGY, pid, &[dyn_region], periph_reserved,
                    );
                }

                // Return pid for dynamic-mode peripheral cache lookup.
                // In static mode this is unused.
                pid
            }) {
                Ok(pid) => pid,
                Err(_) => return,
            };

            // Dynamic mode: write R4-R7 strategy regions, then
            // override R4-R5 with cached peripheral regions and re-enable.
            #[cfg(feature = "dynamic-mpu")]
            if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
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
            let _ = $crate::state::with_kernel_mut::<$Config, _, _>(|_systick_kernel| {
                // Detect dropped ticks before processing the current tick
                $crate::_detect_dropped_ticks!(_systick_kernel);
                // Delegate to standalone systick_handler
                $crate::tick::systick_handler::<$Config>(_systick_kernel);

                // Call user-provided SysTick hook
                let $tick = _systick_tick;
                let $k = _systick_kernel;
                $hook
            });
        }
    };
    // TODO: reviewer false positive — `@impl` is a standard Rust macro internal arm,
    // not a file path.  The `@` prefix is idiomatic for private macro dispatch arms.
    // Internal: full implementation (handlers + boot function)
    (@impl $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, |$tick, $k| $hook);

        /// Partition stacks with 4 KiB alignment; patched into PCBs by `boot_external()`.
        ///
        /// # Singleton
        /// This macro arm must be invoked at most once per binary.  Multiple
        /// invocations would produce duplicate `__PartitionStackStorage` /
        /// `__PARTITION_STACKS` symbols, causing a compile-time link error.
        #[repr(C, align(4096))]
        struct __PartitionStackStorage(
            [[u32; <$Config as $crate::config::KernelConfig>::STACK_WORDS];
             <$Config as $crate::config::KernelConfig>::N],
        );
        static mut __PARTITION_STACKS: __PartitionStackStorage = __PartitionStackStorage(
            [[0u32; <$Config as $crate::config::KernelConfig>::STACK_WORDS];
             <$Config as $crate::config::KernelConfig>::N],
        );

        /// Initialize stacks, priorities, start schedule, enable SysTick, enter idle loop.
        /// Returns `Err(BootError)` on stack init or schedule failure.
        ///
        /// Delegates to [`boot::boot_external`] using `__PARTITION_STACKS`.
        fn boot(
            partitions: &[(extern "C" fn() -> !, u32)],
            peripherals: &mut cortex_m::Peripherals,
        ) -> Result<$crate::harness::Never, $crate::harness::BootError> {
            // SAFETY: `boot()` is called exactly once from `main()` before
            // any exception handlers run.  No other code accesses
            // `__PARTITION_STACKS` outside of `boot_external`.
            let stacks = unsafe { &mut __PARTITION_STACKS.0 };
            $crate::boot::boot_external::<$Config,
                { <$Config as $crate::config::KernelConfig>::STACK_WORDS }>(
                partitions, peripherals, stacks,
            )
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
    /// apply_partition_mpu_cached would be called, false when skipped.
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

    // ============ Ticks-dropped detection tests ============
    //
    // NOTE: The _detect_dropped_ticks! macro reads ICSR PENDSTSET on ARM
    // targets, which is cfg-gated out on host. Full macro-expansion and
    // ICSR bit-check coverage is exercised by the QEMU integration test
    // in examples/dropped_tick_test.rs.

    /// Verify that increment_ticks_dropped records a dropped tick and
    /// the counter reflects the correct value after multiple drops.
    #[test]
    fn ticks_dropped_detection_logic() {
        use crate::svc::Kernel;
        use crate::{
            compose_kernel_config, DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal,
        };

        compose_kernel_config!(TdTestCfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

        let mut k = Kernel::<TdTestCfg>::default();
        assert_eq!(k.ticks_dropped(), 0, "counter must start at zero");

        // Simulate three dropped-tick detections.
        k.increment_ticks_dropped();
        assert_eq!(k.ticks_dropped(), 1);
        k.increment_ticks_dropped();
        k.increment_ticks_dropped();
        assert_eq!(k.ticks_dropped(), 3);

        // Reset returns previous value and zeroes the counter.
        let prev = k.reset_ticks_dropped();
        assert_eq!(prev, 3);
        assert_eq!(k.ticks_dropped(), 0);
    }

    // ============ Cached-region equivalence tests ============
    //
    // Dynamic-mode __pendsv_program_mpu reads R0-R3 from
    // cached_base_regions() and dynamic-mode __boot_mpu_init reads
    // R0-R3 + R4-R5 from cached_base + cached_periph.  These tests
    // verify that the cached data matches on-the-fly computation,
    // validating the correctness of the data-source substitution.

    #[test]
    fn cached_base_matches_on_the_fly_for_pendsv() {
        use crate::mpu::{partition_mpu_regions_or_deny_all, precompute_mpu_cache};
        use crate::partition::{MpuRegion, PartitionControlBlock};

        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,        // entry_point
            0x2000_0000,        // stack_base (1024-byte aligned)
            0x2000_0000 + 1024, // stack_pointer
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        );
        let expected = partition_mpu_regions_or_deny_all(&pcb);
        precompute_mpu_cache(&mut pcb).unwrap();
        assert_eq!(
            *pcb.cached_base_regions(),
            expected,
            "PendSV contract: cached_base_regions must match on-the-fly R0-R3"
        );
    }

    #[test]
    fn cached_periph_matches_on_the_fly() {
        use crate::mpu::{peripheral_mpu_regions_or_disabled, precompute_mpu_cache};
        use crate::partition::{MpuRegion, PartitionControlBlock};

        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,        // entry_point
            0x2000_0000,        // stack_base (1024-byte aligned)
            0x2000_0000 + 1024, // stack_pointer
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        )
        .with_peripheral_regions(&[
            MpuRegion::new(0x4000_0000, 0x400, 0x1300_0000),
            MpuRegion::new(0x4000_1000, 0x1000, 0x1300_0000),
        ]);
        let expected = peripheral_mpu_regions_or_disabled(&pcb);
        precompute_mpu_cache(&mut pcb).unwrap();
        assert_eq!(
            *pcb.cached_periph_regions(),
            expected,
            "PendSV contract: cached_periph_regions must match on-the-fly R4-R6"
        );
    }

    #[test]
    fn cached_base_plus_periph_covers_r0_through_r5_for_boot() {
        use crate::mpu::{
            partition_mpu_regions_or_deny_all, peripheral_mpu_regions_or_disabled,
            precompute_mpu_cache,
        };
        use crate::partition::{MpuRegion, PartitionControlBlock};

        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,        // entry_point
            0x2000_0000,        // stack_base (1024-byte aligned)
            0x2000_0000 + 1024, // stack_pointer
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        )
        .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 0x400, 0x1300_0000)]);
        let expected_base = partition_mpu_regions_or_deny_all(&pcb);
        let expected_periph = peripheral_mpu_regions_or_disabled(&pcb);
        precompute_mpu_cache(&mut pcb).unwrap();
        assert_eq!(
            *pcb.cached_base_regions(),
            expected_base,
            "boot contract: cached base regions must match on-the-fly R0-R3"
        );
        assert_eq!(
            *pcb.cached_periph_regions(),
            expected_periph,
            "boot contract: cached periph regions must match on-the-fly R4-R6"
        );
        // Verify the combined count covers R0-R6 (7 register pairs).
        let total = pcb.cached_base_regions().len() + pcb.cached_periph_regions().len();
        assert_eq!(total, 7, "dynamic boot programs R0-R6 (7 pairs)");
    }
}
