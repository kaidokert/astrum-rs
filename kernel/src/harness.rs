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
//! | Kernel state | Stack-local kernel published via `store_kernel_ptr()` |
//! | `static _SVC` | Forces linker to include the SVC assembly trampoline |
//! | `SysTick` exception handler | Drives the round-robin scheduler |
//! | `PendSV` handler | Via [`define_pendsv!`] |
//! | `MemoryManagement` handler | Via [`define_memmanage_handler!`] |
//! | `boot()` | Safe function: inits stacks, configures exceptions, starts OS |
//!
//! Note: Per-partition stacks are stored in [`PartitionCore`] within the `Kernel`
//! struct, not as a separate static array.
//!
//! After invoking the macro, call the generated `boot()` from `main()`
//! to perform the common startup sequence (partition stack init,
//! exception priorities, SysTick configuration, and first PendSV
//! trigger). Returns `Result<Never, BootError>` for panic-free init.

// TODO: 'mem lifetime propagation — harness macros reference Kernel<'mem, C> indirectly
// via $crate::svc::Kernel::<$Config>::new() and $crate::state::with_kernel_mut(), which
// handle lifetimes internally. No macro-level lifetime changes needed.

// Re-export BootError, Never, and init_rtt from boot module for macro access.
// The canonical definitions live in boot.rs.
pub use crate::boot::{boot_banner, init_fpu, init_rtt, BootError, Never};

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

/// Declare aligned partition stacks for `no_boot` examples.
///
/// Returns `&mut [AlignedStack1K; $count]` backed by a module-level static.
///
/// # Safety
///
/// Must be called exactly once, from `main()`, before any interrupt runs.
/// The returned mutable reference is valid for `'static` but must not alias.
///
/// # Example
///
/// ```ignore
/// let stacks = kernel::partition_stacks!(TestConfig, NP);
/// ```
#[macro_export]
macro_rules! partition_stacks {
    ($Config:ty, $count:expr) => {{
        static mut _PARTITION_STACKS: [$crate::AlignedStack1K; $count] =
            [<$crate::AlignedStack1K as $crate::StackStorage>::ZERO; $count];
        // SAFETY: caller guarantees single-threaded pre-interrupt context.
        unsafe { &mut *(&raw mut _PARTITION_STACKS) }
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
///     // k: &mut Kernel<'mem, Config>
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
    // Basic form: no SysTick hook (legacy: caller must call init_kernel + boot separately)
    ($Config:ty) => {
        $crate::define_unified_harness!(@impl_compat $Config, |_tick, _k| {});
    };
    // Extended form: with SysTick hook (legacy: caller must call init_kernel + boot separately)
    ($Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@impl_compat $Config, |$tick, $k| $hook);
    };
    // no_boot form: handlers only, caller uses kernel::boot directly
    (no_boot, $Config:ty) => {
        $crate::define_unified_harness!(@handlers $Config, |_tick, _k| {});
    };
    // no_boot form with SysTick hook
    (no_boot, $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, |$tick, $k| $hook);
    };
    // Full form: boot() calls init_kernel() internally with the provided schedule and entries.
    // $sched and $entries are expressions evaluated inside boot()'s body.
    ($Config:ty, $sched:expr, $entries:expr) => {
        $crate::define_unified_harness!(@impl $Config, $sched, $entries, |_tick, _k| {});
    };
    // Full form with SysTick hook
    ($Config:ty, $sched:expr, $entries:expr, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@impl $Config, $sched, $entries, |$tick, $k| $hook);
    };
    // Internal: handlers only (SysTick, PendSV, SVC linkage, kernel state)
    (@handlers $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        #[cfg(feature = "dynamic-mpu")]
        static HARNESS_STRATEGY: $crate::mpu_strategy::DynamicStrategy<
            {<$Config as $crate::config::KernelConfig>::N}
        > = $crate::mpu_strategy::DynamicStrategy::<
            {<$Config as $crate::config::KernelConfig>::N}
        >::with_partition_count();

        /// Boot-time MPU initialisation hook called from `boot::boot_preconfigured()`
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

        /// Boot-time MPU initialisation hook called from `boot::boot_preconfigured()`
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
                    let periph_reserved = pcb.peripheral_regions().len();
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

                // Configure all remaining (non-boot) partitions so that
                // each partition's `peripheral_reserved` is set before
                // `wire_boot_peripherals` reads it.  Without this,
                // non-boot partitions with peripherals would have
                // peripheral_reserved=0 and their peripherals would be
                // wired via add_window (global slots) instead of cached
                // in per-partition slots.
                for i in 0..k.partition_count() {
                    let other_pid = u8::try_from(i)
                        .map_err(|_| "boot: partition index exceeds u8")?;
                    if other_pid == pid { continue; }
                    let other_pcb = k.pcb(i)
                        .ok_or("boot: partition missing from table")?;
                    // TODO: assumes each partition has exactly one dynamic region
                    // (cached_dynamic_region).  If the PCB supports multiple
                    // dynamic regions, this call needs updating.
                    let other_dyn = other_pcb.cached_dynamic_region();
                    let other_periph = other_pcb.peripheral_regions().len();
                    $crate::mpu_strategy::MpuStrategy::configure_partition(
                        &HARNESS_STRATEGY, other_pid, &[other_dyn], other_periph,
                    ).map_err(|_| "failed to configure non-boot partition")?;
                }

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
        static _SVC: $crate::svc::SvcDispatchFn =
            $crate::svc::SVC_HANDLER;

        /// PendSV MPU programming shim: reprogram MPU regions for the
        /// incoming partition on every context switch.
        ///
        /// - Static mode: R0-R3 (base) + R4-R5 (peripheral) via
        ///   `apply_partition_mpu_cached` (self-contained disable/enable cycle).
        /// - Dynamic mode: single disable/enable cycle writing R0-R3
        ///   (base partition) then R4-R7 from pre-configured strategy
        ///   values (configured once at boot, not per-switch).
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

                }

                // Return pid for dynamic-mode peripheral cache lookup.
                // In static mode this is unused.
                pid
            }) {
                Ok(pid) => pid,
                Err(_) => return,
            };

            // Dynamic mode: write per-partition R4-R7 strategy regions
            // (peripherals, RAM, cross-partition filtering) and re-enable.
            #[cfg(feature = "dynamic-mpu")]
            if <$Config as $crate::config::KernelConfig>::MPU_ENFORCE {
                let values = HARNESS_STRATEGY.partition_region_values(pid);
                for &(rbar, rasr) in &values {
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

        $crate::define_memmanage_handler!($Config);
    };
    // Internal: full implementation with boot() calling init_kernel() internally.
    // $sched and $entries are macro-level expressions pasted into boot()'s body.
    (@impl $Config:ty, $sched:expr, $entries:expr, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, |$tick, $k| $hook);

        /// Partition stacks backed by `AlignedStack1K`; used by `init_kernel()` to build PCBs.
        ///
        /// # Singleton
        /// This macro arm must be invoked at most once per binary.  Multiple
        /// invocations would produce duplicate `__PARTITION_STACKS` symbols,
        /// causing a compile-time link error.
        static mut __PARTITION_STACKS: [$crate::AlignedStack1K;
             <$Config as $crate::config::KernelConfig>::N] =
            [<$crate::AlignedStack1K as $crate::StackStorage>::ZERO;
             <$Config as $crate::config::KernelConfig>::N];

        // TODO: `init_kernel` is generated in the caller's local scope; acceptable
        // for a harness macro but could shadow other symbols if used carelessly.
        /// Build a kernel from `__PARTITION_STACKS` with the given
        /// [`PartitionSpec`]($crate::partition::PartitionSpec) entries and
        /// store it into global kernel state.
        ///
        /// Each entry in `entries` is a `PartitionSpec` — an
        /// `(entry_point_fn, r0_hint)` pair.  The function pointer is
        /// converted to a `u32` address and written into the corresponding
        /// partition memory descriptor.  `r0_hint` is forwarded via
        /// `with_r0_hint` so that `boot_preconfigured` can pass it to
        /// `init_stack_frame` as `r0`.  MPU regions use sentinel values
        /// `(base=0, size=0, attrs=0)` because the harness does not configure
        /// per-partition MPU regions.
        fn init_kernel(
            sched: $crate::scheduler::ScheduleTable<
                { <$Config as $crate::config::KernelConfig>::SCHED }>,
            entries: &[$crate::partition::PartitionSpec],
        ) -> Result<$crate::svc::Kernel<'static, $Config>, $crate::harness::BootError> {
            use $crate::partition::{ExternalPartitionMemory, MpuRegion};
            // SAFETY: `__PARTITION_STACKS` is a module-level static mut defined
            // by this macro arm, which is invoked at most once per binary (enforced
            // by the linker via duplicate symbol errors).  `init_kernel()` is called
            // exactly once from `boot()` before interrupts are enabled, so there are
            // no concurrent accesses.
            let stacks = unsafe { &mut *(&raw mut __PARTITION_STACKS) };
            let sentinel_mpu = MpuRegion::new(0, 0, 0);
            let mut memories: heapless::Vec<ExternalPartitionMemory<'_>,
                { <$Config as $crate::config::KernelConfig>::N }> = heapless::Vec::new();
            for (i, (stk, spec)) in stacks.iter_mut().zip(entries.iter()).enumerate() {
                let mem = ExternalPartitionMemory::from_aligned_stack(
                    stk, spec.entry_point(), sentinel_mpu, i as u8)
                    .map_err(|e| {
                        $crate::klog!("init_kernel failed: {:?}", e);
                        $crate::harness::BootError::KernelInit(e)
                    })?
                    .with_r0_hint(spec.r0());
                memories.push(mem)
                    .map_err(|_| {
                        $crate::klog!("init_kernel failed: partition table full");
                        $crate::harness::BootError::KernelInit(
                            $crate::partition::ConfigError::PartitionTableFull)
                    })?;
            }
            $crate::svc::Kernel::<$Config>::new(sched, &memories)
                .map_err(|e| {
                    $crate::klog!("init_kernel failed: {:?}", e);
                    $crate::harness::BootError::KernelInit(e)
                })
        }

        /// Boot the kernel: creates the kernel via [`init_kernel`] using the
        /// schedule and entry points provided as macro arguments, then
        /// configures exceptions and enters the idle loop via
        /// [`boot::boot_preconfigured`].
        ///
        /// The kernel lives as a local variable on this function's stack.
        /// Since `boot() -> !` never returns, the stack frame persists
        /// indefinitely and [`store_kernel_ptr`] publishes its address
        /// for ISR access.
        fn boot(
            peripherals: cortex_m::Peripherals,
        ) -> Result<$crate::harness::Never, $crate::harness::BootError> {
            // Init RTT early so klog! output from init_kernel() is visible.
            $crate::harness::init_rtt();
            $crate::harness::boot_banner();
            // Enable FPU before kernel init (no-op when fpu-context is off).
            $crate::harness::init_fpu().map_err(|e| {
                $crate::klog!("boot failed: {}", e);
                e
            })?;
            let mut k = init_kernel($sched, $entries)?;
            // SAFETY: boot() -> ! never returns, so `k` lives forever on
            // this stack frame. store_kernel publishes its address for
            // ISR handlers and installs the SVC dispatch hook.
            // TODO: reviewer flagged as unrelated — required by store_kernel signature change in dff1322
            store_kernel(k);
            unsafe {
                $crate::boot::boot_preconfigured::<$Config>(peripherals)
            }
        }
    };
    // TODO: consider renaming @impl_compat to @impl_legacy once legacy callers are migrated.
    // Internal: legacy implementation (handlers + boot function, caller calls init_kernel separately)
    (@impl_compat $Config:ty, |$tick:ident, $k:ident| $hook:block) => {
        $crate::define_unified_harness!(@handlers $Config, |$tick, $k| $hook);

        /// Partition stacks backed by `AlignedStack1K`; used by `init_kernel()` to build PCBs.
        ///
        /// # Singleton
        /// This macro arm must be invoked at most once per binary.  Multiple
        /// invocations would produce duplicate `__PARTITION_STACKS` symbols,
        /// causing a compile-time link error.
        static mut __PARTITION_STACKS: [$crate::AlignedStack1K;
             <$Config as $crate::config::KernelConfig>::N] =
            [<$crate::AlignedStack1K as $crate::StackStorage>::ZERO;
             <$Config as $crate::config::KernelConfig>::N];

        /// Build a kernel from `__PARTITION_STACKS` with the given
        /// [`PartitionSpec`]($crate::partition::PartitionSpec) entries.
        ///
        /// Returns the kernel so the caller can keep it on the stack.
        /// See the `@impl` arm's `init_kernel` for full documentation.
        #[allow(dead_code)]
        fn init_kernel(
            sched: $crate::scheduler::ScheduleTable<
                { <$Config as $crate::config::KernelConfig>::SCHED }>,
            entries: &[$crate::partition::PartitionSpec],
        ) -> Result<$crate::svc::Kernel<'static, $Config>, $crate::harness::BootError> {
            // Init RTT early so klog! output from init_kernel() is visible.
            $crate::harness::init_rtt();
            $crate::harness::boot_banner();
            // Enable FPU before kernel init (no-op when fpu-context is off).
            $crate::harness::init_fpu().map_err(|e| {
                $crate::klog!("boot failed: {}", e);
                e
            })?;
            use $crate::partition::{ExternalPartitionMemory, MpuRegion};
            // SAFETY: `__PARTITION_STACKS` is a module-level static mut defined
            // by this macro arm, which is invoked at most once per binary (enforced
            // by the linker via duplicate symbol errors).  `init_kernel()` is called
            // exactly once from `main()` before interrupts are enabled, so there are
            // no concurrent accesses.
            let stacks = unsafe { &mut *(&raw mut __PARTITION_STACKS) };
            let sentinel_mpu = MpuRegion::new(0, 0, 0);
            let mut memories: heapless::Vec<ExternalPartitionMemory<'_>,
                { <$Config as $crate::config::KernelConfig>::N }> = heapless::Vec::new();
            for (i, (stk, spec)) in stacks.iter_mut().zip(entries.iter()).enumerate() {
                let mem = ExternalPartitionMemory::from_aligned_stack(
                    stk, spec.entry_point(), sentinel_mpu, i as u8)
                    .map_err(|e| {
                        $crate::klog!("init_kernel failed: {:?}", e);
                        $crate::harness::BootError::KernelInit(e)
                    })?
                    .with_r0_hint(spec.r0());
                memories.push(mem)
                    .map_err(|_| {
                        $crate::klog!("init_kernel failed: partition table full");
                        $crate::harness::BootError::KernelInit(
                            $crate::partition::ConfigError::PartitionTableFull)
                    })?;
            }
            $crate::svc::Kernel::<$Config>::new(sched, &memories)
                .map_err(|e| {
                    $crate::klog!("init_kernel failed: {:?}", e);
                    $crate::harness::BootError::KernelInit(e)
                })
        }

        /// Boot the kernel from pre-configured PCBs.
        ///
        /// The caller must have already called [`init_kernel`] and
        /// [`store_kernel`] to place the kernel on their stack and
        /// publish its pointer.  This function configures exceptions
        /// and enters the idle loop via [`boot::boot_preconfigured`].
        fn boot(
            peripherals: cortex_m::Peripherals,
        ) -> Result<$crate::harness::Never, $crate::harness::BootError> {
            // SAFETY: caller has called init_kernel + store_kernel,
            // so the kernel is fully configured.  boot() is called
            // exactly once from main().
            unsafe {
                $crate::boot::boot_preconfigured::<$Config>(peripherals)
            }
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
        buf: [u8; 128],
        len: usize,
    }

    impl FmtBuf {
        const fn new() -> Self {
            Self {
                buf: [0u8; 128],
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
            if bytes.len() > remaining {
                return Err(core::fmt::Error);
            }
            self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
            self.len += bytes.len();
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

    #[test]
    fn boot_error_kernel_init_wraps_config_error() {
        use crate::partition::ConfigError;

        // Verify KernelInit wraps various ConfigError variants
        let ce = ConfigError::PartitionTableFull;
        let err = BootError::KernelInit(ce);
        assert_eq!(err, BootError::KernelInit(ConfigError::PartitionTableFull));
        assert_ne!(err, BootError::KernelNotInitialized);

        // Verify Display includes the inner error
        let mut buf = FmtBuf::new();
        write!(&mut buf, "{}", err).unwrap();
        assert!(
            buf.as_str().contains("kernel init failed"),
            "Display should contain 'kernel init failed', got: {}",
            buf.as_str()
        );

        // Verify with StackInitFailed variant
        let ce2 = ConfigError::StackInitFailed { partition_id: 3 };
        let err2 = BootError::KernelInit(ce2);
        let mut buf2 = FmtBuf::new();
        write!(&mut buf2, "{}", err2).unwrap();
        assert!(
            buf2.as_str().contains("kernel init failed"),
            "Display should contain 'kernel init failed', got: {}",
            buf2.as_str()
        );
        // The inner ConfigError should be distinguishable
        assert_ne!(err, err2);
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

    /// Verify that `init_rtt` is publicly accessible via `harness::init_rtt`
    /// (re-exported from `boot::init_rtt`).  Under non-RTT backends this is
    /// a no-op; the test confirms the symbol is reachable from macro context.
    #[test]
    fn init_rtt_callable_via_harness_reexport() {
        // Call through the harness re-export path that the macro would use.
        crate::harness::init_rtt();
    }

    /// Verify that calling `init_rtt` twice does not panic, exercising the
    /// AtomicBool double-call guard added to support early RTT init in the
    /// harness macro (init_rtt is called before init_kernel, then again
    /// inside boot_preconfigured).
    #[test]
    fn init_rtt_double_call_is_safe() {
        crate::harness::init_rtt();
        crate::harness::init_rtt();
    }

    /// Verify that `init_fpu` is publicly accessible via `harness::init_fpu`
    /// (re-exported from `boot::init_fpu`).
    #[test]
    #[cfg(not(feature = "fpu-context"))]
    fn init_fpu_callable_via_harness_reexport() {
        crate::harness::init_fpu().unwrap();
    }

    /// Verify that calling `init_fpu` twice does not panic via the harness
    /// re-export path.
    #[test]
    #[cfg(not(feature = "fpu-context"))]
    fn init_fpu_double_call_is_safe() {
        crate::harness::init_fpu().unwrap();
        crate::harness::init_fpu().unwrap();
    }

    // ============ Config error propagation tests ============

    /// Verify end-to-end propagation of `EntryPointNotThumb` through the
    /// harness boot path.  Mirrors `init_kernel()`'s logic (lines 552-557):
    /// `ExternalPartitionMemory::from_aligned_stack(...).map_err(|e| BootError::KernelInit(e))`
    ///
    /// An even entry-point address (missing Thumb bit[0]) must:
    /// 1. Be rejected by `ExternalPartitionMemory::new()` with `EntryPointNotThumb`.
    /// 2. Be wrapped into `BootError::KernelInit` via `.map_err` (as the harness does).
    /// 3. Produce actionable Display output that names the partition and address.
    #[test]
    fn entry_point_not_thumb_propagates_through_boot_path() {
        use crate::partition::{ConfigError, ExternalPartitionMemory, MpuRegion};
        use crate::partition_core::{AlignedStack1K, StackStorage};

        let mut stack = AlignedStack1K::default();
        let sentinel_mpu = MpuRegion::new(0, 0, 0);

        // Even address — missing Thumb bit[0].
        let even_entry = 0x0800_1234u32;

        // Replicate the harness init_kernel() pattern:
        //   ExternalPartitionMemory::from_aligned_stack(stk, spec.entry_point(), sentinel_mpu, i as u8)
        //       .map_err(|e| BootError::KernelInit(e))?
        let boot_err = ExternalPartitionMemory::new(
            stack.as_u32_slice_mut(),
            even_entry,
            sentinel_mpu,
            3u8, // partition_id
        )
        .map_err(BootError::KernelInit)
        .expect_err("even entry point must be rejected");

        // Verify the inner ConfigError is EntryPointNotThumb.
        match &boot_err {
            BootError::KernelInit(ConfigError::EntryPointNotThumb {
                partition_id,
                entry_point,
            }) => {
                assert_eq!(*partition_id, 3);
                assert_eq!(*entry_point, even_entry);
            }
            other => panic!("expected BootError::KernelInit(EntryPointNotThumb), got: {other:?}"),
        }

        // Verify Display output is actionable.
        let mut buf = FmtBuf::new();
        write!(&mut buf, "{}", boot_err).unwrap();
        let msg = buf.as_str();

        assert!(
            msg.contains("kernel init failed"),
            "outer BootError must indicate kernel init failure, got: {msg}"
        );
        assert!(
            msg.contains("partition 3"),
            "must identify the partition, got: {msg}"
        );
        assert!(
            msg.contains("0x08001234"),
            "must include the entry point address, got: {msg}"
        );
        assert!(
            msg.contains("bit[0]"),
            "must explain which bit to set, got: {msg}"
        );
    }
}
