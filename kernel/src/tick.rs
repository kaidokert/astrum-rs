//! SysTick-driven scheduler tick and PendSV trigger.
//!
//! This module provides the [`TickCounter`] for monotonic tick counting and
//! [`configure_systick`] for SysTick timer configuration. The schedule
//! advancement is now handled directly via [`Kernel::advance_schedule_tick`](crate::svc::Kernel::advance_schedule_tick).

/// Describes the trace action to take on a context switch.
///
/// Pure decision logic, separated from actual `rtos_trace` calls for testability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg(any(feature = "trace", test))]
pub(crate) enum TraceAction {
    /// Same partition continues — no trace events needed.
    None,
    /// First partition starting (no outgoing) — emit `task_exec_begin` only.
    BeginOnly(u32),
    /// Partition switch — emit `task_exec_end` then `task_exec_begin(id)`.
    Switch(u32),
}

/// Determine the trace action for a context switch.
///
/// Returns `None` when `prev_active == Some(incoming)` to suppress redundant
/// events when the same partition is scheduled in consecutive slots.
#[cfg(any(feature = "trace", test))]
pub(crate) fn context_switch_trace_action(prev_active: Option<u8>, incoming: u8) -> TraceAction {
    match prev_active {
        Some(prev) if prev == incoming => TraceAction::None,
        Some(_) => TraceAction::Switch(incoming as u32),
        None => TraceAction::BeginOnly(incoming as u32),
    }
}

/// Execute the trace action by calling `rtos_trace::trace` functions.
#[cfg(feature = "trace")]
pub(crate) fn execute_trace_action(action: TraceAction) {
    match action {
        TraceAction::None => {}
        TraceAction::BeginOnly(id) => {
            rtos_trace::trace::task_exec_begin(id);
        }
        TraceAction::Switch(id) => {
            rtos_trace::trace::task_exec_end();
            rtos_trace::trace::task_exec_begin(id);
        }
    }
}

/// Describes what idle action to take based on scheduler and fault state.
///
/// Pure decision logic, separated from side effects for testability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg(test)]
pub(crate) enum IdleAction {
    /// No idle — at least one partition is runnable and scheduled.
    None,
    /// Scheduler returned Idle — emit `system_idle` trace only.
    Idle,
    /// All runnable partitions faulted — emit `system_idle` + `isr_exit`, then
    /// diverge into safe idle loop.
    FaultedIdle,
}

/// Determine the idle action for the current tick.
///
/// Returns `FaultedIdle` when all partitions have faulted (terminal),
/// `Idle` when the scheduler has no work this tick, or `None` otherwise.
#[cfg(test)]
pub(crate) fn determine_idle_action(all_faulted: bool, schedule_idle: bool) -> IdleAction {
    if all_faulted {
        IdleAction::FaultedIdle
    } else if schedule_idle {
        IdleAction::Idle
    } else {
        IdleAction::None
    }
}

/// Check idle conditions and emit trace + enter safe idle as needed.
///
/// Consolidates the idle-path logic shared by both `systick_handler` variants.
/// - `all_faulted`: all runnable partitions have faulted — enter safe idle loop.
/// - `schedule_idle`: scheduler returned `Idle` (no partition scheduled this tick).
///
/// When `all_faulted` is true and the system will diverge into `enter_safe_idle()`,
/// this function emits `isr_exit_to_scheduler()` first to close the ISR trace span
/// opened by `handle_systick`.
#[inline]
fn enter_idle_if_needed(all_faulted: bool, schedule_idle: bool) {
    if all_faulted || schedule_idle {
        #[cfg(feature = "trace")]
        rtos_trace::trace::system_idle();
    }
    if all_faulted {
        // Close the ISR trace span before diverging — enter_safe_idle() is `-> !`
        // so the isr_exit_to_scheduler() in handle_systick would never be reached.
        #[cfg(feature = "trace")]
        rtos_trace::trace::isr_exit_to_scheduler();
        crate::enter_safe_idle();
    }
}

/// Trait defining tick counter operations.
///
/// This trait is used as a bound on `CoreOps::TickCounter` so that
/// generic code can call `get()`, `increment()`, and `sync()` methods.
pub trait TickCounterOps {
    /// Return the current tick count.
    fn get(&self) -> u64;
    /// Increment the counter by one tick.
    fn increment(&mut self);
    /// Synchronize the tick counter to the given value.
    fn sync(&mut self, value: u64);
}

/// Monotonic tick counter incremented on every SysTick interrupt.
pub struct TickCounter {
    ticks: u64,
    /// Previous tick value seen by `assert_monotonic`, for backward-jump detection.
    #[cfg(any(debug_assertions, test))]
    prev_check_tick: u64,
}

impl Default for TickCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl TickCounter {
    /// Create a new counter starting at zero.
    pub const fn new() -> Self {
        Self {
            ticks: 0,
            #[cfg(any(debug_assertions, test))]
            prev_check_tick: 0,
        }
    }

    /// Increment the counter by one tick.
    pub fn increment(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
    }

    /// Return the current tick count.
    pub fn get(&self) -> u64 {
        self.ticks
    }

    /// Synchronize the tick counter to the given value.
    pub fn sync(&mut self, value: u64) {
        self.ticks = value;
    }

    /// Panic if the tick counter has gone backwards since the last check.
    #[cfg(any(debug_assertions, test))]
    pub fn assert_monotonic(&mut self) {
        assert!(
            self.ticks >= self.prev_check_tick,
            "tick counter went backwards: {} < {}",
            self.ticks,
            self.prev_check_tick,
        );
        self.prev_check_tick = self.ticks;
    }

    /// Release-build no-op.
    #[cfg(not(any(debug_assertions, test)))]
    pub fn assert_monotonic(&mut self) {}
}

impl TickCounterOps for TickCounter {
    fn get(&self) -> u64 {
        self.ticks
    }

    fn increment(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
    }

    fn sync(&mut self, value: u64) {
        self.ticks = value;
    }
}

/// Drain partition debug output at tick boundary.
///
/// This helper consolidates the debug draining logic to avoid duplication
/// across the two `systick_handler` variants.
// TODO: Consider deferring debug draining to a lower-priority context (e.g., PendSV
// or idle task) to reduce jitter in the SysTick ISR. Currently this runs on every
// tick which may add overhead in timing-critical scenarios.
#[cfg(all(feature = "partition-debug", not(feature = "dynamic-mpu")))]
#[inline]
fn drain_debug_at_tick<'mem, C: crate::config::KernelConfig>(
    kernel: &mut crate::svc::Kernel<'mem, C>,
) where
    [(); C::N]:,
    [(); C::SCHED]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    let mut ctx = crate::partition_debug::DrainContext::new();
    kernel.drain_debug_pending(&mut ctx, C::DEBUG_BUFFER_SIZE);
}

/// Drain partition debug output at tick boundary (dynamic-mpu variant).
// TODO: Consider deferring debug draining to a lower-priority context (e.g., PendSV
// or idle task) to reduce jitter in the SysTick ISR. Currently this runs on every
// tick which may add overhead in timing-critical scenarios.
#[cfg(all(feature = "partition-debug", feature = "dynamic-mpu"))]
#[inline]
fn drain_debug_at_tick<'mem, C: crate::config::KernelConfig>(
    kernel: &mut crate::svc::Kernel<'mem, C>,
) where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    let mut ctx = crate::partition_debug::DrainContext::new();
    kernel.drain_debug_pending(&mut ctx, C::DEBUG_BUFFER_SIZE);
}

/// Configure the SysTick timer with the given reload value.
#[cfg(not(test))]
pub fn configure_systick(syst: &mut cortex_m::peripheral::SYST, reload: u32) {
    use cortex_m::peripheral::syst::SystClkSource;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(reload);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

// Note: Both systick_handler variants now have the same signature (just `&mut Kernel<'mem, C>`).
// The dynamic-mpu version uses kernel.dynamic_strategy internally, so callers don't need
// to pass a separate strategy reference.

/// SysTick handler: advance schedule, trigger PendSV, expire timed waits.
///
/// This function is called by the `define_unified_harness!` macro's SysTick
/// exception handler. It advances the schedule, triggers PendSV on partition
/// switches, and expires timed waits for blocking syscalls.
///
/// Takes `&mut Kernel<'mem, C>` to allow callers to compose this with other operations
/// (e.g., user hooks) within a single critical section, preserving atomicity.
#[cfg(not(feature = "dynamic-mpu"))]
pub fn systick_handler<'mem, C: crate::config::KernelConfig>(
    kernel: &mut crate::svc::Kernel<'mem, C>,
) where
    [(); C::N]:,
    [(); C::SCHED]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    use crate::scheduler::ScheduleEvent;

    let prev_next = kernel.next_partition();
    let prev_active = kernel.active_partition;
    let event = crate::svc::scheduler::advance_schedule_tick(kernel);
    if let ScheduleEvent::PartitionSwitch(pid) = event {
        if kernel.partition_sp().get(pid as usize)
            != Some(&crate::partition_core::SP_SENTINEL_FAULT)
        {
            #[cfg(feature = "trace")]
            execute_trace_action(context_switch_trace_action(prev_active, pid));
            #[cfg(not(test))]
            cortex_m::peripheral::SCB::set_pendsv();
        } else {
            // Revert: faulted partition must not stay Running
            crate::svc::try_transition(
                kernel.partitions_mut(),
                pid,
                crate::partition::PartitionState::Ready,
            );
            kernel.active_partition = prev_active;
            kernel.set_next_partition(prev_next);
        }
    }

    enter_idle_if_needed(
        kernel.all_runnable_faulted(),
        matches!(event, ScheduleEvent::Idle),
    );

    let current_tick = kernel.tick().get();
    kernel.expire_timed_waits::<{ C::N }>(current_tick);

    #[cfg(feature = "partition-debug")]
    drain_debug_at_tick::<C>(kernel);
}

/// SysTick handler: advance schedule, trigger PendSV, expire timed waits.
///
/// This function is called by the `define_unified_harness!` macro's SysTick
/// exception handler. It advances the schedule, triggers PendSV on partition
/// switches, and expires timed waits for blocking syscalls.
///
/// With `dynamic-mpu`, also handles system window processing (bottom-half for
/// UART transfers, buffer expiry, etc.).
///
/// Takes `&mut Kernel<'mem, C>` to allow callers to compose this with other operations
/// (e.g., user hooks) within a single critical section, preserving atomicity.
#[cfg(feature = "dynamic-mpu")]
pub fn systick_handler<'mem, C: crate::config::KernelConfig>(
    kernel: &mut crate::svc::Kernel<'mem, C>,
) where
    [(); C::N]:,
    [(); C::SCHED]:,
    [(); C::BP]:,
    [(); C::BZ]:,
    [(); C::DR]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    use crate::partition::PartitionState;
    use crate::scheduler::ScheduleEvent;

    let prev_active = kernel.active_partition;
    let event = crate::svc::scheduler::advance_schedule_tick(kernel);
    let current_tick = kernel.tick().get();
    match event {
        ScheduleEvent::PartitionSwitch(pid) => {
            #[cfg(feature = "trace")]
            execute_trace_action(context_switch_trace_action(prev_active, pid));
            kernel.set_next_partition(pid);
            #[cfg(not(test))]
            cortex_m::peripheral::SCB::set_pendsv();
        }
        ScheduleEvent::SystemWindow => {
            let bh = crate::run_bottom_half!(kernel, current_tick, &kernel.dynamic_strategy);
            let has_rx = match bh {
                Ok(b) => b.has_rx_data,
                Err(e) => {
                    crate::klog!("BUG: {}", e);
                    false
                }
            };
            if has_rx {
                if let Some(woken) = kernel.dev_wait_queue.wake_one_reader() {
                    // TODO: partitions_mut() required here — try_transition needs the full table, not a single pcb_mut()
                    crate::svc::try_transition(
                        kernel.partitions_mut(),
                        woken,
                        PartitionState::Ready,
                    );
                }
            }
        }
        ScheduleEvent::Idle | ScheduleEvent::None => {}
    }

    enter_idle_if_needed(
        kernel.all_runnable_faulted(),
        matches!(event, ScheduleEvent::Idle),
    );

    // NOTE: already gated — this function is #[cfg(feature = "dynamic-mpu")]
    kernel.fallback_revoke_expired_buffers();
    kernel.expire_timed_waits::<{ C::N }>(current_tick);

    #[cfg(feature = "partition-debug")]
    drain_debug_at_tick::<C>(kernel);
}

/// RAII guard that clears `in_bottom_half` on drop.
///
/// Ensures the flag is cleared even if `run_bottom_half` panics or returns early.
#[cfg(feature = "dynamic-mpu")]
pub struct BottomHalfGuard<'a> {
    /// Reference to the flag to clear on drop. Public for macro use only.
    #[doc(hidden)]
    pub flag: &'a mut bool,
}

#[cfg(feature = "dynamic-mpu")]
impl Drop for BottomHalfGuard<'_> {
    fn drop(&mut self) {
        *self.flag = false;
    }
}

/// Wraps `run_bottom_half` with guard flag. Returns `Err` on nested calls.
#[cfg(feature = "dynamic-mpu")]
#[macro_export]
macro_rules! run_bottom_half {
    ($kernel:expr, $current_tick:expr, $strategy:expr) => {{
        if $kernel.in_bottom_half {
            Err("nested bottom-half invocation detected")
        } else {
            $kernel.in_bottom_half = true;
            let _guard = $crate::tick::BottomHalfGuard {
                flag: &mut $kernel.in_bottom_half,
            };
            Ok($crate::tick::run_bottom_half(
                &mut $kernel.uart_pair,
                &mut $kernel.isr_ring,
                &mut $kernel.buffers,
                &mut $kernel.hw_uart,
                $current_tick,
                $strategy,
            ))
        }
    }};
}

/// Result of bottom-half processing.
#[cfg(feature = "dynamic-mpu")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BottomHalfResult {
    /// Bytes transferred UART-A TX → UART-B RX.
    pub a_to_b: usize,
    /// Bytes transferred UART-B TX → UART-A RX.
    pub b_to_a: usize,
    /// `true` when at least one device has RX data and a reader should be woken.
    pub has_rx_data: bool,
}

/// Bottom-half processing for system window ticks.
///
/// **Do not call this function directly.** Use the [`run_bottom_half!`] macro,
/// which enforces the single-shot invariant via the `in_bottom_half` guard flag.
///
/// # Safety
///
/// ## Schedule-Dependency Invariant
///
/// **This function must not call any scheduler functions that yield, deschedule,
/// or switch partitions.** During a system window, no partition is running —
/// calling such functions would corrupt scheduler state or cause PendSV to
/// fire with no partition context to save/restore.
///
/// ## Forbidden Functions (must not be called from bottom-half context)
///
/// - `Kernel::trigger_deschedule()` — sets `yield_requested` and pends PendSV
/// - `handle_yield()` — pends PendSV for partition context switch
/// - `Kernel::yield_current_slot()` — advances schedule and triggers PendSV
/// - `SCB::set_pendsv()` — directly pends PendSV exception
/// - Any function that transitions a partition to `Running` state
/// - Any function that expects `CURRENT_PCB` to point to a valid partition
///
/// ## Why This Invariant Exists
///
/// The system window is a kernel-only time slot with no associated partition.
/// PendSV assumes `CURRENT_PCB` points to the currently running partition
/// whose context must be saved. During a system window:
///
/// 1. `CURRENT_PCB` may be null or point to the previous partition (already saved)
/// 2. No partition stack is active — PSP is undefined or stale
/// 3. The scheduler expects the system window to complete without state changes
///
/// Violating this invariant causes undefined behavior: corrupted partition
/// stacks, lost register state, or kernel panic.
///
/// ## Permitted Operations
///
/// - UART data transfer between ring buffers
/// - ISR ring buffer draining (routes bytes to device backends)
/// - Hardware UART TX draining
/// - Buffer pool expiry enforcement (revokes MPU windows)
///
/// Note: Partition waking (`Waiting` → `Ready`) is performed by the caller
/// (`systick_handler`) based on the returned `BottomHalfResult`, not by this
/// function.
#[cfg(feature = "dynamic-mpu")]
pub fn run_bottom_half<const D: usize, const M: usize, const BP: usize, const BZ: usize>(
    uart_pair: &mut crate::virtual_uart::VirtualUartPair,
    isr_ring: &mut crate::split_isr::IsrRingBuffer<D, M>,
    buffers: &mut crate::buffer_pool::BufferPool<BP, BZ>,
    hw_uart: &mut Option<crate::hw_uart::HwUartBackend>,
    current_tick: u64,
    strategy: &dyn crate::mpu_strategy::MpuStrategy,
) -> BottomHalfResult {
    use crate::virtual_device::VirtualDevice;
    let (a_to_b, b_to_a) = uart_pair.transfer();
    let a_id = uart_pair.a.device_id();
    let b_id = uart_pair.b.device_id();
    while isr_ring.pop_with(|tag, data| {
        if tag == a_id {
            uart_pair.a.push_rx(data);
        } else if tag == b_id {
            uart_pair.b.push_rx(data);
        } else if let Some(hw) = hw_uart.as_mut() {
            if hw.device_id() == tag {
                hw.push_rx_from_isr(data);
            }
        }
    }) {}
    if let Some(hw) = hw_uart.as_mut() {
        hw.drain_tx_to_hw();
    }
    buffers.revoke_expired(current_tick, strategy);

    let has_rx_data = uart_pair.a.rx_len() > 0
        || uart_pair.b.rx_len() > 0
        || hw_uart.as_ref().is_some_and(|hw| hw.rx_len() > 0);

    BottomHalfResult {
        a_to_b,
        b_to_a,
        has_rx_data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_counter_starts_at_zero() {
        let tc = TickCounter::new();
        assert_eq!(tc.get(), 0);
    }

    #[test]
    fn tick_counter_increments() {
        let mut tc = TickCounter::new();
        tc.increment();
        assert_eq!(tc.get(), 1);
        tc.increment();
        tc.increment();
        assert_eq!(tc.get(), 3);
    }

    #[test]
    fn tick_counter_wraps_at_max() {
        let mut tc = TickCounter {
            ticks: u64::MAX,
            prev_check_tick: 0,
        };
        tc.increment();
        assert_eq!(tc.get(), 0);
    }

    #[test]
    fn tick_counter_sync_sets_value() {
        let mut tc = TickCounter::new();
        tc.sync(100);
        assert_eq!(tc.get(), 100);
        tc.sync(0);
        assert_eq!(tc.get(), 0);
        tc.sync(u64::MAX);
        assert_eq!(tc.get(), u64::MAX);
    }

    #[test]
    fn assert_monotonic_passes_at_construction() {
        let mut tc = TickCounter::new();
        tc.assert_monotonic(); // both ticks and prev_check_tick are 0
        assert_eq!(tc.prev_check_tick, 0);
    }

    #[test]
    fn assert_monotonic_passes_on_normal_increments() {
        let mut tc = TickCounter::new();
        tc.increment();
        tc.assert_monotonic();
        assert_eq!(tc.prev_check_tick, 1);
        tc.increment();
        tc.increment();
        tc.assert_monotonic();
        assert_eq!(tc.prev_check_tick, 3);
    }

    #[test]
    fn assert_monotonic_passes_on_equal_tick() {
        let mut tc = TickCounter::new();
        tc.increment();
        tc.assert_monotonic();
        // Call again without increment — ticks == prev_check_tick, should pass
        tc.assert_monotonic();
        assert_eq!(tc.prev_check_tick, 1);
    }

    #[test]
    #[should_panic(expected = "tick counter went backwards")]
    fn assert_monotonic_panics_on_backward_sync() {
        let mut tc = TickCounter::new();
        tc.sync(100);
        tc.assert_monotonic();
        // Sync backwards
        tc.sync(50);
        tc.assert_monotonic(); // should panic
    }

    #[test]
    fn assert_monotonic_default_starts_at_zero() {
        let mut tc = TickCounter::default();
        assert_eq!(tc.prev_check_tick, 0);
        tc.assert_monotonic();
        assert_eq!(tc.prev_check_tick, 0);
    }

    // -------------------------------------------------------------------------
    // Context-switch trace action tests
    // -------------------------------------------------------------------------

    #[test]
    fn trace_action_same_partition_returns_none() {
        assert_eq!(
            context_switch_trace_action(Some(0), 0),
            TraceAction::None,
            "same partition must suppress redundant events"
        );
        assert_eq!(context_switch_trace_action(Some(3), 3), TraceAction::None,);
    }

    #[test]
    fn trace_action_different_partition_returns_switch() {
        assert_eq!(
            context_switch_trace_action(Some(0), 1),
            TraceAction::Switch(1),
        );
        assert_eq!(
            context_switch_trace_action(Some(2), 0),
            TraceAction::Switch(0),
        );
    }

    #[test]
    fn trace_action_no_previous_returns_begin_only() {
        assert_eq!(
            context_switch_trace_action(None, 0),
            TraceAction::BeginOnly(0),
        );
        assert_eq!(
            context_switch_trace_action(None, 2),
            TraceAction::BeginOnly(2),
        );
    }

    #[test]
    fn trace_action_id_maps_to_u32() {
        // Verify partition u8 ID is correctly widened to u32 task ID
        let action = context_switch_trace_action(Some(0), 255);
        assert_eq!(action, TraceAction::Switch(255));

        let action = context_switch_trace_action(None, 127);
        assert_eq!(action, TraceAction::BeginOnly(127));
    }

    /// Simulate a round-robin schedule [P0(2), P1(2)] and verify
    /// the sequence of trace actions matches actual partition transitions.
    #[test]
    fn trace_action_sequence_matches_partition_transitions() {
        // Simulate: start with no active, then P0 gets scheduled,
        // then P0 continues (same slot), then P1 gets scheduled.
        let mut active: Option<u8> = None;

        // First switch: None -> P0
        let action = context_switch_trace_action(active, 0);
        assert_eq!(action, TraceAction::BeginOnly(0));
        active = Some(0);

        // Same partition continues (e.g., interior tick re-schedules P0)
        let action = context_switch_trace_action(active, 0);
        assert_eq!(
            action,
            TraceAction::None,
            "no redundant events for same partition"
        );

        // Switch to P1
        let action = context_switch_trace_action(active, 1);
        assert_eq!(action, TraceAction::Switch(1));
        active = Some(1);

        // Switch back to P0
        let action = context_switch_trace_action(active, 0);
        assert_eq!(action, TraceAction::Switch(0));
        active = Some(0);
        let _ = active; // suppress unused warning
    }

    // -------------------------------------------------------------------------
    // Idle action decision tests
    // -------------------------------------------------------------------------

    #[test]
    fn idle_action_normal_returns_none() {
        assert_eq!(
            determine_idle_action(false, false),
            IdleAction::None,
            "normal tick with runnable partitions should produce no idle action"
        );
    }

    #[test]
    fn idle_action_schedule_idle_returns_idle() {
        assert_eq!(
            determine_idle_action(false, true),
            IdleAction::Idle,
            "scheduler idle with healthy partitions should emit system_idle only"
        );
    }

    #[test]
    fn idle_action_all_faulted_returns_faulted_idle() {
        assert_eq!(
            determine_idle_action(true, false),
            IdleAction::FaultedIdle,
            "all-faulted must produce FaultedIdle (includes isr_exit before diverging)"
        );
    }

    #[test]
    fn idle_action_all_faulted_and_idle_returns_faulted_idle() {
        // When both flags are true, the faulted path takes priority
        assert_eq!(
            determine_idle_action(true, true),
            IdleAction::FaultedIdle,
            "faulted takes priority over schedule idle"
        );
    }

    /// Verify that the FaultedIdle action is distinct from Idle —
    /// FaultedIdle requires isr_exit_to_scheduler before diverging to
    /// close the ISR trace span.
    #[test]
    fn idle_action_faulted_idle_distinct_from_idle() {
        assert_ne!(
            IdleAction::FaultedIdle,
            IdleAction::Idle,
            "FaultedIdle and Idle must be distinct variants"
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    mod dynamic_tests {
        use crate::buffer_pool::BufferPool;
        use crate::mpu_strategy::DynamicStrategy;
        use crate::split_isr::IsrRingBuffer;
        use crate::virtual_uart::VirtualUartPair;

        #[test]
        fn run_bottom_half_transfers_uart_data() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            pair.a.push_tx(&[0xAA, 0xBB]);
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                0,
                &ds,
            );
            assert_eq!(bh.a_to_b, 2);
            assert_eq!(bh.b_to_a, 0);

            let mut buf = [0u8; 4];
            assert_eq!(pair.b.pop_rx(&mut buf), 2);
            assert_eq!(&buf[..2], &[0xAA, 0xBB]);
        }

        #[test]
        fn run_bottom_half_handles_empty_inputs() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                0,
                &ds,
            );
            assert_eq!((bh.a_to_b, bh.b_to_a), (0, 0));
            assert!(!bh.has_rx_data);
        }

        #[test]
        fn run_bottom_half_has_rx_data_set_when_data_arrives() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            pair.a.push_tx(&[0x42]);
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                10,
                &ds,
            );
            assert!(bh.has_rx_data);
        }

        #[test]
        fn run_bottom_half_revokes_expired_buffers() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            buffers.lend_to_partition(0, 1, false, &ds).unwrap();
            buffers.set_deadline(0, Some(50)).unwrap();
            buffers.lend_to_partition(1, 2, true, &ds).unwrap();
            buffers.set_deadline(1, Some(200)).unwrap();

            crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                100,
                &ds,
            );

            assert_eq!(
                buffers.get(0).unwrap().state(),
                crate::buffer_pool::BorrowState::Free,
            );
            assert_eq!(buffers.deadline(0), None);

            assert_eq!(
                buffers.get(1).unwrap().state(),
                crate::buffer_pool::BorrowState::BorrowedWrite { owner: 2 },
            );
            assert_eq!(buffers.deadline(1), Some(200));
        }

        // -------------------------------------------------------------------------
        // Bottom-half recursion detection tests
        //
        // These tests verify the guard mechanism that prevents nested bottom-half
        // invocations. The invariant being protected:
        //
        //   **Only one bottom-half invocation may be active at any time.**
        //
        // Why this matters for a hard-realtime RTOS:
        // - Bottom-half processing runs in the kernel's system window (privileged)
        // - Re-entrancy could corrupt UART state, buffer pools, or MPU config
        // - A nested call would indicate a scheduler bug (processing SystemWindow
        //   while already in a system window) or an illegal interrupt nesting
        // - The guard provides a fail-fast defense: panic immediately rather than
        //   corrupt kernel state silently
        //
        // The `in_bottom_half` flag is set before entering bottom-half processing
        // and cleared by an RAII guard on exit. Any attempt to re-enter while the
        // flag is set triggers a panic.
        // -------------------------------------------------------------------------

        /// Verifies that `BottomHalfGuard` clears the flag when dropped normally.
        #[test]
        fn bottom_half_guard_clears_flag_on_drop() {
            use crate::tick::BottomHalfGuard;

            let mut flag = true;
            {
                let _guard = BottomHalfGuard { flag: &mut flag };
                // Flag remains borrowed while guard is alive; we verify state after drop
            }
            assert!(!flag, "flag should be cleared after guard is dropped");
        }

        /// Mock kernel structure for testing recursion detection.
        ///
        /// Simulates the minimal kernel state needed to test the `run_bottom_half!`
        /// macro's guard behavior without requiring the full `Kernel<'mem, C>` type.
        struct MockKernelForRecursion {
            in_bottom_half: bool,
            uart_pair: crate::virtual_uart::VirtualUartPair,
            isr_ring: crate::split_isr::IsrRingBuffer<4, 8>,
            buffers: crate::buffer_pool::BufferPool<4, 32>,
            hw_uart: Option<crate::hw_uart::HwUartBackend>,
            dynamic_strategy: DynamicStrategy,
        }

        impl MockKernelForRecursion {
            fn new() -> Self {
                Self {
                    in_bottom_half: false,
                    uart_pair: crate::virtual_uart::VirtualUartPair::new(0, 1),
                    isr_ring: crate::split_isr::IsrRingBuffer::new(),
                    buffers: crate::buffer_pool::BufferPool::new(),
                    hw_uart: None,
                    dynamic_strategy: DynamicStrategy::new(),
                }
            }
        }

        /// Tests that a single bottom-half invocation works correctly:
        /// flag is set during execution and cleared after.
        #[test]
        fn bottom_half_single_invocation_succeeds() {
            let mut mock = MockKernelForRecursion::new();
            assert!(!mock.in_bottom_half, "flag should start false");

            let result = crate::run_bottom_half!(mock, 0, &mock.dynamic_strategy)
                .expect("single invocation should succeed");

            assert!(
                !mock.in_bottom_half,
                "flag should be cleared after invocation"
            );
            // Verify the function actually ran (empty inputs = zero transfers)
            assert_eq!(result.a_to_b, 0);
            assert_eq!(result.b_to_a, 0);
        }

        /// Tests that nested bottom-half invocation returns Err instead of panicking.
        #[test]
        fn bottom_half_nested_invocation_returns_err() {
            let mut mock = MockKernelForRecursion::new();
            // Simulate being already inside bottom-half processing
            mock.in_bottom_half = true;

            let result = crate::run_bottom_half!(mock, 0, &mock.dynamic_strategy);
            assert_eq!(
                result.unwrap_err(),
                "nested bottom-half invocation detected"
            );
            // Flag should remain set (guard was never created)
            assert!(mock.in_bottom_half, "flag should remain set on nested call");
        }
    }
}
