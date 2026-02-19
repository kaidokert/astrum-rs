use crate::tick::TickCounterOps;

/// Trait for core partition/schedule sub-structs.
pub trait CoreOps {
    type PartTable;
    type SchedTable;
    type TickCounter: TickCounterOps;
    fn partitions(&self) -> &Self::PartTable;
    fn partitions_mut(&mut self) -> &mut Self::PartTable;
    fn schedule(&self) -> &Self::SchedTable;
    fn schedule_mut(&mut self) -> &mut Self::SchedTable;
    /// Returns the current partition index.
    fn current_partition(&self) -> u8;
    /// Sets the current partition index.
    fn set_current_partition(&mut self, id: u8);
    /// Returns the next partition index.
    fn next_partition(&self) -> u8;
    /// Sets the next partition index.
    fn set_next_partition(&mut self, id: u8);
    /// Gets the stack pointer for a partition by index.
    fn get_sp(&self, index: usize) -> Option<u32>;
    /// Sets the stack pointer for a partition by index. Returns true if valid.
    fn set_sp(&mut self, index: usize, sp: u32) -> bool;
    /// Returns a reference to the partition_sp array.
    fn partition_sp(&self) -> &[u32];
    /// Returns a mutable reference to the partition_sp array.
    fn partition_sp_mut(&mut self) -> &mut [u32];
    /// Returns the currently active partition index, if any.
    fn active_partition(&self) -> Option<u8>;
    /// Sets the active partition index.
    fn set_active_partition(&mut self, id: Option<u8>);
    /// Returns a reference to the tick counter.
    fn tick(&self) -> &Self::TickCounter;
    /// Returns a mutable reference to the tick counter.
    fn tick_mut(&mut self) -> &mut Self::TickCounter;
    /// Returns whether a yield has been requested.
    fn yield_requested(&self) -> bool;
    /// Sets the yield_requested flag.
    fn set_yield_requested(&mut self, requested: bool);
    /// Returns a mutable reference to a partition's stack array.
    fn stack_mut(&mut self, index: usize) -> Option<&mut [u32]>;
}

/// Trait for synchronization primitive sub-structs (semaphores, mutexes).
pub trait SyncOps {
    type SemPool;
    type MutPool;
    fn semaphores(&self) -> &Self::SemPool;
    fn semaphores_mut(&mut self) -> &mut Self::SemPool;
    fn mutexes(&self) -> &Self::MutPool;
    fn mutexes_mut(&mut self) -> &mut Self::MutPool;
}

/// Trait for message-passing primitive sub-structs (message queues, queuing ports).
pub trait MsgOps {
    type MsgPool;
    type QueuingPool;
    fn messages(&self) -> &Self::MsgPool;
    fn messages_mut(&mut self) -> &mut Self::MsgPool;
    fn queuing(&self) -> &Self::QueuingPool;
    fn queuing_mut(&mut self) -> &mut Self::QueuingPool;
}

/// Trait for port primitive sub-structs (sampling ports, blackboards).
pub trait PortsOps {
    type SamplingPool;
    type BlackboardPool;
    fn sampling(&self) -> &Self::SamplingPool;
    fn sampling_mut(&mut self) -> &mut Self::SamplingPool;
    fn blackboards(&self) -> &Self::BlackboardPool;
    fn blackboards_mut(&mut self) -> &mut Self::BlackboardPool;
}

/// Trait that bundles every const-generic parameter the [`Kernel`] needs.
///
/// Implement this trait on a zero-sized struct to configure a kernel
/// instance.  Each associated constant maps 1-to-1 to the former
/// const-generic parameter of the same name on `Kernel`.
///
/// [`Kernel`]: crate::svc::Kernel
pub trait KernelConfig {
    /// Maximum number of partitions (no default - must be specified explicitly).
    const N: usize;
    /// Schedule table capacity (number of schedule entries).
    const SCHED: usize = 4;
    /// Stack word count per partition (256 = 1024 bytes for MPU alignment).
    const STACK_WORDS: usize = 256;
    /// Semaphore pool capacity.
    const S: usize = 1;
    /// Semaphore wait-queue depth.
    const SW: usize = 1;
    /// Mutex pool capacity.
    const MS: usize = 1;
    /// Mutex wait-queue depth.
    const MW: usize = 1;
    /// Message-queue pool capacity (also used for queuing ports).
    const QS: usize = 1;
    /// Message-queue depth (messages per queue).
    const QD: usize = 1;
    /// Maximum message size in bytes (message queues & queuing ports).
    const QM: usize = 1;
    /// Message-queue / queuing-port wait-queue depth.
    const QW: usize = 1;
    /// Sampling-port pool capacity.
    const SP: usize = 1;
    /// Sampling-port maximum message size in bytes.
    const SM: usize = 64;
    /// Blackboard pool capacity.
    const BS: usize = 1;
    /// Blackboard maximum message size in bytes.
    const BM: usize = 64;
    /// Blackboard wait-queue depth.
    const BW: usize = 1;

    /// Buffer pool slot count (only used with `dynamic-mpu` feature).
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    /// Buffer slot size in bytes (only used with `dynamic-mpu` feature).
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    /// Device registry capacity (only used with `dynamic-mpu` feature).
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;
    /// Maximum acceptable gap in ticks between system windows.
    ///
    /// During kernel initialization, the schedule table is validated to ensure
    /// system windows occur frequently enough. If `max_ticks_without_system_window()`
    /// exceeds this threshold, `Kernel::new()` returns `ConfigError::SystemWindowTooInfrequent`.
    ///
    /// This ensures the kernel can service device interrupts and perform MPU
    /// region updates within a bounded time. A typical value of 100 ticks
    /// (e.g., 100ms at 1ms/tick) balances responsiveness with partition scheduling
    /// flexibility.
    ///
    /// Only used with the `dynamic-mpu` feature.
    #[cfg(feature = "dynamic-mpu")]
    const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 = 100;

    /// Debug buffer size in bytes per partition (only used with `partition-debug` feature).
    #[cfg(feature = "partition-debug")]
    const DEBUG_BUFFER_SIZE: usize = 256;

    /// SVCall exception priority (0x00 = highest on Cortex-M).
    const SVCALL_PRIORITY: u8 = 0x00;
    /// PendSV exception priority (must be the lowest of the three).
    const PENDSV_PRIORITY: u8 = 0xFF;
    /// SysTick exception priority (must be lower priority than SVCall,
    /// higher priority than PendSV; on Cortex-M a *larger* number means
    /// *lower* priority).
    const SYSTICK_PRIORITY: u8 = 0xFE;

    /// Core clock frequency in Hz.
    ///
    /// This is the frequency of the processor core clock that drives the
    /// SysTick timer. The default value of 12 MHz matches the QEMU
    /// lm3s6965evb emulation target.
    ///
    /// This constant, together with [`TICK_PERIOD_US`], determines the
    /// [`SYSTICK_CYCLES`] value (which is auto-calculated by default).
    ///
    /// [`TICK_PERIOD_US`]: Self::TICK_PERIOD_US
    /// [`SYSTICK_CYCLES`]: Self::SYSTICK_CYCLES
    const CORE_CLOCK_HZ: u32 = 12_000_000;

    /// Tick period in microseconds.
    ///
    /// This defines the desired interval between SysTick interrupts.
    /// The default value of 1000 microseconds gives a 1 ms tick period.
    ///
    /// This constant, together with [`CORE_CLOCK_HZ`], determines the
    /// [`SYSTICK_CYCLES`] value (which is auto-calculated by default).
    ///
    /// [`CORE_CLOCK_HZ`]: Self::CORE_CLOCK_HZ
    /// [`SYSTICK_CYCLES`]: Self::SYSTICK_CYCLES
    const TICK_PERIOD_US: u32 = 1000;

    /// SysTick cycle count determining the tick rate.
    ///
    /// This is the number of core clock cycles between SysTick interrupts.
    /// By default, it is automatically calculated from [`CORE_CLOCK_HZ`]
    /// and [`TICK_PERIOD_US`] using the formula:
    ///
    /// ```text
    /// SYSTICK_CYCLES = (CORE_CLOCK_HZ as u64 * TICK_PERIOD_US as u64 / 1_000_000) as u32
    /// ```
    ///
    /// The intermediate `u64` cast prevents overflow for large clock frequencies.
    /// For the defaults (12 MHz clock, 1000 µs period), this gives 12,000
    /// cycles per tick.
    ///
    /// Implementers may override this constant if a specific cycle count is
    /// required, but the default calculation should suffice for most cases.
    ///
    /// Note: The hardware reload register is set to `SYSTICK_CYCLES - 1`
    /// because SysTick counts down from the reload value to zero inclusive.
    ///
    /// [`CORE_CLOCK_HZ`]: Self::CORE_CLOCK_HZ
    /// [`TICK_PERIOD_US`]: Self::TICK_PERIOD_US
    const SYSTICK_CYCLES: u32 =
        (Self::CORE_CLOCK_HZ as u64 * Self::TICK_PERIOD_US as u64 / 1_000_000) as u32;

    /// Partition/schedule state operations. Must implement `CoreOps` to
    /// allow dispatch() and other methods to call sub-struct methods.
    type Core: Default + CoreOps;
    /// Synchronization primitive operations (semaphores, mutexes).
    /// Must implement `SyncOps` for semaphore/mutex syscall dispatch.
    type Sync: Default + SyncOps;
    /// Message-passing primitive operations (message queues, queuing ports).
    /// Must implement `MsgOps` for message syscall dispatch.
    type Msg: Default + MsgOps;
    /// Sampling ports and blackboards operations. Must implement `PortsOps`
    /// for sampling port and blackboard syscall dispatch.
    type Ports: Default + PortsOps;
}

/// Maximum value for the SysTick RELOAD register (24-bit).
pub const SYSTICK_RELOAD_MAX: u32 = 0xFF_FFFF;

/// Computes the SysTick reload value from clock frequency and tick period.
///
/// The ARM SysTick timer counts down from the reload value to zero, then
/// reloads and fires an interrupt. To achieve a period of N cycles, the
/// RELOAD register must be set to N-1. This function computes and returns
/// that N-1 value directly.
///
/// # Arguments
/// * `core_clock_hz` - Core clock frequency in Hz
/// * `tick_period_us` - Desired tick period in microseconds
///
/// # Returns
/// The reload value (N-1) to write directly to the SysTick RELOAD register,
/// where N is the number of cycles per tick period.
///
/// # Panics
/// Panics at compile time if the computed reload value exceeds the 24-bit
/// maximum (0xFFFFFF) supported by the SysTick RELOAD register, or if the
/// cycle count is zero (which would result in underflow).
///
/// # Note
/// This function uses `assert!` for validation, which is acceptable for
/// compile-time const evaluation but creates a panic path if called at
/// runtime. The kernel targets a panic-free binary; callers should only
/// use this function in const contexts.
// TODO: panic-free policy - this function panics at runtime. For runtime
// use, consider a fallible variant returning Result or Option.
///
/// # Example
/// ```ignore
/// use kernel::config::compute_systick_reload;
/// // 12 MHz clock, 1 ms tick period = 12,000 cycles, reload = 11,999
/// const RELOAD: u32 = compute_systick_reload(12_000_000, 1000);
/// assert_eq!(RELOAD, 11_999);
/// ```
pub const fn compute_systick_reload(core_clock_hz: u32, tick_period_us: u32) -> u32 {
    // Use u64 intermediate to prevent overflow for large clock/period combinations
    let cycles = (core_clock_hz as u64 * tick_period_us as u64) / 1_000_000;
    assert!(cycles > 0, "SysTick cycle count must be at least 1");
    // Subtract 1 for the RELOAD register (counts N-1 down to 0 for N cycles)
    let reload = cycles - 1;
    assert!(
        reload <= SYSTICK_RELOAD_MAX as u64,
        "SysTick reload value exceeds 24-bit maximum (0xFFFFFF)"
    );
    reload as u32
}

/// Compile-time assertion that the three exception priorities are
/// correctly ordered: SVCall < SysTick < PendSV (numerically).
/// On Cortex-M a *larger* priority number means *lower* urgency, so
/// this ensures SVCall is never preempted by the others, and PendSV
/// never preempts SysTick.
pub const fn assert_priority_order<C: KernelConfig>() {
    assert!(
        C::SVCALL_PRIORITY < C::SYSTICK_PRIORITY,
        "SVCall priority must be strictly higher (smaller number) than SysTick priority"
    );
    assert!(
        C::SVCALL_PRIORITY < C::PENDSV_PRIORITY,
        "SVCall priority must be strictly higher (smaller number) than PendSV priority"
    );
    assert!(
        C::PENDSV_PRIORITY > C::SYSTICK_PRIORITY,
        "PendSV priority must be strictly lower (larger number) than SysTick priority"
    );
}

/// Compile-time assertion that `SYSTICK_CYCLES` fits in the 24-bit SysTick
/// RELOAD register.
///
/// The SysTick RELOAD register is 24 bits wide, so the maximum reload value
/// is 0xFF_FFFF (16,777,215).  Since the hardware register is loaded with
/// `SYSTICK_CYCLES - 1`, the check is `SYSTICK_CYCLES - 1 <= SYSTICK_RELOAD_MAX`.
/// Also rejects zero, which would underflow on the subtraction.
pub const fn assert_systick_reload<C: KernelConfig>() {
    assert!(C::SYSTICK_CYCLES > 0, "SYSTICK_CYCLES must be at least 1");
    // SYSTICK_RELOAD_MAX is defined above in this module (0xFF_FFFF).
    assert!(
        C::SYSTICK_CYCLES - 1 <= SYSTICK_RELOAD_MAX,
        "SYSTICK_CYCLES - 1 exceeds 24-bit SysTick RELOAD maximum (0xFFFFFF)"
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg_pools::MsgPools;
    use crate::partition_core::PartitionCore;
    use crate::port_pools::PortPools;
    use crate::sync_pools::SyncPools;

    // ============ compute_systick_reload tests ============

    #[test]
    fn compute_systick_reload_12mhz_1000us() {
        // 12 MHz clock, 1000 us (1 ms) tick period = 12,000 cycles
        // RELOAD register gets N-1 = 11,999
        const RELOAD: u32 = compute_systick_reload(12_000_000, 1000);
        assert_eq!(RELOAD, 11_999);
    }

    #[test]
    fn compute_systick_reload_120mhz_1000us() {
        // 120 MHz clock, 1000 us (1 ms) tick period = 120,000 cycles
        // RELOAD register gets N-1 = 119,999
        const RELOAD: u32 = compute_systick_reload(120_000_000, 1000);
        assert_eq!(RELOAD, 119_999);
    }

    #[test]
    fn compute_systick_reload_64mhz_1000us() {
        // 64 MHz clock, 1000 us (1 ms) tick period = 64,000 cycles
        // RELOAD register gets N-1 = 63,999
        const RELOAD: u32 = compute_systick_reload(64_000_000, 1000);
        assert_eq!(RELOAD, 63_999);
    }

    #[test]
    fn compute_systick_reload_max_valid() {
        // Maximum valid reload value: 0xFFFFFF = 16,777,215
        // Using 16 MHz clock with 1,048,576 us period = 16,777,216 cycles
        // RELOAD register gets N-1 = 16,777,215 (exactly 0xFFFFFF)
        const RELOAD: u32 = compute_systick_reload(16_000_000, 1_048_576);
        assert_eq!(RELOAD, 0xFFFFFF);
    }

    #[test]
    fn compute_systick_reload_non_integer_mhz() {
        // 12.5 MHz clock (12,500,000 Hz), 1000 us tick period = 12,500 cycles
        // RELOAD register gets N-1 = 12,499
        // This tests that we don't lose precision with non-integer MHz frequencies
        const RELOAD: u32 = compute_systick_reload(12_500_000, 1000);
        assert_eq!(RELOAD, 12_499);
    }

    #[test]
    fn compute_systick_reload_sub_mhz() {
        // 500 kHz clock (500,000 Hz), 2000 us tick period = 1,000 cycles
        // RELOAD register gets N-1 = 999
        // This tests clocks below 1 MHz which would fail with premature division
        const RELOAD: u32 = compute_systick_reload(500_000, 2000);
        assert_eq!(RELOAD, 999);
    }

    #[test]
    fn compute_systick_reload_fractional_result() {
        // 8 MHz clock, 125 us tick period = 1,000 cycles
        // RELOAD register gets N-1 = 999
        const RELOAD: u32 = compute_systick_reload(8_000_000, 125);
        assert_eq!(RELOAD, 999);
    }

    // Compile-time assertion tests for compute_systick_reload
    const _: u32 = compute_systick_reload(12_000_000, 1000);
    const _: u32 = compute_systick_reload(120_000_000, 1000);
    const _: u32 = compute_systick_reload(64_000_000, 1000);

    // Compile-time overflow validation test (would fail at compile time if uncommented)
    // const _OVERFLOW: u32 = compute_systick_reload(16_000_000, 1_048_577); // exceeds 24-bit

    #[test]
    #[should_panic(expected = "24-bit maximum")]
    fn compute_systick_reload_panics_on_overflow() {
        // 16 MHz clock with 1,048,577 us period exceeds 24-bit max
        // cycles = 16 * 1_048_577 = 16,777,232, reload = 16,777,231 > 0xFFFFFF
        let _ = compute_systick_reload(16_000_000, 1_048_577);
    }

    #[test]
    #[should_panic(expected = "must be at least 1")]
    fn compute_systick_reload_panics_on_zero_cycles() {
        // Very low frequency and short period results in 0 cycles
        // 100 Hz clock, 1 us period = 0 cycles (truncated)
        let _ = compute_systick_reload(100, 1);
    }

    /// Minimal config using all default values (only N and type aliases required).
    // TODO: type aliases cannot be defaulted - Rust's associated_type_defaults feature is unstable.
    struct DefaultPriority;
    impl KernelConfig for DefaultPriority {
        const N: usize = 2; // N has no default - must be specified
        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    /// Config that overrides priorities (uses default resource pool constants).
    struct CustomPriority;
    impl KernelConfig for CustomPriority {
        const N: usize = 2;
        const PENDSV_PRIORITY: u8 = 0xE0;
        const SYSTICK_PRIORITY: u8 = 0xC0;
        const CORE_CLOCK_HZ: u32 = 80_000_000; // 10 ms tick at 80 MHz
        const TICK_PERIOD_US: u32 = 10_000;
        #[cfg(feature = "partition-debug")]
        const DEBUG_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "dynamic-mpu")]
        const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 = 200;

        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    #[test]
    fn default_priorities_have_correct_values() {
        assert_eq!(DefaultPriority::SVCALL_PRIORITY, 0x00);
        assert_eq!(DefaultPriority::PENDSV_PRIORITY, 0xFF);
        assert_eq!(DefaultPriority::SYSTICK_PRIORITY, 0xFE);
    }

    #[test]
    fn default_systick_cycles_is_auto_calculated() {
        // Verify the trait default calculates SYSTICK_CYCLES from CORE_CLOCK_HZ and TICK_PERIOD_US
        let expected = (DefaultPriority::CORE_CLOCK_HZ as u64
            * DefaultPriority::TICK_PERIOD_US as u64
            / 1_000_000) as u32;
        assert_eq!(DefaultPriority::SYSTICK_CYCLES, expected);
    }

    #[test]
    fn assert_priority_order_accepts_defaults() {
        // Must not panic at runtime.
        assert_priority_order::<DefaultPriority>();
    }

    #[test]
    fn assert_priority_order_accepts_custom_valid() {
        // Must not panic with custom but valid ordering.
        assert_priority_order::<CustomPriority>();
    }

    #[test]
    fn custom_priorities_override_defaults() {
        assert_eq!(CustomPriority::PENDSV_PRIORITY, 0xE0);
        assert_eq!(CustomPriority::SYSTICK_PRIORITY, 0xC0);
    }

    #[test]
    fn custom_systick_cycles_is_auto_calculated() {
        // Verify custom config also gets SYSTICK_CYCLES auto-calculated from clock and period
        let expected = (CustomPriority::CORE_CLOCK_HZ as u64
            * CustomPriority::TICK_PERIOD_US as u64
            / 1_000_000) as u32;
        assert_eq!(CustomPriority::SYSTICK_CYCLES, expected);
    }

    // Compile-time assertions: full priority ordering (including SVCall)
    // is verified at const-eval time for both default and custom configs.
    const _: () = assert_priority_order::<DefaultPriority>();
    const _: () = assert_priority_order::<CustomPriority>();

    // Compile-time assertions: SysTick reload fits in 24-bit RELOAD register.
    const _: () = assert_systick_reload::<DefaultPriority>();
    const _: () = assert_systick_reload::<CustomPriority>();

    #[test]
    fn assert_systick_reload_accepts_default_config() {
        assert_systick_reload::<DefaultPriority>();
    }

    #[test]
    fn assert_systick_reload_accepts_custom_config() {
        assert_systick_reload::<CustomPriority>();
    }

    /// Config with SYSTICK_CYCLES that exceeds the 24-bit RELOAD max.
    struct OverflowSystick;
    impl KernelConfig for OverflowSystick {
        const N: usize = 2;
        const SYSTICK_CYCLES: u32 = SYSTICK_RELOAD_MAX + 2; // 0x1000001, reload would be 0x1000000
        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    #[test]
    #[should_panic(expected = "24-bit SysTick RELOAD maximum")]
    fn assert_systick_reload_panics_on_overflow() {
        assert_systick_reload::<OverflowSystick>();
    }

    /// Config with SYSTICK_CYCLES = 0 (would underflow on RELOAD = cycles - 1).
    struct ZeroSystick;
    impl KernelConfig for ZeroSystick {
        const N: usize = 2;
        const SYSTICK_CYCLES: u32 = 0;
        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    #[test]
    #[should_panic(expected = "must be at least 1")]
    fn assert_systick_reload_panics_on_zero() {
        assert_systick_reload::<ZeroSystick>();
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn default_debug_buffer_size_is_256() {
        assert_eq!(DefaultPriority::DEBUG_BUFFER_SIZE, 256);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn custom_debug_buffer_size_overrides_default() {
        // Verify CustomPriority can override DEBUG_BUFFER_SIZE
        assert_ne!(
            CustomPriority::DEBUG_BUFFER_SIZE,
            DefaultPriority::DEBUG_BUFFER_SIZE
        );
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn default_system_window_max_gap_ticks_is_100() {
        assert_eq!(DefaultPriority::SYSTEM_WINDOW_MAX_GAP_TICKS, 100);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn custom_system_window_max_gap_ticks_overrides_default() {
        // Verify CustomPriority can override SYSTEM_WINDOW_MAX_GAP_TICKS
        assert_ne!(
            CustomPriority::SYSTEM_WINDOW_MAX_GAP_TICKS,
            DefaultPriority::SYSTEM_WINDOW_MAX_GAP_TICKS
        );
    }
}
