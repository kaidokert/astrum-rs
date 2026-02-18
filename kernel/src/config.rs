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
    /// Maximum number of partitions.
    const N: usize;
    /// Schedule table capacity (number of schedule entries).
    const SCHED: usize;
    /// Stack word count per partition (256 = 1024 bytes for MPU alignment).
    const STACK_WORDS: usize;
    /// Semaphore pool capacity.
    const S: usize;
    /// Semaphore wait-queue depth.
    const SW: usize;
    /// Mutex pool capacity.
    const MS: usize;
    /// Mutex wait-queue depth.
    const MW: usize;
    /// Message-queue pool capacity (also used for queuing ports).
    const QS: usize;
    /// Message-queue depth (messages per queue).
    const QD: usize;
    /// Maximum message size in bytes (message queues & queuing ports).
    const QM: usize;
    /// Message-queue / queuing-port wait-queue depth.
    const QW: usize;
    /// Sampling-port pool capacity.
    const SP: usize;
    /// Sampling-port maximum message size in bytes.
    const SM: usize;
    /// Blackboard pool capacity.
    const BS: usize;
    /// Blackboard maximum message size in bytes.
    const BM: usize;
    /// Blackboard wait-queue depth.
    const BW: usize;

    /// Buffer pool slot count (only used with `dynamic-mpu` feature).
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize;
    /// Buffer slot size in bytes (only used with `dynamic-mpu` feature).
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize;
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
    /// Implementors may override this constant if a specific cycle count is
    /// required, but the default calculation should suffice for most cases.
    ///
    /// Note: The hardware reload register is set to `SYSTICK_CYCLES - 1`
    /// because SysTick counts down from the reload value to zero inclusive.
    ///
    /// [`CORE_CLOCK_HZ`]: Self::CORE_CLOCK_HZ
    /// [`TICK_PERIOD_US`]: Self::TICK_PERIOD_US
    // TODO: backward compatibility - the previous default was 120,000 (for 120 MHz).
    // Existing configurations that relied on the old default must now explicitly
    // set CORE_CLOCK_HZ to 120_000_000 to preserve the same timing behavior.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg_pools::MsgPools;
    use crate::partition_core::PartitionCore;
    use crate::port_pools::PortPools;
    use crate::sync_pools::SyncPools;

    /// Minimal config using all default priority values.
    struct DefaultPriority;
    impl KernelConfig for DefaultPriority {
        const N: usize = 2;
        const SCHED: usize = 4;
        const STACK_WORDS: usize = 256;
        const S: usize = 1;
        const SW: usize = 1;
        const MS: usize = 1;
        const MW: usize = 1;
        const QS: usize = 1;
        const QD: usize = 1;
        const QM: usize = 1;
        const QW: usize = 1;
        const SP: usize = 1;
        const SM: usize = 1;
        const BS: usize = 1;
        const BM: usize = 1;
        const BW: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 32;
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = 4;

        type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
        type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports =
            PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
    }

    /// Config that overrides priorities but keeps valid ordering.
    struct CustomPriority;
    impl KernelConfig for CustomPriority {
        const N: usize = 2;
        const SCHED: usize = 4;
        const STACK_WORDS: usize = 256;
        const S: usize = 1;
        const SW: usize = 1;
        const MS: usize = 1;
        const MW: usize = 1;
        const QS: usize = 1;
        const QD: usize = 1;
        const QM: usize = 1;
        const QW: usize = 1;
        const SP: usize = 1;
        const SM: usize = 1;
        const BS: usize = 1;
        const BM: usize = 1;
        const BW: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = 1;
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = 32;
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = 4;

        const SVCALL_PRIORITY: u8 = 0x00;
        const PENDSV_PRIORITY: u8 = 0xE0;
        const SYSTICK_PRIORITY: u8 = 0xC0;
        // Custom timing: 10 ms tick at 80 MHz (SYSTICK_CYCLES auto-calculated)
        const CORE_CLOCK_HZ: u32 = 80_000_000;
        const TICK_PERIOD_US: u32 = 10_000;

        #[cfg(feature = "partition-debug")]
        const DEBUG_BUFFER_SIZE: usize = 512;

        // Custom system window gap threshold for testing
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

    #[cfg(feature = "partition-debug")]
    #[test]
    fn default_debug_buffer_size_is_256() {
        assert_eq!(DefaultPriority::DEBUG_BUFFER_SIZE, 256);
    }

    #[cfg(feature = "partition-debug")]
    #[test]
    fn custom_debug_buffer_size_overrides_default() {
        // CustomPriority overrides DEBUG_BUFFER_SIZE to 512
        assert_eq!(CustomPriority::DEBUG_BUFFER_SIZE, 512);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn default_system_window_max_gap_ticks_is_100() {
        assert_eq!(DefaultPriority::SYSTEM_WINDOW_MAX_GAP_TICKS, 100);
    }

    #[cfg(feature = "dynamic-mpu")]
    #[test]
    fn custom_system_window_max_gap_ticks_overrides_default() {
        // CustomPriority overrides SYSTEM_WINDOW_MAX_GAP_TICKS to 200
        assert_eq!(CustomPriority::SYSTEM_WINDOW_MAX_GAP_TICKS, 200);
    }
}
