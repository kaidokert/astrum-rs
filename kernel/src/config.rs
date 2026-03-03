//! Kernel configuration traits, clock setup, and preset-based composition
//! via `compose_kernel_config!`.
//!
//! # Clock Configuration
//!
//! The [`KernelConfig`] trait controls the SysTick timer through three
//! associated constants:
//!
//! | Constant | Default | Purpose |
//! |---|---|---|
//! | `CORE_CLOCK_HZ` | 12 MHz (QEMU lm3s6965evb) | Processor core clock frequency |
//! | `TICK_PERIOD_US` | 1000 µs (1 ms) | Desired interval between ticks |
//! | `SYSTICK_CYCLES` | auto-calculated | Reload cycle count (leave as default) |
//! | `USE_PROCESSOR_CLOCK` | `true` | SysTick clock source (`true` = core, `false` = external) |
//! | `MPU_ENFORCE` | `false` | Enable MPU enforcement for partition memory isolation |
//!
//! `SYSTICK_CYCLES` is derived automatically:
//!
//! ```text
//! SYSTICK_CYCLES = CORE_CLOCK_HZ * TICK_PERIOD_US / 1_000_000
//! ```
//!
//! ## BSP Override Examples
//!
//! Each BSP crate provides a [`KernelConfig`] implementation that sets
//! `CORE_CLOCK_HZ` to match the target's core clock. `TICK_PERIOD_US`
//! can also be adjusted, but 1000 µs (1 ms) is typical.
//!
//! **nRF52840 — 64 MHz core clock, default 1 ms tick:**
//!
//! ```ignore
//! kernel::compose_kernel_config!(
//!     pub Nrf52840Config<Partitions4, SyncStandard, MsgStandard,
//!                        PortsStandard, DebugEnabled> {
//!         core_clock_hz = 64_000_000;
//!     }
//! );
//! ```
//!
//! **STM32F4 — 168 MHz core clock, 500 µs tick:**
//!
//! ```ignore
//! kernel::compose_kernel_config!(
//!     pub Stm32f4Config<Partitions4, SyncStandard, MsgStandard,
//!                       PortsStandard, DebugEnabled> {
//!         core_clock_hz = 168_000_000;
//!         tick_period_us = 500;
//!     }
//! );
//! ```
//!
//! **SAMD51 — 120 MHz core clock, external reference clock:**
//!
//! ```ignore
//! kernel::compose_kernel_config!(
//!     pub Samd51Config<Partitions4, SyncStandard, MsgStandard,
//!                      PortsStandard, DebugEnabled> {
//!         core_clock_hz = 15_000_000; // HCLK/8 external ref
//!         tick_period_us = 2000;
//!         use_processor_clock = false;
//!     }
//! );
//! ```
//!
//! ## Consequences of Incorrect `CORE_CLOCK_HZ`
//!
//! The SysTick reload value is computed from `CORE_CLOCK_HZ`. If this
//! constant does not match the actual hardware clock frequency:
//!
//! - **Value too high** — The reload value is too large, so ticks fire
//!   slower than expected. A 1 ms tick becomes, say, 2.6 ms. Schedule
//!   windows run long and deadlines are missed late.
//!
//! - **Value too low** — The reload value is too small, so ticks fire
//!   faster than expected. A 1 ms tick becomes, say, 0.38 ms.
//!   Partitions are preempted early and real-time budgets shrink.
//!
//! In both cases every time-dependent kernel service — schedule
//! advancement, timeout expiration, partition budgets — drifts from
//! wall-clock time. This is silent; no assertion fires at runtime.
//! Always verify `CORE_CLOCK_HZ` against the BSP's actual clock-tree
//! setup (PLL configuration, prescalers, etc.).
//!
//! ## SysTick Clock Source (`CLKSOURCE` Bit)
//!
//! The ARM SysTick timer (`SYST_CSR` register, bit 2) supports two
//! clock sources:
//!
//! | CLKSOURCE | Source | Typical use |
//! |---|---|---|
//! | 1 | Processor clock (HCLK) | **Default — used by this kernel** |
//! | 0 | External reference clock | Vendor-specific, often HCLK/8 |
//!
//! The [`USE_PROCESSOR_CLOCK`](KernelConfig::USE_PROCESSOR_CLOCK)
//! constant controls this selection. When `true` (the default),
//! [`boot()`](crate::boot::boot) sets `CLKSOURCE = 1` (processor
//! clock). When `false`, it sets `CLKSOURCE = 0` (external reference).
//!
//! To use an external reference clock instead, a BSP should:
//!
//! 1. Set `USE_PROCESSOR_CLOCK` to `false`.
//! 2. Set `CORE_CLOCK_HZ` to the external reference frequency (e.g.,
//!    HCLK / 8 on STM32).
//!
//! See the SAMD51 example above for a complete configuration.
//!
//! Most BSPs should use the processor clock (the default). The
//! external reference is only useful when the implementation-defined
//! reference provides a more stable or lower-frequency timebase.

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
    /// Returns the base address of a partition's stack storage array.
    fn stack_base(&self, index: usize) -> Option<u32>;
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

/// Sub-trait providing readable constant names for partition-related
/// configuration parameters.
///
/// This trait offers self-documenting names that map 1-to-1 to the
/// single-letter constants on [`KernelConfig`]:
///
/// | `PartitionConfig` | `KernelConfig` | Meaning |
/// |---|---|---|
/// | `COUNT` | `N` | Number of partitions |
/// | `SCHEDULE_CAPACITY` | `SCHED` | Schedule table entries |
/// | `STACK_WORDS` | `STACK_WORDS` | Stack size per partition (in 32-bit words) |
pub trait PartitionConfig {
    /// Number of partitions (maps to [`KernelConfig::N`]).
    const COUNT: usize;
    /// Schedule table capacity (maps to [`KernelConfig::SCHED`]).
    const SCHEDULE_CAPACITY: usize;
    /// Stack size per partition in 32-bit words (maps to [`KernelConfig::STACK_WORDS`]).
    const STACK_WORDS: usize;
}

/// Preset: 1 partition, 4 schedule entries, 256-word (1 KiB) stacks.
pub struct Partitions1;

impl PartitionConfig for Partitions1 {
    const COUNT: usize = 1;
    const SCHEDULE_CAPACITY: usize = 4;
    const STACK_WORDS: usize = 256;
}

/// Preset: 2 partitions, 4 schedule entries, 256-word (1 KiB) stacks.
pub struct Partitions2;

impl PartitionConfig for Partitions2 {
    const COUNT: usize = 2;
    const SCHEDULE_CAPACITY: usize = 4;
    const STACK_WORDS: usize = 256;
}

/// Preset: 3 partitions, 8 schedule entries, 256-word (1 KiB) stacks.
///
/// Fills the gap between [`Partitions2`] and [`Partitions4`] for demos
/// and applications that use exactly three partitions (e.g. blackboard_demo).
pub struct Partitions3;

impl PartitionConfig for Partitions3 {
    const COUNT: usize = 3;
    const SCHEDULE_CAPACITY: usize = 8;
    const STACK_WORDS: usize = 256;
}

/// Preset: 4 partitions, 8 schedule entries, 256-word (1 KiB) stacks.
pub struct Partitions4;

impl PartitionConfig for Partitions4 {
    const COUNT: usize = 4;
    const SCHEDULE_CAPACITY: usize = 8;
    const STACK_WORDS: usize = 256;
}

/// Sub-trait providing readable constant names for synchronization-related
/// configuration parameters.
///
/// This trait offers self-documenting names that map 1-to-1 to the
/// single-letter constants on [`KernelConfig`]:
///
/// | `SyncConfig` | `KernelConfig` | Meaning |
/// |---|---|---|
/// | `SEMAPHORES` | `S` | Semaphore pool capacity |
/// | `SEMAPHORE_WAITQ` | `SW` | Semaphore wait-queue depth |
/// | `MUTEXES` | `MS` | Mutex pool capacity |
/// | `MUTEX_WAITQ` | `MW` | Mutex wait-queue depth |
pub trait SyncConfig {
    /// Semaphore pool capacity (maps to [`KernelConfig::S`]).
    const SEMAPHORES: usize;
    /// Semaphore wait-queue depth (maps to [`KernelConfig::SW`]).
    const SEMAPHORE_WAITQ: usize;
    /// Mutex pool capacity (maps to [`KernelConfig::MS`]).
    const MUTEXES: usize;
    /// Mutex wait-queue depth (maps to [`KernelConfig::MW`]).
    const MUTEX_WAITQ: usize;
}

/// Preset: minimal synchronization (1 semaphore, 1 mutex, wait-queue depth 1).
pub struct SyncMinimal;

impl SyncConfig for SyncMinimal {
    const SEMAPHORES: usize = 1;
    const SEMAPHORE_WAITQ: usize = 1;
    const MUTEXES: usize = 1;
    const MUTEX_WAITQ: usize = 1;
}

/// Preset: rich synchronization (8 semaphores, 4 mutexes, wait-queue depth 4).
pub struct SyncRich;

impl SyncConfig for SyncRich {
    const SEMAPHORES: usize = 8;
    const SEMAPHORE_WAITQ: usize = 4;
    const MUTEXES: usize = 4;
    const MUTEX_WAITQ: usize = 4;
}

/// Preset: standard synchronization (4 semaphores, 4 mutexes, wait-queue depth 4).
pub struct SyncStandard;

impl SyncConfig for SyncStandard {
    const SEMAPHORES: usize = 4;
    const SEMAPHORE_WAITQ: usize = 4;
    const MUTEXES: usize = 4;
    const MUTEX_WAITQ: usize = 4;
}

/// Sub-trait providing readable constant names for message-queue-related
/// configuration parameters.
///
/// This trait offers self-documenting names that map 1-to-1 to the
/// single-letter constants on [`KernelConfig`]:
///
/// | `MsgConfig` | `KernelConfig` | Meaning |
/// |---|---|---|
/// | `QUEUES` | `QS` | Message-queue pool capacity |
/// | `QUEUE_DEPTH` | `QD` | Messages per queue |
/// | `MAX_MSG_SIZE` | `QM` | Maximum message size in bytes |
/// | `QUEUE_WAITQ` | `QW` | Message-queue wait-queue depth |
pub trait MsgConfig {
    /// Message-queue pool capacity (maps to [`KernelConfig::QS`]).
    const QUEUES: usize;
    /// Messages per queue (maps to [`KernelConfig::QD`]).
    const QUEUE_DEPTH: usize;
    /// Maximum message size in bytes (maps to [`KernelConfig::QM`]).
    const MAX_MSG_SIZE: usize;
    /// Message-queue wait-queue depth (maps to [`KernelConfig::QW`]).
    const QUEUE_WAITQ: usize;
}

/// Preset: minimal messaging (1 queue, depth 1, 1-byte messages, wait-queue depth 1).
pub struct MsgMinimal;

impl MsgConfig for MsgMinimal {
    const QUEUES: usize = 1;
    const QUEUE_DEPTH: usize = 1;
    const MAX_MSG_SIZE: usize = 1;
    const QUEUE_WAITQ: usize = 1;
}

/// Preset: small messaging (2 queues, depth 4, 4-byte messages, wait-queue depth 2).
///
/// Covers the common intermediate messaging pattern used by hw_integration
/// and blocking_deschedule tests where minimal is too constrained but rich
/// is overkill.
pub struct MsgSmall;

impl MsgConfig for MsgSmall {
    const QUEUES: usize = 2;
    const QUEUE_DEPTH: usize = 4;
    const MAX_MSG_SIZE: usize = 4;
    const QUEUE_WAITQ: usize = 2;
}

/// Preset: rich messaging (4 queues, depth 4, 64-byte messages, wait-queue depth 4).
pub struct MsgRich;

impl MsgConfig for MsgRich {
    const QUEUES: usize = 4;
    const QUEUE_DEPTH: usize = 4;
    const MAX_MSG_SIZE: usize = 64;
    const QUEUE_WAITQ: usize = 4;
}

/// Preset: standard messaging (4 queues, depth 4, 4-byte messages, wait-queue depth 4).
pub struct MsgStandard;

impl MsgConfig for MsgStandard {
    const QUEUES: usize = 4;
    const QUEUE_DEPTH: usize = 4;
    const MAX_MSG_SIZE: usize = 4;
    const QUEUE_WAITQ: usize = 4;
}

/// Sub-trait providing readable constant names for port-related
/// configuration parameters.
///
/// This trait offers self-documenting names that map 1-to-1 to the
/// single-letter constants on [`KernelConfig`]:
///
/// | `PortsConfig` | `KernelConfig` | Meaning |
/// |---|---|---|
/// | `SAMPLING_PORTS` | `SP` | Sampling-port pool capacity |
/// | `SAMPLING_MAX_MSG_SIZE` | `SM` | Sampling-port maximum message size in bytes |
/// | `BLACKBOARDS` | `BS` | Blackboard pool capacity |
/// | `BLACKBOARD_MAX_MSG_SIZE` | `BM` | Blackboard maximum message size in bytes |
/// | `BLACKBOARD_WAITQ` | `BW` | Blackboard wait-queue depth |
pub trait PortsConfig {
    /// Sampling-port pool capacity (maps to [`KernelConfig::SP`]).
    const SAMPLING_PORTS: usize;
    /// Sampling-port maximum message size in bytes (maps to [`KernelConfig::SM`]).
    const SAMPLING_MAX_MSG_SIZE: usize;
    /// Blackboard pool capacity (maps to [`KernelConfig::BS`]).
    const BLACKBOARDS: usize;
    /// Blackboard maximum message size in bytes (maps to [`KernelConfig::BM`]).
    const BLACKBOARD_MAX_MSG_SIZE: usize;
    /// Blackboard wait-queue depth (maps to [`KernelConfig::BW`]).
    const BLACKBOARD_WAITQ: usize;
}

/// Preset: minimal ports (1 sampling port, 64-byte messages, 1 blackboard, 64-byte messages, wait-queue depth 1).
pub struct PortsMinimal;

impl PortsConfig for PortsMinimal {
    const SAMPLING_PORTS: usize = 1;
    const SAMPLING_MAX_MSG_SIZE: usize = 64;
    const BLACKBOARDS: usize = 1;
    const BLACKBOARD_MAX_MSG_SIZE: usize = 64;
    const BLACKBOARD_WAITQ: usize = 1;
}

/// Preset: tiny ports (all fields 1, for test/minimal configs where SM=64 default is too large).
pub struct PortsTiny;

impl PortsConfig for PortsTiny {
    const SAMPLING_PORTS: usize = 1;
    const SAMPLING_MAX_MSG_SIZE: usize = 1;
    const BLACKBOARDS: usize = 1;
    const BLACKBOARD_MAX_MSG_SIZE: usize = 1;
    const BLACKBOARD_WAITQ: usize = 1;
}

/// Preset: rich ports (4 sampling ports, 64-byte messages, 4 blackboards, 64-byte messages, wait-queue depth 4).
pub struct PortsRich;

impl PortsConfig for PortsRich {
    const SAMPLING_PORTS: usize = 4;
    const SAMPLING_MAX_MSG_SIZE: usize = 64;
    const BLACKBOARDS: usize = 4;
    const BLACKBOARD_MAX_MSG_SIZE: usize = 64;
    const BLACKBOARD_WAITQ: usize = 4;
}

/// Preset: small ports (SP=4, SM=4, BS=4, BM=4, BW=4).
pub struct PortsSmall;
impl PortsConfig for PortsSmall {
    const SAMPLING_PORTS: usize = 4;
    const SAMPLING_MAX_MSG_SIZE: usize = 4;
    const BLACKBOARDS: usize = 4;
    const BLACKBOARD_MAX_MSG_SIZE: usize = 4;
    const BLACKBOARD_WAITQ: usize = 4;
}

/// Preset: standard ports (SP=8, SM=4, BS=4, BM=4, BW=4).
pub struct PortsStandard;
impl PortsConfig for PortsStandard {
    const SAMPLING_PORTS: usize = 8;
    const SAMPLING_MAX_MSG_SIZE: usize = 4;
    const BLACKBOARDS: usize = 4;
    const BLACKBOARD_MAX_MSG_SIZE: usize = 4;
    const BLACKBOARD_WAITQ: usize = 4;
}

/// Sub-trait providing readable constant names for debug-output configuration.
///
/// | `DebugConfig` | Meaning |
/// |---|---|
/// | `BUFFER_SIZE` | Debug buffer size per partition (bytes) |
/// | `AUTO_DRAIN_BUDGET` | Max bytes `drain_debug_auto` drains per call |
pub trait DebugConfig {
    /// Debug ring-buffer capacity in bytes per partition.
    const BUFFER_SIZE: usize;
    /// Maximum bytes drained per `drain_debug_auto` invocation.
    const AUTO_DRAIN_BUDGET: usize;
}

/// Preset: debug enabled (256-byte buffer, 256-byte drain budget).
pub struct DebugEnabled;

impl DebugConfig for DebugEnabled {
    const BUFFER_SIZE: usize = 256;
    const AUTO_DRAIN_BUDGET: usize = 256;
}

/// Preset: debug disabled (zero-size buffer, zero drain budget).
pub struct DebugDisabled;

impl DebugConfig for DebugDisabled {
    const BUFFER_SIZE: usize = 0;
    const AUTO_DRAIN_BUDGET: usize = 0;
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

    /// Maximum bytes `drain_debug_auto` will drain per call.
    /// Set to 0 to disable automatic draining entirely.
    /// Not cfg-gated so that `drain_debug_auto` compiles unconditionally.
    const DEBUG_AUTO_DRAIN_BUDGET: usize = 256;

    /// SVCall exception priority (0x00 = highest on Cortex-M).
    const SVCALL_PRIORITY: u8 = 0x00;
    /// PendSV exception priority (must be the lowest of the three).
    const PENDSV_PRIORITY: u8 = 0xFF;
    /// SysTick exception priority in the three-tier model:
    /// `SVCALL < SYSTICK < MIN_APP_IRQ <= IRQ_DEFAULT < PENDSV`
    /// (numerically, larger = lower urgency).  SysTick sits above all
    /// device IRQs so the tick is never delayed by application handlers.
    const SYSTICK_PRIORITY: u8 = 0x10;
    /// Default NVIC priority for device IRQs in the three-tier model:
    /// `SVCALL < SYSTICK < MIN_APP_IRQ <= IRQ_DEFAULT < PENDSV`
    /// (numerically).  Must be >= MIN_APP_IRQ_PRIORITY.
    const IRQ_DEFAULT_PRIORITY: u8 = 0xC0;
    /// Minimum (numerically largest allowed) priority for application IRQs.
    /// This is the floor of Tier 2 in the three-tier priority model.
    const MIN_APP_IRQ_PRIORITY: u8 = 0x20;

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

    /// SysTick clock source selection.
    ///
    /// When `true` (the default), the SysTick timer is clocked from the
    /// processor core clock (`CLKSOURCE = 1`). When `false`, it uses the
    /// external reference clock (`CLKSOURCE = 0`), which is
    /// vendor-specific (often HCLK/8).
    ///
    /// When set to `false`, [`CORE_CLOCK_HZ`](Self::CORE_CLOCK_HZ) must
    /// reflect the external reference frequency, not the core clock.
    const USE_PROCESSOR_CLOCK: bool = true;

    /// Enable MPU enforcement for partition memory isolation.
    ///
    /// When `true`, the kernel programs the MPU before each partition
    /// context switch to restrict memory access to the active partition's
    /// region. When `false` (the default), the MPU is not programmed and
    /// partitions share the full address space.
    const MPU_ENFORCE: bool = false;

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

    /// Partition sub-config preset used to derive N, SCHED, STACK_WORDS.
    type PartitionCfg: PartitionConfig;
    /// Sync sub-config preset used to derive S, SW, MS, MW.
    type SyncCfg: SyncConfig;
    /// Messaging sub-config preset used to derive QS, QD, QM, QW.
    type MsgCfg: MsgConfig;
    /// Ports sub-config preset used to derive SP, SM, BS, BM, BW.
    type PortsCfg: PortsConfig;
    /// Debug sub-config preset used to derive DEBUG_BUFFER_SIZE, DEBUG_AUTO_DRAIN_BUDGET.
    type DebugCfg: DebugConfig;
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

/// Compile-time assertion enforcing the three-tier priority ordering:
/// `SVCALL < SYSTICK < MIN_APP_IRQ <= IRQ_DEFAULT < PENDSV`
/// (numerically, larger = lower urgency).
pub const fn assert_priority_order<C: KernelConfig>() {
    assert!(
        C::SVCALL_PRIORITY < C::SYSTICK_PRIORITY,
        "SVCall priority must be strictly higher (smaller number) than SysTick priority"
    );
    assert!(
        C::SYSTICK_PRIORITY < C::MIN_APP_IRQ_PRIORITY,
        "SysTick priority must be strictly higher (smaller number) than MIN_APP_IRQ priority"
    );
    assert!(
        C::MIN_APP_IRQ_PRIORITY <= C::IRQ_DEFAULT_PRIORITY,
        "MIN_APP_IRQ priority must be <= (numerically) IRQ default priority"
    );
    assert!(
        C::IRQ_DEFAULT_PRIORITY < C::PENDSV_PRIORITY,
        "IRQ default priority must be strictly higher (smaller number) than PendSV priority"
    );
}

/// Runtime guard ensuring an IRQ priority does not violate the four-tier
/// model floor.  Returns `Err` if `priority` is numerically below (higher
/// urgency than) `min_app_irq_priority`.
///
/// Called from `enable_bound_irqs` (generated by [`bind_interrupts!`]) to
/// catch invalid caller-supplied priorities before they reach the NVIC.
#[inline]
pub fn validate_irq_priority(priority: u8, min_app_irq_priority: u8) -> Result<(), &'static str> {
    if priority >= min_app_irq_priority {
        Ok(())
    } else {
        Err("IRQ priority violates MIN_APP_IRQ_PRIORITY floor")
    }
}

/// Generates the four associated type aliases (`Core`, `Sync`, `Msg`,
/// `Ports`) required by [`KernelConfig`].
///
/// # Forms
///
/// - `kernel_config_types!()` — uses default stack and default sub-config presets.
/// - `kernel_config_types!($stack)` — uses the provided stack type with default sub-config presets.
/// - `kernel_config_types!(@cfg $parts, $sync, $msg, $ports, $debug)` — default stack, explicit sub-configs.
/// - `kernel_config_types!(@cfg $parts, $sync, $msg, $ports, $debug; $stack)` — explicit stack and sub-configs.
#[macro_export]
macro_rules! kernel_config_types {
    () => {
        $crate::kernel_config_types!(@cfg
            $crate::config::Partitions2,
            $crate::config::SyncMinimal,
            $crate::config::MsgMinimal,
            $crate::config::PortsTiny,
            $crate::config::DebugEnabled;
            $crate::partition_core::AlignedStack1K
        );
    };
    ($stack:ty) => {
        $crate::kernel_config_types!(@cfg
            $crate::config::Partitions2,
            $crate::config::SyncMinimal,
            $crate::config::MsgMinimal,
            $crate::config::PortsTiny,
            $crate::config::DebugEnabled;
            $stack
        );
    };
    (@cfg $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty) => {
        $crate::kernel_config_types!(@cfg $parts, $sync, $msg, $ports, $debug; $crate::partition_core::AlignedStack1K);
    };
    (@cfg $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty; $stack:ty) => {
        type Core = $crate::partition_core::PartitionCore<{ Self::N }, { Self::SCHED }, $stack>;
        type Sync =
            $crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
        type Msg =
            $crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
        type Ports = $crate::port_pools::PortPools<
            { Self::SP },
            { Self::SM },
            { Self::BS },
            { Self::BM },
            { Self::BW },
        >;
        type PartitionCfg = $parts;
        type SyncCfg = $sync;
        type MsgCfg = $msg;
        type PortsCfg = $ports;
        type DebugCfg = $debug;
    };
}

/// Re-exports every [`KernelConfig`] associated constant as an inherent
/// constant on `$name`, so callers can write `MyConfig::SCHED` without
/// importing the trait.
#[macro_export]
#[doc(hidden)]
macro_rules! _kernel_config_inherent_consts {
    ($vis:vis $name:ident) => {
        #[allow(dead_code)]
        impl $name {
            $vis const N: usize = <$name as $crate::config::KernelConfig>::N;
            $vis const SCHED: usize = <$name as $crate::config::KernelConfig>::SCHED;
            $vis const STACK_WORDS: usize = <$name as $crate::config::KernelConfig>::STACK_WORDS;
            $vis const S: usize = <$name as $crate::config::KernelConfig>::S;
            $vis const SW: usize = <$name as $crate::config::KernelConfig>::SW;
            $vis const MS: usize = <$name as $crate::config::KernelConfig>::MS;
            $vis const MW: usize = <$name as $crate::config::KernelConfig>::MW;
            $vis const QS: usize = <$name as $crate::config::KernelConfig>::QS;
            $vis const QD: usize = <$name as $crate::config::KernelConfig>::QD;
            $vis const QM: usize = <$name as $crate::config::KernelConfig>::QM;
            $vis const QW: usize = <$name as $crate::config::KernelConfig>::QW;
            $vis const SP: usize = <$name as $crate::config::KernelConfig>::SP;
            $vis const SM: usize = <$name as $crate::config::KernelConfig>::SM;
            $vis const BS: usize = <$name as $crate::config::KernelConfig>::BS;
            $vis const BM: usize = <$name as $crate::config::KernelConfig>::BM;
            $vis const BW: usize = <$name as $crate::config::KernelConfig>::BW;
            #[cfg(feature = "dynamic-mpu")]
            $vis const BP: usize = <$name as $crate::config::KernelConfig>::BP;
            #[cfg(feature = "dynamic-mpu")]
            $vis const BZ: usize = <$name as $crate::config::KernelConfig>::BZ;
            #[cfg(feature = "dynamic-mpu")]
            $vis const DR: usize = <$name as $crate::config::KernelConfig>::DR;
            #[cfg(feature = "dynamic-mpu")]
            $vis const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 =
                <$name as $crate::config::KernelConfig>::SYSTEM_WINDOW_MAX_GAP_TICKS;
            #[cfg(feature = "partition-debug")]
            $vis const DEBUG_BUFFER_SIZE: usize =
                <$name as $crate::config::KernelConfig>::DEBUG_BUFFER_SIZE;
            $vis const DEBUG_AUTO_DRAIN_BUDGET: usize =
                <$name as $crate::config::KernelConfig>::DEBUG_AUTO_DRAIN_BUDGET;
            $vis const SVCALL_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::SVCALL_PRIORITY;
            $vis const PENDSV_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::PENDSV_PRIORITY;
            $vis const SYSTICK_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::SYSTICK_PRIORITY;
            $vis const IRQ_DEFAULT_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::IRQ_DEFAULT_PRIORITY;
            $vis const MIN_APP_IRQ_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::MIN_APP_IRQ_PRIORITY;
            $vis const CORE_CLOCK_HZ: u32 = <$name as $crate::config::KernelConfig>::CORE_CLOCK_HZ;
            $vis const TICK_PERIOD_US: u32 = <$name as $crate::config::KernelConfig>::TICK_PERIOD_US;
            $vis const SYSTICK_CYCLES: u32 = <$name as $crate::config::KernelConfig>::SYSTICK_CYCLES;
            $vis const USE_PROCESSOR_CLOCK: bool =
                <$name as $crate::config::KernelConfig>::USE_PROCESSOR_CLOCK;
            $vis const MPU_ENFORCE: bool = <$name as $crate::config::KernelConfig>::MPU_ENFORCE;
        }
    };
}

#[macro_export]
#[doc(hidden)]
macro_rules! _kernel_config_field {
    (partitions = $v:expr) => {
        const N: usize = $v;
    };
    (schedule_capacity = $v:expr) => {
        const SCHED: usize = $v;
    };
    (stack_words = $v:expr) => {
        const STACK_WORDS: usize = $v;
    };
    (semaphores = $v:expr) => {
        const S: usize = $v;
    };
    (S = $v:expr) => {
        const S: usize = $v;
    };
    (semaphore_waitq = $v:expr) => {
        const SW: usize = $v;
    };
    (SW = $v:expr) => {
        const SW: usize = $v;
    };
    (mutexes = $v:expr) => {
        const MS: usize = $v;
    };
    (MS = $v:expr) => {
        const MS: usize = $v;
    };
    (mutex_waitq = $v:expr) => {
        const MW: usize = $v;
    };
    (MW = $v:expr) => {
        const MW: usize = $v;
    };
    (queues = $v:expr) => {
        const QS: usize = $v;
    };
    (QS = $v:expr) => {
        const QS: usize = $v;
    };
    (queue_depth = $v:expr) => {
        const QD: usize = $v;
    };
    (QD = $v:expr) => {
        const QD: usize = $v;
    };
    (max_msg_size = $v:expr) => {
        const QM: usize = $v;
    };
    (QM = $v:expr) => {
        const QM: usize = $v;
    };
    (queue_waitq = $v:expr) => {
        const QW: usize = $v;
    };
    (QW = $v:expr) => {
        const QW: usize = $v;
    };
    (sampling_ports = $v:expr) => {
        const SP: usize = $v;
    };
    (SP = $v:expr) => {
        const SP: usize = $v;
    };
    (sampling_msg_size = $v:expr) => {
        const SM: usize = $v;
    };
    (SM = $v:expr) => {
        const SM: usize = $v;
    };
    (blackboards = $v:expr) => {
        const BS: usize = $v;
    };
    (BS = $v:expr) => {
        const BS: usize = $v;
    };
    (blackboard_msg_size = $v:expr) => {
        const BM: usize = $v;
    };
    (BM = $v:expr) => {
        const BM: usize = $v;
    };
    (blackboard_waitq = $v:expr) => {
        const BW: usize = $v;
    };
    (BW = $v:expr) => {
        const BW: usize = $v;
    };
    (core_clock_hz = $v:expr) => {
        const CORE_CLOCK_HZ: u32 = $v;
    };
    (tick_period_us = $v:expr) => {
        const TICK_PERIOD_US: u32 = $v;
    };
    (use_processor_clock = $v:expr) => {
        const USE_PROCESSOR_CLOCK: bool = $v;
    };
    (mpu_enforce = $v:expr) => {
        const MPU_ENFORCE: bool = $v;
    };
    (svcall_priority = $v:expr) => {
        const SVCALL_PRIORITY: u8 = $v;
    };
    (pendsv_priority = $v:expr) => {
        const PENDSV_PRIORITY: u8 = $v;
    };
    (systick_priority = $v:expr) => {
        const SYSTICK_PRIORITY: u8 = $v;
    };
    (irq_priority = $v:expr) => {
        const IRQ_DEFAULT_PRIORITY: u8 = $v;
    };
    (min_app_irq_priority = $v:expr) => {
        const MIN_APP_IRQ_PRIORITY: u8 = $v;
    };
    (debug_auto_drain = $v:expr) => {
        const DEBUG_AUTO_DRAIN_BUDGET: usize = $v;
    };
    (buffer_pool_regions = $v:expr) => {
        #[cfg(feature = "dynamic-mpu")]
        const BP: usize = $v;
    };
    (buffer_zone_size = $v:expr) => {
        #[cfg(feature = "dynamic-mpu")]
        const BZ: usize = $v;
    };
    (dynamic_regions = $v:expr) => {
        #[cfg(feature = "dynamic-mpu")]
        const DR: usize = $v;
    };
    (system_window_max_gap_ticks = $v:expr) => {
        #[cfg(feature = "dynamic-mpu")]
        const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 = $v;
    };
    (debug_buffer_size = $v:expr) => {
        #[cfg(feature = "partition-debug")]
        const DEBUG_BUFFER_SIZE: usize = $v;
    };
}

/// Conditional bridge for **DebugConfig** — see [`_compose_sync_default!`].
///
/// Tag-dispatched: `@DBS` = `DEBUG_BUFFER_SIZE`, `@DAD` = `DEBUG_AUTO_DRAIN_BUDGET`.
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_debug_default {
    // ── DBS hit arms ──
    (@DBS, $debug:ty; #[$a:meta] debug_buffer_size = $v:expr; $($r:tt)*) => {};
    (@DBS, $debug:ty; debug_buffer_size = $v:expr; $($r:tt)*) => {};
    (@DBS, $debug:ty; #[$a:meta] const DEBUG_BUFFER_SIZE : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@DBS, $debug:ty; const DEBUG_BUFFER_SIZE : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── DAD hit arms ──
    (@DAD, $debug:ty; #[$a:meta] debug_auto_drain = $v:expr; $($r:tt)*) => {};
    (@DAD, $debug:ty; debug_auto_drain = $v:expr; $($r:tt)*) => {};
    (@DAD, $debug:ty; #[$a:meta] const DEBUG_AUTO_DRAIN_BUDGET : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@DAD, $debug:ty; const DEBUG_AUTO_DRAIN_BUDGET : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── Skip: non-matching attributed friendly-name (shared) ──
    (@ $tag:tt, $debug:ty; #[$a:meta] $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_debug_default!(@ $tag, $debug; $($r)*);
    };
    // ── Skip: non-matching bare friendly-name (shared) ──
    (@ $tag:tt, $debug:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_debug_default!(@ $tag, $debug; $($r)*);
    };
    // ── Skip: non-matching attributed const (shared) ──
    (@ $tag:tt, $debug:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_debug_default!(@ $tag, $debug; $($r)*);
    };
    // ── Skip: non-matching bare const (shared) ──
    (@ $tag:tt, $debug:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_debug_default!(@ $tag, $debug; $($r)*);
    };
    // ── Terminal: emit preset bridge ──
    (@DBS, $debug:ty;) => {
        #[cfg(feature = "partition-debug")]
        const DEBUG_BUFFER_SIZE: usize = <$debug as $crate::config::DebugConfig>::BUFFER_SIZE;
    };
    (@DAD, $debug:ty;) => {
        const DEBUG_AUTO_DRAIN_BUDGET: usize =
            <$debug as $crate::config::DebugConfig>::AUTO_DRAIN_BUDGET;
    };
}

/// Conditionally bridges `N` from a partition preset unless overridden
/// by `partitions = …` or `const N : … = …` in the override block.
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_partition_n_default {
    ($p:ty; partitions = $v:expr; $($r:tt)*) => {};
    ($p:ty; const N : $ty:ty = $v:expr; $($r:tt)*) => {};
    ($p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_n_default!($p; $($r)*);
    };
    ($p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_n_default!($p; $($r)*);
    };
    ($p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_n_default!($p; $($r)*);
    };
    ($p:ty;) => {
        const N: usize = <$p as $crate::config::PartitionConfig>::COUNT;
    };
}

/// Conditionally bridges `SCHED` from a partition preset unless overridden
/// by `schedule_capacity = …` or `const SCHED : … = …` in the override block.
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_partition_sched_default {
    ($p:ty; schedule_capacity = $v:expr; $($r:tt)*) => {};
    ($p:ty; const SCHED : $ty:ty = $v:expr; $($r:tt)*) => {};
    ($p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_sched_default!($p; $($r)*);
    };
    ($p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_sched_default!($p; $($r)*);
    };
    ($p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_sched_default!($p; $($r)*);
    };
    ($p:ty;) => {
        const SCHED: usize = <$p as $crate::config::PartitionConfig>::SCHEDULE_CAPACITY;
    };
}

/// Conditionally bridges `STACK_WORDS` from a partition preset unless
/// overridden by `stack_words = …` or `const STACK_WORDS : … = …`.
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_partition_stack_default {
    ($p:ty; stack_words = $v:expr; $($r:tt)*) => {};
    ($p:ty; const STACK_WORDS : $ty:ty = $v:expr; $($r:tt)*) => {};
    ($p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_stack_default!($p; $($r)*);
    };
    ($p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_stack_default!($p; $($r)*);
    };
    ($p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_partition_stack_default!($p; $($r)*);
    };
    ($p:ty;) => {
        const STACK_WORDS: usize = <$p as $crate::config::PartitionConfig>::STACK_WORDS;
    };
}

/// Conditionally bridges a single `SyncConfig` field from a preset unless
/// the override token stream contains a matching override.
///
/// Called with a tag (`@S`, `@SW`, `@MS`, `@MW`) to select the field.
/// Each field recognises five "hit" forms that suppress the preset bridge:
///   1. `friendly = val;`                    (bare friendly-name)
///   2. `#[attr] friendly = val;`            (attributed friendly-name)
///   3. `RAW = val;`                         (raw const-name assignment)
///   4. `const RAW: ty = val;`               (full const item)
///   5. `#[attr] const RAW: ty = val;`       (attributed const item)
///
/// Non-matching items are skipped via shared TT-munching recursion arms.
/// When the token stream is exhausted without a hit, the preset default is
/// emitted.
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_sync_default {
    // ── S hit arms ──
    (@S, $p:ty; #[$a:meta] semaphores = $v:expr; $($r:tt)*) => {};
    (@S, $p:ty; semaphores = $v:expr; $($r:tt)*) => {};
    (@S, $p:ty; S = $v:expr; $($r:tt)*) => {};
    (@S, $p:ty; #[$a:meta] const S : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@S, $p:ty; const S : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── SW hit arms ──
    (@SW, $p:ty; #[$a:meta] semaphore_waitq = $v:expr; $($r:tt)*) => {};
    (@SW, $p:ty; semaphore_waitq = $v:expr; $($r:tt)*) => {};
    (@SW, $p:ty; SW = $v:expr; $($r:tt)*) => {};
    (@SW, $p:ty; #[$a:meta] const SW : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@SW, $p:ty; const SW : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── MS hit arms ──
    (@MS, $p:ty; #[$a:meta] mutexes = $v:expr; $($r:tt)*) => {};
    (@MS, $p:ty; mutexes = $v:expr; $($r:tt)*) => {};
    (@MS, $p:ty; MS = $v:expr; $($r:tt)*) => {};
    (@MS, $p:ty; #[$a:meta] const MS : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@MS, $p:ty; const MS : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── MW hit arms ──
    (@MW, $p:ty; #[$a:meta] mutex_waitq = $v:expr; $($r:tt)*) => {};
    (@MW, $p:ty; mutex_waitq = $v:expr; $($r:tt)*) => {};
    (@MW, $p:ty; MW = $v:expr; $($r:tt)*) => {};
    (@MW, $p:ty; #[$a:meta] const MW : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@MW, $p:ty; const MW : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── Skip: non-matching attributed friendly-name (shared) ──
    (@ $tag:tt, $p:ty; #[$a:meta] $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_sync_default!(@ $tag, $p; $($r)*);
    };
    // ── Skip: non-matching bare friendly-name (shared) ──
    (@ $tag:tt, $p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_sync_default!(@ $tag, $p; $($r)*);
    };
    // ── Skip: non-matching attributed const (shared) ──
    (@ $tag:tt, $p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_sync_default!(@ $tag, $p; $($r)*);
    };
    // ── Skip: non-matching bare const (shared) ──
    (@ $tag:tt, $p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_sync_default!(@ $tag, $p; $($r)*);
    };
    // ── Terminal: emit preset bridge ──
    (@S, $p:ty;) => {
        const S: usize = <$p as $crate::config::SyncConfig>::SEMAPHORES;
    };
    (@SW, $p:ty;) => {
        const SW: usize = <$p as $crate::config::SyncConfig>::SEMAPHORE_WAITQ;
    };
    (@MS, $p:ty;) => {
        const MS: usize = <$p as $crate::config::SyncConfig>::MUTEXES;
    };
    (@MW, $p:ty;) => {
        const MW: usize = <$p as $crate::config::SyncConfig>::MUTEX_WAITQ;
    };
}

/// Conditional bridge for **MsgConfig** — see [`_compose_sync_default!`].
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_msg_default {
    (@QS, $p:ty; #[$a:meta] queues = $v:expr; $($r:tt)*) => {};
    (@QS, $p:ty; queues = $v:expr; $($r:tt)*) => {};
    (@QS, $p:ty; QS = $v:expr; $($r:tt)*) => {};
    (@QS, $p:ty; #[$a:meta] const QS : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QS, $p:ty; const QS : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QD, $p:ty; #[$a:meta] queue_depth = $v:expr; $($r:tt)*) => {};
    (@QD, $p:ty; queue_depth = $v:expr; $($r:tt)*) => {};
    (@QD, $p:ty; QD = $v:expr; $($r:tt)*) => {};
    (@QD, $p:ty; #[$a:meta] const QD : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QD, $p:ty; const QD : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QM, $p:ty; #[$a:meta] max_msg_size = $v:expr; $($r:tt)*) => {};
    (@QM, $p:ty; max_msg_size = $v:expr; $($r:tt)*) => {};
    (@QM, $p:ty; QM = $v:expr; $($r:tt)*) => {};
    (@QM, $p:ty; #[$a:meta] const QM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QM, $p:ty; const QM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QW, $p:ty; #[$a:meta] queue_waitq = $v:expr; $($r:tt)*) => {};
    (@QW, $p:ty; queue_waitq = $v:expr; $($r:tt)*) => {};
    (@QW, $p:ty; QW = $v:expr; $($r:tt)*) => {};
    (@QW, $p:ty; #[$a:meta] const QW : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@QW, $p:ty; const QW : $ty:ty = $v:expr; $($r:tt)*) => {};
    // ── shared skip arms ──
    (@ $tag:tt, $p:ty; #[$a:meta] $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_msg_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_msg_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_msg_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_msg_default!(@ $tag, $p; $($r)*);
    };
    // ── terminal: emit preset bridge ──
    (@QS, $p:ty;) => { const QS: usize = <$p as $crate::config::MsgConfig>::QUEUES; };
    (@QD, $p:ty;) => { const QD: usize = <$p as $crate::config::MsgConfig>::QUEUE_DEPTH; };
    (@QM, $p:ty;) => { const QM: usize = <$p as $crate::config::MsgConfig>::MAX_MSG_SIZE; };
    (@QW, $p:ty;) => { const QW: usize = <$p as $crate::config::MsgConfig>::QUEUE_WAITQ; };
}

/// Conditional bridge for **PortsConfig** — see [`_compose_sync_default!`].
#[macro_export]
#[doc(hidden)]
macro_rules! _compose_ports_default {
    (@SP, $p:ty; #[$a:meta] sampling_ports = $v:expr; $($r:tt)*) => {};
    (@SP, $p:ty; sampling_ports = $v:expr; $($r:tt)*) => {};
    (@SP, $p:ty; SP = $v:expr; $($r:tt)*) => {};
    (@SP, $p:ty; #[$a:meta] const SP : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@SP, $p:ty; const SP : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@SM, $p:ty; #[$a:meta] sampling_msg_size = $v:expr; $($r:tt)*) => {};
    (@SM, $p:ty; sampling_msg_size = $v:expr; $($r:tt)*) => {};
    (@SM, $p:ty; SM = $v:expr; $($r:tt)*) => {};
    (@SM, $p:ty; #[$a:meta] const SM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@SM, $p:ty; const SM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BS, $p:ty; #[$a:meta] blackboards = $v:expr; $($r:tt)*) => {};
    (@BS, $p:ty; blackboards = $v:expr; $($r:tt)*) => {};
    (@BS, $p:ty; BS = $v:expr; $($r:tt)*) => {};
    (@BS, $p:ty; #[$a:meta] const BS : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BS, $p:ty; const BS : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BM, $p:ty; #[$a:meta] blackboard_msg_size = $v:expr; $($r:tt)*) => {};
    (@BM, $p:ty; blackboard_msg_size = $v:expr; $($r:tt)*) => {};
    (@BM, $p:ty; BM = $v:expr; $($r:tt)*) => {};
    (@BM, $p:ty; #[$a:meta] const BM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BM, $p:ty; const BM : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BW, $p:ty; #[$a:meta] blackboard_waitq = $v:expr; $($r:tt)*) => {};
    (@BW, $p:ty; blackboard_waitq = $v:expr; $($r:tt)*) => {};
    (@BW, $p:ty; BW = $v:expr; $($r:tt)*) => {};
    (@BW, $p:ty; #[$a:meta] const BW : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@BW, $p:ty; const BW : $ty:ty = $v:expr; $($r:tt)*) => {};
    (@ $tag:tt, $p:ty; #[$a:meta] $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_ports_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; $f:ident = $v:expr; $($r:tt)*) => {
        $crate::_compose_ports_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; #[$a:meta] const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_ports_default!(@ $tag, $p; $($r)*);
    };
    (@ $tag:tt, $p:ty; const $n:ident : $ty:ty = $v:expr; $($r:tt)*) => {
        $crate::_compose_ports_default!(@ $tag, $p; $($r)*);
    };
    // ── terminal: emit preset bridge ──
    (@SP, $p:ty;) => { const SP: usize = <$p as $crate::config::PortsConfig>::SAMPLING_PORTS; };
    (@SM, $p:ty;) => { const SM: usize = <$p as $crate::config::PortsConfig>::SAMPLING_MAX_MSG_SIZE; };
    (@BS, $p:ty;) => { const BS: usize = <$p as $crate::config::PortsConfig>::BLACKBOARDS; };
    (@BM, $p:ty;) => { const BM: usize = <$p as $crate::config::PortsConfig>::BLACKBOARD_MAX_MSG_SIZE; };
    (@BW, $p:ty;) => { const BW: usize = <$p as $crate::config::PortsConfig>::BLACKBOARD_WAITQ; };
}

#[macro_export]
#[doc(hidden)]
macro_rules! _kernel_config_body {
    () => {};
    ($field:ident = $v:expr; $($rest:tt)*) => {
        $crate::_kernel_config_field!($field = $v);
        $crate::_kernel_config_body!($($rest)*);
    };
    (const $name:ident : $ty:ty = $v:expr; $($rest:tt)*) => {
        const $name: $ty = $v;
        $crate::_kernel_config_body!($($rest)*);
    };
    (#[$attr:meta] const $name:ident : $ty:ty = $v:expr; $($rest:tt)*) => {
        #[$attr]
        const $name: $ty = $v;
        $crate::_kernel_config_body!($($rest)*);
    };
}

/// # Deprecated — use `compose_kernel_config!` instead
///
/// This macro is **soft-deprecated**. New code should use
/// [`compose_kernel_config!`], which builds a [`KernelConfig`] from
/// reusable sub-config presets with compile-time validation.
///
/// `kernel_config!` is retained as a **low-level escape hatch** for cases
/// that require direct `const` control over individual fields not covered
/// by any existing preset.
///
/// ---
///
/// Generates a config struct, `impl KernelConfig`, and the associated type
/// aliases from just the non-default overrides.
///
/// The macro also generates an inherent `impl` block that re-exports every
/// trait constant, so callers can write `MyConfig::SCHED` without importing
/// the [`KernelConfig`] trait.
///
/// # Forms
///
/// - `kernel_config!(Name { const N: usize = 2; ... })` — uses
///   [`AlignedStack1K`](crate::partition_core::AlignedStack1K).
/// - `kernel_config!(Name [StackType] { const N: usize = 2; ... })` — uses
///   the provided stack type.
///
/// The macro body accepts any items valid inside an `impl` block, including
/// `#[cfg(...)]` attributes on individual constants.
///
/// Doc-attributes (`///`) placed before the struct name are forwarded to
/// the generated struct definition.
#[macro_export]
macro_rules! kernel_config {
    ($(#[$meta:meta])* $vis:vis $name:ident { $($body:tt)* }) => {
        $(#[$meta])*
        $vis struct $name;
        impl $crate::config::KernelConfig for $name {
            $crate::_kernel_config_body!($($body)*);
            $crate::kernel_config_types!();
        }
        $crate::_kernel_config_inherent_consts!($vis $name);
        const _: () = $crate::config::assert_priority_order::<$name>();
        const _: () = $crate::config::assert_systick_reload::<$name>();
    };
    ($(#[$meta:meta])* $vis:vis $name:ident [$stack:ty] { $($body:tt)* }) => {
        $(#[$meta])*
        $vis struct $name;
        impl $crate::config::KernelConfig for $name {
            $crate::_kernel_config_body!($($body)*);
            $crate::kernel_config_types!($stack);
        }
        $crate::_kernel_config_inherent_consts!($vis $name);
        const _: () = $crate::config::assert_priority_order::<$name>();
        const _: () = $crate::config::assert_systick_reload::<$name>();
    };
}

/// Composes a [`KernelConfig`] from five sub-config preset types.
///
/// Each type parameter must implement the corresponding sub-config trait:
/// - `$parts`: [`PartitionConfig`]
/// - `$sync`: [`SyncConfig`]
/// - `$msg`: [`MsgConfig`]
/// - `$ports`: [`PortsConfig`]
/// - `$debug`: [`DebugConfig`]
///
/// An optional `{ field = value; ... }` block after the type parameters
/// overrides non-sub-config fields (e.g. `mpu_enforce`, `core_clock_hz`).
/// The body is processed by [`_kernel_config_body!`].
///
/// The macro generates a zero-sized struct, bridges every sub-config
/// constant to the flat [`KernelConfig`] constant, and calls
/// [`kernel_config_types!`] and [`_kernel_config_inherent_consts!`].
///
/// # Examples
///
/// Basic preset composition with no overrides:
///
/// ```ignore
/// use kernel::{compose_kernel_config, Partitions2, SyncMinimal,
///              MsgMinimal, PortsTiny, DebugEnabled};
///
/// compose_kernel_config!(
///     pub MyConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>
/// );
/// ```
///
/// Override non-sub-config fields with a trailing block:
///
/// ```ignore
/// use kernel::{compose_kernel_config, Partitions4, SyncStandard,
///              MsgStandard, PortsStandard, DebugEnabled};
///
/// compose_kernel_config!(
///     pub AppConfig<Partitions4, SyncStandard, MsgStandard,
///                   PortsStandard, DebugEnabled> {
///         mpu_enforce = true;
///         core_clock_hz = 64_000_000;
///     }
/// );
/// ```
#[macro_export]
macro_rules! compose_kernel_config {
    // With custom stack type and override block.
    ($(#[$meta:meta])* $vis:vis $name:ident [$stack:ty] < $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty > { $($overrides:tt)* }) => {
        $(#[$meta])*
        $vis struct $name;

        impl $crate::config::KernelConfig for $name {
            $crate::_compose_partition_n_default!($parts; $($overrides)*);
            $crate::_compose_partition_sched_default!($parts; $($overrides)*);
            $crate::_compose_partition_stack_default!($parts; $($overrides)*);
            $crate::_compose_sync_default!(@S, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@SW, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@MS, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@MW, $sync; $($overrides)*);
            $crate::_compose_msg_default!(@QS, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QD, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QM, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QW, $msg; $($overrides)*);
            $crate::_compose_ports_default!(@SP, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@SM, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BS, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BM, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BW, $ports; $($overrides)*);
            $crate::_compose_debug_default!(@DBS, $debug; $($overrides)*);
            $crate::_compose_debug_default!(@DAD, $debug; $($overrides)*);
            $crate::_kernel_config_body!($($overrides)*);
            $crate::kernel_config_types!(@cfg $parts, $sync, $msg, $ports, $debug; $stack);
        }
        $crate::_kernel_config_inherent_consts!($vis $name);
        const _: () = $crate::config::assert_priority_order::<$name>();
        const _: () = $crate::config::assert_systick_reload::<$name>();
    };
    // With custom stack type, no override block.
    ($(#[$meta:meta])* $vis:vis $name:ident [$stack:ty] < $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty >) => {
        $crate::compose_kernel_config!($(#[$meta])* $vis $name [$stack] < $parts, $sync, $msg, $ports, $debug > {});
    };
    // Default stack with override block.
    ($(#[$meta:meta])* $vis:vis $name:ident < $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty > { $($overrides:tt)* }) => {
        $(#[$meta])*
        $vis struct $name;

        impl $crate::config::KernelConfig for $name {
            // PartitionConfig — conditionally bridged so overrides can replace them.
            $crate::_compose_partition_n_default!($parts; $($overrides)*);
            $crate::_compose_partition_sched_default!($parts; $($overrides)*);
            $crate::_compose_partition_stack_default!($parts; $($overrides)*);
            // SyncConfig — conditionally bridged so overrides can replace them.
            $crate::_compose_sync_default!(@S, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@SW, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@MS, $sync; $($overrides)*);
            $crate::_compose_sync_default!(@MW, $sync; $($overrides)*);

            // MsgConfig — conditionally bridged so overrides can replace them.
            $crate::_compose_msg_default!(@QS, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QD, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QM, $msg; $($overrides)*);
            $crate::_compose_msg_default!(@QW, $msg; $($overrides)*);
            // PortsConfig — conditionally bridged so overrides can replace them.
            $crate::_compose_ports_default!(@SP, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@SM, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BS, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BM, $ports; $($overrides)*);
            $crate::_compose_ports_default!(@BW, $ports; $($overrides)*);
            // DebugConfig — conditionally bridged so overrides can replace them.
            $crate::_compose_debug_default!(@DBS, $debug; $($overrides)*);
            $crate::_compose_debug_default!(@DAD, $debug; $($overrides)*);

            // Non-sub-config overrides
            $crate::_kernel_config_body!($($overrides)*);

            $crate::kernel_config_types!(@cfg $parts, $sync, $msg, $ports, $debug);
        }
        $crate::_kernel_config_inherent_consts!($vis $name);
        const _: () = $crate::config::assert_priority_order::<$name>();
        const _: () = $crate::config::assert_systick_reload::<$name>();
    };
    // Default stack, no override block.
    ($(#[$meta:meta])* $vis:vis $name:ident < $parts:ty, $sync:ty, $msg:ty, $ports:ty, $debug:ty >) => {
        $crate::compose_kernel_config!($(#[$meta])* $vis $name < $parts, $sync, $msg, $ports, $debug > {});
    };
}

// ── DefaultConfig: sensible defaults for quick prototyping ──────────────

compose_kernel_config!(pub DefaultConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

impl Default for DefaultConfig {
    fn default() -> Self {
        Self
    }
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

    #[test]
    fn compute_systick_reload_min_one_cycle() {
        // Boundary: minimum valid configuration — exactly 1 cycle, reload = 0
        // 1 MHz clock, 1 us period → (1_000_000 * 1) / 1_000_000 = 1 cycle
        const RELOAD: u32 = compute_systick_reload(1_000_000, 1);
        assert_eq!(RELOAD, 0);
    }

    #[test]
    fn compute_systick_reload_high_frequency_clock() {
        // 480 MHz clock (Cortex-M7 high-performance), 1 ms tick
        // cycles = 480_000_000 * 1000 / 1_000_000 = 480_000, reload = 479_999
        const RELOAD: u32 = compute_systick_reload(480_000_000, 1000);
        assert_eq!(RELOAD, 479_999);
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
    struct DefaultPriority;
    impl KernelConfig for DefaultPriority {
        const N: usize = 2; // N has no default - must be specified
        kernel_config_types!();
    }

    /// Config that overrides priorities (uses default resource pool constants).
    struct CustomPriority;
    impl KernelConfig for CustomPriority {
        const N: usize = 2;
        const PENDSV_PRIORITY: u8 = 0xE0;
        const SYSTICK_PRIORITY: u8 = 0x10;
        const IRQ_DEFAULT_PRIORITY: u8 = 0x80;
        const CORE_CLOCK_HZ: u32 = 80_000_000; // 10 ms tick at 80 MHz
        const TICK_PERIOD_US: u32 = 10_000;
        #[cfg(feature = "partition-debug")]
        const DEBUG_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "dynamic-mpu")]
        const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 = 200;

        kernel_config_types!();
    }

    #[test]
    fn default_priorities_have_correct_values() {
        assert_eq!(DefaultPriority::SVCALL_PRIORITY, 0x00);
        assert_eq!(DefaultPriority::PENDSV_PRIORITY, 0xFF);
        assert_eq!(DefaultPriority::SYSTICK_PRIORITY, 0x10);
        assert_eq!(DefaultPriority::IRQ_DEFAULT_PRIORITY, 0xC0);
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
        assert_eq!(CustomPriority::SYSTICK_PRIORITY, 0x10);
        assert_eq!(CustomPriority::IRQ_DEFAULT_PRIORITY, 0x80);
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
        kernel_config_types!();
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
        kernel_config_types!();
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

    // ============ MPU_ENFORCE tests ============

    #[test]
    fn default_mpu_enforce_is_false() {
        let val = DefaultPriority::MPU_ENFORCE;
        assert!(!val);
    }

    /// Config that enables MPU enforcement.
    struct MpuEnabledConfig;
    impl KernelConfig for MpuEnabledConfig {
        const N: usize = 2;
        const MPU_ENFORCE: bool = true;
        kernel_config_types!();
    }

    #[test]
    fn mpu_enabled_config_overrides_default() {
        let val = MpuEnabledConfig::MPU_ENFORCE;
        assert!(val);
    }

    // Compile-time const assertion for MpuEnabledConfig.
    const _: () = assert!(MpuEnabledConfig::MPU_ENFORCE);
    const _: () = assert!(!DefaultPriority::MPU_ENFORCE);
    const _: () = assert_priority_order::<MpuEnabledConfig>();
    const _: () = assert_systick_reload::<MpuEnabledConfig>();

    // ============ Sub-config associated type consistency tests ============

    /// Verify that each sub-config associated type's constants match the
    /// corresponding raw KernelConfig constants on DefaultPriority.
    #[test]
    fn sub_config_types_match_raw_constants() {
        type D = DefaultPriority;
        // PartitionCfg
        assert_eq!(<D as KernelConfig>::PartitionCfg::COUNT, D::N);
        assert_eq!(
            <D as KernelConfig>::PartitionCfg::SCHEDULE_CAPACITY,
            D::SCHED
        );
        assert_eq!(
            <D as KernelConfig>::PartitionCfg::STACK_WORDS,
            D::STACK_WORDS
        );
        // SyncCfg
        assert_eq!(<D as KernelConfig>::SyncCfg::SEMAPHORES, D::S);
        assert_eq!(<D as KernelConfig>::SyncCfg::MUTEXES, D::MS);
        // MsgCfg
        assert_eq!(<D as KernelConfig>::MsgCfg::QUEUES, D::QS);
        assert_eq!(<D as KernelConfig>::MsgCfg::MAX_MSG_SIZE, D::QM);
        // PortsCfg
        assert_eq!(<D as KernelConfig>::PortsCfg::SAMPLING_PORTS, D::SP);
        assert_eq!(<D as KernelConfig>::PortsCfg::BLACKBOARDS, D::BS);
        // DebugCfg
        assert_eq!(
            <D as KernelConfig>::DebugCfg::AUTO_DRAIN_BUDGET,
            D::DEBUG_AUTO_DRAIN_BUDGET
        );
    }

    // ============ PartitionConfig tests ============

    #[test]
    fn partitions2_count_is_2() {
        assert_eq!(Partitions2::COUNT, 2);
    }

    #[test]
    fn partitions2_schedule_capacity_is_4() {
        assert_eq!(Partitions2::SCHEDULE_CAPACITY, 4);
    }

    #[test]
    fn partitions2_stack_words_is_256() {
        assert_eq!(Partitions2::STACK_WORDS, 256);
    }

    // Compile-time assertions for Partitions2 preset values.
    const _: () = assert!(Partitions2::COUNT == 2);
    const _: () = assert!(Partitions2::SCHEDULE_CAPACITY == 4);
    const _: () = assert!(Partitions2::STACK_WORDS == 256);

    // Compile-time assertions for Partitions3 preset values.
    const _: () = assert!(Partitions3::COUNT == 3);
    const _: () = assert!(Partitions3::SCHEDULE_CAPACITY == 8);
    const _: () = assert!(Partitions3::STACK_WORDS == 256);

    #[test]
    fn partitions3_field_values() {
        assert_eq!(Partitions3::COUNT, 3);
        assert_eq!(Partitions3::SCHEDULE_CAPACITY, 8);
        assert_eq!(Partitions3::STACK_WORDS, 256);
    }

    // Compile-time assertions for Partitions4 preset values.
    const _: () = assert!(Partitions4::COUNT == 4);
    const _: () = assert!(Partitions4::SCHEDULE_CAPACITY == 8);
    const _: () = assert!(Partitions4::STACK_WORDS == 256);

    #[test]
    fn partitions4_field_values() {
        assert_eq!(Partitions4::COUNT, 4);
        assert_eq!(Partitions4::SCHEDULE_CAPACITY, 8);
        assert_eq!(Partitions4::STACK_WORDS, 256);
    }

    // ============ SyncConfig tests ============

    #[test]
    fn sync_minimal_semaphores_is_1() {
        assert_eq!(SyncMinimal::SEMAPHORES, 1);
    }

    #[test]
    fn sync_minimal_semaphore_waitq_is_1() {
        assert_eq!(SyncMinimal::SEMAPHORE_WAITQ, 1);
    }

    #[test]
    fn sync_minimal_mutexes_is_1() {
        assert_eq!(SyncMinimal::MUTEXES, 1);
    }

    #[test]
    fn sync_minimal_mutex_waitq_is_1() {
        assert_eq!(SyncMinimal::MUTEX_WAITQ, 1);
    }

    #[test]
    fn sync_rich_semaphores_is_8() {
        assert_eq!(SyncRich::SEMAPHORES, 8);
    }

    #[test]
    fn sync_rich_semaphore_waitq_is_4() {
        assert_eq!(SyncRich::SEMAPHORE_WAITQ, 4);
    }

    #[test]
    fn sync_rich_mutexes_is_4() {
        assert_eq!(SyncRich::MUTEXES, 4);
    }

    #[test]
    fn sync_rich_mutex_waitq_is_4() {
        assert_eq!(SyncRich::MUTEX_WAITQ, 4);
    }

    // Compile-time assertions for SyncMinimal preset values.
    const _: () = assert!(SyncMinimal::SEMAPHORES == 1);
    const _: () = assert!(SyncMinimal::SEMAPHORE_WAITQ == 1);
    const _: () = assert!(SyncMinimal::MUTEXES == 1);
    const _: () = assert!(SyncMinimal::MUTEX_WAITQ == 1);

    // Compile-time assertions for SyncRich preset values.
    const _: () = assert!(SyncRich::SEMAPHORES == 8);
    const _: () = assert!(SyncRich::SEMAPHORE_WAITQ == 4);
    const _: () = assert!(SyncRich::MUTEXES == 4);
    const _: () = assert!(SyncRich::MUTEX_WAITQ == 4);

    #[test]
    fn sync_standard_field_values() {
        assert_eq!(SyncStandard::SEMAPHORES, 4);
        assert_eq!(SyncStandard::SEMAPHORE_WAITQ, 4);
        assert_eq!(SyncStandard::MUTEXES, 4);
        assert_eq!(SyncStandard::MUTEX_WAITQ, 4);
    }

    // ============ MsgConfig tests ============

    // Compile-time assertions for MsgMinimal preset values.
    const _: () = assert!(MsgMinimal::QUEUES == 1);
    const _: () = assert!(MsgMinimal::QUEUE_DEPTH == 1);
    const _: () = assert!(MsgMinimal::MAX_MSG_SIZE == 1);
    const _: () = assert!(MsgMinimal::QUEUE_WAITQ == 1);

    // Compile-time assertions for MsgSmall preset values.
    const _: () = assert!(MsgSmall::QUEUES == 2);
    const _: () = assert!(MsgSmall::QUEUE_DEPTH == 4);
    const _: () = assert!(MsgSmall::MAX_MSG_SIZE == 4);
    const _: () = assert!(MsgSmall::QUEUE_WAITQ == 2);

    #[test]
    fn msg_small_field_values() {
        assert_eq!(MsgSmall::QUEUES, 2);
        assert_eq!(MsgSmall::QUEUE_DEPTH, 4);
        assert_eq!(MsgSmall::MAX_MSG_SIZE, 4);
        assert_eq!(MsgSmall::QUEUE_WAITQ, 2);
    }

    // Compile-time assertions for MsgRich preset values.
    const _: () = assert!(MsgRich::QUEUES == 4);
    const _: () = assert!(MsgRich::QUEUE_DEPTH == 4);
    const _: () = assert!(MsgRich::MAX_MSG_SIZE == 64);
    const _: () = assert!(MsgRich::QUEUE_WAITQ == 4);

    #[test]
    fn msg_rich_field_values() {
        assert_eq!(MsgRich::QUEUES, 4);
        assert_eq!(MsgRich::QUEUE_DEPTH, 4);
        assert_eq!(MsgRich::MAX_MSG_SIZE, 64);
        assert_eq!(MsgRich::QUEUE_WAITQ, 4);
    }

    #[test]
    fn msg_standard_field_values() {
        assert_eq!(MsgStandard::QUEUES, 4);
        assert_eq!(MsgStandard::QUEUE_DEPTH, 4);
        assert_eq!(MsgStandard::MAX_MSG_SIZE, 4);
        assert_eq!(MsgStandard::QUEUE_WAITQ, 4);
    }

    // ============ PortsConfig tests ============

    // Compile-time assertions for PortsMinimal preset values.
    const _: () = assert!(PortsMinimal::SAMPLING_PORTS == 1);
    const _: () = assert!(PortsMinimal::SAMPLING_MAX_MSG_SIZE == 64);
    const _: () = assert!(PortsMinimal::BLACKBOARDS == 1);
    const _: () = assert!(PortsMinimal::BLACKBOARD_MAX_MSG_SIZE == 64);
    const _: () = assert!(PortsMinimal::BLACKBOARD_WAITQ == 1);

    // Compile-time assertions for PortsTiny preset values.
    const _: () = assert!(PortsTiny::SAMPLING_PORTS == 1);
    const _: () = assert!(PortsTiny::SAMPLING_MAX_MSG_SIZE == 1);
    const _: () = assert!(PortsTiny::BLACKBOARDS == 1);
    const _: () = assert!(PortsTiny::BLACKBOARD_MAX_MSG_SIZE == 1);
    const _: () = assert!(PortsTiny::BLACKBOARD_WAITQ == 1);

    #[test]
    fn ports_tiny_field_values() {
        assert_eq!(PortsTiny::SAMPLING_PORTS, 1);
        assert_eq!(PortsTiny::SAMPLING_MAX_MSG_SIZE, 1);
        assert_eq!(PortsTiny::BLACKBOARDS, 1);
        assert_eq!(PortsTiny::BLACKBOARD_MAX_MSG_SIZE, 1);
        assert_eq!(PortsTiny::BLACKBOARD_WAITQ, 1);
    }

    // Compile-time assertions for PortsRich preset values.
    const _: () = assert!(PortsRich::SAMPLING_PORTS == 4);
    const _: () = assert!(PortsRich::SAMPLING_MAX_MSG_SIZE == 64);
    const _: () = assert!(PortsRich::BLACKBOARDS == 4);
    const _: () = assert!(PortsRich::BLACKBOARD_MAX_MSG_SIZE == 64);
    const _: () = assert!(PortsRich::BLACKBOARD_WAITQ == 4);

    #[test]
    fn ports_rich_field_values() {
        assert_eq!(PortsRich::SAMPLING_PORTS, 4);
        assert_eq!(PortsRich::SAMPLING_MAX_MSG_SIZE, 64);
        assert_eq!(PortsRich::BLACKBOARDS, 4);
        assert_eq!(PortsRich::BLACKBOARD_MAX_MSG_SIZE, 64);
        assert_eq!(PortsRich::BLACKBOARD_WAITQ, 4);
    }

    #[test]
    fn ports_small_field_values() {
        assert_eq!(PortsSmall::SAMPLING_PORTS, 4);
        assert_eq!(PortsSmall::SAMPLING_MAX_MSG_SIZE, 4);
        assert_eq!(PortsSmall::BLACKBOARDS, 4);
        assert_eq!(PortsSmall::BLACKBOARD_MAX_MSG_SIZE, 4);
        assert_eq!(PortsSmall::BLACKBOARD_WAITQ, 4);
    }

    // ============ DebugConfig tests ============

    // Compile-time assertions for DebugEnabled preset values.
    const _: () = assert!(DebugEnabled::BUFFER_SIZE == 256);
    const _: () = assert!(DebugEnabled::AUTO_DRAIN_BUDGET == 256);

    // Compile-time assertions for DebugDisabled preset values.
    const _: () = assert!(DebugDisabled::BUFFER_SIZE == 0);
    const _: () = assert!(DebugDisabled::AUTO_DRAIN_BUDGET == 0);

    #[test]
    fn debug_enabled_field_values() {
        assert_eq!(DebugEnabled::BUFFER_SIZE, 256);
        assert_eq!(DebugEnabled::AUTO_DRAIN_BUDGET, 256);
    }

    #[test]
    fn debug_disabled_field_values() {
        assert_eq!(DebugDisabled::BUFFER_SIZE, 0);
        assert_eq!(DebugDisabled::AUTO_DRAIN_BUDGET, 0);
    }

    // ============ kernel_config! macro tests ============

    kernel_config!(MacroDefaultsOnly { const N: usize = 2; });

    #[test]
    fn kernel_config_macro_defaults_only() {
        assert_eq!(MacroDefaultsOnly::N, 2);
        assert_eq!(MacroDefaultsOnly::SCHED, 4);
        assert_eq!(MacroDefaultsOnly::STACK_WORDS, 256);
        assert_eq!(MacroDefaultsOnly::S, 1);
        assert_eq!(MacroDefaultsOnly::SW, 1);
        assert_eq!(MacroDefaultsOnly::MS, 1);
        assert_eq!(MacroDefaultsOnly::MW, 1);
        assert_eq!(MacroDefaultsOnly::QS, 1);
        assert_eq!(MacroDefaultsOnly::QD, 1);
        assert_eq!(MacroDefaultsOnly::QM, 1);
        assert_eq!(MacroDefaultsOnly::QW, 1);
        assert_eq!(MacroDefaultsOnly::CORE_CLOCK_HZ, 12_000_000);
        assert_eq!(MacroDefaultsOnly::TICK_PERIOD_US, 1000);
    }

    kernel_config!(MacroWithOverrides {
        const N: usize = 4;
        const SCHED: usize = 8;
        const S: usize = 4;
        const SW: usize = 2;
        const CORE_CLOCK_HZ: u32 = 64_000_000;
    });

    #[test]
    fn kernel_config_macro_with_overrides() {
        assert_eq!(MacroWithOverrides::N, 4);
        assert_eq!(MacroWithOverrides::SCHED, 8);
        assert_eq!(MacroWithOverrides::S, 4);
        assert_eq!(MacroWithOverrides::SW, 2);
        assert_eq!(MacroWithOverrides::CORE_CLOCK_HZ, 64_000_000);
        // Non-overridden constants retain defaults.
        assert_eq!(MacroWithOverrides::STACK_WORDS, 256);
        assert_eq!(MacroWithOverrides::MS, 1);
        assert_eq!(MacroWithOverrides::MW, 1);
        assert_eq!(MacroWithOverrides::TICK_PERIOD_US, 1000);
    }

    kernel_config!(MacroCustomStack [crate::partition_core::AlignedStack4K] {
        const N: usize = 2;
        const STACK_WORDS: usize = 1024;
    });

    #[test]
    fn kernel_config_macro_custom_stack() {
        assert_eq!(MacroCustomStack::N, 2);
        assert_eq!(MacroCustomStack::STACK_WORDS, 1024);
        // Defaults still apply for non-overridden constants.
        assert_eq!(MacroCustomStack::SCHED, 4);
        assert_eq!(MacroCustomStack::S, 1);
        // Verify the type compiles by constructing the Core type.
        let _core = <MacroCustomStack as KernelConfig>::Core::default();
    }

    // ============ kernel_config! visibility modifier tests ============
    //
    // These tests use a child module to verify that visibility modifiers are
    // actually forwarded.  A `pub` config and its constants should be
    // accessible from a sibling/parent module, while a private (default-vis)
    // config should not leak out of the defining module.

    /// Child module that defines configs with different visibilities.
    mod vis_test {
        // Public config — struct and inherent consts are `pub`.
        kernel_config!(pub PubConfig { const N: usize = 2; });

        // `pub(crate)` config.
        kernel_config!(pub(crate) PubCrateConfig { const N: usize = 3; });

        // `pub` config with custom stack type.
        kernel_config!(pub PubStackConfig [crate::partition_core::AlignedStack4K] {
            const N: usize = 2;
            const STACK_WORDS: usize = 1024;
        });

        // `pub(crate)` config with custom stack type.
        kernel_config!(pub(crate) PubCrateStackConfig [crate::partition_core::AlignedStack4K] {
            const N: usize = 4;
            const STACK_WORDS: usize = 1024;
        });

        // Private (default) config — should only be accessible within this module.
        kernel_config!(PrivateConfig { const N: usize = 5; });

        #[test]
        fn private_config_accessible_inside_defining_module() {
            assert_eq!(PrivateConfig::N, 5);
            assert_eq!(PrivateConfig::SCHED, 4);
        }
    }

    /// Access `pub` and `pub(crate)` configs from the parent module to prove
    /// visibility is actually forwarded through the struct *and* the inherent
    /// constants.
    #[test]
    fn pub_config_accessible_from_parent() {
        assert_eq!(vis_test::PubConfig::N, 2);
        assert_eq!(vis_test::PubConfig::SCHED, 4);
        assert_eq!(vis_test::PubConfig::CORE_CLOCK_HZ, 12_000_000);
    }

    #[test]
    fn pub_crate_config_accessible_from_parent() {
        assert_eq!(vis_test::PubCrateConfig::N, 3);
        assert_eq!(vis_test::PubCrateConfig::SCHED, 4);
        assert_eq!(vis_test::PubCrateConfig::CORE_CLOCK_HZ, 12_000_000);
    }

    #[test]
    fn pub_stack_config_accessible_from_parent() {
        assert_eq!(vis_test::PubStackConfig::N, 2);
        assert_eq!(vis_test::PubStackConfig::STACK_WORDS, 1024);
        assert_eq!(vis_test::PubStackConfig::SCHED, 4);
    }

    #[test]
    fn pub_crate_stack_config_accessible_from_parent() {
        assert_eq!(vis_test::PubCrateStackConfig::N, 4);
        assert_eq!(vis_test::PubCrateStackConfig::STACK_WORDS, 1024);
        assert_eq!(vis_test::PubCrateStackConfig::SCHED, 4);
    }

    // NOTE: `vis_test::PrivateConfig` is intentionally *not* referenced here.
    // It is private to `vis_test` and would fail to compile if accessed from
    // this parent module, confirming that default (private) visibility works.
    // TODO: add a compile_fail doctest for PrivateConfig once trybuild or
    // compile_fail infrastructure is set up for this crate.
    struct FieldMacroConfig;
    impl KernelConfig for FieldMacroConfig {
        _kernel_config_field!(partitions = 3);
        _kernel_config_field!(schedule_capacity = 6);
        _kernel_config_field!(stack_words = 128);
        _kernel_config_field!(semaphores = 2);
        _kernel_config_field!(semaphore_waitq = 2);
        _kernel_config_field!(mutexes = 2);
        _kernel_config_field!(mutex_waitq = 2);
        _kernel_config_field!(queues = 2);
        _kernel_config_field!(queue_depth = 4);
        _kernel_config_field!(max_msg_size = 32);
        _kernel_config_field!(queue_waitq = 2);
        _kernel_config_field!(sampling_ports = 2);
        _kernel_config_field!(sampling_msg_size = 32);
        _kernel_config_field!(blackboards = 2);
        _kernel_config_field!(blackboard_msg_size = 32);
        _kernel_config_field!(blackboard_waitq = 2);
        _kernel_config_field!(core_clock_hz = 48_000_000);
        _kernel_config_field!(tick_period_us = 500);
        _kernel_config_field!(use_processor_clock = false);
        _kernel_config_field!(mpu_enforce = true);
        _kernel_config_field!(svcall_priority = 0x00);
        _kernel_config_field!(pendsv_priority = 0xFF);
        _kernel_config_field!(systick_priority = 0x10);
        _kernel_config_field!(irq_priority = 0x80);
        _kernel_config_field!(min_app_irq_priority = 0x30);
        _kernel_config_field!(debug_auto_drain = 512);
        kernel_config_types!();
    }
    #[test]
    fn kernel_config_field_macro_expands_correctly() {
        assert_eq!(FieldMacroConfig::N, 3);
        assert_eq!(FieldMacroConfig::SCHED, 6);
        assert_eq!(FieldMacroConfig::STACK_WORDS, 128);
        assert_eq!(FieldMacroConfig::S, 2);
        assert_eq!(FieldMacroConfig::SW, 2);
        assert_eq!(FieldMacroConfig::MS, 2);
        assert_eq!(FieldMacroConfig::MW, 2);
        assert_eq!(FieldMacroConfig::QS, 2);
        assert_eq!(FieldMacroConfig::QD, 4);
        assert_eq!(FieldMacroConfig::QM, 32);
        assert_eq!(FieldMacroConfig::QW, 2);
        assert_eq!(FieldMacroConfig::SP, 2);
        assert_eq!(FieldMacroConfig::SM, 32);
        assert_eq!(FieldMacroConfig::BS, 2);
        assert_eq!(FieldMacroConfig::BM, 32);
        assert_eq!(FieldMacroConfig::BW, 2);
        assert_eq!(FieldMacroConfig::CORE_CLOCK_HZ, 48_000_000);
        assert_eq!(FieldMacroConfig::TICK_PERIOD_US, 500);
        let upc = FieldMacroConfig::USE_PROCESSOR_CLOCK;
        assert!(!upc);
        let mpu = FieldMacroConfig::MPU_ENFORCE;
        assert!(mpu);
        assert_eq!(FieldMacroConfig::SVCALL_PRIORITY, 0x00);
        assert_eq!(FieldMacroConfig::PENDSV_PRIORITY, 0xFF);
        assert_eq!(FieldMacroConfig::SYSTICK_PRIORITY, 0x10);
        assert_eq!(FieldMacroConfig::IRQ_DEFAULT_PRIORITY, 0x80);
        assert_eq!(FieldMacroConfig::MIN_APP_IRQ_PRIORITY, 0x30);
        assert_eq!(FieldMacroConfig::DEBUG_AUTO_DRAIN_BUDGET, 512);
    }

    // ============ _kernel_config_body! TT-muncher tests ============

    kernel_config!(FieldSyntaxConfig {
        partitions = 3;
        schedule_capacity = 6;
        stack_words = 128;
        semaphores = 2;
        semaphore_waitq = 2;
        mutexes = 3;
        mutex_waitq = 3;
        core_clock_hz = 48_000_000;
        tick_period_us = 500;
    });

    #[test]
    fn field_syntax_config_values() {
        assert_eq!(FieldSyntaxConfig::N, 3);
        assert_eq!(FieldSyntaxConfig::SCHED, 6);
        assert_eq!(FieldSyntaxConfig::STACK_WORDS, 128);
        assert_eq!(FieldSyntaxConfig::S, 2);
        assert_eq!(FieldSyntaxConfig::SW, 2);
        assert_eq!(FieldSyntaxConfig::MS, 3);
        assert_eq!(FieldSyntaxConfig::MW, 3);
        assert_eq!(FieldSyntaxConfig::CORE_CLOCK_HZ, 48_000_000);
        assert_eq!(FieldSyntaxConfig::TICK_PERIOD_US, 500);
        // Non-overridden fields retain defaults.
        assert_eq!(FieldSyntaxConfig::QS, 1);
        assert_eq!(FieldSyntaxConfig::SP, 1);
    }

    kernel_config!(MixedSyntaxConfig {
        partitions = 2;
        const SCHED: usize = 4;
        semaphores = 8;
        const CORE_CLOCK_HZ: u32 = 64_000_000;
    });

    #[test]
    fn mixed_syntax_config_values() {
        assert_eq!(MixedSyntaxConfig::N, 2);
        assert_eq!(MixedSyntaxConfig::SCHED, 4);
        assert_eq!(MixedSyntaxConfig::S, 8);
        assert_eq!(MixedSyntaxConfig::CORE_CLOCK_HZ, 64_000_000);
        // Non-overridden fields retain defaults.
        assert_eq!(MixedSyntaxConfig::STACK_WORDS, 256);
        assert_eq!(MixedSyntaxConfig::MS, 1);
        assert_eq!(MixedSyntaxConfig::TICK_PERIOD_US, 1000);
    }

    // ============ Feature-gated field alias tests ============

    struct FeatureGatedFieldConfig;
    impl KernelConfig for FeatureGatedFieldConfig {
        _kernel_config_field!(partitions = 2);
        _kernel_config_field!(buffer_pool_regions = 8);
        _kernel_config_field!(buffer_zone_size = 64);
        _kernel_config_field!(dynamic_regions = 6);
        _kernel_config_field!(system_window_max_gap_ticks = 50);
        _kernel_config_field!(debug_buffer_size = 512);
        kernel_config_types!();
    }

    #[test]
    fn feature_gated_field_aliases_expand_correctly() {
        assert_eq!(FeatureGatedFieldConfig::N, 2);
        #[cfg(feature = "dynamic-mpu")]
        {
            assert_eq!(FeatureGatedFieldConfig::BP, 8);
            assert_eq!(FeatureGatedFieldConfig::BZ, 64);
            assert_eq!(FeatureGatedFieldConfig::DR, 6);
            assert_eq!(FeatureGatedFieldConfig::SYSTEM_WINDOW_MAX_GAP_TICKS, 50);
        }
        #[cfg(feature = "partition-debug")]
        {
            assert_eq!(FeatureGatedFieldConfig::DEBUG_BUFFER_SIZE, 512);
        }
    }

    struct FieldMsgSizeConfig;
    impl KernelConfig for FieldMsgSizeConfig {
        _kernel_config_field!(partitions = 2);
        _kernel_config_field!(sampling_msg_size = 128);
        _kernel_config_field!(blackboard_msg_size = 256);
        kernel_config_types!();
    }

    #[test]
    fn msg_size_field_aliases_expand_correctly() {
        assert_eq!(FieldMsgSizeConfig::SM, 128);
        assert_eq!(FieldMsgSizeConfig::BM, 256);
    }

    kernel_config!(FeatureGatedE2EConfig {
        partitions = 2;
        semaphores = 4;
        buffer_pool_regions = 8;
        buffer_zone_size = 64;
        dynamic_regions = 6;
        system_window_max_gap_ticks = 50;
        debug_buffer_size = 512;
        core_clock_hz = 48_000_000;
    });

    #[test]
    fn feature_gated_kernel_config_e2e() {
        assert_eq!(FeatureGatedE2EConfig::N, 2);
        assert_eq!(FeatureGatedE2EConfig::S, 4);
        assert_eq!(FeatureGatedE2EConfig::CORE_CLOCK_HZ, 48_000_000);
        // Non-overridden fields retain defaults.
        assert_eq!(FeatureGatedE2EConfig::SCHED, 4);
        assert_eq!(FeatureGatedE2EConfig::STACK_WORDS, 256);
        #[cfg(feature = "dynamic-mpu")]
        {
            assert_eq!(FeatureGatedE2EConfig::BP, 8);
            assert_eq!(FeatureGatedE2EConfig::BZ, 64);
            assert_eq!(FeatureGatedE2EConfig::DR, 6);
            assert_eq!(FeatureGatedE2EConfig::SYSTEM_WINDOW_MAX_GAP_TICKS, 50);
        }
        #[cfg(feature = "partition-debug")]
        {
            assert_eq!(FeatureGatedE2EConfig::DEBUG_BUFFER_SIZE, 512);
        }
    }

    // ============ Partitions1 preset tests ============

    #[test]
    fn partitions1_preset_values() {
        assert_eq!(Partitions1::COUNT, 1);
        assert_eq!(Partitions1::SCHEDULE_CAPACITY, 4);
        assert_eq!(Partitions1::STACK_WORDS, 256);
    }

    // ============ compose_kernel_config! tests ============

    compose_kernel_config!(ComposedConfig<Partitions2, SyncRich, MsgRich, PortsRich, DebugDisabled>);

    #[test]
    fn composed_bridges_all_sub_configs() {
        // PartitionConfig
        assert_eq!(ComposedConfig::N, Partitions2::COUNT);
        assert_eq!(ComposedConfig::SCHED, Partitions2::SCHEDULE_CAPACITY);
        // SyncConfig
        assert_eq!(ComposedConfig::S, SyncRich::SEMAPHORES);
        assert_eq!(ComposedConfig::SW, SyncRich::SEMAPHORE_WAITQ);
        assert_eq!(ComposedConfig::MS, SyncRich::MUTEXES);
        assert_eq!(ComposedConfig::MW, SyncRich::MUTEX_WAITQ);
        // MsgConfig
        assert_eq!(ComposedConfig::QS, MsgRich::QUEUES);
        assert_eq!(ComposedConfig::QD, MsgRich::QUEUE_DEPTH);
        assert_eq!(ComposedConfig::QM, MsgRich::MAX_MSG_SIZE);
        assert_eq!(ComposedConfig::QW, MsgRich::QUEUE_WAITQ);
        // PortsConfig
        assert_eq!(ComposedConfig::SP, PortsRich::SAMPLING_PORTS);
        assert_eq!(ComposedConfig::SM, PortsRich::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(ComposedConfig::BS, PortsRich::BLACKBOARDS);
        assert_eq!(ComposedConfig::BM, PortsRich::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(ComposedConfig::BW, PortsRich::BLACKBOARD_WAITQ);
        // DebugConfig
        assert_eq!(
            ComposedConfig::DEBUG_AUTO_DRAIN_BUDGET,
            DebugDisabled::AUTO_DRAIN_BUDGET
        );
        // Non-overridden defaults
        assert_eq!(ComposedConfig::CORE_CLOCK_HZ, 12_000_000);
        assert_eq!(ComposedConfig::TICK_PERIOD_US, 1000);
    }

    compose_kernel_config!(ComposedP1<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

    #[test]
    fn composed_with_partitions1() {
        assert_eq!(ComposedP1::N, 1);
        assert_eq!(ComposedP1::SCHED, 4);
        assert_eq!(ComposedP1::STACK_WORDS, 256);
        assert_eq!(ComposedP1::S, SyncMinimal::SEMAPHORES);
        assert_eq!(ComposedP1::SP, PortsTiny::SAMPLING_PORTS);
        assert_eq!(ComposedP1::BS, PortsTiny::BLACKBOARDS);
        assert_eq!(
            ComposedP1::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }

    // Compile-only: compose_kernel_config! works with Partitions3 and MsgSmall.
    compose_kernel_config!(ComposedP3MsgSmall<Partitions3, SyncMinimal, MsgSmall, PortsTiny, DebugDisabled>);

    #[test]
    fn composed_with_partitions3_and_msg_small() {
        assert_eq!(ComposedP3MsgSmall::N, 3);
        assert_eq!(ComposedP3MsgSmall::SCHED, 8);
        assert_eq!(ComposedP3MsgSmall::STACK_WORDS, 256);
        assert_eq!(ComposedP3MsgSmall::QS, 2);
        assert_eq!(ComposedP3MsgSmall::QD, 4);
        assert_eq!(ComposedP3MsgSmall::QM, 4);
        assert_eq!(ComposedP3MsgSmall::QW, 2);
    }

    compose_kernel_config!(ComposedP4<Partitions4, SyncStandard, MsgStandard, PortsStandard, DebugEnabled>);

    #[test]
    fn composed_with_partitions4() {
        // PartitionConfig
        assert_eq!(ComposedP4::N, 4);
        assert_eq!(ComposedP4::SCHED, 8);
        assert_eq!(ComposedP4::STACK_WORDS, 256);
        // SyncConfig
        assert_eq!(ComposedP4::S, SyncStandard::SEMAPHORES);
        assert_eq!(ComposedP4::SW, SyncStandard::SEMAPHORE_WAITQ);
        assert_eq!(ComposedP4::MS, SyncStandard::MUTEXES);
        assert_eq!(ComposedP4::MW, SyncStandard::MUTEX_WAITQ);
        // MsgConfig
        assert_eq!(ComposedP4::QS, MsgStandard::QUEUES);
        assert_eq!(ComposedP4::QD, MsgStandard::QUEUE_DEPTH);
        assert_eq!(ComposedP4::QM, MsgStandard::MAX_MSG_SIZE);
        assert_eq!(ComposedP4::QW, MsgStandard::QUEUE_WAITQ);
        // PortsConfig
        assert_eq!(ComposedP4::SP, PortsStandard::SAMPLING_PORTS);
        assert_eq!(ComposedP4::SM, PortsStandard::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(ComposedP4::BS, PortsStandard::BLACKBOARDS);
        assert_eq!(ComposedP4::BM, PortsStandard::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(ComposedP4::BW, PortsStandard::BLACKBOARD_WAITQ);
        // DebugConfig
        assert_eq!(
            ComposedP4::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }

    compose_kernel_config!(ComposedStdSmall<Partitions2, SyncStandard, MsgStandard, PortsSmall, DebugDisabled>);

    #[test]
    fn composed_std_small_runtime() {
        // SyncStandard bridges
        assert_eq!(ComposedStdSmall::S, SyncStandard::SEMAPHORES);
        assert_eq!(ComposedStdSmall::SW, SyncStandard::SEMAPHORE_WAITQ);
        assert_eq!(ComposedStdSmall::MS, SyncStandard::MUTEXES);
        assert_eq!(ComposedStdSmall::MW, SyncStandard::MUTEX_WAITQ);
        // MsgStandard bridges
        assert_eq!(ComposedStdSmall::QS, MsgStandard::QUEUES);
        assert_eq!(ComposedStdSmall::QD, MsgStandard::QUEUE_DEPTH);
        assert_eq!(ComposedStdSmall::QM, MsgStandard::MAX_MSG_SIZE);
        assert_eq!(ComposedStdSmall::QW, MsgStandard::QUEUE_WAITQ);
        // PortsSmall bridges
        assert_eq!(ComposedStdSmall::SP, PortsSmall::SAMPLING_PORTS);
        assert_eq!(ComposedStdSmall::SM, PortsSmall::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(ComposedStdSmall::BS, PortsSmall::BLACKBOARDS);
        assert_eq!(ComposedStdSmall::BM, PortsSmall::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(ComposedStdSmall::BW, PortsSmall::BLACKBOARD_WAITQ);
    }

    // ============ compose_kernel_config! override tests ============

    compose_kernel_config!(
        ComposedMpuOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugEnabled > {
            mpu_enforce = true;
        }
    );

    #[test]
    fn compose_with_mpu_enforce_override() {
        // Override takes effect
        const { assert!(ComposedMpuOverride::MPU_ENFORCE) };
        // Preset-derived values preserved
        assert_eq!(ComposedMpuOverride::N, Partitions2::COUNT);
        assert_eq!(ComposedMpuOverride::SCHED, Partitions2::SCHEDULE_CAPACITY);
        assert_eq!(ComposedMpuOverride::STACK_WORDS, Partitions2::STACK_WORDS);
        assert_eq!(ComposedMpuOverride::S, SyncMinimal::SEMAPHORES);
        assert_eq!(ComposedMpuOverride::QS, MsgMinimal::QUEUES);
        assert_eq!(ComposedMpuOverride::SP, PortsTiny::SAMPLING_PORTS);
        assert_eq!(
            ComposedMpuOverride::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }

    compose_kernel_config!(
        ComposedDebugBufOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugEnabled > {
            debug_buffer_size = 1024;
        }
    );

    #[test]
    fn compose_with_debug_buffer_size_override() {
        // Feature-gated override takes effect
        #[cfg(feature = "partition-debug")]
        {
            assert_eq!(ComposedDebugBufOverride::DEBUG_BUFFER_SIZE, 1024);
            // Confirm it actually differs from the preset default
            assert_ne!(
                ComposedDebugBufOverride::DEBUG_BUFFER_SIZE,
                DebugEnabled::BUFFER_SIZE
            );
        }
        // Non-overridden sub-config values preserved
        assert_eq!(ComposedDebugBufOverride::N, Partitions2::COUNT);
        assert_eq!(
            ComposedDebugBufOverride::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }

    compose_kernel_config!(
        ComposedDebugDrainOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugEnabled > {
            debug_auto_drain = 512;
        }
    );

    #[test]
    fn compose_with_debug_drain_override() {
        assert_eq!(ComposedDebugDrainOverride::DEBUG_AUTO_DRAIN_BUDGET, 512);
        assert_ne!(
            ComposedDebugDrainOverride::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
        // Non-overridden sub-config values preserved
        assert_eq!(ComposedDebugDrainOverride::N, Partitions2::COUNT);
        #[cfg(feature = "partition-debug")]
        assert_eq!(
            ComposedDebugDrainOverride::DEBUG_BUFFER_SIZE,
            DebugEnabled::BUFFER_SIZE
        );
    }

    compose_kernel_config!(
        ComposedClockOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            core_clock_hz = 64_000_000;
            tick_period_us = 500;
        }
    );

    #[test]
    fn compose_with_clock_override() {
        assert_eq!(ComposedClockOverride::CORE_CLOCK_HZ, 64_000_000);
        assert_eq!(ComposedClockOverride::TICK_PERIOD_US, 500);
        // 64_000_000 * 500 / 1_000_000 = 32_000
        assert_eq!(ComposedClockOverride::SYSTICK_CYCLES, 32_000);
    }

    compose_kernel_config!(
        ComposedStackOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            stack_words = 512;
        }
    );

    #[test]
    fn compose_with_stack_words_override() {
        // Override takes effect
        assert_eq!(ComposedStackOverride::STACK_WORDS, 512);
        assert_ne!(ComposedStackOverride::STACK_WORDS, Partitions2::STACK_WORDS);
        // N and SCHED still come from the preset
        assert_eq!(ComposedStackOverride::N, Partitions2::COUNT);
        assert_eq!(ComposedStackOverride::SCHED, Partitions2::SCHEDULE_CAPACITY);
    }

    compose_kernel_config!(
        ComposedSyncOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            semaphores = 8;
        }
    );

    #[test]
    fn compose_with_sync_override() {
        // Override takes effect
        assert_eq!(ComposedSyncOverride::S, 8);
        assert_ne!(ComposedSyncOverride::S, SyncMinimal::SEMAPHORES);
        // Non-overridden sync fields still come from the preset
        assert_eq!(ComposedSyncOverride::SW, SyncMinimal::SEMAPHORE_WAITQ);
        assert_eq!(ComposedSyncOverride::MS, SyncMinimal::MUTEXES);
        assert_eq!(ComposedSyncOverride::MW, SyncMinimal::MUTEX_WAITQ);
    }

    // raw const-name assignment form: `S = 8;`
    compose_kernel_config!(
        SyncRawNameOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            S = 8;
        }
    );

    #[test]
    fn compose_with_sync_raw_name_override() {
        assert_eq!(SyncRawNameOverride::S, 8);
        assert_ne!(SyncRawNameOverride::S, SyncMinimal::SEMAPHORES);
        // Non-overridden sync fields still come from the preset
        assert_eq!(SyncRawNameOverride::SW, SyncMinimal::SEMAPHORE_WAITQ);
        assert_eq!(SyncRawNameOverride::MS, SyncMinimal::MUTEXES);
        assert_eq!(SyncRawNameOverride::MW, SyncMinimal::MUTEX_WAITQ);
    }

    // ============ compose_kernel_config! msg override tests ============

    compose_kernel_config!(
        ComposedMsgOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            queues = 10;
        }
    );

    #[test]
    fn compose_with_msg_friendly_override() {
        // Override takes effect
        assert_eq!(ComposedMsgOverride::QS, 10);
        assert_ne!(ComposedMsgOverride::QS, MsgMinimal::QUEUES);
        // Non-overridden msg fields still come from the preset
        assert_eq!(ComposedMsgOverride::QD, MsgMinimal::QUEUE_DEPTH);
        assert_eq!(ComposedMsgOverride::QM, MsgMinimal::MAX_MSG_SIZE);
        assert_eq!(ComposedMsgOverride::QW, MsgMinimal::QUEUE_WAITQ);
    }

    compose_kernel_config!(
        MsgRawNameOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            QD = 16;
        }
    );

    #[test]
    fn compose_with_msg_raw_name_override() {
        assert_eq!(MsgRawNameOverride::QD, 16);
        assert_ne!(MsgRawNameOverride::QD, MsgMinimal::QUEUE_DEPTH);
        // Non-overridden msg fields still come from the preset
        assert_eq!(MsgRawNameOverride::QS, MsgMinimal::QUEUES);
        assert_eq!(MsgRawNameOverride::QM, MsgMinimal::MAX_MSG_SIZE);
        assert_eq!(MsgRawNameOverride::QW, MsgMinimal::QUEUE_WAITQ);
    }

    // ============ compose_kernel_config! ports override tests ============

    // friendly-name + raw short-name overrides coexist; non-overridden fields bridge the preset.
    compose_kernel_config!(
        PortsOverrideMixed < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            sampling_ports = 8;
            BM = 128;
        }
    );

    #[test]
    fn compose_with_ports_overrides() {
        assert_eq!(PortsOverrideMixed::SP, 8);
        assert_eq!(PortsOverrideMixed::BM, 128);
        assert_ne!(PortsOverrideMixed::SP, PortsTiny::SAMPLING_PORTS);
        assert_ne!(PortsOverrideMixed::BM, PortsTiny::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(PortsOverrideMixed::SM, PortsTiny::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(PortsOverrideMixed::BS, PortsTiny::BLACKBOARDS);
        assert_eq!(PortsOverrideMixed::BW, PortsTiny::BLACKBOARD_WAITQ);
    }

    // ============ custom user-defined preset tests ============

    struct CustomPartitions;

    impl PartitionConfig for CustomPartitions {
        const COUNT: usize = 6;
        const SCHEDULE_CAPACITY: usize = 16;
        const STACK_WORDS: usize = 512;
    }

    struct CustomSync;

    impl SyncConfig for CustomSync {
        const SEMAPHORES: usize = 16;
        const SEMAPHORE_WAITQ: usize = 8;
        const MUTEXES: usize = 8;
        const MUTEX_WAITQ: usize = 8;
    }

    compose_kernel_config!(CustomPresetConfig<CustomPartitions, CustomSync, MsgStandard, PortsSmall, DebugEnabled>);

    #[test]
    fn custom_presets_bridge_correctly() {
        // CustomPartitions bridges
        assert_eq!(CustomPresetConfig::N, 6);
        assert_eq!(CustomPresetConfig::SCHED, 16);
        assert_eq!(CustomPresetConfig::STACK_WORDS, 512);
        // CustomSync bridges
        assert_eq!(CustomPresetConfig::S, 16);
        assert_eq!(CustomPresetConfig::SW, 8);
        assert_eq!(CustomPresetConfig::MS, 8);
        assert_eq!(CustomPresetConfig::MW, 8);
        // MsgStandard bridges (non-custom, still correct)
        assert_eq!(CustomPresetConfig::QS, MsgStandard::QUEUES);
        assert_eq!(CustomPresetConfig::QD, MsgStandard::QUEUE_DEPTH);
        assert_eq!(CustomPresetConfig::QM, MsgStandard::MAX_MSG_SIZE);
        assert_eq!(CustomPresetConfig::QW, MsgStandard::QUEUE_WAITQ);
        // PortsSmall bridges
        assert_eq!(CustomPresetConfig::SP, PortsSmall::SAMPLING_PORTS);
        assert_eq!(CustomPresetConfig::SM, PortsSmall::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(CustomPresetConfig::BS, PortsSmall::BLACKBOARDS);
        assert_eq!(CustomPresetConfig::BM, PortsSmall::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(CustomPresetConfig::BW, PortsSmall::BLACKBOARD_WAITQ);
        // DebugConfig bridges
        assert_eq!(
            CustomPresetConfig::DEBUG_AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }

    #[test]
    fn custom_presets_preserve_kernel_defaults() {
        assert_eq!(CustomPresetConfig::CORE_CLOCK_HZ, 12_000_000);
        assert_eq!(CustomPresetConfig::TICK_PERIOD_US, 1000);
    }

    // ============ compose_kernel_config! doc-attribute forwarding ============

    compose_kernel_config!(
        /// A documented composed config for testing attribute forwarding.
        DocComposedConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugDisabled>
    );

    #[test]
    fn compose_doc_attribute_compiles() {
        // If this compiles, the doc-attribute was forwarded to the struct.
        assert_eq!(DocComposedConfig::N, 2);
    }

    // ============ compose_kernel_config! custom stack type tests ============

    // With override block — set STACK_WORDS to match AlignedStack4K capacity.
    compose_kernel_config!(
        Composed4KStack[crate::partition_core::AlignedStack4K] < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            stack_words = 1024;
        }
    );

    // No-override form — uses AlignedStack4K with preset defaults.
    compose_kernel_config!(Composed4KNoOverride [crate::partition_core::AlignedStack4K]
        <Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugDisabled>);

    #[test]
    fn compose_custom_stack_bridges_presets() {
        // Preset values bridged correctly.
        assert_eq!(Composed4KStack::N, Partitions2::COUNT);
        assert_eq!(Composed4KStack::SCHED, Partitions2::SCHEDULE_CAPACITY);
        assert_eq!(Composed4KStack::STACK_WORDS, 1024);
        assert_eq!(Composed4KStack::S, SyncMinimal::SEMAPHORES);
        assert_eq!(Composed4KStack::QS, MsgMinimal::QUEUES);
        assert_eq!(Composed4KStack::SP, PortsTiny::SAMPLING_PORTS);
        // Core type uses AlignedStack4K — verify it compiles.
        let _core = <Composed4KStack as KernelConfig>::Core::default();
    }

    #[test]
    fn compose_custom_stack_core_type() {
        // Core type with AlignedStack4K must be larger than with AlignedStack1K.
        type Core4K = <Composed4KStack as KernelConfig>::Core;
        type Core1K = <ComposedConfig as KernelConfig>::Core;
        // Both have N=2. AlignedStack4K is 4096 bytes vs 1024 for 1K,
        // so the 4K Core must be strictly larger.
        assert!(core::mem::size_of::<Core4K>() > core::mem::size_of::<Core1K>());
    }

    #[test]
    fn compose_default_stack_uses_1k() {
        // ComposedConfig (no bracket) uses AlignedStack1K by default.
        // Verify Core type matches explicit AlignedStack1K.
        type CoreDefault = <ComposedConfig as KernelConfig>::Core;
        type CoreExplicit = crate::partition_core::PartitionCore<
            { ComposedConfig::N },
            { ComposedConfig::SCHED },
            crate::partition_core::AlignedStack1K,
        >;
        assert_eq!(
            core::mem::size_of::<CoreDefault>(),
            core::mem::size_of::<CoreExplicit>()
        );
    }

    // ============ compose_kernel_config! comprehensive override tests ============

    // Multi-domain override: one field from each of the 5 sub-config domains
    // plus non-sub-config fields (core_clock_hz, mpu_enforce) in a single block.
    compose_kernel_config!(
        ComposedMultiDomain < Partitions3,
        SyncStandard,
        MsgSmall,
        PortsSmall,
        DebugEnabled > {
            stack_words = 512;
            semaphores = 16;
            queues = 8;
            sampling_ports = 12;
            debug_auto_drain = 128;
            core_clock_hz = 48_000_000;
            mpu_enforce = true;
        }
    );

    #[test]
    fn compose_combined_multi_domain_overrides() {
        // --- Overridden fields take effect ---
        // Partition domain
        assert_eq!(ComposedMultiDomain::STACK_WORDS, 512);
        // Sync domain
        assert_eq!(ComposedMultiDomain::S, 16);
        // Msg domain
        assert_eq!(ComposedMultiDomain::QS, 8);
        // Ports domain
        assert_eq!(ComposedMultiDomain::SP, 12);
        // Debug domain
        assert_eq!(ComposedMultiDomain::DEBUG_AUTO_DRAIN_BUDGET, 128);
        // Non-sub-config fields
        assert_eq!(ComposedMultiDomain::CORE_CLOCK_HZ, 48_000_000);
        const { assert!(ComposedMultiDomain::MPU_ENFORCE) };

        // --- Non-overridden preset fields are bridged correctly ---
        // Partitions3: COUNT=3, SCHEDULE_CAPACITY=8 (STACK_WORDS overridden above)
        assert_eq!(ComposedMultiDomain::N, Partitions3::COUNT);
        assert_eq!(ComposedMultiDomain::SCHED, Partitions3::SCHEDULE_CAPACITY);
        // SyncStandard: SEMAPHORE_WAITQ=4, MUTEXES=4, MUTEX_WAITQ=4
        assert_eq!(ComposedMultiDomain::SW, SyncStandard::SEMAPHORE_WAITQ);
        assert_eq!(ComposedMultiDomain::MS, SyncStandard::MUTEXES);
        assert_eq!(ComposedMultiDomain::MW, SyncStandard::MUTEX_WAITQ);
        // MsgSmall: QUEUE_DEPTH=4, MAX_MSG_SIZE=4, QUEUE_WAITQ=2
        assert_eq!(ComposedMultiDomain::QD, MsgSmall::QUEUE_DEPTH);
        assert_eq!(ComposedMultiDomain::QM, MsgSmall::MAX_MSG_SIZE);
        assert_eq!(ComposedMultiDomain::QW, MsgSmall::QUEUE_WAITQ);
        // PortsSmall: SM=4, BS=4, BM=4, BW=4
        assert_eq!(ComposedMultiDomain::SM, PortsSmall::SAMPLING_MAX_MSG_SIZE);
        assert_eq!(ComposedMultiDomain::BS, PortsSmall::BLACKBOARDS);
        assert_eq!(ComposedMultiDomain::BM, PortsSmall::BLACKBOARD_MAX_MSG_SIZE);
        assert_eq!(ComposedMultiDomain::BW, PortsSmall::BLACKBOARD_WAITQ);
        // DebugEnabled: BUFFER_SIZE=256 (only DEBUG_AUTO_DRAIN_BUDGET overridden)
        #[cfg(feature = "partition-debug")]
        assert_eq!(
            ComposedMultiDomain::DEBUG_BUFFER_SIZE,
            DebugEnabled::BUFFER_SIZE
        );
        // Clock: tick_period_us retains default
        assert_eq!(ComposedMultiDomain::TICK_PERIOD_US, 1000);
        // SYSTICK_CYCLES derived from overridden clock: 48_000_000 * 1000 / 1_000_000
        assert_eq!(ComposedMultiDomain::SYSTICK_CYCLES, 48_000);
    }

    // Dynamic-MPU overrides via compose_kernel_config!, cfg-gated.
    compose_kernel_config!(
        ComposedDynMpu < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            buffer_pool_regions = 16;
            buffer_zone_size = 128;
            dynamic_regions = 8;
            system_window_max_gap_ticks = 200;
        }
    );

    #[test]
    fn compose_dynamic_mpu_overrides() {
        // Preset fields still bridged correctly.
        assert_eq!(ComposedDynMpu::N, Partitions2::COUNT);
        assert_eq!(ComposedDynMpu::S, SyncMinimal::SEMAPHORES);
        assert_eq!(ComposedDynMpu::QS, MsgMinimal::QUEUES);
        assert_eq!(ComposedDynMpu::SP, PortsTiny::SAMPLING_PORTS);
        // Dynamic-MPU overrides (only compiled when feature is active).
        #[cfg(feature = "dynamic-mpu")]
        {
            assert_eq!(ComposedDynMpu::BP, 16);
            assert_eq!(ComposedDynMpu::BZ, 128);
            assert_eq!(ComposedDynMpu::DR, 8);
            assert_eq!(ComposedDynMpu::SYSTEM_WINDOW_MAX_GAP_TICKS, 200);
        }
    }

    compose_kernel_config!(
        ComposedIrqOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            irq_priority = 0x60;
        }
    );

    #[test]
    fn compose_irq_priority_override() {
        assert_eq!(ComposedIrqOverride::IRQ_DEFAULT_PRIORITY, 0x60);
        assert_priority_order::<ComposedIrqOverride>();
    }

    compose_kernel_config!(
        ComposedMinAppIrqOverride < Partitions2,
        SyncMinimal,
        MsgMinimal,
        PortsTiny,
        DebugDisabled > {
            min_app_irq_priority = 0x40;
        }
    );

    #[test]
    fn compose_min_app_irq_priority_override() {
        assert_eq!(ComposedMinAppIrqOverride::MIN_APP_IRQ_PRIORITY, 0x40);
        // Default should remain unchanged for other fields.
        assert_eq!(ComposedMinAppIrqOverride::IRQ_DEFAULT_PRIORITY, 0xC0);
    }

    /// SYSTICK >= MIN_APP_IRQ violates three-tier ordering.
    struct SystickNotAboveMinAppIrq;
    impl KernelConfig for SystickNotAboveMinAppIrq {
        const N: usize = 2;
        const SYSTICK_PRIORITY: u8 = 0x20; // equal to MIN_APP_IRQ_PRIORITY
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "SysTick priority must be strictly higher")]
    fn assert_priority_order_panics_systick_ge_min_app_irq() {
        assert_priority_order::<SystickNotAboveMinAppIrq>();
    }

    /// IRQ_DEFAULT < MIN_APP_IRQ violates three-tier ordering.
    struct IrqBelowMinAppIrq;
    impl KernelConfig for IrqBelowMinAppIrq {
        const N: usize = 2;
        const IRQ_DEFAULT_PRIORITY: u8 = 0x10; // below MIN_APP_IRQ_PRIORITY (0x20)
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "MIN_APP_IRQ priority must be")]
    fn assert_priority_order_panics_irq_below_min_app_irq() {
        assert_priority_order::<IrqBelowMinAppIrq>();
    }

    // ============ DefaultConfig tests ============

    #[test]
    fn default_config_partition_values() {
        assert_eq!(DefaultConfig::N, 2);
        assert_eq!(DefaultConfig::SCHED, 4);
        assert_eq!(DefaultConfig::STACK_WORDS, 256);
    }

    #[test]
    fn default_config_sync_values() {
        assert_eq!(DefaultConfig::S, 1);
        assert_eq!(DefaultConfig::SW, 1);
        assert_eq!(DefaultConfig::MS, 1);
        assert_eq!(DefaultConfig::MW, 1);
    }

    #[test]
    fn default_config_msg_values() {
        assert_eq!(DefaultConfig::QS, 1);
        assert_eq!(DefaultConfig::QD, 1);
        assert_eq!(DefaultConfig::QM, 1);
        assert_eq!(DefaultConfig::QW, 1);
    }

    #[test]
    fn default_config_ports_values() {
        assert_eq!(DefaultConfig::SP, 1);
        assert_eq!(DefaultConfig::SM, 1);
        assert_eq!(DefaultConfig::BS, 1);
        assert_eq!(DefaultConfig::BM, 1);
        assert_eq!(DefaultConfig::BW, 1);
    }

    #[test]
    fn default_config_debug_values() {
        assert_eq!(DefaultConfig::DEBUG_AUTO_DRAIN_BUDGET, 256);
    }

    #[test]
    fn default_config_min_app_irq_priority() {
        assert_eq!(DefaultConfig::MIN_APP_IRQ_PRIORITY, 0x20);
    }

    #[test]
    fn default_config_is_default() {
        fn assert_default<T: Default>() {}
        assert_default::<DefaultConfig>();
    }

    // ============ validate_irq_priority tests ============

    #[test]
    fn validate_irq_priority_accepts_valid() {
        // 0xC0 >= 0x20 — well within the allowed range.
        assert!(validate_irq_priority(0xC0, 0x20).is_ok());
    }

    #[test]
    fn validate_irq_priority_rejects_below_floor() {
        // 0x10 < 0x20 — violates the floor.
        let err = validate_irq_priority(0x10, 0x20).unwrap_err();
        assert!(err.contains("MIN_APP_IRQ_PRIORITY"));
    }

    #[test]
    fn validate_irq_priority_exact_boundary_ok() {
        // priority == min_app_irq_priority is the lowest allowed value.
        assert!(validate_irq_priority(0x20, 0x20).is_ok());
    }

    #[test]
    fn validate_irq_priority_one_below_boundary_err() {
        // priority == min_app_irq_priority - 1 must be rejected.
        let err = validate_irq_priority(0x1F, 0x20).unwrap_err();
        assert!(err.contains("MIN_APP_IRQ_PRIORITY"));
    }

    #[test]
    fn validate_irq_priority_zero_with_nonzero_floor_err() {
        // Priority 0 (highest urgency) with floor 0x20 must be rejected.
        let err = validate_irq_priority(0x00, 0x20).unwrap_err();
        assert!(err.contains("MIN_APP_IRQ_PRIORITY"));
    }

    #[test]
    fn validate_irq_priority_max_priority_ok() {
        // Priority 0xFF (lowest urgency) is always above any reasonable floor.
        assert!(validate_irq_priority(0xFF, 0x20).is_ok());
    }

    // ============ enable_bound_irqs priority-enforcement integration tests ====

    /// DefaultConfig's IRQ_DEFAULT_PRIORITY must pass its own floor.
    #[test]
    fn enable_bound_irqs_default_config_priority_valid() {
        // Simulates the validate_irq_priority call that enable_bound_irqs
        // makes: priority = IRQ_DEFAULT_PRIORITY, floor = MIN_APP_IRQ_PRIORITY.
        let priority = <DefaultConfig as KernelConfig>::IRQ_DEFAULT_PRIORITY;
        let floor = <DefaultConfig as KernelConfig>::MIN_APP_IRQ_PRIORITY;
        assert_eq!(priority, 0xC0);
        assert_eq!(floor, 0x20);
        assert!(validate_irq_priority(priority, floor).is_ok());
    }

    /// DefaultConfig's floor must reject a priority that sneaks into Tier 1.
    #[test]
    fn enable_bound_irqs_default_config_rejects_kernel_tier() {
        let floor = <DefaultConfig as KernelConfig>::MIN_APP_IRQ_PRIORITY;
        // SYSTICK_PRIORITY lives in Tier 1 — must be rejected.
        let kernel_prio = <DefaultConfig as KernelConfig>::SYSTICK_PRIORITY;
        assert!(kernel_prio < floor);
        let err = validate_irq_priority(kernel_prio, floor).unwrap_err();
        assert!(err.contains("MIN_APP_IRQ_PRIORITY"));
    }

    /// Custom config where IRQ_DEFAULT sits exactly at the floor.
    struct IrqAtFloor;
    impl KernelConfig for IrqAtFloor {
        const N: usize = 2;
        const IRQ_DEFAULT_PRIORITY: u8 = 0x20; // exactly MIN_APP_IRQ_PRIORITY
        kernel_config_types!();
    }

    #[test]
    fn enable_bound_irqs_irq_at_floor_valid() {
        let priority = <IrqAtFloor as KernelConfig>::IRQ_DEFAULT_PRIORITY;
        let floor = <IrqAtFloor as KernelConfig>::MIN_APP_IRQ_PRIORITY;
        assert_eq!(priority, floor);
        assert!(validate_irq_priority(priority, floor).is_ok());
    }

    #[test]
    fn enable_bound_irqs_irq_at_floor_rejects_one_below() {
        let floor = <IrqAtFloor as KernelConfig>::MIN_APP_IRQ_PRIORITY;
        // One notch into kernel territory — must fail.
        let bad_prio = floor - 1;
        let err = validate_irq_priority(bad_prio, floor).unwrap_err();
        assert!(err.contains("MIN_APP_IRQ_PRIORITY"));
    }

    /// SVCALL == SYSTICK violates strict inequality requirement.
    struct SvcallEqualsSystick;
    impl KernelConfig for SvcallEqualsSystick {
        const N: usize = 2;
        const SVCALL_PRIORITY: u8 = 0x10;
        const SYSTICK_PRIORITY: u8 = 0x10;
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "SVCall priority must be strictly higher")]
    fn assert_priority_order_panics_svcall_equals_systick() {
        assert_priority_order::<SvcallEqualsSystick>();
    }

    /// SYSTICK == SVCALL == 0x00 violates strict inequality requirement.
    struct SystickEqualsSvcallZero;
    impl KernelConfig for SystickEqualsSvcallZero {
        const N: usize = 2;
        const SVCALL_PRIORITY: u8 = 0x00;
        const SYSTICK_PRIORITY: u8 = 0x00;
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "SVCall priority must be strictly higher")]
    fn assert_priority_order_panics_systick_equals_svcall_zero() {
        assert_priority_order::<SystickEqualsSvcallZero>();
    }

    /// MIN_APP_IRQ == SYSTICK violates strict inequality requirement.
    struct MinAppIrqEqualsSystick;
    impl KernelConfig for MinAppIrqEqualsSystick {
        const N: usize = 2;
        const MIN_APP_IRQ_PRIORITY: u8 = 0x10; // equal to SYSTICK_PRIORITY (0x10)
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "SysTick priority must be strictly higher")]
    fn assert_priority_order_panics_min_app_irq_equals_systick() {
        assert_priority_order::<MinAppIrqEqualsSystick>();
    }

    /// IRQ_DEFAULT == PENDSV violates strict inequality requirement.
    struct IrqEqualsPendsv;
    impl KernelConfig for IrqEqualsPendsv {
        const N: usize = 2;
        const IRQ_DEFAULT_PRIORITY: u8 = 0xFF; // equal to PENDSV_PRIORITY (0xFF)
        kernel_config_types!();
    }

    #[test]
    #[should_panic(expected = "IRQ default priority must be strictly higher")]
    fn assert_priority_order_panics_irq_equals_pendsv() {
        assert_priority_order::<IrqEqualsPendsv>();
    }
}
