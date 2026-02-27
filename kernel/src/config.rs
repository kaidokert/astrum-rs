//! Kernel configuration traits and clock setup.
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
//! kernel::kernel_config!(Nrf52840Config {
//!     const N: usize = 4;
//!     const CORE_CLOCK_HZ: u32 = 64_000_000;
//! });
//! ```
//!
//! **STM32F4 — 168 MHz core clock, 500 µs tick:**
//!
//! ```ignore
//! kernel::kernel_config!(Stm32f4Config {
//!     const N: usize = 4;
//!     const CORE_CLOCK_HZ: u32 = 168_000_000;
//!     const TICK_PERIOD_US: u32 = 500;
//! });
//! ```
//!
//! **SAMD51 — 120 MHz core clock, external reference clock:**
//!
//! ```ignore
//! kernel::kernel_config!(Samd51Config {
//!     const N: usize = 4;
//!     const CORE_CLOCK_HZ: u32 = 15_000_000; // HCLK/8 external ref
//!     const TICK_PERIOD_US: u32 = 2000;
//!     const USE_PROCESSOR_CLOCK: bool = false;
//! });
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

/// Preset: 2 partitions, 4 schedule entries, 256-word (1 KiB) stacks.
pub struct Partitions2;

impl PartitionConfig for Partitions2 {
    const COUNT: usize = 2;
    const SCHEDULE_CAPACITY: usize = 4;
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
// TODO: integrate as KernelConfig associated type (same for PartitionConfig/SyncConfig)
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

/// Preset: rich messaging (4 queues, depth 4, 64-byte messages, wait-queue depth 4).
pub struct MsgRich;

impl MsgConfig for MsgRich {
    const QUEUES: usize = 4;
    const QUEUE_DEPTH: usize = 4;
    const MAX_MSG_SIZE: usize = 64;
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
// TODO: integrate as KernelConfig associated type (same as PartitionConfig/SyncConfig/MsgConfig)
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

/// Generates the four associated type aliases (`Core`, `Sync`, `Msg`,
/// `Ports`) required by [`KernelConfig`].
///
/// # Forms
///
/// - `kernel_config_types!()` — uses
///   [`AlignedStack1K`](crate::partition_core::AlignedStack1K).
/// - `kernel_config_types!($stack)` — uses the provided stack type.
#[macro_export]
macro_rules! kernel_config_types {
    () => {
        $crate::kernel_config_types!($crate::partition_core::AlignedStack1K);
    };
    ($stack:ty) => {
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
    };
}

/// Re-exports every [`KernelConfig`] associated constant as an inherent
/// constant on `$name`, so callers can write `MyConfig::SCHED` without
/// importing the trait.
#[macro_export]
#[doc(hidden)]
macro_rules! _kernel_config_inherent_consts {
    ($name:ident) => {
        #[allow(dead_code)]
        impl $name {
            pub const N: usize = <$name as $crate::config::KernelConfig>::N;
            pub const SCHED: usize = <$name as $crate::config::KernelConfig>::SCHED;
            pub const STACK_WORDS: usize = <$name as $crate::config::KernelConfig>::STACK_WORDS;
            pub const S: usize = <$name as $crate::config::KernelConfig>::S;
            pub const SW: usize = <$name as $crate::config::KernelConfig>::SW;
            pub const MS: usize = <$name as $crate::config::KernelConfig>::MS;
            pub const MW: usize = <$name as $crate::config::KernelConfig>::MW;
            pub const QS: usize = <$name as $crate::config::KernelConfig>::QS;
            pub const QD: usize = <$name as $crate::config::KernelConfig>::QD;
            pub const QM: usize = <$name as $crate::config::KernelConfig>::QM;
            pub const QW: usize = <$name as $crate::config::KernelConfig>::QW;
            pub const SP: usize = <$name as $crate::config::KernelConfig>::SP;
            pub const SM: usize = <$name as $crate::config::KernelConfig>::SM;
            pub const BS: usize = <$name as $crate::config::KernelConfig>::BS;
            pub const BM: usize = <$name as $crate::config::KernelConfig>::BM;
            pub const BW: usize = <$name as $crate::config::KernelConfig>::BW;
            #[cfg(feature = "dynamic-mpu")]
            pub const BP: usize = <$name as $crate::config::KernelConfig>::BP;
            #[cfg(feature = "dynamic-mpu")]
            pub const BZ: usize = <$name as $crate::config::KernelConfig>::BZ;
            #[cfg(feature = "dynamic-mpu")]
            pub const DR: usize = <$name as $crate::config::KernelConfig>::DR;
            #[cfg(feature = "dynamic-mpu")]
            pub const SYSTEM_WINDOW_MAX_GAP_TICKS: u32 =
                <$name as $crate::config::KernelConfig>::SYSTEM_WINDOW_MAX_GAP_TICKS;
            #[cfg(feature = "partition-debug")]
            pub const DEBUG_BUFFER_SIZE: usize =
                <$name as $crate::config::KernelConfig>::DEBUG_BUFFER_SIZE;
            pub const DEBUG_AUTO_DRAIN_BUDGET: usize =
                <$name as $crate::config::KernelConfig>::DEBUG_AUTO_DRAIN_BUDGET;
            pub const SVCALL_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::SVCALL_PRIORITY;
            pub const PENDSV_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::PENDSV_PRIORITY;
            pub const SYSTICK_PRIORITY: u8 =
                <$name as $crate::config::KernelConfig>::SYSTICK_PRIORITY;
            pub const CORE_CLOCK_HZ: u32 = <$name as $crate::config::KernelConfig>::CORE_CLOCK_HZ;
            pub const TICK_PERIOD_US: u32 = <$name as $crate::config::KernelConfig>::TICK_PERIOD_US;
            pub const SYSTICK_CYCLES: u32 = <$name as $crate::config::KernelConfig>::SYSTICK_CYCLES;
            pub const USE_PROCESSOR_CLOCK: bool =
                <$name as $crate::config::KernelConfig>::USE_PROCESSOR_CLOCK;
            pub const MPU_ENFORCE: bool = <$name as $crate::config::KernelConfig>::MPU_ENFORCE;
        }
    };
}

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
    ($(#[$meta:meta])* $name:ident { $($body:tt)* }) => {
        $(#[$meta])*
        struct $name;
        impl $crate::config::KernelConfig for $name {
            $($body)*
            $crate::kernel_config_types!();
        }
        $crate::_kernel_config_inherent_consts!($name);
    };
    ($(#[$meta:meta])* $name:ident [$stack:ty] { $($body:tt)* }) => {
        $(#[$meta])*
        struct $name;
        impl $crate::config::KernelConfig for $name {
            $($body)*
            $crate::kernel_config_types!($stack);
        }
        $crate::_kernel_config_inherent_consts!($name);
    };
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
        kernel_config_types!();
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

        kernel_config_types!();
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

    // ============ MsgConfig tests ============

    // Compile-time assertions for MsgMinimal preset values.
    const _: () = assert!(MsgMinimal::QUEUES == 1);
    const _: () = assert!(MsgMinimal::QUEUE_DEPTH == 1);
    const _: () = assert!(MsgMinimal::MAX_MSG_SIZE == 1);
    const _: () = assert!(MsgMinimal::QUEUE_WAITQ == 1);

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
}
