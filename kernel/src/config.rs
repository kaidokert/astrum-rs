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

    /// SVCall exception priority (0x00 = highest on Cortex-M).
    const SVCALL_PRIORITY: u8 = 0x00;
    /// PendSV exception priority (must be the lowest of the three).
    const PENDSV_PRIORITY: u8 = 0xFF;
    /// SysTick exception priority (must be lower priority than SVCall,
    /// higher priority than PendSV; on Cortex-M a *larger* number means
    /// *lower* priority).
    const SYSTICK_PRIORITY: u8 = 0xFE;
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

    /// Minimal config using all default priority values.
    struct DefaultPriority;
    impl KernelConfig for DefaultPriority {
        const N: usize = 2;
        const SCHED: usize = 4;
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
    }

    /// Config that overrides priorities but keeps valid ordering.
    struct CustomPriority;
    impl KernelConfig for CustomPriority {
        const N: usize = 2;
        const SCHED: usize = 4;
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
    }

    #[test]
    fn default_priorities_have_correct_values() {
        assert_eq!(DefaultPriority::SVCALL_PRIORITY, 0x00);
        assert_eq!(DefaultPriority::PENDSV_PRIORITY, 0xFF);
        assert_eq!(DefaultPriority::SYSTICK_PRIORITY, 0xFE);
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

    // Compile-time assertions: full priority ordering (including SVCall)
    // is verified at const-eval time for both default and custom configs.
    const _: () = assert_priority_order::<DefaultPriority>();
    const _: () = assert_priority_order::<CustomPriority>();
}
