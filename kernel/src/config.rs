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
}
