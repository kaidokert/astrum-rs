use super::*;

// -------------------------------------------------------------------------
// IrqAck dispatch tests
// -------------------------------------------------------------------------

/// Static binding table shared by IrqAck dispatch tests.
static IRQ_ACK_TEST_BINDINGS: [crate::irq_dispatch::IrqBinding; 3] = [
    crate::irq_dispatch::IrqBinding::new(5, 0, 0x01), // IRQ 5 → partition 0
    crate::irq_dispatch::IrqBinding::new(10, 1, 0x02), // IRQ 10 → partition 1
    crate::irq_dispatch::IrqBinding::with_clear_model(
        20,
        0,
        0x04,
        crate::irq_dispatch::IrqClearModel::KernelClears(
            crate::irq_dispatch::ClearStrategy::ClearBit {
                addr: 0x4000_0000,
                bit: 3,
            },
        ),
    ),
];

#[test]
fn irq_ack_success_returns_zero() {
    use crate::syscall::SYS_IRQ_ACK;
    let mut k = kernel(0, 0, 0);
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    // Partition 0 owns IRQ 5.
    k.current_partition = 0;
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 5, 0);
    assert_eq!(result, 0);
    // On ARM, the success path calls NVIC::unpend() then NVIC::unmask()
    // (in that order) to clear any stale pending bit before re-enabling
    // the IRQ. Host tests verify dispatch logic only; the actual NVIC
    // calls are gated behind #[cfg(target_arch = "arm")].
}

#[test]
fn irq_ack_missing_binding_returns_invalid_resource() {
    use crate::syscall::SYS_IRQ_ACK;
    let mut k = kernel(0, 0, 0);
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    k.current_partition = 0;
    // IRQ 99 has no binding.
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 99, 0);
    assert_eq!(result, SvcError::InvalidResource.to_u32());
}

#[test]
fn irq_ack_wrong_partition_returns_permission_denied() {
    use crate::syscall::SYS_IRQ_ACK;
    let mut k = kernel(0, 0, 0);
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    // Partition 1 does not own IRQ 5.
    k.current_partition = 1;
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 5, 0);
    assert_eq!(result, SvcError::PermissionDenied.to_u32());
}

#[test]
fn irq_ack_kernel_clears_returns_operation_failed() {
    use crate::syscall::SYS_IRQ_ACK;
    let mut k = kernel(0, 0, 0);
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    // IRQ 20 uses KernelClears; even the correct owner is rejected.
    k.current_partition = 0;
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 20, 0);
    assert_eq!(result, SvcError::OperationFailed.to_u32());
}

#[test]
fn irq_ack_empty_bindings_returns_invalid_resource() {
    use crate::syscall::SYS_IRQ_ACK;
    // Empty irq_bindings → irq_ack_inner finds no binding → InvalidResource.
    let mut k = kernel(0, 0, 0);
    assert!(k.irq_bindings.is_empty());
    k.current_partition = 0;
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 5, 0);
    assert_eq!(result, SvcError::InvalidResource.to_u32());
}

#[test]
fn store_irq_bindings_sets_field() {
    let mut k = kernel(0, 0, 0);
    assert!(k.irq_bindings.is_empty());
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    assert_eq!(k.irq_bindings.len(), 3);
}

#[test]
fn with_irq_bindings_stores_bindings() {
    use crate::partition_core::AlignedStack256B;
    let mut sched = ScheduleTable::<4>::new();
    sched.add(ScheduleEntry::new(0, 50)).unwrap();
    sched.add(ScheduleEntry::new(1, 50)).unwrap();
    #[cfg(feature = "dynamic-mpu")]
    sched.add_system_window(1).unwrap();
    let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
    let mut stack0 = AlignedStack256B::default();
    let mut stack1 = AlignedStack256B::default();
    let m0 = ExternalPartitionMemory::from_aligned_stack(&mut stack0, 0x0800_0001, mpu, 0).unwrap();
    let m1 = ExternalPartitionMemory::from_aligned_stack(&mut stack1, 0x0800_1001, mpu, 1).unwrap();
    let mut k = Kernel::<TestConfig>::new(sched, &[m0, m1]).expect("two partitions should succeed");
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    assert_eq!(k.irq_bindings.len(), 3);
    assert_eq!(k.irq_bindings[0].irq_num, 5);
    assert_eq!(k.irq_bindings[1].irq_num, 10);
    assert_eq!(k.irq_bindings[2].irq_num, 20);
}

#[test]
fn with_irq_bindings_irq_ack_dispatch_succeeds() {
    use crate::partition_core::AlignedStack256B;
    use crate::syscall::SYS_IRQ_ACK;
    let mut sched = ScheduleTable::<4>::new();
    sched.add(ScheduleEntry::new(0, 50)).unwrap();
    sched.add(ScheduleEntry::new(1, 50)).unwrap();
    #[cfg(feature = "dynamic-mpu")]
    sched.add_system_window(1).unwrap();
    let mpu = MpuRegion::new(0x2000_0000, 1024, 0x03);
    let mut stack0 = AlignedStack256B::default();
    let mut stack1 = AlignedStack256B::default();
    let m0 = ExternalPartitionMemory::from_aligned_stack(&mut stack0, 0x0800_0001, mpu, 0).unwrap();
    let m1 = ExternalPartitionMemory::from_aligned_stack(&mut stack1, 0x0800_1001, mpu, 1).unwrap();
    let mut k = Kernel::<TestConfig>::new(sched, &[m0, m1]).expect("two partitions should succeed");
    k.store_irq_bindings(&IRQ_ACK_TEST_BINDINGS);
    k.current_partition = 0;
    let result = dispatch_r0(&mut k, SYS_IRQ_ACK, 5, 0);
    assert_eq!(result, 0, "IrqAck should succeed for partition 0, IRQ 5");
}
