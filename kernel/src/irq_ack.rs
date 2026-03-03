//! IRQ acknowledge validation.
//!
//! Pure validation logic for the `SYS_IRQ_ACK` syscall, separated from
//! hardware interaction for testability.

use crate::irq_dispatch::{lookup_binding, IrqBinding, IrqClearModel};
use rtos_traits::syscall::SvcError;

/// Validate an IRQ acknowledge request.
///
/// Returns `0` on success, or an [`SvcError`] code when:
/// - No binding exists for `irq_num` → [`SvcError::InvalidResource`]
/// - `caller` does not own the binding → [`SvcError::PermissionDenied`]
/// - The binding uses [`IrqClearModel::KernelClears`] → [`SvcError::OperationFailed`]
#[must_use = "error code must be forwarded to the caller"]
pub fn irq_ack_inner(bindings: &[IrqBinding], caller: u8, irq_num: u8) -> u32 {
    let idx = match lookup_binding(bindings, irq_num) {
        Some(i) => i,
        None => return SvcError::InvalidResource.to_u32(),
    };
    let binding = match bindings.get(idx) {
        Some(b) => b,
        None => return SvcError::InvalidResource.to_u32(),
    };
    if binding.partition_id != caller {
        return SvcError::PermissionDenied.to_u32();
    }
    if matches!(binding.clear_model, IrqClearModel::KernelClears(_)) {
        return SvcError::OperationFailed.to_u32();
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq_dispatch::{ClearStrategy, IrqClearModel};

    const BINDINGS: [IrqBinding; 3] = [
        IrqBinding::new(5, 0, 0x01),  // IRQ 5 → partition 0, PartitionAcks
        IrqBinding::new(10, 1, 0x02), // IRQ 10 → partition 1, PartitionAcks
        IrqBinding::with_clear_model(
            20,
            2,
            0x04,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x4000_0000,
                bit: 3,
            }),
        ),
    ];

    #[test]
    fn valid_ack_returns_zero() {
        assert_eq!(irq_ack_inner(&BINDINGS, 0, 5), 0);
        assert_eq!(irq_ack_inner(&BINDINGS, 1, 10), 0);
    }

    #[test]
    fn missing_binding_returns_invalid_resource() {
        assert_eq!(
            irq_ack_inner(&BINDINGS, 0, 99),
            SvcError::InvalidResource.to_u32(),
        );
    }

    #[test]
    fn wrong_partition_returns_permission_denied() {
        // IRQ 5 belongs to partition 0; caller 1 should be rejected.
        assert_eq!(
            irq_ack_inner(&BINDINGS, 1, 5),
            SvcError::PermissionDenied.to_u32(),
        );
    }

    #[test]
    fn kernel_clears_returns_operation_failed() {
        // IRQ 20 uses KernelClears; even the correct owner is rejected.
        assert_eq!(
            irq_ack_inner(&BINDINGS, 2, 20),
            SvcError::OperationFailed.to_u32(),
        );
    }

    #[test]
    fn empty_bindings_returns_invalid_resource() {
        assert_eq!(irq_ack_inner(&[], 0, 5), SvcError::InvalidResource.to_u32(),);
    }

    #[test]
    fn multiple_bindings_correct_lookup() {
        // Each IRQ resolves to its own binding independently.
        assert_eq!(irq_ack_inner(&BINDINGS, 0, 5), 0);
        assert_eq!(irq_ack_inner(&BINDINGS, 1, 10), 0);
        // Wrong caller for IRQ 10.
        assert_eq!(
            irq_ack_inner(&BINDINGS, 0, 10),
            SvcError::PermissionDenied.to_u32(),
        );
    }

    #[test]
    fn boundary_irq_num_zero() {
        let table = [IrqBinding::new(0, 0, 0x01)];
        assert_eq!(irq_ack_inner(&table, 0, 0), 0);
    }

    #[test]
    fn boundary_irq_num_max() {
        let table = [IrqBinding::new(255, 3, 0x80)];
        assert_eq!(irq_ack_inner(&table, 3, 255), 0);
        // Wrong caller at boundary.
        assert_eq!(
            irq_ack_inner(&table, 0, 255),
            SvcError::PermissionDenied.to_u32(),
        );
    }

    // ---- #[must_use] return-value contract tests ----

    #[test]
    fn irq_ack_inner_return_must_distinguish_success_from_each_error() {
        // irq_ack_inner returns a u32 status code. Callers MUST forward it
        // to the syscall return register — dropping it silently hides errors.
        let success = irq_ack_inner(&BINDINGS, 0, 5);
        let not_found = irq_ack_inner(&BINDINGS, 0, 99);
        let wrong_owner = irq_ack_inner(&BINDINGS, 1, 5);
        let kernel_clears = irq_ack_inner(&BINDINGS, 2, 20);

        // Success is zero.
        assert_eq!(success, 0);
        // Each error is non-zero.
        assert_ne!(not_found, 0);
        assert_ne!(wrong_owner, 0);
        assert_ne!(kernel_clears, 0);
        // Each error class is distinguishable.
        assert_ne!(not_found, wrong_owner);
        assert_ne!(not_found, kernel_clears);
        assert_ne!(wrong_owner, kernel_clears);
    }

    #[test]
    fn kernel_clears_wrong_caller_returns_permission_denied() {
        // Permission check takes priority over clear-model check.
        assert_eq!(
            irq_ack_inner(&BINDINGS, 0, 20),
            SvcError::PermissionDenied.to_u32(),
        );
    }
}
