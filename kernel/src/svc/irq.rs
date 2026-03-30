use crate::irq_dispatch::IrqBinding;
use crate::PartitionId;

/// Handle an `IrqAck` syscall: validate the binding and unmask the IRQ.
///
/// Returns `0` on success, or an `SvcError` code on failure.
#[must_use]
pub fn handle_irq_ack(bindings: &[IrqBinding], caller: PartitionId, irq_num: u8) -> u32 {
    let result = crate::irq_ack::irq_ack_inner(bindings, caller, irq_num);
    #[cfg(target_arch = "arm")]
    if result == 0 {
        // SAFETY: irq_num was validated by irq_ack_inner (binding found).
        // Clear any stale pending bit before unmasking: if the IRQ
        // fired while masked, the pending flag would cause an
        // immediate spurious interrupt on unmask.
        unsafe {
            cortex_m::peripheral::NVIC::unpend(crate::irq_dispatch::IrqNr(irq_num));
            cortex_m::peripheral::NVIC::unmask(crate::irq_dispatch::IrqNr(irq_num));
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq_dispatch::{ClearStrategy, IrqClearModel};
    use rtos_traits::syscall::SvcError;

    const B: [IrqBinding; 2] = [IrqBinding::new(5, 0, 0x01), IrqBinding::new(10, 1, 0x02)];

    #[test]
    fn delegates_to_irq_ack_inner() {
        assert_eq!(handle_irq_ack(&B, 0, 5), 0);
        assert_eq!(handle_irq_ack(&B, 1, 10), 0);
        assert_eq!(
            handle_irq_ack(&B, 0, 99),
            SvcError::InvalidResource.to_u32()
        );
        assert_eq!(
            handle_irq_ack(&B, 1, 5),
            SvcError::PermissionDenied.to_u32()
        );
        assert_eq!(
            handle_irq_ack(&[], 0, 5),
            SvcError::InvalidResource.to_u32()
        );
    }

    #[test]
    fn kernel_clears_rejected() {
        let b = [IrqBinding::with_clear_model(
            7,
            0,
            0x01,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x4000_0000,
                bit: 3,
            }),
        )];
        assert_eq!(handle_irq_ack(&b, 0, 7), SvcError::OperationFailed.to_u32());
    }
}
