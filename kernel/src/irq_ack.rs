//! IRQ acknowledge validation.
//!
//! Pure validation logic for the `SYS_IRQ_ACK` syscall, separated from
//! hardware interaction for testability.

use crate::irq_dispatch::{lookup_binding, IrqBinding, IrqClearModel};
use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};
use rtos_traits::syscall::SvcError;

/// Atomic holder for a `&'static [IrqBinding]` slice.
///
/// Uses [`compare_exchange`](AtomicPtr::compare_exchange) on the pointer to
/// guarantee at-most-once registration.  The length is published with
/// `Release` ordering *after* the pointer CAS succeeds, so a reader's
/// `Acquire` load of the length establishes a happens-before edge that
/// makes the pointer visible as well.
pub(crate) struct BindingTableRef {
    ptr: AtomicPtr<IrqBinding>,
    len: AtomicUsize,
}

impl BindingTableRef {
    pub const fn new() -> Self {
        Self {
            ptr: AtomicPtr::new(core::ptr::null_mut()),
            len: AtomicUsize::new(0),
        }
    }

    /// Register a binding table.  Only the first call takes effect;
    /// subsequent calls are no-ops (idempotent).
    pub fn register(&self, bindings: &'static [IrqBinding]) {
        let new_ptr = bindings.as_ptr() as *mut IrqBinding;
        // Claim the pointer slot via compare_exchange.  If another caller
        // already stored a non-null pointer, the CAS fails and we skip
        // the length store — preserving the existing (ptr, len) pair.
        if self
            .ptr
            .compare_exchange(
                core::ptr::null_mut(),
                new_ptr,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            // CAS succeeded — publish the length with Release ordering.
            // A reader's Acquire load of `len` synchronises-with this
            // Release store, making the prior Relaxed CAS on `ptr`
            // visible via the intra-thread sequenced-before relation.
            self.len.store(bindings.len(), Ordering::Release);
        }
    }

    /// Access the registered binding table.
    ///
    /// Returns `None` if no table has been registered yet.
    /// Otherwise calls `f` with the slice and returns `Some(result)`.
    pub fn with<F, R>(&self, f: F) -> Option<R>
    where
        F: FnOnce(&[IrqBinding]) -> R,
    {
        let len = self.len.load(Ordering::Acquire);
        if len == 0 {
            return None;
        }
        let ptr = self.ptr.load(Ordering::Relaxed);
        // SAFETY: `register` stores `ptr` (CAS, Relaxed) sequenced-before
        // `len` (Release).  This Acquire load of `len` synchronises-with
        // that Release store, guaranteeing the prior CAS on `ptr` is
        // visible.  Both values originate from a valid `&'static
        // [IrqBinding]`, so the resulting slice is valid for the `'static`
        // lifetime.
        let slice = unsafe { core::slice::from_raw_parts(ptr, len) };
        Some(f(slice))
    }
}

static GLOBAL_TABLE: BindingTableRef = BindingTableRef::new();

/// Register the global IRQ binding table.
///
/// Stores a reference to a `&'static [IrqBinding]` so that other kernel
/// subsystems can query IRQ bindings without passing the slice explicitly.
///
/// Only the first call takes effect; subsequent calls are no-ops.
pub fn register_bindings(bindings: &'static [IrqBinding]) {
    GLOBAL_TABLE.register(bindings);
}

/// Access the registered binding table.
///
/// Returns `None` if [`register_bindings`] has not been called yet.
/// Otherwise calls `f` with the registered slice and returns `Some(result)`.
pub fn with_bindings<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&[IrqBinding]) -> R,
{
    GLOBAL_TABLE.with(f)
}

/// Validate an IRQ acknowledge request.
///
/// Returns `0` on success, or an [`SvcError`] code when:
/// - No binding exists for `irq_num` → [`SvcError::InvalidResource`]
/// - `caller` does not own the binding → [`SvcError::PermissionDenied`]
/// - The binding uses [`IrqClearModel::KernelClears`] → [`SvcError::OperationFailed`]
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

    #[test]
    fn kernel_clears_wrong_caller_returns_permission_denied() {
        // Permission check takes priority over clear-model check.
        assert_eq!(
            irq_ack_inner(&BINDINGS, 0, 20),
            SvcError::PermissionDenied.to_u32(),
        );
    }

    // ---- binding table registration tests ----
    // Each test creates its own BindingTableRef instance for full
    // isolation — no shared global state between tests.

    #[test]
    fn with_bindings_returns_none_before_registration() {
        let table = BindingTableRef::new();
        assert_eq!(table.with(|s| s.len()), None);
    }

    #[test]
    fn with_bindings_returns_some_after_registration() {
        static REG: [IrqBinding; 2] = [IrqBinding::new(1, 0, 0x01), IrqBinding::new(2, 1, 0x02)];
        let table = BindingTableRef::new();
        table.register(&REG);
        let result = table.with(|s| s.len());
        assert_eq!(result, Some(2));
    }

    #[test]
    fn registered_slice_matches_original() {
        static REG: [IrqBinding; 3] = [
            IrqBinding::new(5, 0, 0x01),
            IrqBinding::new(10, 1, 0x02),
            IrqBinding::new(15, 2, 0x04),
        ];
        let table = BindingTableRef::new();
        table.register(&REG);
        let matches =
            table.with(|s| s.len() == REG.len() && s.iter().zip(REG.iter()).all(|(a, b)| a == b));
        assert_eq!(matches, Some(true));
    }

    #[test]
    fn registered_bindings_support_lookup() {
        static REG: [IrqBinding; 2] = [IrqBinding::new(7, 0, 0x10), IrqBinding::new(42, 1, 0x20)];
        let table = BindingTableRef::new();
        table.register(&REG);
        let found = table.with(|s| lookup_binding(s, 42));
        assert_eq!(found, Some(Some(1)));
    }

    #[test]
    fn register_is_idempotent() {
        static REG_A: [IrqBinding; 1] = [IrqBinding::new(1, 0, 0x01)];
        static REG_B: [IrqBinding; 2] = [IrqBinding::new(2, 0, 0x02), IrqBinding::new(3, 1, 0x04)];
        let table = BindingTableRef::new();
        table.register(&REG_A);
        table.register(&REG_B); // second call is a no-op
        let len = table.with(|s| s.len());
        assert_eq!(len, Some(1)); // first registration wins
    }
}
