//! IRQ-to-partition dispatch mapping.
//!
//! Each `IrqBinding` records which partition should be notified (and with
//! which event bits) when a particular hardware IRQ fires.

#[cfg(not(test))]
use crate::config::KernelConfig;
use crate::events;
use crate::partition::{PartitionState, PartitionTable};

/// Signal a partition with event bits, returning `true` if it was woken
/// (transitioned from `Waiting` to `Ready`).
///
/// This is the testable inner logic used by ISR dispatch handlers.
/// Returns `false` for invalid partition indices or when the target
/// does not transition to `Ready`.
pub fn signal_partition_inner<const N: usize>(
    t: &mut PartitionTable<N>,
    target: usize,
    event_bits: u32,
) -> bool {
    let was_waiting = t
        .get(target)
        .is_some_and(|p| p.state() == PartitionState::Waiting);
    events::event_set(t, target, event_bits);
    was_waiting
        && t.get(target)
            .is_some_and(|p| p.state() == PartitionState::Ready)
}

/// ISR-callable wrapper that signals a partition and pends a context switch
/// if the partition was woken.
///
/// This is intended to be called from a hardware IRQ handler. It:
/// 1. Acquires mutable access to the kernel state in a critical section
/// 2. Calls [`signal_partition_inner`] on the kernel's partition table
/// 3. Pends PendSV if the target partition transitioned to `Ready`
///
/// # Type Parameters
///
/// - `C`: The kernel configuration type, which determines partition table size
///   and other kernel parameters.
#[cfg(not(test))]
pub fn signal_partition_from_isr<C: KernelConfig>(partition_id: u8, event_bits: u32)
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core: crate::config::CoreOps<
        PartTable = crate::partition::PartitionTable<{ C::N }>,
        SchedTable = crate::scheduler::ScheduleTable<{ C::SCHED }>,
    >,
    C::Sync: crate::config::SyncOps<
        SemPool = crate::semaphore::SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = crate::mutex::MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: crate::config::MsgOps<
        MsgPool = crate::message::MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = crate::queuing::QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: crate::config::PortsOps<
        SamplingPool = crate::sampling::SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = crate::blackboard::BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    crate::state::with_kernel_mut::<C, _, _>(|kernel| {
        let woken =
            signal_partition_inner(kernel.partitions_mut(), partition_id as usize, event_bits);
        if woken {
            // SAFETY: SCB::set_pendsv() sets the PendSV pending bit in the
            // ICSR register. This is always safe on Cortex-M — it merely
            // requests a PendSV exception at the next opportunity and has no
            // preconditions beyond running on a Cortex-M core.
            cortex_m::peripheral::SCB::set_pendsv();
        }
    });
}

/// Generic newtype wrapping a raw IRQ number.
///
/// This eliminates the need for per-peripheral newtype wrappers — any IRQ
/// number can be used with `NVIC::unmask`/`mask`/`pend` via `IrqNr(n)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrqNr(pub u8);

#[cfg(all(not(test), target_arch = "arm"))]
// SAFETY: All u8 values (0..=255) are architecturally valid external
// interrupt numbers on Cortex-M.  The caller is responsible for only
// using numbers that exist on their specific device.
unsafe impl cortex_m::interrupt::InterruptNumber for IrqNr {
    fn number(self) -> u16 {
        self.0 as u16
    }
}

/// A const-friendly mapping from an IRQ number to a (partition, event_bits) pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrqBinding {
    pub irq_num: u8,
    pub partition_id: u8,
    pub event_bits: u32,
    pub clear_model: IrqClearModel,
}

impl IrqBinding {
    /// Create a new IRQ binding (defaults to `IrqClearModel::PartitionAcks`).
    pub const fn new(irq_num: u8, partition_id: u8, event_bits: u32) -> Self {
        Self {
            irq_num,
            partition_id,
            event_bits,
            clear_model: IrqClearModel::PartitionAcks,
        }
    }

    /// Create a new IRQ binding with an explicit clear model.
    pub const fn with_clear_model(
        irq_num: u8,
        partition_id: u8,
        event_bits: u32,
        clear_model: IrqClearModel,
    ) -> Self {
        Self {
            irq_num,
            partition_id,
            event_bits,
            clear_model,
        }
    }
}

/// How the kernel should clear a pending interrupt source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClearStrategy {
    /// Write `value` to the MMIO register at `addr`.
    WriteRegister { addr: u32, value: u32 },
    /// Clear a single bit in the register at `addr`.
    ClearBit { addr: u32, bit: u8 },
}

/// Who is responsible for acknowledging a hardware interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqClearModel {
    /// The partition will acknowledge the interrupt itself.
    PartitionAcks,
    /// The kernel clears the interrupt using the given strategy.
    KernelClears(ClearStrategy),
}

/// Linear scan of `bindings` for the first entry whose `irq_num` matches `irq`.
/// Returns `Some(index)` on hit, `None` on miss or empty slice.
///
/// Uses a `while` loop (not iterators) so the function is `const`-compatible.
pub const fn lookup_binding(bindings: &[IrqBinding], irq: u8) -> Option<usize> {
    let mut i = 0;
    while i < bindings.len() {
        if let Some(b) = bindings.get(i) {
            if b.irq_num == irq {
                return Some(i);
            }
        }
        i += 1;
    }
    None
}

/// O(n²) check for duplicate `irq_num` values in `bindings`.
/// Returns `true` if any two entries share the same IRQ number.
///
/// Intended for use in `const` assertions inside the `bind_interrupts!` macro
/// to reject invalid configurations at compile time.
pub const fn has_duplicate_irqs(bindings: &[IrqBinding]) -> bool {
    let mut i = 0;
    while i < bindings.len() {
        let mut j = i + 1;
        while j < bindings.len() {
            if let (Some(bi), Some(bj)) = (bindings.get(i), bindings.get(j)) {
                if bi.irq_num == bj.irq_num {
                    return true;
                }
            }
            j += 1;
        }
        i += 1;
    }
    false
}

/// Check whether any binding has a `partition_id` that is out of range.
/// Returns `true` if any entry has `partition_id >= max_partitions`.
///
/// Intended for use in `const` assertions inside the `bind_interrupts!` macro
/// to reject invalid partition mappings at compile time.
pub const fn has_invalid_partition_id(bindings: &[IrqBinding], max_partitions: usize) -> bool {
    let mut i = 0;
    while i < bindings.len() {
        if let Some(b) = bindings.get(i) {
            if (b.partition_id as usize) >= max_partitions {
                return true;
            }
        }
        i += 1;
    }
    false
}

/// Check whether any binding has `event_bits == 0`.
/// Returns `true` if any entry has zero event bits, which indicates a
/// configuration error (signaling zero bits is a no-op).
///
/// Intended for use in `const` assertions inside the `bind_interrupts!` macro
/// to reject no-op bindings at compile time.
pub const fn has_zero_event_bits(bindings: &[IrqBinding]) -> bool {
    let mut i = 0;
    while i < bindings.len() {
        if let Some(b) = bindings.get(i) {
            if b.event_bits == 0 {
                return true;
            }
        }
        i += 1;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};

    fn pcb(id: u8) -> PartitionControlBlock {
        let o = (id as u32) * 0x1000;
        PartitionControlBlock::new(
            id,
            0x0800_0000 + o,
            0x2000_0000 + o,
            0x2000_0400 + o,
            MpuRegion::new(0x2000_0000 + o, 4096, 0),
        )
    }

    fn tbl() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        t.add(pcb(0)).unwrap();
        t.add(pcb(1)).unwrap();
        // Both start Ready; transition to Running for testing.
        t.get_mut(0)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t.get_mut(1)
            .unwrap()
            .transition(PartitionState::Running)
            .unwrap();
        t
    }

    #[test]
    fn signal_running_partition_sets_flags_returns_false() {
        let mut t = tbl();
        let woke = signal_partition_inner(&mut t, 0, 0b0101);
        assert!(!woke);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0101);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Running);
    }

    #[test]
    fn signal_waiting_with_matching_mask_wakes() {
        let mut t = tbl();
        // Put partition 0 into Waiting on bit 0.
        events::event_wait(&mut t, 0, 0b0001);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        let woke = signal_partition_inner(&mut t, 0, 0b0001);
        assert!(woke);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
    }

    #[test]
    fn signal_waiting_with_nonmatching_mask_stays_waiting() {
        let mut t = tbl();
        events::event_wait(&mut t, 0, 0b0001);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        let woke = signal_partition_inner(&mut t, 0, 0b1100);
        assert!(!woke);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b1100);
    }

    #[test]
    fn signal_invalid_partition_returns_false() {
        let mut t = tbl();
        let woke = signal_partition_inner(&mut t, 99, 0b0001);
        assert!(!woke);
    }

    #[test]
    fn multiple_signals_accumulate_flags() {
        let mut t = tbl();
        // Put partition 0 into Waiting on bits 0 and 1.
        events::event_wait(&mut t, 0, 0b0011);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        // First signal: bit 2 — no overlap with wait mask.
        let woke1 = signal_partition_inner(&mut t, 0, 0b0100);
        assert!(!woke1);
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0100);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Waiting);

        // Second signal: bit 0 — overlaps wait mask, should wake.
        let woke2 = signal_partition_inner(&mut t, 0, 0b0001);
        assert!(woke2);
        assert_eq!(t.get(0).unwrap().state(), PartitionState::Ready);
        // Both signals accumulated.
        assert_eq!(t.get(0).unwrap().event_flags(), 0b0101);
    }

    #[test]
    fn new_constructor_sets_fields() {
        let b = IrqBinding::new(7, 2, 0x0000_0010);
        assert_eq!(b.irq_num, 7);
        assert_eq!(b.partition_id, 2);
        assert_eq!(b.event_bits, 0x0000_0010);
        assert_eq!(b.clear_model, IrqClearModel::PartitionAcks);
    }

    #[test]
    fn direct_struct_construction() {
        let b = IrqBinding {
            irq_num: 15,
            partition_id: 0,
            event_bits: 0xDEAD_BEEF,
            clear_model: IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x400,
                bit: 5,
            }),
        };
        assert_eq!(b.irq_num, 15);
        assert_eq!(b.partition_id, 0);
        assert_eq!(b.event_bits, 0xDEAD_BEEF);
        assert_eq!(
            b.clear_model,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x400,
                bit: 5
            })
        );
    }

    #[test]
    fn const_construction() {
        const BINDING: IrqBinding = IrqBinding::new(3, 1, 0x01);
        assert_eq!(BINDING.irq_num, 3);
        assert_eq!(BINDING.partition_id, 1);
        assert_eq!(BINDING.event_bits, 0x01);
        assert_eq!(BINDING.clear_model, IrqClearModel::PartitionAcks);
    }

    #[test]
    fn new_defaults_to_partition_acks() {
        let b = IrqBinding::new(1, 0, 0x10);
        assert_eq!(b.clear_model, IrqClearModel::PartitionAcks);
    }

    #[test]
    fn with_clear_model_stores_kernel_clears() {
        let strategy = ClearStrategy::WriteRegister {
            addr: 0xE000_E280,
            value: 1 << 7,
        };
        let b = IrqBinding::with_clear_model(7, 0, 0x01, IrqClearModel::KernelClears(strategy));
        assert_eq!(b.irq_num, 7);
        assert_eq!(b.partition_id, 0);
        assert_eq!(b.event_bits, 0x01);
        assert_eq!(b.clear_model, IrqClearModel::KernelClears(strategy));
    }

    #[test]
    fn with_clear_model_stores_partition_acks() {
        let b = IrqBinding::with_clear_model(3, 1, 0x04, IrqClearModel::PartitionAcks);
        assert_eq!(b.clear_model, IrqClearModel::PartitionAcks);
    }

    #[test]
    fn with_clear_model_const_evaluation() {
        const B: IrqBinding = IrqBinding::with_clear_model(
            9,
            2,
            0x08,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x300,
                bit: 4,
            }),
        );
        assert_eq!(B.irq_num, 9);
        assert_eq!(B.partition_id, 2);
        assert_eq!(B.event_bits, 0x08);
        assert_eq!(
            B.clear_model,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x300,
                bit: 4,
            })
        );
    }

    #[test]
    fn clone_produces_equal_value() {
        let a = IrqBinding::new(10, 3, 0xFF);
        // Call Clone::clone explicitly through a reference to avoid clone_on_copy lint.
        let b = Clone::clone(&a);
        assert_eq!(a, b);
    }

    #[test]
    fn copy_semantics() {
        let a = IrqBinding::new(10, 3, 0xFF);
        let b = a; // Copy
        assert_eq!(a, b); // `a` still usable — Copy
    }

    #[test]
    fn partial_eq_detects_difference() {
        let a = IrqBinding::new(1, 2, 3);
        let b = IrqBinding::new(1, 2, 4);
        assert_ne!(a, b);
    }

    #[test]
    fn debug_format_contains_fields() {
        let b = IrqBinding::new(5, 1, 0x80);
        let dbg = format!("{:?}", b);
        assert!(dbg.contains("IrqBinding"));
        assert!(dbg.contains("irq_num: 5"));
        assert!(dbg.contains("partition_id: 1"));
        assert!(dbg.contains("event_bits: 128"));
    }

    #[test]
    fn boundary_values() {
        let b = IrqBinding::new(u8::MAX, u8::MAX, u32::MAX);
        assert_eq!(b.irq_num, 255);
        assert_eq!(b.partition_id, 255);
        assert_eq!(b.event_bits, u32::MAX);

        let z = IrqBinding::new(0, 0, 0);
        assert_eq!(z.irq_num, 0);
        assert_eq!(z.partition_id, 0);
        assert_eq!(z.event_bits, 0);
    }

    #[test]
    fn const_array_of_bindings() {
        const TABLE: [IrqBinding; 3] = [
            IrqBinding::new(0, 0, 0x01),
            IrqBinding::new(1, 0, 0x02),
            IrqBinding::new(2, 1, 0x04),
        ];
        assert_eq!(TABLE[0].irq_num, 0);
        assert_eq!(TABLE[1].event_bits, 0x02);
        assert_eq!(TABLE[2].partition_id, 1);
    }

    // ---- lookup_binding tests ----

    const BINDINGS: [IrqBinding; 3] = [
        IrqBinding::new(5, 0, 0x01),
        IrqBinding::new(10, 1, 0x02),
        IrqBinding::new(15, 2, 0x04),
    ];

    #[test]
    fn lookup_binding_finds_first_entry() {
        assert_eq!(lookup_binding(&BINDINGS, 5), Some(0));
    }

    #[test]
    fn lookup_binding_finds_middle_entry() {
        assert_eq!(lookup_binding(&BINDINGS, 10), Some(1));
    }

    #[test]
    fn lookup_binding_finds_last_entry() {
        assert_eq!(lookup_binding(&BINDINGS, 15), Some(2));
    }

    #[test]
    fn lookup_binding_returns_none_on_miss() {
        assert_eq!(lookup_binding(&BINDINGS, 99), None);
    }

    #[test]
    fn lookup_binding_empty_slice() {
        assert_eq!(lookup_binding(&[], 5), None);
    }

    #[test]
    fn lookup_binding_returns_first_match() {
        const DUPS: [IrqBinding; 3] = [
            IrqBinding::new(7, 0, 0x01),
            IrqBinding::new(7, 1, 0x02),
            IrqBinding::new(7, 2, 0x04),
        ];
        assert_eq!(lookup_binding(&DUPS, 7), Some(0));
    }

    #[test]
    fn lookup_binding_usable_in_const() {
        const RESULT: Option<usize> = lookup_binding(&BINDINGS, 10);
        assert_eq!(RESULT, Some(1));
    }

    // ---- has_duplicate_irqs tests ----

    #[test]
    fn has_duplicate_irqs_false_for_unique() {
        assert!(!has_duplicate_irqs(&BINDINGS));
    }

    #[test]
    fn has_duplicate_irqs_true_for_duplicates() {
        const DUPS: [IrqBinding; 3] = [
            IrqBinding::new(5, 0, 0x01),
            IrqBinding::new(10, 1, 0x02),
            IrqBinding::new(5, 2, 0x04),
        ];
        assert!(has_duplicate_irqs(&DUPS));
    }

    #[test]
    fn has_duplicate_irqs_false_for_empty() {
        assert!(!has_duplicate_irqs(&[]));
    }

    #[test]
    fn has_duplicate_irqs_false_for_single() {
        const ONE: [IrqBinding; 1] = [IrqBinding::new(3, 0, 0x01)];
        assert!(!has_duplicate_irqs(&ONE));
    }

    #[test]
    fn has_duplicate_irqs_adjacent_duplicates() {
        const ADJ: [IrqBinding; 2] = [IrqBinding::new(8, 0, 0x01), IrqBinding::new(8, 1, 0x02)];
        assert!(has_duplicate_irqs(&ADJ));
    }

    #[test]
    fn has_duplicate_irqs_usable_in_const() {
        const {
            assert!(!has_duplicate_irqs(&[
                IrqBinding::new(5, 0, 0x01),
                IrqBinding::new(10, 1, 0x02),
                IrqBinding::new(15, 2, 0x04),
            ]));
            assert!(has_duplicate_irqs(&[
                IrqBinding::new(1, 0, 0x01),
                IrqBinding::new(1, 1, 0x02),
            ]));
        }
    }

    // ---- has_invalid_partition_id tests ----

    #[test]
    fn has_invalid_partition_id_false_for_valid() {
        const TABLE: [IrqBinding; 3] = [
            IrqBinding::new(0, 0, 0x01),
            IrqBinding::new(1, 1, 0x02),
            IrqBinding::new(2, 2, 0x04),
        ];
        assert!(!has_invalid_partition_id(&TABLE, 4));
    }

    #[test]
    fn has_invalid_partition_id_true_for_out_of_range() {
        const TABLE: [IrqBinding; 3] = [
            IrqBinding::new(0, 0, 0x01),
            IrqBinding::new(1, 5, 0x02), // partition 5 >= max 4
            IrqBinding::new(2, 1, 0x04),
        ];
        assert!(has_invalid_partition_id(&TABLE, 4));
    }

    #[test]
    fn has_invalid_partition_id_false_for_empty() {
        assert!(!has_invalid_partition_id(&[], 4));
    }

    #[test]
    fn has_invalid_partition_id_boundary_equal_is_invalid() {
        // partition_id == max_partitions is out of range
        const TABLE: [IrqBinding; 1] = [IrqBinding::new(0, 4, 0x01)];
        assert!(has_invalid_partition_id(&TABLE, 4));
    }

    #[test]
    fn has_invalid_partition_id_boundary_max_minus_one_is_valid() {
        const TABLE: [IrqBinding; 1] = [IrqBinding::new(0, 3, 0x01)];
        assert!(!has_invalid_partition_id(&TABLE, 4));
    }

    #[test]
    fn has_invalid_partition_id_usable_in_const() {
        const {
            assert!(!has_invalid_partition_id(
                &[IrqBinding::new(0, 0, 0x01), IrqBinding::new(1, 1, 0x02),],
                2,
            ));
            assert!(has_invalid_partition_id(&[IrqBinding::new(0, 2, 0x01)], 2,));
        }
    }

    // ---- has_zero_event_bits tests ----

    #[test]
    fn has_zero_event_bits_true_for_single_zero() {
        const TABLE: [IrqBinding; 1] = [IrqBinding::new(0, 0, 0)];
        assert!(has_zero_event_bits(&TABLE));
    }

    #[test]
    fn has_zero_event_bits_false_for_nonzero() {
        const TABLE: [IrqBinding; 2] = [IrqBinding::new(0, 0, 0x01), IrqBinding::new(1, 1, 0x02)];
        assert!(!has_zero_event_bits(&TABLE));
    }

    #[test]
    fn has_zero_event_bits_true_when_any_zero_in_multi() {
        const TABLE: [IrqBinding; 3] = [
            IrqBinding::new(0, 0, 0x01),
            IrqBinding::new(1, 1, 0),
            IrqBinding::new(2, 0, 0x04),
        ];
        assert!(has_zero_event_bits(&TABLE));
    }

    #[test]
    fn has_zero_event_bits_false_for_empty() {
        assert!(!has_zero_event_bits(&[]));
    }

    #[test]
    fn has_zero_event_bits_usable_in_const() {
        const {
            assert!(has_zero_event_bits(&[IrqBinding::new(0, 0, 0)]));
            assert!(!has_zero_event_bits(&[IrqBinding::new(0, 0, 1)]));
        }
    }

    // ---- IrqNr tests ----

    #[test]
    fn irq_nr_construction() {
        let irq = IrqNr(42);
        assert_eq!(irq.0, 42);
    }

    #[test]
    fn irq_nr_equality() {
        assert_eq!(IrqNr(7), IrqNr(7));
        assert_ne!(IrqNr(7), IrqNr(8));
    }

    #[test]
    fn irq_nr_boundary_values() {
        let zero = IrqNr(0);
        assert_eq!(zero.0, 0);

        let max = IrqNr(255);
        assert_eq!(max.0, 255);
    }

    #[test]
    fn irq_nr_copy_semantics() {
        let a = IrqNr(10);
        let b = a; // Copy
        assert_eq!(a, b); // `a` still usable — Copy
    }

    #[test]
    fn irq_nr_clone() {
        let a = IrqNr(33);
        let b = Clone::clone(&a);
        assert_eq!(a, b);
    }

    #[test]
    fn irq_nr_debug() {
        let dbg = format!("{:?}", IrqNr(99));
        assert!(dbg.contains("IrqNr"));
        assert!(dbg.contains("99"));
    }

    // ---- ClearStrategy tests ----

    const WR: ClearStrategy = ClearStrategy::WriteRegister {
        addr: 0x100,
        value: 1,
    };
    const CB: ClearStrategy = ClearStrategy::ClearBit {
        addr: 0x200,
        bit: 3,
    };

    #[test]
    fn clear_strategy_construct_match_eq_copy_clone() {
        // Construction + exhaustive match
        match WR {
            ClearStrategy::WriteRegister { addr, value } => {
                assert_eq!(addr, 0x100);
                assert_eq!(value, 1);
            }
            ClearStrategy::ClearBit { .. } => panic!("wrong variant"),
        }
        match CB {
            ClearStrategy::ClearBit { addr, bit } => {
                assert_eq!(addr, 0x200);
                assert_eq!(bit, 3);
            }
            ClearStrategy::WriteRegister { .. } => panic!("wrong variant"),
        }
        // Equality
        assert_eq!(WR, WR);
        assert_ne!(WR, CB);
        // Copy: original still usable after move
        let a = WR;
        let b = a;
        assert_eq!(a, b);
        // Clone
        assert_eq!(CB, Clone::clone(&CB));
        // Debug
        let d = format!("{WR:?}");
        assert!(d.contains("WriteRegister") && d.contains("256") && d.contains("1"));
        let d2 = format!("{CB:?}");
        assert!(d2.contains("ClearBit") && d2.contains("512") && d2.contains("3"));
    }

    // ---- IrqClearModel tests ----

    #[test]
    fn irq_clear_model_construct_match_eq_copy_clone() {
        let pa = IrqClearModel::PartitionAcks;
        let kc = IrqClearModel::KernelClears(CB);
        // Exhaustive match — PartitionAcks
        match pa {
            IrqClearModel::PartitionAcks => {}
            IrqClearModel::KernelClears(_) => panic!("wrong variant"),
        }
        // Exhaustive match — KernelClears
        match kc {
            IrqClearModel::KernelClears(s) => assert_eq!(s, CB),
            IrqClearModel::PartitionAcks => panic!("wrong variant"),
        }
        // Equality
        assert_eq!(pa, IrqClearModel::PartitionAcks);
        assert_eq!(kc, IrqClearModel::KernelClears(CB));
        assert_ne!(pa, kc);
        // Copy
        let cp = kc;
        assert_eq!(kc, cp);
        // Clone
        assert_eq!(pa, Clone::clone(&pa));
        // Debug
        let d = format!("{pa:?}");
        assert!(d.contains("PartitionAcks"));
        let d2 = format!("{kc:?}");
        assert!(d2.contains("KernelClears") && d2.contains("ClearBit"));
    }

    // ---- dispatch data-path tests (mixed clear models) ----

    #[test]
    fn lookup_mixed_clear_models_preserves_variant() {
        const MIXED: [IrqBinding; 3] = [
            IrqBinding::new(5, 0, 0x01), // PartitionAcks (default)
            IrqBinding::with_clear_model(
                10,
                1,
                0x02,
                IrqClearModel::KernelClears(ClearStrategy::WriteRegister {
                    addr: 0xE000_E280,
                    value: 1 << 7,
                }),
            ),
            IrqBinding::with_clear_model(
                15,
                2,
                0x04,
                IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                    addr: 0x4001_0000,
                    bit: 3,
                }),
            ),
        ];
        // Lookup returns correct index for each IRQ.
        let idx0 = lookup_binding(&MIXED, 5).unwrap();
        let idx1 = lookup_binding(&MIXED, 10).unwrap();
        let idx2 = lookup_binding(&MIXED, 15).unwrap();
        assert_eq!(idx0, 0);
        assert_eq!(idx1, 1);
        assert_eq!(idx2, 2);
        // clear_model preserved through table indexing.
        assert_eq!(MIXED[idx0].clear_model, IrqClearModel::PartitionAcks);
        assert_eq!(
            MIXED[idx1].clear_model,
            IrqClearModel::KernelClears(ClearStrategy::WriteRegister {
                addr: 0xE000_E280,
                value: 1 << 7,
            })
        );
        assert_eq!(
            MIXED[idx2].clear_model,
            IrqClearModel::KernelClears(ClearStrategy::ClearBit {
                addr: 0x4001_0000,
                bit: 3,
            })
        );
    }

    #[test]
    fn clear_bit_shift_values() {
        // Verify the wrapping_shl computation matches expectations
        // for the ClearBit dispatch path (must not panic for any u8).
        for bit in 0u8..=255 {
            let expected = 1u32.wrapping_shl(bit as u32);
            let s = ClearStrategy::ClearBit { addr: 0x100, bit };
            match s {
                ClearStrategy::ClearBit { bit: b, .. } => {
                    assert_eq!(1u32.wrapping_shl(b as u32), expected);
                }
                _ => panic!("wrong variant"),
            }
        }
    }
}
