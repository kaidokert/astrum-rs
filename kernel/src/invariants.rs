//! Kernel invariant assertions.
//!
//! This module provides runtime checks for kernel invariants. These checks
//! are only enabled in debug builds and tests to avoid overhead in release
//! builds.
//!
//! # Usage
//!
//! Call `assert_kernel_invariants()` at strategic points (e.g., after context
//! switches, syscall entry/exit) to verify the kernel's internal state is
//! consistent.

use crate::partition::{PartitionControlBlock, PartitionState};
use crate::scheduler::ScheduleEntry;

/// Assert that at most one partition has state `Running`.
///
/// This is a fundamental invariant of the scheduler: only one partition
/// can execute at a time on a single-core system. Violating this invariant
/// indicates a scheduler bug.
///
/// # Panics
///
/// Panics if more than one partition is in the `Running` state.
#[cfg(any(debug_assertions, test))]
pub fn assert_partition_state_consistency(partitions: &[PartitionControlBlock]) {
    let running_count = partitions
        .iter()
        .filter(|p| p.state() == PartitionState::Running)
        .count();
    if running_count > 1 {
        panic!(
            "invariant violation: {} partitions in Running state (expected at most 1)",
            running_count
        );
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_partition_state_consistency(_partitions: &[PartitionControlBlock]) {}

/// Assert that partition IDs match their array indices.
///
/// The partition table must maintain the invariant that `partitions[i].id() == i`
/// for all partitions. This ensures O(1) lookup by partition ID and prevents
/// corruption of partition identity.
///
/// # Panics
///
/// Panics if any partition's ID does not match its array index.
#[cfg(any(debug_assertions, test))]
pub fn assert_partition_table_integrity(partitions: &[PartitionControlBlock]) {
    for (index, partition) in partitions.iter().enumerate() {
        let pid = partition.id();
        if pid as usize != index {
            panic!(
                "invariant violation: partition at index {} has id {} (expected id == index)",
                index, pid
            );
        }
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_partition_table_integrity(_partitions: &[PartitionControlBlock]) {}

/// Assert that each partition's saved stack pointer is within its stack region.
///
/// For each partition, if the saved SP is non-zero (initialized), it must fall
/// within the range `[stack_base, stack_base + stack_size]`. The upper bound
/// is inclusive because the initial SP points to the top of the stack (one
/// past the last usable word).
///
/// Partitions with SP == 0 are skipped; they have not yet been initialized
/// and will receive their initial SP during the first context switch.
///
/// # Panics
///
/// Panics if any initialized partition's SP falls outside its stack bounds,
/// or if the slices have mismatched lengths.
#[cfg(any(debug_assertions, test))]
pub fn assert_stack_pointer_bounds(partitions: &[PartitionControlBlock], partition_sp: &[u32]) {
    // Ensure slices have matching lengths to detect kernel state corruption.
    assert!(
        partitions.len() == partition_sp.len(),
        "invariant violation: partitions.len() ({}) != partition_sp.len() ({})",
        partitions.len(),
        partition_sp.len()
    );

    for (pcb, &sp) in partitions.iter().zip(partition_sp.iter()) {
        // Skip uninitialized partitions (SP == 0)
        if sp == 0 {
            continue;
        }

        let pid = pcb.id();

        // ARM Cortex-M requires 4-byte (word) aligned stack pointers.
        if sp % 4 != 0 {
            panic!(
                "invariant violation: partition {} SP 0x{:08x} is not 4-byte aligned",
                pid, sp
            );
        }

        let stack_base = pcb.stack_base();
        let stack_size = pcb.stack_size();
        // TODO: wrapping_add is used defensively here, but if stacks actually wrapped
        // around address space, the linear comparisons below would be invalid. This is
        // acceptable because valid stack regions never wrap on Cortex-M.
        let stack_top = stack_base.wrapping_add(stack_size);

        // SP must be within [stack_base, stack_top] (inclusive on both ends)
        if sp < stack_base || sp > stack_top {
            panic!(
                "invariant violation: partition {} SP 0x{:08x} outside stack bounds \
                 [0x{:08x}, 0x{:08x}]",
                pid, sp, stack_base, stack_top
            );
        }
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_stack_pointer_bounds(_partitions: &[PartitionControlBlock], _partition_sp: &[u32]) {}

/// Assert that `active_partition` matches the single Running partition.
#[cfg(any(debug_assertions, test))]
pub fn assert_running_matches_active(
    partitions: &[PartitionControlBlock],
    active_partition: Option<u8>,
) {
    if let Some(pid) = active_partition {
        let idx = pid as usize;
        assert!(idx < partitions.len(), "active_partition {pid} OOB");
        let st = partitions[idx].state();
        assert!(st == PartitionState::Running, "active {pid} is {st:?}");
        for (i, p) in partitions.iter().enumerate() {
            if i != idx && p.state() == PartitionState::Running {
                panic!("partition {i} Running, active is {pid}");
            }
        }
    } else {
        for (i, p) in partitions.iter().enumerate() {
            if p.state() == PartitionState::Running {
                panic!("partition {i} Running, active is None");
            }
        }
    }
}

#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_running_matches_active(
    _partitions: &[PartitionControlBlock],
    _active_partition: Option<u8>,
) {
}

/// Assert every schedule entry's `partition_index < partition_count`.
#[cfg(any(debug_assertions, test))]
pub fn assert_schedule_indices_in_bounds(entries: &[ScheduleEntry], partition_count: usize) {
    for (i, e) in entries.iter().enumerate() {
        let pi = e.partition_index as usize;
        assert!(
            pi < partition_count,
            "entry {i} index {pi} >= {partition_count}"
        );
    }
}

#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_schedule_indices_in_bounds(_entries: &[ScheduleEntry], _partition_count: usize) {}

/// Assert that no two partitions have overlapping MPU regions, except that
/// Data-vs-Data overlaps are permitted for shared-memory IPC.
///
/// For every pair of distinct partitions, checks each partition's exclusive
/// regions (stack, peripherals) against the other partition's full set of
/// accessible regions (data, stack, peripherals). This catches every overlap
/// combination except Data-vs-Data.
///
/// # Panics
///
/// Panics if any region overlap is found (other than Data-vs-Data).
#[cfg(any(debug_assertions, test))]
pub fn assert_no_overlapping_mpu_regions(partitions: &[PartitionControlBlock]) {
    for (i, pcb_i) in partitions.iter().enumerate() {
        let exclusive_i = pcb_i.exclusive_static_regions();
        let accessible_i = pcb_i.accessible_static_regions();
        for pcb_j in partitions.iter().skip(i + 1) {
            let exclusive_j = pcb_j.exclusive_static_regions();
            let accessible_j = pcb_j.accessible_static_regions();
            // Check exclusive_i against accessible_j: catches stack/peripheral
            // of partition i overlapping any region of partition j.
            for &(base_a, size_a) in exclusive_i.iter() {
                for &(base_b, size_b) in accessible_j.iter() {
                    if base_a < base_b.wrapping_add(size_b) && base_b < base_a.wrapping_add(size_a)
                    {
                        panic!(
                            "invariant violation: partition {} region [0x{:08x}, 0x{:08x}) \
                             overlaps partition {} region [0x{:08x}, 0x{:08x})",
                            pcb_i.id(),
                            base_a,
                            base_a.wrapping_add(size_a),
                            pcb_j.id(),
                            base_b,
                            base_b.wrapping_add(size_b),
                        );
                    }
                }
            }
            // Check accessible_i against exclusive_j: catches any region of
            // partition i overlapping stack/peripheral of partition j.
            for &(base_a, size_a) in accessible_i.iter() {
                for &(base_b, size_b) in exclusive_j.iter() {
                    if base_a < base_b.wrapping_add(size_b) && base_b < base_a.wrapping_add(size_a)
                    {
                        panic!(
                            "invariant violation: partition {} region [0x{:08x}, 0x{:08x}) \
                             overlaps partition {} region [0x{:08x}, 0x{:08x})",
                            pcb_i.id(),
                            base_a,
                            base_a.wrapping_add(size_a),
                            pcb_j.id(),
                            base_b,
                            base_b.wrapping_add(size_b),
                        );
                    }
                }
            }
        }
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_no_overlapping_mpu_regions(_partitions: &[PartitionControlBlock]) {}

/// Assert that no semaphore's current count exceeds its configured maximum.
///
/// Each entry in `semaphore_counts` is a `(current_count, max_count)` pair.
#[cfg(any(debug_assertions, test))]
pub fn assert_semaphore_count_bounded(semaphore_counts: &[(u32, u32)]) {
    for (i, &(count, max)) in semaphore_counts.iter().enumerate() {
        if count > max {
            panic!(
                "invariant violation: semaphore {} count {} exceeds max {}",
                i, count, max
            );
        }
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_semaphore_count_bounded(_semaphore_counts: &[(u32, u32)]) {}

/// Assert that `next_partition` does not point to a `Waiting` partition.
///
/// Panics if `partitions[next_partition]` is `Waiting` or out of bounds.
#[cfg(any(debug_assertions, test))]
pub fn assert_next_partition_not_waiting(partitions: &[PartitionControlBlock], next_partition: u8) {
    assert!(
        partitions[next_partition as usize].state() != PartitionState::Waiting,
        "invariant violation: next_partition ({next_partition}) is Waiting — \
         cannot switch to a blocked partition"
    );
}

#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_next_partition_not_waiting(
    _partitions: &[PartitionControlBlock],
    _next_partition: u8,
) {
}

/// Assert that `address` is aligned to `required_alignment`.
///
/// Provides defense-in-depth for kernel storage placement. The linker and
/// init code guarantee alignment at boot, but this catch-all detects pointer
/// corruption at dispatch time in debug builds. Uses `usize` for portability
/// across 32-bit Cortex-M targets and 64-bit host-side test builds.
///
/// # Panics
///
/// Panics if `address % required_alignment != 0`.
// TODO(panic-free): convert to Result so callers in handler mode
// (SysTick via `_unified_handle_tick!`) can degrade gracefully instead
// of issuing an unrecoverable panic.
#[cfg(any(debug_assertions, test))]
pub fn assert_storage_alignment(address: usize, required_alignment: usize) {
    let offset = address % required_alignment;
    if offset != 0 {
        panic!(
            "invariant violation: storage address 0x{:08x} misaligned by {} bytes \
             (required {} byte alignment)",
            address, offset, required_alignment
        );
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_storage_alignment(_address: usize, _required_alignment: usize) {}

/// Bug 05 invariant: if any partition transitioned Running → Waiting during
/// the current syscall dispatch, `yield_requested` must be `true`.
///
/// # Panics
///
/// Panics if the invariant is violated (debug/test builds only).
#[cfg(any(debug_assertions, test))]
pub fn assert_waiting_implies_yield_requested(
    partitions: &[PartitionControlBlock],
    entry_states: &[PartitionState],
    yield_requested: bool,
) {
    assert!(
        partitions.len() == entry_states.len(),
        "invariant violation: partitions.len() ({}) != entry_states.len() ({})",
        partitions.len(),
        entry_states.len()
    );
    for (i, (pcb, &entry)) in partitions.iter().zip(entry_states.iter()).enumerate() {
        if entry == PartitionState::Running
            && pcb.state() == PartitionState::Waiting
            && !yield_requested
        {
            panic!(
                "invariant violation: partition {} transitioned Running → Waiting \
                 but yield_requested is false — blocking must trigger deschedule",
                i
            );
        }
    }
}

#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_waiting_implies_yield_requested(
    _partitions: &[PartitionControlBlock],
    _entry_states: &[PartitionState],
    _yield_requested: bool,
) {
}

/// Assert that all non-zero PCB stack_base and mpu_region.base values fall
/// within the storage address range [storage_start, storage_end).
///
/// Zero addresses are skipped as they indicate sentinel/uninitialized values.
///
/// # Panics
///
/// Panics if any non-zero address falls outside the valid range.
#[cfg(any(debug_assertions, test))]
pub fn assert_pcb_addresses_in_storage(
    partitions: &[PartitionControlBlock],
    storage_start: u32,
    storage_end: u32,
) {
    for pcb in partitions {
        let sb = pcb.stack_base();
        if sb != 0 && (sb < storage_start || sb >= storage_end) {
            panic!(
                "invariant violation: partition {} stack_base 0x{:08x} outside storage \
                 [0x{:08x}, 0x{:08x})",
                pcb.id(),
                sb,
                storage_start,
                storage_end
            );
        }
        let mb = pcb.mpu_region().base();
        if mb != 0 && (mb < storage_start || mb >= storage_end) {
            panic!(
                "invariant violation: partition {} mpu_region base 0x{:08x} outside storage \
                 [0x{:08x}, 0x{:08x})",
                pcb.id(),
                mb,
                storage_start,
                storage_end
            );
        }
    }
}

/// No-op version for release builds.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_pcb_addresses_in_storage(
    _partitions: &[PartitionControlBlock],
    _storage_start: u32,
    _storage_end: u32,
) {
}

/// Assert all kernel invariants hold.
///
/// In debug builds and tests, this function performs runtime validation of
/// kernel state. In release builds, this is a no-op that compiles away.
///
/// # Panics
///
/// Panics if any kernel invariant is violated (debug/test builds only).
#[cfg(any(debug_assertions, test))]
pub fn assert_kernel_invariants(
    partitions: &[PartitionControlBlock],
    active_partition: Option<u8>,
    semaphore_counts: &[(u32, u32)],
    next_partition: Option<u8>,
    partition_sp: &[u32],
) {
    assert_partition_table_integrity(partitions);
    assert_partition_state_consistency(partitions);
    assert_running_matches_active(partitions, active_partition);
    assert_semaphore_count_bounded(semaphore_counts);
    assert_stack_pointer_bounds(partitions, partition_sp);
    if let Some(np) = next_partition {
        assert_next_partition_not_waiting(partitions, np);
    }
    assert_no_overlapping_mpu_regions(partitions);
}

/// No-op version for release builds.
///
/// This function compiles to nothing, ensuring zero runtime overhead.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_kernel_invariants(
    _partitions: &[PartitionControlBlock],
    _active_partition: Option<u8>,
    _semaphore_counts: &[(u32, u32)],
    _next_partition: Option<u8>,
    _partition_sp: &[u32],
) {
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::MpuRegion;

    fn make_pcb(id: u8) -> PartitionControlBlock {
        PartitionControlBlock::new(
            id,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0x2000_0000, 4096, 0x0306_0000),
        )
    }

    /// Like `make_pcb`, but offsets regions by `id * 0x1_0000` so that
    /// PCBs with distinct IDs never overlap. Use in tests that call
    /// `assert_kernel_invariants` with multiple partitions.
    fn make_disjoint_pcb(id: u8) -> PartitionControlBlock {
        let off = u32::from(id) * 0x0001_0000;
        PartitionControlBlock::new(
            id,
            0x0800_0000,
            0x2000_0000 + off,
            0x2000_0400 + off,
            MpuRegion::new(0x2000_0000 + off, 4096, 0x0306_0000),
        )
    }

    #[test]
    fn kernel_invariants_empty_partitions_none_active() {
        // No partitions with no active partition is valid.
        assert_kernel_invariants(&[], None, &[], None, &[]);
    }

    #[test]
    fn kernel_invariants_all_ready_none_active() {
        // All Ready partitions with no active partition is valid.
        let partitions = [
            make_disjoint_pcb(0),
            make_disjoint_pcb(1),
            make_disjoint_pcb(2),
        ];
        assert_kernel_invariants(&partitions, None, &[], None, &[0; 3]);
    }

    #[test]
    fn kernel_invariants_one_running_matches_active() {
        // One Running partition matching active_partition is valid.
        let mut p0 = make_disjoint_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(
            &[p0, make_disjoint_pcb(1), make_disjoint_pcb(2)],
            Some(0),
            &[],
            None,
            &[0; 3],
        );
    }

    #[test]
    fn kernel_invariants_idempotent() {
        // Invariant checks should be idempotent and safe to call repeatedly.
        let mut p1 = make_disjoint_pcb(1);
        p1.transition(PartitionState::Running).unwrap();
        let partitions = [make_disjoint_pcb(0), p1, make_disjoint_pcb(2)];
        for _ in 0..10 {
            assert_kernel_invariants(&partitions, Some(1), &[], None, &[0; 3]);
        }
    }

    #[test]
    #[should_panic(expected = "2 partitions in Running state")]
    fn kernel_invariants_two_running_panics() {
        // Two Running partitions triggers assert_partition_state_consistency.
        let mut p0 = make_pcb(0);
        let mut p1 = make_pcb(1);
        p0.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(&[p0, p1], Some(0), &[], None, &[0; 2]);
    }

    #[test]
    #[should_panic(expected = "active 0 is Ready")]
    fn kernel_invariants_active_not_running_panics() {
        // active_partition points to a non-Running partition.
        assert_kernel_invariants(&[make_pcb(0), make_pcb(1)], Some(0), &[], None, &[0; 2]);
    }

    #[test]
    fn partition_state_consistency_empty_slice() {
        // Empty partition list is valid (no Running partitions).
        assert_partition_state_consistency(&[]);
    }

    #[test]
    fn partition_state_consistency_all_ready() {
        // All partitions in Ready state is valid.
        let partitions = [make_pcb(0), make_pcb(1), make_pcb(2)];
        assert_partition_state_consistency(&partitions);
    }

    #[test]
    fn partition_state_consistency_one_running() {
        // Exactly one Running partition is valid.
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        let partitions = [p0, make_pcb(1), make_pcb(2)];
        assert_partition_state_consistency(&partitions);
    }

    #[test]
    fn partition_state_consistency_one_waiting() {
        // One Waiting partition with others Ready is valid.
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        p0.transition(PartitionState::Waiting).unwrap();
        let partitions = [p0, make_pcb(1)];
        assert_partition_state_consistency(&partitions);
    }

    #[test]
    #[should_panic(expected = "2 partitions in Running state")]
    fn partition_state_consistency_two_running_panics() {
        // Two Running partitions violates the invariant.
        let mut p0 = make_pcb(0);
        let mut p1 = make_pcb(1);
        p0.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Running).unwrap();
        let partitions = [p0, p1, make_pcb(2)];
        assert_partition_state_consistency(&partitions);
    }

    #[test]
    #[should_panic(expected = "3 partitions in Running state")]
    fn partition_state_consistency_three_running_panics() {
        // Three Running partitions violates the invariant.
        let mut p0 = make_pcb(0);
        let mut p1 = make_pcb(1);
        let mut p2 = make_pcb(2);
        p0.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Running).unwrap();
        p2.transition(PartitionState::Running).unwrap();
        let partitions = [p0, p1, p2];
        assert_partition_state_consistency(&partitions);
    }

    #[test]
    fn partition_table_integrity_empty_slice() {
        // Empty partition list is valid.
        assert_partition_table_integrity(&[]);
    }

    #[test]
    fn partition_table_integrity_valid_ids() {
        // Partitions with IDs matching their indices are valid.
        let partitions = [make_pcb(0), make_pcb(1), make_pcb(2)];
        assert_partition_table_integrity(&partitions);
    }

    #[test]
    #[should_panic(expected = "partition at index 0 has id 1")]
    fn partition_table_integrity_mismatch_at_index_0() {
        // Partition at index 0 with id 1 violates the invariant.
        let partitions = [make_pcb(1), make_pcb(1)];
        assert_partition_table_integrity(&partitions);
    }

    #[test]
    #[should_panic(expected = "partition at index 1 has id 5")]
    fn partition_table_integrity_mismatch_at_index_1() {
        // Partition at index 1 with id 5 violates the invariant.
        let partitions = [make_pcb(0), make_pcb(5)];
        assert_partition_table_integrity(&partitions);
    }

    // ------------------------------------------------------------------
    // assert_stack_pointer_bounds
    // ------------------------------------------------------------------

    #[test]
    fn stack_pointer_bounds_empty_slices() {
        // Empty partition and SP lists are valid.
        assert_stack_pointer_bounds(&[], &[]);
    }

    #[test]
    fn stack_pointer_bounds_uninitialized_sp_skipped() {
        // Partitions with SP == 0 are skipped (not yet initialized).
        let partitions = [make_pcb(0), make_pcb(1)];
        let partition_sp = [0u32, 0u32];
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn stack_pointer_bounds_valid_sp_at_base() {
        // SP exactly at stack_base is valid.
        // make_pcb creates: stack_base = 0x2000_0000, stack_size = 1024
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0000u32]; // SP at stack base
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn stack_pointer_bounds_valid_sp_at_top() {
        // SP exactly at stack_top (stack_base + stack_size) is valid.
        // This is the initial SP position.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0400u32]; // SP at stack top
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn stack_pointer_bounds_valid_sp_in_middle() {
        // SP in the middle of stack region is valid.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0200u32]; // SP at stack_base + 512
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn stack_pointer_bounds_multiple_valid() {
        // Multiple partitions with valid SPs.
        let partitions = [make_pcb(0), make_pcb(1), make_pcb(2)];
        let partition_sp = [0x2000_0100, 0x2000_0200, 0x2000_0300];
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn stack_pointer_bounds_mixed_initialized_and_uninitialized() {
        // Mix of initialized and uninitialized partitions.
        let partitions = [make_pcb(0), make_pcb(1), make_pcb(2)];
        let partition_sp = [0x2000_0200, 0, 0x2000_0100]; // p1 uninitialized
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partition 0 SP 0x1fff0000 outside stack bounds")]
    fn stack_pointer_bounds_below_base_panics() {
        // SP below stack_base violates the invariant.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x1fff_0000u32]; // Below stack_base
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partition 0 SP 0x20000500 outside stack bounds")]
    fn stack_pointer_bounds_above_top_panics() {
        // SP above stack_top violates the invariant.
        // stack_top = 0x2000_0000 + 0x400 = 0x2000_0400
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0500u32]; // Above stack_top
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partition 1 SP 0x30000000 outside stack bounds")]
    fn stack_pointer_bounds_second_partition_out_of_bounds() {
        // Second partition has invalid SP while first is valid.
        let partitions = [make_pcb(0), make_pcb(1)];
        let partition_sp = [0x2000_0200, 0x3000_0000]; // p1 out of bounds
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partitions.len() (2) != partition_sp.len() (1)")]
    fn stack_pointer_bounds_length_mismatch_more_partitions() {
        // More partitions than SPs violates the invariant.
        let partitions = [make_pcb(0), make_pcb(1)];
        let partition_sp = [0x2000_0200u32];
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partitions.len() (1) != partition_sp.len() (2)")]
    fn stack_pointer_bounds_length_mismatch_more_sps() {
        // More SPs than partitions violates the invariant.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0200, 0x2000_0100];
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partition 0 SP 0x20000201 is not 4-byte aligned")]
    fn stack_pointer_bounds_misaligned_sp_panics() {
        // SP not 4-byte aligned violates ARM Cortex-M requirements.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0201u32]; // Misaligned by 1
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    #[should_panic(expected = "partition 0 SP 0x20000202 is not 4-byte aligned")]
    fn stack_pointer_bounds_misaligned_sp_by_2_panics() {
        // SP misaligned by 2 bytes also violates alignment.
        let partitions = [make_pcb(0)];
        let partition_sp = [0x2000_0202u32]; // Misaligned by 2
        assert_stack_pointer_bounds(&partitions, &partition_sp);
    }

    #[test]
    fn running_matches_active_none_valid() {
        assert_running_matches_active(&[], None);
        assert_running_matches_active(&[make_pcb(0), make_pcb(1)], None);
    }

    #[test]
    fn running_matches_active_some_matches() {
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_running_matches_active(&[p0, make_pcb(1), make_pcb(2)], Some(0));
    }

    #[test]
    #[should_panic(expected = "active 0 is Ready")]
    fn running_matches_active_some_but_not_running() {
        assert_running_matches_active(&[make_pcb(0), make_pcb(1)], Some(0));
    }

    #[test]
    #[should_panic(expected = "partition 1 Running, active is None")]
    fn running_matches_active_none_but_one_running() {
        let mut p1 = make_pcb(1);
        p1.transition(PartitionState::Running).unwrap();
        assert_running_matches_active(&[make_pcb(0), p1], None);
    }

    #[test]
    #[should_panic(expected = "partition 1 Running, active is 0")]
    fn running_matches_active_wrong_partition_running() {
        let mut p0 = make_pcb(0);
        let mut p1 = make_pcb(1);
        p0.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Running).unwrap();
        assert_running_matches_active(&[p0, p1], Some(0));
    }

    #[test]
    #[should_panic(expected = "active_partition 5 OOB")]
    fn running_matches_active_out_of_bounds() {
        assert_running_matches_active(&[make_pcb(0), make_pcb(1)], Some(5));
    }

    #[test]
    fn schedule_indices_valid() {
        assert_schedule_indices_in_bounds(&[], 4);
        let entries = [ScheduleEntry::new(0, 100), ScheduleEntry::new(1, 200)];
        assert_schedule_indices_in_bounds(&entries, 3);
    }

    #[test]
    #[should_panic(expected = "entry 0 index 0 >= 0")]
    fn schedule_indices_zero_partitions() {
        assert_schedule_indices_in_bounds(&[ScheduleEntry::new(0, 100)], 0);
    }

    #[test]
    #[should_panic(expected = "entry 1 index 3 >= 2")]
    fn schedule_indices_out_of_bounds() {
        let entries = [ScheduleEntry::new(0, 100), ScheduleEntry::new(3, 200)];
        assert_schedule_indices_in_bounds(&entries, 2);
    }

    #[test]
    fn schedule_indices_boundary_valid() {
        assert_schedule_indices_in_bounds(&[ScheduleEntry::new(3, 100)], 4);
    }

    #[test]
    #[should_panic(expected = "entry 0 index 4 >= 4")]
    fn schedule_indices_boundary_invalid() {
        assert_schedule_indices_in_bounds(&[ScheduleEntry::new(4, 100)], 4);
    }

    // ------------------------------------------------------------------
    // assert_no_overlapping_mpu_regions
    // ------------------------------------------------------------------

    /// Build a PCB with explicit data-region base/size and stack base/size.
    fn make_region_pcb(
        id: u8,
        data_base: u32,
        data_size: u32,
        sb: u32,
        ss: u32,
    ) -> PartitionControlBlock {
        PartitionControlBlock::new(
            id,
            0x0800_0000,
            sb,
            sb.wrapping_add(ss),
            MpuRegion::new(data_base, data_size, 0),
        )
    }

    #[test]
    fn no_overlap_empty_and_single() {
        assert_no_overlapping_mpu_regions(&[]);
        assert_no_overlapping_mpu_regions(&[make_region_pcb(
            0,
            0x2000_0000,
            4096,
            0x2000_1000,
            1024,
        )]);
    }

    #[test]
    fn no_overlap_disjoint_partitions() {
        let p0 = make_region_pcb(0, 0x2000_0000, 4096, 0x2000_1000, 1024);
        let p1 = make_region_pcb(1, 0x2000_2000, 4096, 0x2000_3000, 1024);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    fn no_overlap_adjacent_regions() {
        // End of one == start of next — not overlapping.
        let p0 = make_region_pcb(0, 0x2000_0000, 0x1000, 0x2000_1000, 0x1000);
        let p1 = make_region_pcb(1, 0x2000_2000, 0x1000, 0x2000_3000, 0x1000);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    fn shared_data_regions_not_checked() {
        // Overlapping data regions are accepted: partitions may share data
        // regions for shared-memory IPC, so only exclusive regions (stack,
        // peripherals) are checked for overlap.
        let p0 = make_region_pcb(0, 0x2000_0000, 4096, 0x2001_0000, 1024);
        let p1 = make_region_pcb(1, 0x2000_0800, 4096, 0x2002_0000, 1024);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    #[should_panic(expected = "overlaps partition 1")]
    fn overlap_data_stack() {
        // Partition 0's data region overlaps partition 1's stack.
        // Only Data-vs-Data overlaps are permitted; Data-vs-Stack is rejected.
        let p0 = make_region_pcb(0, 0x2000_0000, 0x2000, 0x2001_0000, 1024);
        let p1 = make_region_pcb(1, 0x2002_0000, 4096, 0x2000_0800, 1024);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    #[should_panic(expected = "overlaps partition 1")]
    fn overlap_peripheral() {
        let p0 = make_region_pcb(0, 0x2000_0000, 4096, 0x2000_1000, 1024)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0000, 4096, 0)]);
        let p1 = make_region_pcb(1, 0x2000_2000, 4096, 0x2000_3000, 1024)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0800, 4096, 0)]);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    fn no_overlap_three_disjoint_partitions() {
        let p0 = make_region_pcb(0, 0x2000_0000, 0x1000, 0x2000_1000, 0x1000);
        let p1 = make_region_pcb(1, 0x2000_2000, 0x1000, 0x2000_3000, 0x1000);
        let p2 = make_region_pcb(2, 0x2000_4000, 0x1000, 0x2000_5000, 0x1000);
        assert_no_overlapping_mpu_regions(&[p0, p1, p2]);
    }

    #[test]
    fn three_partitions_shared_data_allowed() {
        // P0 data overlaps P1 data, P1 data overlaps P2 data — all permitted.
        // Stacks are fully disjoint from each other and from all data regions.
        let p0 = make_region_pcb(0, 0x2000_0000, 0x2000, 0x2001_0000, 0x400);
        let p1 = make_region_pcb(1, 0x2000_1000, 0x2000, 0x2002_0000, 0x400);
        let p2 = make_region_pcb(2, 0x2000_2000, 0x2000, 0x2003_0000, 0x400);
        assert_no_overlapping_mpu_regions(&[p0, p1, p2]);
    }

    #[test]
    #[should_panic(expected = "overlaps partition")]
    fn three_partitions_data_stack_overlap_rejected() {
        // P0 data [0x2000_0000, 0x2000_2000) overlaps P1 stack [0x2000_0800, 0x2000_0C00).
        let p0 = make_region_pcb(0, 0x2000_0000, 0x2000, 0x2001_0000, 0x400);
        let p1 = make_region_pcb(1, 0x2002_0000, 0x1000, 0x2000_0800, 0x400);
        let p2 = make_region_pcb(2, 0x2003_0000, 0x1000, 0x2004_0000, 0x400);
        assert_no_overlapping_mpu_regions(&[p0, p1, p2]);
    }

    #[test]
    #[should_panic(expected = "overlaps partition 1")]
    fn overlap_stack_stack() {
        // P0 stack [0x2000_2000, 0x2000_2400) overlaps P1 stack [0x2000_2200, 0x2000_2600).
        let p0 = make_region_pcb(0, 0x2000_0000, 0x1000, 0x2000_2000, 0x400);
        let p1 = make_region_pcb(1, 0x2000_4000, 0x1000, 0x2000_2200, 0x400);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    #[should_panic(expected = "overlaps partition 1")]
    fn overlap_data_peripheral() {
        // P0 data [0x4000_0000, 0x4000_2000) overlaps P1 peripheral [0x4000_0800, 0x4000_1800).
        let p0 = make_region_pcb(0, 0x4000_0000, 0x2000, 0x2000_0000, 0x400);
        let p1 = make_region_pcb(1, 0x2000_2000, 0x1000, 0x2000_4000, 0x400)
            .with_peripheral_regions(&[MpuRegion::new(0x4000_0800, 0x1000, 0)]);
        assert_no_overlapping_mpu_regions(&[p0, p1]);
    }

    #[test]
    fn semaphore_count_bounded_valid_counts() {
        // count < max for all semaphores.
        assert_semaphore_count_bounded(&[(0, 5), (3, 10), (0, 1)]);
    }

    #[test]
    fn semaphore_count_bounded_at_max() {
        // count == max is the boundary: valid (signal would overflow, but
        // the invariant only checks current state).
        assert_semaphore_count_bounded(&[(5, 5), (1, 1), (0, 0)]);
    }

    #[test]
    #[should_panic(expected = "semaphore 0 count 6 exceeds max 5")]
    fn semaphore_count_bounded_exceeds_max() {
        assert_semaphore_count_bounded(&[(6, 5)]);
    }

    #[test]
    #[should_panic(expected = "semaphore 0 count 3 exceeds max 2")]
    fn kernel_invariants_catches_semaphore_violation() {
        // Valid partition state but invalid semaphore count.
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(&[p0, make_pcb(1)], Some(0), &[(3, 2)], None, &[0; 2]);
    }

    #[test]
    fn next_partition_not_waiting_ready_passes() {
        let partitions = [make_pcb(0), make_pcb(1)];
        assert_next_partition_not_waiting(&partitions, 0);
        assert_next_partition_not_waiting(&partitions, 1);
    }

    #[test]
    #[should_panic(expected = "next_partition (1) is Waiting")]
    fn next_partition_not_waiting_panics_on_waiting() {
        let mut p1 = make_pcb(1);
        p1.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Waiting).unwrap();
        assert_next_partition_not_waiting(&[make_pcb(0), p1], 1);
    }

    #[test]
    fn kernel_invariants_with_next_partition_ready() {
        let mut p0 = make_disjoint_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(&[p0, make_disjoint_pcb(1)], Some(0), &[], Some(1), &[0; 2]);
    }

    #[test]
    #[should_panic(expected = "next_partition (1) is Waiting")]
    fn kernel_invariants_catches_waiting_next_partition() {
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        let mut p1 = make_pcb(1);
        p1.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Waiting).unwrap();
        assert_kernel_invariants(&[p0, p1], Some(0), &[], Some(1), &[0; 2]);
    }

    #[test]
    #[should_panic(expected = "partition at index 0 has id 1")]
    fn kernel_invariants_catches_table_integrity_violation() {
        assert_kernel_invariants(&[make_pcb(1), make_pcb(1)], None, &[], None, &[0; 2]);
    }

    #[test]
    fn kernel_invariants_delegates_stack_pointer_bounds() {
        // Valid SP within stack region passes through master check.
        let mut p0 = make_disjoint_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(
            &[p0, make_disjoint_pcb(1)],
            Some(0),
            &[],
            None,
            &[0x2000_0200, 0],
        );
    }

    #[test]
    #[should_panic(expected = "partition 0 SP 0x10000000 outside stack bounds")]
    fn kernel_invariants_catches_stack_pointer_violation() {
        let mut p0 = make_pcb(0);
        p0.transition(PartitionState::Running).unwrap();
        assert_kernel_invariants(&[p0, make_pcb(1)], Some(0), &[], None, &[0x1000_0000, 0]);
    }

    #[test]
    #[should_panic(expected = "partitions.len() (2) != partition_sp.len() (1)")]
    fn kernel_invariants_catches_sp_length_mismatch() {
        assert_kernel_invariants(&[make_pcb(0), make_pcb(1)], None, &[], None, &[0]);
    }

    #[test]
    fn kernel_invariants_allows_shared_data_regions() {
        // Overlapping data regions are accepted by the kernel invariant
        // check — only exclusive regions (stack, peripherals) are checked.
        let partitions = [
            make_region_pcb(0, 0x2000_0000, 0x2000, 0x2001_0000, 1024),
            make_region_pcb(1, 0x2000_0800, 0x2000, 0x2002_0000, 1024),
        ];
        assert_kernel_invariants(&partitions, None, &[], None, &[0; 2]);
    }

    // ------------------------------------------------------------------
    // assert_storage_alignment
    // ------------------------------------------------------------------

    #[test]
    fn storage_alignment_power_of_two_aligned() {
        assert_storage_alignment(0x2000_0000, 4096);
        assert_storage_alignment(0x2000_0400, 1024);
        assert_storage_alignment(0x2000_0000, 1);
        assert_storage_alignment(0, 4096);
    }

    #[test]
    fn storage_alignment_exact_multiple() {
        // 0x2000_1000 == 0x2000_0000 + 4096 — aligned to 4096.
        assert_storage_alignment(0x2000_1000, 4096);
        // 0x2000_0800 == 0x2000_0000 + 2048 — aligned to 2048 but not 4096.
        assert_storage_alignment(0x2000_0800, 2048);
    }

    #[test]
    #[should_panic(expected = "storage address 0x20000100 misaligned by 256 bytes \
             (required 4096 byte alignment)")]
    fn storage_alignment_misaligned_panics() {
        // 0x2000_0100 % 4096 == 256.
        assert_storage_alignment(0x2000_0100, 4096);
    }

    #[test]
    #[should_panic(expected = "misaligned by 1 bytes (required 4 byte alignment)")]
    fn storage_alignment_off_by_one_panics() {
        assert_storage_alignment(0x2000_0001, 4);
    }

    #[test]
    #[should_panic(expected = "misaligned by 2048 bytes (required 4096 byte alignment)")]
    fn storage_alignment_half_aligned_panics() {
        // Aligned to 2048 but not to 4096.
        assert_storage_alignment(0x2000_0800, 4096);
    }

    // -- assert_waiting_implies_yield_requested --

    #[test]
    fn waiting_yield_valid_cases() {
        assert_waiting_implies_yield_requested(&[], &[], false); // empty
        assert_waiting_implies_yield_requested(&[make_pcb(0)], &[PartitionState::Ready], false);
        let mut p = make_pcb(0);
        p.transition(PartitionState::Running).unwrap();
        assert_waiting_implies_yield_requested(&[p], &[PartitionState::Running], false);
        // Running → Waiting with yield_requested=true.
        let mut p = make_pcb(0);
        p.transition(PartitionState::Running).unwrap();
        p.transition(PartitionState::Waiting).unwrap();
        assert_waiting_implies_yield_requested(&[p], &[PartitionState::Running], true);
        // Already Waiting at entry — not a new transition.
        let mut pw = make_pcb(0);
        pw.transition(PartitionState::Running).unwrap();
        pw.transition(PartitionState::Waiting).unwrap();
        assert_waiting_implies_yield_requested(&[pw], &[PartitionState::Waiting], false);
        // Multi-partition: P1 Running → Waiting with yield set.
        let mut p1 = make_pcb(1);
        p1.transition(PartitionState::Running).unwrap();
        p1.transition(PartitionState::Waiting).unwrap();
        assert_waiting_implies_yield_requested(
            &[make_pcb(0), p1],
            &[PartitionState::Ready, PartitionState::Running],
            true,
        );
    }

    #[test]
    #[should_panic(expected = "partition 0 transitioned Running")]
    fn waiting_yield_missing_yield_panics() {
        let mut p = make_pcb(0);
        p.transition(PartitionState::Running).unwrap();
        p.transition(PartitionState::Waiting).unwrap();
        assert_waiting_implies_yield_requested(&[p], &[PartitionState::Running], false);
    }

    #[test]
    #[should_panic(expected = "partitions.len() (2) != entry_states.len() (1)")]
    fn waiting_yield_length_mismatch_panics() {
        assert_waiting_implies_yield_requested(
            &[make_pcb(0), make_pcb(1)],
            &[PartitionState::Ready],
            false,
        );
    }

    // ------------------------------------------------------------------
    // assert_pcb_addresses_in_storage
    // ------------------------------------------------------------------

    #[test]
    fn pcb_addresses_in_storage_empty() {
        assert_pcb_addresses_in_storage(&[], 0x2000_0000, 0x2001_0000);
    }

    #[test]
    fn pcb_addresses_in_storage_valid() {
        // stack_base=0x2000_0000, mpu base=0x2000_1000 — both in range.
        let p = make_region_pcb(0, 0x2000_1000, 0x1000, 0x2000_0000, 0x400);
        assert_pcb_addresses_in_storage(&[p], 0x2000_0000, 0x2000_2000);
    }

    #[test]
    #[should_panic(expected = "stack_base 0x1fff0000 outside storage")]
    fn pcb_addresses_stack_base_out_of_range() {
        let p = make_region_pcb(0, 0x2000_0000, 0x1000, 0x1FFF_0000, 0x400);
        assert_pcb_addresses_in_storage(&[p], 0x2000_0000, 0x2001_0000);
    }

    #[test]
    #[should_panic(expected = "mpu_region base 0x30000000 outside storage")]
    fn pcb_addresses_mpu_base_out_of_range() {
        let p = make_region_pcb(0, 0x3000_0000, 0x1000, 0x2000_0000, 0x400);
        assert_pcb_addresses_in_storage(&[p], 0x2000_0000, 0x2001_0000);
    }

    #[test]
    fn pcb_addresses_zero_skipped() {
        // Both stack_base=0 and mpu_base=0 — should be skipped.
        let p = PartitionControlBlock::new(0, 0x0800_0000, 0, 0, MpuRegion::new(0, 0, 0));
        assert_pcb_addresses_in_storage(&[p], 0x2000_0000, 0x2001_0000);
    }

    #[test]
    fn pcb_addresses_multiple_mixed() {
        // P0: valid addresses. P1: all zeros (sentinel). P2: valid addresses.
        let p0 = make_region_pcb(0, 0x2000_0000, 0x1000, 0x2000_1000, 0x400);
        let p1 = PartitionControlBlock::new(1, 0x0800_0000, 0, 0, MpuRegion::new(0, 0, 0));
        let p2 = make_region_pcb(2, 0x2000_2000, 0x1000, 0x2000_3000, 0x400);
        assert_pcb_addresses_in_storage(&[p0, p1, p2], 0x2000_0000, 0x2000_4000);
    }
}
