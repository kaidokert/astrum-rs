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

/// Assert all kernel invariants hold.
///
/// In debug builds and tests, this function performs runtime validation of
/// kernel state. In release builds, this is a no-op that compiles away.
///
/// # Panics
///
/// Panics if any kernel invariant is violated (debug/test builds only).
#[cfg(any(debug_assertions, test))]
pub fn assert_kernel_invariants() {
    // Invariant checks will be added in subsequent commits.
    // This is the entry point that will call individual check functions.
    //
    // TODO: integrate assert_stack_pointer_bounds here once assert_kernel_invariants
    // gains access to kernel state (partitions and partition_sp slices). Currently
    // this function has no parameters to pass to the individual checks.
}

/// No-op version for release builds.
///
/// This function compiles to nothing, ensuring zero runtime overhead.
#[cfg(not(any(debug_assertions, test)))]
#[inline(always)]
pub fn assert_kernel_invariants() {}

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

    #[test]
    fn test_assert_kernel_invariants_does_not_panic() {
        // The stub should complete without panicking.
        // Once invariants are added, this test verifies the happy path.
        assert_kernel_invariants();
    }

    #[test]
    fn test_assert_kernel_invariants_is_callable_multiple_times() {
        // Invariant checks should be idempotent and safe to call repeatedly.
        for _ in 0..10 {
            assert_kernel_invariants();
        }
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
}
