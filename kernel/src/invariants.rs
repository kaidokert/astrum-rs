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
}
