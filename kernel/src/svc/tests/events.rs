use super::*;

// -------------------------------------------------------------------------
// Event dispatch tests
// -------------------------------------------------------------------------

#[test]
fn event_wait_dispatches_to_events_module() {
    let mut t = tbl();
    ev_data::event_set(&mut t, PartitionId::new(0), 0b1010);
    let mut ef = frame(SYS_EVT_WAIT, 0xDEADBEEF, 0b1110);
    dispatch_syscall(&mut ef, &mut t, 0);
    assert_eq!(ef.r0, 0b1010);
    assert_eq!(t.get(0).unwrap().event_flags(), 0);
}

#[test]
fn event_set_dispatches_to_events_module() {
    let mut t = tbl();
    let mut ef = frame(SYS_EVT_SET, 1, 0b0101);
    dispatch_syscall(&mut ef, &mut t, 0);
    assert_eq!(ef.r0, 0);
    assert_eq!(t.get(1).unwrap().event_flags(), 0b0101);
}

#[test]
fn event_clear_dispatches_to_events_module() {
    let mut t = tbl();
    ev_data::event_set(&mut t, PartitionId::new(0), 0b1111);
    let mut ef = frame(SYS_EVT_CLEAR, 0xDEADBEEF, 0b0101);
    dispatch_syscall(&mut ef, &mut t, 0);
    assert_eq!(ef.r0, 0b1111, "event_clear must return previous flags");
    assert_eq!(t.get(0).unwrap().event_flags(), 0b1010);
}

#[test]
fn event_invalid_partition_returns_error_code() {
    let inv = SvcError::InvalidPartition.to_u32();
    let mut t = tbl();
    let mut ef = frame(SYS_EVT_WAIT, 0xDEADBEEF, 0b0001);
    dispatch_syscall(&mut ef, &mut t, 99);
    assert_eq!(ef.r0, inv);
    let mut ef = frame(SYS_EVT_SET, 99, 0b0001);
    dispatch_syscall(&mut ef, &mut t, 0);
    assert_eq!(ef.r0, inv);
    let mut ef = frame(SYS_EVT_CLEAR, 0xDEADBEEF, 0b0001);
    dispatch_syscall(&mut ef, &mut t, 99);
    assert_eq!(ef.r0, inv);
}

#[test]
fn dispatch_event_wait_blocking_triggers_deschedule() {
    let mut k = kernel(0, 0, 0);
    // No bits set — event_wait should block (return 0) and trigger deschedule.
    let mut ef = frame(SYS_EVT_WAIT, 0, 0b1010);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "blocking EventWait must return 0");
    assert!(
        k.yield_requested(),
        "blocking EventWait must trigger deschedule"
    );
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting,
        "blocked partition must be in Waiting state"
    );
}

#[test]
fn dispatch_event_wait_immediate_no_deschedule() {
    let mut k = kernel(0, 0, 0);
    // Pre-set bits so event_wait returns immediately with matched bits.
    ev_data::event_set(k.partitions_mut(), PartitionId::new(0), 0b1010);
    let mut ef = frame(SYS_EVT_WAIT, 0, 0b1110);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0b1010);
    assert!(!k.yield_requested());
}

#[test]
fn dispatch_event_wait_blocking_saves_wait_mask() {
    let mut k = kernel(0, 0, 0);
    // No bits set — EventWait should block and save the wait mask in the PCB.
    let mask = 0b1100_0011;
    let mut ef = frame(SYS_EVT_WAIT, 0, mask);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "blocked EventWait must return 0");
    assert!(
        k.yield_requested(),
        "blocking EventWait must trigger deschedule"
    );
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting,
        "blocked partition must be Waiting"
    );
    assert_eq!(
        k.partitions().get(0).unwrap().event_wait_mask(),
        mask,
        "PCB must save the wait mask for wake-up"
    );
}

/// Confused-deputy regression: EventWait must use current_partition, not r1.
/// If r1=1 but current_partition=0, the wait must operate on partition 0.
#[test]
fn event_wait_uses_current_partition_not_r1() {
    let mut k = kernel(0, 0, 0);
    // r1=1 (attacker tries to operate on partition 1), mask in r2
    let mut ef = frame(SYS_EVT_WAIT, 1, 0b0011);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    // current_partition=0 has no bits set, so it blocks
    assert_eq!(ef.r0, 0, "EventWait must block on partition 0");
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting,
        "partition 0 must be Waiting (not partition 1)"
    );
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Running,
        "partition 1 must be unaffected"
    );
}

/// Confused-deputy regression: EventClear must use current_partition, not r1.
/// If r1=1 but current_partition=0, the clear must operate on partition 0.
#[test]
fn event_clear_uses_current_partition_not_r1() {
    let mut k = kernel(0, 0, 0);
    // Pre-set bits on both partitions
    ev_data::event_set(k.partitions_mut(), PartitionId::new(0), 0b1111);
    ev_data::event_set(k.partitions_mut(), PartitionId::new(1), 0b1111);
    // r1=1 (attacker tries to clear partition 1's flags), mask in r2
    let mut ef = frame(SYS_EVT_CLEAR, 1, 0b0101);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0b1111, "EventClear must return previous flags");
    assert_eq!(
        k.partitions().get(0).unwrap().event_flags(),
        0b1010,
        "partition 0 flags must be cleared (not partition 1)"
    );
    assert_eq!(
        k.partitions().get(1).unwrap().event_flags(),
        0b1111,
        "partition 1 flags must be unaffected"
    );
}

/// EventSet must still route to the target in r1 (by design — you signal
/// another partition). Verify r1=1 targets partition 1.
#[test]
fn event_set_still_targets_r1() {
    let mut k = kernel(0, 0, 0);
    // current_partition=0, r1=1 → should set flags on partition 1
    let mut ef = frame(SYS_EVT_SET, 1, 0b0110);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "EventSet must succeed");
    assert_eq!(
        k.partitions().get(1).unwrap().event_flags(),
        0b0110,
        "partition 1 must have the flags set via r1"
    );
    assert_eq!(
        k.partitions().get(0).unwrap().event_flags(),
        0,
        "partition 0 must be unaffected"
    );
}
