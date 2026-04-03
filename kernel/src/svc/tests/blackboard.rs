use super::*;

// -------------------------------------------------------------------------
// Blackboard tests
// -------------------------------------------------------------------------

#[test]
fn bb_nonblocking_read_empty_returns_error_without_enqueue() {
    use crate::blackboard::BlackboardError;
    let mut k = kernel(0, 0, 0);
    let id = k.blackboards_mut().create().unwrap();
    let mut buf = [0u8; 64];
    // Non-blocking read (timeout=0) on empty board returns error
    assert_eq!(
        k.blackboards_mut().read_blackboard(id, pid(0), &mut buf, 0),
        Err(BlackboardError::BoardEmpty)
    );
    // Caller was NOT enqueued
    assert_eq!(k.blackboards().get(id).unwrap().waiting_readers(), 0);
}

#[test]
fn bb_blocking_read_and_display_wake() {
    use crate::blackboard::ReadBlackboardOutcome;
    let mut k = kernel(0, 0, 0);
    let id = k.blackboards_mut().create().unwrap();
    let mut buf = [0u8; 64];
    // Blocking read (timeout>0) enqueues the caller
    assert_eq!(
        k.blackboards_mut().read_blackboard(id, pid(0), &mut buf, 1),
        Ok(ReadBlackboardOutcome::ReaderBlocked)
    );
    assert_eq!(k.blackboards().get(id).unwrap().waiting_readers(), 1);
    // Display wakes the blocked reader
    let woken = k
        .blackboards_mut()
        .display_blackboard(id, &[0xAA, 0xBB])
        .unwrap();
    assert_eq!(woken.as_slice(), &[pid(0)]);
    // Non-blocking read now succeeds
    let outcome = k
        .blackboards_mut()
        .read_blackboard(id, pid(0), &mut buf, 0)
        .unwrap();
    assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 2 });
    assert_eq!(&buf[..2], &[0xAA, 0xBB]);
}

#[test]
fn bb_blocking_read_wakes_partition() {
    use crate::blackboard::ReadBlackboardOutcome;
    let mut k = kernel(0, 0, 0);
    let id = k.blackboards_mut().create().unwrap();
    let mut buf = [0u8; 64];
    // Transition partition 1 to Waiting and block it on the blackboard
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    assert_eq!(
        k.blackboards_mut().read_blackboard(id, pid(1), &mut buf, 1),
        Ok(ReadBlackboardOutcome::ReaderBlocked)
    );
    // Display wakes partition 1
    let woken = k.blackboards_mut().display_blackboard(id, &[0x01]).unwrap();
    assert_eq!(woken.as_slice(), &[pid(1)]);
    for &wpid in woken.iter() {
        try_transition(k.partitions_mut(), wpid, PartitionState::Ready);
    }
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Ready
    );
}

#[test]
fn bb_invalid_board_errors() {
    use crate::blackboard::BlackboardError;
    let mut k = kernel(0, 0, 0);
    let r: Result<heapless::Vec<PartitionId, 4>, _> =
        k.blackboards_mut().display_blackboard(99, &[1]);
    assert_eq!(r, Err(BlackboardError::InvalidBoard));
}

#[test]
fn bb_clear_svc_dispatch() {
    use crate::blackboard::BlackboardError;
    use crate::syscall::SYS_BB_CLEAR;
    let mut k = kernel(0, 0, 0);
    let id = k.blackboards_mut().create().unwrap();
    let _ = k.blackboards_mut().display_blackboard(id, &[42]).unwrap();
    // Clear via SVC dispatch
    let mut ef = frame(SYS_BB_CLEAR, id as u32, 0);
    // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
    // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
    // test Kernel with valid partition tables.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Non-blocking read after clear should fail
    let mut buf = [0u8; 64];
    assert_eq!(
        k.blackboards_mut().read_blackboard(id, pid(0), &mut buf, 0),
        Err(BlackboardError::BoardEmpty)
    );
    // Invalid board via SVC
    let mut ef = frame(SYS_BB_CLEAR, 99, 0);
    // SAFETY: `ef` is a valid stack-allocated ExceptionFrame constructed by
    // `frame()` with initialized r0-r3 fields; `k` is a properly initialized
    // test Kernel with valid partition tables.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
}

/// BbDisplay and BbRead must reject out-of-bounds pointers with
/// `SvcError::InvalidPointer`.
#[test]
fn blackboard_syscalls_reject_out_of_bounds_pointer() {
    // BbDisplay: r1 = board id, r2 = data len, r3 = data ptr (out-of-bounds)
    let mut k = kernel(0, 0, 0);
    k.blackboards_mut().create().unwrap();
    let mut ef = frame4(crate::syscall::SYS_BB_DISPLAY, 0, 4, 0xDEAD_0000);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(
        ef.r0,
        SvcError::InvalidPointer.to_u32(),
        "BbDisplay should reject out-of-bounds pointer"
    );

    // BbRead: r1 = board id, r2 = timeout, r3 = buf ptr (out-of-bounds)
    let mut k = kernel(0, 0, 0);
    k.blackboards_mut().create().unwrap();
    let mut ef = frame4(crate::syscall::SYS_BB_READ, 0, 0, 0xDEAD_0000);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(
        ef.r0,
        SvcError::InvalidPointer.to_u32(),
        "BbRead should reject out-of-bounds pointer"
    );
}
