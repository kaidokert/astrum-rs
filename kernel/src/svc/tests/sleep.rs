use super::*;
use rtos_traits::ids::PartitionId;

// -------------------------------------------------------------------------
// Sleep and timed-wait expiry tests
// -------------------------------------------------------------------------
//
// # SAFETY — Test Dispatch Justification
//
// All `unsafe { k.dispatch(&mut ef) }` calls in this test module share
// the same safety justification:
//
// ## Test Isolation
//
// Each `#[test]` function creates its own `Kernel` and `ExceptionFrame`
// instances on the stack. No global state is shared between tests, so
// tests cannot interfere with each other's kernel or partition state.
//
// ## Single-Threaded Execution
//
// Rust's default test runner executes tests in a single-threaded manner
// (unless `--test-threads=N` is specified, which these tests do not
// rely on). Even with parallel test execution, each test has isolated
// stack-local state. The `dispatch()` function is not re-entrant, but
// since each test owns its own `Kernel` instance, concurrent test
// execution does not cause data races.
//
// ## Valid ExceptionFrame Construction
//
// The `frame()` test helper constructs `ExceptionFrame` instances with
// valid register values. Unlike hardware exception entry, these are not
// actual stacked registers, but the dispatch logic only reads/writes
// the r0-r3 fields which are always initialized. The remaining fields
// (r12, lr, pc, xpsr) are set to zero, which is safe because dispatch
// does not use them.
//
// ## Kernel Construction
//
// The `kernel()` and `kernel_with_registry()` helpers construct `Kernel`
// instances with properly initialized partition tables (via `tbl()`),
// schedule tables, and resource pools. All partitions have valid MPU
// regions and are transitioned to Running state before dispatch.
//
// ## Host-Mode Pointer Validation
//
// Tests run on the host (not target hardware) where `validate_user_ptr`
// checks pass for any pointer within the partition's configured MPU
// region. Tests that exercise pointer validation use `mmap` to allocate
// memory at addresses matching the partition's MPU region, ensuring
// the kernel's bounds checks succeed.

/// Safe wrapper around `Kernel::dispatch` for test use.
///
/// See the module-level SAFETY documentation above for the full
/// justification of why calling `dispatch` is sound in test context.
fn test_dispatch(k: &mut Kernel<'_, TestConfig>, ef: &mut ExceptionFrame) {
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(ef) };
}

// --- sleep timer expiry tests ---

#[test]
fn expire_timed_waits_wakes_sleeping_partition() {
    let mut k = kernel(0, 0, 0);
    k.partitions_mut().get_mut(0).unwrap().set_sleep_until(100);
    k.sleep_queue.push(PartitionId::new(0), 100).unwrap();
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
    k.expire_timed_waits::<8>(99);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
    assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 100);
    k.expire_timed_waits::<8>(100);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Ready
    );
    assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 0);
    assert!(k.yield_requested);
}

#[test]
fn expire_timed_waits_sleep_no_false_wake() {
    let mut k = kernel(0, 0, 0);
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    k.expire_timed_waits::<8>(1000);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
}

#[test]
fn expire_timed_waits_sleep_transition_fail_preserves_sleep_until() {
    let mut k = kernel(0, 0, 0);
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Ready)
        .unwrap();
    k.partitions_mut().get_mut(0).unwrap().set_sleep_until(50);
    k.sleep_queue.push(PartitionId::new(0), 50).unwrap();
    k.expire_timed_waits::<8>(50);
    assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 50);
    assert!(k.sleep_queue.is_empty());
}

// --- SYS_SLEEP_TICKS dispatch tests ---

#[test]
fn sleep_ticks_zero_returns_immediately() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(50);
    let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 0, 0);
    test_dispatch(&mut k, &mut ef);
    assert_eq!(ef.r0, 0);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Running
    );
    assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 0);
    assert!(k.sleep_queue.is_empty());
    assert!(!k.yield_requested);
}

#[test]
fn sleep_ticks_nonzero_blocks_partition() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(100);
    let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 50, 0);
    test_dispatch(&mut k, &mut ef);
    assert_eq!(ef.r0, 0);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
    assert_eq!(k.partitions().get(0).unwrap().sleep_until(), 150);
    assert!(!k.sleep_queue.is_empty());
    assert!(k.yield_requested);
}

#[test]
fn sleep_ticks_queue_full_returns_error() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(10);
    for i in 0..4u8 {
        let _ = k
            .sleep_queue
            .push(PartitionId::new(i as u32), 1000 + i as u64);
    }
    let mut ef = frame(crate::syscall::SYS_SLEEP_TICKS, 50, 0);
    test_dispatch(&mut k, &mut ef);
    assert_eq!(ef.r0, SvcError::WaitQueueFull.to_u32());
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Running
    );
}

// --- expire_timed_waits: blocked receiver/sender tests ---

#[test]
fn expire_timed_waits_wakes_blocked_receiver() {
    use crate::queuing::RecvQueuingOutcome;
    use crate::sampling::PortDirection;

    let mut k = kernel(0, 0, 0);
    // Create a destination queuing port (empty queue → recv blocks).
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .unwrap();

    // Partition 0 attempts a timed receive with timeout=50 at tick=100.
    // Queue is empty so the receiver gets blocked with expiry=150.
    let mut buf = [0u8; 4];
    let outcome = k
        .queuing_mut()
        .receive_queuing_message(dst, PartitionId::new(0), &mut buf, 50, 100)
        .unwrap();
    assert_eq!(
        outcome,
        RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 150 }
    );

    // Simulate the SVC handler: transition partition 0 from Running→Waiting.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );

    // Tick 150: expiry fires, partition should move Waiting→Ready.
    k.expire_timed_waits::<8>(150);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Ready
    );
}

#[test]
fn expire_timed_waits_wakes_blocked_sender() {
    use crate::queuing::SendQueuingOutcome;
    use crate::sampling::PortDirection;

    let mut k = kernel(0, 0, 0);
    // Create a source port and a connected destination port.
    let src = k.queuing_mut().create_port(PortDirection::Source).unwrap();
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .unwrap();
    k.queuing_mut().connect_ports(src, dst).unwrap();

    // Fill the destination queue (depth=4).
    for i in 0..4u8 {
        let outcome = k
            .queuing_mut()
            .send_routed(src, PartitionId::new(0), &[i; 4], 0, 0)
            .unwrap();
        assert!(matches!(outcome, SendQueuingOutcome::Delivered { .. }));
    }

    // Partition 1 attempts a timed send with timeout=100 at tick=50.
    // Queue is full so the sender blocks with expiry=150.
    let outcome = k
        .queuing_mut()
        .send_routed(src, PartitionId::new(1), &[0xFF; 4], 100, 50)
        .unwrap();
    assert_eq!(
        outcome,
        SendQueuingOutcome::SenderBlocked { expiry_tick: 150 }
    );

    // Simulate the SVC handler: transition partition 1 Running→Waiting.
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Waiting
    );

    // Tick 150: expiry fires, partition should move Waiting→Ready.
    k.expire_timed_waits::<8>(150);
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Ready
    );
}

#[test]
fn expire_timed_waits_non_expired_stays_waiting() {
    use crate::queuing::RecvQueuingOutcome;
    use crate::sampling::PortDirection;

    let mut k = kernel(0, 0, 0);
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .unwrap();

    // Block partition 0 as receiver with expiry at tick 200.
    let mut buf = [0u8; 4];
    let outcome = k
        .queuing_mut()
        .receive_queuing_message(dst, PartitionId::new(0), &mut buf, 100, 100)
        .unwrap();
    assert_eq!(
        outcome,
        RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 200 }
    );

    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();

    // Tick 150: before expiry — partition must remain Waiting.
    k.expire_timed_waits::<8>(150);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );

    // Tick 199: still before expiry.
    k.expire_timed_waits::<8>(199);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
}

// --- device wait queue expiry tests ---

#[test]
fn expire_timed_waits_device_reader_expiry() {
    let mut k = kernel(0, 0, 0);
    k.dev_wait_queue_mut()
        .block_reader(PartitionId::new(0), 100)
        .unwrap();
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Waiting)
        .unwrap();
    // Before expiry: stays Waiting.
    k.expire_timed_waits::<8>(99);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
    assert_eq!(k.dev_wait_queue().len(), 1);
    // At expiry: transitions Waiting→Ready.
    k.expire_timed_waits::<8>(100);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Ready
    );
    assert!(k.dev_wait_queue().is_empty());
}
