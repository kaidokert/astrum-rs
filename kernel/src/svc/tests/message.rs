use super::*;

// -------------------------------------------------------------------------
// Message dispatch tests
// -------------------------------------------------------------------------

#[test]
fn msg_send_recv_pointer_based() {
    let mut k = kernel(0, 0, 2);
    // Send to queue 0 from partition 0
    let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
    let outcome = k.messages_mut().send(0, 0, &data).unwrap();
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

    // Receive from queue 0 into partition 1's buffer
    let mut recv_buf = [0u8; 4];
    let outcome = k.messages_mut().recv(0, 1, &mut recv_buf).unwrap();
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
    assert_eq!(recv_buf, [0xDE, 0xAD, 0xBE, 0xEF]);
}

#[test]
fn msg_send_recv_multiple_queues() {
    let mut k = kernel(0, 0, 2);
    // Send different data to queue 0 and queue 1
    let data_q0: [u8; 4] = [1, 2, 3, 4];
    let data_q1: [u8; 4] = [5, 6, 7, 8];

    let outcome = k.messages_mut().send(0, 0, &data_q0).unwrap();
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

    let outcome = k.messages_mut().send(1, 0, &data_q1).unwrap();
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));

    // Recv from queue 1 first
    let mut buf = [0u8; 4];
    let outcome = k.messages_mut().recv(1, 1, &mut buf).unwrap();
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
    assert_eq!(buf, [5, 6, 7, 8]);

    // Recv from queue 0
    let mut buf = [0u8; 4];
    let outcome = k.messages_mut().recv(0, 1, &mut buf).unwrap();
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
    assert_eq!(buf, [1, 2, 3, 4]);
}

#[test]
fn msg_invalid_queue_id_returns_max() {
    let mut k = kernel(0, 0, 1);
    assert!(k.messages_mut().send(99, 0, &[1; 4]).is_err());
    assert!(k.messages_mut().recv(99, 0, &mut [0; 4]).is_err());
}

#[test]
fn msg_send_blocks_and_wakes() {
    let mut k = kernel(0, 0, 1);
    // Fill the depth-4 queue to capacity
    for i in 0..4u8 {
        let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
    }

    // Next send blocks partition 1
    let outcome = k.messages_mut().send(0, 1, &[99; 4]).unwrap();
    assert_eq!(
        outcome,
        SendOutcome::SenderBlocked {
            blocked: PartitionId::new(1)
        }
    );
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(Some(1)));
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Waiting
    );

    // Recv should wake partition 1
    let mut buf = [0u8; 4];
    let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
    assert_eq!(
        outcome,
        RecvOutcome::Received {
            wake_sender: Some(PartitionId::new(1))
        }
    );
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(None));
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Ready
    );
    assert_eq!(buf, [0; 4]); // first message enqueued
}

#[test]
fn msg_recv_blocks_and_wakes() {
    let mut k = kernel(0, 0, 1);
    // Recv on empty queue blocks partition 0
    let mut buf = [0u8; 4];
    let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
    assert_eq!(
        outcome,
        RecvOutcome::ReceiverBlocked {
            blocked: PartitionId::new(0)
        }
    );
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(Some(0)));
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );

    // Send should wake partition 0
    let outcome = k.messages_mut().send(0, 1, &[9; 4]).unwrap();
    assert_eq!(
        outcome,
        SendOutcome::Delivered {
            wake_receiver: Some(PartitionId::new(0))
        }
    );
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Ready
    );
}

#[test]
fn msg_send_blocks_sets_yield_requested() {
    let mut k = kernel(0, 0, 1);
    // Fill the depth-4 queue to capacity
    for i in 0..4u8 {
        let outcome = k.messages_mut().send(0, 0, &[i; 4]).unwrap();
        assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(None));
    }
    // yield_requested should still be false after successful sends
    assert!(!k.yield_requested());

    // Next send blocks partition 1
    let outcome = k.messages_mut().send(0, 1, &[99; 4]).unwrap();
    assert_eq!(
        outcome,
        SendOutcome::SenderBlocked {
            blocked: PartitionId::new(1)
        }
    );
    assert_eq!(apply_send_outcome(k.partitions_mut(), outcome), Ok(Some(1)));
    // Caller is responsible for triggering deschedule
    k.trigger_deschedule();

    // After blocking and triggering deschedule, yield_requested should be true
    assert!(k.yield_requested());
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Waiting
    );
}

#[test]
fn msg_recv_blocks_sets_yield_requested() {
    let mut k = kernel(0, 0, 1);
    // yield_requested should initially be false
    assert!(!k.yield_requested());

    // Recv on empty queue blocks partition 0
    let mut buf = [0u8; 4];
    let outcome = k.messages_mut().recv(0, 0, &mut buf).unwrap();
    assert_eq!(
        outcome,
        RecvOutcome::ReceiverBlocked {
            blocked: PartitionId::new(0)
        }
    );

    // apply_recv_outcome calls trigger_deschedule internally
    assert_eq!(apply_recv_outcome(&mut k, outcome), Ok(Some(0)));

    // yield_requested should now be true after blocking
    assert!(k.yield_requested());
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
}
