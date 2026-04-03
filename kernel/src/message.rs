use crate::waitqueue::WaitQueue;
use rtos_traits::ids::PartitionId;

#[derive(Debug, PartialEq, Eq)]
pub enum MsgError {
    InvalidQueue,
    WaitQueueFull,
    SizeMismatch,
}

/// Outcome of a successful `send` operation.
#[derive(Debug, PartialEq, Eq)]
pub enum SendOutcome {
    /// Message was enqueued. If `wake_receiver` is `Some(pid)`, the kernel
    /// should transition that partition from Waiting to Ready.
    Delivered { wake_receiver: Option<PartitionId> },
    /// Queue was full; the caller (pid = `blocked`) has been placed on the
    /// sender wait-queue. The kernel should transition it to Waiting.
    SenderBlocked { blocked: PartitionId },
}

/// Outcome of a successful `recv` operation.
#[derive(Debug, PartialEq, Eq)]
pub enum RecvOutcome {
    /// A message was dequeued into the caller's buffer. If `wake_sender` is
    /// `Some(pid)`, the kernel should transition that partition from Waiting
    /// to Ready.
    Received { wake_sender: Option<PartitionId> },
    /// Queue was empty; the caller (pid = `blocked`) has been placed on the
    /// receiver wait-queue. The kernel should transition it to Waiting.
    ReceiverBlocked { blocked: PartitionId },
}

/// Fixed-depth, fixed-message-size queue with independent const-generic
/// parameters. Each queue can be instantiated with different `D` (depth),
/// `M` (message size in bytes), and `W` (wait-queue capacity).
#[derive(Debug)]
pub struct MessageQueue<const D: usize, const M: usize, const W: usize> {
    buf: heapless::Deque<[u8; M], D>,
    sender_wq: WaitQueue<W>,
    receiver_wq: WaitQueue<W>,
}

#[allow(clippy::new_without_default)]
impl<const D: usize, const M: usize, const W: usize> MessageQueue<D, M, W> {
    pub const fn new() -> Self {
        Self {
            buf: heapless::Deque::new(),
            sender_wq: WaitQueue::new(),
            receiver_wq: WaitQueue::new(),
        }
    }

    pub fn depth(&self) -> usize {
        self.buf.len()
    }

    /// Attempt to enqueue `data` into this queue.
    ///
    /// Returns `SendOutcome` describing what the kernel should do, or
    /// `MsgError` on invalid arguments.
    pub fn send(&mut self, caller: usize, data: &[u8]) -> Result<SendOutcome, MsgError> {
        if data.len() != M {
            return Err(MsgError::SizeMismatch);
        }
        let mut msg = [0u8; M];
        // Defense-in-depth: use get/get_mut to avoid panicking indexing
        // even though data.len() == M is already checked above.
        let dst = msg.get_mut(..M).ok_or(MsgError::SizeMismatch)?;
        let src = data.get(..M).ok_or(MsgError::SizeMismatch)?;
        dst.copy_from_slice(src);
        let caller_pid = PartitionId::new(caller as u32);
        if self.buf.push_back(msg).is_err() {
            self.sender_wq
                .push(caller_pid)
                .map_err(|_| MsgError::WaitQueueFull)?;
            return Ok(SendOutcome::SenderBlocked {
                blocked: caller_pid,
            });
        }
        let wake = self.receiver_wq.pop_front();
        Ok(SendOutcome::Delivered {
            wake_receiver: wake,
        })
    }

    /// Attempt to dequeue a message into `buf`.
    ///
    /// Returns `RecvOutcome` describing what the kernel should do, or
    /// `MsgError` on invalid arguments.
    pub fn recv(&mut self, caller: usize, buf: &mut [u8]) -> Result<RecvOutcome, MsgError> {
        if buf.len() != M {
            return Err(MsgError::SizeMismatch);
        }
        if let Some(msg) = self.buf.pop_front() {
            // Defense-in-depth: use get/get_mut to avoid panicking indexing
            // even though buf.len() == M is already checked above.
            let dst = buf.get_mut(..M).ok_or(MsgError::SizeMismatch)?;
            let src = msg.get(..M).ok_or(MsgError::SizeMismatch)?;
            dst.copy_from_slice(src);
            let wake = self.sender_wq.pop_front();
            return Ok(RecvOutcome::Received { wake_sender: wake });
        }
        let caller_pid = PartitionId::new(caller as u32);
        self.receiver_wq
            .push(caller_pid)
            .map_err(|_| MsgError::WaitQueueFull)?;
        Ok(RecvOutcome::ReceiverBlocked {
            blocked: caller_pid,
        })
    }

    /// Return the compile-time message size for this queue.
    pub const fn msg_size(&self) -> usize {
        M
    }
}

/// Fixed-capacity pool of identically-typed message queues, indexed by ID.
/// `S` = max number of queues, `D`/`M`/`W` = per-queue depth/msg-size/wait-capacity.
pub struct MessagePool<const S: usize, const D: usize, const M: usize, const W: usize> {
    queues: heapless::Vec<MessageQueue<D, M, W>, S>,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const D: usize, const M: usize, const W: usize> MessagePool<S, D, M, W> {
    pub const fn new() -> Self {
        Self {
            queues: heapless::Vec::new(),
        }
    }

    pub fn add(&mut self, queue: MessageQueue<D, M, W>) -> Result<(), MessageQueue<D, M, W>> {
        self.queues.push(queue)
    }

    pub fn get(&self, id: usize) -> Option<&MessageQueue<D, M, W>> {
        self.queues.get(id)
    }

    pub fn send(
        &mut self,
        queue_id: usize,
        caller: usize,
        data: &[u8],
    ) -> Result<SendOutcome, MsgError> {
        let q = self
            .queues
            .get_mut(queue_id)
            .ok_or(MsgError::InvalidQueue)?;
        q.send(caller, data)
    }

    pub fn recv(
        &mut self,
        queue_id: usize,
        caller: usize,
        buf: &mut [u8],
    ) -> Result<RecvOutcome, MsgError> {
        let q = self
            .queues
            .get_mut(queue_id)
            .ok_or(MsgError::InvalidQueue)?;
        q.recv(caller, buf)
    }

    /// Return the compile-time message size `M` for queues in this pool.
    pub const fn msg_size(&self) -> usize {
        M
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pid(v: u32) -> PartitionId {
        PartitionId::new(v)
    }

    #[test]
    fn send_and_recv_basic() {
        let mut q = MessageQueue::<4, 4, 4>::new();
        let outcome = q.send(0, &[1, 2, 3, 4]).unwrap();
        assert_eq!(
            outcome,
            SendOutcome::Delivered {
                wake_receiver: None
            }
        );
        let mut buf = [0u8; 4];
        let outcome = q.recv(1, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::Received { wake_sender: None });
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn fifo_ordering() {
        let mut q = MessageQueue::<4, 4, 4>::new();
        q.send(0, &[10, 20, 30, 40]).unwrap();
        q.send(0, &[50, 60, 70, 80]).unwrap();
        let mut b1 = [0u8; 4];
        let mut b2 = [0u8; 4];
        q.recv(1, &mut b1).unwrap();
        q.recv(1, &mut b2).unwrap();
        assert_eq!(b1, [10, 20, 30, 40]);
        assert_eq!(b2, [50, 60, 70, 80]);
    }

    #[test]
    fn full_queue_blocks_sender() {
        let mut q = MessageQueue::<2, 4, 4>::new();
        q.send(0, &[1; 4]).unwrap();
        q.send(0, &[2; 4]).unwrap();
        let outcome = q.send(1, &[3; 4]).unwrap();
        assert_eq!(outcome, SendOutcome::SenderBlocked { blocked: pid(1) });
    }

    #[test]
    fn recv_wakes_blocked_sender() {
        let mut q = MessageQueue::<1, 4, 4>::new();
        q.send(0, &[1; 4]).unwrap();
        // Queue full, sender 1 blocks
        let outcome = q.send(1, &[2; 4]).unwrap();
        assert_eq!(outcome, SendOutcome::SenderBlocked { blocked: pid(1) });
        // Recv should pop a message and report that sender 1 can be woken
        let mut buf = [0u8; 4];
        let outcome = q.recv(2, &mut buf).unwrap();
        assert_eq!(
            outcome,
            RecvOutcome::Received {
                wake_sender: Some(pid(1))
            }
        );
        assert_eq!(buf, [1; 4]);
    }

    #[test]
    fn empty_queue_blocks_receiver() {
        let mut q = MessageQueue::<4, 4, 4>::new();
        let mut buf = [0u8; 4];
        let outcome = q.recv(0, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::ReceiverBlocked { blocked: pid(0) });
    }

    #[test]
    fn send_wakes_blocked_receiver() {
        let mut q = MessageQueue::<4, 4, 4>::new();
        // Receiver 0 blocks on empty queue
        let mut buf = [0u8; 4];
        let outcome = q.recv(0, &mut buf).unwrap();
        assert_eq!(outcome, RecvOutcome::ReceiverBlocked { blocked: pid(0) });
        // Send should deliver and report that receiver 0 can be woken
        let outcome = q.send(1, &[5, 6, 7, 8]).unwrap();
        assert_eq!(
            outcome,
            SendOutcome::Delivered {
                wake_receiver: Some(pid(0))
            }
        );
    }

    #[test]
    fn size_mismatch_errors() {
        let mut q = MessageQueue::<4, 4, 4>::new();
        assert_eq!(q.send(0, &[1; 2]), Err(MsgError::SizeMismatch));
        assert_eq!(q.recv(0, &mut [0u8; 2]), Err(MsgError::SizeMismatch));
    }

    #[test]
    fn sender_wait_queue_full() {
        let mut q = MessageQueue::<1, 4, 2>::new();
        q.send(0, &[1; 4]).unwrap(); // fills the buffer
        q.send(1, &[2; 4]).unwrap(); // blocks, wq slot 1
        q.send(2, &[3; 4]).unwrap(); // blocks, wq slot 2
        assert_eq!(q.send(3, &[4; 4]), Err(MsgError::WaitQueueFull));
    }

    #[test]
    fn receiver_wait_queue_full() {
        let mut q = MessageQueue::<4, 4, 2>::new();
        let mut buf = [0u8; 4];
        q.recv(0, &mut buf).unwrap(); // blocks, wq slot 1
        q.recv(1, &mut buf).unwrap(); // blocks, wq slot 2
        assert_eq!(q.recv(2, &mut buf), Err(MsgError::WaitQueueFull));
    }

    #[test]
    fn heterogeneous_queue_types_compile() {
        // Demonstrates that different queues can have independent parameters,
        // unlike the old MessagePool design which forced identical configs.
        let mut small: MessageQueue<2, 1, 2> = MessageQueue::new();
        let mut large: MessageQueue<8, 16, 4> = MessageQueue::new();
        small.send(0, &[0xAA]).unwrap();
        large.send(0, &[0u8; 16]).unwrap();
        let mut sb = [0u8; 1];
        let mut lb = [0u8; 16];
        small.recv(1, &mut sb).unwrap();
        large.recv(1, &mut lb).unwrap();
        assert_eq!(sb, [0xAA]);
        assert_eq!(lb, [0u8; 16]);
    }

    #[test]
    fn pool_send_recv_by_queue_id() {
        let mut pool = MessagePool::<4, 4, 4, 4>::new();
        pool.add(MessageQueue::new()).unwrap();
        pool.add(MessageQueue::new()).unwrap();

        pool.send(0, 0, &[1, 2, 3, 4]).unwrap();
        pool.send(1, 0, &[5, 6, 7, 8]).unwrap();

        let mut buf = [0u8; 4];
        pool.recv(0, 1, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
        pool.recv(1, 1, &mut buf).unwrap();
        assert_eq!(buf, [5, 6, 7, 8]);
    }

    #[test]
    fn pool_invalid_queue_id() {
        let mut pool = MessagePool::<4, 4, 4, 4>::new();
        assert_eq!(pool.send(0, 0, &[1; 4]), Err(MsgError::InvalidQueue));
        assert_eq!(pool.recv(0, 0, &mut [0; 4]), Err(MsgError::InvalidQueue));
    }
}
