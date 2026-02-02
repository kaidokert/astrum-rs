use crate::sampling::PortDirection;

/// Status information for a queuing port, returned by the `QueuingStatus` syscall.
/// `#[repr(C)]` ensures a stable layout for writing directly to user-provided pointers.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueuingPortStatus {
    pub nb_messages: u32,
    pub max_nb_messages: u32,
    pub max_message_size: u32,
    /// 0 = Source, 1 = Destination.
    pub direction: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum QueuingError {
    PoolFull,
    DirectionViolation,
    MessageTooLarge,
    QueueFull,
    QueueEmpty,
    WaitQueueFull,
    InvalidPort,
}

#[derive(Debug, PartialEq, Eq)]
pub enum SendQueuingOutcome {
    Delivered { wake_receiver: Option<u8> },
    SenderBlocked { expiry_tick: u64 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum RecvQueuingOutcome {
    Received {
        msg_len: usize,
        wake_sender: Option<u8>,
    },
    ReceiverBlocked {
        expiry_tick: u64,
    },
}

/// A queuing port with FIFO message buffering.
/// `D` = depth, `M` = max message size in bytes, `W` = wait-queue capacity.
///
/// Each enqueued message stores its actual length alongside the fixed-size
/// buffer so that `recv` can report the original size to the caller.
#[derive(Debug)]
pub struct QueuingPort<const D: usize, const M: usize, const W: usize> {
    direction: PortDirection,
    buf: heapless::Deque<(usize, [u8; M]), D>,
    sender_wq: heapless::Deque<(u8, u64), W>,
    receiver_wq: heapless::Deque<(u8, u64), W>,
    connected_port: Option<usize>,
}

#[allow(clippy::new_without_default)]
impl<const D: usize, const M: usize, const W: usize> QueuingPort<D, M, W> {
    pub const fn new(direction: PortDirection) -> Self {
        Self {
            direction,
            buf: heapless::Deque::new(),
            sender_wq: heapless::Deque::new(),
            receiver_wq: heapless::Deque::new(),
            connected_port: None,
        }
    }

    pub fn direction(&self) -> PortDirection {
        self.direction
    }

    pub const fn max_msg_size(&self) -> usize {
        M
    }

    pub const fn max_nb_messages(&self) -> usize {
        D
    }

    pub fn nb_messages(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    pub fn is_full(&self) -> bool {
        self.buf.is_full()
    }

    pub fn pending_senders(&self) -> usize {
        self.sender_wq.len()
    }

    pub fn pending_receivers(&self) -> usize {
        self.receiver_wq.len()
    }

    /// Enqueue a message. Only allowed on Source ports.
    /// Returns `Ok(wake_receiver)` — if a receiver was blocked, its partition
    /// ID is returned so the kernel can wake it.
    pub fn send(&mut self, caller: u8, data: &[u8]) -> Result<Option<u8>, QueuingError> {
        if self.direction != PortDirection::Source {
            return Err(QueuingError::DirectionViolation);
        }
        if data.len() > M {
            return Err(QueuingError::MessageTooLarge);
        }
        let mut msg = [0u8; M];
        msg[..data.len()].copy_from_slice(data);
        if self.buf.push_back((data.len(), msg)).is_err() {
            // u64::MAX = no timeout; kernel wakes this sender unconditionally on next recv.
            self.sender_wq
                .push_back((caller, u64::MAX))
                .map_err(|_| QueuingError::WaitQueueFull)?;
            return Err(QueuingError::QueueFull);
        }
        Ok(self.receiver_wq.pop_front().map(|(pid, _)| pid))
    }

    pub fn send_queuing_message(
        &mut self,
        caller: u8,
        data: &[u8],
        timeout_ticks: u64,
        current_tick: u64,
    ) -> Result<SendQueuingOutcome, QueuingError> {
        if self.direction != PortDirection::Source {
            return Err(QueuingError::DirectionViolation);
        }
        if data.len() > M {
            return Err(QueuingError::MessageTooLarge);
        }
        if self.buf.is_full() {
            if timeout_ticks == 0 {
                return Err(QueuingError::QueueFull);
            }
            let expiry = current_tick + timeout_ticks;
            self.sender_wq
                .push_back((caller, expiry))
                .map_err(|_| QueuingError::WaitQueueFull)?;
            return Ok(SendQueuingOutcome::SenderBlocked {
                expiry_tick: expiry,
            });
        }
        let mut msg = [0u8; M];
        msg[..data.len()].copy_from_slice(data);
        // push_back cannot fail: we just confirmed the buffer is not full.
        let _ = self.buf.push_back((data.len(), msg));
        Ok(SendQueuingOutcome::Delivered {
            wake_receiver: self.receiver_wq.pop_front().map(|(pid, _)| pid),
        })
    }

    /// Dequeue a message. Only allowed on Destination ports.
    ///
    /// Copies up to `buf.len()` bytes of the message into `buf`. Returns
    /// `Ok((msg_len, wake_sender))` where `msg_len` is the original message
    /// length. If `buf` is smaller than the message, the data is truncated
    /// but `msg_len` still reflects the full size so the caller can detect
    /// truncation (`msg_len > buf.len()`).
    pub fn recv(
        &mut self,
        caller: u8,
        buf: &mut [u8],
    ) -> Result<(usize, Option<u8>), QueuingError> {
        if self.direction != PortDirection::Destination {
            return Err(QueuingError::DirectionViolation);
        }
        if let Some((len, msg)) = self.buf.pop_front() {
            let copy_len = len.min(buf.len());
            buf[..copy_len].copy_from_slice(&msg[..copy_len]);
            return Ok((len, self.sender_wq.pop_front().map(|(pid, _)| pid)));
        }
        self.receiver_wq
            .push_back((caller, u64::MAX))
            .map_err(|_| QueuingError::WaitQueueFull)?;
        Err(QueuingError::QueueEmpty)
    }

    pub fn receive_queuing_message(
        &mut self,
        caller: u8,
        buf: &mut [u8],
        timeout_ticks: u64,
        current_tick: u64,
    ) -> Result<RecvQueuingOutcome, QueuingError> {
        if self.direction != PortDirection::Destination {
            return Err(QueuingError::DirectionViolation);
        }
        if let Some((len, msg)) = self.buf.pop_front() {
            let copy_len = len.min(buf.len());
            buf[..copy_len].copy_from_slice(&msg[..copy_len]);
            return Ok(RecvQueuingOutcome::Received {
                msg_len: len,
                wake_sender: self.sender_wq.pop_front().map(|(pid, _)| pid),
            });
        }
        if timeout_ticks == 0 {
            return Err(QueuingError::QueueEmpty);
        }
        let expiry = current_tick + timeout_ticks;
        self.receiver_wq
            .push_back((caller, expiry))
            .map_err(|_| QueuingError::WaitQueueFull)?;
        Ok(RecvQueuingOutcome::ReceiverBlocked {
            expiry_tick: expiry,
        })
    }

    /// Remove expired waiters from `wq`, appending their partition IDs to `out`.
    /// Skips the destructive pop/push cycle when no entries have expired.
    ///
    /// The caller must ensure `out` has enough remaining capacity for all
    /// expired entries. In debug builds this is asserted; in release the
    /// excess entries are silently dropped (the waiter is still removed from
    /// the queue, so it will not block forever).
    fn drain_expired<const E: usize>(
        wq: &mut heapless::Deque<(u8, u64), W>,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        if !wq.iter().any(|&(_, expiry)| current_tick >= expiry) {
            return;
        }
        let n = wq.len();
        for _ in 0..n {
            if let Some((pid, expiry)) = wq.pop_front() {
                if current_tick >= expiry {
                    debug_assert!(
                        !out.is_full(),
                        "tick_timeouts: output vec capacity E={} is too small",
                        E
                    );
                    let _ = out.push(pid);
                } else {
                    let _ = wq.push_back((pid, expiry));
                }
            }
        }
    }

    /// Remove senders whose timeout has expired, appending their partition IDs
    /// to `out`. The caller provides the output vector so that capacity is
    /// managed in one place (see `tick_timeouts`).
    pub fn drain_expired_senders<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        Self::drain_expired(&mut self.sender_wq, current_tick, out);
    }

    /// Remove receivers whose timeout has expired, appending their partition
    /// IDs to `out`.
    pub fn drain_expired_receivers<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        Self::drain_expired(&mut self.receiver_wq, current_tick, out);
    }

    /// Returns the full status of this queuing port as a C-compatible struct.
    pub fn status(&self) -> QueuingPortStatus {
        QueuingPortStatus {
            nb_messages: self.buf.len() as u32,
            max_nb_messages: D as u32,
            max_message_size: M as u32,
            direction: match self.direction {
                PortDirection::Source => 0,
                PortDirection::Destination => 1,
            },
        }
    }
}

/// Fixed-capacity pool of queuing ports.
/// `S` = max ports, `D` = depth, `M` = message size, `W` = wait-queue capacity.
///
/// Port IDs are vector indices, giving O(1) lookup.
pub struct QueuingPortPool<const S: usize, const D: usize, const M: usize, const W: usize> {
    ports: heapless::Vec<QueuingPort<D, M, W>, S>,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const D: usize, const M: usize, const W: usize> QueuingPortPool<S, D, M, W> {
    pub const fn new() -> Self {
        Self {
            ports: heapless::Vec::new(),
        }
    }

    pub fn create_port(&mut self, direction: PortDirection) -> Result<usize, QueuingError> {
        let id = self.ports.len();
        self.ports
            .push(QueuingPort::new(direction))
            .map_err(|_| QueuingError::PoolFull)?;
        Ok(id)
    }

    pub fn get(&self, id: usize) -> Option<&QueuingPort<D, M, W>> {
        self.ports.get(id)
    }

    pub fn get_mut(&mut self, id: usize) -> Option<&mut QueuingPort<D, M, W>> {
        self.ports.get_mut(id)
    }

    pub fn len(&self) -> usize {
        self.ports.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ports.is_empty()
    }

    pub fn send_queuing_message(
        &mut self,
        port_id: usize,
        caller: u8,
        data: &[u8],
        timeout_ticks: u64,
        current_tick: u64,
    ) -> Result<SendQueuingOutcome, QueuingError> {
        self.ports
            .get_mut(port_id)
            .ok_or(QueuingError::InvalidPort)?
            .send_queuing_message(caller, data, timeout_ticks, current_tick)
    }

    pub fn receive_queuing_message(
        &mut self,
        port_id: usize,
        caller: u8,
        buf: &mut [u8],
        timeout_ticks: u64,
        current_tick: u64,
    ) -> Result<RecvQueuingOutcome, QueuingError> {
        self.ports
            .get_mut(port_id)
            .ok_or(QueuingError::InvalidPort)?
            .receive_queuing_message(caller, buf, timeout_ticks, current_tick)
    }

    pub fn get_queuing_port_status(
        &self,
        port_id: usize,
    ) -> Result<QueuingPortStatus, QueuingError> {
        self.ports
            .get(port_id)
            .ok_or(QueuingError::InvalidPort)
            .map(|p| p.status())
    }

    /// Check all queuing port wait queues for expired timeouts.
    /// Returns the partition IDs that were unblocked (up to `E` entries).
    /// The kernel should transition each returned partition to Ready.
    ///
    /// # Panics
    ///
    /// Panics (debug) if more than `E` waiters expire in a single tick.
    /// The caller must size `E` to cover the worst case (sum of all wait-queue
    /// capacities across every port).
    pub fn tick_timeouts<const E: usize>(&mut self, current_tick: u64) -> heapless::Vec<u8, E> {
        let mut unblocked = heapless::Vec::new();
        for port in self.ports.iter_mut() {
            port.drain_expired_senders(current_tick, &mut unblocked);
            port.drain_expired_receivers(current_tick, &mut unblocked);
        }
        unblocked
    }

    pub fn connect_ports(&mut self, src_id: usize, dst_id: usize) -> Result<(), QueuingError> {
        if src_id >= self.ports.len() || dst_id >= self.ports.len() {
            return Err(QueuingError::InvalidPort);
        }
        if self.ports[src_id].direction != PortDirection::Source {
            return Err(QueuingError::DirectionViolation);
        }
        if self.ports[dst_id].direction != PortDirection::Destination {
            return Err(QueuingError::DirectionViolation);
        }
        self.ports[src_id].connected_port = Some(dst_id);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_port_and_initial_state() {
        let mut pool = QueuingPortPool::<4, 8, 16, 4>::new();
        assert!(pool.is_empty());
        let id = pool.create_port(PortDirection::Source).unwrap();
        let p = pool.get(id).unwrap();
        assert_eq!(p.direction(), PortDirection::Source);
        assert_eq!((p.max_msg_size(), p.max_nb_messages()), (16, 8));
        assert_eq!(p.nb_messages(), 0);
        assert!(p.is_empty());
        assert!(!p.is_full());
        assert_eq!((p.pending_senders(), p.pending_receivers()), (0, 0));
        let id2 = pool.create_port(PortDirection::Destination).unwrap();
        assert_ne!(id, id2);
        assert_eq!(
            pool.get(id2).unwrap().direction(),
            PortDirection::Destination
        );
        assert_eq!(pool.len(), 2);
        assert!(pool.get(99).is_none());
    }

    #[test]
    fn pool_capacity_and_get_mut() {
        let mut pool = QueuingPortPool::<2, 4, 8, 2>::new();
        let id = pool.create_port(PortDirection::Source).unwrap();
        pool.create_port(PortDirection::Destination).unwrap();
        assert_eq!(
            pool.create_port(PortDirection::Source),
            Err(QueuingError::PoolFull)
        );
        assert_eq!(pool.len(), 2);
        pool.get_mut(id).unwrap().send(0, &[1; 8]).unwrap();
        assert_eq!(pool.get(id).unwrap().nb_messages(), 1);
    }

    #[test]
    fn send_recv_and_direction_errors() {
        let mut src = QueuingPort::<4, 4, 4>::new(PortDirection::Source);
        assert_eq!(src.send(0, &[1, 2, 3, 4]).unwrap(), None);
        assert_eq!(src.nb_messages(), 1);
        // Direction violations
        let mut dst = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        assert_eq!(dst.send(0, &[1; 4]), Err(QueuingError::DirectionViolation));
        let mut buf = [0u8; 4];
        assert_eq!(src.recv(0, &mut buf), Err(QueuingError::DirectionViolation));
        // Size errors
        assert_eq!(src.send(0, &[1; 5]), Err(QueuingError::MessageTooLarge));
    }

    #[test]
    fn queue_full_and_empty_blocking() {
        let mut src = QueuingPort::<2, 4, 4>::new(PortDirection::Source);
        src.send(0, &[1; 4]).unwrap();
        src.send(0, &[2; 4]).unwrap();
        assert!(src.is_full());
        assert_eq!(src.send(1, &[3; 4]), Err(QueuingError::QueueFull));
        assert_eq!(src.pending_senders(), 1);
        let mut dst = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        let mut buf = [0u8; 4];
        assert_eq!(dst.recv(0, &mut buf), Err(QueuingError::QueueEmpty));
        assert_eq!(dst.pending_receivers(), 1);
    }

    #[test]
    fn wake_blocked_receiver_and_sender() {
        let mut src = QueuingPort::<4, 4, 4>::new(PortDirection::Source);
        src.receiver_wq.push_back((2, u64::MAX)).unwrap();
        assert_eq!(src.send(0, &[1; 4]).unwrap(), Some(2));
        let mut dst = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        dst.buf.push_back((4, [1; 4])).unwrap();
        dst.sender_wq.push_back((3, u64::MAX)).unwrap();
        let mut buf = [0u8; 4];
        let (sz, wake) = dst.recv(1, &mut buf).unwrap();
        assert_eq!((sz, wake), (4, Some(3)));
    }

    #[test]
    fn wait_queue_full_errors() {
        let mut src = QueuingPort::<1, 4, 2>::new(PortDirection::Source);
        src.send(0, &[1; 4]).unwrap();
        let _ = src.send(1, &[2; 4]);
        let _ = src.send(2, &[3; 4]);
        assert_eq!(src.send(3, &[4; 4]), Err(QueuingError::WaitQueueFull));
        let mut dst = QueuingPort::<4, 4, 2>::new(PortDirection::Destination);
        let mut buf = [0u8; 4];
        let _ = dst.recv(0, &mut buf);
        let _ = dst.recv(1, &mut buf);
        assert_eq!(dst.recv(2, &mut buf), Err(QueuingError::WaitQueueFull));
    }

    #[test]
    fn fifo_ordering_and_message_length() {
        let mut src = QueuingPort::<4, 8, 4>::new(PortDirection::Source);
        src.send(0, &[10, 20, 30]).unwrap();
        src.send(0, &[40, 50, 60, 70, 80]).unwrap();
        let mut dst = QueuingPort::<4, 8, 4>::new(PortDirection::Destination);
        while let Some(entry) = src.buf.pop_front() {
            dst.buf.push_back(entry).unwrap();
        }
        // First message: 3 bytes
        let mut b1 = [0u8; 8];
        let (len1, _) = dst.recv(1, &mut b1).unwrap();
        assert_eq!(len1, 3);
        assert_eq!(&b1[..len1], &[10, 20, 30]);
        // Second message: 5 bytes — FIFO order preserved
        let mut b2 = [0u8; 8];
        let (len2, _) = dst.recv(1, &mut b2).unwrap();
        assert_eq!(len2, 5);
        assert_eq!(&b2[..len2], &[40, 50, 60, 70, 80]);
    }

    #[test]
    fn recv_into_smaller_buffer_truncates() {
        let mut src = QueuingPort::<4, 8, 4>::new(PortDirection::Source);
        src.send(0, &[1, 2, 3, 4, 5]).unwrap();
        let mut dst = QueuingPort::<4, 8, 4>::new(PortDirection::Destination);
        while let Some(entry) = src.buf.pop_front() {
            dst.buf.push_back(entry).unwrap();
        }
        let mut small = [0u8; 3];
        let (msg_len, _) = dst.recv(1, &mut small).unwrap();
        // msg_len is the original length; caller detects truncation
        assert_eq!(msg_len, 5);
        assert_eq!(small, [1, 2, 3]);
    }

    #[test]
    fn send_queuing_delivers_when_space_available() {
        let mut p = QueuingPort::<2, 8, 4>::new(PortDirection::Source);
        let o = p.send_queuing_message(0, &[1, 2, 3], 100, 50).unwrap();
        assert_eq!(
            o,
            SendQueuingOutcome::Delivered {
                wake_receiver: None
            }
        );
        assert_eq!(p.nb_messages(), 1);
    }

    #[test]
    fn send_queuing_wakes_blocked_receiver() {
        let mut p = QueuingPort::<2, 8, 4>::new(PortDirection::Source);
        p.receiver_wq.push_back((5, u64::MAX)).unwrap();
        let o = p.send_queuing_message(1, &[4; 4], 0, 0).unwrap();
        assert_eq!(
            o,
            SendQueuingOutcome::Delivered {
                wake_receiver: Some(5)
            }
        );
    }

    #[test]
    fn send_queuing_rejects_destination_port() {
        let mut dst = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        assert_eq!(
            dst.send_queuing_message(0, &[1; 4], 0, 0),
            Err(QueuingError::DirectionViolation)
        );
    }

    #[test]
    fn send_queuing_rejects_oversized_message() {
        let mut p = QueuingPort::<2, 8, 4>::new(PortDirection::Source);
        assert_eq!(
            p.send_queuing_message(0, &[1; 9], 0, 0),
            Err(QueuingError::MessageTooLarge)
        );
    }

    #[test]
    fn send_queuing_full_zero_timeout_returns_queue_full() {
        let mut q = QueuingPort::<1, 4, 4>::new(PortDirection::Source);
        q.send_queuing_message(0, &[1; 4], 0, 0).unwrap();
        assert_eq!(
            q.send_queuing_message(1, &[2; 4], 0, 10),
            Err(QueuingError::QueueFull)
        );
        assert_eq!(q.pending_senders(), 0);
    }

    #[test]
    fn send_queuing_full_nonzero_timeout_blocks_sender() {
        let mut q = QueuingPort::<1, 4, 4>::new(PortDirection::Source);
        q.send_queuing_message(0, &[1; 4], 0, 0).unwrap();
        let o = q.send_queuing_message(1, &[2; 4], 50, 100).unwrap();
        assert_eq!(o, SendQueuingOutcome::SenderBlocked { expiry_tick: 150 });
        assert_eq!(q.pending_senders(), 1);
    }

    #[test]
    fn send_queuing_pool_delivers() {
        let mut pool = QueuingPortPool::<4, 2, 8, 4>::new();
        let id = pool.create_port(PortDirection::Source).unwrap();
        let o = pool.send_queuing_message(id, 0, &[1, 2], 0, 0).unwrap();
        assert_eq!(
            o,
            SendQueuingOutcome::Delivered {
                wake_receiver: None
            }
        );
    }

    #[test]
    fn send_queuing_pool_invalid_port() {
        let mut pool = QueuingPortPool::<4, 2, 8, 4>::new();
        assert_eq!(
            pool.send_queuing_message(99, 0, &[1], 0, 0),
            Err(QueuingError::InvalidPort)
        );
    }

    #[test]
    fn receive_queuing_delivers_and_wakes_sender() {
        let mut p = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        p.buf.push_back((2, [1, 2, 0, 0])).unwrap();
        p.sender_wq.push_back((3, 999)).unwrap();
        let mut buf = [0u8; 4];
        let o = p.receive_queuing_message(0, &mut buf, 0, 0).unwrap();
        assert_eq!(
            o,
            RecvQueuingOutcome::Received {
                msg_len: 2,
                wake_sender: Some(3),
            }
        );
        assert_eq!(&buf[..2], &[1, 2]);
    }

    #[test]
    fn receive_queuing_empty_zero_timeout_returns_error() {
        let mut p = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        let mut buf = [0u8; 4];
        assert_eq!(
            p.receive_queuing_message(0, &mut buf, 0, 10),
            Err(QueuingError::QueueEmpty)
        );
        assert_eq!(p.pending_receivers(), 0);
    }

    #[test]
    fn receive_queuing_empty_nonzero_timeout_blocks() {
        let mut p = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        let mut buf = [0u8; 4];
        let o = p.receive_queuing_message(1, &mut buf, 50, 100).unwrap();
        assert_eq!(o, RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 150 });
        assert_eq!(p.pending_receivers(), 1);
    }

    #[test]
    fn receive_queuing_rejects_source_port() {
        let mut src = QueuingPort::<4, 4, 4>::new(PortDirection::Source);
        let mut buf = [0u8; 4];
        assert_eq!(
            src.receive_queuing_message(0, &mut buf, 0, 0),
            Err(QueuingError::DirectionViolation)
        );
    }

    #[test]
    fn receive_queuing_no_sender_to_wake() {
        let mut p = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        p.buf.push_back((3, [10, 20, 30, 0])).unwrap();
        let mut buf = [0u8; 4];
        let o = p.receive_queuing_message(0, &mut buf, 0, 0).unwrap();
        assert_eq!(
            o,
            RecvQueuingOutcome::Received {
                msg_len: 3,
                wake_sender: None,
            }
        );
        assert_eq!(&buf[..3], &[10, 20, 30]);
    }

    #[test]
    fn status_returns_all_fields() {
        let mut p = QueuingPort::<8, 16, 4>::new(PortDirection::Source);
        let s = p.status();
        assert_eq!(s.nb_messages, 0);
        assert_eq!(s.max_nb_messages, 8);
        assert_eq!(s.max_message_size, 16);
        assert_eq!(s.direction, 0); // Source = 0

        p.buf.push_back((4, [0u8; 16])).unwrap();
        p.buf.push_back((4, [0u8; 16])).unwrap();
        let s = p.status();
        assert_eq!(s.nb_messages, 2);
        assert_eq!(s.max_nb_messages, 8);
        assert_eq!(s.max_message_size, 16);
        assert_eq!(s.direction, 0); // Source = 0
    }

    #[test]
    fn status_via_pool() {
        let mut pool = QueuingPortPool::<4, 4, 8, 4>::new();
        let id = pool.create_port(PortDirection::Destination).unwrap();
        let s = pool.get_queuing_port_status(id).unwrap();
        assert_eq!(s.nb_messages, 0);
        assert_eq!(s.max_nb_messages, 4);
        assert_eq!(s.max_message_size, 8);
        assert_eq!(s.direction, 1); // Destination = 1
    }

    #[test]
    fn status_pool_invalid_port() {
        let pool = QueuingPortPool::<4, 4, 4, 4>::new();
        assert_eq!(
            pool.get_queuing_port_status(99),
            Err(QueuingError::InvalidPort)
        );
    }

    #[test]
    fn connect_ports_sets_connected_port() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        let d = pool.create_port(PortDirection::Destination).unwrap();
        pool.connect_ports(s, d).unwrap();
        assert_eq!(pool.get(s).unwrap().connected_port, Some(d));
    }

    #[test]
    fn connect_ports_rejects_wrong_directions() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        let d1 = pool.create_port(PortDirection::Destination).unwrap();
        let d2 = pool.create_port(PortDirection::Destination).unwrap();
        let s1 = pool.create_port(PortDirection::Source).unwrap();
        // src must be Source
        assert_eq!(
            pool.connect_ports(d1, d2),
            Err(QueuingError::DirectionViolation)
        );
        // dst must be Destination
        assert_eq!(
            pool.connect_ports(s1, s1),
            Err(QueuingError::DirectionViolation)
        );
    }

    #[test]
    fn connect_ports_rejects_invalid_ids() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        assert_eq!(pool.connect_ports(s, 99), Err(QueuingError::InvalidPort));
        assert_eq!(pool.connect_ports(99, s), Err(QueuingError::InvalidPort));
    }

    #[test]
    fn receive_queuing_pool_delegates_and_rejects_invalid() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        let d = pool.create_port(PortDirection::Destination).unwrap();
        pool.get_mut(d)
            .unwrap()
            .buf
            .push_back((1, [42, 0, 0, 0]))
            .unwrap();
        let mut buf = [0u8; 4];
        let o = pool.receive_queuing_message(d, 0, &mut buf, 0, 0).unwrap();
        assert_eq!(
            o,
            RecvQueuingOutcome::Received {
                msg_len: 1,
                wake_sender: None,
            }
        );
        assert_eq!(buf[0], 42);
        assert_eq!(
            pool.receive_queuing_message(99, 0, &mut buf, 0, 0),
            Err(QueuingError::InvalidPort)
        );
    }

    // --- tick_timeouts tests ---

    #[test]
    fn tick_timeouts_expires_single_sender() {
        let mut pool = QueuingPortPool::<4, 1, 4, 4>::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        // Fill queue, then block a sender with expiry at tick 100
        pool.send_queuing_message(s, 0, &[1; 4], 0, 0).unwrap();
        let o = pool.send_queuing_message(s, 1, &[2; 4], 50, 50).unwrap();
        assert_eq!(o, SendQueuingOutcome::SenderBlocked { expiry_tick: 100 });
        assert_eq!(pool.get(s).unwrap().pending_senders(), 1);

        // Tick 99: not yet expired
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(99);
        assert!(unblocked.is_empty());
        assert_eq!(pool.get(s).unwrap().pending_senders(), 1);

        // Tick 100: expired
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(100);
        assert_eq!(unblocked.as_slice(), &[1]);
        assert_eq!(pool.get(s).unwrap().pending_senders(), 0);
    }

    #[test]
    fn tick_timeouts_expires_single_receiver() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        let d = pool.create_port(PortDirection::Destination).unwrap();
        // Block a receiver with expiry at tick 200
        let mut buf = [0u8; 4];
        let o = pool
            .receive_queuing_message(d, 2, &mut buf, 100, 100)
            .unwrap();
        assert_eq!(o, RecvQueuingOutcome::ReceiverBlocked { expiry_tick: 200 });
        assert_eq!(pool.get(d).unwrap().pending_receivers(), 1);

        // Tick 199: not yet expired
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(199);
        assert!(unblocked.is_empty());
        assert_eq!(pool.get(d).unwrap().pending_receivers(), 1);

        // Tick 200: expired
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(200);
        assert_eq!(unblocked.as_slice(), &[2]);
        assert_eq!(pool.get(d).unwrap().pending_receivers(), 0);
    }

    #[test]
    fn tick_timeouts_no_expiry_keeps_waiters() {
        let mut pool = QueuingPortPool::<4, 1, 4, 4>::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        pool.send_queuing_message(s, 0, &[1; 4], 0, 0).unwrap();
        // Block with expiry at tick 500
        pool.send_queuing_message(s, 1, &[2; 4], 400, 100).unwrap();

        // Tick 200: well before expiry
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(200);
        assert!(unblocked.is_empty());
        assert_eq!(pool.get(s).unwrap().pending_senders(), 1);
    }

    #[test]
    fn tick_timeouts_multiple_concurrent() {
        let mut pool = QueuingPortPool::<4, 1, 4, 4>::new();
        let s = pool.create_port(PortDirection::Source).unwrap();
        let d = pool.create_port(PortDirection::Destination).unwrap();

        // Block two senders with different expiries
        pool.send_queuing_message(s, 0, &[1; 4], 0, 0).unwrap();
        pool.send_queuing_message(s, 1, &[2; 4], 50, 50).unwrap(); // expiry 100
        pool.send_queuing_message(s, 2, &[3; 4], 100, 50).unwrap(); // expiry 150

        // Block one receiver
        let mut buf = [0u8; 4];
        pool.receive_queuing_message(d, 3, &mut buf, 50, 50)
            .unwrap(); // expiry 100

        // Tick 100: sender 1 and receiver 3 expire; sender 2 stays
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(100);
        assert_eq!(unblocked.len(), 2);
        assert!(unblocked.contains(&1));
        assert!(unblocked.contains(&3));
        assert_eq!(pool.get(s).unwrap().pending_senders(), 1);
        assert_eq!(pool.get(d).unwrap().pending_receivers(), 0);

        // Tick 150: sender 2 expires
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(150);
        assert_eq!(unblocked.as_slice(), &[2]);
        assert_eq!(pool.get(s).unwrap().pending_senders(), 0);
    }

    #[test]
    fn tick_timeouts_empty_pool_returns_empty() {
        let mut pool = QueuingPortPool::<4, 4, 4, 4>::new();
        pool.create_port(PortDirection::Source).unwrap();
        let unblocked: heapless::Vec<u8, 8> = pool.tick_timeouts(1000);
        assert!(unblocked.is_empty());
    }
}
