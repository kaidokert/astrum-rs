use crate::sampling::PortDirection;

#[derive(Debug, PartialEq, Eq)]
pub enum QueuingError {
    PoolFull,
    DirectionViolation,
    MessageTooLarge,
    QueueFull,
    QueueEmpty,
    WaitQueueFull,
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
    sender_wq: heapless::Deque<u8, W>,
    receiver_wq: heapless::Deque<u8, W>,
}

#[allow(clippy::new_without_default)]
impl<const D: usize, const M: usize, const W: usize> QueuingPort<D, M, W> {
    pub const fn new(direction: PortDirection) -> Self {
        Self {
            direction,
            buf: heapless::Deque::new(),
            sender_wq: heapless::Deque::new(),
            receiver_wq: heapless::Deque::new(),
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
            self.sender_wq
                .push_back(caller)
                .map_err(|_| QueuingError::WaitQueueFull)?;
            return Err(QueuingError::QueueFull);
        }
        Ok(self.receiver_wq.pop_front())
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
            return Ok((len, self.sender_wq.pop_front()));
        }
        self.receiver_wq
            .push_back(caller)
            .map_err(|_| QueuingError::WaitQueueFull)?;
        Err(QueuingError::QueueEmpty)
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
        src.receiver_wq.push_back(2).unwrap();
        assert_eq!(src.send(0, &[1; 4]).unwrap(), Some(2));
        let mut dst = QueuingPort::<4, 4, 4>::new(PortDirection::Destination);
        dst.buf.push_back((4, [1; 4])).unwrap();
        dst.sender_wq.push_back(3).unwrap();
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
}
