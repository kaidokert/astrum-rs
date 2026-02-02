#[derive(Debug, PartialEq, Eq)]
pub enum BlackboardError {
    PoolFull,
    MessageTooLarge,
    InvalidBoard,
    BoardEmpty,
    WaitQueueFull,
}

/// Outcome of a blackboard read operation.
#[derive(Debug, PartialEq, Eq)]
pub enum ReadBlackboardOutcome {
    /// Data was available and copied into the caller's buffer.
    Read { msg_len: usize },
    /// Board was empty and caller has been enqueued (timeout > 0).
    ReaderBlocked,
}

/// Single-message buffer with wake-all semantics.
/// `M` = max message size, `W` = wait-queue capacity.
#[derive(Debug)]
pub struct Blackboard<const M: usize, const W: usize> {
    id: usize,
    data: [u8; M],
    current_size: usize,
    is_empty: bool,
    wait_queue: heapless::Deque<u8, W>,
}

#[allow(clippy::new_without_default)]
impl<const M: usize, const W: usize> Blackboard<M, W> {
    pub const fn new(id: usize) -> Self {
        Self {
            id,
            data: [0u8; M],
            current_size: 0,
            is_empty: true,
            wait_queue: heapless::Deque::new(),
        }
    }
    pub fn id(&self) -> usize {
        self.id
    }
    pub const fn max_size(&self) -> usize {
        M
    }
    pub fn current_size(&self) -> usize {
        self.current_size
    }
    pub fn is_empty(&self) -> bool {
        self.is_empty
    }
    pub fn data(&self) -> &[u8] {
        &self.data[..self.current_size]
    }
    pub fn waiting_readers(&self) -> usize {
        self.wait_queue.len()
    }

    /// Overwrite content and wake all blocked readers.
    pub fn display(&mut self, data: &[u8]) -> Result<heapless::Vec<u8, W>, BlackboardError> {
        if data.len() > M {
            return Err(BlackboardError::MessageTooLarge);
        }
        self.data[..data.len()].copy_from_slice(data);
        self.current_size = data.len();
        self.is_empty = false;
        let mut woken = heapless::Vec::new();
        while let Some(pid) = self.wait_queue.pop_front() {
            // Safety: woken has capacity W, same as the wait queue,
            // so this cannot fail.
            woken.push(pid).unwrap();
        }
        Ok(woken)
    }

    /// Read current content.
    ///
    /// - `timeout == 0` (non-blocking): returns `Err(BoardEmpty)` immediately
    ///   if the board is empty, without enqueuing the caller.
    /// - `timeout > 0` (blocking): if empty, enqueues the caller and returns
    ///   `Ok(ReaderBlocked)`.
    pub fn read(
        &mut self,
        caller: u8,
        buf: &mut [u8],
        timeout: u32,
    ) -> Result<ReadBlackboardOutcome, BlackboardError> {
        if self.is_empty {
            if timeout == 0 {
                return Err(BlackboardError::BoardEmpty);
            }
            self.wait_queue
                .push_back(caller)
                .map_err(|_| BlackboardError::WaitQueueFull)?;
            return Ok(ReadBlackboardOutcome::ReaderBlocked);
        }
        let n = self.current_size.min(buf.len());
        buf[..n].copy_from_slice(&self.data[..n]);
        Ok(ReadBlackboardOutcome::Read {
            msg_len: self.current_size,
        })
    }

    pub fn clear(&mut self) {
        self.current_size = 0;
        self.is_empty = true;
    }
}

/// Fixed-capacity pool. `S` = max boards, `M` = message size, `W` = wait-queue cap.
pub struct BlackboardPool<const S: usize, const M: usize, const W: usize> {
    boards: heapless::Vec<Blackboard<M, W>, S>,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const M: usize, const W: usize> BlackboardPool<S, M, W> {
    pub const fn new() -> Self {
        Self {
            boards: heapless::Vec::new(),
        }
    }

    pub fn create(&mut self) -> Result<usize, BlackboardError> {
        let id = self.boards.len();
        self.boards
            .push(Blackboard::new(id))
            .map_err(|_| BlackboardError::PoolFull)?;
        Ok(id)
    }
    pub fn get(&self, id: usize) -> Option<&Blackboard<M, W>> {
        self.boards.get(id)
    }
    pub fn get_mut(&mut self, id: usize) -> Option<&mut Blackboard<M, W>> {
        self.boards.get_mut(id)
    }
    pub fn len(&self) -> usize {
        self.boards.len()
    }
    pub fn is_empty(&self) -> bool {
        self.boards.is_empty()
    }

    pub fn display_blackboard(
        &mut self,
        id: usize,
        data: &[u8],
    ) -> Result<heapless::Vec<u8, W>, BlackboardError> {
        self.boards
            .get_mut(id)
            .ok_or(BlackboardError::InvalidBoard)?
            .display(data)
    }

    pub fn read_blackboard(
        &mut self,
        id: usize,
        caller: u8,
        buf: &mut [u8],
        timeout: u32,
    ) -> Result<ReadBlackboardOutcome, BlackboardError> {
        self.boards
            .get_mut(id)
            .ok_or(BlackboardError::InvalidBoard)?
            .read(caller, buf, timeout)
    }

    pub fn clear_blackboard(&mut self, id: usize) -> Result<(), BlackboardError> {
        self.boards
            .get_mut(id)
            .ok_or(BlackboardError::InvalidBoard)?
            .clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_and_initial_state() {
        let bb = Blackboard::<64, 4>::new(0);
        assert_eq!((bb.id(), bb.max_size(), bb.current_size()), (0, 64, 0));
        assert!(bb.is_empty());
        assert_eq!((bb.data(), bb.waiting_readers()), (&[][..], 0));
    }

    #[test]
    fn display_writes_data() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let woken: heapless::Vec<u8, 4> = bb.display(&[1, 2, 3]).unwrap();
        assert!(!bb.is_empty());
        assert_eq!(bb.data(), &[1, 2, 3]);
        assert!(woken.is_empty());
    }

    #[test]
    fn display_overwrites_previous() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let _: heapless::Vec<u8, 4> = bb.display(&[1, 2, 3]).unwrap();
        let _: heapless::Vec<u8, 4> = bb.display(&[4, 5]).unwrap();
        assert_eq!(bb.data(), &[4, 5]);
    }

    #[test]
    fn display_wakes_all_waiters() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let mut buf = [0u8; 16];
        // Blocking reads (timeout > 0) enqueue callers
        for i in 1..=3u8 {
            assert_eq!(
                bb.read(i, &mut buf, 1),
                Ok(ReadBlackboardOutcome::ReaderBlocked)
            );
        }
        assert_eq!(bb.waiting_readers(), 3);
        let woken: heapless::Vec<u8, 4> = bb.display(&[10]).unwrap();
        assert_eq!(woken.as_slice(), &[1, 2, 3]);
        assert_eq!(bb.waiting_readers(), 0);
    }

    #[test]
    fn display_rejects_oversized() {
        let mut bb = Blackboard::<4, 2>::new(0);
        let r: Result<heapless::Vec<u8, 2>, _> = bb.display(&[1; 5]);
        assert_eq!(r, Err(BlackboardError::MessageTooLarge));
    }

    #[test]
    fn read_nonblocking_empty_returns_error_without_enqueue() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let mut buf = [0u8; 16];
        // Non-blocking read (timeout=0) on empty board returns error
        assert_eq!(bb.read(5, &mut buf, 0), Err(BlackboardError::BoardEmpty));
        // Caller was NOT enqueued
        assert_eq!(bb.waiting_readers(), 0);
    }

    #[test]
    fn read_blocking_empty_enqueues_caller() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let mut buf = [0u8; 16];
        // Blocking read (timeout>0) on empty board enqueues caller
        assert_eq!(
            bb.read(5, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(bb.waiting_readers(), 1);
    }

    #[test]
    fn read_returns_data_and_truncation() {
        let mut bb = Blackboard::<16, 4>::new(0);
        let _: heapless::Vec<u8, 4> = bb.display(&[1, 2, 3, 4, 5]).unwrap();
        let mut buf = [0u8; 16];
        assert_eq!(
            bb.read(1, &mut buf, 0),
            Ok(ReadBlackboardOutcome::Read { msg_len: 5 })
        );
        assert_eq!(&buf[..5], &[1, 2, 3, 4, 5]);
        // Truncation: small buffer gets prefix, returns original length
        let mut small = [0u8; 3];
        assert_eq!(
            bb.read(1, &mut small, 0),
            Ok(ReadBlackboardOutcome::Read { msg_len: 5 })
        );
        assert_eq!(small, [1, 2, 3]);
        // Clear resets to empty
        bb.clear();
        assert!(bb.is_empty());
    }

    #[test]
    fn read_blocking_wait_queue_full() {
        // W=2: wait queue can hold 2 readers
        let mut bb = Blackboard::<16, 2>::new(0);
        let mut buf = [0u8; 16];
        // Fill the wait queue with blocking reads
        assert_eq!(
            bb.read(1, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(
            bb.read(2, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(bb.waiting_readers(), 2);
        // Third blocking reader should get WaitQueueFull
        assert_eq!(bb.read(3, &mut buf, 1), Err(BlackboardError::WaitQueueFull));
        // Wait queue unchanged — the third reader was not enqueued
        assert_eq!(bb.waiting_readers(), 2);
    }

    #[test]
    fn pool_create_capacity_and_lookup() {
        let mut pool = BlackboardPool::<2, 8, 2>::new();
        assert!(pool.is_empty());
        let a = pool.create().unwrap();
        let b = pool.create().unwrap();
        assert_eq!((pool.len(), a, b), (2, 0, 1));
        assert_eq!(pool.create(), Err(BlackboardError::PoolFull));
        assert!(pool.get(a).unwrap().is_empty());
        assert!(pool.get_mut(b).is_some());
    }

    #[test]
    fn pool_display_and_read() {
        let mut pool = BlackboardPool::<4, 16, 4>::new();
        let id = pool.create().unwrap();
        let mut buf = [0u8; 16];
        // Block readers (timeout>0) then display
        assert_eq!(
            pool.read_blackboard(id, 1, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(
            pool.read_blackboard(id, 2, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        let woken: heapless::Vec<u8, 4> = pool.display_blackboard(id, &[42]).unwrap();
        assert_eq!(woken.as_slice(), &[1, 2]);
        let outcome = pool.read_blackboard(id, 0, &mut buf, 0).unwrap();
        assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 1 });
        assert_eq!(&buf[..1], &[42]);
        // Overwrite
        let _: heapless::Vec<u8, 4> = pool.display_blackboard(id, &[7, 8]).unwrap();
        let outcome = pool.read_blackboard(id, 0, &mut buf, 0).unwrap();
        assert_eq!(outcome, ReadBlackboardOutcome::Read { msg_len: 2 });
        assert_eq!(&buf[..2], &[7, 8]);
    }

    #[test]
    fn pool_invalid_ids() {
        let mut pool = BlackboardPool::<4, 8, 2>::new();
        let mut buf = [0u8; 8];
        let r: Result<heapless::Vec<u8, 2>, _> = pool.display_blackboard(99, &[1]);
        assert_eq!(r, Err(BlackboardError::InvalidBoard));
        assert_eq!(
            pool.read_blackboard(99, 0, &mut buf, 0),
            Err(BlackboardError::InvalidBoard)
        );
        assert_eq!(
            pool.clear_blackboard(99),
            Err(BlackboardError::InvalidBoard)
        );
    }

    #[test]
    fn pool_clear_then_nonblocking_read_returns_empty() {
        let mut pool = BlackboardPool::<4, 16, 4>::new();
        let id = pool.create().unwrap();
        let _: heapless::Vec<u8, 4> = pool.display_blackboard(id, &[1]).unwrap();
        pool.clear_blackboard(id).unwrap();
        let mut buf = [0u8; 16];
        assert_eq!(
            pool.read_blackboard(id, 0, &mut buf, 0),
            Err(BlackboardError::BoardEmpty)
        );
        // Caller was not enqueued
        assert_eq!(pool.get(id).unwrap().waiting_readers(), 0);
    }

    #[test]
    fn pool_clear_then_blocking_read_enqueues() {
        let mut pool = BlackboardPool::<4, 16, 4>::new();
        let id = pool.create().unwrap();
        let _: heapless::Vec<u8, 4> = pool.display_blackboard(id, &[1]).unwrap();
        pool.clear_blackboard(id).unwrap();
        let mut buf = [0u8; 16];
        assert_eq!(
            pool.read_blackboard(id, 0, &mut buf, 1),
            Ok(ReadBlackboardOutcome::ReaderBlocked)
        );
        assert_eq!(pool.get(id).unwrap().waiting_readers(), 1);
    }
}
