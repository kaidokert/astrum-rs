use crate::partition::{PartitionState, PartitionTable, TransitionError};
use crate::waitqueue::WaitQueue;

#[derive(Debug, PartialEq, Eq)]
pub enum MutexError {
    InvalidMutex,
    InvalidPartition,
    WaitQueueFull,
    NotOwner,
    AlreadyOwned,
    Transition(TransitionError),
}
impl From<TransitionError> for MutexError {
    fn from(e: TransitionError) -> Self {
        MutexError::Transition(e)
    }
}

pub struct MutexPool<const S: usize, const W: usize> {
    owners: [Option<u8>; S],
    queues: [WaitQueue<W>; S],
    len: usize,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const W: usize> MutexPool<S, W> {
    pub const fn new(count: usize) -> Self {
        Self {
            owners: [None; S],
            queues: [const { WaitQueue::new() }; S],
            len: if count < S { count } else { S },
        }
    }
    fn slot(&self, id: usize) -> Result<Option<u8>, MutexError> {
        if id < self.len {
            Ok(self.owners[id])
        } else {
            Err(MutexError::InvalidMutex)
        }
    }
    pub fn owner(&self, id: usize) -> Result<Option<u8>, MutexError> {
        self.slot(id)
    }
    pub fn lock<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        id: usize,
        caller: usize,
    ) -> Result<bool, MutexError> {
        self.slot(id)?;
        // Validate caller fits in u8 before any cast (partition IDs are stored as u8)
        let caller_u8 = u8::try_from(caller).map_err(|_| MutexError::InvalidPartition)?;
        // Use .get()/.get_mut() per Panic-Free Policy even though slot() validated bounds
        let owner = self.owners.get(id).ok_or(MutexError::InvalidMutex)?;
        if *owner == Some(caller_u8) {
            return Err(MutexError::AlreadyOwned);
        }
        if owner.is_none() {
            *self.owners.get_mut(id).ok_or(MutexError::InvalidMutex)? = Some(caller_u8);
            return Ok(true);
        }
        let queue = self.queues.get(id).ok_or(MutexError::InvalidMutex)?;
        if queue.is_full() {
            return Err(MutexError::WaitQueueFull);
        }
        parts
            .get_mut(caller)
            .ok_or(MutexError::InvalidPartition)?
            .transition(PartitionState::Waiting)?;
        // push cannot fail: we checked is_full above and hold &mut self.
        let _ = self
            .queues
            .get_mut(id)
            .ok_or(MutexError::InvalidMutex)?
            .push(caller_u8);
        Ok(false)
    }
    pub fn unlock<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        id: usize,
        caller: usize,
    ) -> Result<(), MutexError> {
        self.slot(id)?;
        // Validate caller fits in u8 before any cast (partition IDs are stored as u8)
        let caller_u8 = u8::try_from(caller).map_err(|_| MutexError::InvalidPartition)?;
        // Use .get()/.get_mut() per Panic-Free Policy even though slot() validated bounds
        let owner = self.owners.get(id).ok_or(MutexError::InvalidMutex)?;
        if *owner != Some(caller_u8) {
            return Err(MutexError::NotOwner);
        }
        if let Some(pid) = self
            .queues
            .get_mut(id)
            .ok_or(MutexError::InvalidMutex)?
            .pop_front()
        {
            parts
                .get_mut(pid as usize)
                .ok_or(MutexError::InvalidPartition)?
                .transition(PartitionState::Ready)?;
            *self.owners.get_mut(id).ok_or(MutexError::InvalidMutex)? = Some(pid);
            return Ok(());
        }
        *self.owners.get_mut(id).ok_or(MutexError::InvalidMutex)? = None;
        Ok(())
    }
}

#[rustfmt::skip]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock as PCB, PartitionState::*};
    const R: MpuRegion = MpuRegion::new(0, 4096, 0);
    fn tbl<const N: usize>(n: u8) -> PartitionTable<N> {
        let mut t = PartitionTable::new();
        for i in 0..n { t.add(PCB::new(i, 0x800_0000, 0x2000_0000, 0x2000_0400, R)).unwrap(); t.get_mut(i as usize).unwrap().transition(Running).unwrap(); }
        t
    }
    #[test] fn lock_unlock_ownership_errors_and_invalid() {
        let (mut t, mut p) = (tbl::<4>(2), MutexPool::<4, 4>::new(1));
        assert_eq!(p.lock(&mut t, 99, 0), Err(MutexError::InvalidMutex));
        assert_eq!(p.unlock(&mut t, 99, 0), Err(MutexError::InvalidMutex));
        assert_eq!(p.unlock(&mut t, 0, 0), Err(MutexError::NotOwner));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        assert_eq!(p.owner(0), Ok(Some(0)));
        assert_eq!(p.lock(&mut t, 0, 0), Err(MutexError::AlreadyOwned));
        assert_eq!(p.unlock(&mut t, 0, 1), Err(MutexError::NotOwner));
        p.unlock(&mut t, 0, 0).unwrap();
        assert_eq!(p.owner(0), Ok(None));
    }
    #[test] fn wait_queue_transfer() {
        let (mut t, mut p) = (tbl::<4>(3), MutexPool::<1, 4>::new(1));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        assert_eq!(p.lock(&mut t, 0, 1), Ok(false)); assert_eq!(t.get(1).unwrap().state(), Waiting);
        assert_eq!(p.lock(&mut t, 0, 2), Ok(false)); assert_eq!(t.get(2).unwrap().state(), Waiting);
        p.unlock(&mut t, 0, 0).unwrap();
        assert_eq!(p.owner(0), Ok(Some(1)));
        assert_eq!(t.get(1).unwrap().state(), Ready);
        p.unlock(&mut t, 0, 1).unwrap();
        assert_eq!(p.owner(0), Ok(Some(2)));
        p.unlock(&mut t, 0, 2).unwrap();
        assert_eq!(p.owner(0), Ok(None));
    }
    #[test] fn wait_queue_full() {
        let (mut t, mut p) = (tbl::<8>(6), MutexPool::<1, 4>::new(1));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        for i in 1..5usize { assert_eq!(p.lock(&mut t, 0, i), Ok(false)); }
        assert_eq!(p.lock(&mut t, 0, 5), Err(MutexError::WaitQueueFull));
    }
    #[test] fn mutex_lock_returns_false_when_blocking() {
        // Setup: P0 and P1 both in Running state
        let (mut t, mut p) = (tbl::<4>(2), MutexPool::<1, 4>::new(1));
        // P0 acquires mutex (should succeed with Ok(true))
        let p0_result = p.lock(&mut t, 0, 0);
        assert_eq!(p0_result, Ok(true));
        assert_eq!(p.owner(0), Ok(Some(0)));
        // P1 tries to acquire held mutex - must return Ok(false) exactly
        let p1_result = p.lock(&mut t, 0, 1);
        assert_eq!(p1_result, Ok(false), "lock() must return Ok(false) when blocking");
        // Verify P1's state transitioned to Waiting
        let p1_state = t.get(1).unwrap().state();
        assert_eq!(p1_state, Waiting, "blocked partition must be in Waiting state");
        // P0 still owns the mutex
        assert_eq!(p.owner(0), Ok(Some(0)));
    }
}
