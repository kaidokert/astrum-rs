use crate::partition::{PartitionState, PartitionTable, TransitionError};
use crate::waitqueue::WaitQueue;
use rtos_traits::ids::PartitionId;

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
    pub fn owner(&self, id: usize) -> Result<Option<PartitionId>, MutexError> {
        self.slot(id)
            .map(|opt| opt.map(|v| PartitionId::new(v as u32)))
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
            .push(PartitionId::new(caller_u8 as u32));
        Ok(false)
    }
    /// Release all mutexes owned by `pid` and remove `pid` from all wait queues.
    ///
    /// Used during partition restart to clean up IPC state. When `pid` owns a
    /// mutex and there are waiters, the next waiter is promoted to owner and
    /// transitioned to `Ready`. Removes `pid` from every mutex wait queue.
    pub fn release_mutexes_for_partition<const N: usize>(
        &mut self,
        pid: PartitionId,
        parts: &mut PartitionTable<N>,
    ) {
        let pid_u8 = pid.as_raw() as u8;
        for i in 0..self.len {
            if let Some(owner) = self.owners.get_mut(i) {
                if *owner == Some(pid_u8) {
                    // Hand off to next waiter, mirroring unlock() logic.
                    if let Some(queue) = self.queues.get_mut(i) {
                        if let Some(next) = queue.pop_front() {
                            *owner = Some(next.as_raw() as u8);
                            if let Some(pcb) = parts.get_mut(next.as_raw() as usize) {
                                let _ = pcb.transition(PartitionState::Ready);
                            }
                        } else {
                            *owner = None;
                        }
                    } else {
                        *owner = None;
                    }
                }
            }
            if let Some(queue) = self.queues.get_mut(i) {
                queue.remove_by_id(pid);
            }
        }
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
                .get_mut(pid.as_raw() as usize)
                .ok_or(MutexError::InvalidPartition)?
                .transition(PartitionState::Ready)?;
            *self.owners.get_mut(id).ok_or(MutexError::InvalidMutex)? = Some(pid.as_raw() as u8);
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
    fn pid(v: u32) -> PartitionId { PartitionId::new(v) }
    #[test] fn lock_unlock_ownership_errors_and_invalid() {
        let (mut t, mut p) = (tbl::<4>(2), MutexPool::<4, 4>::new(1));
        assert_eq!(p.lock(&mut t, 99, 0), Err(MutexError::InvalidMutex));
        assert_eq!(p.unlock(&mut t, 99, 0), Err(MutexError::InvalidMutex));
        assert_eq!(p.unlock(&mut t, 0, 0), Err(MutexError::NotOwner));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        assert_eq!(p.owner(0), Ok(Some(pid(0))));
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
        assert_eq!(p.owner(0), Ok(Some(pid(1))));
        assert_eq!(t.get(1).unwrap().state(), Ready);
        p.unlock(&mut t, 0, 1).unwrap();
        assert_eq!(p.owner(0), Ok(Some(pid(2))));
        p.unlock(&mut t, 0, 2).unwrap();
        assert_eq!(p.owner(0), Ok(None));
    }
    #[test] fn wait_queue_full() {
        let (mut t, mut p) = (tbl::<8>(6), MutexPool::<1, 4>::new(1));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        for i in 1..5usize { assert_eq!(p.lock(&mut t, 0, i), Ok(false)); }
        assert_eq!(p.lock(&mut t, 0, 5), Err(MutexError::WaitQueueFull));
    }
    #[test] fn release_mutexes_for_partition_clears_owner_and_drains_waitqueue() {
        let (mut t, mut p) = (tbl::<4>(3), MutexPool::<2, 4>::new(2));
        // P0 owns mutex 0; P1 and P2 are waiting on mutex 0
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        assert_eq!(p.lock(&mut t, 0, 1), Ok(false));
        assert_eq!(p.lock(&mut t, 0, 2), Ok(false));
        // P1 owns mutex 1
        assert_eq!(p.lock(&mut t, 1, 1), Ok(true));
        // Release all IPC for P1: clears mutex 1 ownership, removes P1 from mutex 0 waitqueue
        p.release_mutexes_for_partition(pid(1), &mut t);
        assert_eq!(p.owner(0), Ok(Some(pid(0))), "P0 still owns mutex 0");
        assert_eq!(p.owner(1), Ok(None), "P1 ownership of mutex 1 cleared");
        // Unlock mutex 0: P1 was removed from queue, so P2 is next
        p.unlock(&mut t, 0, 0).unwrap();
        assert_eq!(p.owner(0), Ok(Some(pid(2))), "P2 promoted after P1 removed from queue");
    }
    #[test] fn release_mutexes_for_partition_owner_removed_promotes_waiter() {
        let (mut t, mut p) = (tbl::<4>(3), MutexPool::<1, 4>::new(1));
        // P0 owns mutex 0; P1 and P2 are waiting
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        assert_eq!(p.lock(&mut t, 0, 1), Ok(false));
        assert_eq!(p.lock(&mut t, 0, 2), Ok(false));
        assert_eq!(t.get(1).unwrap().state(), Waiting);
        assert_eq!(t.get(2).unwrap().state(), Waiting);
        // Clean up P0 (the owner): next waiter P1 must be promoted
        p.release_mutexes_for_partition(pid(0), &mut t);
        assert_eq!(p.owner(0), Ok(Some(pid(1))), "P1 promoted to owner after P0 cleanup");
        assert_eq!(t.get(1).unwrap().state(), Ready, "P1 transitioned to Ready");
        assert_eq!(t.get(2).unwrap().state(), Waiting, "P2 still waiting");
    }
    #[test] fn release_mutexes_noop_for_uninvolved_partition() {
        let (mut t, mut p) = (tbl::<4>(2), MutexPool::<2, 4>::new(1));
        assert_eq!(p.lock(&mut t, 0, 0), Ok(true));
        p.release_mutexes_for_partition(pid(1), &mut t); // P1 has no involvement
        assert_eq!(p.owner(0), Ok(Some(pid(0))), "P0 ownership unchanged");
    }
    #[test] fn mutex_lock_returns_false_when_blocking() {
        // Setup: P0 and P1 both in Running state
        let (mut t, mut p) = (tbl::<4>(2), MutexPool::<1, 4>::new(1));
        // P0 acquires mutex (should succeed with Ok(true))
        let p0_result = p.lock(&mut t, 0, 0);
        assert_eq!(p0_result, Ok(true));
        assert_eq!(p.owner(0), Ok(Some(pid(0))));
        // P1 tries to acquire held mutex - must return Ok(false) exactly
        let p1_result = p.lock(&mut t, 0, 1);
        assert_eq!(p1_result, Ok(false), "lock() must return Ok(false) when blocking");
        // Verify P1's state transitioned to Waiting
        let p1_state = t.get(1).unwrap().state();
        assert_eq!(p1_state, Waiting, "blocked partition must be in Waiting state");
        // P0 still owns the mutex
        assert_eq!(p.owner(0), Ok(Some(pid(0))));
    }
}
