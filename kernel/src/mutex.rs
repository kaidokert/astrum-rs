use crate::partition::{PartitionState, PartitionTable};

#[derive(Debug, PartialEq, Eq)]
pub enum MutexError {
    InvalidMutex,
    InvalidPartition,
    WaitQueueFull,
    NotOwner,
    AlreadyOwned,
}

pub struct MutexPool<const S: usize, const W: usize> {
    owners: [Option<u8>; S],
    queues: [heapless::Deque<u8, W>; S],
    len: usize,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const W: usize> MutexPool<S, W> {
    pub fn new(count: usize) -> Self {
        Self {
            owners: [None; S],
            queues: core::array::from_fn(|_| heapless::Deque::new()),
            len: count.min(S),
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
    ) -> Result<(), MutexError> {
        self.slot(id)?;
        if self.owners[id] == Some(caller as u8) {
            return Err(MutexError::AlreadyOwned);
        }
        if self.owners[id].is_none() {
            self.owners[id] = Some(caller as u8);
            return Ok(());
        }
        self.queues[id]
            .push_back(caller as u8)
            .map_err(|_| MutexError::WaitQueueFull)?;
        if parts
            .get_mut(caller)
            .ok_or(MutexError::InvalidPartition)?
            .transition(PartitionState::Waiting)
            .is_err()
        {
            self.queues[id].pop_back();
            return Err(MutexError::InvalidPartition);
        }
        Ok(())
    }
    pub fn unlock<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        id: usize,
        caller: usize,
    ) -> Result<(), MutexError> {
        self.slot(id)?;
        if self.owners[id] != Some(caller as u8) {
            return Err(MutexError::NotOwner);
        }
        if let Some(pid) = self.queues[id].pop_front() {
            if parts
                .get_mut(pid as usize)
                .ok_or(MutexError::InvalidPartition)?
                .transition(PartitionState::Ready)
                .is_err()
            {
                self.queues[id].push_front(pid).unwrap();
                return Err(MutexError::InvalidPartition);
            }
            self.owners[id] = Some(pid);
            return Ok(());
        }
        self.owners[id] = None;
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
        p.lock(&mut t, 0, 0).unwrap();
        assert_eq!(p.owner(0), Ok(Some(0)));
        assert_eq!(p.lock(&mut t, 0, 0), Err(MutexError::AlreadyOwned));
        assert_eq!(p.unlock(&mut t, 0, 1), Err(MutexError::NotOwner));
        p.unlock(&mut t, 0, 0).unwrap();
        assert_eq!(p.owner(0), Ok(None));
    }
    #[test] fn wait_queue_transfer() {
        let (mut t, mut p) = (tbl::<4>(3), MutexPool::<1, 4>::new(1));
        p.lock(&mut t, 0, 0).unwrap();
        p.lock(&mut t, 0, 1).unwrap(); assert_eq!(t.get(1).unwrap().state(), Waiting);
        p.lock(&mut t, 0, 2).unwrap(); assert_eq!(t.get(2).unwrap().state(), Waiting);
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
        p.lock(&mut t, 0, 0).unwrap();
        for i in 1..5usize { p.lock(&mut t, 0, i).unwrap(); }
        assert_eq!(p.lock(&mut t, 0, 5), Err(MutexError::WaitQueueFull));
    }
}
