use crate::partition::{PartitionState, PartitionTable, TransitionError};
#[derive(Debug, PartialEq, Eq)]
pub enum SemaphoreError {
    InvalidSemaphore,
    InvalidPartition,
    WaitQueueFull,
    CountOverflow,
    Transition(TransitionError),
}
impl From<TransitionError> for SemaphoreError {
    fn from(e: TransitionError) -> Self {
        SemaphoreError::Transition(e)
    }
}
#[derive(Debug)]
pub struct Semaphore<const W: usize> {
    count: u32,
    max_count: u32,
    wait_queue: heapless::Deque<u8, W>,
}
impl<const W: usize> Semaphore<W> {
    pub const fn new(initial: u32, max_count: u32) -> Self {
        Self {
            count: initial,
            max_count,
            wait_queue: heapless::Deque::new(),
        }
    }
    pub fn count(&self) -> u32 {
        self.count
    }
}
pub struct SemaphorePool<const S: usize, const W: usize> {
    slots: heapless::Vec<Semaphore<W>, S>,
}
#[allow(clippy::new_without_default)]
impl<const S: usize, const W: usize> SemaphorePool<S, W> {
    pub const fn new() -> Self {
        Self {
            slots: heapless::Vec::new(),
        }
    }
    pub fn add(&mut self, sem: Semaphore<W>) -> Result<(), Semaphore<W>> {
        self.slots.push(sem)
    }
    pub fn get(&self, id: usize) -> Option<&Semaphore<W>> {
        self.slots.get(id)
    }
    pub fn wait<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        sem_id: usize,
        caller: usize,
    ) -> Result<(), SemaphoreError> {
        let sem = self
            .slots
            .get_mut(sem_id)
            .ok_or(SemaphoreError::InvalidSemaphore)?;
        if sem.count > 0 {
            sem.count -= 1;
            return Ok(());
        }
        sem.wait_queue
            .push_back(caller as u8)
            .map_err(|_| SemaphoreError::WaitQueueFull)?;
        let pcb = match parts.get_mut(caller) {
            Some(p) => p,
            None => {
                sem.wait_queue.pop_back();
                return Err(SemaphoreError::InvalidPartition);
            }
        };
        if let Err(e) = pcb.transition(PartitionState::Waiting) {
            sem.wait_queue.pop_back();
            return Err(e.into());
        }
        Ok(())
    }
    pub fn signal<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        sem_id: usize,
    ) -> Result<(), SemaphoreError> {
        let sem = self
            .slots
            .get_mut(sem_id)
            .ok_or(SemaphoreError::InvalidSemaphore)?;
        if let Some(pid) = sem.wait_queue.pop_front() {
            let pcb = match parts.get_mut(pid as usize) {
                Some(p) => p,
                None => {
                    sem.wait_queue.push_front(pid).unwrap();
                    return Err(SemaphoreError::InvalidPartition);
                }
            };
            if let Err(e) = pcb.transition(PartitionState::Ready) {
                sem.wait_queue.push_front(pid).unwrap();
                return Err(e.into());
            }
            return Ok(());
        }
        if sem.count < sem.max_count {
            sem.count += 1;
            Ok(())
        } else {
            Err(SemaphoreError::CountOverflow)
        }
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
    #[test] fn wait_signal_overflow_invalid() {
        let (mut t, mut p) = (tbl::<4>(3), SemaphorePool::<4, 4>::new());
        p.add(Semaphore::new(2, 3)).unwrap();
        p.wait(&mut t, 0, 0).unwrap(); assert_eq!(p.get(0).unwrap().count(), 1);
        p.wait(&mut t, 0, 0).unwrap(); assert_eq!(p.get(0).unwrap().count(), 0);
        p.add(Semaphore::new(0, 1)).unwrap();
        p.wait(&mut t, 1, 1).unwrap(); assert_eq!(t.get(1).unwrap().state(), Waiting);
        p.signal(&mut t, 1).unwrap(); assert_eq!(t.get(1).unwrap().state(), Ready);
        p.signal(&mut t, 0).unwrap(); assert_eq!(p.get(0).unwrap().count(), 1);
        p.add(Semaphore::new(3, 3)).unwrap();
        assert_eq!(p.signal(&mut t, 2), Err(SemaphoreError::CountOverflow));
        assert_eq!(p.wait(&mut t, 99, 0), Err(SemaphoreError::InvalidSemaphore));
    }
    #[test] fn fifo_and_queue_full() {
        let (mut t, mut p) = (tbl::<8>(8), SemaphorePool::<1, 4>::new());
        p.add(Semaphore::new(0, 1)).unwrap();
        for i in 0..3usize { p.wait(&mut t, 0, i).unwrap(); }
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(0).unwrap().state(), Ready);
        assert_eq!(t.get(1).unwrap().state(), Waiting);
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(1).unwrap().state(), Ready);
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(2).unwrap().state(), Ready);
        for i in 3..7u8 { p.wait(&mut t, 0, i as usize).unwrap(); }
        assert_eq!(p.wait(&mut t, 0, 7), Err(SemaphoreError::WaitQueueFull));
    }
}
