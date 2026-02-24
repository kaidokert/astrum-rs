use crate::partition::{PartitionState, PartitionTable, TransitionError};
use crate::waitqueue::TimedWaitQueue;
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
    wait_queue: TimedWaitQueue<W>,
}
impl<const W: usize> Semaphore<W> {
    pub const fn new(initial: u32, max_count: u32) -> Self {
        Self {
            count: initial,
            max_count,
            wait_queue: TimedWaitQueue::new(),
        }
    }
    pub fn count(&self) -> u32 {
        self.count
    }
    pub fn max_count(&self) -> u32 {
        self.max_count
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
    ) -> Result<bool, SemaphoreError> {
        let sem = self
            .slots
            .get_mut(sem_id)
            .ok_or(SemaphoreError::InvalidSemaphore)?;
        if sem.count > 0 {
            sem.count -= 1;
            return Ok(true);
        }
        if sem.wait_queue.len() >= W {
            return Err(SemaphoreError::WaitQueueFull);
        }
        let pcb = parts
            .get_mut(caller)
            .ok_or(SemaphoreError::InvalidPartition)?;
        pcb.transition(PartitionState::Waiting)?;
        // Cannot fail: len() < W was checked above and we hold &mut self.
        sem.wait_queue
            .push(caller as u8, u64::MAX)
            .expect("wait_queue push after len check");
        Ok(false)
    }
    pub fn wait_timed<const N: usize>(
        &mut self,
        parts: &mut PartitionTable<N>,
        sem_id: usize,
        caller: usize,
        timeout: u32,
        current_tick: u64,
    ) -> Result<bool, SemaphoreError> {
        let sem = self
            .slots
            .get_mut(sem_id)
            .ok_or(SemaphoreError::InvalidSemaphore)?;
        if sem.count > 0 {
            sem.count -= 1;
            return Ok(true);
        }
        if sem.wait_queue.len() >= W {
            return Err(SemaphoreError::WaitQueueFull);
        }
        let pcb = parts
            .get_mut(caller)
            .ok_or(SemaphoreError::InvalidPartition)?;
        pcb.transition(PartitionState::Waiting)?;
        let expiry = current_tick
            .saturating_add(timeout as u64)
            .min(u64::MAX - 1);
        // Cannot fail: len() < W was checked above and we hold &mut self.
        sem.wait_queue
            .push(caller as u8, expiry)
            .expect("wait_queue push after len check");
        Ok(false)
    }
    // TODO: reviewer false positive — queuing port timeout logic already
    // existed in expire_timed_waits (queuing.tick_timeouts) before this diff;
    // this commit adds only the semaphore tick_timeouts hook.
    pub fn tick_timeouts<const E: usize>(
        &mut self,
        current_tick: u64,
        out: &mut heapless::Vec<u8, E>,
    ) {
        for sem in self.slots.iter_mut() {
            sem.wait_queue.drain_expired(current_tick, out);
        }
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
        if let Some(pid) = sem.wait_queue.pop_front_pid() {
            let pcb = parts
                .get_mut(pid as usize)
                .ok_or(SemaphoreError::InvalidPartition)?;
            pcb.transition(PartitionState::Ready)?;
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
        assert_eq!(p.wait(&mut t, 0, 0), Ok(true)); assert_eq!(p.get(0).unwrap().count(), 1);
        assert_eq!(p.wait(&mut t, 0, 0), Ok(true)); assert_eq!(p.get(0).unwrap().count(), 0);
        p.add(Semaphore::new(0, 1)).unwrap();
        assert_eq!(p.wait(&mut t, 1, 1), Ok(false)); assert_eq!(t.get(1).unwrap().state(), Waiting);
        p.signal(&mut t, 1).unwrap(); assert_eq!(t.get(1).unwrap().state(), Ready);
        p.signal(&mut t, 0).unwrap(); assert_eq!(p.get(0).unwrap().count(), 1);
        p.add(Semaphore::new(3, 3)).unwrap();
        assert_eq!(p.signal(&mut t, 2), Err(SemaphoreError::CountOverflow));
        assert_eq!(p.wait(&mut t, 99, 0), Err(SemaphoreError::InvalidSemaphore));
    }
    #[test] fn fifo_and_queue_full() {
        let (mut t, mut p) = (tbl::<8>(8), SemaphorePool::<1, 4>::new());
        p.add(Semaphore::new(0, 1)).unwrap();
        for i in 0..3usize { assert_eq!(p.wait(&mut t, 0, i), Ok(false)); }
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(0).unwrap().state(), Ready);
        assert_eq!(t.get(1).unwrap().state(), Waiting);
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(1).unwrap().state(), Ready);
        p.signal(&mut t, 0).unwrap(); assert_eq!(t.get(2).unwrap().state(), Ready);
        for i in 3..7u8 { assert_eq!(p.wait(&mut t, 0, i as usize), Ok(false)); }
        assert_eq!(p.wait(&mut t, 0, 7), Err(SemaphoreError::WaitQueueFull));
    }
    #[test]
    fn sem_wait_returns_false_when_blocking() {
        let mut t = tbl::<4>(1);
        let mut p = SemaphorePool::<4, 4>::new();
        p.add(Semaphore::new(0, 1)).unwrap();
        let result = p.wait(&mut t, 0, 0);
        assert_eq!(result, Ok(false), "wait() on zero-count semaphore must return Ok(false)");
        assert_eq!(t.get(0).unwrap().state(), Waiting, "blocked partition must be in Waiting state");
    }
}
