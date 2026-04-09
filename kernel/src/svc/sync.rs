use crate::mutex::{MutexError, MutexPool};
use crate::partition::PartitionTable;
use crate::semaphore::{SemaphoreError, SemaphorePool};
use crate::svc::SvcError;
use rtos_traits::ids::{MutexId, SemaphoreId};

pub fn handle_sem_wait<const N: usize, const S: usize, const W: usize>(
    sem: &mut SemaphorePool<S, W>,
    pt: &mut PartitionTable<N>,
    id: SemaphoreId,
    caller: usize,
) -> (u32, bool) {
    match sem.wait(pt, id.as_raw() as usize, caller) {
        Ok(true) => (1, false),
        Ok(false) => (0, true),
        Err(e) => (sem_error_to_svc(e), false),
    }
}

pub fn handle_sem_signal<const N: usize, const S: usize, const W: usize>(
    sem: &mut SemaphorePool<S, W>,
    pt: &mut PartitionTable<N>,
    id: SemaphoreId,
) -> u32 {
    sem.signal(pt, id.as_raw() as usize)
        .map_or_else(sem_error_to_svc, |()| 0)
}

pub fn handle_mtx_lock<const N: usize, const S: usize, const W: usize>(
    mtx: &mut MutexPool<S, W>,
    pt: &mut PartitionTable<N>,
    id: MutexId,
    caller: usize,
) -> (u32, bool) {
    match mtx.lock(pt, id.as_raw() as usize, caller) {
        Ok(true) => (1, false),
        Ok(false) => (0, true),
        Err(e) => (mtx_error_to_svc(e), false),
    }
}

pub fn handle_mtx_unlock<const N: usize, const S: usize, const W: usize>(
    mtx: &mut MutexPool<S, W>,
    pt: &mut PartitionTable<N>,
    id: MutexId,
    caller: usize,
) -> u32 {
    mtx.unlock(pt, id.as_raw() as usize, caller)
        .map_or_else(mtx_error_to_svc, |()| 0)
}

fn sem_error_to_svc(e: SemaphoreError) -> u32 {
    match e {
        SemaphoreError::InvalidSemaphore => SvcError::InvalidResource.to_u32(),
        SemaphoreError::InvalidPartition => SvcError::InvalidPartition.to_u32(),
        SemaphoreError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        SemaphoreError::CountOverflow => SvcError::OperationFailed.to_u32(),
        SemaphoreError::Transition(_) => SvcError::TransitionFailed.to_u32(),
    }
}

fn mtx_error_to_svc(e: MutexError) -> u32 {
    match e {
        MutexError::InvalidMutex => SvcError::InvalidResource.to_u32(),
        MutexError::InvalidPartition => SvcError::InvalidPartition.to_u32(),
        MutexError::WaitQueueFull => SvcError::WaitQueueFull.to_u32(),
        MutexError::AlreadyOwned => SvcError::OperationFailed.to_u32(),
        MutexError::NotOwner => SvcError::OperationFailed.to_u32(),
        MutexError::Transition(_) => SvcError::TransitionFailed.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState};
    use crate::semaphore::Semaphore;

    fn pt() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        for i in 0..2u8 {
            let base = 0x2000_0000 + (i as u32) * 0x1000;
            let pcb = PartitionControlBlock::new(
                i,
                0x800_0000,
                base,
                base + 0x400,
                MpuRegion::new(base, 4096, 0),
            );
            t.add(pcb).unwrap();
            t.get_mut(i as usize)
                .unwrap()
                .transition(PartitionState::Running)
                .unwrap();
        }
        t
    }
    #[test]
    fn sem() {
        let (mut p, mut s) = (pt(), SemaphorePool::<4, 4>::new());
        s.add(Semaphore::new(1, 2)).unwrap();
        assert_eq!(
            handle_sem_wait(&mut s, &mut p, SemaphoreId::new(0), 0),
            (1, false)
        );
        assert_eq!(
            handle_sem_wait(&mut s, &mut p, SemaphoreId::new(0), 1),
            (0, true)
        );
        assert_eq!(
            handle_sem_wait(&mut s, &mut p, SemaphoreId::new(99), 0).0,
            SvcError::InvalidResource.to_u32()
        );
        assert_eq!(
            handle_sem_signal(&mut s, &mut p, SemaphoreId::new(99)),
            SvcError::InvalidResource.to_u32()
        );
        let (mut q, mut t) = (pt(), SemaphorePool::<4, 4>::new());
        t.add(Semaphore::new(0, 2)).unwrap();
        assert_eq!(handle_sem_signal(&mut t, &mut q, SemaphoreId::new(0)), 0);
        assert_eq!(
            handle_sem_wait(&mut t, &mut q, SemaphoreId::new(0), 0),
            (1, false)
        );
        assert_eq!(
            sem_error_to_svc(SemaphoreError::InvalidPartition),
            SvcError::InvalidPartition.to_u32()
        );
        assert_eq!(
            sem_error_to_svc(SemaphoreError::WaitQueueFull),
            SvcError::WaitQueueFull.to_u32()
        );
        assert_eq!(
            sem_error_to_svc(SemaphoreError::CountOverflow),
            SvcError::OperationFailed.to_u32()
        );
        assert_eq!(
            sem_error_to_svc(SemaphoreError::Transition(
                crate::partition::TransitionError
            )),
            SvcError::TransitionFailed.to_u32()
        );
    }
    #[test]
    fn mtx() {
        let (mut p, mut m) = (pt(), MutexPool::<4, 4>::new(4));
        assert_eq!(
            handle_mtx_lock(&mut m, &mut p, MutexId::new(0), 0),
            (1, false)
        );
        assert_eq!(
            handle_mtx_lock(&mut m, &mut p, MutexId::new(0), 1),
            (0, true)
        );
        assert_eq!(
            handle_mtx_lock(&mut m, &mut p, MutexId::new(99), 0).0,
            SvcError::InvalidResource.to_u32()
        );
        assert_eq!(
            handle_mtx_lock(&mut m, &mut p, MutexId::new(0), 0).0,
            SvcError::OperationFailed.to_u32()
        );
        assert_eq!(
            handle_mtx_unlock(&mut m, &mut p, MutexId::new(0), 1),
            SvcError::OperationFailed.to_u32()
        );
        assert_eq!(handle_mtx_unlock(&mut m, &mut p, MutexId::new(0), 0), 0);
        assert_eq!(
            handle_mtx_unlock(&mut m, &mut p, MutexId::new(99), 0),
            SvcError::InvalidResource.to_u32()
        );
        assert_eq!(
            mtx_error_to_svc(MutexError::InvalidPartition),
            SvcError::InvalidPartition.to_u32()
        );
        assert_eq!(
            mtx_error_to_svc(MutexError::WaitQueueFull),
            SvcError::WaitQueueFull.to_u32()
        );
        assert_eq!(
            mtx_error_to_svc(MutexError::Transition(crate::partition::TransitionError)),
            SvcError::TransitionFailed.to_u32()
        );
    }
}
