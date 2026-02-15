//! Synchronization primitives pools.

use crate::config::SyncOps;
use crate::mutex::MutexPool;
use crate::semaphore::SemaphorePool;

/// Groups synchronization primitive pools (semaphores and mutexes).
pub struct SyncPools<const S: usize, const SW: usize, const MS: usize, const MW: usize>
where
    [(); S]:,
    [(); SW]:,
    [(); MS]:,
    [(); MW]:,
{
    semaphores: SemaphorePool<S, SW>,
    mutexes: MutexPool<MS, MW>,
}

impl<const S: usize, const SW: usize, const MS: usize, const MW: usize> SyncPools<S, SW, MS, MW>
where
    [(); S]:,
    [(); SW]:,
    [(); MS]:,
    [(); MW]:,
{
    /// Creates a new SyncPools with empty semaphore pool and full-capacity mutex pool.
    pub const fn new() -> Self {
        Self {
            semaphores: SemaphorePool::new(),
            mutexes: MutexPool::new(MS),
        }
    }

    pub fn semaphores(&self) -> &SemaphorePool<S, SW> {
        &self.semaphores
    }

    pub fn semaphores_mut(&mut self) -> &mut SemaphorePool<S, SW> {
        &mut self.semaphores
    }

    pub fn mutexes(&self) -> &MutexPool<MS, MW> {
        &self.mutexes
    }

    pub fn mutexes_mut(&mut self) -> &mut MutexPool<MS, MW> {
        &mut self.mutexes
    }
}

impl<const S: usize, const SW: usize, const MS: usize, const MW: usize> Default
    for SyncPools<S, SW, MS, MW>
where
    [(); S]:,
    [(); SW]:,
    [(); MS]:,
    [(); MW]:,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const S: usize, const SW: usize, const MS: usize, const MW: usize> SyncOps
    for SyncPools<S, SW, MS, MW>
where
    [(); S]:,
    [(); SW]:,
    [(); MS]:,
    [(); MW]:,
{
    type SemPool = SemaphorePool<S, SW>;
    type MutPool = MutexPool<MS, MW>;
    fn semaphores(&self) -> &Self::SemPool {
        &self.semaphores
    }
    fn semaphores_mut(&mut self) -> &mut Self::SemPool {
        &mut self.semaphores
    }
    fn mutexes(&self) -> &Self::MutPool {
        &self.mutexes
    }
    fn mutexes_mut(&mut self) -> &mut Self::MutPool {
        &mut self.mutexes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construction_and_field_access() {
        let mut pools: SyncPools<4, 4, 2, 2> = SyncPools::new();

        // Verify new() creates empty semaphore pool and full-capacity mutex pool
        assert!(pools.semaphores().get(0).is_none());
        assert!(pools.mutexes().owner(0).is_ok());

        // Verify default() matches new()
        let dflt: SyncPools<4, 4, 2, 2> = SyncPools::default();
        assert!(dflt.semaphores().get(0).is_none());

        // Test mutable semaphore accessor
        use crate::semaphore::Semaphore;
        assert!(pools.semaphores_mut().add(Semaphore::new(1, 5)).is_ok());
        assert!(pools.semaphores().get(0).is_some());
        assert_eq!(pools.semaphores().get(0).unwrap().count(), 1);

        // Test mutable mutex accessor - new() now initializes with full capacity
        assert!(pools.mutexes().owner(0).is_ok());
        assert_eq!(pools.mutexes().owner(0).unwrap(), None);

        // Verify mutexes_mut returns mutable reference to the pool
        let mutex_pool = pools.mutexes_mut();
        assert!(mutex_pool.owner(1).is_ok());
    }
}
