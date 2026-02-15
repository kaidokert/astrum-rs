//! Port primitive pools (sampling ports and blackboards).

use crate::blackboard::BlackboardPool;
use crate::sampling::SamplingPortPool;

/// Groups port-based IPC pools (sampling ports and blackboards).
pub struct PortPools<
    const SP: usize,
    const SM: usize,
    const BS: usize,
    const BM: usize,
    const BW: usize,
> where
    [(); SP]:,
    [(); SM]:,
    [(); BS]:,
    [(); BM]:,
    [(); BW]:,
{
    sampling: SamplingPortPool<SP, SM>,
    blackboards: BlackboardPool<BS, BM, BW>,
}

impl<const SP: usize, const SM: usize, const BS: usize, const BM: usize, const BW: usize>
    PortPools<SP, SM, BS, BM, BW>
where
    [(); SP]:,
    [(); SM]:,
    [(); BS]:,
    [(); BM]:,
    [(); BW]:,
{
    /// Creates a new PortPools with empty sampling port pool and blackboard pool.
    pub const fn new() -> Self {
        Self {
            sampling: SamplingPortPool::new(),
            blackboards: BlackboardPool::new(),
        }
    }

    pub fn sampling(&self) -> &SamplingPortPool<SP, SM> {
        &self.sampling
    }

    pub fn sampling_mut(&mut self) -> &mut SamplingPortPool<SP, SM> {
        &mut self.sampling
    }

    pub fn blackboards(&self) -> &BlackboardPool<BS, BM, BW> {
        &self.blackboards
    }

    pub fn blackboards_mut(&mut self) -> &mut BlackboardPool<BS, BM, BW> {
        &mut self.blackboards
    }
}

impl<const SP: usize, const SM: usize, const BS: usize, const BM: usize, const BW: usize> Default
    for PortPools<SP, SM, BS, BM, BW>
where
    [(); SP]:,
    [(); SM]:,
    [(); BS]:,
    [(); BM]:,
    [(); BW]:,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sampling::PortDirection;

    #[test]
    fn construction_and_field_access() {
        let mut pools: PortPools<4, 16, 4, 32, 4> = PortPools::new();

        // Verify new() creates empty pools
        assert!(pools.sampling().is_empty());
        assert!(pools.blackboards().is_empty());

        // Verify default() matches new()
        let dflt: PortPools<4, 16, 4, 32, 4> = PortPools::default();
        assert!(dflt.sampling().is_empty());
        assert!(dflt.blackboards().is_empty());

        // Test mutable sampling pool accessor
        assert!(pools
            .sampling_mut()
            .create_port(PortDirection::Source, 100)
            .is_ok());
        assert!(!pools.sampling().is_empty());
        assert_eq!(pools.sampling().len(), 1);

        // Test mutable blackboard pool accessor
        assert!(pools.blackboards_mut().create().is_ok());
        assert!(!pools.blackboards().is_empty());
        assert_eq!(pools.blackboards().len(), 1);
    }
}
