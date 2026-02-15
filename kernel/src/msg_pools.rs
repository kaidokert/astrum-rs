//! Message primitive pools.

use crate::config::MsgOps;
use crate::message::MessagePool;
use crate::queuing::QueuingPortPool;

/// Groups message-passing pools (message queues and queuing ports).
pub struct MsgPools<const QS: usize, const QD: usize, const QM: usize, const QW: usize>
where
    [(); QS]:,
    [(); QD]:,
    [(); QM]:,
    [(); QW]:,
{
    messages: MessagePool<QS, QD, QM, QW>,
    queuing: QueuingPortPool<QS, QD, QM, QW>,
}

impl<const QS: usize, const QD: usize, const QM: usize, const QW: usize> MsgPools<QS, QD, QM, QW>
where
    [(); QS]:,
    [(); QD]:,
    [(); QM]:,
    [(); QW]:,
{
    /// Creates a new MsgPools with empty message pool and queuing port pool.
    pub const fn new() -> Self {
        Self {
            messages: MessagePool::new(),
            queuing: QueuingPortPool::new(),
        }
    }

    pub fn messages(&self) -> &MessagePool<QS, QD, QM, QW> {
        &self.messages
    }

    pub fn messages_mut(&mut self) -> &mut MessagePool<QS, QD, QM, QW> {
        &mut self.messages
    }

    pub fn queuing(&self) -> &QueuingPortPool<QS, QD, QM, QW> {
        &self.queuing
    }

    pub fn queuing_mut(&mut self) -> &mut QueuingPortPool<QS, QD, QM, QW> {
        &mut self.queuing
    }
}

impl<const QS: usize, const QD: usize, const QM: usize, const QW: usize> Default
    for MsgPools<QS, QD, QM, QW>
where
    [(); QS]:,
    [(); QD]:,
    [(); QM]:,
    [(); QW]:,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const QS: usize, const QD: usize, const QM: usize, const QW: usize> MsgOps
    for MsgPools<QS, QD, QM, QW>
where
    [(); QS]:,
    [(); QD]:,
    [(); QM]:,
    [(); QW]:,
{
    type MsgPool = MessagePool<QS, QD, QM, QW>;
    fn messages(&self) -> &Self::MsgPool {
        &self.messages
    }
    fn messages_mut(&mut self) -> &mut Self::MsgPool {
        &mut self.messages
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::MessageQueue;
    use crate::sampling::PortDirection;

    #[test]
    fn construction_and_field_access() {
        let mut pools: MsgPools<4, 4, 16, 4> = MsgPools::new();

        // Verify new() creates empty pools
        assert!(pools.messages().get(0).is_none());
        assert!(pools.queuing().is_empty());

        // Verify default() matches new()
        let dflt: MsgPools<4, 4, 16, 4> = MsgPools::default();
        assert!(dflt.messages().get(0).is_none());
        assert!(dflt.queuing().is_empty());

        // Test mutable message pool accessor
        assert!(pools.messages_mut().add(MessageQueue::new()).is_ok());
        assert!(pools.messages().get(0).is_some());

        // Test mutable queuing pool accessor
        assert!(pools
            .queuing_mut()
            .create_port(PortDirection::Source)
            .is_ok());
        assert!(!pools.queuing().is_empty());
        assert_eq!(pools.queuing().len(), 1);
    }
}
