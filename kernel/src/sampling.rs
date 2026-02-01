#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDirection {
    Source,
    Destination,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Validity {
    Valid,
    Invalid,
}

#[derive(Debug, PartialEq, Eq)]
pub enum SamplingError {
    PoolFull,
    DirectionViolation,
    MessageTooLarge,
}

#[derive(Debug)]
pub struct SamplingPort<const M: usize> {
    id: usize,
    direction: PortDirection,
    max_size: usize,
    refresh_period: u32,
    data: [u8; M],
    current_size: usize,
    timestamp: u64,
    connected_port: Option<usize>,
}

impl<const M: usize> SamplingPort<M> {
    pub const fn new(id: usize, direction: PortDirection, refresh_period: u32) -> Self {
        Self {
            id,
            direction,
            max_size: M,
            refresh_period,
            data: [0u8; M],
            current_size: 0,
            timestamp: 0,
            connected_port: None,
        }
    }

    pub fn id(&self) -> usize {
        self.id
    }

    pub fn direction(&self) -> PortDirection {
        self.direction
    }

    pub fn max_size(&self) -> usize {
        self.max_size
    }

    pub fn refresh_period(&self) -> u32 {
        self.refresh_period
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.current_size]
    }

    pub fn current_size(&self) -> usize {
        self.current_size
    }

    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    pub fn connected_port(&self) -> Option<usize> {
        self.connected_port
    }

    pub fn write_data(&mut self, buf: &[u8], timestamp: u64) -> Result<(), SamplingError> {
        if self.direction != PortDirection::Source {
            return Err(SamplingError::DirectionViolation);
        }
        if buf.len() > M {
            return Err(SamplingError::MessageTooLarge);
        }
        self.data[..buf.len()].copy_from_slice(buf);
        self.current_size = buf.len();
        self.timestamp = timestamp;
        Ok(())
    }

    pub fn read_data(&self) -> Result<(&[u8], u64), SamplingError> {
        if self.direction != PortDirection::Destination {
            return Err(SamplingError::DirectionViolation);
        }
        Ok((&self.data[..self.current_size], self.timestamp))
    }

    pub fn validity(&self, current_time: u64) -> Validity {
        if self.current_size == 0 {
            return Validity::Invalid;
        }
        if current_time.wrapping_sub(self.timestamp) > self.refresh_period as u64 {
            return Validity::Invalid;
        }
        Validity::Valid
    }
}

pub struct SamplingPortPool<const S: usize, const M: usize> {
    ports: heapless::Vec<SamplingPort<M>, S>,
    // TODO: ID allocation uses Vec index, which is fragile if port deletion is added.
    // Replace with a monotonic counter or generation-based ID scheme when delete is needed.
    next_id: usize,
}

#[allow(clippy::new_without_default)]
impl<const S: usize, const M: usize> SamplingPortPool<S, M> {
    pub const fn new() -> Self {
        Self {
            ports: heapless::Vec::new(),
            next_id: 0,
        }
    }

    pub fn create_port(
        &mut self,
        direction: PortDirection,
        refresh_period: u32,
    ) -> Result<usize, SamplingError> {
        let id = self.next_id;
        self.ports
            .push(SamplingPort::new(id, direction, refresh_period))
            .map_err(|_| SamplingError::PoolFull)?;
        self.next_id += 1;
        Ok(id)
    }

    pub fn get(&self, id: usize) -> Option<&SamplingPort<M>> {
        self.ports.iter().find(|p| p.id == id)
    }

    pub fn get_mut(&mut self, id: usize) -> Option<&mut SamplingPort<M>> {
        self.ports.iter_mut().find(|p| p.id == id)
    }

    pub fn len(&self) -> usize {
        self.ports.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ports.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_source_port() {
        let mut pool = SamplingPortPool::<4, 64>::new();
        assert!(pool.is_empty());

        let id = pool.create_port(PortDirection::Source, 100).unwrap();
        let p = pool.get(id).unwrap();

        assert_eq!(p.id(), 0);
        assert_eq!(p.direction(), PortDirection::Source);
        assert_eq!(p.max_size(), 64);
        assert_eq!(p.refresh_period(), 100);
        assert_eq!(p.current_size(), 0);
        assert_eq!(p.timestamp(), 0);
        assert_eq!(p.connected_port(), None);
        assert_eq!(p.data(), &[]);
    }

    #[test]
    fn create_destination_port() {
        let mut pool = SamplingPortPool::<4, 64>::new();
        let id = pool.create_port(PortDirection::Destination, 200).unwrap();
        let p = pool.get(id).unwrap();

        assert_eq!(p.id(), id);
        assert_eq!(p.direction(), PortDirection::Destination);
        assert_eq!(p.refresh_period(), 200);
    }

    #[test]
    fn multiple_ports_get_distinct_ids() {
        let mut pool = SamplingPortPool::<4, 64>::new();
        let id0 = pool.create_port(PortDirection::Source, 100).unwrap();
        let id1 = pool.create_port(PortDirection::Destination, 200).unwrap();

        assert_ne!(id0, id1);
        assert_eq!(pool.len(), 2);
    }

    #[test]
    fn pool_full_returns_error() {
        let mut pool = SamplingPortPool::<2, 8>::new();
        pool.create_port(PortDirection::Source, 10).unwrap();
        pool.create_port(PortDirection::Destination, 20).unwrap();

        let result = pool.create_port(PortDirection::Source, 30);
        assert_eq!(result, Err(SamplingError::PoolFull));
        assert_eq!(pool.len(), 2);
    }

    #[test]
    fn get_invalid_id_returns_none() {
        let pool = SamplingPortPool::<2, 8>::new();
        assert!(pool.get(99).is_none());
    }

    #[test]
    fn write_data_on_source_port() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let id = pool.create_port(PortDirection::Source, 100).unwrap();

        let port = pool.get_mut(id).unwrap();
        port.write_data(&[1, 2, 3], 50).unwrap();

        let port = pool.get(id).unwrap();
        assert_eq!(port.current_size(), 3);
        assert_eq!(port.data(), &[1, 2, 3]);
        assert_eq!(port.timestamp(), 50);
    }

    #[test]
    fn write_data_on_destination_port_is_rejected() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let id = pool.create_port(PortDirection::Destination, 100).unwrap();

        let port = pool.get_mut(id).unwrap();
        let result = port.write_data(&[1, 2, 3], 50);
        assert_eq!(result, Err(SamplingError::DirectionViolation));
    }

    #[test]
    fn read_data_on_destination_port() {
        let mut port = SamplingPort::<16>::new(0, PortDirection::Destination, 100);
        // Simulate data arrival (in real use, the kernel copies from source to destination)
        port.data[..3].copy_from_slice(&[10, 20, 30]);
        port.current_size = 3;
        port.timestamp = 75;

        let (data, ts) = port.read_data().unwrap();
        assert_eq!(data, &[10, 20, 30]);
        assert_eq!(ts, 75);
    }

    #[test]
    fn read_data_on_source_port_is_rejected() {
        let port = SamplingPort::<16>::new(0, PortDirection::Source, 100);
        let result = port.read_data();
        assert_eq!(result, Err(SamplingError::DirectionViolation));
    }

    #[test]
    fn write_data_too_large_is_rejected() {
        let mut port = SamplingPort::<4>::new(0, PortDirection::Source, 100);
        let result = port.write_data(&[1, 2, 3, 4, 5], 10);
        assert_eq!(result, Err(SamplingError::MessageTooLarge));
    }

    #[test]
    fn validity_no_data_is_invalid() {
        let port = SamplingPort::<16>::new(0, PortDirection::Source, 100);
        assert_eq!(port.validity(0), Validity::Invalid);
    }

    #[test]
    fn validity_fresh_data_is_valid() {
        let mut port = SamplingPort::<16>::new(0, PortDirection::Source, 100);
        port.write_data(&[1], 50).unwrap();
        assert_eq!(port.validity(100), Validity::Valid);
    }

    #[test]
    fn validity_stale_data_is_invalid() {
        let mut port = SamplingPort::<16>::new(0, PortDirection::Source, 100);
        port.write_data(&[1], 50).unwrap();
        assert_eq!(port.validity(200), Validity::Invalid);
    }
}
