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
    InvalidPort,
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
    // Port IDs equal their Vec index (port N is at ports[N]). This invariant holds
    // because ports are append-only — deletion is deliberately unsupported. ARINC 653
    // sampling ports are created at system init and persist for the partition's lifetime,
    // so deletion is out of scope. If deletion were ever added, a generation-based ID
    // scheme (or a slot map) would be needed to prevent stale-ID aliasing.
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

    /// O(1) lookup by port ID. Port IDs equal their Vec index (see invariant above).
    pub fn get(&self, id: usize) -> Option<&SamplingPort<M>> {
        self.ports.get(id)
    }

    /// O(1) mutable lookup by port ID.
    pub fn get_mut(&mut self, id: usize) -> Option<&mut SamplingPort<M>> {
        self.ports.get_mut(id)
    }

    pub fn len(&self) -> usize {
        self.ports.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ports.is_empty()
    }

    fn write_port_buffer(&mut self, id: usize, data: &[u8], ts: u64) -> Result<(), SamplingError> {
        let port = self.get_mut(id).ok_or(SamplingError::InvalidPort)?;
        port.data[..data.len()].copy_from_slice(data);
        port.current_size = data.len();
        port.timestamp = ts;
        Ok(())
    }

    pub fn connect_ports(&mut self, src: usize, dst: usize) -> Result<(), SamplingError> {
        if self.get(src).ok_or(SamplingError::InvalidPort)?.direction() != PortDirection::Source {
            return Err(SamplingError::DirectionViolation);
        }
        if self.get(dst).ok_or(SamplingError::InvalidPort)?.direction()
            != PortDirection::Destination
        {
            return Err(SamplingError::DirectionViolation);
        }
        // SAFETY: get(src) succeeded at line above (returned Some via ok_or),
        // so get_mut(src) on the same pool with the same index cannot fail.
        self.get_mut(src).unwrap().connected_port = Some(dst);
        Ok(())
    }

    pub fn write_sampling_message(
        &mut self,
        port_id: usize,
        data: &[u8],
        timestamp: u64,
    ) -> Result<(), SamplingError> {
        let port = self.get(port_id).ok_or(SamplingError::InvalidPort)?;
        if port.direction() != PortDirection::Source {
            return Err(SamplingError::DirectionViolation);
        }
        if data.len() > M {
            return Err(SamplingError::MessageTooLarge);
        }
        let dest = port.connected_port;
        if let Some(dst_id) = dest {
            self.write_port_buffer(dst_id, data, timestamp)?;
        }
        self.write_port_buffer(port_id, data, timestamp)?;
        Ok(())
    }

    pub fn read_sampling_message(
        &self,
        port_id: usize,
        buf: &mut [u8],
        current_tick: u64,
    ) -> Result<(usize, Validity), SamplingError> {
        let port = self.get(port_id).ok_or(SamplingError::InvalidPort)?;
        let (data, _ts) = port.read_data()?;
        let size = data.len();
        buf[..size].copy_from_slice(data);
        let validity = port.validity(current_tick);
        Ok((size, validity))
    }

    /// Kernel-side transfer: copies the current message from a source port
    /// into a destination port.
    pub fn transfer(
        &mut self,
        source_id: usize,
        destination_id: usize,
    ) -> Result<(), SamplingError> {
        let src = self.get(source_id).ok_or(SamplingError::InvalidPort)?;
        if src.direction() != PortDirection::Source {
            return Err(SamplingError::DirectionViolation);
        }
        let len = src.current_size();
        let ts = src.timestamp();
        // Copy source data to a stack buffer to release the immutable borrow
        let mut tmp = [0u8; M];
        tmp[..len].copy_from_slice(src.data());

        let dst = self
            .get_mut(destination_id)
            .ok_or(SamplingError::InvalidPort)?;
        if dst.direction() != PortDirection::Destination {
            return Err(SamplingError::DirectionViolation);
        }
        dst.data[..len].copy_from_slice(&tmp[..len]);
        dst.current_size = len;
        dst.timestamp = ts;
        Ok(())
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
        let mut pool = SamplingPortPool::<4, 16>::new();
        let src = pool.create_port(PortDirection::Source, 100).unwrap();
        let dst = pool.create_port(PortDirection::Destination, 100).unwrap();

        pool.write_sampling_message(src, &[10, 20, 30], 75).unwrap();
        pool.transfer(src, dst).unwrap();

        let dst_port = pool.get(dst).unwrap();
        let (data, ts) = dst_port.read_data().unwrap();
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

    #[test]
    fn pool_write_and_read_round_trip() {
        let mut pool = SamplingPortPool::<4, 64>::new();
        let src = pool.create_port(PortDirection::Source, 100).unwrap();
        let dst = pool.create_port(PortDirection::Destination, 100).unwrap();

        pool.write_sampling_message(src, &[10, 20, 30], 50).unwrap();
        pool.transfer(src, dst).unwrap();

        let mut buf = [0u8; 64];
        let (size, validity) = pool.read_sampling_message(dst, &mut buf, 80).unwrap();
        assert_eq!(size, 3);
        assert_eq!(&buf[..size], &[10, 20, 30]);
        assert_eq!(validity, Validity::Valid);
    }

    #[test]
    fn pool_write_overwrites_previous() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let id = pool.create_port(PortDirection::Source, 100).unwrap();

        pool.write_sampling_message(id, &[1, 2, 3], 10).unwrap();
        pool.write_sampling_message(id, &[4, 5], 20).unwrap();

        let port = pool.get(id).unwrap();
        assert_eq!(port.data(), &[4, 5]);
        assert_eq!(port.current_size(), 2);
        assert_eq!(port.timestamp(), 20);
    }

    #[test]
    fn pool_read_validity_stale() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let src = pool.create_port(PortDirection::Source, 50).unwrap();
        let dst = pool.create_port(PortDirection::Destination, 50).unwrap();

        pool.write_sampling_message(src, &[7, 8], 10).unwrap();
        pool.transfer(src, dst).unwrap();

        let mut buf = [0u8; 16];
        let (size, validity) = pool.read_sampling_message(dst, &mut buf, 100).unwrap();
        assert_eq!(size, 2);
        assert_eq!(&buf[..size], &[7, 8]);
        assert_eq!(validity, Validity::Invalid);
    }

    #[test]
    fn pool_write_wrong_direction() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let dst = pool.create_port(PortDirection::Destination, 100).unwrap();
        let result = pool.write_sampling_message(dst, &[1], 10);
        assert_eq!(result, Err(SamplingError::DirectionViolation));
    }

    #[test]
    fn pool_read_wrong_direction() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let src = pool.create_port(PortDirection::Source, 100).unwrap();
        let mut buf = [0u8; 16];
        let result = pool.read_sampling_message(src, &mut buf, 0);
        assert_eq!(result, Err(SamplingError::DirectionViolation));
    }

    #[test]
    fn pool_write_invalid_port() {
        let mut pool = SamplingPortPool::<4, 16>::new();
        let result = pool.write_sampling_message(99, &[1], 10);
        assert_eq!(result, Err(SamplingError::InvalidPort));
    }

    #[test]
    fn pool_read_invalid_port() {
        let pool = SamplingPortPool::<4, 16>::new();
        let mut buf = [0u8; 16];
        let result = pool.read_sampling_message(99, &mut buf, 0);
        assert_eq!(result, Err(SamplingError::InvalidPort));
    }

    #[test]
    fn pool_write_size_too_large() {
        let mut pool = SamplingPortPool::<4, 4>::new();
        let id = pool.create_port(PortDirection::Source, 100).unwrap();
        let result = pool.write_sampling_message(id, &[1, 2, 3, 4, 5], 10);
        assert_eq!(result, Err(SamplingError::MessageTooLarge));
    }

    #[test]
    fn id_lookup_after_multiple_creations() {
        let mut pool = SamplingPortPool::<8, 32>::new();
        let mut ids = [0usize; 5];
        for (i, slot) in ids.iter_mut().enumerate() {
            let dir = if i % 2 == 0 {
                PortDirection::Source
            } else {
                PortDirection::Destination
            };
            *slot = pool.create_port(dir, (i as u32 + 1) * 10).unwrap();
        }

        // Each ID should match its creation order and retrieve the correct port.
        for (i, &id) in ids.iter().enumerate() {
            assert_eq!(id, i, "port ID should equal creation index");
            let port = pool.get(id).expect("get() must find the port");
            assert_eq!(port.id(), id);
            assert_eq!(port.refresh_period(), (i as u32 + 1) * 10);
        }

        // Mutable lookup should also work for every port.
        for &id in &ids {
            assert!(pool.get_mut(id).is_some(), "get_mut() must find the port");
        }

        // Out-of-range IDs must return None.
        assert!(pool.get(ids.len()).is_none());
        assert!(pool.get(usize::MAX).is_none());
    }

    #[test]
    fn connect_routed_write_and_errors() {
        let mut pool = SamplingPortPool::<4, 64>::new();
        let src = pool.create_port(PortDirection::Source, 100).unwrap();
        let dst = pool.create_port(PortDirection::Destination, 100).unwrap();
        assert_eq!(
            pool.connect_ports(dst, src),
            Err(SamplingError::DirectionViolation)
        );
        assert_eq!(pool.connect_ports(src, 99), Err(SamplingError::InvalidPort));
        pool.connect_ports(src, dst).unwrap();
        assert_eq!(pool.get(src).unwrap().connected_port(), Some(dst));
        pool.write_sampling_message(src, &[10, 20, 30], 50).unwrap();
        let mut buf = [0u8; 64];
        let (sz, v) = pool.read_sampling_message(dst, &mut buf, 80).unwrap();
        assert_eq!((sz, &buf[..sz], v), (3, &[10, 20, 30][..], Validity::Valid));
    }
}
