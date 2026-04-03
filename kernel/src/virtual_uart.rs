//! Virtual UART backend with TX/RX ring buffers and kernel-side state.
//!
//! `VirtualUartBackend` provides per-device TX and RX byte ring buffers
//! backed by `heapless::Deque<u8, CAPACITY>`. Single-core Cortex-M only.

use heapless::Deque;

use crate::virtual_device::{DeviceError, VirtualDevice};
use crate::PartitionId;

/// IOCTL command: drain all bytes from the TX ring buffer.
pub const IOCTL_FLUSH: u32 = 0x01;
/// IOCTL command: return number of bytes available in the RX ring buffer.
pub const IOCTL_AVAILABLE: u32 = 0x02;
/// IOCTL command: set the loopback peer device ID (`arg` is the peer ID).
pub const IOCTL_SET_PEER: u32 = 0x03;

/// Kernel-side state for one virtual UART channel.
pub struct VirtualUartBackend {
    device_id: u8,
    tx: Deque<u8, { Self::CAPACITY }>,
    rx: Deque<u8, { Self::CAPACITY }>,
    /// Bitmask of partitions that have opened this device (max 8).
    open_partitions: u8,
    /// Peer device ID for loopback wiring (connects two UARTs).
    loopback_peer: Option<u8>,
}

impl VirtualUartBackend {
    /// Ring buffer capacity for both TX and RX channels.
    pub const CAPACITY: usize = 64;

    /// Create a new backend with the given device ID.
    pub const fn new(device_id: u8) -> Self {
        Self {
            device_id,
            tx: Deque::new(),
            rx: Deque::new(),
            open_partitions: 0,
            loopback_peer: None,
        }
    }

    pub fn device_id(&self) -> u8 {
        self.device_id
    }

    pub fn open_partitions(&self) -> u8 {
        self.open_partitions
    }

    /// Mark a partition (0..7) as having opened this device.
    pub fn open(&mut self, partition_id: PartitionId) {
        debug_assert!(partition_id.as_raw() < 8);
        self.open_partitions |= 1 << partition_id.as_raw();
    }

    /// Mark a partition as having closed this device.
    pub fn close(&mut self, partition_id: PartitionId) {
        debug_assert!(partition_id.as_raw() < 8);
        self.open_partitions &= !(1 << partition_id.as_raw());
    }

    pub fn is_open(&self, partition_id: PartitionId) -> bool {
        (self.open_partitions & (1 << partition_id.as_raw())) != 0
    }

    pub fn set_loopback_peer(&mut self, peer_id: u8) {
        self.loopback_peer = Some(peer_id);
    }

    pub fn clear_loopback_peer(&mut self) {
        self.loopback_peer = None;
    }

    pub fn loopback_peer(&self) -> Option<u8> {
        self.loopback_peer
    }

    /// Shared helper: push bytes into a ring buffer, stopping when full.
    fn push_buf(deque: &mut Deque<u8, { Self::CAPACITY }>, data: &[u8]) -> usize {
        let mut count = 0;
        for &b in data {
            if deque.push_back(b).is_err() {
                break;
            }
            count += 1;
        }
        count
    }

    /// Enqueue bytes into the TX ring buffer. Returns count written.
    pub fn push_tx(&mut self, data: &[u8]) -> usize {
        Self::push_buf(&mut self.tx, data)
    }

    /// Dequeue one byte from the TX ring buffer (kernel/ISR drains this).
    pub fn pop_tx(&mut self) -> Option<u8> {
        self.tx.pop_front()
    }

    pub fn tx_len(&self) -> usize {
        self.tx.len()
    }

    /// Enqueue bytes into the RX ring buffer (from ISR top-half).
    /// Returns count written.
    pub fn push_rx(&mut self, data: &[u8]) -> usize {
        Self::push_buf(&mut self.rx, data)
    }

    /// Drain bytes from the RX ring buffer into `buf` (for partition read).
    /// Returns count read.
    pub fn pop_rx(&mut self, buf: &mut [u8]) -> usize {
        // NOTE: Reviewer suggested `zip(&mut self.rx)` but `&mut Deque`
        // delegates to `iter_mut()` which borrows in-place without popping.
        // We need `pop_front()` to actually drain the ring buffer.
        let mut count = 0;
        for slot in buf.iter_mut() {
            if let Some(b) = self.rx.pop_front() {
                *slot = b;
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    pub fn rx_len(&self) -> usize {
        self.rx.len()
    }

    /// Number of free slots remaining in the RX ring buffer.
    pub fn rx_free_space(&self) -> usize {
        Self::CAPACITY - self.rx.len()
    }
}

impl VirtualUartBackend {
    /// Validate partition_id range and check that the partition has opened
    /// this device. Consolidates the repeated open-check boilerplate.
    fn require_open(&self, partition_id: PartitionId) -> Result<(), DeviceError> {
        if partition_id.as_raw() >= 8 {
            return Err(DeviceError::InvalidPartition);
        }
        if !self.is_open(partition_id) {
            return Err(DeviceError::NotOpen);
        }
        Ok(())
    }
}

impl VirtualDevice for VirtualUartBackend {
    fn device_id(&self) -> u8 {
        self.device_id
    }

    fn open(&mut self, partition_id: PartitionId) -> Result<(), DeviceError> {
        if partition_id.as_raw() >= 8 {
            return Err(DeviceError::InvalidPartition);
        }
        self.open_partitions |= 1 << partition_id.as_raw();
        Ok(())
    }

    fn close(&mut self, partition_id: PartitionId) -> Result<(), DeviceError> {
        if partition_id.as_raw() >= 8 {
            return Err(DeviceError::InvalidPartition);
        }
        self.open_partitions &= !(1 << partition_id.as_raw());
        Ok(())
    }

    fn read(&mut self, partition_id: PartitionId, buf: &mut [u8]) -> Result<usize, DeviceError> {
        self.require_open(partition_id)?;
        Ok(self.pop_rx(buf))
    }

    fn write(&mut self, partition_id: PartitionId, data: &[u8]) -> Result<usize, DeviceError> {
        self.require_open(partition_id)?;
        Ok(self.push_tx(data))
    }

    fn ioctl(&mut self, partition_id: PartitionId, cmd: u32, arg: u32) -> Result<u32, DeviceError> {
        self.require_open(partition_id)?;
        match cmd {
            IOCTL_FLUSH => {
                while self.pop_tx().is_some() {}
                Ok(0)
            }
            IOCTL_AVAILABLE => Ok(self.rx_len() as u32),
            IOCTL_SET_PEER => {
                self.set_loopback_peer(arg as u8);
                Ok(0)
            }
            _ => Err(DeviceError::NotFound),
        }
    }
}

/// A pair of virtual UARTs wired for loopback: UART-A TX routes to
/// UART-B RX and vice versa. Call [`transfer()`](VirtualUartPair::transfer)
/// as the bottom-half to drain each side's TX into the peer's RX.
pub struct VirtualUartPair {
    pub a: VirtualUartBackend,
    pub b: VirtualUartBackend,
}

impl VirtualUartPair {
    /// Create a new pair with the given device IDs.
    pub fn new(a_id: u8, b_id: u8) -> Self {
        let mut a = VirtualUartBackend::new(a_id);
        let mut b = VirtualUartBackend::new(b_id);
        a.set_loopback_peer(b_id);
        b.set_loopback_peer(a_id);
        Self { a, b }
    }

    /// Look up a backend by device ID, returning a mutable trait-object
    /// reference or `None`.
    pub fn get_mut(&mut self, id: u8) -> Option<&mut dyn VirtualDevice> {
        if self.a.device_id == id {
            Some(&mut self.a)
        } else if self.b.device_id == id {
            Some(&mut self.b)
        } else {
            None
        }
    }

    /// Bottom-half work: drain UART-A TX → UART-B RX, then
    /// UART-B TX → UART-A RX. Returns `(a_to_b, b_to_a)` byte counts.
    pub fn transfer(&mut self) -> (usize, usize) {
        let a_to_b = Self::route(&mut self.a, &mut self.b);
        let b_to_a = Self::route(&mut self.b, &mut self.a);
        (a_to_b, b_to_a)
    }

    /// Drain bytes from `src` TX into `dst` RX one at a time until
    /// either the source is empty or the destination is full.
    fn route(src: &mut VirtualUartBackend, dst: &mut VirtualUartBackend) -> usize {
        let mut count = 0;
        while let Some(byte) = src.tx.pop_front() {
            if dst.rx.push_back(byte).is_err() {
                // Destination full — put the byte back.
                let _ = src.tx.push_front(byte);
                break;
            }
            count += 1;
        }
        count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pid(v: u8) -> PartitionId {
        PartitionId::new(v as u32)
    }

    #[test]
    fn new_backend_is_empty() {
        let ub = VirtualUartBackend::new(7);
        assert_eq!(ub.device_id(), 7);
        assert_eq!(ub.tx_len(), 0);
        assert_eq!(ub.rx_len(), 0);
        assert_eq!(ub.open_partitions(), 0);
        assert_eq!(ub.loopback_peer(), None);
    }

    #[test]
    fn tx_push_pop_and_fifo() {
        let mut ub = VirtualUartBackend::new(0);
        assert_eq!(ub.push_tx(&[]), 0);
        assert_eq!(ub.push_tx(&[0xAB]), 1);
        assert_eq!(ub.tx_len(), 1);
        assert_eq!(ub.pop_tx(), Some(0xAB));
        assert_eq!(ub.pop_tx(), None);
        // Multi-byte FIFO
        assert_eq!(ub.push_tx(&[1, 2, 3, 4, 5]), 5);
        for expected in 1..=5u8 {
            assert_eq!(ub.pop_tx(), Some(expected));
        }
        assert_eq!(ub.pop_tx(), None);
    }

    #[test]
    fn tx_buffer_full_and_partial_write() {
        let mut ub = VirtualUartBackend::new(0);
        let cap = VirtualUartBackend::CAPACITY;
        assert_eq!(ub.push_tx(&[0xFFu8; VirtualUartBackend::CAPACITY]), cap);
        assert_eq!(ub.tx_len(), cap);
        assert_eq!(ub.push_tx(&[0x01]), 0);
        // Drain one, push two => only 1 fits
        ub.pop_tx();
        assert_eq!(ub.push_tx(&[0xAA, 0xBB]), 1);
        assert_eq!(ub.tx_len(), cap);
    }

    #[test]
    fn rx_push_pop_fifo_and_partial_drain() {
        let mut ub = VirtualUartBackend::new(0);
        let mut buf = [0u8; 4];
        assert_eq!(ub.pop_rx(&mut buf), 0); // empty
        assert_eq!(ub.push_rx(&[]), 0);
        assert_eq!(ub.push_rx(&[0xCD]), 1);
        assert_eq!(ub.rx_len(), 1);
        assert_eq!(ub.pop_rx(&mut buf[..1]), 1);
        assert_eq!(buf[0], 0xCD);
        // Multi-byte + pop larger buf than available
        assert_eq!(ub.push_rx(&[10, 20, 30, 40]), 4);
        let mut big = [0u8; 8];
        assert_eq!(ub.pop_rx(&mut big), 4);
        assert_eq!(&big[..4], &[10, 20, 30, 40]);
        // Partial drain: read fewer bytes than available
        ub.push_rx(&[1, 2, 3, 4, 5]);
        let mut small = [0u8; 3];
        assert_eq!(ub.pop_rx(&mut small), 3);
        assert_eq!(&small, &[1, 2, 3]);
        assert_eq!(ub.rx_len(), 2);
    }

    #[test]
    fn rx_buffer_full_and_partial_write() {
        let mut ub = VirtualUartBackend::new(0);
        let cap = VirtualUartBackend::CAPACITY;
        assert_eq!(ub.push_rx(&[0xEEu8; VirtualUartBackend::CAPACITY]), cap);
        assert_eq!(ub.push_rx(&[0x01]), 0);
        let mut tmp = [0u8; 1];
        ub.pop_rx(&mut tmp);
        assert_eq!(ub.push_rx(&[0xAA, 0xBB]), 1);
        assert_eq!(ub.rx_len(), cap);
    }

    #[test]
    fn partition_bitmask_and_loopback() {
        let mut ub = VirtualUartBackend::new(1);
        assert!(!ub.is_open(pid(0)));
        ub.open(pid(0));
        assert!(ub.is_open(pid(0)));
        assert_eq!(ub.open_partitions(), 0b0000_0001);
        ub.open(pid(3));
        assert_eq!(ub.open_partitions(), 0b0000_1001);
        ub.close(pid(0));
        assert!(!ub.is_open(pid(0)));
        assert!(ub.is_open(pid(3)));
        ub.open(pid(3)); // idempotent
        assert_eq!(ub.open_partitions(), 0b0000_1000);
        ub.close(pid(5)); // already-closed is no-op
        assert_eq!(ub.open_partitions(), 0b0000_1000);
        // All eight partitions
        for i in 0..8u32 {
            ub.open(PartitionId::new(i));
        }
        assert_eq!(ub.open_partitions(), 0xFF);
        // Loopback peer
        assert_eq!(ub.loopback_peer(), None);
        ub.set_loopback_peer(2);
        assert_eq!(ub.loopback_peer(), Some(2));
        ub.clear_loopback_peer();
        assert_eq!(ub.loopback_peer(), None);
    }

    #[test]
    fn tx_and_rx_are_independent() {
        let mut ub = VirtualUartBackend::new(0);
        ub.push_tx(&[1, 2, 3]);
        ub.push_rx(&[10, 20, 30]);
        assert_eq!(ub.pop_tx(), Some(1));
        let mut buf = [0u8; 2];
        assert_eq!(ub.pop_rx(&mut buf), 2);
        assert_eq!(buf, [10, 20]);
        assert_eq!(ub.tx_len(), 2);
        assert_eq!(ub.rx_len(), 1);
    }

    // ---- VirtualDevice trait tests ----

    #[test]
    fn trait_open_close_manages_bitmask() {
        let mut ub = VirtualUartBackend::new(5);
        assert_eq!(VirtualDevice::device_id(&ub), 5);

        // Open partition 2
        VirtualDevice::open(&mut ub, pid(2)).unwrap();
        assert!(ub.is_open(pid(2)));
        assert_eq!(ub.open_partitions(), 0b0000_0100);

        // Open partition 0 as well
        VirtualDevice::open(&mut ub, pid(0)).unwrap();
        assert_eq!(ub.open_partitions(), 0b0000_0101);

        // Close partition 2
        VirtualDevice::close(&mut ub, pid(2)).unwrap();
        assert!(!ub.is_open(pid(2)));
        assert!(ub.is_open(pid(0)));
        assert_eq!(ub.open_partitions(), 0b0000_0001);

        // Close partition 0
        VirtualDevice::close(&mut ub, pid(0)).unwrap();
        assert_eq!(ub.open_partitions(), 0);
    }

    #[test]
    fn trait_read_write_not_open_error() {
        let mut ub = VirtualUartBackend::new(1);
        let mut buf = [0u8; 4];

        // read/write without opening should fail
        assert_eq!(
            VirtualDevice::read(&mut ub, pid(0), &mut buf),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::write(&mut ub, pid(0), &[1, 2]),
            Err(DeviceError::NotOpen)
        );
    }

    #[test]
    fn trait_write_delegates_to_push_tx() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        // Write data via trait — should go into TX buffer
        let n = VirtualDevice::write(&mut ub, pid(0), &[0xAA, 0xBB, 0xCC]).unwrap();
        assert_eq!(n, 3);
        assert_eq!(ub.tx_len(), 3);

        // Verify FIFO order via pop_tx
        assert_eq!(ub.pop_tx(), Some(0xAA));
        assert_eq!(ub.pop_tx(), Some(0xBB));
        assert_eq!(ub.pop_tx(), Some(0xCC));
    }

    #[test]
    fn trait_read_delegates_to_pop_rx() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        // Pre-fill RX buffer (as if ISR pushed data)
        ub.push_rx(&[10, 20, 30, 40]);

        // Read via trait
        let mut buf = [0u8; 3];
        let n = VirtualDevice::read(&mut ub, pid(0), &mut buf).unwrap();
        assert_eq!(n, 3);
        assert_eq!(buf, [10, 20, 30]);
        assert_eq!(ub.rx_len(), 1); // one byte left
    }

    #[test]
    fn trait_full_lifecycle() {
        let mut ub = VirtualUartBackend::new(3);

        // 1. open
        VirtualDevice::open(&mut ub, pid(1)).unwrap();

        // 2. write
        let written = VirtualDevice::write(&mut ub, pid(1), &[1, 2, 3]).unwrap();
        assert_eq!(written, 3);

        // 3. simulate RX data arrival
        ub.push_rx(&[0xDE, 0xAD]);

        // 4. read
        let mut buf = [0u8; 4];
        let read = VirtualDevice::read(&mut ub, pid(1), &mut buf).unwrap();
        assert_eq!(read, 2);
        assert_eq!(&buf[..2], &[0xDE, 0xAD]);

        // 5. close
        VirtualDevice::close(&mut ub, pid(1)).unwrap();

        // 6. operations fail after close
        assert_eq!(
            VirtualDevice::read(&mut ub, pid(1), &mut buf),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::write(&mut ub, pid(1), &[0]),
            Err(DeviceError::NotOpen)
        );
    }

    #[test]
    fn trait_ioctl_not_open_error() {
        let mut ub = VirtualUartBackend::new(1);
        assert_eq!(
            VirtualDevice::ioctl(&mut ub, pid(0), IOCTL_FLUSH, 0),
            Err(DeviceError::NotOpen)
        );
    }

    #[test]
    fn trait_ioctl_flush_drains_tx() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        VirtualDevice::write(&mut ub, pid(0), &[1, 2, 3, 4, 5]).unwrap();
        assert_eq!(ub.tx_len(), 5);

        let result = VirtualDevice::ioctl(&mut ub, pid(0), IOCTL_FLUSH, 0).unwrap();
        assert_eq!(result, 0);
        assert_eq!(ub.tx_len(), 0);
    }

    #[test]
    fn trait_ioctl_available_returns_rx_len() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        let avail = VirtualDevice::ioctl(&mut ub, pid(0), IOCTL_AVAILABLE, 0).unwrap();
        assert_eq!(avail, 0);

        ub.push_rx(&[10, 20, 30]);
        let avail = VirtualDevice::ioctl(&mut ub, pid(0), IOCTL_AVAILABLE, 0).unwrap();
        assert_eq!(avail, 3);
    }

    #[test]
    fn trait_ioctl_set_peer() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        assert_eq!(ub.loopback_peer(), None);
        let result = VirtualDevice::ioctl(&mut ub, pid(0), IOCTL_SET_PEER, 42).unwrap();
        assert_eq!(result, 0);
        assert_eq!(ub.loopback_peer(), Some(42));
    }

    #[test]
    fn trait_ioctl_unknown_command() {
        let mut ub = VirtualUartBackend::new(1);
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        assert_eq!(
            VirtualDevice::ioctl(&mut ub, pid(0), 0xFF, 0),
            Err(DeviceError::NotFound)
        );
    }

    #[test]
    fn trait_invalid_partition_id_rejected() {
        let mut ub = VirtualUartBackend::new(1);
        let mut buf = [0u8; 4];

        // partition_id >= 8 must be rejected on all trait methods
        assert_eq!(
            VirtualDevice::open(&mut ub, pid(8)),
            Err(DeviceError::InvalidPartition)
        );
        assert_eq!(
            VirtualDevice::close(&mut ub, pid(8)),
            Err(DeviceError::InvalidPartition)
        );
        assert_eq!(
            VirtualDevice::read(&mut ub, pid(8), &mut buf),
            Err(DeviceError::InvalidPartition)
        );
        assert_eq!(
            VirtualDevice::write(&mut ub, pid(8), &[1]),
            Err(DeviceError::InvalidPartition)
        );
        assert_eq!(
            VirtualDevice::ioctl(&mut ub, pid(8), IOCTL_FLUSH, 0),
            Err(DeviceError::InvalidPartition)
        );

        // u8::MAX should also be rejected
        assert_eq!(
            VirtualDevice::open(&mut ub, pid(255)),
            Err(DeviceError::InvalidPartition)
        );
    }

    #[test]
    fn trait_multi_partition_isolation() {
        let mut ub = VirtualUartBackend::new(1);

        // Partition 0 opens the device
        VirtualDevice::open(&mut ub, pid(0)).unwrap();

        // Partition 1 hasn't opened — should fail
        assert_eq!(
            VirtualDevice::write(&mut ub, pid(1), &[0xAA]),
            Err(DeviceError::NotOpen)
        );

        // Partition 0 can write
        assert_eq!(VirtualDevice::write(&mut ub, pid(0), &[0xBB]).unwrap(), 1);

        // Now partition 1 opens
        VirtualDevice::open(&mut ub, pid(1)).unwrap();
        assert_eq!(VirtualDevice::write(&mut ub, pid(1), &[0xCC]).unwrap(), 1);

        // Close partition 0 — partition 1 should still work
        VirtualDevice::close(&mut ub, pid(0)).unwrap();
        assert_eq!(
            VirtualDevice::write(&mut ub, pid(0), &[0xDD]),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(VirtualDevice::write(&mut ub, pid(1), &[0xEE]).unwrap(), 1);
    }

    // ---- VirtualUartPair tests ----

    #[test]
    fn pair_new_sets_loopback_peers() {
        let pair = VirtualUartPair::new(10, 20);
        assert_eq!(pair.a.device_id(), 10);
        assert_eq!(pair.b.device_id(), 20);
        assert_eq!(pair.a.loopback_peer(), Some(20));
        assert_eq!(pair.b.loopback_peer(), Some(10));
    }

    #[test]
    fn pair_a_writes_b_reads() {
        let mut pair = VirtualUartPair::new(0, 1);
        pair.a.push_tx(&[0xAA, 0xBB, 0xCC]);

        let (a_to_b, b_to_a) = pair.transfer();
        assert_eq!(a_to_b, 3);
        assert_eq!(b_to_a, 0);

        // A's TX should be drained
        assert_eq!(pair.a.tx_len(), 0);
        // B's RX should have the data
        let mut buf = [0u8; 4];
        let n = pair.b.pop_rx(&mut buf);
        assert_eq!(n, 3);
        assert_eq!(&buf[..3], &[0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn pair_b_writes_a_reads() {
        let mut pair = VirtualUartPair::new(0, 1);
        pair.b.push_tx(&[1, 2, 3, 4]);

        let (a_to_b, b_to_a) = pair.transfer();
        assert_eq!(a_to_b, 0);
        assert_eq!(b_to_a, 4);

        assert_eq!(pair.b.tx_len(), 0);
        let mut buf = [0u8; 4];
        let n = pair.a.pop_rx(&mut buf);
        assert_eq!(n, 4);
        assert_eq!(&buf, &[1, 2, 3, 4]);
    }

    #[test]
    fn pair_bidirectional() {
        let mut pair = VirtualUartPair::new(0, 1);
        pair.a.push_tx(&[0x10, 0x20]);
        pair.b.push_tx(&[0x30, 0x40, 0x50]);

        let (a_to_b, b_to_a) = pair.transfer();
        assert_eq!(a_to_b, 2);
        assert_eq!(b_to_a, 3);

        // B received A's data
        let mut buf_b = [0u8; 4];
        assert_eq!(pair.b.pop_rx(&mut buf_b), 2);
        assert_eq!(&buf_b[..2], &[0x10, 0x20]);

        // A received B's data
        let mut buf_a = [0u8; 4];
        assert_eq!(pair.a.pop_rx(&mut buf_a), 3);
        assert_eq!(&buf_a[..3], &[0x30, 0x40, 0x50]);
    }

    #[test]
    fn pair_transfer_empty_is_noop() {
        let mut pair = VirtualUartPair::new(0, 1);

        let (a_to_b, b_to_a) = pair.transfer();
        assert_eq!(a_to_b, 0);
        assert_eq!(b_to_a, 0);
        assert_eq!(pair.a.tx_len(), 0);
        assert_eq!(pair.a.rx_len(), 0);
        assert_eq!(pair.b.tx_len(), 0);
        assert_eq!(pair.b.rx_len(), 0);
    }

    #[test]
    fn pair_partial_transfer_rx_nearly_full() {
        let cap = VirtualUartBackend::CAPACITY;
        let mut pair = VirtualUartPair::new(0, 1);

        // Fill B's RX to cap-4 slots
        let prefill = cap - 4;
        pair.b.push_rx(&vec![0xFFu8; prefill]);
        assert_eq!(pair.b.rx_len(), prefill);

        // A writes 10 bytes — only 4 should transfer
        pair.a.push_tx(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        let (a_to_b, _) = pair.transfer();
        assert_eq!(a_to_b, 4);

        // B's RX is now full
        assert_eq!(pair.b.rx_len(), cap);

        // A still has 6 bytes in TX that didn't transfer
        assert_eq!(pair.a.tx_len(), 6);

        // The 6 remaining bytes in A's TX are the tail of the original data
        for expected in 5..=10u8 {
            assert_eq!(pair.a.pop_tx(), Some(expected));
        }
    }

    #[test]
    fn pair_full_capacity_transfer() {
        let cap = VirtualUartBackend::CAPACITY;
        let mut pair = VirtualUartPair::new(0, 1);

        // Fill A's TX completely
        pair.a.push_tx(&[0xABu8; VirtualUartBackend::CAPACITY]);
        assert_eq!(pair.a.tx_len(), cap);

        let (a_to_b, _) = pair.transfer();
        assert_eq!(a_to_b, cap);
        assert_eq!(pair.a.tx_len(), 0);
        assert_eq!(pair.b.rx_len(), cap);
    }
}
