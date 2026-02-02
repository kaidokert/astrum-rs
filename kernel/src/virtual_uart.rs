//! Virtual UART backend with TX/RX ring buffers and kernel-side state.
//!
//! `VirtualUartBackend` provides per-device TX and RX byte ring buffers
//! backed by `heapless::Deque<u8, 64>`. Single-core Cortex-M only.

use heapless::Deque;

/// Kernel-side state for one virtual UART channel.
pub struct VirtualUartBackend {
    device_id: u8,
    tx: Deque<u8, 64>,
    rx: Deque<u8, 64>,
    /// Bitmask of partitions that have opened this device (max 8).
    open_partitions: u8,
    /// Peer device ID for loopback wiring (connects two UARTs).
    loopback_peer: Option<u8>,
}

impl VirtualUartBackend {
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
    pub fn open(&mut self, partition_id: u8) {
        debug_assert!(partition_id < 8);
        self.open_partitions |= 1 << partition_id;
    }

    /// Mark a partition as having closed this device.
    pub fn close(&mut self, partition_id: u8) {
        debug_assert!(partition_id < 8);
        self.open_partitions &= !(1 << partition_id);
    }

    pub fn is_open(&self, partition_id: u8) -> bool {
        (self.open_partitions & (1 << partition_id)) != 0
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
    fn push_buf(deque: &mut Deque<u8, 64>, data: &[u8]) -> usize {
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(ub.push_tx(&[0xFFu8; 64]), 64);
        assert_eq!(ub.tx_len(), 64);
        assert_eq!(ub.push_tx(&[0x01]), 0);
        // Drain one, push two => only 1 fits
        ub.pop_tx();
        assert_eq!(ub.push_tx(&[0xAA, 0xBB]), 1);
        assert_eq!(ub.tx_len(), 64);
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
        assert_eq!(ub.push_rx(&[0xEEu8; 64]), 64);
        assert_eq!(ub.push_rx(&[0x01]), 0);
        let mut tmp = [0u8; 1];
        ub.pop_rx(&mut tmp);
        assert_eq!(ub.push_rx(&[0xAA, 0xBB]), 1);
        assert_eq!(ub.rx_len(), 64);
    }

    #[test]
    fn partition_bitmask_and_loopback() {
        let mut ub = VirtualUartBackend::new(1);
        assert!(!ub.is_open(0));
        ub.open(0);
        assert!(ub.is_open(0));
        assert_eq!(ub.open_partitions(), 0b0000_0001);
        ub.open(3);
        assert_eq!(ub.open_partitions(), 0b0000_1001);
        ub.close(0);
        assert!(!ub.is_open(0));
        assert!(ub.is_open(3));
        ub.open(3); // idempotent
        assert_eq!(ub.open_partitions(), 0b0000_1000);
        ub.close(5); // already-closed is no-op
        assert_eq!(ub.open_partitions(), 0b0000_1000);
        // All eight partitions
        for i in 0..8 {
            ub.open(i);
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
}
