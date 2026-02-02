//! Hardware UART backend with TX/RX ring buffers and `UartRegs` integration.
//!
//! `HwUartBackend` wraps a [`UartRegs`] hardware accessor alongside
//! software TX and RX ring buffers, implementing the [`VirtualDevice`]
//! trait for partition-aware access. Single-core Cortex-M only.

use heapless::Deque;

use crate::uart_hal::UartRegs;
use crate::virtual_device::{DeviceError, VirtualDevice};

/// IOCTL command: drain all bytes from the TX ring buffer.
pub const IOCTL_FLUSH: u32 = 0x01;
/// IOCTL command: return number of bytes available in the RX ring buffer.
pub const IOCTL_AVAILABLE: u32 = 0x02;

/// Ring buffer capacity for both TX and RX channels.
const CAPACITY: usize = 64;

/// Maximum number of partitions supported (limited by `u8` bitmask).
const MAX_PARTITIONS: u8 = 8;

/// Hardware UART backend with kernel-side TX/RX ring buffers.
///
/// `write()` pushes into TX; an ISR/bottom-half drains TX to hardware.
/// An ISR top-half calls `push_rx()` from hardware; `read()` pops RX.
pub struct HwUartBackend {
    device_id: u8,
    regs: UartRegs,
    tx: Deque<u8, CAPACITY>,
    rx: Deque<u8, CAPACITY>,
    /// Bitmask of partitions that have opened this device (max 8).
    open_partitions: u8,
}

impl HwUartBackend {
    /// Create a new hardware UART backend.
    pub fn new(device_id: u8, regs: UartRegs) -> Self {
        Self {
            device_id,
            regs,
            tx: Deque::new(),
            rx: Deque::new(),
            open_partitions: 0,
        }
    }

    /// Return a reference to the underlying UART registers.
    pub fn regs(&self) -> &UartRegs {
        &self.regs
    }

    /// Number of bytes currently in the TX ring buffer.
    pub fn tx_len(&self) -> usize {
        self.tx.len()
    }

    /// Number of bytes currently in the RX ring buffer.
    pub fn rx_len(&self) -> usize {
        self.rx.len()
    }

    /// Push bytes into the RX ring buffer (called by ISR top-half).
    pub fn push_rx(&mut self, data: &[u8]) -> usize {
        let mut count = 0;
        for &b in data {
            if self.rx.push_back(b).is_err() {
                break;
            }
            count += 1;
        }
        count
    }

    fn validate_partition(partition_id: u8) -> Result<(), DeviceError> {
        if partition_id >= MAX_PARTITIONS {
            return Err(DeviceError::InvalidPartition);
        }
        Ok(())
    }

    fn require_open(&self, partition_id: u8) -> Result<(), DeviceError> {
        Self::validate_partition(partition_id)?;
        if self.open_partitions & (1 << partition_id) == 0 {
            return Err(DeviceError::NotOpen);
        }
        Ok(())
    }

    /// ISR top-half: read bytes from UART DR into the RX ring buffer.
    ///
    /// Reads via [`UartRegs::read_byte`] until the receive FIFO is empty
    /// (RXFE set) or the RX ring buffer is full. Returns the number of
    /// bytes transferred.
    pub fn isr_rx_to_ring(&mut self) -> usize {
        let mut count = 0;
        while !self.rx.is_full() {
            match self.regs.read_byte() {
                Some(b) => {
                    // Ring not full (checked above), push cannot fail.
                    let _ = self.rx.push_back(b);
                    count += 1;
                }
                None => break, // RXFE — hardware FIFO empty
            }
        }
        count
    }

    /// Bottom-half drain: pop bytes from the TX ring buffer and write
    /// them to UART DR.
    ///
    /// Writes via [`UartRegs::is_tx_full`] + DR write until the
    /// hardware transmit FIFO is full (TXFF set) or the TX ring buffer
    /// is empty. Returns the number of bytes transferred.
    pub fn drain_tx_to_hw(&mut self) -> usize {
        let mut count = 0;
        while !self.regs.is_tx_full() {
            match self.tx.pop_front() {
                Some(b) => {
                    self.regs.write_byte(b);
                    count += 1;
                }
                None => break, // TX ring empty
            }
        }
        count
    }

    /// Inject bytes into the RX ring buffer from an ISR or test context.
    ///
    /// Pushes bytes until the buffer is full or all data is consumed.
    /// Returns the number of bytes actually enqueued.
    pub fn push_rx_from_isr(&mut self, data: &[u8]) -> usize {
        let mut count = 0;
        for &b in data {
            if self.rx.push_back(b).is_err() {
                break;
            }
            count += 1;
        }
        count
    }
}

impl VirtualDevice for HwUartBackend {
    fn device_id(&self) -> u8 {
        self.device_id
    }

    fn open(&mut self, partition_id: u8) -> Result<(), DeviceError> {
        Self::validate_partition(partition_id)?;
        self.open_partitions |= 1 << partition_id;
        Ok(())
    }

    fn close(&mut self, partition_id: u8) -> Result<(), DeviceError> {
        Self::validate_partition(partition_id)?;
        self.open_partitions &= !(1 << partition_id);
        Ok(())
    }

    fn read(&mut self, partition_id: u8, buf: &mut [u8]) -> Result<usize, DeviceError> {
        self.require_open(partition_id)?;
        let mut count = 0;
        for slot in buf.iter_mut() {
            if let Some(b) = self.rx.pop_front() {
                *slot = b;
                count += 1;
            } else {
                break;
            }
        }
        Ok(count)
    }

    fn write(&mut self, partition_id: u8, data: &[u8]) -> Result<usize, DeviceError> {
        self.require_open(partition_id)?;
        let mut count = 0;
        for &b in data {
            if self.tx.push_back(b).is_err() {
                break;
            }
            count += 1;
        }
        Ok(count)
    }

    fn ioctl(&mut self, partition_id: u8, cmd: u32, _arg: u32) -> Result<u32, DeviceError> {
        self.require_open(partition_id)?;
        match cmd {
            IOCTL_FLUSH => {
                self.tx.clear();
                Ok(0)
            }
            IOCTL_AVAILABLE => Ok(self.rx.len() as u32),
            _ => Err(DeviceError::NotFound),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_backend(id: u8) -> HwUartBackend {
        HwUartBackend::new(id, UartRegs::new(0x4000_C000))
    }

    #[test]
    fn new_backend_is_empty() {
        let hw = make_backend(1);
        assert_eq!(VirtualDevice::device_id(&hw), 1);
        assert_eq!(hw.tx_len(), 0);
        assert_eq!(hw.rx_len(), 0);
        assert_eq!(hw.regs().base(), 0x4000_C000);
    }

    #[test]
    fn open_close_manages_partition_bitmask() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        assert_eq!(hw.open_partitions, 0b0000_0001);
        VirtualDevice::open(&mut hw, 3).unwrap();
        assert_eq!(hw.open_partitions, 0b0000_1001);
        VirtualDevice::close(&mut hw, 0).unwrap();
        assert_eq!(hw.open_partitions, 0b0000_1000);
        VirtualDevice::close(&mut hw, 3).unwrap();
        assert_eq!(hw.open_partitions, 0);
    }

    #[test]
    fn write_pushes_to_tx_buffer() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        let n = VirtualDevice::write(&mut hw, 0, &[0xAA, 0xBB, 0xCC]).unwrap();
        assert_eq!(n, 3);
        assert_eq!(hw.tx_len(), 3);
    }

    #[test]
    fn read_pops_from_rx_buffer() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        hw.push_rx(&[10, 20, 30, 40]);
        let mut buf = [0u8; 3];
        let n = VirtualDevice::read(&mut hw, 0, &mut buf).unwrap();
        assert_eq!(n, 3);
        assert_eq!(buf, [10, 20, 30]);
        assert_eq!(hw.rx_len(), 1);
    }

    #[test]
    fn ioctl_flush_and_available() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        VirtualDevice::write(&mut hw, 0, &[1, 2, 3, 4, 5]).unwrap();
        assert_eq!(hw.tx_len(), 5);
        assert_eq!(VirtualDevice::ioctl(&mut hw, 0, IOCTL_FLUSH, 0).unwrap(), 0);
        assert_eq!(hw.tx_len(), 0);
        assert_eq!(
            VirtualDevice::ioctl(&mut hw, 0, IOCTL_AVAILABLE, 0).unwrap(),
            0
        );
        hw.push_rx(&[0xDE, 0xAD]);
        assert_eq!(
            VirtualDevice::ioctl(&mut hw, 0, IOCTL_AVAILABLE, 0).unwrap(),
            2
        );
    }

    #[test]
    fn full_lifecycle_open_write_read_close() {
        let mut hw = make_backend(5);
        VirtualDevice::open(&mut hw, 1).unwrap();
        assert_eq!(VirtualDevice::write(&mut hw, 1, &[0xCA, 0xFE]).unwrap(), 2);
        hw.push_rx(&[0xBE, 0xEF]);
        let mut buf = [0u8; 4];
        assert_eq!(VirtualDevice::read(&mut hw, 1, &mut buf).unwrap(), 2);
        assert_eq!(&buf[..2], &[0xBE, 0xEF]);
        VirtualDevice::close(&mut hw, 1).unwrap();
        assert_eq!(
            VirtualDevice::read(&mut hw, 1, &mut buf),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::write(&mut hw, 1, &[0]),
            Err(DeviceError::NotOpen)
        );
    }

    #[test]
    fn error_not_open_invalid_partition_unknown_ioctl() {
        let mut hw = make_backend(1);
        let mut buf = [0u8; 4];
        assert_eq!(
            VirtualDevice::write(&mut hw, 0, &[1]),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::read(&mut hw, 0, &mut buf),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::ioctl(&mut hw, 0, IOCTL_FLUSH, 0),
            Err(DeviceError::NotOpen)
        );
        assert_eq!(
            VirtualDevice::open(&mut hw, 8),
            Err(DeviceError::InvalidPartition)
        );
        assert_eq!(
            VirtualDevice::close(&mut hw, 255),
            Err(DeviceError::InvalidPartition)
        );
        VirtualDevice::open(&mut hw, 0).unwrap();
        assert_eq!(
            VirtualDevice::ioctl(&mut hw, 0, 0xFF, 0),
            Err(DeviceError::NotFound)
        );
    }

    #[test]
    fn tx_buffer_full_returns_partial_count() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        let n = VirtualDevice::write(&mut hw, 0, &[0xAAu8; CAPACITY]).unwrap();
        assert_eq!(n, CAPACITY);
        assert_eq!(VirtualDevice::write(&mut hw, 0, &[0x01]).unwrap(), 0);
    }

    // ---- ISR top-half / bottom-half drain tests ----

    #[test]
    fn isr_rx_to_ring_stops_when_rxfe() {
        let mut hw = make_backend(1);
        // Set RXFE in the mock FR register — hardware FIFO is empty.
        hw.regs()
            .write(crate::uart_hal::FR, crate::uart_hal::FR_RXFE);
        let n = hw.isr_rx_to_ring();
        assert_eq!(n, 0);
        assert_eq!(hw.rx_len(), 0);
    }

    #[test]
    fn isr_rx_to_ring_fills_until_ring_full() {
        let mut hw = make_backend(1);
        // FR = 0 → RXFE clear, so read_byte() will keep returning DR value.
        // DR = 0x42, so each read returns 0x42 until ring is full.
        hw.regs().write(crate::uart_hal::DR, 0x42);
        let n = hw.isr_rx_to_ring();
        assert_eq!(n, CAPACITY);
        assert_eq!(hw.rx_len(), CAPACITY);
        // Verify all bytes are 0x42.
        let mut buf = [0u8; CAPACITY];
        VirtualDevice::open(&mut hw, 0).unwrap();
        let read = VirtualDevice::read(&mut hw, 0, &mut buf).unwrap();
        assert_eq!(read, CAPACITY);
        assert!(buf.iter().all(|&b| b == 0x42));
    }

    #[test]
    fn isr_rx_to_ring_partial_fill() {
        let mut hw = make_backend(1);
        // Pre-fill RX ring with CAPACITY - 3 bytes, leaving 3 slots.
        for _ in 0..(CAPACITY - 3) {
            hw.push_rx_from_isr(&[0xFF]);
        }
        assert_eq!(hw.rx_len(), CAPACITY - 3);
        // DR = 0xAB, RXFE clear → reads until ring full (3 more).
        hw.regs().write(crate::uart_hal::DR, 0xAB);
        let n = hw.isr_rx_to_ring();
        assert_eq!(n, 3);
        assert_eq!(hw.rx_len(), CAPACITY);
    }

    #[test]
    fn drain_tx_to_hw_stops_when_txff() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        VirtualDevice::write(&mut hw, 0, &[1, 2, 3]).unwrap();
        // Set TXFF in mock FR register — hardware FIFO is full.
        hw.regs()
            .write(crate::uart_hal::FR, crate::uart_hal::FR_TXFF);
        let n = hw.drain_tx_to_hw();
        assert_eq!(n, 0);
        // TX ring should still have all 3 bytes.
        assert_eq!(hw.tx_len(), 3);
    }

    #[test]
    fn drain_tx_to_hw_drains_all() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        VirtualDevice::write(&mut hw, 0, &[0xAA, 0xBB, 0xCC]).unwrap();
        // FR = 0 → TXFF clear, so writes proceed.
        let n = hw.drain_tx_to_hw();
        assert_eq!(n, 3);
        assert_eq!(hw.tx_len(), 0);
        // Last byte written to DR should be 0xCC.
        assert_eq!(hw.regs().read(crate::uart_hal::DR), 0xCC);
    }

    #[test]
    fn drain_tx_to_hw_empty_ring() {
        let mut hw = make_backend(1);
        // TX ring is empty — nothing to drain.
        let n = hw.drain_tx_to_hw();
        assert_eq!(n, 0);
    }

    #[test]
    fn push_rx_from_isr_basic() {
        let mut hw = make_backend(1);
        let n = hw.push_rx_from_isr(&[10, 20, 30]);
        assert_eq!(n, 3);
        assert_eq!(hw.rx_len(), 3);
        // Verify FIFO order via VirtualDevice read.
        VirtualDevice::open(&mut hw, 0).unwrap();
        let mut buf = [0u8; 4];
        let read = VirtualDevice::read(&mut hw, 0, &mut buf).unwrap();
        assert_eq!(read, 3);
        assert_eq!(&buf[..3], &[10, 20, 30]);
    }

    #[test]
    fn push_rx_from_isr_stops_when_full() {
        let mut hw = make_backend(1);
        // Fill to capacity.
        let n = hw.push_rx_from_isr(&[0xFFu8; CAPACITY]);
        assert_eq!(n, CAPACITY);
        // One more byte should not fit.
        let n = hw.push_rx_from_isr(&[0x01]);
        assert_eq!(n, 0);
        assert_eq!(hw.rx_len(), CAPACITY);
    }

    #[test]
    fn full_isr_roundtrip_tx_drain_then_rx_fill() {
        let mut hw = make_backend(1);
        VirtualDevice::open(&mut hw, 0).unwrap();
        // Partition writes 4 bytes into TX ring.
        VirtualDevice::write(&mut hw, 0, &[1, 2, 3, 4]).unwrap();
        assert_eq!(hw.tx_len(), 4);
        // Bottom-half drains TX to hardware.
        let drained = hw.drain_tx_to_hw();
        assert_eq!(drained, 4);
        assert_eq!(hw.tx_len(), 0);
        // ISR injects 3 bytes into RX ring.
        let injected = hw.push_rx_from_isr(&[0xA, 0xB, 0xC]);
        assert_eq!(injected, 3);
        // Partition reads RX.
        let mut buf = [0u8; 4];
        let read = VirtualDevice::read(&mut hw, 0, &mut buf).unwrap();
        assert_eq!(read, 3);
        assert_eq!(&buf[..3], &[0xA, 0xB, 0xC]);
    }
}
