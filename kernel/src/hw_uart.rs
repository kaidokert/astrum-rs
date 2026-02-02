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
}
