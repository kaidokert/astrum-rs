//! Hardware UART backend with TX/RX ring buffers and `UartRegs` integration.
//!
//! `HwUartBackend` wraps a [`UartRegs`] hardware accessor alongside
//! software TX and RX ring buffers, implementing the [`VirtualDevice`]
//! trait for partition-aware access. Single-core Cortex-M only.

use heapless::Deque;

use crate::split_isr::IsrRingBuffer;
use crate::uart_hal::UartRegs;
use crate::virtual_device::{DeviceError, VirtualDevice};

/// UART1 interrupt number on the LM3S6965 (NVIC IRQ 6).
pub const UART1_IRQ: u8 = 6;

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

    /// Drain bytes from the TX ring buffer into `buf`.
    ///
    /// Pops bytes until the TX ring is empty or `buf` is full.
    /// Returns the number of bytes transferred. Useful for software
    /// loopback where the caller wants to capture TX data without
    /// writing to hardware.
    pub fn drain_tx(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        for slot in buf.iter_mut() {
            match self.tx.pop_front() {
                Some(b) => {
                    *slot = b;
                    count += 1;
                }
                None => break,
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

/// UART1 ISR top-half: check interrupt source, drain RX FIFO, clear
/// interrupt, and push a notification into the ISR ring buffer.
///
/// Returns `true` if an RX interrupt was pending and handled, `false`
/// if no RX interrupt was active (nothing done).
///
/// This is a free function so examples can call it directly from their
/// interrupt vector table entry.
pub fn uart1_isr_top_half<const D: usize, const M: usize>(
    backend: &mut HwUartBackend,
    isr_ring: &mut IsrRingBuffer<D, M>,
) -> bool {
    let ris = backend.regs.read_ris();
    let rx_mask = UartRegs::rx_ris_mask();

    // No RX-related interrupt pending — nothing to do.
    if ris & rx_mask == 0 {
        return false;
    }

    // Drain hardware RX FIFO into the software ring buffer.
    backend.isr_rx_to_ring();

    // Clear the RX interrupt sources at the UART level.
    backend.regs.clear_rx_interrupt();

    // Notify the bottom-half via the ISR ring buffer.
    // Tag = device_id so the bottom-half can route the notification.
    // Payload is the actual RX-related interrupt sources that fired.
    let actual_sources = (ris & rx_mask) as u8;
    let _ = isr_ring.push_from_isr(backend.device_id, &[actual_sources]);

    true
}

/// Newtype wrapper implementing `InterruptNumber` for UART1 IRQ.
#[cfg(not(test))]
#[derive(Copy, Clone)]
struct Uart1Irq;

#[cfg(not(test))]
// SAFETY: UART1_IRQ (6) is a valid interrupt number for LM3S6965.
unsafe impl cortex_m::interrupt::InterruptNumber for Uart1Irq {
    fn number(self) -> u16 {
        UART1_IRQ as u16
    }
}

/// Enable UART1 interrupt in the NVIC.
///
/// # Safety
///
/// The caller must ensure this is called with interrupts properly
/// configured and the NVIC peripheral is valid.
#[cfg(not(test))]
pub fn enable_uart1_irq(nvic: &mut cortex_m::peripheral::NVIC) {
    // SAFETY: UART1 IRQ number is correct for LM3S6965 and the caller
    // provides a valid NVIC reference.
    unsafe {
        nvic.set_priority(Uart1Irq, 0x40);
        cortex_m::peripheral::NVIC::unmask(Uart1Irq);
    }
}

/// Disable UART1 interrupt in the NVIC.
#[cfg(not(test))]
pub fn disable_uart1_irq() {
    cortex_m::peripheral::NVIC::mask(Uart1Irq);
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

    // ---- UART1 IRQ constant ----

    #[test]
    fn uart1_irq_is_six() {
        assert_eq!(UART1_IRQ, 6);
    }

    // ---- uart1_isr_top_half tests ----

    #[test]
    fn isr_top_half_no_rx_interrupt_returns_false() {
        let mut hw = make_backend(2);
        let mut ring = IsrRingBuffer::<4, 8>::new();
        // RIS = 0 → no RX interrupt pending.
        hw.regs().write(crate::uart_hal::RIS, 0);
        let handled = uart1_isr_top_half(&mut hw, &mut ring);
        assert!(!handled);
        assert!(ring.is_empty());
        assert_eq!(hw.rx_len(), 0);
    }

    #[test]
    fn isr_top_half_rxris_drains_and_clears() {
        let mut hw = make_backend(2);
        let mut ring = IsrRingBuffer::<4, 8>::new();
        // Set RXRIS in RIS → RX interrupt pending.
        hw.regs()
            .write(crate::uart_hal::RIS, crate::uart_hal::RIS_RXRIS);
        // Put data in the hardware FIFO (DR=0x42, FR=0 means RXFE clear).
        hw.regs().write(crate::uart_hal::DR, 0x42);
        let handled = uart1_isr_top_half(&mut hw, &mut ring);
        assert!(handled);
        // RX ring should have data (CAPACITY bytes, since mock RXFE stays 0).
        assert_eq!(hw.rx_len(), CAPACITY);
        // ICR should have the clear bits written.
        let icr = hw.regs().read(crate::uart_hal::ICR);
        assert_ne!(icr & crate::uart_hal::ICR_RXIC, 0);
        assert_ne!(icr & crate::uart_hal::ICR_RTIC, 0);
        // ISR ring should have one notification with the device ID tag
        // and the actual interrupt source as payload.
        assert_eq!(ring.len(), 1);
        ring.pop_with(|tag, payload| {
            assert_eq!(tag, 2); // device_id = 2
            assert_eq!(payload, &[crate::uart_hal::RIS_RXRIS as u8]);
        });
    }

    #[test]
    fn isr_top_half_rtris_also_handled() {
        let mut hw = make_backend(3);
        let mut ring = IsrRingBuffer::<4, 8>::new();
        // Only RTRIS set (receive timeout, no RXRIS).
        hw.regs()
            .write(crate::uart_hal::RIS, crate::uart_hal::RIS_RTRIS);
        // RXFE set → no data to drain, but interrupt still handled.
        hw.regs()
            .write(crate::uart_hal::FR, crate::uart_hal::FR_RXFE);
        let handled = uart1_isr_top_half(&mut hw, &mut ring);
        assert!(handled);
        // No data drained (RXFE), but interrupt cleared and notification pushed.
        assert_eq!(hw.rx_len(), 0);
        assert_eq!(ring.len(), 1);
        ring.pop_with(|tag, payload| {
            assert_eq!(tag, 3); // device_id = 3
                                // Payload should reflect RTRIS, not the hardcoded RXRIS.
            assert_eq!(payload, &[crate::uart_hal::RIS_RTRIS as u8]);
        });
    }

    #[test]
    fn isr_top_half_unrelated_interrupt_ignored() {
        let mut hw = make_backend(1);
        let mut ring = IsrRingBuffer::<4, 8>::new();
        // Set a non-RX bit in RIS (e.g. bit 0 — not RXRIS or RTRIS).
        hw.regs().write(crate::uart_hal::RIS, 1 << 0);
        let handled = uart1_isr_top_half(&mut hw, &mut ring);
        assert!(!handled);
        assert!(ring.is_empty());
    }

    #[test]
    fn isr_top_half_full_ring_still_clears_interrupt() {
        let mut hw = make_backend(1);
        let mut ring = IsrRingBuffer::<1, 8>::new();
        // Fill the ISR ring so the notification push will fail.
        ring.push_from_isr(0, &[0]).unwrap();
        assert!(ring.is_full());
        // Set RX interrupt and RXFE (no data to drain).
        hw.regs()
            .write(crate::uart_hal::RIS, crate::uart_hal::RIS_RXRIS);
        hw.regs()
            .write(crate::uart_hal::FR, crate::uart_hal::FR_RXFE);
        let handled = uart1_isr_top_half(&mut hw, &mut ring);
        assert!(handled);
        // Interrupt is still cleared even though ring push failed.
        let icr = hw.regs().read(crate::uart_hal::ICR);
        assert_ne!(icr & crate::uart_hal::ICR_RXIC, 0);
    }
}
