//! SysTick-driven scheduler tick and PendSV trigger.
//!
//! This module provides the [`TickCounter`] for monotonic tick counting and
//! [`configure_systick`] for SysTick timer configuration. The schedule
//! advancement is now handled directly via [`Kernel::advance_schedule_tick`](crate::svc::Kernel::advance_schedule_tick).

/// Monotonic tick counter incremented on every SysTick interrupt.
pub struct TickCounter {
    ticks: u64,
}

impl Default for TickCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl TickCounter {
    /// Create a new counter starting at zero.
    pub const fn new() -> Self {
        Self { ticks: 0 }
    }

    /// Increment the counter by one tick.
    pub fn increment(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
    }

    /// Return the current tick count.
    pub fn get(&self) -> u64 {
        self.ticks
    }

    /// Synchronize the tick counter to the given value.
    pub fn sync(&mut self, value: u64) {
        self.ticks = value;
    }
}

/// Configure the SysTick timer with the given reload value.
#[cfg(not(test))]
pub fn configure_systick(syst: &mut cortex_m::peripheral::SYST, reload: u32) {
    use cortex_m::peripheral::syst::SystClkSource;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(reload);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

/// Result of bottom-half processing.
#[cfg(feature = "dynamic-mpu")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BottomHalfResult {
    /// Bytes transferred UART-A TX → UART-B RX.
    pub a_to_b: usize,
    /// Bytes transferred UART-B TX → UART-A RX.
    pub b_to_a: usize,
    /// `true` when at least one device has RX data and a reader should be woken.
    pub has_rx_data: bool,
}

/// Bottom-half processing for system window ticks.
#[cfg(feature = "dynamic-mpu")]
pub fn run_bottom_half<const D: usize, const M: usize, const BP: usize, const BZ: usize>(
    uart_pair: &mut crate::virtual_uart::VirtualUartPair,
    isr_ring: &mut crate::split_isr::IsrRingBuffer<D, M>,
    buffers: &mut crate::buffer_pool::BufferPool<BP, BZ>,
    hw_uart: &mut Option<crate::hw_uart::HwUartBackend>,
    current_tick: u64,
    strategy: &dyn crate::mpu_strategy::MpuStrategy,
) -> BottomHalfResult {
    use crate::virtual_device::VirtualDevice;
    let (a_to_b, b_to_a) = uart_pair.transfer();
    let a_id = uart_pair.a.device_id();
    let b_id = uart_pair.b.device_id();
    while isr_ring.pop_with(|tag, data| {
        if tag == a_id {
            uart_pair.a.push_rx(data);
        } else if tag == b_id {
            uart_pair.b.push_rx(data);
        } else if let Some(hw) = hw_uart.as_mut() {
            if hw.device_id() == tag {
                hw.push_rx_from_isr(data);
            }
        }
    }) {}
    if let Some(hw) = hw_uart.as_mut() {
        hw.drain_tx_to_hw();
    }
    buffers.revoke_expired(current_tick, strategy);

    let has_rx_data = uart_pair.a.rx_len() > 0
        || uart_pair.b.rx_len() > 0
        || hw_uart.as_ref().is_some_and(|hw| hw.rx_len() > 0);

    BottomHalfResult {
        a_to_b,
        b_to_a,
        has_rx_data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_counter_starts_at_zero() {
        let tc = TickCounter::new();
        assert_eq!(tc.get(), 0);
    }

    #[test]
    fn tick_counter_increments() {
        let mut tc = TickCounter::new();
        tc.increment();
        assert_eq!(tc.get(), 1);
        tc.increment();
        tc.increment();
        assert_eq!(tc.get(), 3);
    }

    #[test]
    fn tick_counter_wraps_at_max() {
        let mut tc = TickCounter { ticks: u64::MAX };
        tc.increment();
        assert_eq!(tc.get(), 0);
    }

    #[test]
    fn tick_counter_sync_sets_value() {
        let mut tc = TickCounter::new();
        tc.sync(100);
        assert_eq!(tc.get(), 100);
        tc.sync(0);
        assert_eq!(tc.get(), 0);
        tc.sync(u64::MAX);
        assert_eq!(tc.get(), u64::MAX);
    }

    #[cfg(feature = "dynamic-mpu")]
    mod dynamic_tests {
        use crate::buffer_pool::BufferPool;
        use crate::mpu_strategy::DynamicStrategy;
        use crate::split_isr::IsrRingBuffer;
        use crate::virtual_uart::VirtualUartPair;

        #[test]
        fn run_bottom_half_transfers_uart_data() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            pair.a.push_tx(&[0xAA, 0xBB]);
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                0,
                &ds,
            );
            assert_eq!(bh.a_to_b, 2);
            assert_eq!(bh.b_to_a, 0);

            let mut buf = [0u8; 4];
            assert_eq!(pair.b.pop_rx(&mut buf), 2);
            assert_eq!(&buf[..2], &[0xAA, 0xBB]);
        }

        #[test]
        fn run_bottom_half_handles_empty_inputs() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                0,
                &ds,
            );
            assert_eq!((bh.a_to_b, bh.b_to_a), (0, 0));
            assert!(!bh.has_rx_data);
        }

        #[test]
        fn run_bottom_half_has_rx_data_set_when_data_arrives() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            pair.a.push_tx(&[0x42]);
            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                10,
                &ds,
            );
            assert!(bh.has_rx_data);
        }

        #[test]
        fn run_bottom_half_revokes_expired_buffers() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            buffers.lend_to_partition(0, 1, false, &ds).unwrap();
            buffers.set_deadline(0, Some(50)).unwrap();
            buffers.lend_to_partition(1, 2, true, &ds).unwrap();
            buffers.set_deadline(1, Some(200)).unwrap();

            crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                100,
                &ds,
            );

            assert_eq!(
                buffers.get(0).unwrap().state(),
                crate::buffer_pool::BorrowState::Free,
            );
            assert_eq!(buffers.deadline(0), None);

            assert_eq!(
                buffers.get(1).unwrap().state(),
                crate::buffer_pool::BorrowState::BorrowedWrite { owner: 2 },
            );
            assert_eq!(buffers.deadline(1), Some(200));
        }
    }
}
