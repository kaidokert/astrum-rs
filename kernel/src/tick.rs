//! SysTick-driven scheduler tick and PendSV trigger.

use crate::kernel::KernelState;

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
    ///
    /// Used by the SysTick handler to copy the authoritative tick from
    /// `KernelState` into `Kernel` each tick.
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

/// Advance the schedule table by one tick. On partition switch, sets PendSV
/// pending and returns the new active partition id.
#[cfg(not(feature = "dynamic-mpu"))]
pub fn on_systick<const P: usize, const S: usize>(state: &mut KernelState<P, S>) -> Option<u8> {
    let next = state.advance_schedule_tick();
    if next.is_some() {
        #[cfg(not(test))]
        cortex_m::peripheral::SCB::set_pendsv();
    }
    next
}

/// Advance the schedule table by one tick. Returns a
/// [`ScheduleEvent`](crate::scheduler::ScheduleEvent) indicating what
/// happened (partition switch, system window, or nothing).
#[cfg(feature = "dynamic-mpu")]
pub fn on_systick<const P: usize, const S: usize>(
    state: &mut KernelState<P, S>,
) -> crate::scheduler::ScheduleEvent {
    let event = state.advance_schedule_tick();
    if matches!(event, crate::scheduler::ScheduleEvent::PartitionSwitch(_)) {
        #[cfg(not(test))]
        cortex_m::peripheral::SCB::set_pendsv();
    }
    event
}

/// Like [`on_systick`], but also reconfigures the MPU for the new partition
/// before PendSV fires. Call this from the SysTick handler when MPU
/// isolation is required.
#[cfg(all(not(test), not(feature = "dynamic-mpu")))]
pub fn on_systick_mpu<const P: usize, const S: usize>(
    state: &mut KernelState<P, S>,
    mpu: &cortex_m::peripheral::MPU,
) -> Option<u8> {
    let next = on_systick(state);
    if let Some(pid) = next {
        if let Some(pcb) = state.partitions().get(pid as usize) {
            crate::mpu::apply_partition_mpu(mpu, pcb);
        }
    }
    next
}

/// Like [`on_systick_mpu`], but also prepares dynamic MPU regions R4-R7
/// via a [`DynamicStrategy`](crate::mpu_strategy::DynamicStrategy).
///
/// On each partition switch this function:
/// 1. Applies the static MPU regions (R0-R3) via `apply_partition_mpu`.
/// 2. Calls `DynamicStrategy::configure_partition` with the new partition's
///    data region so R4 tracks its private RAM.
///
/// **Hardware programming** of R4-R7 is deferred to the PendSV handler
/// (via [`define_pendsv_dynamic!`](crate::define_pendsv_dynamic)).  This
/// avoids a race condition where code executes with a stale memory map
/// between SysTick and PendSV.
#[cfg(all(not(test), feature = "dynamic-mpu"))]
pub fn on_systick_dynamic<const P: usize, const S: usize>(
    state: &mut KernelState<P, S>,
    mpu: &cortex_m::peripheral::MPU,
    strategy: &crate::mpu_strategy::DynamicStrategy,
) -> crate::scheduler::ScheduleEvent {
    use crate::mpu_strategy::MpuStrategy;
    use crate::scheduler::ScheduleEvent;
    let event = on_systick(state);
    if let ScheduleEvent::PartitionSwitch(pid) = event {
        if let Some(pcb) = state.partitions().get(pid as usize) {
            // Program static regions R0-R3.
            crate::mpu::apply_partition_mpu(mpu, pcb);

            // Prepare dynamic R4 from the partition's data region.
            if let Some(regions) = crate::mpu::partition_dynamic_regions(pcb) {
                let _ = strategy.configure_partition(pid, &regions);
            }
            // NOTE: program_regions() is NOT called here — the actual
            // hardware write happens in PendSV (define_pendsv_dynamic!).
        }
    }
    event
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
///
/// Called by the SysTick handler (or test harness) when
/// [`on_systick`] returns [`ScheduleEvent::SystemWindow`].  Performs
/// deferred kernel work that must not run during partition time slots:
///
/// 1. Routes bytes between paired virtual UARTs via
///    [`VirtualUartPair::transfer`].
/// 2. Drains pending ISR ring-buffer entries, routing each record's
///    payload to the UART backend identified by `tag` (device ID).
///    Entries tagged with the hardware UART's device ID are routed to
///    its RX buffer via [`HwUartBackend::push_rx_from_isr`].
/// 3. If `hw_uart` is `Some`, calls
///    [`HwUartBackend::drain_tx_to_hw`] to flush the TX ring buffer
///    to hardware.
/// 4. Revokes expired buffer pool borrows whose deadline has elapsed.
///
/// Returns a [`BottomHalfResult`] with transfer counts and `has_rx_data`.
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

    // Check whether any device has pending RX data for the caller.
    let has_rx_data = uart_pair.a.rx_len() > 0
        || uart_pair.b.rx_len() > 0
        || hw_uart.as_ref().is_some_and(|hw| hw.rx_len() > 0);

    BottomHalfResult {
        a_to_b,
        b_to_a,
        has_rx_data,
    }
}

/// Test-mode helper: simulates [`on_systick_dynamic`] without hardware.
#[cfg(all(test, feature = "dynamic-mpu"))]
pub fn on_systick_dynamic_test<const P: usize, const S: usize>(
    state: &mut KernelState<P, S>,
    strategy: &crate::mpu_strategy::DynamicStrategy,
) -> crate::scheduler::ScheduleEvent {
    use crate::mpu_strategy::MpuStrategy;
    use crate::scheduler::ScheduleEvent;
    let event = on_systick(state);
    if let ScheduleEvent::PartitionSwitch(pid) = event {
        if let Some(pcb) = state.partitions().get(pid as usize) {
            if let Some(regions) = crate::mpu::partition_dynamic_regions(pcb) {
                let _ = strategy.configure_partition(pid, &regions);
            }
        }
    }
    event
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionConfig};
    use crate::scheduler::{ScheduleEntry, ScheduleTable};

    #[cfg(not(feature = "dynamic-mpu"))]
    fn make_state() -> KernelState<4, 8> {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 3)).unwrap();
        s.add(ScheduleEntry::new(1, 2)).unwrap();
        s.start();
        let r = MpuRegion::new(0, 4096, 0);
        KernelState::new(
            s,
            &[
                PartitionConfig {
                    id: 0,
                    entry_point: 0x1000,
                    stack_base: 0x2000_0000,
                    stack_size: 1024,
                    mpu_region: r,
                },
                PartitionConfig {
                    id: 1,
                    entry_point: 0x2000,
                    stack_base: 0x2000_1000,
                    stack_size: 1024,
                    mpu_region: r,
                },
            ],
        )
        .unwrap()
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn tick_drives_partition_switches() {
        let mut st = make_state();
        assert_eq!((on_systick(&mut st), st.active_partition()), (None, None));
        assert_eq!(on_systick(&mut st), None);
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (Some(1), Some(1))
        );
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (None, Some(1))
        );
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (Some(0), Some(0))
        );
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn multiple_major_frames() {
        let mut st = make_state();
        let mut sw = heapless::Vec::<u8, 8>::new();
        for _ in 0..10 {
            if let Some(p) = on_systick(&mut st) {
                sw.push(p).unwrap();
            }
        }
        assert_eq!(sw.as_slice(), &[1, 0, 1, 0]);
    }

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
        use super::*;
        use crate::mpu::RBAR_ADDR_MASK;
        use crate::mpu_strategy::DynamicStrategy;
        use crate::scheduler::ScheduleEvent;

        fn dyn_state() -> KernelState<4, 8> {
            let mut s = ScheduleTable::new();
            s.add(ScheduleEntry::new(0, 2)).unwrap();
            s.add(ScheduleEntry::new(1, 2)).unwrap();
            s.start();
            let pc = |i: u8, base: u32| PartitionConfig {
                id: i,
                entry_point: 0,
                stack_base: base,
                stack_size: 4096,
                mpu_region: MpuRegion::new(base, 4096, 0),
            };
            KernelState::new(s, &[pc(0, 0x2000_0000), pc(1, 0x2000_8000)]).unwrap()
        }

        fn tick(st: &mut KernelState<4, 8>, ds: &DynamicStrategy) -> ScheduleEvent {
            crate::tick::on_systick_dynamic_test(st, ds)
        }

        #[test]
        fn configures_r4_on_switch() {
            let (mut st, ds) = (dyn_state(), DynamicStrategy::new());
            assert_eq!(tick(&mut st, &ds), ScheduleEvent::None);
            assert_eq!(tick(&mut st, &ds), ScheduleEvent::PartitionSwitch(1));
            let d = ds.slot(4).unwrap();
            assert_eq!((d.base, d.owner), (0x2000_8000, 1));
            assert_eq!(tick(&mut st, &ds), ScheduleEvent::None); // still p1's slot
            assert_eq!(tick(&mut st, &ds), ScheduleEvent::PartitionSwitch(0));
            let d = ds.slot(4).unwrap();
            assert_eq!((d.base, d.owner), (0x2000_0000, 0));
        }

        #[test]
        fn compute_region_values_after_configure() {
            let (mut st, ds) = (dyn_state(), DynamicStrategy::new());
            tick(&mut st, &ds);
            tick(&mut st, &ds); // switch to p1
            let vals = ds.compute_region_values();
            assert_eq!(vals[0].0 & RBAR_ADDR_MASK, 0x2000_8000);
            assert_ne!(vals[0].1, 0);
            for &(_, rasr) in &vals[1..] {
                assert_eq!(rasr, 0);
            }
        }

        #[test]
        fn no_switch_leaves_strategy_empty() {
            let (mut st, ds) = (dyn_state(), DynamicStrategy::new());
            assert_eq!(tick(&mut st, &ds), ScheduleEvent::None);
            assert!(ds.slot(4).is_none());
        }

        #[test]
        fn multiple_frames_alternate_r4() {
            let (mut st, ds) = (dyn_state(), DynamicStrategy::new());
            let mut owners = heapless::Vec::<u8, 8>::new();
            for _ in 0..8 {
                if let ScheduleEvent::PartitionSwitch(pid) = tick(&mut st, &ds) {
                    assert_eq!(ds.slot(4).unwrap().owner, pid);
                    owners.push(pid).unwrap();
                }
            }
            assert_eq!(owners.as_slice(), &[1, 0, 1, 0]);
        }

        #[test]
        fn r4_holds_data_region_not_guard() {
            // After a partition switch, R4 should hold the partition's
            // data region (RW, full-access) — not the 32-byte no-access
            // stack guard that occupies Region 3.
            let (mut st, ds) = (dyn_state(), DynamicStrategy::new());
            tick(&mut st, &ds); // tick 0: None
            tick(&mut st, &ds); // tick 1: switch to P1

            let desc = ds.slot(4).expect("R4 should be occupied");
            // P1's data region is 4096 bytes at 0x2000_8000.
            assert_eq!(desc.base, 0x2000_8000);
            assert_eq!(desc.size, 4096);
            // The guard is only 32 bytes — verify R4 is not the guard.
            assert_ne!(desc.size, 32);
            // RASR should have AP = FULL_ACCESS (bits [26:24] = 0b011).
            let ap = (desc.permissions >> 24) & 0x7;
            assert_eq!(ap, crate::mpu::AP_FULL_ACCESS);
        }

        // ---- Bottom-half processing tests ----

        use crate::buffer_pool::BufferPool;
        use crate::hw_uart::HwUartBackend;
        use crate::split_isr::IsrRingBuffer;
        use crate::uart_hal::UartRegs;
        use crate::virtual_device::VirtualDevice;
        use crate::virtual_uart::VirtualUartPair;

        /// Helper: build a schedule with a system window between two
        /// partition slots so we can observe the event sequence.
        fn dyn_state_with_syswin() -> KernelState<4, 8> {
            let mut s = ScheduleTable::new();
            s.add(ScheduleEntry::new(0, 2)).unwrap();
            s.add_system_window(1).unwrap();
            s.add(ScheduleEntry::new(1, 2)).unwrap();
            s.start();
            let pc = |i: u8, base: u32| PartitionConfig {
                id: i,
                entry_point: 0,
                stack_base: base,
                stack_size: 4096,
                mpu_region: MpuRegion::new(base, 4096, 0),
            };
            KernelState::new(s, &[pc(0, 0x2000_0000), pc(1, 0x2000_8000)]).unwrap()
        }

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
        fn run_bottom_half_drains_isr_ring() {
            // UART pair with device IDs 0 and 1.
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();
            // Push ISR records tagged with device IDs 0 and 1.
            ring.push_from_isr(1, &[10]).unwrap();
            ring.push_from_isr(0, &[20, 30]).unwrap();

            crate::tick::run_bottom_half(&mut pair, &mut ring, &mut buffers, &mut hw_uart, 0, &ds);
            assert!(ring.is_empty());
            // Tag 1 → UART-B RX, tag 0 → UART-A RX.
            let mut buf = [0u8; 4];
            assert_eq!(pair.b.pop_rx(&mut buf), 1);
            assert_eq!(buf[0], 10);
            assert_eq!(pair.a.pop_rx(&mut buf), 2);
            assert_eq!(&buf[..2], &[20, 30]);
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
        fn system_window_tick_triggers_bottom_half() {
            let mut st = dyn_state_with_syswin();
            let ds = DynamicStrategy::new();
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();

            // Stage data *before* ticking — the bottom-half should
            // process it when the system window fires.
            pair.a.push_tx(&[0x42]);
            ring.push_from_isr(1, &[0xFF]).unwrap();

            // Tick through a full major frame.  Invoke run_bottom_half
            // on SystemWindow events (mirroring what _harness_handle_tick
            // does) and verify its side effects.
            let mut saw_system_window = false;
            for _ in 0..5 {
                let event = on_systick_dynamic_test(&mut st, &ds);
                if matches!(event, ScheduleEvent::SystemWindow) {
                    crate::tick::run_bottom_half(
                        &mut pair,
                        &mut ring,
                        &mut buffers,
                        &mut None,
                        0,
                        &ds,
                    );
                    saw_system_window = true;
                }
            }
            assert!(saw_system_window);
            // Side effects: UART-A TX (0x42) transferred to UART-B RX,
            // then ISR record (tag=1, payload=0xFF) also routed to UART-B RX.
            let mut buf = [0u8; 4];
            assert_eq!(pair.b.pop_rx(&mut buf), 2);
            assert_eq!(&buf[..2], &[0x42, 0xFF]);
            // ISR ring is drained.
            assert!(ring.is_empty());
        }

        #[test]
        fn partition_tick_skips_bottom_half() {
            let mut st = dyn_state_with_syswin();
            let ds = DynamicStrategy::new();

            let mut partition_events = 0u32;
            let mut system_events = 0u32;

            // Tick through two full major frames (10 ticks).
            for _ in 0..10 {
                let event = on_systick_dynamic_test(&mut st, &ds);
                match event {
                    ScheduleEvent::PartitionSwitch(_) => partition_events += 1,
                    ScheduleEvent::SystemWindow => system_events += 1,
                    ScheduleEvent::None => {}
                }
            }
            // Each major frame: P0(2) -> SysWin(1) -> P1(2) = 5 ticks
            // Two frames = 10 ticks, 2 partition switches + 1 syswin per frame
            assert_eq!(partition_events, 4); // P0, P1 per frame x2
            assert_eq!(system_events, 2);
        }

        // ---- hw_uart bottom-half tests ----

        #[test]
        fn run_bottom_half_drains_hw_uart_tx() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw = HwUartBackend::new(5, UartRegs::new(0x4000_C000));
            // Manually push into TX via VirtualDevice::write after opening.
            hw.open(0).unwrap();
            hw.write(0, &[0xAA, 0xBB, 0xCC]).unwrap();
            assert_eq!(hw.tx_len(), 3);

            let mut hw_uart = Some(hw);
            let ds = DynamicStrategy::new();
            crate::tick::run_bottom_half(&mut pair, &mut ring, &mut buffers, &mut hw_uart, 0, &ds);

            // TX ring should be drained to hardware.
            assert_eq!(hw_uart.as_ref().unwrap().tx_len(), 0);
        }

        #[test]
        fn run_bottom_half_routes_isr_to_hw_uart_rx() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let hw = HwUartBackend::new(5, UartRegs::new(0x4000_C000));
            let mut hw_uart = Some(hw);

            // Push ISR records: one for virtual UART-A (id=0), one for
            // hw_uart (id=5).
            ring.push_from_isr(0, &[0x10]).unwrap();
            ring.push_from_isr(5, &[0x20, 0x30]).unwrap();

            let ds = DynamicStrategy::new();
            crate::tick::run_bottom_half(&mut pair, &mut ring, &mut buffers, &mut hw_uart, 0, &ds);

            assert!(ring.is_empty());
            // Virtual UART-A should have received tag-0 data.
            let mut buf = [0u8; 4];
            assert_eq!(pair.a.pop_rx(&mut buf), 1);
            assert_eq!(buf[0], 0x10);
            // hw_uart should have received tag-5 data via push_rx_from_isr.
            assert_eq!(hw_uart.as_ref().unwrap().rx_len(), 2);
            hw_uart.as_mut().unwrap().open(0).unwrap();
            let mut hw_buf = [0u8; 4];
            let n = hw_uart.as_mut().unwrap().read(0, &mut hw_buf).unwrap();
            assert_eq!(n, 2);
            assert_eq!(&hw_buf[..2], &[0x20, 0x30]);
        }

        #[test]
        fn run_bottom_half_hw_uart_loopback_copies_tx_to_rx() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw = HwUartBackend::new(5, UartRegs::new(0x4000_C000));
            hw.set_loopback(true);
            hw.open(0).unwrap();
            hw.write(0, &[0xCA, 0xFE]).unwrap();
            assert_eq!(hw.tx_len(), 2);
            assert_eq!(hw.rx_len(), 0);

            let mut hw_uart = Some(hw);
            let ds = DynamicStrategy::new();
            crate::tick::run_bottom_half(&mut pair, &mut ring, &mut buffers, &mut hw_uart, 0, &ds);

            let hw = hw_uart.as_mut().unwrap();
            // TX drained, bytes looped back into RX.
            assert_eq!(hw.tx_len(), 0);
            assert_eq!(hw.rx_len(), 2);
            let mut buf = [0u8; 4];
            let n = hw.read(0, &mut buf).unwrap();
            assert_eq!(n, 2);
            assert_eq!(&buf[..2], &[0xCA, 0xFE]);
        }

        #[test]
        fn run_bottom_half_hw_uart_none_is_noop() {
            // When hw_uart is None, ISR entries for unknown tags are
            // silently dropped and no drain occurs.
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart: Option<HwUartBackend> = None;

            ring.push_from_isr(99, &[0xFF]).unwrap();
            let ds = DynamicStrategy::new();
            crate::tick::run_bottom_half(&mut pair, &mut ring, &mut buffers, &mut hw_uart, 0, &ds);
            // Ring should be drained (record consumed but tag unmatched).
            assert!(ring.is_empty());
            // No crash, no data routed.
            assert_eq!(pair.a.rx_len(), 0);
            assert_eq!(pair.b.rx_len(), 0);
        }

        // ---- buffer revocation bottom-half tests ----

        #[test]
        fn run_bottom_half_revokes_expired_buffers() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            // Lend slot 0 with deadline=50, slot 1 with deadline=200.
            buffers.lend_to_partition(0, 1, false, &ds).unwrap();
            buffers.set_deadline(0, Some(50)).unwrap();
            buffers.lend_to_partition(1, 2, true, &ds).unwrap();
            buffers.set_deadline(1, Some(200)).unwrap();

            // At tick=100, slot 0 is expired (deadline 50 <= 100), slot 1 is not.
            crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                100,
                &ds,
            );

            // Slot 0: revoked (free, MPU window removed, deadline cleared).
            assert_eq!(
                buffers.get(0).unwrap().state(),
                crate::buffer_pool::BorrowState::Free,
            );
            assert!(ds.slot(5).is_none());
            assert_eq!(buffers.deadline(0), None);

            // Slot 1: still active (deadline 200 > 100).
            assert_eq!(
                buffers.get(1).unwrap().state(),
                crate::buffer_pool::BorrowState::BorrowedWrite { owner: 2 },
            );
            assert!(ds.slot(6).is_some());
            assert_eq!(buffers.deadline(1), Some(200));
        }

        #[test]
        fn run_bottom_half_no_deadline_not_revoked() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            // Lend slot 0 with no deadline.
            buffers.lend_to_partition(0, 1, true, &ds).unwrap();

            // Even at a very high tick, the slot should not be revoked.
            crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                u64::MAX,
                &ds,
            );

            assert_eq!(
                buffers.get(0).unwrap().state(),
                crate::buffer_pool::BorrowState::BorrowedWrite { owner: 1 },
            );
            assert!(ds.slot(5).is_some());
        }

        // ---- device reader wake-up bottom-half tests ----

        #[test]
        fn run_bottom_half_has_rx_data_set_when_data_arrives() {
            let mut pair = VirtualUartPair::new(0, 1);
            let mut ring = IsrRingBuffer::<4, 8>::new();
            let mut buffers = BufferPool::<4, 32>::new();
            let mut hw_uart = None;
            let ds = DynamicStrategy::new();

            // Push data into UART-A TX so transfer delivers it to UART-B RX.
            pair.a.push_tx(&[0x42]);

            let bh = crate::tick::run_bottom_half(
                &mut pair,
                &mut ring,
                &mut buffers,
                &mut hw_uart,
                10,
                &ds,
            );

            // Data arrived in UART-B RX → has_rx_data should be true.
            assert!(bh.has_rx_data);
        }

        #[test]
        fn run_bottom_half_has_rx_data_false_when_no_data() {
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
                10,
                &ds,
            );

            // No data arrived → has_rx_data should be false.
            assert!(!bh.has_rx_data);
        }
    }
}
