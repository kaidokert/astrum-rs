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

    /// Set the tick count to an arbitrary value (crate-internal, for testing).
    #[cfg(test)]
    pub(crate) fn set(&mut self, value: u64) {
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
/// 1. Applies the static MPU regions (R0-R2) via `apply_partition_mpu`.
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
            // Program static regions R0-R2.
            crate::mpu::apply_partition_mpu(mpu, pcb);

            // Prepare dynamic R4 from the partition's data region.
            // partition_mpu_regions returns [bg(R0), code(R1), data(R2)];
            // the data region at index 2 is the one destined for R4.
            if let Some(regions) = crate::mpu::partition_mpu_regions(pcb) {
                let _ = strategy.configure_partition(pid, &regions[2..]);
            }
            // NOTE: program_regions() is NOT called here — the actual
            // hardware write happens in PendSV (define_pendsv_dynamic!).
        }
    }
    event
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
            if let Some(regions) = crate::mpu::partition_mpu_regions(pcb) {
                // partition_mpu_regions returns [bg(R0), code(R1), data(R2)];
                // the data region at index 2 is the one destined for R4.
                let _ = strategy.configure_partition(pid, &regions[2..]);
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
    }
}
