use crate::partition::{PartitionConfig, PartitionControlBlock, PartitionTable};
#[cfg(feature = "dynamic-mpu")]
use crate::scheduler::ScheduleEvent;
use crate::scheduler::ScheduleTable;
use crate::tick::TickCounter;

/// Abstracts over the return type of `ScheduleTable::force_advance` /
/// `advance_tick`, which is `Option<u8>` without `dynamic-mpu` and
/// `ScheduleEvent` with it.  This lets `yield_current_slot` (and the
/// harness yield macro) be written once regardless of feature gate.
pub trait YieldResult {
    /// Extract the partition id when the result represents a switch.
    fn partition_id(&self) -> Option<u8>;
}

#[cfg(not(feature = "dynamic-mpu"))]
impl YieldResult for Option<u8> {
    #[inline]
    fn partition_id(&self) -> Option<u8> {
        *self
    }
}

#[cfg(feature = "dynamic-mpu")]
impl YieldResult for ScheduleEvent {
    #[inline]
    fn partition_id(&self) -> Option<u8> {
        match self {
            ScheduleEvent::PartitionSwitch(pid) => Some(*pid),
            _ => None,
        }
    }
}

pub struct KernelState<const P: usize, const S: usize> {
    partitions: PartitionTable<P>,
    schedule: ScheduleTable<S>,
    active_partition: Option<u8>,
    tick: TickCounter,
}

const fn align_down_8(addr: u32) -> u32 {
    addr & !7
}

impl<const P: usize, const S: usize> KernelState<P, S> {
    pub fn new(schedule: ScheduleTable<S>, configs: &[PartitionConfig]) -> Option<Self> {
        let mut partitions = PartitionTable::new();
        for c in configs {
            let sp = align_down_8(c.stack_base.wrapping_add(c.stack_size));
            let pcb =
                PartitionControlBlock::new(c.id, c.entry_point, c.stack_base, sp, c.mpu_region);
            if partitions.add(pcb).is_err() {
                return None;
            }
        }
        Some(Self {
            partitions,
            schedule,
            active_partition: None,
            tick: TickCounter::new(),
        })
    }

    pub fn partitions(&self) -> &PartitionTable<P> {
        &self.partitions
    }
    pub fn partitions_mut(&mut self) -> &mut PartitionTable<P> {
        &mut self.partitions
    }
    pub fn schedule(&self) -> &ScheduleTable<S> {
        &self.schedule
    }
    pub fn active_partition(&self) -> Option<u8> {
        self.active_partition
    }
    pub fn tick(&self) -> &TickCounter {
        &self.tick
    }

    /// Force-advance the schedule to the next slot, forfeiting remaining
    /// ticks.  Updates `active_partition` and returns the schedule result.
    /// Called by the harness when a partition yields.
    ///
    /// The concrete return type depends on the feature gate
    /// (`Option<u8>` or `ScheduleEvent`), but both implement
    /// [`YieldResult`] so callers can extract the partition id
    /// uniformly.
    pub fn yield_current_slot(&mut self) -> impl YieldResult {
        let result = self.schedule.force_advance();
        if let Some(pid) = result.partition_id() {
            self.active_partition = Some(pid);
        }
        result
    }

    /// Advance the schedule table by one tick. If a partition switch occurs,
    /// updates `active_partition` and returns `Some(partition_id)`.
    #[cfg(not(feature = "dynamic-mpu"))]
    pub fn advance_schedule_tick(&mut self) -> Option<u8> {
        self.tick.increment();
        let next = self.schedule.advance_tick();
        if let Some(pid) = next {
            self.active_partition = Some(pid);
        }
        next
    }

    /// Advance the schedule table by one tick. Returns a [`ScheduleEvent`]
    /// that distinguishes partition switches from system window slots.
    #[cfg(feature = "dynamic-mpu")]
    pub fn advance_schedule_tick(&mut self) -> ScheduleEvent {
        self.tick.increment();
        let event = self.schedule.advance_tick();
        if let ScheduleEvent::PartitionSwitch(pid) = event {
            self.active_partition = Some(pid);
        }
        event
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionState};
    use crate::scheduler::ScheduleEntry;

    fn pcfg(id: u8, base: u32, size: u32) -> PartitionConfig {
        PartitionConfig {
            id,
            entry_point: 0x0800_0000 + (id as u32) * 0x1000,
            stack_base: base,
            stack_size: size,
            mpu_region: MpuRegion::new(base, 4096, 0x0306_0000),
        }
    }

    fn sched2() -> ScheduleTable<4> {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 100)).unwrap();
        s.add(ScheduleEntry::new(1, 100)).unwrap();
        s
    }

    #[test]
    fn init_two_partitions() {
        let cfgs = [pcfg(0, 0x2000_0000, 1024), pcfg(1, 0x2000_1000, 2048)];
        let mut ks: KernelState<4, 4> = KernelState::new(sched2(), &cfgs).unwrap();
        assert_eq!(ks.partitions().len(), 2);
        assert_eq!(ks.active_partition(), None);
        assert_eq!(ks.tick().get(), 0);
        ks.advance_schedule_tick();
        assert_eq!(ks.tick().get(), 1);
        assert_eq!(ks.schedule().major_frame_ticks, 200);
        let p0 = ks.partitions().get(0).unwrap();
        assert_eq!(p0.id(), 0);
        assert_eq!(p0.state(), PartitionState::Ready);
        assert_eq!(p0.stack_pointer(), 0x2000_0400);
        assert_eq!(p0.stack_pointer() % 8, 0);
        let p1 = ks.partitions().get(1).unwrap();
        assert_eq!(p1.id(), 1);
        assert_eq!(p1.stack_pointer(), 0x2000_1800);
        assert_eq!(p1.stack_pointer() % 8, 0);
    }

    #[test]
    fn init_four_partitions() {
        let cfgs = [
            pcfg(0, 0x2000_0000, 1024),
            pcfg(1, 0x2000_1000, 2048),
            pcfg(2, 0x2000_2000, 512),
            pcfg(3, 0x2000_3000, 4096),
        ];
        let mut s = ScheduleTable::new();
        for i in 0..4u8 {
            s.add(ScheduleEntry::new(i, 50)).unwrap();
        }
        let ks: KernelState<4, 4> = KernelState::new(s, &cfgs).unwrap();
        assert_eq!(ks.partitions().len(), 4);

        // Hand-computed: sp = align_down_8(stack_base + stack_size)
        let expected_sps: [u32; 4] = [
            0x2000_0400, // 0x2000_0000 + 1024 = 0x2000_0400 (aligned)
            0x2000_1800, // 0x2000_1000 + 2048 = 0x2000_1800 (aligned)
            0x2000_2200, // 0x2000_2000 +  512 = 0x2000_2200 (aligned)
            0x2000_4000, // 0x2000_3000 + 4096 = 0x2000_4000 (aligned)
        ];
        for (i, &expected_sp) in expected_sps.iter().enumerate() {
            let pcb = ks.partitions().get(i).unwrap();
            assert_eq!(pcb.id(), i as u8);
            assert_eq!(pcb.state(), PartitionState::Ready);
            assert_eq!(pcb.stack_pointer(), expected_sp);
            assert_eq!(pcb.stack_pointer() % 8, 0);
        }
    }

    #[test]
    fn unaligned_stack_and_capacity() {
        let ks: KernelState<4, 4> =
            KernelState::new(sched2(), &[pcfg(0, 0x2000_0000, 1023)]).unwrap();
        assert_eq!(ks.partitions().get(0).unwrap().stack_pointer(), 0x2000_03F8);
        let two = [pcfg(0, 0x2000_0000, 1024), pcfg(1, 0x2000_1000, 1024)];
        assert!(KernelState::<1, 4>::new(sched2(), &two).is_none());
    }

    /// Build a started 2-slot schedule: P0 for 5 ticks, P1 for 3 ticks.
    #[cfg(not(feature = "dynamic-mpu"))]
    fn started_sched() -> ScheduleTable<4> {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 5)).unwrap();
        s.add(ScheduleEntry::new(1, 3)).unwrap();
        s.start();
        s
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    fn ks_started() -> KernelState<4, 4> {
        let cfgs = [pcfg(0, 0x2000_0000, 1024), pcfg(1, 0x2000_1000, 1024)];
        KernelState::new(started_sched(), &cfgs).unwrap()
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn yield_current_slot_advances_partition() {
        let mut ks = ks_started();
        // Consume 2 ticks in slot 0 (P0, duration=5)
        assert_eq!(ks.advance_schedule_tick(), None);
        assert_eq!(ks.advance_schedule_tick(), None);
        // Yield: skip remaining 3 ticks, advance to P1
        assert_eq!(ks.yield_current_slot().partition_id(), Some(1));
        assert_eq!(ks.active_partition(), Some(1));
        // P1 slot now has 3 ticks
        assert_eq!(ks.advance_schedule_tick(), None);
        assert_eq!(ks.advance_schedule_tick(), None);
        assert_eq!(ks.advance_schedule_tick(), Some(0)); // wraps to P0
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn yield_current_slot_at_start() {
        let mut ks = ks_started();
        // Yield immediately without consuming any ticks
        assert_eq!(ks.yield_current_slot().partition_id(), Some(1));
        assert_eq!(ks.active_partition(), Some(1));
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn yield_current_slot_wraps_around() {
        let mut ks = ks_started();
        // Advance to P1 first
        assert_eq!(ks.yield_current_slot().partition_id(), Some(1));
        // Yield again: wraps back to P0
        assert_eq!(ks.yield_current_slot().partition_id(), Some(0));
        assert_eq!(ks.active_partition(), Some(0));
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn yield_does_not_increment_tick() {
        let mut ks = ks_started();
        let tick_before = ks.tick().get();
        ks.yield_current_slot();
        // yield_current_slot does NOT increment the tick counter
        assert_eq!(ks.tick().get(), tick_before);
    }

    #[cfg(not(feature = "dynamic-mpu"))]
    #[test]
    fn yield_unstarted_returns_none() {
        let cfgs = [pcfg(0, 0x2000_0000, 1024)];
        let mut ks: KernelState<4, 4> = KernelState::new(sched2(), &cfgs).unwrap();
        // Schedule not started
        assert_eq!(ks.yield_current_slot().partition_id(), None);
        assert_eq!(ks.active_partition(), None);
    }
}
