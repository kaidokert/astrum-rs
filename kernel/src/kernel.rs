use crate::partition::{PartitionConfig, PartitionControlBlock, PartitionTable};
use crate::scheduler::ScheduleTable;

pub struct KernelState<const P: usize, const S: usize> {
    partitions: PartitionTable<P>,
    schedule: ScheduleTable<S>,
    active_partition: Option<u8>,
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
        })
    }

    pub fn partitions(&self) -> &PartitionTable<P> {
        &self.partitions
    }
    pub fn schedule(&self) -> &ScheduleTable<S> {
        &self.schedule
    }
    pub fn active_partition(&self) -> Option<u8> {
        self.active_partition
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
        let ks: KernelState<4, 4> = KernelState::new(sched2(), &cfgs).unwrap();
        assert_eq!(ks.partitions().len(), 2);
        assert_eq!(ks.active_partition(), None);
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
}
