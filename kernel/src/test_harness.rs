use crate::config::KernelConfig;
use crate::partition::{ConfigError, MpuRegion, PartitionConfig, PartitionState, TransitionError};
use crate::scheduler::{ScheduleEntry, ScheduleTable};
use crate::svc::Kernel;
use heapless::Vec;

pub struct HarnessConfig;

impl KernelConfig for HarnessConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const STACK_WORDS: usize = 256;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 4;
    const MW: usize = 4;
    const QS: usize = 4;
    const QD: usize = 4;
    const QM: usize = 4;
    const QW: usize = 4;
    const SP: usize = 4;
    const SM: usize = 4;
    const BS: usize = 4;
    const BM: usize = 4;
    const BW: usize = 4;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 4;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    type Core =
        crate::partition_core::PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = crate::port_pools::PortPools<
        { Self::SP },
        { Self::SM },
        { Self::BS },
        { Self::BM },
        { Self::BW },
    >;
}

/// Base address for partition flash (code) regions in the test memory map.
const FLASH_BASE: u32 = 0x0800_0000;
/// Base address for partition RAM (stack/data) regions in the test memory map.
const RAM_BASE: u32 = 0x2000_0000;
/// Address offset between consecutive partition regions.
const PARTITION_OFFSET: u32 = 0x1000;
/// Stack size in bytes for each test partition, derived from STACK_WORDS.
const STACK_SIZE_BYTES: u32 = HarnessConfig::STACK_WORDS as u32 * 4;

#[derive(Debug)]
pub enum HarnessError {
    InvalidPartitionCount,
    ScheduleFull,
    ConfigsFull,
    KernelInit(ConfigError),
    Transition(TransitionError),
    PartitionNotFound,
}

pub struct KernelTestHarness {
    kernel: Kernel<HarnessConfig>,
}

impl KernelTestHarness {
    pub fn with_partitions(n: usize) -> Result<Self, HarnessError> {
        if n == 0 || n > HarnessConfig::N {
            return Err(HarnessError::InvalidPartitionCount);
        }
        let mut schedule = ScheduleTable::new();
        for i in 0..n {
            schedule
                .add(ScheduleEntry::new(i as u8, 10))
                .map_err(|_| HarnessError::ScheduleFull)?;
        }
        #[cfg(feature = "dynamic-mpu")]
        schedule
            .add_system_window(10)
            .map_err(|_| HarnessError::ScheduleFull)?;
        let mut configs: Vec<PartitionConfig, { HarnessConfig::N }> = Vec::new();
        for i in 0..n {
            let o = (i as u32) * PARTITION_OFFSET;
            configs
                .push(PartitionConfig {
                    id: i as u8,
                    entry_point: FLASH_BASE + o,
                    stack_base: RAM_BASE + o,
                    stack_size: STACK_SIZE_BYTES,
                    mpu_region: MpuRegion::new(RAM_BASE + o, STACK_SIZE_BYTES, 0),
                    peripheral_regions: Vec::new(),
                })
                .map_err(|_| HarnessError::ConfigsFull)?;
        }
        let mut kernel = Kernel::new(
            schedule,
            &configs,
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
        )
        .map_err(HarnessError::KernelInit)?;
        for i in 0..n {
            kernel
                .partitions_mut()
                .get_mut(i)
                .ok_or(HarnessError::PartitionNotFound)?
                .transition(PartitionState::Running)
                .map_err(HarnessError::Transition)?;
        }
        kernel.current_partition = 0;
        Ok(Self { kernel })
    }

    pub fn kernel(&self) -> &Kernel<HarnessConfig> {
        &self.kernel
    }

    pub fn kernel_mut(&mut self) -> &mut Kernel<HarnessConfig> {
        &mut self.kernel
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert_eq!(h.kernel().partitions().len(), 2);
        assert_eq!(h.kernel().current_partition, 0);
        for i in 0..2 {
            assert_eq!(
                h.kernel().partitions().get(i).unwrap().state(),
                PartitionState::Running
            );
        }
    }

    #[test]
    fn four_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        assert_eq!(h.kernel().partitions().len(), 4);
        for i in 0..4 {
            assert_eq!(
                h.kernel().partitions().get(i).unwrap().state(),
                PartitionState::Running
            );
        }
    }

    #[test]
    fn kernel_mut_accessor() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        h.kernel_mut().current_partition = 1;
        assert_eq!(h.kernel().current_partition, 1);
    }
}
