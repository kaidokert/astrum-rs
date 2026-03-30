//! Partition-aware virtual device trait, error types, and fixed-capacity registry.
//!
//! [`VirtualDevice`] defines an abstract interface for devices that enforce
//! partition-level access control via `open`/`close` gating.
//! [`DeviceError`] enumerates the error conditions for device operations.
//! [`DeviceRegistry`] provides O(n) lookup by device ID with compile-time
//! capacity, suitable for static initialization on `no_std` targets.

/// Errors returned by virtual device operations.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DeviceError {
    NotFound,
    PermissionDenied,
    NotOpen,
    AlreadyOpen,
    BufferFull,
    BufferEmpty,
    InvalidPartition,
    RegistryFull,
    DuplicateId,
}

/// Partition-aware virtual device trait.
pub trait VirtualDevice: Send {
    fn device_id(&self) -> u8;
    fn open(&mut self, partition_id: crate::ids::PartitionId) -> Result<(), DeviceError>;
    fn close(&mut self, partition_id: crate::ids::PartitionId) -> Result<(), DeviceError>;
    fn read(
        &mut self,
        partition_id: crate::ids::PartitionId,
        buf: &mut [u8],
    ) -> Result<usize, DeviceError>;
    fn write(
        &mut self,
        partition_id: crate::ids::PartitionId,
        data: &[u8],
    ) -> Result<usize, DeviceError>;
    fn ioctl(
        &mut self,
        partition_id: crate::ids::PartitionId,
        cmd: u32,
        arg: u32,
    ) -> Result<u32, DeviceError>;
}

/// Fixed-capacity registry of virtual devices looked up by device ID.
pub struct DeviceRegistry<'a, const N: usize> {
    devices: [Option<&'a mut dyn VirtualDevice>; N],
    count: usize,
}

impl<'a, const N: usize> Default for DeviceRegistry<'a, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a, const N: usize> DeviceRegistry<'a, N> {
    pub const fn new() -> Self {
        Self {
            devices: [const { None }; N],
            count: 0,
        }
    }

    /// Register a device. Returns `DuplicateId` if the device ID is already
    /// registered, or `RegistryFull` if capacity is exhausted.
    pub fn add(&mut self, dev: &'a mut dyn VirtualDevice) -> Result<(), DeviceError> {
        let id = dev.device_id();
        for slot in self.devices[..self.count].iter().flatten() {
            if slot.device_id() == id {
                return Err(DeviceError::DuplicateId);
            }
        }
        if self.count >= N {
            return Err(DeviceError::RegistryFull);
        }
        self.devices[self.count] = Some(dev);
        self.count += 1;
        Ok(())
    }

    pub fn get_mut(&mut self, id: u8) -> Option<&mut dyn VirtualDevice> {
        for dev in self.devices[..self.count].iter_mut().flatten() {
            if dev.device_id() == id {
                return Some(&mut **dev);
            }
        }
        None
    }

    pub fn len(&self) -> usize {
        self.count
    }
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::clone_on_copy)]
    fn device_error_clone() {
        let original = DeviceError::PermissionDenied;
        let cloned = original.clone();
        // If Clone weren't derived, this wouldn't compile.
        assert_eq!(original, cloned);
    }

    #[test]
    fn device_error_copy() {
        let err = DeviceError::AlreadyOpen;
        let copied = err;
        // If Copy weren't derived, `err` would be moved and unusable here.
        assert_eq!(err, copied);
    }

    #[test]
    fn device_error_all_variants_distinct() {
        let variants = [
            DeviceError::NotFound,
            DeviceError::PermissionDenied,
            DeviceError::NotOpen,
            DeviceError::AlreadyOpen,
            DeviceError::BufferFull,
            DeviceError::BufferEmpty,
            DeviceError::InvalidPartition,
            DeviceError::RegistryFull,
            DeviceError::DuplicateId,
        ];
        for (i, a) in variants.iter().enumerate() {
            for (j, b) in variants.iter().enumerate() {
                if i == j {
                    assert_eq!(a, b);
                } else {
                    assert_ne!(a, b);
                }
            }
        }
    }

    use crate::ids::PartitionId;

    fn pid(v: u8) -> PartitionId {
        PartitionId::new(v as u32)
    }

    struct MockDev(u8, Option<PartitionId>);
    impl MockDev {
        fn new(id: u8) -> Self {
            Self(id, None)
        }
        fn require_open(&self, p: PartitionId) -> Result<(), DeviceError> {
            match self.1 == Some(p) {
                true => Ok(()),
                false => Err(DeviceError::NotOpen),
            }
        }
    }
    impl VirtualDevice for MockDev {
        fn device_id(&self) -> u8 {
            self.0
        }
        fn open(&mut self, p: PartitionId) -> Result<(), DeviceError> {
            if self.1.is_some() {
                return Err(DeviceError::PermissionDenied);
            }
            self.1 = Some(p);
            Ok(())
        }
        fn close(&mut self, p: PartitionId) -> Result<(), DeviceError> {
            self.require_open(p)?;
            self.1 = None;
            Ok(())
        }
        fn read(&mut self, p: PartitionId, buf: &mut [u8]) -> Result<usize, DeviceError> {
            self.require_open(p)?;
            buf[0] = self.0;
            Ok(1)
        }
        fn write(&mut self, p: PartitionId, data: &[u8]) -> Result<usize, DeviceError> {
            self.require_open(p)?;
            Ok(data.len())
        }
        fn ioctl(&mut self, p: PartitionId, _: u32, _: u32) -> Result<u32, DeviceError> {
            self.require_open(p)?;
            Ok(0)
        }
    }

    #[test]
    fn registry_add_and_lookup() {
        let (mut d1, mut d2) = (MockDev::new(1), MockDev::new(2));
        let mut reg = DeviceRegistry::<2>::new();
        assert!(reg.is_empty());
        reg.add(&mut d1).unwrap();
        reg.add(&mut d2).unwrap();
        assert_eq!(reg.len(), 2);
        assert_eq!(reg.get_mut(1).unwrap().device_id(), 1);
        assert_eq!(reg.get_mut(2).unwrap().device_id(), 2);
        assert!(reg.get_mut(99).is_none());
    }

    #[test]
    fn registry_returns_error_when_full() {
        let (mut d1, mut d2, mut d3) = (MockDev::new(1), MockDev::new(2), MockDev::new(3));
        let mut reg = DeviceRegistry::<2>::new();
        reg.add(&mut d1).unwrap();
        reg.add(&mut d2).unwrap();
        assert_eq!(reg.add(&mut d3), Err(DeviceError::RegistryFull));
    }

    #[test]
    fn registry_returns_error_on_duplicate_id() {
        let (mut d1, mut dup) = (MockDev::new(1), MockDev::new(1));
        let mut reg = DeviceRegistry::<4>::new();
        reg.add(&mut d1).unwrap();
        assert_eq!(reg.add(&mut dup), Err(DeviceError::DuplicateId));
    }

    #[test]
    fn get_mut_full_device_lifecycle() {
        let mut dev = MockDev::new(10);
        let mut reg = DeviceRegistry::<4>::new();
        reg.add(&mut dev).unwrap();
        let d = reg.get_mut(10).unwrap();
        assert_eq!(d.read(pid(1), &mut [0; 4]), Err(DeviceError::NotOpen));
        assert_eq!(d.write(pid(1), &[1]), Err(DeviceError::NotOpen));
        d.open(pid(1)).unwrap();
        let mut buf = [0u8; 4];
        assert_eq!(d.read(pid(1), &mut buf).unwrap(), 1);
        assert_eq!(d.write(pid(1), &[0xAA]).unwrap(), 1);
        assert_eq!(d.ioctl(pid(1), 0, 0).unwrap(), 0);
        d.close(pid(1)).unwrap();
        assert_eq!(d.close(pid(1)), Err(DeviceError::NotOpen));
    }
}
