//! Virtual device registry and trait for dynamic-MPU device abstraction.

/// Errors returned by virtual device operations.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DeviceError {
    NotFound,
    PermissionDenied,
    NotOpen,
    BufferFull,
    BufferEmpty,
    InvalidPartition,
}

/// Partition-aware virtual device trait.
pub trait VirtualDevice {
    fn device_id(&self) -> u8;
    fn open(&mut self, partition_id: u8) -> Result<(), DeviceError>;
    fn close(&mut self, partition_id: u8) -> Result<(), DeviceError>;
    fn read(&mut self, partition_id: u8, buf: &mut [u8]) -> Result<usize, DeviceError>;
    fn write(&mut self, partition_id: u8, data: &[u8]) -> Result<usize, DeviceError>;
    fn ioctl(&mut self, partition_id: u8, cmd: u32, arg: u32) -> Result<u32, DeviceError>;
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

    /// Register a device. Panics on duplicate IDs or full registry since these
    /// indicate misconfiguration during system initialization.
    pub fn add(&mut self, dev: &'a mut dyn VirtualDevice) {
        let id = dev.device_id();
        for slot in self.devices[..self.count].iter().flatten() {
            if slot.device_id() == id {
                panic!("duplicate device ID {}", id);
            }
        }
        assert!(self.count < N, "device registry full (capacity {})", N);
        self.devices[self.count] = Some(dev);
        self.count += 1;
    }

    pub fn get_mut(&mut self, id: u8) -> Option<&mut dyn VirtualDevice> {
        for dev in self.devices[..self.count].iter_mut().flatten() {
            if dev.device_id() == id {
                return Some(*dev);
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

    struct MockDev(u8, Option<u8>);
    impl MockDev {
        fn new(id: u8) -> Self {
            Self(id, None)
        }
        fn require_open(&self, pid: u8) -> Result<(), DeviceError> {
            match self.1 == Some(pid) {
                true => Ok(()),
                false => Err(DeviceError::NotOpen),
            }
        }
    }
    impl VirtualDevice for MockDev {
        fn device_id(&self) -> u8 {
            self.0
        }
        fn open(&mut self, pid: u8) -> Result<(), DeviceError> {
            if self.1.is_some() {
                return Err(DeviceError::PermissionDenied);
            }
            self.1 = Some(pid);
            Ok(())
        }
        fn close(&mut self, pid: u8) -> Result<(), DeviceError> {
            self.require_open(pid)?;
            self.1 = None;
            Ok(())
        }
        fn read(&mut self, pid: u8, buf: &mut [u8]) -> Result<usize, DeviceError> {
            self.require_open(pid)?;
            buf[0] = self.0;
            Ok(1)
        }
        fn write(&mut self, pid: u8, data: &[u8]) -> Result<usize, DeviceError> {
            self.require_open(pid)?;
            Ok(data.len())
        }
        fn ioctl(&mut self, pid: u8, _: u32, _: u32) -> Result<u32, DeviceError> {
            self.require_open(pid)?;
            Ok(0)
        }
    }

    #[test]
    fn registry_add_and_lookup() {
        let (mut d1, mut d2) = (MockDev::new(1), MockDev::new(2));
        let mut reg = DeviceRegistry::<2>::new();
        assert!(reg.is_empty());
        reg.add(&mut d1);
        reg.add(&mut d2);
        assert_eq!(reg.len(), 2);
        assert_eq!(reg.get_mut(1).unwrap().device_id(), 1);
        assert_eq!(reg.get_mut(2).unwrap().device_id(), 2);
        assert!(reg.get_mut(99).is_none());
    }

    #[test]
    #[should_panic(expected = "device registry full")]
    fn registry_panics_when_full() {
        let (mut d1, mut d2, mut d3) = (MockDev::new(1), MockDev::new(2), MockDev::new(3));
        let mut reg = DeviceRegistry::<2>::new();
        reg.add(&mut d1);
        reg.add(&mut d2);
        reg.add(&mut d3);
    }

    #[test]
    #[should_panic(expected = "duplicate device ID")]
    fn registry_panics_on_duplicate_id() {
        let (mut d1, mut dup) = (MockDev::new(1), MockDev::new(1));
        let mut reg = DeviceRegistry::<4>::new();
        reg.add(&mut d1);
        reg.add(&mut dup);
    }

    #[test]
    fn get_mut_full_device_lifecycle() {
        let mut dev = MockDev::new(10);
        let mut reg = DeviceRegistry::<4>::new();
        reg.add(&mut dev);
        let d = reg.get_mut(10).unwrap();
        assert_eq!(d.read(1, &mut [0; 4]), Err(DeviceError::NotOpen));
        assert_eq!(d.write(1, &[1]), Err(DeviceError::NotOpen));
        d.open(1).unwrap();
        let mut buf = [0u8; 4];
        assert_eq!(d.read(1, &mut buf).unwrap(), 1);
        assert_eq!(d.write(1, &[0xAA]).unwrap(), 1);
        assert_eq!(d.ioctl(1, 0, 0).unwrap(), 0);
        d.close(1).unwrap();
        assert_eq!(d.close(1), Err(DeviceError::NotOpen));
    }
}
