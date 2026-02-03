//! Partition-aware virtual device trait and error types.
//!
//! [`VirtualDevice`] defines an abstract interface for devices that enforce
//! partition-level access control via `open`/`close` gating.
//! [`DeviceError`] enumerates the error conditions for device operations.

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
pub trait VirtualDevice {
    fn device_id(&self) -> u8;
    fn open(&mut self, partition_id: u8) -> Result<(), DeviceError>;
    fn close(&mut self, partition_id: u8) -> Result<(), DeviceError>;
    fn read(&mut self, partition_id: u8, buf: &mut [u8]) -> Result<usize, DeviceError>;
    fn write(&mut self, partition_id: u8, data: &[u8]) -> Result<usize, DeviceError>;
    fn ioctl(&mut self, partition_id: u8, cmd: u32, arg: u32) -> Result<u32, DeviceError>;
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
}
