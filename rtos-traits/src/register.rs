/// Trait for register bank access at word-aligned offsets.
///
/// Each offset is a byte offset from the bank's base address.
/// Implementations must handle the mapping from offset to the
/// actual register location.
// TODO(panic-free): RegisterBank methods currently return bare u32 / (), which forces
// implementers to panic or return junk on invalid input (e.g. unaligned offsets). Migrate
// the trait to return Result<u32, RegisterError> once the kernel-wide error strategy is
// settled, so callers can propagate errors instead of panicking in handler mode.
pub trait RegisterBank {
    /// Read a 32-bit register at the given byte offset.
    fn read(&self, offset: usize) -> u32;

    /// Write a 32-bit value to the register at the given byte offset.
    fn write(&mut self, offset: usize, value: u32);

    /// Read-modify-write: reads the register, applies `f`, writes back.
    // TODO: default implementation is a non-atomic read-modify-write. While `&mut self`
    // prevents task-level races, it does not protect against ISR-driven register updates
    // or hardware side-effects between the read and write. Implementers requiring atomic
    // RMW should override this method.
    fn modify<F: FnOnce(u32) -> u32>(&mut self, offset: usize, f: F) {
        let val = self.read(offset);
        self.write(offset, f(val));
    }
}

// TODO(object-safety): The generic `modify<F>` method makes `RegisterBank` not object-safe
// (`dyn RegisterBank` cannot be used). If HAL abstractions need trait objects, consider
// moving `modify` to a separate `RegisterBankExt` extension trait or using a non-generic
// signature (e.g. `fn modify(&mut self, offset: usize, f: &dyn FnOnce(u32) -> u32)`).

/// MMIO-backed register bank using volatile pointer access.
///
/// Wraps a raw `*mut u32` base pointer. Each offset is a byte offset
/// that must be word-aligned (multiple of 4).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MmioRegisterBank {
    base: *mut u32,
}

// SAFETY: MMIO registers are fixed hardware addresses that are globally accessible
// regardless of which thread accesses them. The pointer does not alias normal memory
// and volatile access is inherently thread-unsafe at the hardware level (requiring
// external synchronization such as a Mutex), but the *capability* to access MMIO
// must be transferable across threads/partitions for kernel use.
unsafe impl Send for MmioRegisterBank {}
unsafe impl Sync for MmioRegisterBank {}

impl MmioRegisterBank {
    /// Create a new MMIO register bank from a base pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure `base` points to a valid MMIO region
    /// that remains valid for the lifetime of this struct, and that
    /// offsets used with `read`/`write` stay within the region.
    pub unsafe fn new(base: *mut u32) -> Self {
        Self { base }
    }
}

impl RegisterBank for MmioRegisterBank {
    fn read(&self, offset: usize) -> u32 {
        // TODO(panic-free): convert to Result once RegisterBank returns Result
        assert!(
            offset.is_multiple_of(4),
            "MMIO read offset must be word-aligned (multiple of 4)"
        );
        // SAFETY: caller of `MmioRegisterBank::new` guarantees `base` points to a valid
        // MMIO region. The assert above ensures the offset is word-aligned so the
        // resulting pointer maintains the u32 alignment of `base`. The volatile read
        // is required for MMIO semantics.
        unsafe { self.base.byte_add(offset).read_volatile() }
    }

    fn write(&mut self, offset: usize, value: u32) {
        // TODO(panic-free): convert to Result once RegisterBank returns Result
        assert!(
            offset.is_multiple_of(4),
            "MMIO write offset must be word-aligned (multiple of 4)"
        );
        // SAFETY: caller of `MmioRegisterBank::new` guarantees `base` points to a valid
        // MMIO region. The assert above ensures the offset is word-aligned so the
        // resulting pointer maintains the u32 alignment of `base`. The volatile write
        // is required for MMIO semantics.
        unsafe { self.base.byte_add(offset).write_volatile(value) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal mock register bank backed by a fixed-size array.
    struct MockRegisterBank {
        regs: [u32; 4],
    }

    impl MockRegisterBank {
        fn new() -> Self {
            Self { regs: [0; 4] }
        }
    }

    impl RegisterBank for MockRegisterBank {
        fn read(&self, offset: usize) -> u32 {
            assert!(
                offset.is_multiple_of(4),
                "mock read offset must be word-aligned"
            );
            *self
                .regs
                .get(offset / 4)
                .expect("mock read offset out of range")
        }

        fn write(&mut self, offset: usize, value: u32) {
            assert!(
                offset.is_multiple_of(4),
                "mock write offset must be word-aligned"
            );
            *self
                .regs
                .get_mut(offset / 4)
                .expect("mock write offset out of range") = value;
        }
    }

    #[test]
    fn read_write_roundtrip() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0xDEAD_BEEF);
        assert_eq!(bank.read(0), 0xDEAD_BEEF);

        bank.write(4, 0x1234_5678);
        assert_eq!(bank.read(4), 0x1234_5678);
        // First register unchanged
        assert_eq!(bank.read(0), 0xDEAD_BEEF);
    }

    #[test]
    fn modify_reads_then_writes_transformed_value() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0x0000_00FF);

        // modify should read 0xFF, apply the OR, write back 0xFF0F
        bank.modify(0, |v| v | 0x0000_0F00);
        assert_eq!(bank.read(0), 0x0000_0FFF);
    }

    #[test]
    fn modify_on_different_offsets() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0x0001);
        bank.write(8, 0x0010);

        bank.modify(0, |v| v << 4);
        bank.modify(8, |v| v | 0xFF00);

        assert_eq!(bank.read(0), 0x0010);
        assert_eq!(bank.read(8), 0xFF10);
    }

    #[test]
    fn modify_clear_bits() {
        let mut bank = MockRegisterBank::new();
        bank.write(4, 0xFFFF_FFFF);

        bank.modify(4, |v| v & 0x0000_FFFF);
        assert_eq!(bank.read(4), 0x0000_FFFF);
    }
}
