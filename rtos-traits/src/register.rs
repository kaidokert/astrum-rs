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

/// Maximum number of distinct registers a MockRegisterBank can hold.
#[cfg(any(test, feature = "mock-hal"))]
const MOCK_MAX_REGS: usize = 64;

/// Maximum number of writes a MockRegisterBank can log.
#[cfg(any(test, feature = "mock-hal"))]
const MOCK_MAX_LOG: usize = 128;

/// Mock register bank for testing, backed by fixed-size arrays (no_std compatible).
///
/// Provides pre-loading via `set()`, write log recording, and assertion
/// helpers (`assert_wrote`, `assert_sequence`) for verifying register access
/// patterns in unit tests.
#[cfg(any(test, feature = "mock-hal"))]
pub struct MockRegisterBank {
    /// Sparse register storage: (offset, value) pairs.
    regs: [(usize, u32); MOCK_MAX_REGS],
    reg_count: usize,
    /// Write log: (offset, value) for each write call.
    log: [(usize, u32); MOCK_MAX_LOG],
    log_count: usize,
}

#[cfg(any(test, feature = "mock-hal"))]
impl MockRegisterBank {
    /// Create an empty mock register bank. All reads return 0 by default.
    pub fn new() -> Self {
        Self {
            regs: [(0, 0); MOCK_MAX_REGS],
            reg_count: 0,
            log: [(0, 0); MOCK_MAX_LOG],
            log_count: 0,
        }
    }

    /// Pre-load a register value before test execution.
    pub fn set(&mut self, offset: usize, value: u32) {
        // Update existing entry if found.
        for i in 0..self.reg_count {
            if self.regs[i].0 == offset {
                self.regs[i].1 = value;
                return;
            }
        }
        assert!(
            self.reg_count < MOCK_MAX_REGS,
            "MockRegisterBank: exceeded max register slots"
        );
        self.regs[self.reg_count] = (offset, value);
        self.reg_count += 1;
    }

    /// Look up a register value by offset.
    fn get(&self, offset: usize) -> Option<u32> {
        for i in 0..self.reg_count {
            if self.regs[i].0 == offset {
                return Some(self.regs[i].1);
            }
        }
        None
    }

    /// Assert that a specific (offset, value) pair appears in the write log.
    ///
    /// # Panics
    /// Panics if the write was not recorded.
    pub fn assert_wrote(&self, offset: usize, value: u32) {
        let log = self.write_log();
        assert!(
            log.contains(&(offset, value)),
            "expected write({:#x}, {:#010x}) not found in log: {:?}",
            offset,
            value,
            log
        );
    }

    /// Assert that the entire write log matches `expected` exactly.
    ///
    /// # Panics
    /// Panics if the log differs from `expected`.
    pub fn assert_sequence(&self, expected: &[(usize, u32)]) {
        assert_eq!(self.write_log(), expected, "write log mismatch");
    }

    /// Return a reference to the write log.
    pub fn write_log(&self) -> &[(usize, u32)] {
        &self.log[..self.log_count]
    }
}

#[cfg(any(test, feature = "mock-hal"))]
impl Default for MockRegisterBank {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(any(test, feature = "mock-hal"))]
impl RegisterBank for MockRegisterBank {
    fn read(&self, offset: usize) -> u32 {
        self.get(offset).unwrap_or(0)
    }

    fn write(&mut self, offset: usize, value: u32) {
        // Update register storage.
        self.set(offset, value);
        // Record in write log.
        assert!(
            self.log_count < MOCK_MAX_LOG,
            "MockRegisterBank: exceeded max write log entries"
        );
        self.log[self.log_count] = (offset, value);
        self.log_count += 1;
    }
}

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

    #[test]
    fn read_default_zero() {
        let bank = MockRegisterBank::new();
        assert_eq!(bank.read(0), 0);
        assert_eq!(bank.read(0x100), 0);
        assert_eq!(bank.read(0xFFFC), 0);
    }

    #[test]
    fn write_read_roundtrip() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0xDEAD_BEEF);
        assert_eq!(bank.read(0), 0xDEAD_BEEF);

        bank.write(4, 0x1234_5678);
        assert_eq!(bank.read(4), 0x1234_5678);
        assert_eq!(bank.read(0), 0xDEAD_BEEF);
    }

    #[test]
    fn write_log_records_all_writes() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0xAA);
        bank.write(4, 0xBB);
        bank.write(0, 0xCC);

        assert_eq!(bank.write_log(), &[(0, 0xAA), (4, 0xBB), (0, 0xCC)]);
    }

    #[test]
    fn modify_logs_the_write() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 0x00FF);
        bank.modify(0, |v| v | 0x0F00);
        assert_eq!(bank.read(0), 0x0FFF);
        // Log: initial write + modify's write
        assert_eq!(bank.write_log(), &[(0, 0x00FF), (0, 0x0FFF)]);
    }

    #[test]
    fn assert_wrote_passes_on_match() {
        let mut bank = MockRegisterBank::new();
        bank.write(8, 0x42);
        bank.assert_wrote(8, 0x42);
    }

    #[test]
    #[should_panic(expected = "not found in log")]
    fn assert_wrote_fails_on_mismatch() {
        let mut bank = MockRegisterBank::new();
        bank.write(8, 0x42);
        bank.assert_wrote(8, 0x99);
    }

    #[test]
    fn assert_sequence_passes_on_match() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 1);
        bank.write(4, 2);
        bank.write(8, 3);
        bank.assert_sequence(&[(0, 1), (4, 2), (8, 3)]);
    }

    #[test]
    #[should_panic(expected = "write log mismatch")]
    fn assert_sequence_fails_on_mismatch() {
        let mut bank = MockRegisterBank::new();
        bank.write(0, 1);
        bank.write(4, 2);
        bank.assert_sequence(&[(4, 2), (0, 1)]);
    }

    #[test]
    fn set_preloads_value() {
        let mut bank = MockRegisterBank::new();
        bank.set(0x10, 0xBEEF);
        assert_eq!(bank.read(0x10), 0xBEEF);
        // set() should not appear in write log
        assert!(bank.write_log().is_empty());
    }

    #[test]
    fn set_then_modify() {
        let mut bank = MockRegisterBank::new();
        bank.set(0, 0xFF00);
        bank.modify(0, |v| v | 0x00FF);
        assert_eq!(bank.read(0), 0xFFFF);
        // Only modify's write in the log
        assert_eq!(bank.write_log(), &[(0, 0xFFFF)]);
    }

    #[test]
    fn default_trait() {
        let bank = MockRegisterBank::default();
        assert_eq!(bank.read(0), 0);
        assert!(bank.write_log().is_empty());
    }
}
