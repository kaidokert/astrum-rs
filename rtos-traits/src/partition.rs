//! Partition-level type definitions shared between kernel and plib.

/// Signature for a partition body function that receives an argument in `r0`.
pub type PartitionBody = extern "C" fn(u32) -> !;
/// Signature for a partition entry point (no arguments, diverging).
pub type PartitionEntry = extern "C" fn() -> !;
/// Signature for an interrupt service routine handler.
pub type IsrHandler = unsafe extern "C" fn();

/// A partition descriptor: entry point address paired with its `r0` argument.
///
/// The entry point is stored as an [`EntryAddr`] (a `u32` address) rather than
/// a function pointer, so that both `PartitionEntry` and `PartitionBody`
/// signatures can be represented without `transmute`.
#[repr(C)]
#[derive(Clone, Debug)]
pub struct PartitionSpec {
    entry_point: EntryAddr,
    r0: u32,
    data_mpu: Option<MpuRegion>,
    code_mpu: Option<MpuRegion>,
    peripherals: &'static [MpuRegion],
    fault_policy: FaultPolicy,
    error_handler: Option<u32>,
}

impl PartitionSpec {
    pub fn new(entry_point: impl EntryPointFn, r0: u32) -> Self {
        Self {
            entry_point: EntryAddr::from_entry(entry_point),
            r0,
            data_mpu: None,
            code_mpu: None,
            peripherals: &[],
            fault_policy: FaultPolicy::StayDead,
            error_handler: None,
        }
    }
    /// Create a spec from a [`PartitionEntry`] fn pointer with `r0 = 0`.
    ///
    /// Accepts a bare function item directly — no `as` cast required.
    // TODO(panic-free): inherits debug_assert! panic from EntryAddr::from_entry;
    // in handler mode (SVC) this is unrecoverable. Align with Panic-Free Policy.
    pub fn entry(f: PartitionEntry) -> Self {
        Self::new(f, 0)
    }
    /// Create a spec from a [`PartitionBody`] fn pointer with an explicit `r0`.
    // TODO(panic-free): inherits debug_assert! panic from EntryAddr::from_entry;
    // in handler mode (SVC) this is unrecoverable. Align with Panic-Free Policy.
    pub fn body(f: PartitionBody, r0: u32) -> Self {
        Self::new(f, r0)
    }
    pub const fn entry_point(&self) -> EntryAddr {
        self.entry_point
    }
    pub const fn r0(&self) -> u32 {
        self.r0
    }
    pub const fn data_mpu(&self) -> Option<MpuRegion> {
        self.data_mpu
    }
    pub const fn code_mpu(&self) -> Option<MpuRegion> {
        self.code_mpu
    }
    pub const fn peripherals(&self) -> &'static [MpuRegion] {
        self.peripherals
    }
    pub const fn fault_policy(&self) -> FaultPolicy {
        self.fault_policy
    }
    pub const fn error_handler(&self) -> Option<u32> {
        self.error_handler
    }

    // -- builder methods --

    pub const fn with_r0(mut self, r0: u32) -> Self {
        self.r0 = r0;
        self
    }
    pub const fn with_data_mpu(mut self, region: MpuRegion) -> Self {
        self.data_mpu = Some(region);
        self
    }
    pub const fn with_code_mpu(mut self, region: MpuRegion) -> Self {
        self.code_mpu = Some(region);
        self
    }
    pub const fn with_peripherals(mut self, regions: &'static [MpuRegion]) -> Self {
        self.peripherals = regions;
        self
    }
    pub const fn with_fault_policy(mut self, policy: FaultPolicy) -> Self {
        self.fault_policy = policy;
        self
    }
    pub const fn with_error_handler(mut self, addr: u32) -> Self {
        self.error_handler = Some(addr);
        self
    }

    /// Construct a `PartitionSpec` from a raw `u32` entry-point address.
    ///
    /// This bypasses the `EntryPointFn` trait so that host-mode tests (64-bit)
    /// can build specs without triggering the `debug_assert!` inside
    /// `EntryAddr::from_entry`.
    #[doc(hidden)]
    pub fn from_raw_entry(entry_point: u32, r0: u32) -> Self {
        Self {
            entry_point: EntryAddr::from(entry_point),
            r0,
            data_mpu: None,
            code_mpu: None,
            peripherals: &[],
            fault_policy: FaultPolicy::StayDead,
            error_handler: None,
        }
    }
}

impl From<(PartitionEntry, u32)> for PartitionSpec {
    fn from((entry_point, r0): (PartitionEntry, u32)) -> Self {
        Self::new(entry_point, r0)
    }
}
impl From<PartitionEntry> for PartitionSpec {
    fn from(entry_point: PartitionEntry) -> Self {
        Self::new(entry_point, 0)
    }
}
impl From<PartitionBody> for PartitionSpec {
    fn from(body: PartitionBody) -> Self {
        Self::new(body, 0)
    }
}

/// A peripheral MPU region descriptor: base address, size, and RASR
/// permission/attribute bits.
///
/// The struct is `Copy` so it can live inside stack-allocated partition
/// configuration arrays without heap allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuRegion {
    base: u32,
    size: u32,
    permissions: u32,
}

impl MpuRegion {
    pub const fn new(base: u32, size: u32, permissions: u32) -> Self {
        Self {
            base,
            size,
            permissions,
        }
    }

    pub fn base(&self) -> u32 {
        self.base
    }

    pub fn size(&self) -> u32 {
        self.size
    }

    pub fn permissions(&self) -> u32 {
        self.permissions
    }

    /// Returns `true` when the region's base and size satisfy all ARMv7-M
    /// MPU constraints:
    /// 1. `size >= 32` (minimum MPU region size)
    /// 2. `size` is a power of two
    /// 3. `base` is aligned to `size` (`base & (size - 1) == 0`)
    /// 4. `base + size` does not overflow `u32`
    pub fn is_mappable(&self) -> bool {
        let base = self.base;
        let size = self.size;
        size >= 32
            && size.is_power_of_two()
            && (base & (size - 1)) == 0
            && base.checked_add(size).is_some()
    }
}

/// Policy controlling what happens when a partition faults.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum FaultPolicy {
    /// Partition stays in Faulted state permanently.
    #[default]
    StayDead,
    /// Partition is warm-restarted (state preserved) up to `max` times.
    WarmRestart { max: u32 },
    /// Partition is cold-restarted (full reset) up to `max` times.
    ColdRestart { max: u32 },
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::PartitionEntry {}
    impl Sealed for super::PartitionBody {}
}

/// Trait abstracting over both `PartitionEntry` and `PartitionBody` function
/// pointer types, converting either into an [`EntryAddr`].
///
/// Sealed: external crates cannot implement this trait.
pub trait EntryPointFn: sealed::Sealed {
    /// Return the raw function pointer address.
    #[doc(hidden)]
    fn into_usize(self) -> usize;

    fn into_entry_addr(self) -> EntryAddr;
}

impl EntryPointFn for PartitionEntry {
    #[inline]
    fn into_usize(self) -> usize {
        self as *const () as usize
    }
    #[inline]
    fn into_entry_addr(self) -> EntryAddr {
        EntryAddr::from_entry(self)
    }
}

impl EntryPointFn for PartitionBody {
    #[inline]
    fn into_usize(self) -> usize {
        self as *const () as usize
    }
    #[inline]
    fn into_entry_addr(self) -> EntryAddr {
        EntryAddr::from_entry(self)
    }
}

/// Compile-time check that a function has the correct entry-point signature.
///
/// - `check_entry_sig!(my_fn)` — asserts `my_fn` is a [`PartitionEntry`] (`extern "C" fn() -> !`).
/// - `check_entry_sig!(my_fn, body)` — asserts `my_fn` is a [`PartitionBody`] (`extern "C" fn(u32) -> !`).
#[macro_export]
macro_rules! check_entry_sig {
    ($fn:path) => {
        const _: $crate::partition::PartitionEntry = $fn;
    };
    ($fn:path, body) => {
        const _: $crate::partition::PartitionBody = $fn;
    };
}

/// Type-safe wrapper for a partition entry-point address.
///
/// Internally stores a `u32`, which is the native pointer width on Cortex-M.
/// The `usize`-to-`u32` cast in `from_entry` is lossless on 32-bit targets;
/// on wider hosts (e.g. 64-bit simulator builds) the cast would truncate.
/// `from_entry` contains a `debug_assert!` that catches this at runtime in
/// debug builds.
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct EntryAddr(u32);

impl EntryAddr {
    /// Unified constructor accepting any [`EntryPointFn`] implementer
    /// (`PartitionEntry` or `PartitionBody`).
    #[inline]
    pub fn from_entry(f: impl EntryPointFn) -> Self {
        let addr = f.into_usize();
        debug_assert!(
            addr <= u32::MAX as usize,
            "EntryAddr::from_entry: address {:#x} exceeds u32::MAX, truncation would occur",
            addr
        );
        Self(addr as u32)
    }
    #[inline]
    pub fn raw(self) -> u32 {
        self.0
    }
}
impl From<PartitionEntry> for EntryAddr {
    fn from(f: PartitionEntry) -> Self {
        Self::from_entry(f)
    }
}
impl From<PartitionBody> for EntryAddr {
    fn from(f: PartitionBody) -> Self {
        Self::from_entry(f)
    }
}
impl From<u32> for EntryAddr {
    fn from(v: u32) -> Self {
        Self(v)
    }
}
impl From<EntryAddr> for u32 {
    fn from(v: EntryAddr) -> Self {
        v.0
    }
}

impl core::fmt::Debug for EntryAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "EntryAddr({:#010x})", self.0)
    }
}

impl PartialEq<u32> for EntryAddr {
    fn eq(&self, other: &u32) -> bool {
        self.0 == *other
    }
}

impl PartialEq<EntryAddr> for u32 {
    fn eq(&self, other: &EntryAddr) -> bool {
        other == self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::empty_loop)]
    extern "C" fn _dummy_entry() -> ! {
        loop {}
    }

    #[allow(clippy::empty_loop)]
    extern "C" fn _dummy_body(_: u32) -> ! {
        loop {}
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_addr_from_fn() {
        let addr = EntryAddr::from_entry(_dummy_entry);
        assert_eq!(addr.raw(), _dummy_entry as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_addr_from_body() {
        let addr = EntryAddr::from_entry(_dummy_body);
        assert_eq!(addr.raw(), _dummy_body as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_raw_round_trip() {
        let raw = 0x0800_1234u32;
        let addr = EntryAddr::from(raw);
        assert_eq!(addr.raw(), raw);
    }

    #[test]
    fn entry_addr_from_u32() {
        let addr = EntryAddr::from(0x0800_0000u32);
        assert_eq!(addr.raw(), 0x0800_0000);
    }

    #[test]
    fn entry_addr_into_u32() {
        let addr = EntryAddr::from(0x2000_0000u32);
        let v: u32 = addr.into();
        assert_eq!(v, 0x2000_0000);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_addr_from_partition_entry() {
        let ep: PartitionEntry = _dummy_entry;
        let addr = EntryAddr::from(ep);
        assert_eq!(addr.raw(), ep as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_addr_from_partition_body() {
        let body: PartitionBody = _dummy_body;
        let addr = EntryAddr::from(body);
        assert_eq!(addr.raw(), body as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_debug_hex() {
        use core::fmt::Write;
        let addr = EntryAddr::from(0x0800_0000u32);
        let mut buf = [0u8; 64];
        let mut cursor = WriteBuf::new(&mut buf);
        write!(cursor, "{:?}", addr).unwrap();
        assert_eq!(cursor.as_str(), "EntryAddr(0x08000000)");
    }

    /// Minimal stack-allocated write buffer for no_std Debug formatting.
    struct WriteBuf<'a> {
        buf: &'a mut [u8],
        pos: usize,
    }
    impl<'a> WriteBuf<'a> {
        fn new(buf: &'a mut [u8]) -> Self {
            Self { buf, pos: 0 }
        }
        fn as_str(&self) -> &str {
            core::str::from_utf8(&self.buf[..self.pos]).unwrap()
        }
    }
    impl core::fmt::Write for WriteBuf<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let end = self.pos + bytes.len();
            if end > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.pos..end].copy_from_slice(bytes);
            self.pos = end;
            Ok(())
        }
    }

    #[test]
    fn entry_addr_partial_eq_u32_both_directions() {
        let addr = EntryAddr::from(0x0800_0100u32);
        assert_eq!(addr, 0x0800_0100u32);
        assert_eq!(0x0800_0100u32, addr);
        assert_ne!(addr, 0x0800_0001u32);
    }

    /// On 64-bit hosts, function addresses exceed `u32::MAX`, so the
    /// `debug_assert!` in `from_entry` must fire to catch truncation.
    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn entry_addr_from_entry_catches_truncation_on_64bit() {
        let _ = EntryAddr::from_entry(_dummy_entry as PartitionEntry);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_point_fn_partition_entry() {
        let ep: PartitionEntry = _dummy_entry;
        let addr = ep.into_entry_addr();
        assert_eq!(addr.raw(), _dummy_entry as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn entry_point_fn_partition_body() {
        let body: PartitionBody = _dummy_body;
        let addr = body.into_entry_addr();
        assert_eq!(addr.raw(), _dummy_body as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn entry_point_fn_entry_catches_truncation_on_64bit() {
        let ep: PartitionEntry = _dummy_entry;
        let _ = ep.into_entry_addr();
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn entry_point_fn_body_catches_truncation_on_64bit() {
        let body: PartitionBody = _dummy_body;
        let _ = body.into_entry_addr();
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn from_entry_with_partition_entry() {
        let ep: PartitionEntry = _dummy_entry;
        let addr = EntryAddr::from_entry(ep);
        assert_eq!(addr.raw(), _dummy_entry as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn from_entry_with_partition_body() {
        let body: PartitionBody = _dummy_body;
        let addr = EntryAddr::from_entry(body);
        assert_eq!(addr.raw(), _dummy_body as *const () as usize as u32);
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn from_entry_entry_catches_truncation_on_64bit() {
        let ep: PartitionEntry = _dummy_entry;
        let _ = EntryAddr::from_entry(ep);
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn from_entry_body_catches_truncation_on_64bit() {
        let body: PartitionBody = _dummy_body;
        let _ = EntryAddr::from_entry(body);
    }

    /// On 64-bit hosts, `PartitionSpec::new()` must trigger the truncation
    /// guard via `EntryAddr::from_entry`.
    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn partition_spec_new_catches_truncation_on_64bit() {
        let _ = PartitionSpec::new(_dummy_entry as PartitionEntry, 0);
    }

    /// `PartitionSpec::new()` now accepts `PartitionBody` directly via
    /// `impl EntryPointFn`.
    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_new_with_partition_body() {
        let body: PartitionBody = _dummy_body;
        let expected_addr = body as *const () as usize as u32;
        let spec = PartitionSpec::new(body, 42);
        assert_eq!(spec.entry_point().raw(), expected_addr);
        assert_eq!(spec.r0(), 42);
    }

    /// On 64-bit hosts, `PartitionSpec::new()` with a `PartitionBody` must
    /// trigger the truncation guard.
    #[cfg(target_pointer_width = "64")]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn spec_new_body_catches_truncation_on_64bit() {
        let body: PartitionBody = _dummy_body;
        let _ = PartitionSpec::new(body, 0);
    }

    // --- MpuRegion::is_mappable tests ---

    #[test]
    fn is_mappable_valid_region() {
        // 4 KiB at aligned base — typical peripheral region
        let r = MpuRegion::new(0x4000_C000, 4096, 0x0300_0000);
        assert!(r.is_mappable());
    }

    #[test]
    fn is_mappable_minimum_size() {
        // 32 bytes is the minimum valid MPU region
        let r = MpuRegion::new(0, 32, 0);
        assert!(r.is_mappable());
    }

    #[test]
    fn is_mappable_rejects_size_too_small() {
        let r = MpuRegion::new(0, 16, 0);
        assert!(!r.is_mappable());
        let r = MpuRegion::new(0, 0, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn is_mappable_rejects_non_power_of_two() {
        let r = MpuRegion::new(0, 48, 0);
        assert!(!r.is_mappable());
        let r = MpuRegion::new(0, 100, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn is_mappable_rejects_misaligned_base() {
        // base 0x100 not aligned to size 4096
        let r = MpuRegion::new(0x0000_0100, 4096, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn is_mappable_rejects_address_overflow() {
        let r = MpuRegion::new(0xFFFF_FF00, 256, 0);
        assert!(!r.is_mappable());
    }

    #[test]
    fn mpu_region_accessors() {
        let r = MpuRegion::new(0x2000_0000, 256, 0x0300_0000);
        assert_eq!(r.base(), 0x2000_0000);
        assert_eq!(r.size(), 256);
        assert_eq!(r.permissions(), 0x0300_0000);
    }

    // --- check_entry_sig! tests ---

    check_entry_sig!(_dummy_entry);
    check_entry_sig!(_dummy_body, body);

    #[test]
    fn check_entry_sig_entry_compiles() {
        // The const assertion above already validates at compile time.
        // This test ensures the macro is usable from within the test module.
        let _: PartitionEntry = _dummy_entry;
    }

    #[test]
    fn check_entry_sig_body_compiles() {
        let _: PartitionBody = _dummy_body;
    }

    // --- PartitionSpec::entry() / body() tests ---

    /// Proves `entry()` accepts a bare function item without an `as` cast.
    /// The coercion from function item → PartitionEntry is validated at
    /// compile time; calling entry() at runtime is tested separately.
    const _ENTRY_COERCE: PartitionEntry = _dummy_entry;

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_entry_accepts_bare_fn_item() {
        // If this compiles, `entry()` accepts a bare fn item with no cast.
        let _ = PartitionSpec::entry(_dummy_entry);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_entry_round_trips_address() {
        let spec = PartitionSpec::entry(_dummy_entry);
        assert_eq!(spec.entry_point().raw(), _dummy_entry as *const () as u32);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_entry_sets_r0_zero() {
        let spec = PartitionSpec::entry(_dummy_entry);
        assert_eq!(spec.r0(), 0);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_body_preserves_r0() {
        let spec = PartitionSpec::body(_dummy_body, 0xCAFE);
        assert_eq!(spec.r0(), 0xCAFE);
        assert_eq!(spec.entry_point().raw(), _dummy_body as *const () as u32);
    }

    /// Verify that `PartitionSpec::entry()` detects truncation when the
    /// address exceeds `u32::MAX`. Uses a transmuted large address so the
    /// test is deterministic (not dependent on linker address assignment).
    #[cfg(target_pointer_width = "64")]
    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn spec_entry_catches_truncation() {
        // SAFETY: we only need an address value that exceeds u32::MAX to
        // trigger the debug_assert!; the function is never actually called.
        let fake: PartitionEntry = unsafe { core::mem::transmute(0x1_0000_0001_usize) };
        let _ = PartitionSpec::entry(fake);
    }

    /// Same as above but for `PartitionSpec::body()`.
    #[cfg(target_pointer_width = "64")]
    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "exceeds u32::MAX")]
    fn spec_body_catches_truncation() {
        // SAFETY: we only need an address value that exceeds u32::MAX to
        // trigger the debug_assert!; the function is never actually called.
        let fake: PartitionBody = unsafe { core::mem::transmute(0x1_0000_0001_usize) };
        let _ = PartitionSpec::body(fake, 0);
    }

    #[test]
    fn fault_policy_default_is_stay_dead() {
        assert_eq!(FaultPolicy::default(), FaultPolicy::StayDead);
    }

    #[test]
    fn fault_policy_variants_are_distinct() {
        let stay = FaultPolicy::StayDead;
        let warm = FaultPolicy::WarmRestart { max: 3 };
        let cold = FaultPolicy::ColdRestart { max: 5 };
        assert_ne!(stay, warm);
        assert_ne!(stay, cold);
        assert_ne!(warm, cold);
        assert_eq!(warm, FaultPolicy::WarmRestart { max: 3 });
        assert_eq!(cold, FaultPolicy::ColdRestart { max: 5 });
    }

    // --- PartitionSpec builder & default tests ---

    /// Helper: build a default PartitionSpec without going through
    /// `EntryPointFn` (works on 64-bit hosts too).
    fn base_spec() -> PartitionSpec {
        PartitionSpec {
            entry_point: EntryAddr::from(0x0800_0000u32),
            r0: 0,
            data_mpu: None,
            code_mpu: None,
            peripherals: &[],
            fault_policy: FaultPolicy::StayDead,
            error_handler: None,
        }
    }

    /// Verify the *production* `PartitionSpec::new` constructor defaults all
    /// optional fields (only runnable on 32-bit targets where EntryAddr
    /// construction from function pointers does not panic).
    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_new_production_defaults_all_optional_fields() {
        let spec = PartitionSpec::new(_dummy_entry as PartitionEntry, 0);
        assert!(spec.data_mpu().is_none());
        assert!(spec.code_mpu().is_none());
        assert!(spec.peripherals().is_empty());
        assert_eq!(spec.fault_policy(), FaultPolicy::StayDead);
        assert!(spec.error_handler().is_none());
    }

    /// Verify that `PartitionSpec::entry()` also defaults the new fields.
    #[cfg(target_pointer_width = "32")]
    #[test]
    fn spec_entry_defaults_all_optional_fields() {
        let spec = PartitionSpec::entry(_dummy_entry);
        assert!(spec.data_mpu().is_none());
        assert!(spec.code_mpu().is_none());
        assert!(spec.peripherals().is_empty());
        assert_eq!(spec.fault_policy(), FaultPolicy::StayDead);
        assert!(spec.error_handler().is_none());
    }

    #[test]
    fn spec_with_data_mpu() {
        let region = MpuRegion::new(0x2000_0000, 4096, 0x03);
        let spec = base_spec().with_data_mpu(region);
        assert_eq!(spec.data_mpu(), Some(region));
    }

    #[test]
    fn spec_with_code_mpu() {
        let region = MpuRegion::new(0x0800_0000, 8192, 0x06);
        let spec = base_spec().with_code_mpu(region);
        assert_eq!(spec.code_mpu(), Some(region));
    }

    static TEST_PERIPHERALS: [MpuRegion; 2] = [
        MpuRegion::new(0x4000_C000, 4096, 0x03),
        MpuRegion::new(0x4002_0000, 4096, 0x03),
    ];

    #[test]
    fn spec_with_peripherals() {
        let spec = base_spec().with_peripherals(&TEST_PERIPHERALS);
        assert_eq!(spec.peripherals().len(), 2);
        assert_eq!(spec.peripherals()[0], TEST_PERIPHERALS[0]);
        assert_eq!(spec.peripherals()[1], TEST_PERIPHERALS[1]);
    }

    #[test]
    fn spec_with_fault_policy() {
        let spec = base_spec().with_fault_policy(FaultPolicy::WarmRestart { max: 3 });
        assert_eq!(spec.fault_policy(), FaultPolicy::WarmRestart { max: 3 });
    }

    #[test]
    fn spec_with_error_handler() {
        let spec = base_spec().with_error_handler(0x0800_2000);
        assert_eq!(spec.error_handler(), Some(0x0800_2000));
    }

    #[test]
    fn spec_with_r0() {
        let spec = base_spec().with_r0(42);
        assert_eq!(spec.r0(), 42);
    }

    #[test]
    fn spec_builder_chaining() {
        let data = MpuRegion::new(0x2000_0000, 4096, 0x03);
        let code = MpuRegion::new(0x0800_0000, 8192, 0x06);
        let spec = base_spec()
            .with_r0(99)
            .with_data_mpu(data)
            .with_code_mpu(code)
            .with_peripherals(&TEST_PERIPHERALS)
            .with_fault_policy(FaultPolicy::ColdRestart { max: 5 })
            .with_error_handler(0x0800_3000);
        assert_eq!(spec.r0(), 99);
        assert_eq!(spec.data_mpu(), Some(data));
        assert_eq!(spec.code_mpu(), Some(code));
        assert_eq!(spec.peripherals().len(), 2);
        assert_eq!(spec.fault_policy(), FaultPolicy::ColdRestart { max: 5 });
        assert_eq!(spec.error_handler(), Some(0x0800_3000));
    }
}
