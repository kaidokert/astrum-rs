//! Fixed-size buffer pool for shared-memory IPC between partitions.
//!
//! Each slot contains a byte buffer and ownership/borrow metadata.
//! When the `dynamic-mpu` feature is enabled, the pool can coordinate
//! with [`crate::mpu_strategy::DynamicStrategy`] to map buffer slots
//! as MPU windows for zero-copy cross-partition lending.

use crate::mpu_strategy::{MpuError, MpuStrategy};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorrowState {
    Free,
    BorrowedRead { owner: u8 },
    BorrowedWrite { owner: u8 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum BufferPoolError {
    InvalidSlot,
    NotBorrowed,
    AlreadyBorrowed,
    NotOwner,
}

/// Errors from MPU-aware buffer lending operations.
#[derive(Debug, PartialEq, Eq)]
pub enum BufferError {
    InvalidSlot,
    SlotNotFree,
    SlotNotBorrowed,
    /// The buffer slot SIZE is not a valid MPU region size (must be
    /// a power-of-two >= 32).
    InvalidSize,
    /// An MPU operation failed — wraps the underlying [`MpuError`].
    Mpu(MpuError),
    /// Caller does not own this buffer slot.
    NotOwner,
    /// Slot is already lent to another partition.
    AlreadyLent,
    /// Slot is not currently lent.
    NotLent,
    /// Cannot lend a buffer to the owning partition itself.
    SelfLend,
}

/// Tracks a cross-partition buffer lending: which partition the buffer was
/// lent to and the MPU region used for the mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LendRecord {
    pub target: u8,
    pub region_id: u8,
    pub writable: bool,
}

/// The access mode requested when borrowing a buffer slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorrowMode {
    Read,
    Write,
}

/// Flags for SYS_BUF_LEND, packed into upper bits of r2.
pub mod lend_flags {
    /// Grant AP_FULL_ACCESS instead of AP_RO_RO to the target.
    pub const WRITABLE: u32 = 1 << 8;
}

/// A single buffer slot containing a fixed-size byte array and metadata.
///
/// `repr(C, align(32))` ensures the `data` field (placed first) is
/// 32-byte aligned, which is the minimum MPU region alignment for
/// SIZE >= 32.  For SIZE > 32 the caller must ensure the containing
/// array also satisfies the larger alignment (e.g. via linker
/// placement or `#[repr(align(…))]` on the pool).
#[repr(C, align(32))]
pub struct BufferSlot<const SIZE: usize> {
    data: [u8; SIZE],
    state: BorrowState,
    /// MPU region ID assigned by `lend_to_partition`, cleared on revoke.
    mpu_region: Option<u8>,
    /// Cross-partition lend tracking, set by share/unshare operations.
    lent_to: Option<LendRecord>,
}

impl<const SIZE: usize> Default for BufferSlot<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SIZE: usize> BufferSlot<SIZE> {
    pub const fn new() -> Self {
        Self {
            data: [0u8; SIZE],
            state: BorrowState::Free,
            mpu_region: None,
            lent_to: None,
        }
    }

    pub fn state(&self) -> BorrowState {
        self.state
    }

    pub fn data(&self) -> &[u8; SIZE] {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut [u8; SIZE] {
        &mut self.data
    }

    /// Return the MPU region ID assigned by `lend_to_partition`, if any.
    pub fn mpu_region(&self) -> Option<u8> {
        self.mpu_region
    }

    /// Return the active lend record, if this slot is lent to another partition.
    pub fn lent_to(&self) -> Option<&LendRecord> {
        self.lent_to.as_ref()
    }
}

pub struct BufferPool<const SLOTS: usize, const SIZE: usize> {
    slots: [BufferSlot<SIZE>; SLOTS],
    /// Per-slot optional deadline (tick count). When set, the slot can be
    /// automatically revoked by [`revoke_expired`] once the current tick
    /// exceeds the deadline.
    deadlines: [Option<u64>; SLOTS],
}

impl<const SLOTS: usize, const SIZE: usize> Default for BufferPool<SLOTS, SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SLOTS: usize, const SIZE: usize> BufferPool<SLOTS, SIZE> {
    /// Create a new buffer pool with all slots free.
    pub const fn new() -> Self {
        Self {
            slots: [const { BufferSlot::new() }; SLOTS],
            deadlines: [None; SLOTS],
        }
    }

    /// Allocate the first free slot, atomically transitioning it to the
    /// requested borrow mode. Returns the slot index or `None` if all
    /// slots are in use.
    pub fn alloc(&mut self, partition_id: u8, mode: BorrowMode) -> Option<usize> {
        let idx = self
            .slots
            .iter()
            .position(|s| s.state == BorrowState::Free)?;
        self.slots[idx].state = match mode {
            BorrowMode::Read => BorrowState::BorrowedRead {
                owner: partition_id,
            },
            BorrowMode::Write => BorrowState::BorrowedWrite {
                owner: partition_id,
            },
        };
        Some(idx)
    }

    /// Borrow a specific slot, assigning ownership to `partition_id`.
    pub fn borrow(
        &mut self,
        slot: usize,
        partition_id: u8,
        mode: BorrowMode,
    ) -> Result<(), BufferPoolError> {
        let s = self
            .slots
            .get_mut(slot)
            .ok_or(BufferPoolError::InvalidSlot)?;
        match s.state {
            BorrowState::Free => {
                s.state = match mode {
                    BorrowMode::Read => BorrowState::BorrowedRead {
                        owner: partition_id,
                    },
                    BorrowMode::Write => BorrowState::BorrowedWrite {
                        owner: partition_id,
                    },
                };
                Ok(())
            }
            _ => Err(BufferPoolError::AlreadyBorrowed),
        }
    }

    /// Release a borrowed slot, returning it to the free state.
    ///
    /// The caller must be the current owner of the slot; otherwise
    /// `NotOwner` is returned.
    pub fn release(&mut self, slot: usize, partition_id: u8) -> Result<(), BufferPoolError> {
        let s = self
            .slots
            .get_mut(slot)
            .ok_or(BufferPoolError::InvalidSlot)?;
        match s.state {
            BorrowState::Free => Err(BufferPoolError::NotBorrowed),
            BorrowState::BorrowedRead { owner } | BorrowState::BorrowedWrite { owner }
                if owner != partition_id =>
            {
                Err(BufferPoolError::NotOwner)
            }
            _ => {
                if s.lent_to.is_some() {
                    return Err(BufferPoolError::AlreadyBorrowed);
                }
                s.state = BorrowState::Free;
                Ok(())
            }
        }
    }

    /// Return a reference to a slot (for inspection/testing).
    pub fn get(&self, slot: usize) -> Option<&BufferSlot<SIZE>> {
        self.slots.get(slot)
    }

    /// Return a mutable reference to a slot.
    pub fn get_mut(&mut self, slot: usize) -> Option<&mut BufferSlot<SIZE>> {
        self.slots.get_mut(slot)
    }

    /// Share a borrowed slot with another partition.
    ///
    /// Accepts `BorrowedWrite` (any `writable` value) or `BorrowedRead`
    /// (only when `writable == false`).  A `BorrowedRead` owner requesting
    /// `writable == true` gets `Err(BufferError::NotOwner)`.
    ///
    /// A [`LendRecord`] is stored in the slot so the mapping can later be
    /// revoked.  Returns the MPU region ID on success.
    pub fn share_with_partition(
        &mut self,
        slot: usize,
        owner: u8,
        target: u8,
        writable: bool,
        strategy: &dyn MpuStrategy,
    ) -> Result<u8, BufferError> {
        if owner == target {
            return Err(BufferError::SelfLend);
        }
        let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
        match s.state {
            BorrowState::BorrowedWrite { owner: o } if o == owner => {}
            BorrowState::BorrowedRead { owner: o } if o == owner => {
                if writable {
                    return Err(BufferError::NotOwner);
                }
            }
            _ => return Err(BufferError::NotOwner),
        }
        if s.lent_to.is_some() {
            return Err(BufferError::AlreadyLent);
        }

        let ap = if writable {
            crate::mpu::AP_FULL_ACCESS
        } else {
            crate::mpu::AP_RO_RO
        };
        let base = s.data.as_ptr() as u32;
        let size_field = crate::mpu::encode_size(SIZE as u32).ok_or(BufferError::InvalidSize)?;
        let rasr = crate::mpu::build_rasr(size_field, ap, true, (false, false, false));

        let region_id = strategy
            .add_window(base, SIZE as u32, rasr, target)
            .map_err(BufferError::Mpu)?;

        s.lent_to = Some(LendRecord {
            target,
            region_id,
            writable,
        });
        Ok(region_id)
    }

    /// Remove the MPU window previously created by
    /// [`share_with_partition`](Self::share_with_partition).
    ///
    /// The slot must be borrowed (`BorrowedWrite` or `BorrowedRead`) by
    /// `owner` and currently lent to `target`.  On success the MPU window
    /// is removed and the `lent_to` field is cleared.
    pub fn unshare_from_partition(
        &mut self,
        slot: usize,
        owner: u8,
        target: u8,
        strategy: &dyn MpuStrategy,
    ) -> Result<(), BufferError> {
        let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
        match s.state {
            BorrowState::BorrowedWrite { owner: o } | BorrowState::BorrowedRead { owner: o }
                if o == owner => {}
            _ => return Err(BufferError::NotOwner),
        }
        let lr = s.lent_to.ok_or(BufferError::NotLent)?;
        if lr.target != target {
            return Err(BufferError::NotOwner);
        }
        strategy.remove_window(lr.region_id);
        s.lent_to = None;
        Ok(())
    }

    /// Lend a buffer slot to a partition, installing an MPU window via `strategy`.
    ///
    /// Uses `AP_RO_RO` when `writable` is false, `AP_FULL_ACCESS` when true.
    /// Returns the MPU region ID (5-7) on success.
    pub fn lend_to_partition(
        &mut self,
        slot: usize,
        partition_id: u8,
        writable: bool,
        strategy: &dyn MpuStrategy,
    ) -> Result<u8, BufferError> {
        let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
        if s.state != BorrowState::Free {
            return Err(BufferError::SlotNotFree);
        }

        let base = s.data.as_ptr() as u32;
        let ap = if writable {
            crate::mpu::AP_FULL_ACCESS
        } else {
            crate::mpu::AP_RO_RO
        };
        let size_field = crate::mpu::encode_size(SIZE as u32).ok_or(BufferError::InvalidSize)?;
        let rasr = crate::mpu::build_rasr(size_field, ap, true, (false, false, false));

        let region_id = strategy
            .add_window(base, SIZE as u32, rasr, partition_id)
            .map_err(BufferError::Mpu)?;

        s.state = if writable {
            BorrowState::BorrowedWrite {
                owner: partition_id,
            }
        } else {
            BorrowState::BorrowedRead {
                owner: partition_id,
            }
        };
        s.mpu_region = Some(region_id);
        Ok(region_id)
    }

    /// Revoke a previously lent buffer slot: remove its MPU window and
    /// return the slot to the free state.
    pub fn revoke_from_partition(
        &mut self,
        slot: usize,
        strategy: &dyn MpuStrategy,
    ) -> Result<(), BufferError> {
        let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
        let region_id = s.mpu_region.ok_or(BufferError::SlotNotBorrowed)?;

        strategy.remove_window(region_id);
        s.state = BorrowState::Free;
        s.mpu_region = None;
        self.deadlines[slot] = None;
        Ok(())
    }

    /// Set (or clear) the deadline for a borrowed slot.
    ///
    /// Returns `Err(BufferError::InvalidSlot)` if the index is out of range,
    /// or `Err(BufferError::SlotNotBorrowed)` if the slot is free.
    pub fn set_deadline(&mut self, slot: usize, deadline: Option<u64>) -> Result<(), BufferError> {
        let s = self.slots.get(slot).ok_or(BufferError::InvalidSlot)?;
        if s.state == BorrowState::Free {
            return Err(BufferError::SlotNotBorrowed);
        }
        self.deadlines[slot] = deadline;
        Ok(())
    }

    /// Return the deadline for the given slot, or `None` if unset/out-of-range.
    pub fn deadline(&self, slot: usize) -> Option<u64> {
        self.deadlines.get(slot).copied().flatten()
    }

    /// Transfer ownership of a borrowed slot to a different partition.
    ///
    /// Preserves the borrow mode (Read/Write) — only the `owner` field in
    /// the `BorrowState` is changed.  No MPU operations, no data movement.
    pub fn transfer_ownership(
        &mut self,
        slot: usize,
        current_owner: u8,
        new_owner: u8,
    ) -> Result<(), BufferError> {
        let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
        let new_state = match s.state {
            BorrowState::Free => return Err(BufferError::SlotNotBorrowed),
            BorrowState::BorrowedRead { owner } | BorrowState::BorrowedWrite { owner }
                if owner != current_owner =>
            {
                return Err(BufferError::NotOwner);
            }
            BorrowState::BorrowedRead { .. } => BorrowState::BorrowedRead { owner: new_owner },
            BorrowState::BorrowedWrite { .. } => BorrowState::BorrowedWrite { owner: new_owner },
        };
        if new_owner == current_owner {
            return Err(BufferError::SelfLend);
        }
        if s.lent_to.is_some() {
            return Err(BufferError::AlreadyLent);
        }
        s.state = new_state;
        Ok(())
    }

    /// Copy data from a borrowed slot's internal buffer into `dst`.
    ///
    /// Both `BorrowedRead` and `BorrowedWrite` owners may read.  The copy
    /// length is `min(dst.len(), slot.data.len())`.  Returns the number of
    /// bytes copied on success.
    pub fn read_from_slot(
        &self,
        slot: usize,
        owner: u8,
        dst: &mut [u8],
    ) -> Result<usize, BufferError> {
        let s = self.slots.get(slot).ok_or(BufferError::InvalidSlot)?;
        match s.state {
            BorrowState::Free => return Err(BufferError::SlotNotBorrowed),
            BorrowState::BorrowedRead { owner: o } | BorrowState::BorrowedWrite { owner: o } => {
                if o != owner {
                    return Err(BufferError::NotOwner);
                }
            }
        }
        let len = dst.len().min(s.data.len());
        let dst_slice = dst.get_mut(..len).ok_or(BufferError::InvalidSlot)?;
        let src_slice = s.data.get(..len).ok_or(BufferError::InvalidSlot)?;
        dst_slice.copy_from_slice(src_slice);
        Ok(len)
    }

    /// Iterate all slots, revoking any whose deadline has passed
    /// (`deadline <= current_tick`). Each revoked slot has its MPU window
    /// removed via `strategy.remove_window`. Slots without a deadline
    /// (`None`) are never revoked.
    ///
    /// Returns the number of slots revoked.
    pub fn revoke_expired(&mut self, current_tick: u64, strategy: &dyn MpuStrategy) -> usize {
        let mut count = 0usize;
        for i in 0..SLOTS {
            let expired = matches!(self.deadlines.get(i), Some(Some(dl)) if *dl <= current_tick);
            if !expired {
                continue;
            }
            if self.revoke_from_partition(i, strategy).is_ok() {
                count += 1;
            }
        }
        count
    }
}

#[rustfmt::skip]
#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn alloc_borrow_release_lifecycle() {
        let mut pool = BufferPool::<3, 32>::new();
        // Alloc atomically claims slot 0
        let idx = pool.alloc(5, BorrowMode::Read).unwrap();
        assert_eq!(idx, 0);
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 5 });
        assert_eq!(*pool.get(0).unwrap().data(), [0u8; 32]);
        // Alloc skips slot 0, claims slot 1
        let idx = pool.alloc(3, BorrowMode::Write).unwrap();
        assert_eq!(idx, 1);
        assert_eq!(pool.get(1).unwrap().state(), BorrowState::BorrowedWrite { owner: 3 });
        // Alloc claims slot 2
        let idx = pool.alloc(0, BorrowMode::Write).unwrap();
        assert_eq!(idx, 2);
        // Pool exhausted
        assert_eq!(pool.alloc(0, BorrowMode::Read), None);
        // Release read-borrowed slot (owner 5), becomes allocatable
        pool.release(0, 5).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::Free);
        let idx = pool.alloc(7, BorrowMode::Write).unwrap();
        assert_eq!(idx, 0);
        // Release write-borrowed slot (owner 3)
        pool.release(1, 3).unwrap();
        assert_eq!(pool.get(1).unwrap().state(), BorrowState::Free);
    }

    #[test] fn borrow_specific_slot() {
        let mut pool = BufferPool::<2, 16>::new();
        pool.borrow(0, 1, BorrowMode::Read).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 1 });
        pool.borrow(1, 2, BorrowMode::Write).unwrap();
        assert_eq!(pool.get(1).unwrap().state(), BorrowState::BorrowedWrite { owner: 2 });
    }

    #[test] fn double_borrow_rejected() {
        let mut pool = BufferPool::<2, 16>::new();
        // read then read
        pool.borrow(0, 1, BorrowMode::Read).unwrap();
        assert_eq!(pool.borrow(0, 2, BorrowMode::Read), Err(BufferPoolError::AlreadyBorrowed));
        // read then write
        assert_eq!(pool.borrow(0, 2, BorrowMode::Write), Err(BufferPoolError::AlreadyBorrowed));
        // write then write
        pool.borrow(1, 1, BorrowMode::Write).unwrap();
        assert_eq!(pool.borrow(1, 2, BorrowMode::Write), Err(BufferPoolError::AlreadyBorrowed));
        // write then read
        assert_eq!(pool.borrow(1, 2, BorrowMode::Read), Err(BufferPoolError::AlreadyBorrowed));
    }

    #[test] fn release_checks_ownership() {
        let mut pool = BufferPool::<2, 16>::new();
        pool.borrow(0, 1, BorrowMode::Read).unwrap();
        pool.borrow(1, 2, BorrowMode::Write).unwrap();
        // Wrong owner cannot release
        assert_eq!(pool.release(0, 99), Err(BufferPoolError::NotOwner));
        assert_eq!(pool.release(1, 1), Err(BufferPoolError::NotOwner));
        // Correct owner can release
        pool.release(0, 1).unwrap();
        pool.release(1, 2).unwrap();
    }

    #[test] fn release_when_free_and_invalid_slots() {
        let mut pool = BufferPool::<2, 16>::new();
        // Release a free slot is an error
        assert_eq!(pool.release(0, 0), Err(BufferPoolError::NotBorrowed));
        // Invalid slot index for all operations
        assert_eq!(pool.borrow(5, 0, BorrowMode::Read), Err(BufferPoolError::InvalidSlot));
        assert_eq!(pool.borrow(5, 0, BorrowMode::Write), Err(BufferPoolError::InvalidSlot));
        assert_eq!(pool.release(5, 0), Err(BufferPoolError::InvalidSlot));
    }

    #[test] fn reuse_slot_after_release() {
        let mut pool = BufferPool::<1, 16>::new();
        pool.borrow(0, 1, BorrowMode::Write).unwrap();
        pool.release(0, 1).unwrap();
        // Can borrow again with different mode and owner
        pool.borrow(0, 2, BorrowMode::Read).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 2 });
    }

    // ------------------------------------------------------------------
    // MPU window integration (lend / revoke via DynamicStrategy)
    // ------------------------------------------------------------------

    use crate::mpu_strategy::DynamicStrategy;
    use crate::mpu::{AP_FULL_ACCESS, AP_RO_RO, RASR_AP_SHIFT, RASR_AP_MASK};

    #[test] fn lend_read_only_sets_ap_ro_ro() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();
        let rid = pool.lend_to_partition(0, 1, false, &ds).unwrap();
        assert_eq!(rid, 5); // First dynamic window → R5

        // Slot should be BorrowedRead with correct owner.
        let s = pool.get(0).unwrap();
        assert_eq!(s.state(), BorrowState::BorrowedRead { owner: 1 });
        assert_eq!(s.mpu_region(), Some(5));

        // Verify the MPU window has AP_RO_RO.
        let desc = ds.slot(rid).expect("window should exist");
        let ap = (desc.permissions >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_RO_RO);
        assert_eq!(desc.owner, 1);
        assert_eq!(desc.size, 32);
    }

    #[test] fn lend_writable_sets_ap_full_access() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();
        let rid = pool.lend_to_partition(0, 2, true, &ds).unwrap();
        assert_eq!(rid, 5);

        let s = pool.get(0).unwrap();
        assert_eq!(s.state(), BorrowState::BorrowedWrite { owner: 2 });
        assert_eq!(s.mpu_region(), Some(5));

        let desc = ds.slot(rid).unwrap();
        let ap = (desc.permissions >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_FULL_ACCESS);
        assert_eq!(desc.owner, 2);
        assert_eq!(desc.size, 32);
    }

    #[test] fn lend_revoke_lifecycle() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();

        // Lend slot 0 read-only to partition 1.
        let rid = pool.lend_to_partition(0, 1, false, &ds).unwrap();
        assert_eq!(rid, 5);
        assert!(ds.slot(5).is_some());

        // Revoke removes MPU window and frees the slot.
        pool.revoke_from_partition(0, &ds).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::Free);
        assert_eq!(pool.get(0).unwrap().mpu_region(), None);
        assert!(ds.slot(5).is_none());
    }

    #[test] fn lend_multiple_slots_allocates_sequential_regions() {
        let mut pool = BufferPool::<3, 32>::new();
        let ds = DynamicStrategy::new();

        let r0 = pool.lend_to_partition(0, 1, false, &ds).unwrap();
        let r1 = pool.lend_to_partition(1, 2, true, &ds).unwrap();
        let r2 = pool.lend_to_partition(2, 3, false, &ds).unwrap();
        assert_eq!((r0, r1, r2), (5, 6, 7));

        // All three windows should be populated.
        assert!(ds.slot(5).is_some());
        assert!(ds.slot(6).is_some());
        assert!(ds.slot(7).is_some());
    }

    #[test] fn lend_exhaustion_returns_mpu_window_exhausted() {
        let mut pool = BufferPool::<4, 32>::new();
        let ds = DynamicStrategy::new();

        // Fill all 3 dynamic window slots (R5-R7).
        pool.lend_to_partition(0, 1, false, &ds).unwrap();
        pool.lend_to_partition(1, 2, true, &ds).unwrap();
        pool.lend_to_partition(2, 3, false, &ds).unwrap();

        // Fourth lend should fail — no MPU windows left.
        assert_eq!(
            pool.lend_to_partition(3, 4, true, &ds),
            Err(BufferError::Mpu(MpuError::SlotExhausted)),
        );
        // Slot 3 should remain free.
        assert_eq!(pool.get(3).unwrap().state(), BorrowState::Free);
    }

    #[test] fn lend_already_borrowed_slot_fails() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, false, &ds).unwrap();
        assert_eq!(
            pool.lend_to_partition(0, 2, true, &ds),
            Err(BufferError::SlotNotFree),
        );
    }

    #[test] fn revoke_unborrowed_slot_fails() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(
            pool.revoke_from_partition(0, &ds),
            Err(BufferError::SlotNotBorrowed),
        );
    }

    #[test] fn revoke_invalid_slot_fails() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(
            pool.revoke_from_partition(5, &ds),
            Err(BufferError::InvalidSlot),
        );
    }

    #[test] fn lend_invalid_slot_fails() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(
            pool.lend_to_partition(5, 1, false, &ds),
            Err(BufferError::InvalidSlot),
        );
    }

    #[test] fn lend_invalid_size_returns_error() {
        // SIZE=16 is below the 32-byte MPU minimum → encode_size returns None.
        let mut pool = BufferPool::<1, 16>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(
            pool.lend_to_partition(0, 1, false, &ds),
            Err(BufferError::InvalidSize),
        );
    }

    #[test] fn lend_revoke_relend_reuses_region() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();

        let r1 = pool.lend_to_partition(0, 1, false, &ds).unwrap();
        assert_eq!(r1, 5);
        pool.revoke_from_partition(0, &ds).unwrap();

        // After revoke, slot is free and region is available again.
        let r2 = pool.lend_to_partition(0, 2, true, &ds).unwrap();
        assert_eq!(r2, 5); // Same region reused.
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedWrite { owner: 2 });
    }

    #[test] fn revoke_middle_slot_frees_region_for_reuse() {
        let mut pool = BufferPool::<3, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, false, &ds).unwrap(); // R5
        pool.lend_to_partition(1, 2, true, &ds).unwrap();  // R6
        pool.lend_to_partition(2, 3, false, &ds).unwrap(); // R7

        // Revoke slot 1 (R6), freeing the middle region.
        pool.revoke_from_partition(1, &ds).unwrap();
        assert!(ds.slot(6).is_none());
        assert!(ds.slot(5).is_some());
        assert!(ds.slot(7).is_some());

        // Re-lend slot 1 on the same pool — should reuse R6.
        let r = pool.lend_to_partition(1, 4, true, &ds).unwrap();
        assert_eq!(r, 6);
        assert_eq!(pool.get(1).unwrap().state(), BorrowState::BorrowedWrite { owner: 4 });
        assert_eq!(pool.get(1).unwrap().mpu_region(), Some(6));
    }

    // ------------------------------------------------------------------
    // Deadline & revoke_expired tests
    // ------------------------------------------------------------------

    #[test] fn no_deadline_not_revoked() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, false, &ds).unwrap();
        // No deadline set — revoke_expired should skip it.
        assert_eq!(pool.revoke_expired(1000, &ds), 0);
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 1 });
        assert!(ds.slot(5).is_some());
    }

    #[test] fn future_deadline_not_revoked() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, true, &ds).unwrap();
        pool.set_deadline(0, Some(500)).unwrap();
        // current_tick=100 < deadline=500 — should not revoke.
        assert_eq!(pool.revoke_expired(100, &ds), 0);
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedWrite { owner: 1 });
        assert!(ds.slot(5).is_some());
        assert_eq!(pool.deadline(0), Some(500));
    }

    #[test] fn expired_deadline_revoked_with_mpu_cleanup() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();

        let rid = pool.lend_to_partition(0, 1, false, &ds).unwrap();
        assert_eq!(rid, 5);
        pool.set_deadline(0, Some(50)).unwrap();

        // current_tick=50 == deadline=50 → expired (deadline <= current_tick).
        assert_eq!(pool.revoke_expired(50, &ds), 1);

        // Slot freed, MPU window removed, deadline cleared.
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::Free);
        assert_eq!(pool.get(0).unwrap().mpu_region(), None);
        assert!(ds.slot(5).is_none());
        assert_eq!(pool.deadline(0), None);
    }

    #[test] fn mixed_deadlines_only_expired_revoked() {
        let mut pool = BufferPool::<4, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, false, &ds).unwrap(); // R5
        pool.lend_to_partition(1, 2, true, &ds).unwrap();  // R6
        pool.lend_to_partition(2, 3, false, &ds).unwrap(); // R7

        // Slot 0: deadline in the past → should be revoked
        pool.set_deadline(0, Some(10)).unwrap();
        // Slot 1: no deadline → should NOT be revoked
        // Slot 2: deadline in the future → should NOT be revoked
        pool.set_deadline(2, Some(200)).unwrap();

        let revoked = pool.revoke_expired(100, &ds);
        assert_eq!(revoked, 1);

        // Slot 0: revoked
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::Free);
        assert!(ds.slot(5).is_none());
        assert_eq!(pool.deadline(0), None);

        // Slot 1: untouched (no deadline)
        assert_eq!(pool.get(1).unwrap().state(), BorrowState::BorrowedWrite { owner: 2 });
        assert!(ds.slot(6).is_some());

        // Slot 2: untouched (future deadline)
        assert_eq!(pool.get(2).unwrap().state(), BorrowState::BorrowedRead { owner: 3 });
        assert!(ds.slot(7).is_some());
        assert_eq!(pool.deadline(2), Some(200));
    }

    #[test] fn set_deadline_invalid_slot() {
        let mut pool = BufferPool::<1, 32>::new();
        assert_eq!(pool.set_deadline(5, Some(100)), Err(BufferError::InvalidSlot));
    }

    #[test] fn set_deadline_free_slot() {
        let mut pool = BufferPool::<1, 32>::new();
        assert_eq!(pool.set_deadline(0, Some(100)), Err(BufferError::SlotNotBorrowed));
    }

    #[test] fn revoke_from_partition_clears_deadline() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();

        pool.lend_to_partition(0, 1, false, &ds).unwrap();
        pool.set_deadline(0, Some(999)).unwrap();
        assert_eq!(pool.deadline(0), Some(999));

        // Manual revoke should also clear the deadline.
        pool.revoke_from_partition(0, &ds).unwrap();
        assert_eq!(pool.deadline(0), None);
    }

    // ------------------------------------------------------------------
    // LendRecord & new BufferError variants
    // ------------------------------------------------------------------

    #[test] fn new_slot_lent_to_is_none() {
        let slot = BufferSlot::<32>::new();
        assert_eq!(slot.lent_to(), None);
    }

    #[test] fn new_pool_slots_lent_to_is_none() {
        let pool = BufferPool::<3, 32>::new();
        for i in 0..3 {
            assert_eq!(pool.get(i).unwrap().lent_to(), None);
        }
    }

    #[test] fn lend_record_equality() {
        let a = LendRecord { target: 1, region_id: 5, writable: false };
        let b = LendRecord { target: 1, region_id: 5, writable: false };
        let c = LendRecord { target: 2, region_id: 5, writable: false };
        let d = LendRecord { target: 1, region_id: 5, writable: true };
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_ne!(a, d);
    }

    #[test] fn buffer_error_new_variants_are_distinct() {
        let variants: [BufferError; 4] = [
            BufferError::NotOwner,
            BufferError::AlreadyLent,
            BufferError::NotLent,
            BufferError::SelfLend,
        ];
        for i in 0..variants.len() {
            for j in (i + 1)..variants.len() {
                assert_ne!(variants[i], variants[j]);
            }
        }
    }

    // ------------------------------------------------------------------
    // share_with_partition tests
    // ------------------------------------------------------------------

    #[test] fn share_success_sets_ap_ro_ro_and_lend_record() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap(); // slot 0, owner 1
        let rid = pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        // MPU window created with AP_RO_RO for the *target* partition.
        let desc = ds.slot(rid).expect("window should exist");
        let ap = (desc.permissions >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_RO_RO);
        assert_eq!(desc.owner, 2); // target owns the MPU window
        assert_eq!(desc.size, 32);
        // Slot state unchanged — still BorrowedWrite by owner 1.
        let s = pool.get(0).unwrap();
        assert_eq!(s.state(), BorrowState::BorrowedWrite { owner: 1 });
        // LendRecord populated correctly.
        let lr = s.lent_to().expect("lent_to should be set");
        assert_eq!(lr.target, 2);
        assert_eq!(lr.region_id, rid);
        assert!(!lr.writable);
    }

    #[test] fn share_writable_sets_ap_full_access_and_lend_record() {
        let mut pool = BufferPool::<2, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap(); // slot 0, owner 1
        let rid = pool.share_with_partition(0, 1, 2, true, &ds).unwrap();
        // MPU window created with AP_FULL_ACCESS for the *target* partition.
        let desc = ds.slot(rid).expect("window should exist");
        let ap = (desc.permissions >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_FULL_ACCESS);
        assert_eq!(desc.owner, 2);
        assert_eq!(desc.size, 32);
        // Slot state unchanged — still BorrowedWrite by owner 1.
        let s = pool.get(0).unwrap();
        assert_eq!(s.state(), BorrowState::BorrowedWrite { owner: 1 });
        // LendRecord populated correctly with writable=true.
        let lr = s.lent_to().expect("lent_to should be set");
        assert_eq!(lr.target, 2);
        assert_eq!(lr.region_id, rid);
        assert!(lr.writable);
    }

    #[test] fn share_not_owner_wrong_partition() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        assert_eq!(pool.share_with_partition(0, 99, 2, false, &ds), Err(BufferError::NotOwner));
    }

    #[test] fn share_read_mode_owner_writable_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Read).unwrap(); // BorrowedRead
        // writable=true from BorrowedRead is not allowed.
        assert_eq!(pool.share_with_partition(0, 1, 2, true, &ds), Err(BufferError::NotOwner));
    }

    #[test] fn share_self_lend_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        assert_eq!(pool.share_with_partition(0, 1, 1, false, &ds), Err(BufferError::SelfLend));
    }

    #[test] fn share_already_lent_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert_eq!(pool.share_with_partition(0, 1, 3, false, &ds), Err(BufferError::AlreadyLent));
    }

    #[test] fn share_free_slot_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(pool.share_with_partition(0, 1, 2, false, &ds), Err(BufferError::NotOwner));
    }

    #[test] fn share_invalid_slot_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(pool.share_with_partition(5, 1, 2, false, &ds), Err(BufferError::InvalidSlot));
    }

    // ------------------------------------------------------------------
    // unshare_from_partition tests
    // ------------------------------------------------------------------

    #[test] fn unshare_success_removes_window_and_clears_lend() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        let rid = pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert!(ds.slot(rid).is_some());
        pool.unshare_from_partition(0, 1, 2, &ds).unwrap();
        // MPU window removed.
        assert!(ds.slot(rid).is_none());
        // lent_to cleared.
        assert_eq!(pool.get(0).unwrap().lent_to(), None);
        // Slot remains BorrowedWrite by owner.
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedWrite { owner: 1 });
    }

    #[test] fn unshare_not_owner_wrong_partition() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert_eq!(pool.unshare_from_partition(0, 99, 2, &ds), Err(BufferError::NotOwner));
    }

    #[test] fn unshare_not_lent() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        // Slot is BorrowedWrite but not lent.
        assert_eq!(pool.unshare_from_partition(0, 1, 2, &ds), Err(BufferError::NotLent));
    }

    #[test] fn unshare_wrong_target() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        // Lent to 2, but unshare says 3.
        assert_eq!(pool.unshare_from_partition(0, 1, 3, &ds), Err(BufferError::NotOwner));
    }

    #[test] fn unshare_invalid_slot() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        assert_eq!(pool.unshare_from_partition(5, 1, 2, &ds), Err(BufferError::InvalidSlot));
    }

    #[test] fn share_read_mode_owner_readonly_success() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Read).unwrap(); // BorrowedRead
        let rid = pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        // MPU window created with AP_RO_RO for the target.
        let desc = ds.slot(rid).expect("window should exist");
        let ap = (desc.permissions >> RASR_AP_SHIFT) & RASR_AP_MASK;
        assert_eq!(ap, AP_RO_RO);
        assert_eq!(desc.owner, 2);
        // Slot remains BorrowedRead by owner 1.
        let s = pool.get(0).unwrap();
        assert_eq!(s.state(), BorrowState::BorrowedRead { owner: 1 });
        let lr = s.lent_to().expect("lent_to should be set");
        assert_eq!(lr.target, 2);
        assert_eq!(lr.region_id, rid);
        assert!(!lr.writable);
    }

    #[test] fn unshare_read_mode_owner_success() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Read).unwrap(); // BorrowedRead
        let rid = pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert!(ds.slot(rid).is_some());
        pool.unshare_from_partition(0, 1, 2, &ds).unwrap();
        // MPU window removed.
        assert!(ds.slot(rid).is_none());
        // lent_to cleared.
        assert_eq!(pool.get(0).unwrap().lent_to(), None);
        // Slot remains BorrowedRead by owner (not corrupted).
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 1 });
    }

    // ------------------------------------------------------------------
    // release guard: reject release while lent
    // ------------------------------------------------------------------

    #[test] fn release_while_lent_rejected() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert_eq!(pool.release(0, 1), Err(BufferPoolError::AlreadyBorrowed));
    }

    #[test] fn release_after_unshare_succeeds() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        pool.unshare_from_partition(0, 1, 2, &ds).unwrap();
        pool.release(0, 1).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::Free);
    }

    // ------------------------------------------------------------------
    // transfer_ownership tests
    // ------------------------------------------------------------------

    #[test] fn transfer_ownership_preserves_borrowed_read() {
        let mut pool = BufferPool::<2, 32>::new();
        pool.alloc(1, BorrowMode::Read).unwrap(); // slot 0, BorrowedRead { owner: 1 }
        pool.transfer_ownership(0, 1, 2).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedRead { owner: 2 });
    }

    #[test] fn transfer_ownership_preserves_borrowed_write() {
        let mut pool = BufferPool::<2, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap(); // slot 0, BorrowedWrite { owner: 1 }
        pool.transfer_ownership(0, 1, 3).unwrap();
        assert_eq!(pool.get(0).unwrap().state(), BorrowState::BorrowedWrite { owner: 3 });
    }

    #[test] fn transfer_ownership_invalid_slot() {
        let mut pool = BufferPool::<1, 32>::new();
        assert_eq!(pool.transfer_ownership(5, 1, 2), Err(BufferError::InvalidSlot));
    }

    #[test] fn transfer_ownership_free_slot() {
        let mut pool = BufferPool::<1, 32>::new();
        assert_eq!(pool.transfer_ownership(0, 1, 2), Err(BufferError::SlotNotBorrowed));
    }

    #[test] fn transfer_ownership_wrong_owner() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        assert_eq!(pool.transfer_ownership(0, 99, 2), Err(BufferError::NotOwner));
    }

    #[test] fn transfer_ownership_self_transfer() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        assert_eq!(pool.transfer_ownership(0, 1, 1), Err(BufferError::SelfLend));
    }

    #[test] fn transfer_ownership_already_lent() {
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        assert_eq!(pool.transfer_ownership(0, 1, 3), Err(BufferError::AlreadyLent));
    }

    // ------------------------------------------------------------------
    // read_from_slot tests
    // ------------------------------------------------------------------

    #[test] fn read_from_slot_invalid_slot() {
        let pool = BufferPool::<2, 32>::new();
        let mut dst = [0u8; 32];
        assert_eq!(pool.read_from_slot(5, 1, &mut dst), Err(BufferError::InvalidSlot));
    }

    #[test] fn read_from_slot_free_slot() {
        let pool = BufferPool::<1, 32>::new();
        let mut dst = [0u8; 32];
        assert_eq!(pool.read_from_slot(0, 1, &mut dst), Err(BufferError::SlotNotBorrowed));
    }

    #[test] fn read_from_slot_wrong_owner() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        pool.get_mut(0).unwrap().data_mut().fill(0xAB);
        let mut dst = [0u8; 32];
        assert_eq!(pool.read_from_slot(0, 99, &mut dst), Err(BufferError::NotOwner));
        // dst must be untouched (all zeros).
        assert_eq!(dst, [0u8; 32]);
    }

    #[test] fn read_from_slot_wrong_owner_borrowed_read() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Read).unwrap();
        pool.get_mut(0).unwrap().data_mut().fill(0xAB);
        let mut dst = [0u8; 32];
        assert_eq!(pool.read_from_slot(0, 99, &mut dst), Err(BufferError::NotOwner));
        // dst must be untouched (all zeros).
        assert_eq!(dst, [0u8; 32]);
    }

    #[test] fn read_from_slot_lent_slot_succeeds() {
        // A slot that is BorrowedWrite + lent_to is still readable by the owner.
        // BorrowState has no Lent variant; lending is tracked via the lent_to field.
        // TODO: reviewer asked about rejecting reads on lent slots — current design
        // allows the owner to read even while the slot is lent (non-destructive).
        let mut pool = BufferPool::<1, 32>::new();
        let ds = DynamicStrategy::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        let pattern: [u8; 32] = core::array::from_fn(|i| i as u8);
        pool.get_mut(0).unwrap().data_mut().copy_from_slice(&pattern);
        pool.share_with_partition(0, 1, 2, false, &ds).unwrap();
        let mut dst = [0u8; 32];
        let n = pool.read_from_slot(0, 1, &mut dst).unwrap();
        assert_eq!(n, 32);
        assert_eq!(dst, pattern);
    }

    #[test] fn read_from_slot_borrowed_write_success() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        let pattern: [u8; 32] = core::array::from_fn(|i| i as u8);
        pool.get_mut(0).unwrap().data_mut().copy_from_slice(&pattern);
        let mut dst = [0u8; 32];
        let n = pool.read_from_slot(0, 1, &mut dst).unwrap();
        assert_eq!(n, 32);
        assert_eq!(dst, pattern);
    }

    #[test] fn read_from_slot_borrowed_read_success() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Read).unwrap();
        let pattern: [u8; 32] = core::array::from_fn(|i| (0xFF - i) as u8);
        pool.get_mut(0).unwrap().data_mut().copy_from_slice(&pattern);
        let mut dst = [0u8; 32];
        let n = pool.read_from_slot(0, 1, &mut dst).unwrap();
        assert_eq!(n, 32);
        assert_eq!(dst, pattern);
    }

    #[test] fn read_from_slot_dst_smaller_than_buffer() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        let pattern: [u8; 32] = core::array::from_fn(|i| i as u8);
        pool.get_mut(0).unwrap().data_mut().copy_from_slice(&pattern);
        let mut dst = [0u8; 8];
        let n = pool.read_from_slot(0, 1, &mut dst).unwrap();
        assert_eq!(n, 8);
        assert_eq!(dst, pattern[..8]);
    }

    #[test] fn read_from_slot_dst_larger_than_buffer() {
        let mut pool = BufferPool::<1, 32>::new();
        pool.alloc(1, BorrowMode::Write).unwrap();
        let pattern: [u8; 32] = core::array::from_fn(|i| i as u8);
        pool.get_mut(0).unwrap().data_mut().copy_from_slice(&pattern);
        let mut dst = [0xFFu8; 64];
        let n = pool.read_from_slot(0, 1, &mut dst).unwrap();
        assert_eq!(n, 32);
        assert_eq!(dst[..32], pattern);
        // Bytes beyond the buffer size remain untouched.
        assert_eq!(dst[32..], [0xFF; 32]);
    }
}
