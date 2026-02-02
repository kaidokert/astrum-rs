/// Fixed-size buffer pool for shared-memory IPC between partitions.
///
/// Each slot contains a byte buffer and ownership/borrow metadata.
/// No MPU interaction — pure data structure for allocation tracking.

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

/// The access mode requested when borrowing a buffer slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorrowMode {
    Read,
    Write,
}

pub struct BufferSlot<const SIZE: usize> {
    data: [u8; SIZE],
    state: BorrowState,
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
        }
    }

    pub fn state(&self) -> BorrowState {
        self.state
    }

    pub fn data(&self) -> &[u8; SIZE] {
        &self.data
    }
}

pub struct BufferPool<const SLOTS: usize, const SIZE: usize> {
    slots: [BufferSlot<SIZE>; SLOTS],
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
                s.state = BorrowState::Free;
                Ok(())
            }
        }
    }

    /// Return a reference to a slot (for inspection/testing).
    pub fn get(&self, slot: usize) -> Option<&BufferSlot<SIZE>> {
        self.slots.get(slot)
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
}
