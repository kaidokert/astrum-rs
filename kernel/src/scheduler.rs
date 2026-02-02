//! Static schedule table for ARINC 653-style partition scheduling.

/// A single slot in the schedule table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduleEntry {
    pub partition_index: u8,
    pub duration_ticks: u32,
    /// Reserved for kernel bottom-half processing when true.
    #[cfg(feature = "dynamic-mpu")]
    pub is_system_window: bool,
}

impl ScheduleEntry {
    pub const fn new(partition_index: u8, duration_ticks: u32) -> Self {
        Self {
            partition_index,
            duration_ticks,
            #[cfg(feature = "dynamic-mpu")]
            is_system_window: false,
        }
    }
}

/// Outcome of a schedule tick when `dynamic-mpu` is enabled.
#[cfg(feature = "dynamic-mpu")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleEvent {
    PartitionSwitch(u8),
    SystemWindow,
    None,
}

/// Fixed-size schedule table with major frame wraparound.
pub struct ScheduleTable<const N: usize> {
    entries: heapless::Vec<ScheduleEntry, N>,
    pub major_frame_ticks: u32,
    current_slot: usize,
    ticks_remaining: u32,
    started: bool,
}

#[allow(clippy::new_without_default)]
impl<const N: usize> ScheduleTable<N> {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
            major_frame_ticks: 0,
            current_slot: 0,
            ticks_remaining: 0,
            started: false,
        }
    }

    /// Returns a slice of the current schedule entries.
    // TODO: reviewer false positive – backing store is heapless::Vec, not MaybeUninit;
    // Deref<Target = [ScheduleEntry]> makes this correct without unsafe.
    pub fn entries(&self) -> &[ScheduleEntry] {
        &self.entries
    }

    /// Returns the number of entries in the schedule table.
    // TODO: reviewer false positive – heapless::Vec::len() already returns the
    // initialized count, not the capacity.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns `true` if the schedule table has no entries.
    // TODO: reviewer false positive – heapless::Vec::is_empty() checks len() == 0,
    // not the backing array capacity.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Add a schedule entry. Returns `Err` if table is full or duration is zero.
    pub fn add(&mut self, entry: ScheduleEntry) -> Result<(), ScheduleEntry> {
        if entry.duration_ticks == 0 {
            return Err(entry);
        }
        self.entries.push(entry)?;
        self.major_frame_ticks = self.major_frame_ticks.saturating_add(entry.duration_ticks);
        Ok(())
    }

    /// Add a system window entry. Returns `Err` if table is full or duration is zero.
    #[cfg(feature = "dynamic-mpu")]
    pub fn add_system_window(&mut self, duration_ticks: u32) -> Result<(), ScheduleEntry> {
        let entry = ScheduleEntry {
            partition_index: 0,
            duration_ticks,
            is_system_window: true,
        };
        self.add(entry)
    }

    /// Reset to the first slot. Call after adding all entries.
    pub fn start(&mut self) {
        self.current_slot = 0;
        self.ticks_remaining = self.entries.first().map_or(0, |e| e.duration_ticks);
        self.started = true;
    }

    pub fn current_partition(&self) -> Option<u8> {
        self.entries
            .get(self.current_slot)
            .map(|e| e.partition_index)
    }

    /// Move to the next slot (with wraparound) and reload `ticks_remaining`.
    /// Returns the index into `self.entries` of the new slot.
    fn step_to_next_slot(&mut self) -> usize {
        self.current_slot += 1;
        if self.current_slot >= self.entries.len() {
            self.current_slot = 0;
        }
        self.ticks_remaining = self.entries[self.current_slot].duration_ticks;
        self.current_slot
    }

    /// Immediately advance to the next schedule slot, forfeiting any
    /// remaining ticks in the current slot.  Returns the new partition
    /// index, or `None` if the table is not started or empty.
    #[cfg(not(feature = "dynamic-mpu"))]
    pub fn force_advance(&mut self) -> Option<u8> {
        if !self.started || self.entries.is_empty() {
            return None;
        }
        let idx = self.step_to_next_slot();
        Some(self.entries[idx].partition_index)
    }

    /// Advance by one tick. Returns `Some(partition_index)` on slot change.
    #[cfg(not(feature = "dynamic-mpu"))]
    pub fn advance_tick(&mut self) -> Option<u8> {
        if !self.started || self.entries.is_empty() {
            return None;
        }
        self.ticks_remaining = self.ticks_remaining.saturating_sub(1);
        if self.ticks_remaining == 0 {
            let idx = self.step_to_next_slot();
            return Some(self.entries[idx].partition_index);
        }
        None
    }

    /// Immediately advance to the next schedule slot, forfeiting any
    /// remaining ticks in the current slot.  Returns a [`ScheduleEvent`]
    /// describing the new slot, or [`ScheduleEvent::None`] if not started
    /// or empty.
    #[cfg(feature = "dynamic-mpu")]
    pub fn force_advance(&mut self) -> ScheduleEvent {
        if !self.started || self.entries.is_empty() {
            return ScheduleEvent::None;
        }
        let idx = self.step_to_next_slot();
        let entry = &self.entries[idx];
        if entry.is_system_window {
            return ScheduleEvent::SystemWindow;
        }
        ScheduleEvent::PartitionSwitch(entry.partition_index)
    }

    /// Advance by one tick. Returns a [`ScheduleEvent`] that distinguishes
    /// partition switches from system window slots.
    #[cfg(feature = "dynamic-mpu")]
    pub fn advance_tick(&mut self) -> ScheduleEvent {
        if !self.started || self.entries.is_empty() {
            return ScheduleEvent::None;
        }
        self.ticks_remaining = self.ticks_remaining.saturating_sub(1);
        if self.ticks_remaining == 0 {
            let idx = self.step_to_next_slot();
            let entry = &self.entries[idx];
            if entry.is_system_window {
                return ScheduleEvent::SystemWindow;
            }
            return ScheduleEvent::PartitionSwitch(entry.partition_index);
        }
        ScheduleEvent::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(not(feature = "dynamic-mpu"))]
    fn table(slots: &[(u8, u32)]) -> ScheduleTable<4> {
        let mut t = ScheduleTable::new();
        for &(p, d) in slots {
            t.add(ScheduleEntry::new(p, d)).unwrap();
        }
        t.start();
        t
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn empty_table_returns_none() {
        let mut t = table(&[]);
        assert!(t.current_partition().is_none());
        assert_eq!(t.advance_tick(), None);
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn single_partition_wraps() {
        let mut t = table(&[(0, 3)]);
        assert_eq!(t.current_partition(), Some(0));
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(0));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn two_partitions_alternate() {
        let mut t = table(&[(0, 2), (1, 3)]);
        assert_eq!(t.major_frame_ticks, 5);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(1));
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(0));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn major_frame_wraparound_sequence() {
        let mut t = table(&[(0, 1), (1, 1), (2, 1)]);
        for &exp in &[1u8, 2, 0, 1, 2, 0] {
            assert_eq!(t.advance_tick(), Some(exp));
        }
    }

    #[test]
    fn table_full_rejects() {
        let mut t: ScheduleTable<2> = ScheduleTable::new();
        assert!(t.add(ScheduleEntry::new(0, 10)).is_ok());
        assert!(t.add(ScheduleEntry::new(1, 10)).is_ok());
        assert!(t.add(ScheduleEntry::new(2, 10)).is_err());
        assert_eq!(t.major_frame_ticks, 20);
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn advance_tick_before_start_is_noop() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        t.add(ScheduleEntry::new(0, 5)).unwrap();
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        // After start, it should work normally
        t.start();
        assert_eq!(t.current_partition(), Some(0));
    }

    #[test]
    fn entries_returns_added_entries() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
        assert_eq!(t.entries(), &[]);

        t.add(ScheduleEntry::new(0, 10)).unwrap();
        t.add(ScheduleEntry::new(1, 20)).unwrap();
        t.add(ScheduleEntry::new(2, 5)).unwrap();

        assert!(!t.is_empty());
        assert_eq!(t.len(), 3);

        let entries = t.entries();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0], ScheduleEntry::new(0, 10));
        assert_eq!(entries[1], ScheduleEntry::new(1, 20));
        assert_eq!(entries[2], ScheduleEntry::new(2, 5));
    }

    #[test]
    fn zero_duration_rejected() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        assert!(t.add(ScheduleEntry::new(0, 0)).is_err());
        assert_eq!(t.major_frame_ticks, 0);
        assert!(t.entries.is_empty());
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn force_advance_mid_slot() {
        let mut t = table(&[(0, 5), (1, 3)]);
        // Consume 2 of 5 ticks in slot 0
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        // Force advance skips remaining 3 ticks
        assert_eq!(t.force_advance(), Some(1));
        // ticks_remaining should be reset to slot 1's duration (3)
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(0)); // wraps back
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn force_advance_at_slot_boundary() {
        let mut t = table(&[(0, 2), (1, 2)]);
        // Exhaust slot 0 via advance_tick (switches to slot 1)
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(1));
        // Now at slot 1 with full ticks_remaining; force advance to slot 0
        assert_eq!(t.force_advance(), Some(0));
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(1));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn force_advance_single_entry_wraps() {
        let mut t = table(&[(7, 4)]);
        // Single entry: force advance wraps to same slot
        assert_eq!(t.force_advance(), Some(7));
        // ticks_remaining should be fully reloaded
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(7));
    }

    #[test]
    #[cfg(not(feature = "dynamic-mpu"))]
    fn force_advance_unstarted_returns_none() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        t.add(ScheduleEntry::new(0, 5)).unwrap();
        // Not started
        assert_eq!(t.force_advance(), None);
        // Empty table
        let mut t2: ScheduleTable<4> = ScheduleTable::new();
        t2.start();
        assert_eq!(t2.force_advance(), None);
    }

    #[cfg(feature = "dynamic-mpu")]
    mod dynamic_mpu_tests {
        use super::*;

        /// Build a started schedule table from `(partition_index, duration)` pairs.
        fn table(slots: &[(u8, u32)]) -> ScheduleTable<8> {
            let mut t = ScheduleTable::new();
            for &(p, d) in slots {
                t.add(ScheduleEntry::new(p, d)).unwrap();
            }
            t.start();
            t
        }

        /// Build a started schedule table that can include system-window slots.
        /// Pass `None` for a system window or `Some(partition_index)` for a
        /// normal partition slot; the second element is the duration.
        fn table_with_windows(slots: &[(Option<u8>, u32)]) -> ScheduleTable<8> {
            let mut t = ScheduleTable::new();
            for &(p, d) in slots {
                match p {
                    Some(idx) => t.add(ScheduleEntry::new(idx, d)).unwrap(),
                    None => t.add_system_window(d).unwrap(),
                }
            }
            t.start();
            t
        }

        #[test]
        fn system_window_entry_defaults() {
            let e = ScheduleEntry::new(0, 10);
            assert!(!e.is_system_window);
        }

        #[test]
        fn add_system_window_and_rejection() {
            let mut t: ScheduleTable<4> = ScheduleTable::new();
            assert!(t.add_system_window(5).is_ok());
            assert_eq!(t.major_frame_ticks, 5);
            assert!(t.entries[0].is_system_window);
            assert_eq!(t.entries[0].duration_ticks, 5);
            assert!(t.add_system_window(0).is_err()); // zero duration
            let mut t2: ScheduleTable<2> = ScheduleTable::new();
            assert!(t2.add(ScheduleEntry::new(0, 5)).is_ok());
            assert!(t2.add_system_window(3).is_ok());
            assert!(t2.add_system_window(2).is_err()); // full
        }

        #[test]
        fn advance_tick_edge_cases() {
            let mut t: ScheduleTable<4> = ScheduleTable::new();
            t.start();
            assert_eq!(t.advance_tick(), ScheduleEvent::None); // empty
            let mut t2: ScheduleTable<4> = ScheduleTable::new();
            t2.add(ScheduleEntry::new(0, 5)).unwrap();
            assert_eq!(t2.advance_tick(), ScheduleEvent::None); // not started
        }

        #[test]
        fn advance_tick_partition_only() {
            let mut t = table(&[(0, 2), (1, 2)]);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
        }

        #[test]
        fn mixed_partition_and_system_window() {
            let mut t = table_with_windows(&[(Some(0), 2), (None, 1), (Some(1), 2)]);
            assert_eq!(t.major_frame_ticks, 5);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::SystemWindow);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
            // Second major frame
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::SystemWindow);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
        }

        #[test]
        fn consecutive_system_windows() {
            let mut t = table_with_windows(&[(Some(0), 1), (None, 1), (None, 1), (Some(1), 1)]);
            assert_eq!(t.advance_tick(), ScheduleEvent::SystemWindow);
            assert_eq!(t.advance_tick(), ScheduleEvent::SystemWindow);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
        }

        #[test]
        fn current_partition_during_system_window() {
            let mut t = table_with_windows(&[(Some(0), 1), (None, 1)]);
            assert_eq!(t.current_partition(), Some(0));
            t.advance_tick(); // into system window
            assert_eq!(t.current_partition(), Some(0));
        }

        #[test]
        fn force_advance_mid_slot() {
            let mut t = table(&[(0, 5), (1, 3)]);
            // Consume 2 of 5 ticks in slot 0
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            // Force advance skips remaining 3 ticks
            assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(1));
            // ticks_remaining reset to 3
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
        }

        #[test]
        fn force_advance_at_slot_boundary() {
            let mut t = table(&[(0, 2), (1, 2)]);
            // Exhaust slot 0 via advance_tick
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
            // Force advance from fresh slot 1 to slot 0
            assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(0));
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
        }

        #[test]
        fn force_advance_single_entry_wraps() {
            let mut t = table(&[(7, 4)]);
            assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(7));
            // ticks_remaining fully reloaded
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(7));
        }

        #[test]
        fn force_advance_unstarted_returns_none() {
            let mut t: ScheduleTable<4> = ScheduleTable::new();
            t.add(ScheduleEntry::new(0, 5)).unwrap();
            assert_eq!(t.force_advance(), ScheduleEvent::None);
            let mut t2: ScheduleTable<4> = ScheduleTable::new();
            t2.start();
            assert_eq!(t2.force_advance(), ScheduleEvent::None);
        }

        #[test]
        fn force_advance_into_system_window() {
            let mut t = table_with_windows(&[(Some(0), 3), (None, 2)]);
            // Force advance from partition slot into system window
            assert_eq!(t.force_advance(), ScheduleEvent::SystemWindow);
            // ticks_remaining should be 2
            assert_eq!(t.advance_tick(), ScheduleEvent::None);
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
        }
    }
}
