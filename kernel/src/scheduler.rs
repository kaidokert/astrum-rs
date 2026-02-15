//! Static schedule table for ARINC 653-style partition scheduling.
//!
//! # State-Agnostic Scheduling
//!
//! The [`ScheduleTable`] returns partition IDs unconditionally, without
//! checking [`PartitionState`](crate::partition::PartitionState). This is
//! intentional: state checking happens in the harness/tick handler, not here.
//!
//! When a partition is in `Waiting` state:
//! - The schedule table still returns that partition's ID on its slot
//! - The harness triggers PendSV, which switches context to the partition
//! - The partition immediately re-enters the kernel (yields) because it's
//!   blocked on an IPC primitive
//! - This "busy yield" continues until the blocking condition clears or
//!   the slot expires
//!
//! This design keeps the scheduler simple and stateless. The harness and
//! IPC primitives handle the complexity of blocked partitions.

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

/// Outcome of a schedule tick.
///
/// `SystemWindow` is only returned when the `dynamic-mpu` feature is enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleEvent {
    /// Switch to the specified partition.
    PartitionSwitch(u8),
    /// System window for kernel bottom-half processing (dynamic-mpu only).
    #[cfg(feature = "dynamic-mpu")]
    SystemWindow,
    /// No event (tick within current slot).
    None,
}

/// Narrow interface for schedule table access without const bounds.
pub trait ScheduleTableOps {
    /// Returns a slice of schedule entries.
    fn entries(&self) -> &[ScheduleEntry];
    /// Returns the number of entries in the schedule table.
    fn len(&self) -> usize;
    /// Returns `true` if the schedule table has no entries.
    fn is_empty(&self) -> bool;
    /// Returns the major frame duration in ticks.
    fn major_frame_ticks(&self) -> u32;
    /// Returns the current partition index, if any.
    fn current_partition(&self) -> Option<u8>;
}

/// Mutable interface for schedule table manipulation without const bounds.
pub trait ScheduleTableOpsMut: ScheduleTableOps {
    /// Add a schedule entry. Returns `Err` if table is full or duration is zero.
    fn add(&mut self, entry: ScheduleEntry) -> Result<(), ScheduleEntry>;
    /// Reset to the first slot. Call after adding all entries.
    fn start(&mut self);
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
    /// remaining ticks in the current slot.  Returns a [`ScheduleEvent`]
    /// describing the new slot, or [`ScheduleEvent::None`] if not started
    /// or empty.
    pub fn force_advance(&mut self) -> ScheduleEvent {
        if !self.started || self.entries.is_empty() {
            return ScheduleEvent::None;
        }
        let idx = self.step_to_next_slot();
        let entry = &self.entries[idx];
        #[cfg(feature = "dynamic-mpu")]
        if entry.is_system_window {
            return ScheduleEvent::SystemWindow;
        }
        ScheduleEvent::PartitionSwitch(entry.partition_index)
    }

    /// Advance by one tick. Returns a [`ScheduleEvent`] indicating
    /// whether a slot transition occurred.
    pub fn advance_tick(&mut self) -> ScheduleEvent {
        if !self.started || self.entries.is_empty() {
            return ScheduleEvent::None;
        }
        self.ticks_remaining = self.ticks_remaining.saturating_sub(1);
        if self.ticks_remaining == 0 {
            let idx = self.step_to_next_slot();
            let entry = &self.entries[idx];
            #[cfg(feature = "dynamic-mpu")]
            if entry.is_system_window {
                return ScheduleEvent::SystemWindow;
            }
            return ScheduleEvent::PartitionSwitch(entry.partition_index);
        }
        ScheduleEvent::None
    }
}

impl<const N: usize> ScheduleTableOps for ScheduleTable<N> {
    fn entries(&self) -> &[ScheduleEntry] {
        self.entries()
    }

    fn len(&self) -> usize {
        self.entries.len()
    }

    fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    fn major_frame_ticks(&self) -> u32 {
        self.major_frame_ticks
    }

    fn current_partition(&self) -> Option<u8> {
        self.current_partition()
    }
}

impl<const N: usize> ScheduleTableOpsMut for ScheduleTable<N> {
    fn add(&mut self, entry: ScheduleEntry) -> Result<(), ScheduleEntry> {
        self.add(entry)
    }

    fn start(&mut self) {
        self.start()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(slots: &[(u8, u32)]) -> ScheduleTable<4> {
        let mut t = ScheduleTable::new();
        for &(p, d) in slots {
            t.add(ScheduleEntry::new(p, d)).unwrap();
        }
        t.start();
        t
    }

    #[test]
    fn empty_table_returns_none() {
        let mut t = table(&[]);
        assert!(t.current_partition().is_none());
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
    }

    #[test]
    fn single_partition_wraps() {
        let mut t = table(&[(0, 3)]);
        assert_eq!(t.current_partition(), Some(0));
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
    }

    #[test]
    fn two_partitions_alternate() {
        let mut t = table(&[(0, 2), (1, 3)]);
        assert_eq!(t.major_frame_ticks, 5);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
    }

    #[test]
    fn major_frame_wraparound_sequence() {
        let mut t = table(&[(0, 1), (1, 1), (2, 1)]);
        for &exp in &[1u8, 2, 0, 1, 2, 0] {
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(exp));
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
    fn advance_tick_before_start_is_noop() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        t.add(ScheduleEntry::new(0, 5)).unwrap();
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
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
    fn force_advance_mid_slot() {
        let mut t = table(&[(0, 5), (1, 3)]);
        // Consume 2 of 5 ticks in slot 0
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        // Force advance skips remaining 3 ticks
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(1));
        // ticks_remaining should be reset to slot 1's duration (3)
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0)); // wraps back
    }

    #[test]
    fn force_advance_at_slot_boundary() {
        let mut t = table(&[(0, 2), (1, 2)]);
        // Exhaust slot 0 via advance_tick (switches to slot 1)
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
        // Now at slot 1 with full ticks_remaining; force advance to slot 0
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(0));
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
    }

    #[test]
    fn force_advance_single_entry_wraps() {
        let mut t = table(&[(7, 4)]);
        // Single entry: force advance wraps to same slot
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(7));
        // ticks_remaining should be fully reloaded
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(7));
    }

    #[test]
    fn force_advance_unstarted_returns_none() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        t.add(ScheduleEntry::new(0, 5)).unwrap();
        // Not started
        assert_eq!(t.force_advance(), ScheduleEvent::None);
        // Empty table
        let mut t2: ScheduleTable<4> = ScheduleTable::new();
        t2.start();
        assert_eq!(t2.force_advance(), ScheduleEvent::None);
    }

    // -------------------------------------------------------------------------
    // State-agnostic scheduling tests
    //
    // These tests document that ScheduleTable returns partition IDs without
    // checking PartitionState. The schedule table has no concept of partition
    // state — it only knows partition indices and durations.
    //
    // State checking happens in the harness/tick handler, not here. When a
    // partition is Waiting, the harness still switches to it; the partition
    // immediately yields, creating a "busy yield" loop until unblocked or
    // the slot expires.
    // -------------------------------------------------------------------------

    /// Documents that ScheduleTable returns partition IDs unconditionally.
    /// The table has no knowledge of partition state (Ready/Running/Waiting).
    #[test]
    fn schedule_returns_id_regardless_of_external_state() {
        // Setup: 3 partitions, each gets 2 ticks
        let mut t = table(&[(0, 2), (1, 2), (2, 2)]);

        // Even if we imagine partition 1 is "Waiting", the schedule table
        // doesn't know or care — it returns partition 1 when its slot comes.
        assert_eq!(t.advance_tick(), ScheduleEvent::None); // tick 1 of P0
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1)); // switch to P1 regardless of state
        assert_eq!(t.current_partition(), Some(1));

        // Continue: P1's slot runs to completion even if "Waiting"
        assert_eq!(t.advance_tick(), ScheduleEvent::None); // tick 1 of P1
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(2)); // switch to P2

        // And P2, then wrap back to P0
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0));
    }

    /// Documents behavior when all partitions would be "Waiting" (hypothetically).
    /// The schedule table continues cycling through slots, returning each
    /// partition ID in turn. The harness handles the idle loop scenario.
    #[test]
    fn all_partitions_waiting_cycles_through_slots() {
        // Setup: 2 partitions with 1-tick slots (fast cycling)
        let mut t = table(&[(0, 1), (1, 1)]);

        // Imagine both partitions are "Waiting". The schedule table doesn't
        // know this — it just cycles: P0 -> P1 -> P0 -> P1 -> ...
        // In the real harness, each switch triggers PendSV, the partition
        // yields immediately (because it's blocked), and the cycle continues.
        for _ in 0..10 {
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1)); // P0 -> P1
            assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0)); // P1 -> P0
        }
    }

    /// Documents that force_advance (yield) also returns partition ID
    /// unconditionally, without state checks.
    #[test]
    fn force_advance_returns_id_regardless_of_state() {
        let mut t = table(&[(0, 5), (1, 5), (2, 5)]);

        // Yield from P0 -> P1 (even if P1 is "Waiting")
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(1));

        // Yield from P1 -> P2 (even if P2 is "Waiting")
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(2));

        // Yield from P2 -> P0 (wraps around)
        assert_eq!(t.force_advance(), ScheduleEvent::PartitionSwitch(0));
    }

    /// Documents slot advancement behavior: ticks_remaining decrements
    /// each tick, switch happens when exhausted, regardless of what the
    /// harness does with the returned partition ID.
    #[test]
    fn slot_advancement_is_deterministic() {
        let mut t = table(&[(0, 3), (1, 2)]);

        // P0: 3 ticks
        assert_eq!(t.advance_tick(), ScheduleEvent::None); // tick 1
        assert_eq!(t.advance_tick(), ScheduleEvent::None); // tick 2
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1)); // tick 3, switch

        // P1: 2 ticks
        assert_eq!(t.advance_tick(), ScheduleEvent::None); // tick 1
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(0)); // tick 2, switch

        // Back to P0, same pattern repeats forever
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::None);
        assert_eq!(t.advance_tick(), ScheduleEvent::PartitionSwitch(1));
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
