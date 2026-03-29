//! Health monitoring types for partition liveness and schedule integrity.

/// Overall health status of the system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    /// All partitions running within deadlines.
    Ok,
    /// One or more non-critical violations detected.
    Degraded,
    /// A critical violation requiring immediate action.
    Critical,
}

/// Action the kernel should take in response to a health violation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthAction {
    /// Log the violation but take no corrective action.
    Log,
    /// Restart the partition identified by its index.
    Restart(u8),
    /// Halt the system.
    Halt,
}

/// Types of health violations the monitor can detect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthViolation {
    /// A major frame exceeded its deadline.
    ScheduleOverrun,
    /// Partition at the given index has not been scheduled for too long.
    PartitionStall(u8),
    /// System tick drift exceeded the configured threshold.
    TickDrift,
    /// A partition missed its heartbeat deadline.
    HeartbeatTimeout,
}

/// Mutable state maintained by the health partition for schedule monitoring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealthState {
    /// Tick value at the start of the last observed major frame.
    last_major_frame_tick: u64,
    /// Major frame count when `last_major_frame_tick` was recorded.
    last_major_frame_count: u32,
}

impl HealthState {
    /// Create a new `HealthState` seeded with the current tick and frame count.
    pub fn new(current_tick: u64, current_frame_count: u32) -> Self {
        Self {
            last_major_frame_tick: current_tick,
            last_major_frame_count: current_frame_count,
        }
    }

    /// Returns the tick value at the start of the last observed major frame.
    pub fn last_major_frame_tick(&self) -> u64 {
        self.last_major_frame_tick
    }

    /// Returns the major frame count when the tick baseline was recorded.
    pub fn last_major_frame_count(&self) -> u32 {
        self.last_major_frame_count
    }

    /// Check whether the current major frame has exceeded its deadline.
    ///
    /// If the major frame count has advanced since the last check, the tick
    /// baseline is reset to `current_tick` (the frame completed in time).
    /// Otherwise, elapsed ticks since the frame started are compared against
    /// `deadline_ticks`.
    ///
    /// Returns `(HealthStatus::Ok, None)` when within the deadline, or
    /// `(HealthStatus::Critical, Some(HealthViolation::ScheduleOverrun))`
    /// when the deadline is exceeded.
    // TODO: baseline uses observation time (`current_tick`) rather than actual
    // frame start time, which introduces sampling drift equal to monitor jitter.
    // Consider accepting a `frame_start_tick` parameter for precise detection.
    pub fn check_schedule_health(
        &mut self,
        current_tick: u64,
        current_frame_count: u32,
        deadline_ticks: u32,
    ) -> (HealthStatus, Option<HealthViolation>) {
        // Compute elapsed BEFORE resetting baseline so that overruns during
        // frame transitions are not masked.
        let elapsed = current_tick.saturating_sub(self.last_major_frame_tick);

        // A new major frame started — reset the baseline.
        if current_frame_count != self.last_major_frame_count {
            self.last_major_frame_tick = current_tick;
            self.last_major_frame_count = current_frame_count;
        }

        if elapsed > deadline_ticks as u64 {
            (
                HealthStatus::Critical,
                Some(HealthViolation::ScheduleOverrun),
            )
        } else {
            (HealthStatus::Ok, None)
        }
    }
}

/// Configuration for the system health monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemHealthConfig {
    /// Maximum ticks allowed for one major frame before an overrun is raised.
    pub major_frame_deadline_ticks: u32,
    /// Number of major frames a partition may go unscheduled before a stall.
    pub partition_liveness_frames: u16,
    /// Maximum acceptable tick drift in parts-per-million.
    pub tick_drift_ppm: u16,
    /// Action to take when a violation is detected.
    pub on_violation: HealthAction,
}

impl Default for SystemHealthConfig {
    fn default() -> Self {
        Self {
            major_frame_deadline_ticks: 1000,
            partition_liveness_frames: 3,
            tick_drift_ppm: 100,
            on_violation: HealthAction::Log,
        }
    }
}

impl SystemHealthConfig {
    /// Validate configuration values. Returns `Err` with a description on failure.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.major_frame_deadline_ticks == 0 {
            return Err("major_frame_deadline_ticks must be non-zero");
        }
        if self.partition_liveness_frames == 0 {
            return Err("partition_liveness_frames must be non-zero");
        }
        if self.tick_drift_ppm == 0 {
            return Err("tick_drift_ppm must be non-zero");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- HealthStatus construction ---

    #[test]
    fn health_status_variants() {
        let ok = HealthStatus::Ok;
        let degraded = HealthStatus::Degraded;
        let critical = HealthStatus::Critical;
        assert_eq!(ok, HealthStatus::Ok);
        assert_eq!(degraded, HealthStatus::Degraded);
        assert_eq!(critical, HealthStatus::Critical);
        assert_ne!(ok, degraded);
        assert_ne!(degraded, critical);
    }

    #[test]
    fn health_status_copy_and_debug() {
        let s = HealthStatus::Degraded;
        let s2 = s;
        assert_eq!(s, s2);
        assert_eq!(format!("{:?}", HealthStatus::Ok), "Ok");
    }

    // --- HealthAction construction ---

    #[test]
    fn health_action_variants() {
        assert_eq!(HealthAction::Log, HealthAction::Log);
        assert_eq!(HealthAction::Halt, HealthAction::Halt);
        assert_ne!(HealthAction::Halt, HealthAction::Log);
        let a = HealthAction::Restart(3);
        assert_eq!(a, HealthAction::Restart(3));
        assert_ne!(a, HealthAction::Restart(0));
        assert_eq!(format!("{:?}", HealthAction::Restart(1)), "Restart(1)");
    }

    // --- HealthViolation construction ---

    #[test]
    fn violation_variants() {
        assert_eq!(
            HealthViolation::ScheduleOverrun,
            HealthViolation::ScheduleOverrun
        );
        let v = HealthViolation::PartitionStall(2);
        assert_eq!(v, HealthViolation::PartitionStall(2));
        assert_ne!(v, HealthViolation::PartitionStall(5));
        assert_eq!(HealthViolation::TickDrift, HealthViolation::TickDrift);
        assert_ne!(
            HealthViolation::HeartbeatTimeout,
            HealthViolation::TickDrift
        );
        assert_eq!(
            format!("{:?}", HealthViolation::PartitionStall(7)),
            "PartitionStall(7)"
        );
    }

    // --- SystemHealthConfig default ---

    #[test]
    fn config_default_values() {
        let cfg = SystemHealthConfig::default();
        assert_eq!(cfg.major_frame_deadline_ticks, 1000);
        assert_eq!(cfg.partition_liveness_frames, 3);
        assert_eq!(cfg.tick_drift_ppm, 100);
        assert_eq!(cfg.on_violation, HealthAction::Log);
    }

    #[test]
    fn config_default_validates() {
        assert!(SystemHealthConfig::default().validate().is_ok());
    }

    // --- SystemHealthConfig validation ---

    #[test]
    fn config_zero_deadline_rejected() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(
            cfg.validate(),
            Err("major_frame_deadline_ticks must be non-zero")
        );
    }

    #[test]
    fn config_zero_liveness_rejected() {
        let cfg = SystemHealthConfig {
            partition_liveness_frames: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(
            cfg.validate(),
            Err("partition_liveness_frames must be non-zero")
        );
    }

    #[test]
    fn config_zero_drift_rejected() {
        let cfg = SystemHealthConfig {
            tick_drift_ppm: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(cfg.validate(), Err("tick_drift_ppm must be non-zero"));
    }

    #[test]
    fn config_custom_valid() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 500,
            partition_liveness_frames: 10,
            tick_drift_ppm: 50,
            on_violation: HealthAction::Halt,
        };
        assert!(cfg.validate().is_ok());
        assert_eq!(cfg.on_violation, HealthAction::Halt);
    }

    #[test]
    fn config_copy_debug_restart() {
        let cfg = SystemHealthConfig::default();
        let cfg2 = cfg;
        assert_eq!(cfg, cfg2);
        let s = format!("{:?}", cfg);
        assert!(s.contains("SystemHealthConfig"));

        let cfg3 = SystemHealthConfig {
            major_frame_deadline_ticks: 200,
            partition_liveness_frames: 1,
            tick_drift_ppm: 500,
            on_violation: HealthAction::Restart(2),
        };
        assert!(cfg3.validate().is_ok());
        assert_eq!(cfg3.on_violation, HealthAction::Restart(2));
    }

    #[test]
    fn config_all_zero_reports_deadline_first() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 0,
            partition_liveness_frames: 0,
            tick_drift_ppm: 0,
            on_violation: HealthAction::Log,
        };
        assert_eq!(
            cfg.validate(),
            Err("major_frame_deadline_ticks must be non-zero")
        );
    }

    #[test]
    fn config_boundary_values() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: u32::MAX,
            partition_liveness_frames: u16::MAX,
            tick_drift_ppm: u16::MAX,
            on_violation: HealthAction::Log,
        };
        assert!(cfg.validate().is_ok());
    }

    // --- HealthState construction ---

    #[test]
    fn health_state_new() {
        let state = HealthState::new(100, 5);
        assert_eq!(state.last_major_frame_tick(), 100);
        assert_eq!(state.last_major_frame_count(), 5);
    }

    // --- check_schedule_health: normal operation ---

    #[test]
    fn schedule_health_ok_within_deadline() {
        let mut state = HealthState::new(1000, 3);
        // 500 ticks elapsed, deadline is 1000 — well within bounds.
        let (status, violation) = state.check_schedule_health(1500, 3, 1000);
        assert_eq!(status, HealthStatus::Ok);
        assert_eq!(violation, None);
    }

    #[test]
    fn schedule_health_ok_no_elapsed() {
        let mut state = HealthState::new(500, 1);
        let (status, violation) = state.check_schedule_health(500, 1, 1000);
        assert_eq!(status, HealthStatus::Ok);
        assert_eq!(violation, None);
    }

    // --- check_schedule_health: exact deadline boundary ---

    #[test]
    fn schedule_health_ok_at_exact_deadline() {
        let mut state = HealthState::new(0, 0);
        // Exactly at deadline — not exceeded, should be Ok.
        let (status, violation) = state.check_schedule_health(1000, 0, 1000);
        assert_eq!(status, HealthStatus::Ok);
        assert_eq!(violation, None);
    }

    // --- check_schedule_health: overrun detection ---

    #[test]
    fn schedule_health_critical_on_overrun() {
        let mut state = HealthState::new(0, 0);
        // 1001 ticks elapsed, deadline is 1000 — overrun by 1.
        let (status, violation) = state.check_schedule_health(1001, 0, 1000);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
    }

    #[test]
    fn schedule_health_critical_large_overrun() {
        let mut state = HealthState::new(100, 2);
        let (status, violation) = state.check_schedule_health(5100, 2, 500);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
    }

    // --- check_schedule_health: frame advancement resets baseline ---

    #[test]
    fn schedule_health_resets_on_new_frame() {
        let mut state = HealthState::new(0, 0);
        // Frame advanced within deadline — elapsed (800) <= 1000, baseline resets.
        let (status, violation) = state.check_schedule_health(800, 1, 1000);
        assert_eq!(status, HealthStatus::Ok);
        assert_eq!(violation, None);
        assert_eq!(state.last_major_frame_tick(), 800);
        assert_eq!(state.last_major_frame_count(), 1);
    }

    #[test]
    fn schedule_health_detects_overrun_on_frame_transition() {
        let mut state = HealthState::new(0, 0);
        // Frame advanced but previous frame overran: elapsed (5000) > 1000.
        let (status, violation) = state.check_schedule_health(5000, 1, 1000);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
        // Baseline still resets for the new frame.
        assert_eq!(state.last_major_frame_tick(), 5000);
        assert_eq!(state.last_major_frame_count(), 1);
    }

    #[test]
    fn schedule_health_overrun_after_frame_reset() {
        let mut state = HealthState::new(0, 0);
        // Frame advances at tick 400 (within 500-tick deadline).
        let (status, _) = state.check_schedule_health(400, 1, 500);
        assert_eq!(status, HealthStatus::Ok);
        // Same frame, 501 ticks later — overrun.
        let (status, violation) = state.check_schedule_health(901, 1, 500);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
    }

    #[test]
    fn schedule_health_multiple_frame_advances() {
        let mut state = HealthState::new(0, 0);
        // Simulate several frames completing on time.
        for frame in 1..=5 {
            let tick = frame as u64 * 800;
            let (status, violation) = state.check_schedule_health(tick, frame, 1000);
            assert_eq!(status, HealthStatus::Ok);
            assert_eq!(violation, None);
            assert_eq!(state.last_major_frame_count(), frame);
        }
    }

    #[test]
    fn schedule_health_skipped_frames_reset() {
        let mut state = HealthState::new(0, 0);
        // Frame count jumps from 0 to 5 (frames were completed while we
        // weren't checking). Elapsed since last baseline is checked first,
        // so this correctly detects the overrun even though frames advanced.
        let (status, violation) = state.check_schedule_health(10000, 5, 1000);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
        // Baseline still resets for the new frame.
        assert_eq!(state.last_major_frame_count(), 5);
        assert_eq!(state.last_major_frame_tick(), 10000);
    }

    #[test]
    fn schedule_health_deadline_one_tick() {
        let mut state = HealthState::new(0, 0);
        // Deadline of 1 tick — exactly 1 elapsed is ok.
        let (status, _) = state.check_schedule_health(1, 0, 1);
        assert_eq!(status, HealthStatus::Ok);
        // 2 elapsed is overrun.
        let (status, violation) = state.check_schedule_health(2, 0, 1);
        assert_eq!(status, HealthStatus::Critical);
        assert_eq!(violation, Some(HealthViolation::ScheduleOverrun));
    }
}
